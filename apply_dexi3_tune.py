#!/usr/bin/env python3
"""
Apply the validated DEXI-3 flight tune over USB.

Captured 2026-06-17 from the first DEXI-3 (H743-AIO, board 1240) after Dennis
flew it "very, very well." The tune is essentially PX4 stock multicopter
firmware defaults + airmode OFF. It deliberately REPLACES the 4701 airframe's
inherited QAV250 gains (0.076/airmode 1), which flew worse (arm jitter from
airmode 2/1; a lower-gain "calmer" variant flight-tested to a slow wallow and
was rejected).

Usage:
    source venv/bin/activate
    python apply_dexi3_tune.py                # auto-detect USB port, apply + verify
    python apply_dexi3_tune.py --dry-run      # show what would change, write nothing

NOTE on portability to another drone:
  * The target drone should already be flashed with airframe SYS_AUTOSTART=4701
    (this only overlays the tune block; comms/EKF/battery come from the airframe).
  * SENS_BOARD_ROT is PER PHYSICAL MOUNT. This drone is Yaw 45 (=1). VERIFY how
    the FC is mounted in the target drone before trusting it — it is applied here
    but flagged loudly. A wrong board-rot cross-couples roll/pitch -> oscillation.
  * MPC_THR_HOVER is intentionally NOT set — it's mass/thrust specific; let each
    drone learn its own via the hover-thrust estimator.
  * Same-build DEXI-3 assumed. Different motors/props/weight -> this is a great
    starting point, but run PX4 autotune (MC_AT_EN=1) for a per-airframe tune.
"""
import sys, time, glob, struct, argparse
from pymavlink import mavutil

import dexi_params  # tune comes from dexi-fc-params (params/dexi-3.json)

# name -> (value, is_int)
# NOTE: PX4 effective rate gain = MC_xxxRATE_K * MC_xxxRATE_P. The K multipliers
# are part of drone #1's validated tune (K=0.7 roll/pitch = a 30% master-gain cut;
# default is 1.0) and MUST be set, or a target drone with a different K flies at the
# wrong effective gain. Drone #1 effective roll P = 0.7*0.15 = 0.105.
# The validated DEXI-3 tune. NOT defined here — source of truth is
# DroneBlocks/dexi-fc-params -> src/dexi-3.json, block "tune", vendored at
# params/dexi-3.json (refresh: ./fetch-params.sh). The web configurator and the
# .params downloads compose the same block, so all three stay identical.
#
# NOTE: effective rate gain = K * P (roll = 0.7 * 0.15 = 0.105). The K lines ARE
# part of the tune — a set that drops them transfers the wrong effective gain.
#
# name -> (value, is_int)
TUNE = {n: (v, k == "int") for n, v, k in dexi_params.load_blocks(["tune"])}

def i2f(i):  # int bit-pattern -> float field
    return struct.unpack('<f', struct.pack('<i', int(i)))[0]
def f2i(f):  # float field -> int bit-pattern
    return struct.unpack('<i', struct.pack('<f', f))[0]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    port = args.port or (sorted(glob.glob('/dev/cu.usbmodem*')) or [None])[0]
    if not port:
        sys.exit("No USB serial port found — is the DEXI-3 plugged in?")
    print(f"# port {port}")

    m = mavutil.mavlink_connection(port, baud=115200)
    if not m.wait_heartbeat(timeout=15):
        sys.exit("No MAVLink heartbeat — board not streaming on this port.")
    print(f"# heartbeat sys {m.target_system} comp {m.target_component}")

    if args.dry_run:
        print("# DRY RUN — nothing written")
        for n,(v,_) in TUNE.items(): print(f"  would set {n:18s} = {v}")
        return

    for n,(v,is_int) in TUNE.items():
        if is_int:
            m.mav.param_set_send(m.target_system, m.target_component, n.encode(),
                                 i2f(v), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        else:
            m.mav.param_set_send(m.target_system, m.target_component, n.encode(),
                                 float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.1)

    # verify
    found = {}
    deadline = time.time() + 15
    last = 0
    while time.time() < deadline and len(found) < len(TUNE):
        if time.time() - last > 2:
            for n in TUNE:
                if n not in found:
                    m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
            last = time.time()
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is None: continue
        pid = msg.param_id
        if isinstance(pid, bytes): pid = pid.decode(errors='ignore')
        pid = pid.rstrip('\x00')
        if pid in TUNE:
            v, is_int = TUNE[pid]
            found[pid] = f2i(msg.param_value) if is_int else msg.param_value

    print("--- readback ---")
    ok = True
    for n,(v,is_int) in TUNE.items():
        got = found.get(n)
        good = got is not None and (got == v if is_int else abs(got - v) < 1e-5)
        ok = ok and good
        print(f"{n:18s} want {v:<7} got {got}  [{'OK' if good else 'MISMATCH'}]")
    print("ALL OK — tune applied" if ok else "SOME MISMATCH — re-check")
    print("\n*** REMINDER: confirm SENS_BOARD_ROT matches THIS drone's physical FC mount. ***")

if __name__ == "__main__":
    main()
