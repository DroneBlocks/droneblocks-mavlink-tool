#!/usr/bin/env python3
"""
Set a chosen list of params on the connected FC to EXACTLY match a baseline dump
(value + type read straight from the dump file). Used to bring drone #2 to indoor
parity with the golden drone #1 — WITHOUT touching per-board calibration, RC cal,
or known per-unit quirks.

Usage:
    python reconcile_to_baseline.py drone1_baseline.params
"""
import sys, time, glob, struct
from pymavlink import mavutil

# The config we WANT to match the golden indoor drone (everything that affects
# flow / EKF / battery / throttle feel). Deliberately excludes: RC*, CAL_*,
# SENS_BOARD_*_OFF, CA_ROTOR* (cosmetic scale), MAV_1_CONFIG (#1 quirk),
# COM_FLTMODE* (tx-specific), MC_ACRO_* (acro-only).
PARITY = [
    "SYS_HAS_GPS", "GPS_1_CONFIG",          # GPS misconfig -> off (no GPS hardware)
    "SENS_FLOW_MAXHGT", "SENS_FLOW_MINHGT", # flow valid-height gating
    "EKF2_MIN_RNG", "EKF2_RNG_A_HMAX", "EKF2_RNG_NOISE",  # range-finder EKF
    "BAT1_V_EMPTY", "BAT1_CAPACITY",        # battery
    "MPC_THR_CURVE", "MPC_THR_MIN",         # throttle mapping (flight feel)
]
INT_TYPES = {1,2,3,4,5,6,7,8}  # MAV_PARAM_TYPE ints; 9/10 = REAL32/64

def load_baseline(path):
    out = {}
    for line in open(path):
        if line.startswith('#'): continue
        p = line.rstrip('\n').split('\t')
        if len(p) >= 5:
            out[p[2]] = (p[3], int(p[4]))   # name -> (value_str, type)
    return out

def i2f(i): return struct.unpack('<f', struct.pack('<i', int(i)))[0]
def f2i(f): return struct.unpack('<i', struct.pack('<f', f))[0]

def main():
    if len(sys.argv) < 2: sys.exit("usage: reconcile_to_baseline.py <baseline.params>")
    base = load_baseline(sys.argv[1])
    targets = {}
    for name in PARITY:
        if name not in base:
            print(f"  WARN {name} not in baseline; skipping"); continue
        vstr, typ = base[name]
        is_int = typ in INT_TYPES
        targets[name] = (int(float(vstr)) if is_int else float(vstr), typ, is_int)

    port = sorted(glob.glob('/dev/cu.usbmodem*'))[0]
    m = mavutil.mavlink_connection(port, baud=115200)
    for _ in range(4):
        hb = m.wait_heartbeat(timeout=6)
        if hb and m.target_system: break
    print(f"# {port} sys {m.target_system} comp {m.target_component}")

    for name,(val,typ,is_int) in targets.items():
        field = i2f(val) if is_int else float(val)
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), field, typ)
        time.sleep(0.1)

    found = {}; deadline = time.time()+12; last=0
    while time.time()<deadline and len(found)<len(targets):
        if time.time()-last>2:
            for n in targets:
                if n not in found:
                    m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
            last=time.time()
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is None: continue
        pid = msg.param_id
        if isinstance(pid, bytes): pid = pid.decode(errors='ignore')
        pid = pid.rstrip('\x00')
        if pid in targets:
            _,_,is_int = targets[pid]
            found[pid] = f2i(msg.param_value) if is_int else msg.param_value

    print("--- readback (target = golden drone #1) ---")
    ok = True
    for name,(val,typ,is_int) in targets.items():
        got = found.get(name)
        good = got is not None and (got==val if is_int else abs(got-val)<1e-4)
        ok = ok and good
        print(f"{name:18s} want {str(val):>8}  got {got}  [{'OK' if good else 'MISMATCH'}]")
    print("ALL OK — indoor parity applied" if ok else "SOME MISMATCH — re-check")

if __name__ == "__main__":
    main()
