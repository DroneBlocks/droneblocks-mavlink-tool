#!/usr/bin/env python3
"""Provision a DEXI-3 (H743-AIO) flight controller for INDOOR OPTICAL FLOW, over USB.

Does, deterministically and repeatably, exactly what the px4-web-configurator's
"DEXI Indoor (Flow Only)" provisioning does — board comms wiring + the indoor-flow
EKF profile — and prints every step so you can see what happened.

Why a script instead of the web app:
  * MAVLink-FIRST connect — sends a GCS heartbeat the instant it opens the port,
    so PX4's USB console comes up in MAVLink mode (not nsh). This is the "connected
    but no data / SYS 0" gremlin in the web app.
  * Correct INT encoding — int params are BIT-CAST into the float field (PX4 1.17
    reads the wire bytes per type). Sending the numeric value (the old web-app bug)
    stored e.g. SER_TEL1_BAUD=500000 as 1223959552 and silently killed optical flow.
  * Authoritative verification — reads every value back via the nsh `param show`
    console (the raw stored int), not the ambiguous MAVLink wire float.

Param set source of truth:
  lib/boards.ts  (droneblocks-h743-aio hardwareDefaults)
  lib/profiles.ts (indoor-flow profile)  — keep in sync if those change.

Usage:
  ./venv/bin/python provision_dexi3_flow.py            # provision the connected FC
  ./venv/bin/python provision_dexi3_flow.py --watch    # mass-update: provision each FC as you plug it in
  ./venv/bin/python provision_dexi3_flow.py --verify-only   # just check, don't write
  ./venv/bin/python provision_dexi3_flow.py --no-reboot     # skip final reboot+persistence check
  ./venv/bin/python provision_dexi3_flow.py --device /dev/cu.usbmodem01
"""
import argparse, glob, re, struct, sys, time
import serial
from pymavlink import mavutil

# ── DEXI-3 (H743-AIO) airframe ──────────────────────────────────────────────
AIRFRAME_ID = 4701                          # DroneBlocks H743-AIO / UP-T201

# ── The full provisioning set (name, value, kind) ───────────────────────────
PARAMS = [
    # Comms / board wiring  (lib/boards.ts droneblocks-h743-aio hardwareDefaults)
    ("MAV_0_CONFIG",   101,    "int"),      # mavlink-router on TELEM1
    ("MAV_2_CONFIG",   103,    "int"),      # mavlink instance on TELEM3 (UP-T201 line)
    ("SER_TEL1_BAUD",  500000, "int"),      # mavlink-router baud
    ("SER_TEL2_BAUD",  921600, "int"),      # uXRCE-DDS baud
    ("SER_TEL3_BAUD",  115200, "int"),      # UP-T201 optical flow baud
    ("UXRCE_DDS_CFG",  102,    "int"),      # uXRCE-DDS on TELEM2
    ("RC_CRSF_PRT_CFG",0,      "int"),      # CRSF auto-detected on RC pad — keep standalone driver off
    # Navigation: indoor optical flow  (lib/profiles.ts indoor-flow)
    ("EKF2_EV_CTRL",   0,      "int"),      # external vision off
    ("EKF2_OF_CTRL",   1,      "int"),      # optical flow on
    ("EKF2_HGT_REF",   2,      "int"),      # range = height reference
    ("EKF2_RNG_CTRL",  2,      "int"),      # range always fused
    ("EKF2_BARO_CTRL", 1,      "int"),      # baro smooths vertical (no bob)
    ("EKF2_GPS_CTRL",  0,      "int"),      # GPS off
    ("EKF2_MAG_TYPE",  5,      "int"),      # gyro yaw (mag off — glitches indoors)
    ("COM_ARM_WO_GPS", 1,      "int"),      # allow arming without GPS
    # Flight limits
    ("MPC_XY_VEL_MAX", 4.0,    "float"),    # indoor speed cap (m/s)
    # ── Flight tune (validated on two DEXI-3s, 2026-06-17) ──────────────────
    # Replaces the 4701 airframe's inherited QAV250 defaults (0.076 / airmode 1),
    # which flew with arm-jitter + a wallow. Set explicitly here (not relied on
    # from the airframe) so it lands even on boards still running old firmware,
    # and survives a "reset to defaults". NOTE: effective gain = K * P — the K
    # multipliers are part of the tune, do not drop them.
    ("SENS_BOARD_ROT",  1,     "int"),      # FC mounted Yaw 45 in the DEXI-3 frame
    ("MC_AIRMODE",      0,     "int"),      # OFF — fixes motor jitter on arm
    ("MC_ROLLRATE_K",   0.7,   "float"),    # master rate gain (effective P = K*P = 0.105)
    ("MC_PITCHRATE_K",  0.7,   "float"),
    ("MC_YAWRATE_K",    1.0,   "float"),
    ("MC_ROLLRATE_P",   0.15,  "float"),
    ("MC_PITCHRATE_P",  0.15,  "float"),
    ("MC_ROLLRATE_I",   0.2,   "float"),
    ("MC_PITCHRATE_I",  0.2,   "float"),
    ("MC_ROLLRATE_D",   0.003, "float"),
    ("MC_PITCHRATE_D",  0.003, "float"),
    ("MC_ROLLRATE_MAX", 220.0, "float"),
    ("MC_PITCHRATE_MAX",220.0, "float"),
    ("MC_YAWRATE_P",    0.2,   "float"),
    ("MC_YAWRATE_I",    0.1,   "float"),
    ("MC_YAWRATE_MAX",  200.0, "float"),
    ("MC_ROLL_P",       6.5,   "float"),
    ("MC_PITCH_P",      6.5,   "float"),
    ("MC_YAW_P",        2.8,   "float"),
    ("MPC_MAN_TILT_MAX",60.0,  "float"),    # beginner tilt limit
]

INT32  = mavutil.mavlink.MAV_PARAM_TYPE_INT32
REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
DEV_SHELL = 10
SC_FLAGS = (mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
            mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
            mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI)

def _bitcast_int_to_float(i):   # the bytes PX4 expects in param_value for an INT param
    return struct.unpack("<f", struct.pack("<i", int(i)))[0]

# ── Connection (MAVLink-first, beats PX4 USB nsh autostart) ──────────────────
def _connect_mavlink(device, timeout):
    end = time.time() + timeout
    while time.time() < end:
        for d in ([device] if device else glob.glob("/dev/cu.usbmodem*")):
            if not d:
                continue
            try:
                c = mavutil.mavlink_connection(d, baud=57600, source_system=255, source_component=190)
                c.mav.heartbeat_send(6, 8, 0, 0, 0)          # GCS heartbeat FIRST — wins MAVLink mode
                t0 = time.time()
                while time.time() - t0 < 8:
                    c.mav.heartbeat_send(6, 8, 0, 0, 0)
                    hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=0.4)
                    if hb and (hb.autopilot == 12 or hb.get_srcComponent() == 1):
                        c.target_system, c.target_component = hb.get_srcSystem(), 1
                        return c, d
            except Exception:
                pass
        time.sleep(1)
    return None, None

def _raw_reboot(device):
    """FC USB stuck in nsh mode? Reboot it over the raw text console so the next
    open can win MAVLink mode on the fresh boot."""
    for d in ([device] if device else glob.glob("/dev/cu.usbmodem*")):
        if not d:
            continue
        try:
            s = serial.Serial(d, 115200, timeout=1)
            s.write(b"\nreboot\n"); time.sleep(0.5); s.close()
            return True
        except Exception:
            pass
    return False

def connect(device=None, timeout=70):
    # Patient MAVLink-first connect: a GCS heartbeat goes out the instant we open
    # the port, which wins MAVLink mode on a fresh boot. We deliberately do NOT
    # auto-reboot a non-responding board — on this flaky board that just wedges a
    # perfectly good app. If this returns None, power-cycle the board and re-run.
    return _connect_mavlink(device, timeout=timeout)

# ── nsh console over MAVLink SERIAL_CONTROL ─────────────────────────────────
def nsh(m, cmd, settle=1.6):
    # Tolerant of the USB port dropping mid-command (e.g. a `reboot` tears the
    # CDC down while we're still draining) — just return what we got.
    try:
        m.mav.heartbeat_send(6, 8, 0, 0, 0)
        b = (cmd + "\n").encode()
        while b:
            chunk, b = b[:70], b[70:]
            m.mav.serial_control_send(DEV_SHELL, SC_FLAGS, 0, 0, len(chunk), list(chunk) + [0] * (70 - len(chunk)))
        out, end = b"", time.time() + settle
        while time.time() < end:
            m.mav.serial_control_send(DEV_SHELL, SC_FLAGS, 0, 0, 0, [0] * 70)
            msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
            if msg and msg.count > 0:
                out += bytes(msg.data[:msg.count]); end = time.time() + 0.4
        return out.decode(errors="replace")
    except Exception:
        return ""

def read_param(m, name):
    """Authoritative stored value via nsh `param show` (raw int/float)."""
    mt = re.search(rf"{re.escape(name)}\s*\[[^\]]*\]\s*:\s*(-?[\d.]+)", nsh(m, f"param show {name}"))
    if not mt:
        return None
    s = mt.group(1)
    return float(s) if "." in s else int(s)

def set_param(m, name, value, kind):
    if kind == "int":
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             _bitcast_int_to_float(value), INT32)      # BIT-CAST (the fix)
    else:
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             float(value), REAL32)
    time.sleep(0.15)

def matches(stored, value, kind):
    if stored is None:
        return False
    return stored == value if kind == "int" else abs(float(stored) - float(value)) < 1e-4

def flow_check(m, secs=10):
    for ts, tc in ((1, 1), (0, 0)):
        m.mav.command_long_send(ts, tc, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 100000, 0, 0, 0, 0, 0)
    cnt = q = dist = None; cnt = 0; last = 0; end = time.time() + secs
    while time.time() < end:
        if time.time() - last >= 1: m.mav.heartbeat_send(6, 8, 0, 0, 0); last = time.time()
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg and msg.get_type() == "OPTICAL_FLOW_RAD":
            cnt += 1; q = msg.quality; dist = msg.distance
    return cnt, q, dist

# ── Provision one FC ────────────────────────────────────────────────────────
def provision(device=None, do_reboot=True, verify_only=False):
    print("─" * 64)
    print("connecting (MAVLink-first)…")
    m, d = connect(device)
    if not m:
        print("✗ no flight controller found on USB"); return False
    fw = nsh(m, "ver all")
    hw = re.search(r"HW arch:\s*(\S+)", fw); ver = re.search(r"PX4 version:\s*(\S+)", fw)
    print(f"✓ connected on {d} — {hw.group(1) if hw else '?'}  PX4 {ver.group(1) if ver else '?'}  (sys {m.target_system})")

    if verify_only:
        print("\nVERIFY-ONLY — current stored values:")
        return _verify(m)

    # Airframe: ensure SYS_AUTOSTART = 4701 (reboot only if it needs changing)
    cur = read_param(m, "SYS_AUTOSTART")
    if cur != AIRFRAME_ID:
        print(f"\nairframe SYS_AUTOSTART {cur} → {AIRFRAME_ID} (DEXI-3); saving + rebooting to apply…")
        set_param(m, "SYS_AUTOSTART", AIRFRAME_ID, "int"); time.sleep(0.5)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1); nsh(m, "reboot", 0.5); time.sleep(7)
        m, d = connect(device)
        if not m: print("✗ FC did not return after airframe reboot"); return False
        print(f"✓ reconnected — SYS_AUTOSTART = {read_param(m,'SYS_AUTOSTART')}")
    else:
        print(f"\nairframe already DEXI-3 (SYS_AUTOSTART = {cur})")

    # Apply the full set
    print(f"\napplying {len(PARAMS)} params (int=bit-cast, float=direct):")
    for name, val, kind in PARAMS:
        set_param(m, name, val, kind)
        print(f"  set {name:16} = {val}  [{kind}]")
    print("force-saving to flash (PREFLIGHT_STORAGE param1=1)…")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1); nsh(m, "param save", 2.0)

    if do_reboot:
        print("rebooting to confirm persistence…")
        nsh(m, "reboot", 0.5); time.sleep(7)
        m, d = connect(device)
        if not m: print("✗ FC did not return after reboot (params were saved)"); return False
        print(f"✓ reconnected on {d}")

    ok = _verify(m)

    cnt, q, dist = flow_check(m)
    if cnt:
        print(f"\nOPTICAL FLOW: streaming ~{cnt/10:.1f} Hz, quality {q}/255, dist {dist:.2f} m  ✓")
    else:
        print("\nOPTICAL FLOW: no OPTICAL_FLOW_RAD seen — sensor not attached/powered, or check wiring "
              "(this does NOT fail provisioning; params are what matter)")

    print("\n" + ("█ PROVISION PASS — all params stored correctly █" if ok
                  else "█ PROVISION FAIL — see MISMATCH above █"))
    return ok

def _verify(m):
    print("\nverifying (authoritative nsh param show):")
    allok = True
    for name, val, kind in PARAMS:
        stored = read_param(m, name); ok = matches(stored, val, kind); allok = allok and ok
        print(f"  {name:16} = {str(stored):<11} (want {val})  {'OK' if ok else 'MISMATCH ✗'}")
    return allok

# ── Mass-update watch loop ──────────────────────────────────────────────────
def watch():
    print("WATCH MODE — plug in a DEXI-3 FC to provision it; unplug when done; Ctrl-C to stop.\n")
    while True:
        while not glob.glob("/dev/cu.usbmodem*"):
            time.sleep(1)
        provision()
        print("\n…unplug this FC, then plug in the next one.")
        while glob.glob("/dev/cu.usbmodem*"):
            time.sleep(1)
        print("(FC removed)\n")

def main():
    ap = argparse.ArgumentParser(description="Provision DEXI-3 for indoor optical flow over USB.")
    ap.add_argument("--device", help="serial device (default: auto-detect /dev/cu.usbmodem*)")
    ap.add_argument("--watch", action="store_true", help="mass-update loop")
    ap.add_argument("--no-reboot", action="store_true", help="skip final reboot+persistence check")
    ap.add_argument("--verify-only", action="store_true", help="read+check only, write nothing")
    a = ap.parse_args()
    if a.watch:
        watch()
    else:
        ok = provision(a.device, do_reboot=not a.no_reboot, verify_only=a.verify_only)
        sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
