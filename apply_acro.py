#!/usr/bin/env python3
"""Apply DEXI-3 ACRO max-rate params over USB — additive, touches nothing else.

The DEXI-3 indoor-flow flight kit ships with snappy ACRO rates so flips/rolls
feel responsive (PX4's 300/300/100 deg/s defaults are sluggish). This is an
ADDITIVE overlay: it writes ONLY MC_ACRO_{R,P,Y}_MAX, force-saves, and verifies.
It does not touch comms / EKF / airframe params — run it after provisioning
(provision_dexi3_flow.py) on the same bench connection.

Mirrors the px4-web-configurator "DEXI-3 ACRO rates" additive profile.

Run:  ./venv/bin/python apply_acro.py            # auto-detect /dev/cu.usbmodem*
      ./venv/bin/python apply_acro.py --verify-only
"""
import argparse, sys, time
from pymavlink import mavutil
from provision_dexi3_flow import connect, set_param, read_param, nsh, matches

# All floats (REAL32) — deg/s max rate in ACRO mode.
ACRO = [
    ("MC_ACRO_R_MAX", 720.0, "float"),  # roll
    ("MC_ACRO_P_MAX", 720.0, "float"),  # pitch
    ("MC_ACRO_Y_MAX", 400.0, "float"),  # yaw
]


def main():
    ap = argparse.ArgumentParser(description="Apply DEXI-3 ACRO max rates over USB (additive).")
    ap.add_argument("--device", help="serial device (default: auto-detect /dev/cu.usbmodem*)")
    ap.add_argument("--verify-only", action="store_true", help="read+check only, write nothing")
    args = ap.parse_args()

    m, dev = connect(args.device)
    if not m:
        sys.exit("✗ no MAVLink FC found over USB — power-cycle and retry")
    print(f"connected on {dev}")

    if not args.verify_only:
        print(f"applying {len(ACRO)} ACRO params (additive — nothing else touched):")
        for name, val, kind in ACRO:
            print(f"  set {name:<16} = {val}  [{kind}]")
            set_param(m, name, val, kind)
            time.sleep(0.15)
        print("force-saving to flash (PREFLIGHT_STORAGE param1=1)…")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1); nsh(m, "param save", 2.0)

    print("\nverifying (authoritative nsh param show):")
    ok = True
    for name, val, kind in ACRO:
        stored = read_param(m, name)
        good = stored is not None and matches(stored, val, kind)
        ok = ok and good
        print(f"  {name:<16} = {str(stored):<10}  (want {val})  {'OK' if good else 'FAIL'}")

    print("\n█ ACRO PASS — rates stored █" if ok else "\n█ ACRO FAIL — see above █")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
