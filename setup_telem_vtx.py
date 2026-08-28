#!/usr/bin/env python3
"""
Configure a MAVLink telemetry radio on the DEXI-3 (H743-AIO) VTX port.

Context: on the DEXI-3 AIO the VTX/DJI connector and the ROS2 "TEL2" JST are the
SAME UART — USART2 = PX4 TELEM2 (/dev/ttyS1). A provisioned DEXI-3 uses TELEM2 for
uXRCE-DDS (companion ROS2) at 921600. A FLIGHT-ONLY (no-compute) DEXI-3 has no
companion, so we free USART2 and put a plain MAVLink GCS instance on it for a
900 MHz SiK-style telem radio wired to the VTX port.

Leaves the indoor optical-flow config, the companion TEL1 line (MAV_0), and the
UP-T201 flow line (MAV_2 / TELEM3) untouched.

Reuses provision_dexi3_flow's tested MAVLink-first connect + INT32 bit-cast
param_set (avoids the float-as-int corruption bug) + force-save + reboot.

    cd ~/_dev/droneblocks-mavlink-tool
    ./venv/bin/python setup_telem_vtx.py            # apply + reboot + verify
    ./venv/bin/python setup_telem_vtx.py --verify-only
"""
import argparse
import time

from pymavlink import mavutil
import provision_dexi3_flow as p

# name, value, kind — all int
TELEM_PARAMS = [
    ("UXRCE_DDS_CFG", 0,     "int"),   # disable ROS2/DDS on TELEM2 -> frees USART2 (VTX port)
    ("MAV_1_CONFIG",  102,   "int"),   # MAVLink instance 1 on TELEM2 (the VTX/USART2 port)
    ("MAV_1_MODE",    0,     "int"),   # Normal — standard GCS telemetry stream (the "default" config)
    ("MAV_1_RATE",    0,     "int"),   # 0 = auto (~half link bandwidth)
    ("SER_TEL2_BAUD", 57600, "int"),   # SiK 900 MHz radio default serial baud — must match the radio
]


def force_save(m):
    print("force-saving to flash (PREFLIGHT_STORAGE param1=1)…")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(3)


def verify(m):
    ok = True
    for name, value, kind in TELEM_PARAMS:
        stored = p.read_param(m, name)
        good = p.matches(stored, value, kind)
        print(f"  {'✓' if good else '✗'} {name} = {stored} (want {value})")
        ok = ok and good
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", help="serial device (default: auto-detect /dev/cu.usbmodem*)")
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--no-reboot", action="store_true")
    a = ap.parse_args()

    m, dev = p.connect(a.device)
    if not m:
        print("✗ No FC found. Is the DEXI-3 plugged in via USB (shows as /dev/cu.usbmodem*)? "
              "Power-cycle and retry.")
        return 1
    print(f"connected on {dev} (sys {m.target_system})")

    if a.verify_only:
        return 0 if verify(m) else 1

    for name, value, kind in TELEM_PARAMS:
        for attempt in range(3):
            p.set_param(m, name, value, kind)
            if p.matches(p.read_param(m, name), value, kind):
                print(f"  set {name} = {value}")
                break
        else:
            print(f"  ✗ failed to set {name}")

    force_save(m)

    if not a.no_reboot:
        print("rebooting to confirm persistence…")
        p.nsh(m, "reboot", 0.5)
        time.sleep(7)
        m, dev = p.connect(a.device)
        if not m:
            print("✗ FC did not return after reboot (params were saved). Re-run --verify-only.")
            return 1

    print("\nverify:")
    ok = verify(m)
    print("\n✓ telem radio configured on VTX port (TELEM2 @ 57600, Normal)."
          if ok else "\n✗ verification failed — see above.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
