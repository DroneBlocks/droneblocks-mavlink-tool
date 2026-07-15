#!/usr/bin/env python3
"""Interactive end-to-end flash for a fresh DroneBlocks H743-AIO (DEXI-3 FC), over USB.

One command, one physical step (the DFU button). It walks you through:
  [1] bootloader  → dfu-util  (firmware/droneblocks-h743-aio/*_bootloader.bin)
  [2] app firmware→ px_uploader (DroneBlocks PX4 branch, *_default.px4)
  [3] DEXI-3 params→ provision_dexi3_flow.py (indoor optical flow)

Verified live 2026-06-11: after the DFU `:leave`, the board lands in the fresh
bootloader and px_uploader syncs immediately — no replug — and the app boots
clean. The only thing you touch is the BOOT button.

Run:  ./venv/bin/python flash_new_fc.py
"""
import os, subprocess, sys, time
from shutil import which
from serial_ports import fc_ports, pxup_port_arg

HERE       = os.path.dirname(os.path.abspath(__file__))
FWDIR      = os.path.join(HERE, "firmware")
ASSETS     = os.path.join(FWDIR, "droneblocks-h743-aio")
BOOTLOADER = os.path.join(ASSETS, "droneblocks_h743-aio_bootloader.bin")
APP        = os.path.join(ASSETS, "droneblocks_h743-aio_default.px4")
PXUP       = os.path.join(FWDIR, "px_uploader.py")
PROVISION  = os.path.join(HERE, "provision_dexi3_flow.py")
PY         = sys.executable

def usbmodems():
    return fc_ports()   # cross-platform (macOS usbmodem / Windows COM / Linux ttyACM)

def in_dfu():
    out = subprocess.run(["dfu-util", "-l"], capture_output=True, text=True).stdout
    return "0483:df11" in out

def wait(cond, timeout, poll=1.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(poll)
    return False

def wait_app(timeout=60):
    """Confirm the app booted by getting a PX4 autopilot heartbeat (MAVLink-first)."""
    from pymavlink import mavutil
    end = time.time() + timeout
    while time.time() < end:
        for d in usbmodems():
            try:
                c = mavutil.mavlink_connection(d, baud=57600, source_system=255, source_component=190)
                c.mav.heartbeat_send(6, 8, 0, 0, 0); t0 = time.time()
                while time.time() - t0 < 6:
                    c.mav.heartbeat_send(6, 8, 0, 0, 0)
                    hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=0.4)
                    if hb and (hb.autopilot == 12 or hb.get_srcComponent() == 1):
                        c.close(); return d
                c.close()
            except Exception:
                pass
        time.sleep(2)
    return None

def main():
    for f in (BOOTLOADER, APP, PXUP, PROVISION):
        if not os.path.exists(f):
            sys.exit(f"missing asset: {f}\n(run firmware/fetch-latest.sh, or git pull)")
    if not which("dfu-util"):
        hint = "winget install dfu-util" if sys.platform.startswith("win") else "brew install dfu-util"
        sys.exit(f"dfu-util not found — install it ({hint}) and ensure it's on PATH")

    print("=" * 66)
    print(" DroneBlocks H743-AIO — full flash: bootloader + PX4 + DEXI-3 params")
    print("=" * 66)

    # ── [1/3] Bootloader via DFU ───────────────────────────────────────────
    # The DFU write can glitch mid-download (transient USB) and leave the chip
    # with no working bootloader — and a glitched write also wedges the DFU
    # descriptors, so recovery is: unplug, re-enter DFU clean, re-flash. We VERIFY
    # the PX4 bootloader actually enumerates and retry until it does (never march
    # on with a half-flashed board).
    print("\n[1/3] BOOTLOADER (DFU)")
    print("  Unplug the FC, then HOLD the BOOT button, plug in USB, hold ~2 s, release.")
    for attempt in range(1, 6):
        input(f"  → (try {attempt}) Press Enter once it's plugged in while holding BOOT… ")
        if not wait(in_dfu, 25):
            print("  ✗ no DFU device — hold BOOT *while* plugging in. Try again.")
            continue
        print("  ✓ DFU detected — flashing bootloader → 0x08000000")
        r = subprocess.run(["dfu-util", "-a", "0", "-d", "0483:df11",
                            "-s", "0x08000000:leave", "-D", BOOTLOADER],
                           capture_output=True, text=True)
        log = r.stdout + r.stderr
        wrote = "File downloaded successfully" in log
        # benign: dfu-util exits non-zero on the ':leave' get_status quirk — ignore that.
        up = wait(lambda: bool(usbmodems()), 20)   # PX4 bootloader CDC must appear
        if wrote and up:
            print("  ✓ bootloader installed and running (PX4 bootloader enumerated)")
            break
        print("  ✗ write didn't take (no PX4 bootloader came up — transient DFU glitch).")
        print("    Unplug the FC and re-enter DFU (hold BOOT, replug) — we'll re-flash.")
    else:
        sys.exit("  ✗ bootloader flash failed after retries. Recover DFU and re-run.")

    # ── [2/3] App firmware via px_uploader ─────────────────────────────────
    print("\n[2/3] APP FIRMWARE (DroneBlocks PX4 branch)")
    print("  (if it says 'Waiting for bootloader', unplug/replug USB to catch it)")
    rc = subprocess.run([PY, "-u", PXUP, "--port", pxup_port_arg(), APP]).returncode
    print("  waiting for the app to boot…")
    dev = wait_app(60)
    if not dev:
        input("  app not up — unplug ~5 s and replug USB (power cycle), then press Enter… ")
        dev = wait_app(60)
        if not dev:
            sys.exit("  ✗ app did not boot — check the px_uploader output above.")
    print(f"  ✓ app running on {dev}")

    # ── [3/3] DEXI-3 params ────────────────────────────────────────────────
    print("\n[3/3] DEXI-3 INDOOR-FLOW PARAMS")
    rc = subprocess.run([PY, PROVISION, "--no-reboot"]).returncode

    print("\n" + "=" * 66)
    print(" ✅ DONE — bootloader + PX4 + params flashed." if rc == 0
          else " ⚠️  Flash done, but provision reported issues — see above.")
    print("=" * 66)
    sys.exit(rc)

if __name__ == "__main__":
    main()
