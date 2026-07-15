#!/usr/bin/env python3
"""Batch flasher for DroneBlocks H743-AIO (DEXI-3 FC) — flash N boards back-to-back.

ZERO keypresses per board. It auto-detects whatever you plug in and does the right
thing, so a 10-drone run is just plug → (wait) → unplug → next:

  • BARE / foreign board (in DFU: hold BOOT while plugging USB)
        → full flash: bootloader (dfu-util) + app (px_uploader) + DEXI-3 params
  • ALREADY-FLASHED DroneBlocks board (plug in NORMALLY, no BOOT button)
        → px_uploader reboots its existing bootloader and re-flashes the SAME app
          build (keeps the fleet byte-identical) + DEXI-3 params
        → with --params-only: skips the app flash, just (re)writes params

So: bare boards need the BOOT button once (for the bootloader); boards that already
have a DroneBlocks bootloader don't need it at all.

Run:  ./venv/bin/python flash_batch.py                 # flash until Ctrl-C
      ./venv/bin/python flash_batch.py --count 10      # stop after 10 boards
      ./venv/bin/python flash_batch.py --params-only   # already-flashed → just params

The firmware version (from firmware/droneblocks-h743-aio/manifest.json) is printed up
front and on every PASS line, so the run is self-documenting.
"""
import argparse, json, os, subprocess, sys, time
from shutil import which
from serial_ports import fc_ports, pxup_port_arg

HERE       = os.path.dirname(os.path.abspath(__file__))
FWDIR      = os.path.join(HERE, "firmware")
ASSETS     = os.path.join(FWDIR, "droneblocks-h743-aio")
BOOTLOADER = os.path.join(ASSETS, "droneblocks_h743-aio_bootloader.bin")
APP        = os.path.join(ASSETS, "droneblocks_h743-aio_default.px4")
MANIFEST   = os.path.join(ASSETS, "manifest.json")
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


def app_heartbeat(timeout=6):
    """Return a port if a PX4 *app* (not just the bootloader) is alive, else None."""
    from pymavlink import mavutil
    end = time.time() + timeout
    while time.time() < end:
        for d in usbmodems():
            try:
                c = mavutil.mavlink_connection(d, baud=57600, source_system=255, source_component=190)
                c.mav.heartbeat_send(6, 8, 0, 0, 0); t0 = time.time()
                while time.time() - t0 < 4:
                    c.mav.heartbeat_send(6, 8, 0, 0, 0)
                    hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=0.4)
                    if hb and (hb.autopilot == 12 or hb.get_srcComponent() == 1):
                        c.close(); return d
                c.close()
            except Exception:
                pass
        time.sleep(1)
    return None


def detect_state():
    """What's plugged in right now? 'dfu' | 'app' | 'usb' (bootloader/unknown CDC) | None."""
    if in_dfu():
        return "dfu"
    if usbmodems():
        return "app" if app_heartbeat(6) else "usb"
    return None


# ── stages ──────────────────────────────────────────────────────────────────
def stage_bootloader_dfu():
    """DFU bootloader write, verify+retry on the transient-glitch failure mode."""
    print("  [bootloader] DFU write…")
    for attempt in range(1, 6):
        if not in_dfu() and not wait(in_dfu, 15):
            print("    ✗ board left DFU — re-enter DFU (hold BOOT, replug).")
            return False
        r = subprocess.run(["dfu-util", "-a", "0", "-d", "0483:df11",
                            "-s", "0x08000000:leave", "-D", BOOTLOADER],
                           capture_output=True, text=True)
        wrote = "File downloaded successfully" in (r.stdout + r.stderr)
        up = wait(lambda: bool(usbmodems()), 20)   # PX4 bootloader CDC must appear
        if wrote and up:
            print("    ✓ bootloader installed (PX4 bootloader enumerated)")
            return True
        print(f"    ✗ write didn't take (try {attempt}/5) — transient glitch, retrying…")
        wait(in_dfu, 8)
    print("    ✗ bootloader failed after 5 tries — re-enter DFU and restart this board.")
    return False


def stage_app():
    """Flash the app. px_uploader auto-reboots a running app into its bootloader,
    so this works whether the board is sitting in the bootloader OR running an app."""
    print("  [app] flashing firmware (px_uploader auto-reboots if app is running)…")
    rc = subprocess.run([PY, "-u", PXUP, "--port", pxup_port_arg(), APP],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
    dev = app_heartbeat(60)
    if not dev:
        print("    ✗ app did not boot — unplug/replug this board and restart it.")
        return False
    print(f"    ✓ app running on {dev}")
    return True


def stage_params():
    print("  [params] provisioning DEXI-3 profile…")
    rc = subprocess.run([PY, PROVISION, "--no-reboot"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
    if rc != 0:
        print("    ✗ provision reported issues — run provision_dexi3_flow.py --verify-only to inspect.")
        return False
    print("    ✓ params provisioned (PROVISION PASS)")
    return True


def flash_board(state, params_only):
    """Drive the right stages for the detected board state. Returns True on success."""
    if state == "dfu":
        if params_only:
            print("  ⚠️  board is in DFU (no app yet) — --params-only can't apply; doing a full flash.")
        return stage_bootloader_dfu() and stage_app() and stage_params()

    # 'app' or 'usb': a DroneBlocks bootloader is already present.
    if params_only:
        if state != "app":
            print("  ⚠️  no running app to talk to (board in bootloader) — flashing app first, then params.")
            return stage_app() and stage_params()
        print("  (params-only: keeping existing firmware)")
        return stage_params()
    return stage_app() and stage_params()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=0, help="stop after N boards (0 = until Ctrl-C)")
    ap.add_argument("--params-only", action="store_true",
                    help="for already-flashed boards: skip the app flash, only (re)write params")
    args = ap.parse_args()

    for f in (BOOTLOADER, APP, PXUP, PROVISION):
        if not os.path.exists(f):
            sys.exit(f"missing asset: {f}\n(run firmware/fetch-latest.sh, or git pull)")
    if not which("dfu-util"):
        hint = "winget install dfu-util" if sys.platform.startswith("win") else "brew install dfu-util"
        sys.exit(f"dfu-util not found — install it ({hint}) and ensure it's on PATH")

    m = json.load(open(MANIFEST))
    print("=" * 72)
    print(" DroneBlocks H743-AIO — BATCH flash")
    print(f"   board   : {m['board']} (board_id {m['board_id']})")
    print(f"   firmware: {m['version']}  built {m['built_at']}")
    print(f"   mode    : {'PARAMS-ONLY (keep existing firmware)' if args.params_only else 'flash firmware + params'}")
    print(f"   target  : {f'{args.count} boards' if args.count else 'until Ctrl-C'}")
    print("=" * 72)
    print(" Bare board → hold BOOT + plug (DFU).  Already-flashed → just plug in.")

    ok = fail = n = 0
    try:
        while True:
            n += 1
            print(f"\n┌─ DRONE {n} " + "─" * 58)
            print("│  Plug in a board…  (waiting)")
            if not wait(lambda: detect_state() is not None, 600):
                print("│  nothing plugged in for 10 min — stopping.")
                n -= 1
                break
            state = detect_state()
            label = {"dfu": "DFU (bare) → full flash",
                     "app": "running app → re-flash + params",
                     "usb": "bootloader → flash + params"}[state]
            print(f"│  ✓ detected: {label}  (hands off until done)")
            good = flash_board(state, args.params_only)
            if good:
                ok += 1
                print(f"└─ ✅ DRONE {n} DONE ({m['version']}).  Unplug it; plug in the next board.")
            else:
                fail += 1
                print(f"└─ ⚠️  DRONE {n} FAILED — see above.  Unplug, fix, retry as the next board.")
            if args.count and n >= args.count:
                break
            print("   …waiting for you to unplug this board…")
            wait(lambda: detect_state() is None, 600)
    except KeyboardInterrupt:
        n -= 1

    print("\n" + "=" * 72)
    print(f" BATCH COMPLETE — {ok} OK, {fail} failed, {n} boards handled.")
    print("=" * 72)
    sys.exit(0 if fail == 0 else 1)


if __name__ == "__main__":
    main()
