#!/usr/bin/env python3
"""Cross-platform discovery of the FC's USB serial port(s).

This tool grew up on macOS, where a DEXI-3 FC always shows up as
/dev/cu.usbmodem*. On Windows it's COMx and on Linux /dev/ttyACM* — and a naive
"list every serial port" also grabs unrelated devices (Bluetooth SPP, USB-serial
adapters). So we filter by USB vendor ID: prefer VIDs seen on PX4 flight
controllers + their bootloaders, and fall back to any *real* (non-virtual) USB
serial port, which still excludes Bluetooth ports (they report no VID).

macOS behavior is deliberately unchanged — the proven /dev/cu.usbmodem* glob.
"""
import glob
import sys

# USB vendor IDs seen on PX4 FCs and their bootloaders:
#   0x0483 STMicro   — the DFU ROM loader (0483:df11) and boards that keep the
#                      STM VID for their CDC-ACM app/bootloader port
#   0x26AC 3DR/PX4   — classic PX4 CDC VID
#   0x3185 / 0x2DAE  — Auterion (newer PX4 boards)
#   0x1209 pid.codes — generic open-hardware USB VID
#   0x1B8C DroneBlocks — our own H743-AIO reports this, and it is NOT a PX4 VID,
#                        so without it the fallback branch below can hand back an
#                        unrelated USB serial device.
PX4_VIDS = {0x0483, 0x26AC, 0x3185, 0x2DAE, 0x1209, 0x1B8C}

# VIDs that enumerate as USB CDC next to an FC but must never be written to.
# 0x1915 Nordic Semiconductor — nRF dongles and the nRF Sniffer for Bluetooth LE
# 0x0403 FTDI / 0x10C4 SiLabs — generic USB-serial adapters (ESCs, radios, jigs)
BLOCKED_VIDS = {0x1915, 0x0403, 0x10C4}


def fc_ports():
    """Likely FC serial-port device names, most-likely first.

    macOS: /dev/cu.usbmodem* (unchanged). Windows/Linux: USB serial ports,
    preferring known PX4 vendor IDs, else any real (VID-bearing) USB serial port.
    """
    from serial.tools.list_ports import comports

    # Drop virtual/Bluetooth ports (no VID) and anything on the blocklist. The
    # blocklist matters: an nRF sniffer enumerates as /dev/cu.usbmodem* on macOS
    # exactly like the FC, so the old bare glob picked between them by luck of
    # sort order, and a param write could land on the wrong device.
    real = [p for p in comports()
            if p.vid is not None and p.vid not in BLOCKED_VIDS]
    known = sorted(p.device for p in real if p.vid in PX4_VIDS)
    if known:
        return known

    if sys.platform == "darwin":
        # Fall back to the historical glob, still minus the blocked devices.
        blocked = {p.device for p in comports() if p.vid in BLOCKED_VIDS}
        return [d for d in sorted(glob.glob("/dev/cu.usbmodem*")) if d not in blocked]

    return sorted(p.device for p in real)


def use_utf8_console():
    """Force UTF-8 stdout/stderr. Windows cmd defaults to cp1252, which raises
    UnicodeEncodeError on the box-drawing/emoji chars these scripts print
    (─ ✓ ✗ ✅ ⚠️). No-op where the stream is already UTF-8 or not reconfigurable."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


def pxup_port_arg():
    """The --port value to hand px_uploader.

    px_uploader globs wildcard patterns on unix, but on Windows it can't glob —
    it takes a literal comma-separated list of port names and tries each. So on
    Windows we hand it the detected COM port(s) first (fast hit), then a full
    COM0..63 sweep as a fallback in case the port re-enumerated after a reboot.
    """
    if sys.platform == "darwin":
        return "/dev/cu.usbmodem*"
    if sys.platform.startswith("win"):
        candidates = fc_ports() + [f"COM{i}" for i in range(64)]
        return ",".join(dict.fromkeys(candidates))        # de-dupe, keep order
    return "/dev/ttyACM*,/dev/cu.usbmodem*"               # linux/other unix
