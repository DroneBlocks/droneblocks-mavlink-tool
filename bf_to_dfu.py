#!/usr/bin/env python3
"""Reboot a Betaflight FC into its DFU bootloader over USB serial (no BOOT button).
Tries the CLI `bl` command, then falls back to an MSP_SET_REBOOT(bootloader) frame."""
import glob, sys, time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else (glob.glob("/dev/cu.usbmodem*") or [None])[0]
if not port:
    sys.exit("no /dev/cu.usbmodem* found")
print(f"opening {port}")
s = serial.Serial(port, 115200, timeout=1)
time.sleep(0.3)

# --- Method 1: CLI `bl` ---
s.write(b"#\r\n"); time.sleep(0.5)
resp = s.read(2000).decode(errors="replace")
print("CLI response after '#':", repr(resp[-200:]))
s.write(b"bl\r\n"); time.sleep(0.5)
print("sent 'bl'")

# --- Method 2: MSP_SET_REBOOT (cmd 68, payload 1 = reboot to bootloader) ---
try:
    frame = bytes([0x24, 0x4D, 0x3C, 0x01, 68, 0x01]) + bytes([0x01 ^ 68 ^ 0x01])
    s.write(frame); time.sleep(0.3)
    print("sent MSP_SET_REBOOT(bootloader)")
except Exception as e:
    print("msp reboot err", e)

s.close()
print("done — board should now enumerate as DFU (0483:df11)")
