#!/usr/bin/env python3
"""Shared plumbing for bench tests that stream sensor data off a connected FC.

Two things every bench script needs and nobody should copy-paste again:

1. ``connect()`` — cross-platform, via ``serial_ports.fc_ports()``. Works on
   Windows COM ports and Linux /dev/ttyACM* as well as macOS, so a bench test
   runs the same on the Mac and on rogbeast.
2. ``patch_pymavlink()`` — pymavlink 2.4.49 raises ``TypeError`` when it caches an
   *instanced* message (anything with a sensor index, which includes
   DISTANCE_SENSOR). Without this, any script subscribing to those messages dies
   a few seconds in. Call it before ``mavutil.mavlink_connection``.
"""
import sys
import time

from pymavlink import mavutil

import serial_ports

_PATCHED = False


def patch_pymavlink():
    """Work around the pymavlink 2.4.49 instanced-message TypeError. Idempotent."""
    global _PATCHED
    if _PATCHED:
        return
    original = mavutil.add_message

    def safe_add(messages, mtype, msg):
        try:
            return original(messages, mtype, msg)
        except TypeError:
            prev = messages.get(mtype)
            if prev is not None and getattr(prev, '_instances', None) is None:
                prev._instances = {}
            try:
                return original(messages, mtype, msg)
            except TypeError:
                messages[mtype] = msg

    mavutil.add_message = safe_add
    _PATCHED = True


def connect(baud=115200, timeout=20, quiet=False):
    """Return a heartbeating mavlink connection to the first FC we can find."""
    patch_pymavlink()
    ports = serial_ports.fc_ports()
    if not ports:
        sys.exit("no flight controller found — is it plugged in and powered?")
    port = ports[0]
    m = mavutil.mavlink_connection(port, baud=baud)
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = m.wait_heartbeat(timeout=6)
        if hb and m.target_system:
            if not quiet:
                print(f"# {port} sys {m.target_system} comp {m.target_component}")
            return m
    sys.exit(f"no heartbeat on {port}")


def request(m, msg_ids, interval_us=10000):
    """Ask the FC to stream the given MAVLink message IDs.

    Common ids: 106 OPTICAL_FLOW_RAD, 132 DISTANCE_SENSOR, 105 HIGHRES_IMU,
    30 ATTITUDE, 148 AUTOPILOT_VERSION.
    """
    for mid in msg_ids:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mid, interval_us, 0, 0, 0, 0, 0)
        time.sleep(0.15)
