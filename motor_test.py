#!/usr/bin/env python3
"""Spin individual PX4 motors to verify motor mapping / spin direction.

This is the same operation as QGC's Setup > Motors screen: it sends
MAV_CMD_DO_MOTOR_TEST over MAVLink (through mavlink-router).

    python motor_test.py <host> <motor|all> [throttle_pct] [seconds]

Examples:
    python motor_test.py 192.168.4.1 1            # spin PX4 Motor 1 @ 6% for 3s
    python motor_test.py 192.168.4.1 2 8 3        # Motor 2 @ 8% for 3s
    python motor_test.py 192.168.4.1 all          # sweep 1..4, one at a time

*** PROPELLERS OFF. Keep hands clear. ***
"""
import sys
import time

from pymavlink import mavutil

MOTOR_TEST_THROTTLE_PERCENT = 0  # param2 type


def connect(host, port=14550):
    uri = f'udpout:{host}:{port}'
    m = mavutil.mavlink_connection(uri)
    # Nudge the FC so it starts streaming to us, then wait for a real sysid.
    deadline = time.time() + 15
    while time.time() < deadline:
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and m.target_system != 0:
            break
    if m.target_system == 0:
        raise SystemExit('No heartbeat from flight controller — check mavlink-router / host.')
    print(f'Connected to system {m.target_system}, component {m.target_component}', flush=True)
    return m


def spin(m, motor, throttle, secs):
    print(f'  Motor {motor}: {throttle}% for {secs}s ...', flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        motor,                         # param1: motor number (1-based, PX4 Motor function)
        MOTOR_TEST_THROTTLE_PERCENT,   # param2: throttle type = percent
        throttle,                      # param3: throttle value
        secs,                          # param4: timeout / run duration (s)
        0,                             # param5: motor count (0 = single)
        0,                             # param6: test order (0 = default)
        0)
    # let it run, then a beat of silence before the next motor
    time.sleep(secs + 1.0)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    host = sys.argv[1]
    which = sys.argv[2]
    throttle = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
    secs = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0

    m = connect(host)
    motors = [1, 2, 3, 4] if which.lower() == 'all' else [int(which)]
    for mot in motors:
        spin(m, mot, throttle, secs)
    print('Done.')


if __name__ == '__main__':
    main()
