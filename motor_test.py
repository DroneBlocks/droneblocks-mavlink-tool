#!/usr/bin/env python3
"""Spin individual PX4 motors to verify motor mapping / spin direction.

This is the same operation as QGC's Actuators > Motors sliders: it sends
MAV_CMD_ACTUATOR_TEST over MAVLink, either through mavlink-router (a host)
or straight to the FC over USB (a /dev/ path). Needs COM_MOT_TEST_EN=1 and
a disarmed vehicle. PX4 1.16 does NOT handle MAV_CMD_DO_MOTOR_TEST (it acks
UNSUPPORTED), which is why this script no longer sends it.

    python motor_test.py <host|/dev/port> <motor|all> [throttle_pct] [seconds]

Examples:
    python motor_test.py 192.168.4.1 1            # spin PX4 Motor 1 @ 6% for 3s
    python motor_test.py 192.168.4.1 2 8 3        # Motor 2 @ 8% for 3s
    python motor_test.py 192.168.4.1 all          # sweep 1..4, one at a time
    python motor_test.py /dev/cu.usbmodem01 all 25 1   # over USB

*** PROPELLERS OFF. Keep hands clear. ***
"""
import sys
import time

from pymavlink import mavutil

MAV_CMD_ACTUATOR_TEST = 310  # not in pymavlink's bundled ardupilotmega dialect
ACTUATOR_TEST_MAX_TIMEOUT = 3.0  # PX4 caps each command at 3s; refresh to run longer
ACK_RESULTS = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED', 4: 'FAILED'}


def connect(host, port=14550):
    if host.startswith('/dev/'):
        m = mavutil.mavlink_connection(host, baud=57600)
    else:
        m = mavutil.mavlink_connection(f'udpout:{host}:{port}')
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
        raise SystemExit('No heartbeat from flight controller — check mavlink-router / host / USB port.')
    print(f'Connected to system {m.target_system}, component {m.target_component}', flush=True)
    return m


def spin(m, motor, throttle, secs):
    print(f'  Motor {motor}: {throttle}% for {secs}s ...', flush=True)
    end = time.time() + secs
    while True:
        left = end - time.time()
        if left <= 0:
            break
        m.mav.command_long_send(
            m.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            MAV_CMD_ACTUATOR_TEST, 0,
            throttle / 100.0,                          # param1: value, 0..1 for motors
            min(left, ACTUATOR_TEST_MAX_TIMEOUT),      # param2: timeout (s), motor stops after it
            0, 0,
            motor,                                     # param5: output function (1 = Motor 1)
            0, 0)
        ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if ack is None or ack.result != 0:
            result = 'no ACK' if ack is None else ACK_RESULTS.get(ack.result, ack.result)
            raise SystemExit(f'  Motor {motor}: rejected ({result}) — disarmed? COM_MOT_TEST_EN=1?')
        time.sleep(min(left, ACTUATOR_TEST_MAX_TIMEOUT - 0.5))
    # a beat of silence before the next motor
    time.sleep(1.0)


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
