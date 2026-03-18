#!/usr/bin/env python3
"""Fetch and display all parameters from a PX4 flight controller via mavlink-router."""

import struct
import sys
import time

from pymavlink import mavutil


def decode_param_value(float_value, param_type):
    """Decode MAVLink parameter value based on its type."""
    if param_type == 9:  # REAL32
        return float_value
    float_bytes = struct.pack('f', float_value)
    if param_type in (5, 6):  # UINT32/INT32
        return struct.unpack('i' if param_type == 6 else 'I', float_bytes)[0]
    elif param_type in (3, 4):  # UINT16/INT16
        v = struct.unpack('I', float_bytes)[0] & 0xFFFF
        return v if param_type == 3 else (v if v < 32768 else v - 65536)
    elif param_type in (1, 2):  # UINT8/INT8
        v = struct.unpack('I', float_bytes)[0] & 0xFF
        return v if param_type == 1 else (v if v < 128 else v - 256)
    return float_value


def fetch_params(host, port=14550, timeout=60):
    uri = f'udpout:{host}:{port}'
    print(f'Connecting to {uri}...')
    conn = mavutil.mavlink_connection(uri, source_system=255, source_component=0)

    # Send a GCS heartbeat to register with mavlink-router (Server mode)
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    print('Waiting for heartbeat...')
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print('No heartbeat received. Is mavlink-router running?')
        sys.exit(1)
    print(f'Connected — system={conn.target_system}, component={conn.target_component}')

    print('Requesting parameters...')
    conn.mav.param_request_list_send(conn.target_system, conn.target_component)

    params = {}
    param_count = 0
    start = time.time()
    last_msg = start

    while time.time() - start < timeout:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if msg:
            last_msg = time.time()
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('utf-8').rstrip('\x00')
            params[pid] = decode_param_value(msg.param_value, msg.param_type)
            param_count = msg.param_count
            if len(params) % 100 == 0:
                print(f'  ...received {len(params)}/{param_count}')
            if param_count > 0 and len(params) >= param_count:
                break
        elif time.time() - last_msg > 5.0 and params:
            break

    print(f'\nReceived {len(params)}/{param_count} parameters\n')
    for name in sorted(params.keys()):
        print(f'{name} = {params[name]}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f'Usage: {sys.argv[0]} <host> [port]')
        print(f'  host  Raspberry Pi IP (e.g. 192.168.68.59)')
        print(f'  port  UDP port (default: 14550)')
        sys.exit(1)

    host = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 14550
    fetch_params(host, port)
