#!/usr/bin/env python3
"""Get and set PX4 parameters over the network via mavlink-router.

Usage:
    python params.py <host>                          # List all parameters
    python params.py <host> EKF2_EV_CTRL             # Get one parameter
    python params.py <host> EKF2_EV_CTRL 1           # Set a parameter
    python params.py <host> -f EKF2                   # Filter params by prefix
    python params.py <host> --port 14550              # Custom UDP port

Examples:
    python params.py 192.168.68.60
    python params.py 192.168.68.60 EKF2_HGT_REF
    python params.py 192.168.68.60 EKF2_HGT_REF 2
    python params.py 192.168.68.60 -f EKF2_EV
"""

import argparse
import struct
import sys
import time

from pymavlink import mavutil

# MAV_PARAM_TYPE enum
PARAM_TYPES = {
    1: 'UINT8', 2: 'INT8', 3: 'UINT16', 4: 'INT16',
    5: 'UINT32', 6: 'INT32', 9: 'REAL32',
}


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


def encode_param_value(value, param_type):
    """Encode a value for MAVLink parameter setting."""
    if param_type == 9:  # REAL32
        return float(value)
    int_val = int(value)
    if param_type in (1, 3, 5):  # unsigned
        int_bytes = struct.pack('I', int_val & 0xFFFFFFFF)
    elif param_type in (2, 4, 6):  # signed
        int_bytes = struct.pack('i', int_val)
    else:
        return float(value)
    return struct.unpack('f', int_bytes)[0]


def connect(host, port):
    """Connect to FC via mavlink-router and return connection."""
    uri = f'udpout:{host}:{port}'
    conn = mavutil.mavlink_connection(uri, source_system=255, source_component=0)

    # Send GCS heartbeat to register with mavlink-router (Server mode)
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print(f'No heartbeat from {uri}. Is mavlink-router running?')
        sys.exit(1)

    return conn


def fetch_all_params(conn, timeout=60):
    """Fetch all parameters. Returns dict of {name: (value, type)}."""
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
            params[pid] = (decode_param_value(msg.param_value, msg.param_type), msg.param_type)
            param_count = msg.param_count
            if len(params) % 100 == 0:
                print(f'  ...received {len(params)}/{param_count}', file=sys.stderr)
            if param_count > 0 and len(params) >= param_count:
                break
        elif time.time() - last_msg > 5.0 and params:
            break

    return params


def fetch_one_param(conn, name, timeout=5):
    """Fetch a single parameter by name. Returns (value, type) or None."""
    name_bytes = name.encode('utf-8')
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        name_bytes.ljust(16, b'\x00')[:16], -1
    )

    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if msg:
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('utf-8').rstrip('\x00')
            if pid == name:
                return (decode_param_value(msg.param_value, msg.param_type), msg.param_type)
    return None


def set_param(conn, name, value, timeout=5):
    """Set a parameter. Returns new value on success, None on failure."""
    # First fetch to get the type
    result = fetch_one_param(conn, name)
    if result is None:
        print(f'Parameter {name} not found')
        return None

    current_value, param_type = result

    # Parse value based on type
    if param_type == 9:  # REAL32
        new_value = float(value)
    else:
        new_value = int(float(value))

    encoded = encode_param_value(new_value, param_type)

    name_bytes = name.encode('utf-8')
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name_bytes.ljust(16, b'\x00')[:16],
        encoded, param_type
    )

    # Wait for confirmation
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if msg:
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('utf-8').rstrip('\x00')
            if pid == name:
                confirmed = decode_param_value(msg.param_value, msg.param_type)
                return confirmed

    return None


def format_value(value, param_type):
    """Format a parameter value for display."""
    if param_type == 9:
        if value == int(value):
            return f'{int(value)}.0'
        return f'{value}'
    return str(int(value))


def main():
    parser = argparse.ArgumentParser(
        description='Get and set PX4 parameters over the network via mavlink-router.',
        epilog='Examples:\n'
               '  python params.py 192.168.68.60                      # List all\n'
               '  python params.py 192.168.68.60 EKF2_HGT_REF         # Get one\n'
               '  python params.py 192.168.68.60 EKF2_HGT_REF 2       # Set one\n'
               '  python params.py 192.168.68.60 -f EKF2               # Filter\n',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('host', help='Raspberry Pi IP address')
    parser.add_argument('param', nargs='?', help='Parameter name to get or set')
    parser.add_argument('value', nargs='?', help='New value (omit to read)')
    parser.add_argument('-f', '--filter', help='Filter params by prefix')
    parser.add_argument('--port', type=int, default=14550, help='UDP port (default: 14550)')

    args = parser.parse_args()

    print(f'Connecting to {args.host}:{args.port}...', file=sys.stderr)
    conn = connect(args.host, args.port)
    print(f'Connected to system {conn.target_system}', file=sys.stderr)

    if args.param and args.value is not None:
        # SET mode
        old = fetch_one_param(conn, args.param)
        if old is None:
            print(f'Parameter {args.param} not found')
            sys.exit(1)

        old_value, param_type = old
        print(f'{args.param} = {format_value(old_value, param_type)} (current)', file=sys.stderr)

        # Reconnect since fetch consumed messages
        conn = connect(args.host, args.port)
        confirmed = set_param(conn, args.param, args.value)

        if confirmed is not None:
            print(f'{args.param} = {format_value(confirmed, param_type)}')
            if confirmed != old_value:
                print(f'Changed from {format_value(old_value, param_type)}', file=sys.stderr)
            else:
                print(f'Value unchanged', file=sys.stderr)
        else:
            print(f'Failed to set {args.param}')
            sys.exit(1)

    elif args.param:
        # GET one param
        result = fetch_one_param(conn, args.param)
        if result is None:
            print(f'Parameter {args.param} not found')
            sys.exit(1)
        value, param_type = result
        print(f'{args.param} = {format_value(value, param_type)}')

    else:
        # LIST mode (all or filtered)
        params = fetch_all_params(conn)
        prefix = args.filter.upper() if args.filter else None

        count = 0
        for name in sorted(params.keys()):
            if prefix and not name.startswith(prefix):
                continue
            value, param_type = params[name]
            print(f'{name} = {format_value(value, param_type)}')
            count += 1

        print(f'\n{count} parameters', file=sys.stderr)


if __name__ == '__main__':
    main()
