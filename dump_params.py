#!/usr/bin/env python3
"""
Dump ALL params from the connected FC to a sorted QGC-style .params file,
plus a version header. Reusable for any drone — use it to snapshot a golden
reference drone and to diff a new drone against it.

Usage:
    source venv/bin/activate
    python dump_params.py drone1_baseline.params
"""
import sys, time
from pymavlink import mavutil

import fcbench
import serial_ports

def connect():
    # Cross-platform: fcbench uses serial_ports.fc_ports() and opens the port
    # explicitly, so this works on Windows COM ports and Linux ttyACM too.
    m = fcbench.connect()
    return m, (serial_ports.fc_ports() or ['?'])[0]

def get_version(m):
    # ask for AUTOPILOT_VERSION
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0,0,0,0,0,0)
    msg = m.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
    if not msg: return {}
    def hexid(arr):
        try: return ''.join(f'{b:02x}' for b in arr)[:16]
        except Exception: return str(arr)
    return {
        'flight_sw_version': msg.flight_sw_version,
        'board_version': msg.board_version,
        'vendor_id': msg.vendor_id, 'product_id': msg.product_id,
        'flight_custom': hexid(msg.flight_custom_version),
    }

def dump_all(m):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params = {}; total = None
    last = time.time(); deadline = time.time()+40
    while time.time() < deadline:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is None:
            if total and len(params) >= total: break
            # re-request missing tail by listing again (cheap on PX4)
            if time.time()-last > 4:
                m.mav.param_request_list_send(m.target_system, m.target_component)
                last = time.time()
            continue
        total = msg.param_count
        pid = msg.param_id
        if isinstance(pid, bytes): pid = pid.decode(errors='ignore')
        pid = pid.rstrip('\x00')
        params[pid] = (msg.param_value, msg.param_type)
        last = time.time()
    return params, total

def main():
    if len(sys.argv) < 2: sys.exit("usage: dump_params.py <outfile.params>")
    out = sys.argv[1]
    m, port = connect()
    ver = get_version(m)
    params, total = dump_all(m)
    print(f"# captured {len(params)}/{total} params")
    with open(out, 'w') as f:
        f.write("# DEXI-3 param dump\n")
        for k,v in ver.items(): f.write(f"# {k}: {v}\n")
        f.write(f"# param_count: {total}  captured: {len(params)}\n")
        f.write("# MAV_ID\tCOMP_ID\tNAME\tVALUE\tTYPE\n")
        for name in sorted(params):
            val, typ = params[name]
            # integer-typed params: show as int
            if typ in (mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
                       mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
                       mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT32):
                import struct
                ival = struct.unpack('<i', struct.pack('<f', val))[0]
                f.write(f"1\t1\t{name}\t{ival}\t{typ}\n")
            else:
                f.write(f"1\t1\t{name}\t{val:.6g}\t{typ}\n")
    print(f"# wrote {out}")
    if ver:
        print("# version:", ", ".join(f"{k}={v}" for k,v in ver.items()))

if __name__ == "__main__":
    main()
