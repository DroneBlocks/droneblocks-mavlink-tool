#!/usr/bin/env python3
"""Read or set FC parameters over USB, safely.

    python setp.py NAME                 # read
    python setp.py NAME VALUE [...]     # set pairs, verify each

INT32 params travel as raw bits in a float field. Sending float(3) writes
1077936128 instead of 3, so ints are bit-cast on both read and write. Every write
is read back and compared; a mismatch is printed, not swallowed.
"""
import sys, time, struct
import fcbench
from pymavlink import mavutil

REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32


def read(m, name):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 4:
        x = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if x and x.param_id.strip('\x00') == name:
            if x.param_type == INT32:
                return struct.unpack('<i', struct.pack('<f', x.param_value))[0], 'int'
            return x.param_value, 'float'
    return None, None


def main():
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    m = fcbench.connect(quiet=True)
    if len(a) == 1:
        v, k = read(m, a[0])
        print('{} = {} ({})'.format(a[0], v, k))
        return
    for i in range(0, len(a) - 1, 2):
        name, val = a[i], float(a[i + 1])
        before, kind = read(m, name)
        if kind is None:
            print('  ??  {:<18} not found'.format(name)); continue
        if kind == 'int':
            bits = struct.unpack('<f', struct.pack('<i', int(val)))[0]
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(), bits, INT32)
        else:
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(), val, REAL32)
        time.sleep(1.2)
        after, _ = read(m, name)
        ok = after is not None and abs(float(after) - val) < 1e-4
        print('  {}  {:<18} {} -> {}   ({})'.format('OK ' if ok else '** MISMATCH **',
                                                    name, before, after, kind))


if __name__ == '__main__':
    main()
