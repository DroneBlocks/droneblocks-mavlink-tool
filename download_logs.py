#!/usr/bin/env python3
"""
Download PX4 ulog(s) from the FC's SD card over MAVLink (USB serial).

Usage:
    source venv/bin/activate
    python download_logs.py 157 155 154 153      # download these log ids
    python download_logs.py --list               # just list logs

Writes ./logs/log_<id>.ulg. Chunked with gap-fill + re-request; tolerant of the
flaky USB CDC link (re-requests missing 90-byte blocks until complete).
"""
import sys, os, time, glob, math, argparse
from pymavlink import mavutil

CHUNK = 90  # LOG_DATA payload size

def connect():
    port = sorted(glob.glob('/dev/cu.usbmodem*'))[0]
    m = mavutil.mavlink_connection(port, baud=115200)
    for _ in range(4):
        hb = m.wait_heartbeat(timeout=6)
        if hb and m.target_system:
            print(f"# {port} sys {m.target_system} comp {m.target_component}")
            return m
    sys.exit("no heartbeat")

def get_list(m):
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
    logs={}; num=None; last=0; deadline=time.time()+15
    while time.time()<deadline:
        if time.time()-last>3:
            m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF); last=time.time()
        msg=m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
        if msg is None:
            if num is not None and len(logs)>=num: break
            continue
        num=msg.num_logs; logs[msg.id]=msg.size
        if num and len(logs)>=num: break
    return logs

def download(m, log_id, size, out):
    nchunks = math.ceil(size / CHUNK)
    buf = bytearray(size)
    have = bytearray(nchunks)
    got = 0
    t0 = time.time()
    def request_from(ofs):
        m.mav.log_request_data_send(m.target_system, m.target_component, log_id, ofs, 0xFFFFFFFF)
    request_from(0)
    last_progress = time.time()
    last_rerequest = time.time()
    stall_deadline = time.time() + 180  # hard cap per log
    while got < nchunks and time.time() < stall_deadline:
        msg = m.recv_match(type='LOG_DATA', blocking=True, timeout=1.5)
        now = time.time()
        if msg is not None and msg.id == log_id:
            idx = msg.ofs // CHUNK
            if 0 <= idx < nchunks and not have[idx]:
                n = msg.count
                buf[msg.ofs:msg.ofs+n] = bytes(msg.data[:n])
                have[idx] = 1; got += 1
                last_progress = now
        # if stalled, re-request from first missing block
        if now - last_progress > 1.2 and now - last_rerequest > 1.0:
            first_missing = have.find(b'\x00')
            if first_missing == -1:
                break
            request_from(first_missing * CHUNK)
            last_rerequest = now
            if now - t0 > 4 and int(now) % 3 == 0:
                pct = 100*got/nchunks
                print(f"\r  id {log_id}: {pct:5.1f}%  ({got}/{nchunks} blk, {got*CHUNK/1024:.0f} KB)", end="", flush=True)
    m.mav.log_request_end_send(m.target_system, m.target_component)
    dt = time.time()-t0
    pct = 100*got/nchunks
    with open(out, 'wb') as f:
        f.write(buf)
    print(f"\r  id {log_id}: {pct:5.1f}% complete  ({got*CHUNK/1024:.0f} KB in {dt:.0f}s) -> {out}")
    return pct >= 99.9

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*", type=int)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    m = connect()
    logs = get_list(m)
    if args.list or not args.ids:
        for i in sorted(logs): print(f"  id {i:3d}  {logs[i]/1024:8.0f} KB")
        return
    os.makedirs("logs", exist_ok=True)
    for log_id in args.ids:
        if log_id not in logs:
            print(f"  id {log_id}: NOT FOUND"); continue
        download(m, log_id, logs[log_id], f"logs/log_{log_id}.ulg")

if __name__ == "__main__":
    main()
