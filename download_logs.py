#!/usr/bin/env python3
"""
Download PX4 ulog(s) from the FC's SD card over MAVLink (USB serial).

Usage:
    source venv/bin/activate
    python download_logs.py 157 155 154 153      # download these log ids
    python download_logs.py --list               # just list logs

Writes ./logs/log_<id>.ulg, and REFUSES to overwrite an existing one
(log ids restart per aircraft). Use flightlog.py sync for board-filed storage.
 Chunked with gap-fill + re-request; tolerant of the
flaky USB CDC link (re-requests missing 90-byte blocks until complete).
"""
import sys, os, time, glob, math, argparse
from pymavlink import mavutil

CHUNK = 90  # LOG_DATA payload size

def connect():
    # Cross-platform via fcbench: serial_ports.fc_ports() for discovery, and the
    # port opened explicitly so a Windows COM name is not mistaken for a log file.
    import fcbench
    return fcbench.connect()

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

def _board_of(path):
    """sys_uuid tail of an existing ulog, or None. Used only to name the owner in
    the overwrite refusal, so a missing pyulog must not break downloading."""
    try:
        from pyulog import ULog
        return ULog(path, message_name_filter_list=[]).msg_info_dict.get('sys_uuid', '')[-8:] or None
    except Exception:
        return None


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
    # Give up when the link goes quiet, not on wall-clock: a healthy USB link
    # does ~800 KB/s, so the old fixed 180s cap silently truncated big logs.
    # MAX is a backstop for a link that trickles forever, ~100x slower than healthy.
    STALL = 20.0
    MAX = max(60.0, size / 8000.0)
    while got < nchunks and time.time() - t0 < MAX:
        msg = m.recv_match(type='LOG_DATA', blocking=True, timeout=1.5)
        now = time.time()
        if msg is not None and msg.id == log_id:
            idx = msg.ofs // CHUNK
            if 0 <= idx < nchunks and not have[idx]:
                # Clamp: a final chunk reported as a full 90 bytes would otherwise
                # run past `size`, and bytearray slice-assignment GROWS the buffer
                # rather than truncating, appending garbage past the real EOF.
                n = min(msg.count, size - msg.ofs)
                if n > 0:
                    buf[msg.ofs:msg.ofs+n] = bytes(msg.data[:n])
                have[idx] = 1; got += 1
                last_progress = now
        # if stalled, re-request from first missing block
        if now - last_progress > 1.2 and now - last_rerequest > 1.0:
            first_missing = have.find(b'\x00')
            if first_missing == -1:
                break
            if now - last_progress > STALL:
                break                      # FC has gone quiet despite re-requests
            request_from(first_missing * CHUNK)
            last_rerequest = now
            if now - t0 > 4 and int(now) % 3 == 0:
                pct = 100*got/nchunks
                print(f"\r  id {log_id}: {pct:5.1f}%  ({got}/{nchunks} blk, {got*CHUNK/1024:.0f} KB)", end="", flush=True)
    m.mav.log_request_end_send(m.target_system, m.target_component)
    dt = time.time()-t0
    complete = got >= nchunks

    # Never write a partial ulog. Missing blocks stay zero in the pre-allocated
    # buffer, so the file lands at the right size under the right name and looks
    # fine — but Flight Review rejects it. One 90-byte hole is enough: in block 0
    # it fails header parsing, early in the data section it raises KeyError, and
    # mid-file it can leave every dataset empty. Writing it anyway is how the
    # gap-riddled logs in logs/ got there.
    if not complete:
        missing = nchunks - got
        print(f"\r  id {log_id}: FAILED at {100*got/nchunks:5.1f}% "
              f"({missing} of {nchunks} blocks missing after {dt:.0f}s). Nothing written — "
              f"a ulog with gaps will not open in Flight Review. Re-run to retry.")
        return False
    with open(out, 'wb') as f:
        f.write(buf)
    print(f"\r  id {log_id}: 100.0% complete  ({got*CHUNK/1024:.0f} KB in {dt:.0f}s) -> {out}")
    return True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*", type=int)
    ap.add_argument("--list", action="store_true")
    ap.add_argument('--force', action='store_true',
                    help='overwrite an existing logs/log_<id>.ulg (may belong to another aircraft)')
    args = ap.parse_args()
    m = connect()
    logs = get_list(m)
    if args.list or not args.ids:
        for i in sorted(logs): print(f"  id {i:3d}  {logs[i]/1024:8.0f} KB")
        return
    os.makedirs("logs", exist_ok=True)
    failed = []
    for log_id in args.ids:
        if log_id not in logs:
            print(f"  id {log_id}: NOT FOUND"); failed.append(log_id); continue
        out = f"logs/log_{log_id}.ulg"
        # Log ids restart per aircraft, so logs/log_25.ulg from a second airframe
        # lands on the first one's file with no warning. Four boards already share
        # this directory and two of their id ranges overlap. Refuse rather than
        # silently destroy a flight we cannot re-fly.
        if os.path.exists(out) and not args.force:
            owner = _board_of(out)
            print(f"  id {log_id}: REFUSING, {out} already exists"
                  + (f" and belongs to board ...{owner}" if owner else ""))
            print(f"             rename it, use --out, or pass --force to overwrite.")
            failed.append(log_id); continue
        if not download(m, log_id, logs[log_id], out):
            failed.append(log_id)
    if failed:
        # Exit non-zero so a batch failure is visible to whatever called this.
        print(f"\n{len(failed)} of {len(args.ids)} log(s) did not download: "
              f"{' '.join(str(i) for i in failed)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
