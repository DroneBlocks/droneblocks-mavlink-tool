"""Sanity-check a ulog that came off the FC over MAVLink.

A download can "complete" with the wrong bytes: LOG_ENTRY ids are positions in a
sorted list that shifts between connections, and an interrupted transfer can leave
the FC streaming LOG_DATA into the next one. The result opens, parses, and reads
as a plausible but wrong flight (2026-11-24: a 241 s flight read as 110 s, never
armed). Two checks catch it from the file alone:

- coverage: share of the file's bytes that parse into logged messages. Clean
  DEXI logs sit at 0.84-0.90; interleaved chunks drop it to ~0.5.
- time order: sensor_combined is published in order. A step back of more than
  0.5 s means a second log's bytes were spliced in. Isolated garbage timestamps
  (resync noise, seen in clean logs too) are dropped first.
"""
import io, os, sys, contextlib
import numpy as np

MIN_COVERAGE = 0.75
MAX_BACKSTEP_S = 0.5


def check(path):
    """Return (ok, reasons, info). info has duration, session path, armed."""
    from pyulog import ULog
    with contextlib.redirect_stdout(io.StringIO()):
        u = ULog(path)
    size = os.path.getsize(path)
    parsed = sum(len(d.data['timestamp']) * sum(v.dtype.itemsize for v in d.data.values())
                 for d in u.data_list)
    cov = parsed / size if size else 0.0
    reasons = []
    if cov < MIN_COVERAGE:
        reasons.append(f"only {cov:.0%} of the bytes parse (clean logs are 84-90%)")
    t0 = u.start_timestamp
    ts = [d.data['timestamp'].astype(np.float64) for d in u.data_list if d.name == 'sensor_combined']
    dur = (u.last_timestamp - t0) / 1e6
    if ts:
        t = (ts[0] - t0) / 1e6
        t = t[(t >= -1) & (t < 7 * 86400)]           # drop resync garbage
        back = np.diff(t) < -MAX_BACKSTEP_S
        if back.any():
            reasons.append(f"sensor_combined steps back in time {int(back.sum())}x "
                           f"(first at {t[1:][back][0]:.1f} s): bytes from another log")
        if len(t):
            dur = float(np.percentile(t, 99.9))
    sess = next((m.message.split()[-1] for m in u.logged_messages
                 if 'Opened full log file' in m.message), '?')
    armed = any(len(d.data['armed']) and d.data['armed'].any()
                for d in u.data_list if d.name == 'actuator_armed')
    return not reasons, reasons, dict(duration=dur, session=sess, armed=armed, coverage=cov)


if __name__ == '__main__':
    bad = 0
    for p in sys.argv[1:]:
        ok, why, info = check(p)
        print(f"{'OK ' if ok else 'BAD'} {p}  {info['duration']:.0f} s  "
              f"{'armed' if info['armed'] else 'never armed'}  {info['session']}  "
              f"coverage {info['coverage']:.2f}")
        for r in why: print(f"      {r}")
        bad += not ok
    sys.exit(1 if bad else 0)
