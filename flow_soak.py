#!/usr/bin/env python3
"""Static optical-flow soak: does the flow module drop out with nothing moving?

Park the aircraft on a box roughly 1.2 m above a patch of floor the sensor reads
cleanly. PROPS OFF, battery in, USB connected. Nothing moves for the whole run,
which takes the floor, the airframe's motion, the controller and the estimator
out of the loop and leaves only the sensor.

    python flow_soak.py --minutes 10 --label cf

Reading the result. On a UP-T201 the reported quality is a BINARY VALID FLAG,
not a gradient: the module sends 245 when it has a fix and 0 when it does not.
PX4 republishes at half the module's rate and averages two raw reads
(VehicleOpticalFlow.cpp), so over MAVLink you see 245 (both good), ~122 (one of
the two blind) or 0 (both blind). True sensor-level blind rate is therefore
    %(q==0) + 0.5 * %(q==122)

Any sustained dropout here indicts the module. A clean run means the sensor is
sound and flight failures are coming from the surface it is looking at.
"""
import argparse
import json
import time

import numpy as np

import fcbench

FLAG_GOOD = 245


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--minutes', type=float, default=10.0)
    ap.add_argument('--label', default='soak', help='tag for the saved JSON')
    ap.add_argument('--out', default=None, help='output JSON path')
    a = ap.parse_args()

    m = fcbench.connect()
    fcbench.request(m, (106, 132, 105))   # flow, rangefinder, IMU (for temperature)

    secs = a.minutes * 60
    flow, rng, temp = [], [], []
    t0 = time.time()
    mark = 60.0
    print(f"# soak '{a.label}': {a.minutes:.0f} min. Nothing should move.")
    print(f"# {'min':>4} {'n':>6} {'q245%':>7} {'q122%':>7} {'q0%':>6} {'h m':>7} {'degC':>6}")

    while time.time() - t0 < secs:
        msg = m.recv_match(type=['OPTICAL_FLOW_RAD', 'DISTANCE_SENSOR', 'HIGHRES_IMU'],
                           blocking=True, timeout=2)
        if msg is None:
            continue
        el = time.time() - t0
        kind = msg.get_type()
        if kind == 'OPTICAL_FLOW_RAD':
            flow.append((el, msg.quality))
        elif kind == 'DISTANCE_SENSOR':
            rng.append((el, msg.current_distance / 100.0))
        else:
            temp.append((el, getattr(msg, 'temperature', float('nan'))))

        if el >= mark:
            win = np.array([q for t, q in flow if t > mark - 60])
            h = [d for t, d in rng if t > mark - 60]
            c = [v for t, v in temp if t > mark - 60]
            if win.size:
                print(f"  {mark/60:4.0f} {win.size:6d} "
                      f"{100*(win == FLAG_GOOD).mean():6.1f}% "
                      f"{100*((win > 0) & (win < FLAG_GOOD)).mean():6.1f}% "
                      f"{100*(win == 0).mean():5.1f}% "
                      f"{np.median(h) if h else float('nan'):7.2f} "
                      f"{np.mean(c) if c else float('nan'):6.1f}", flush=True)
            mark += 60.0

    if not flow:
        raise SystemExit("no OPTICAL_FLOW_RAD received — is the module powered?")

    t = np.array([x[0] for x in flow])
    q = np.array([x[1] for x in flow])
    half = (q > 0) & (q < FLAG_GOOD)
    true_blind = 100 * ((q == 0).mean() + 0.5 * half.mean())

    out = a.out or f"soak_{a.label}.json"
    json.dump({'label': a.label, 't': t.tolist(), 'q': q.tolist(),
               'rng': [list(x) for x in rng], 'temp': [list(x) for x in temp]},
              open(out, 'w'))

    print(f"\n===== SOAK RESULT: {a.label} =====")
    print(f"  duration {t[-1]/60:.1f} min   samples {q.size}   rate {q.size/t[-1]:.1f} Hz")
    print(f"  q=245 both raw good  : {100*(q == FLAG_GOOD).mean():6.2f}%")
    print(f"  q=122 one raw blind  : {100*half.mean():6.2f}%")
    print(f"  q=0   both blind     : {100*(q == 0).mean():6.2f}%")
    print(f"  TRUE sensor-level blind: {true_blind:.2f}%")

    runs, start = [], None
    for i, bad in enumerate(q < FLAG_GOOD):
        if bad and start is None:
            start = t[i]
        elif not bad and start is not None:
            runs.append((start, t[i] - start))
            start = None
    if start is not None:
        runs.append((start, t[-1] - start))
    if runs:
        d = np.array([r[1] for r in runs])
        print(f"\n  {len(runs)} DROPOUT EVENTS (first 15):")
        for st, du in runs[:15]:
            print(f"    t={st:8.1f}s   {du*1000:8.0f} ms")
        print(f"    longest {d.max()*1000:.0f} ms   total {d.sum():.2f}s of {t[-1]:.0f}s")

    if rng:
        r = np.array([x[1] for x in rng])
        print(f"\n  height {r.min():.2f}-{r.max():.2f} m "
              f"(spread {100*(r.max()-r.min()):.1f} cm) — confirms nothing moved")
    if temp:
        c = np.array([x[1] for x in temp])
        print(f"  temperature {c[0]:.1f} -> {c[-1]:.1f} degC across the soak")

    print(f"\n  VERDICT: {'*** INTERMITTENT SENSOR *** dropouts with nothing moving' if true_blind > 0.05 else 'CLEAN — the sensor is not the fault'}")


if __name__ == '__main__':
    main()
