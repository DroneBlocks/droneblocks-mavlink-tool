#!/usr/bin/env python3
"""Score an accel calibration on the bench.

Gravity is the only calibration reference we have indoors. If the accel cal is
correct, the magnitude of the acceleration vector reads 9.807 m/s^2 in EVERY
orientation, and each axis reads +/-9.807 when that axis points down.

Params can only tell you the 6-position routine RAN (scale factors move off
1.0000). They cannot tell you it was CORRECT: if the surface you held the board
against was not square, both IMUs get the same wrong scale and the numbers look
beautifully self-consistent while being 2% off.

Usage:
    python calcheck.py                 # interactive 6-face check
    python calcheck.py --level-only    # just the level face, 20 s
"""
import argparse, math, sys, time
import fcbench

G = 9.80665
FACES = [
    ("LEVEL, props up",      'z', -1),
    ("UPSIDE DOWN",          'z', +1),
    ("NOSE DOWN",            'x', +1),
    ("NOSE UP",              'x', -1),
    ("LEFT SIDE DOWN",       'y', +1),
    ("RIGHT SIDE DOWN",      'y', -1),
]


def sample(m, secs):
    """Collect HIGHRES_IMU for `secs`, return list of (x, y, z) in m/s^2."""
    out, t0 = [], time.time()
    while time.time() - t0 < secs:
        msg = m.recv_match(type='HIGHRES_IMU', blocking=True, timeout=2)
        if msg is None:
            continue
        out.append((msg.xacc, msg.yacc, msg.zacc))
    return out


def stats(s):
    n = len(s)
    mean = [sum(v[i] for v in s) / n for i in range(3)]
    mag = [math.sqrt(sum(c * c for c in v)) for v in s]
    mmean = sum(mag) / n
    msd = math.sqrt(sum((x - mmean) ** 2 for x in mag) / n)
    return mean, mmean, msd, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port'); ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--secs', type=float, default=8.0, help='seconds per face')
    ap.add_argument('--level-only', action='store_true')
    a = ap.parse_args()

    m = fcbench.connect(a.baud)
    fcbench.request(m, [105], interval_us=10000)   # HIGHRES_IMU at 100 Hz

    faces = FACES[:1] if a.level_only else FACES
    results = []
    for name, axis, sign in faces:
        input(f"\n>>> Hold the aircraft {name}, steady, then press Enter. ")
        print(f"    sampling {a.secs:.0f} s, do not move it...")
        s = sample(m, a.secs)
        if len(s) < 20:
            print("    NOT ENOUGH SAMPLES, is HIGHRES_IMU streaming?")
            continue
        mean, mmean, msd, n = stats(s)
        idx = 'xyz'.index(axis)
        want = sign * G
        axis_err = mean[idx] - want
        mag_err = mmean - G
        results.append((name, mmean, mag_err, axis_err, msd, n))
        print(f"    |a| = {mmean:6.3f}  err {mag_err:+.3f} m/s^2 ({mag_err/G*100:+.2f}%)"
              f"   {axis} = {mean[idx]:+6.3f} (want {want:+.2f}, err {axis_err:+.3f})"
              f"   sd {msd:.3f}  n={n}")

    if not results:
        sys.exit("no faces measured")

    print("\n===== CALIBRATION SCORE =====")
    print(f"{'face':<20} {'|a|':>7} {'err %':>8} {'axis err':>9}")
    worst = 0.0
    for name, mmean, mag_err, axis_err, msd, n in results:
        print(f"{name:<20} {mmean:7.3f} {mag_err/G*100:+8.2f} {axis_err:+9.3f}")
        worst = max(worst, abs(mag_err / G * 100))

    print()
    if worst < 0.5:
        print(f"  VERDICT: GOOD. Worst face is {worst:.2f}% off gravity.")
    elif worst < 1.5:
        print(f"  VERDICT: ACCEPTABLE. Worst face {worst:.2f}%. Usable, but a")
        print("           re-run on a squarer surface would tighten it.")
    else:
        print(f"  VERDICT: POOR. Worst face {worst:.2f}% off gravity.")
        print("           The 6-position ran but the orientations were not square.")
        print("           This error becomes learned accel bias in flight.")
    print("\n  Note: taken at the current board temperature. The IMUs run 44-49 C")
    print("  in flight, and none of our aircraft use thermal calibration (TC_*),")
    print("  so a cold result is a floor, not the in-flight truth.")


if __name__ == '__main__':
    main()
