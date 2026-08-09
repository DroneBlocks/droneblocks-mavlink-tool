#!/usr/bin/env python3
"""Qualify a magnetometer install from a PX4 flight log.

The question this answers: does the compass reading move with motor current?

A static calibration removes a CONSTANT hard-iron offset. It cannot remove one
that grows with the current flowing through nearby arms and ESCs. That failure
mode is invisible on a bench (motors are off) and shows up in flight as heading
wander under load, or Position hold slowly rotating its idea of north. The test
is to correlate calibrated field magnitude against battery current.

    source venv/bin/activate
    python analyze_mag_current.py logs/log_133.ulg \
        --params dexi-10-BACKUP-2026-08-09-postflight.params \
        --criteria validation/criteria.json \
        --out metrics.json

IMPORTANT: sensor_mag in the log is RAW. The calibration (offsets, scales and
off-diagonal soft-iron terms) lives in the CAL_MAG* params, which is why a
.params capture from the same aircraft is required rather than optional. Reading
raw magnitudes understates the field and overstates the noise; on the first DEXI-10
flight raw read 0.3498 G std 0.0344 where calibrated read 0.5207 G std 0.0051.
"""
import argparse
import os
import json
import sys

import numpy as np
from pyulog import ULog

# PX4 magnetometer device types, from src/drivers/drv_sensor.h
MAG_DEVTYPE = {
    0x01: 'HMC5883', 0x04: 'AK8963', 0x05: 'LIS3MDL', 0x06: 'IST8310',
    0x07: 'RM3100', 0x08: 'QMC5883L', 0x09: 'AK09916', 0x0A: 'VCM1193L',
    0x0B: 'IST8308', 0x0D: 'MMC5983MA', 0x0E: 'IIS2MDC', 0x0F: 'QMC5883P',
    0x43: 'BMM150', 0x62: 'LSM303AGR', 0x88: 'UAVCAN', 0xE5: 'BMM350',
}
BUS_TYPE = {0: 'UNKNOWN', 1: 'I2C', 2: 'SPI', 3: 'UAVCAN', 4: 'SIM', 5: 'SERIAL', 6: 'MAVLINK'}


def decode_device_id(devid):
    """PX4 packs bus_type:3 | bus:5 | address:8 | devtype:8 into the device id."""
    d = int(devid) & 0xFFFFFFFF
    devtype = (d >> 16) & 0xFF
    return {
        'raw': d,
        'bus_type': BUS_TYPE.get(d & 7, d & 7),
        'bus': (d >> 3) & 0x1F,
        'address': f'0x{(d >> 8) & 0xFF:02X}',
        'devtype': f'0x{devtype:02X}',
        'name': MAG_DEVTYPE.get(devtype, f'devtype 0x{devtype:02X}'),
    }


def read_params(path):
    """Parse a QGC .params file into {name: float}."""
    out = {}
    with open(path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.rstrip('\n').split('\t')
            if len(parts) >= 4:
                out[parts[2]] = float(parts[3])
    return out


def calibrated_magnitude(x, y, z, p, slot):
    """Apply the PX4 mag calibration: |Scale * (raw - offset)|.

    Scale is symmetric with the off-diagonal soft-iron terms on the shoulders,
    matching how PX4 stores XODIAG/YODIAG/ZODIAG.
    """
    off = np.array([p[f'CAL_MAG{slot}_{a}OFF'] for a in 'XYZ'])
    sc = [p[f'CAL_MAG{slot}_{a}SCALE'] for a in 'XYZ']
    od = [p[f'CAL_MAG{slot}_{a}ODIAG'] for a in 'XYZ']
    S = np.array([[sc[0], od[0], od[1]],
                  [od[0], sc[1], od[2]],
                  [od[1], od[2], sc[2]]])
    v = np.vstack([x - off[0], y - off[1], z - off[2]])
    return np.linalg.norm(S @ v, axis=0)


def slot_for_device(params, devid):
    """Find which CAL_MAGn slot holds this device id."""
    for slot in range(4):
        if int(params.get(f'CAL_MAG{slot}_ID', 0)) == int(devid):
            return slot
    return None


def analyze(ulog_path, params):
    topics = ['sensor_mag', 'battery_status', 'estimator_aid_src_mag',
              'vehicle_gps_position', 'actuator_armed']
    u = ULog(ulog_path, topics)
    D = {(d.name, d.multi_id): d.data for d in u.data_list}
    t0 = u.start_timestamp
    secs = lambda ts: (np.asarray(ts, float) - t0) / 1e6

    if ('actuator_armed', 0) not in D:
        sys.exit('log has no actuator_armed; cannot find the armed window')
    arm = D[('actuator_armed', 0)]
    at, av = secs(arm['timestamp']), np.asarray(arm['armed'], bool)
    if not av.any():
        sys.exit('never armed in this log; nothing to qualify')
    i = np.where(av)[0]
    t_arm, t_dis = float(at[i[0]]), float(at[i[-1]])

    bat = D[('battery_status', 0)]
    bt, cur = secs(bat['timestamp']), np.asarray(bat['current_a'], float)
    fw = (bt >= t_arm) & (bt <= t_dis)

    m = {
        'log': ulog_path,
        'armed_window_s': [round(t_arm, 2), round(t_dis, 2)],
        'armed_duration_s': round(t_dis - t_arm, 1),
        'current_a': {
            'mean': round(float(cur[fw].mean()), 2),
            'min': round(float(cur[fw].min()), 2),
            'max': round(float(cur[fw].max()), 2),
            'span': round(float(cur[fw].max() - cur[fw].min()), 2),
        },
        'magnetometers': [],
    }

    for key in sorted(k for k in D if k[0] == 'sensor_mag'):
        d = D[key]
        mt = secs(d['timestamp'])
        devid = int(np.asarray(d['device_id'])[0])
        slot = slot_for_device(params, devid)
        info = decode_device_id(devid)
        entry = {'instance': key[1], 'device': info, 'cal_slot': slot}
        if slot is None:
            entry['skipped'] = 'device id not present in any CAL_MAGn_ID; cannot calibrate'
            m['magnetometers'].append(entry)
            continue
        prio = int(params.get(f'CAL_MAG{slot}_PRIO', 0))
        entry['priority'] = prio
        entry['enabled'] = prio > 0
        entry['rotation'] = int(params.get(f'CAL_MAG{slot}_ROT', -1))

        mag = calibrated_magnitude(np.asarray(d['x'], float), np.asarray(d['y'], float),
                                   np.asarray(d['z'], float), params, slot)
        w = (mt >= t_arm) & (mt <= t_dis)
        if w.sum() < 10:
            entry['skipped'] = f'only {int(w.sum())} samples while armed'
            m['magnetometers'].append(entry)
            continue
        v, ci = mag[w], np.interp(mt[w], bt, cur)
        A = np.vstack([ci, np.ones_like(ci)]).T
        slope, _ = np.linalg.lstsq(A, v, rcond=None)[0]
        r = float(np.corrcoef(ci, v)[0, 1]) if ci.std() > 0 else 0.0
        span = float(ci.max() - ci.min())
        entry.update({
            'samples': int(w.sum()),
            'sample_rate_hz': round(float(w.sum()) / max(t_dis - t_arm, 1e-9), 2),
            'field_gauss': {
                'mean': round(float(v.mean()), 4), 'std': round(float(v.std()), 4),
                'min': round(float(v.min()), 4), 'max': round(float(v.max()), 4),
                'std_pct': round(float(v.std() / v.mean() * 100), 2),
            },
            'vs_current': {
                'slope_mG_per_A': round(float(slope * 1000), 3),
                'r': round(r, 4),
                'abs_r': round(abs(r), 4),
                'swing_mG': round(float(abs(slope) * span * 1000), 2),
                'swing_pct_of_field': round(float(abs(slope) * span / v.mean() * 100), 2),
            },
        })
        m['magnetometers'].append(entry)

    if ('estimator_aid_src_mag', 0) in D:
        e = D[('estimator_aid_src_mag', 0)]
        et = secs(e['timestamp'])
        w = (et >= t_arm) & (et <= t_dis)
        axes = []
        for ax in range(3):
            tr = np.asarray(e[f'test_ratio[{ax}]'], float)[w]
            axes.append({
                'axis': ax, 'mean': round(float(tr.mean()), 4),
                'p95': round(float(np.percentile(tr, 95)), 4),
                'max': round(float(tr.max()), 4),
                'frac_over_1_pct': round(float((tr > 1).mean() * 100), 3),
            })
        m['ekf_mag'] = {
            'samples': int(w.sum()),
            'note': 'PX4 rejects a measurement when test_ratio exceeds 1.0',
            'test_ratio_p95_worst_axis': round(max(a['p95'] for a in axes), 4),
            'innovation_rejected_pct': round(
                float(np.asarray(e['innovation_rejected'], bool)[w].mean() * 100), 3),
            'axes': axes,
        }

    if ('vehicle_gps_position', 0) in D:
        g = D[('vehicle_gps_position', 0)]
        gt = secs(g['timestamp'])
        w = (gt >= t_arm) & (gt <= t_dis)
        gps = {}
        for field, label in [('satellites_used', 'sats'), ('eph', 'eph_m'),
                             ('hdop', 'hdop'), ('fix_type', 'fix_type')]:
            if field in g:
                v = np.asarray(g[field], float)[w]
                gps[label] = {'mean': round(float(v.mean()), 2),
                              'min': round(float(v.min()), 2),
                              'max': round(float(v.max()), 2)}
        m['gps'] = gps
    return m


def primary_mag(metrics):
    for e in metrics['magnetometers']:
        if e.get('enabled') and 'field_gauss' in e:
            return e
    return None


def evaluate(metrics, criteria):
    """Check the metrics against pre-registered thresholds. Returns (rows, passed)."""
    p = primary_mag(metrics)
    if p is None:
        return [('primary magnetometer present', 'none enabled', 'required', False)], False

    def get(path, default=None):
        cur = metrics
        for k in path.split('.'):
            if isinstance(cur, dict) and k in cur:
                cur = cur[k]
            else:
                return default
        return cur

    checks = [
        ('mag |B| vs current, |r|', p['vs_current']['abs_r'],
         criteria['mag_current_abs_r_max'], 'max'),
        ('mag |B| swing over current range (%)', p['vs_current']['swing_pct_of_field'],
         criteria['mag_current_swing_pct_max'], 'max'),
        ('mag |B| mean (gauss)', p['field_gauss']['mean'],
         (criteria['mag_field_gauss_min'], criteria['mag_field_gauss_max']), 'range'),
        ('mag |B| std (% of mean)', p['field_gauss']['std_pct'],
         criteria['mag_field_std_pct_max'], 'max'),
        ('EKF mag test_ratio p95', get('ekf_mag.test_ratio_p95_worst_axis'),
         criteria['ekf_test_ratio_p95_max'], 'max'),
        ('EKF mag rejected (%)', get('ekf_mag.innovation_rejected_pct'),
         criteria['ekf_rejected_pct_max'], 'max'),
        ('GPS eph mean (m)', get('gps.eph_m.mean'), criteria['gps_eph_mean_m_max'], 'max'),
        ('GPS sats mean', get('gps.sats.mean'), criteria['gps_sats_mean_min'], 'min'),
        ('current range swept (A)', metrics['current_a']['span'],
         criteria['current_span_a_min'], 'min'),
    ]
    rows, passed = [], True
    for name, value, limit, kind in checks:
        if value is None:
            rows.append((name, 'n/a', 'not in log', None))
            continue
        if kind == 'max':
            ok, lim = value <= limit, f'<= {limit}'
        elif kind == 'min':
            ok, lim = value >= limit, f'>= {limit}'
        else:
            ok, lim = limit[0] <= value <= limit[1], f'{limit[0]} to {limit[1]}'
        rows.append((name, value, lim, ok))
        passed = passed and ok
    return rows, passed


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ulog')
    ap.add_argument('--params', required=True,
                    help='QGC .params capture from the SAME aircraft (for CAL_MAG*)')
    ap.add_argument('--criteria', help='JSON of pre-registered thresholds')
    ap.add_argument('--out', help='write metrics JSON here')
    a = ap.parse_args()

    params = read_params(a.params)
    metrics = analyze(a.ulog, params)

    c = metrics['current_a']
    print(f"\nlog {a.ulog}")
    print(f"armed {metrics['armed_duration_s']} s, "
          f"current mean {c['mean']} A, range {c['min']} to {c['max']} A (span {c['span']} A)\n")
    print(f"{'magnetometer':<34}{'role':<10}{'|B| G':>9}{'std':>8}{'mG/A':>9}{'r':>8}{'swing':>8}")
    for e in metrics['magnetometers']:
        if 'field_gauss' not in e:
            print(f"  {e['device']['name']:<32}{e.get('skipped', '')}")
            continue
        role = 'PRIMARY' if e['enabled'] else 'disabled'
        loc = f"{e['device']['name']} ({e['device']['bus_type']}{e['device']['bus']} {e['device']['address']})"
        print(f"{loc:<34}{role:<10}{e['field_gauss']['mean']:>9.4f}{e['field_gauss']['std']:>8.4f}"
              f"{e['vs_current']['slope_mG_per_A']:>+9.2f}{e['vs_current']['r']:>+8.3f}"
              f"{e['vs_current']['swing_pct_of_field']:>7.1f}%")

    if 'ekf_mag' in metrics:
        k = metrics['ekf_mag']
        print(f"\nEKF mag: test_ratio p95 {k['test_ratio_p95_worst_axis']} (gate 1.0), "
              f"rejected {k['innovation_rejected_pct']}%")
    if metrics.get('gps'):
        g = metrics['gps']
        print("GPS: " + ", ".join(f"{k} mean {v['mean']}" for k, v in g.items()))

    ok = None
    if a.criteria:
        with open(a.criteria) as f:
            crit = json.load(f)
        rows, ok = evaluate(metrics, crit.get('thresholds', crit))
        print(f"\n{'check':<40}{'value':>12}{'limit':>16}   verdict")
        for name, value, lim, good in rows:
            verdict = 'SKIP' if good is None else ('PASS' if good else 'FAIL')
            print(f"{name:<40}{str(value):>12}{lim:>16}   {verdict}")
        print(f"\nOVERALL: {'PASS' if ok else 'FAIL'}")
        # basename only: absolute paths are machine-specific and these files get
        # committed. The run's manifest.json carries the authoritative reference.
        metrics['criteria_file'] = os.path.basename(a.criteria)
        metrics['verdict'] = 'PASS' if ok else 'FAIL'

    if a.out:
        with open(a.out, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"\nwrote {a.out}")
    return 0 if ok is not False else 1


if __name__ == '__main__':
    sys.exit(main())
