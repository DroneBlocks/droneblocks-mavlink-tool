#!/usr/bin/env python3
"""Flip/vibration analyzer for DEXI ulogs.

Usage:  ./venv/bin/python analyze_flip_vibration.py logs/log_240.ulg

Reports, from one .ulg:
  - ESC/motor health (per-motor RPM range, error/failure counts, RPM dropouts) -> rules ESC desync in/out
  - onboard 8kHz gyro FFT dominant peak per axis (the real vibration frequency)
  - gyro spectrum from sensor_combined (needs SDLOG_PROFILE bit "High rate")
  - gyro noise vs RPM (does vibration grow with throttle? = rotating imbalance signature)
  - motor fundamental Hz reference (compare to detected peak: RPM-locked vs fixed resonance)

Prereq for good data: SDLOG_PROFILE must include High-rate (bit 4). We set 273 on the DEXI-5.
"""
import sys, numpy as np
from pyulog import ULog

path = sys.argv[1] if len(sys.argv) > 1 else "logs/log_240.ulg"
u = ULog(path)
P = u.initial_parameters
print(f"===== {path} =====")
print(f"airframe={P.get('SYS_AUTOSTART')} PX4 airmode={P.get('MC_AIRMODE')} "
      f"acroR={P.get('MC_ACRO_R_MAX')} rateMax={P.get('MC_ROLLRATE_MAX')} "
      f"DNF_EN={P.get('IMU_GYRO_DNF_EN')} gyroCut={P.get('IMU_GYRO_CUTOFF')} "
      f"batScale={P.get('MC_BAT_SCALE_EN')} sdlog={P.get('SDLOG_PROFILE')}")

def ds(n):
    try: return u.get_dataset(n)
    except Exception: return None

# ---- ESC health ----
esc = ds("esc_status")
rpms = None
if esc:
    et = esc.data["timestamp"]/1e6
    print("\n=== ESC / motor health ===")
    rl = []
    for i in range(4):
        k = f"esc[{i}].esc_rpm"
        if k not in esc.data: continue
        rpm = esc.data[k].astype(float); rl.append(rpm)
        err = esc.data.get(f"esc[{i}].esc_errorcount")
        fail = esc.data.get(f"esc[{i}].failures")
        drops = int(np.sum((rpm[1:]<100)&(rpm[:-1]>1500)))
        e = int(err[-1]-err[0]) if err is not None else -1
        fm = int(np.max(fail)) if fail is not None else -1
        print(f"  ESC{i}: rpm {np.min(rpm):.0f}-{np.max(rpm):.0f} (mean {np.mean(rpm):.0f})  err+={e} fail={fm} dropouts={drops}")
    if rl: rpms = np.array(rl)

# ---- onboard FFT ----
fft = ds("sensor_gyro_fft")
if fft:
    print("\n=== onboard gyro FFT (dominant peak, SNR>0) ===")
    print(f"  resolution={np.median(fft.data['resolution_hz']):.1f}Hz sample_rate={np.median(fft.data['sensor_sample_rate_hz']):.0f}Hz")
    for ax in "xyz":
        p = fft.data[f"peak_frequencies_{ax}[0]"]; s = fft.data[f"peak_snr_{ax}[0]"]
        v = p[(p>0)&(s>0)]
        if v.size: print(f"  gyro_{ax}: median={np.median(v):.0f}Hz range {np.min(v):.0f}-{np.max(v):.0f}Hz (n={v.size})")

# ---- gyro spectrum + noise vs RPM ----
sc = ds("sensor_combined")
if sc and rpms is not None:
    t = sc.data["timestamp"]/1e6; dt = np.median(np.diff(t)); fs = 1/dt
    g = [sc.data[f"gyro_rad[{i}]"] for i in range(3)]
    mean_rpm = np.mean(rpms, axis=0)
    rpm_sc = np.interp(t, et, mean_rpm)
    print(f"\n=== gyro spectrum (sensor_combined fs={fs:.0f}Hz, Nyquist {fs/2:.0f}Hz) ===")
    def top(sig, mask, lbl):
        s = sig[mask]-np.mean(sig[mask])
        if s.size < 256: return
        m = np.abs(np.fft.rfft(s*np.hanning(s.size))); fr = np.fft.rfftfreq(s.size, dt)
        b = fr>=15; idx = np.argsort(m[b])[::-1][:4]
        print(f"  {lbl}: rms={np.std(sig[mask]):.3f} peaks: "+", ".join(f"{fr[b][i]:.0f}Hz" for i in idx))
    hi = rpm_sc >= np.percentile(rpm_sc,80); lo = rpm_sc <= np.percentile(rpm_sc,50)
    print("-- high throttle --");  [top(g[i],hi,f"gyro_{'xyz'[i]}") for i in range(3)]
    print("-- low throttle --");   [top(g[i],lo,f"gyro_{'xyz'[i]}") for i in range(3)]
    hr = np.mean(mean_rpm[mean_rpm>=np.percentile(mean_rpm,80)])
    print(f"\nref: high-throttle mean RPM={hr:.0f} -> fundamental {hr/60:.0f}Hz (imbalance), 2x prop {hr/30:.0f}Hz")
    print("\n=== gyro noise vs RPM (rising with throttle = rotating imbalance) ===")
    order = np.argsort(rpm_sc)
    for bidx in np.array_split(order, 6):
        noise = sum(np.std(g[i][bidx]) for i in range(3))
        print(f"  RPM~{np.mean(rpm_sc[bidx]):5.0f}: noise={noise:.3f} rad/s")
