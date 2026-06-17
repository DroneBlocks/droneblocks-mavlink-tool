#!/usr/bin/env python3
"""
Diff two param dumps (from dump_params.py). Categorizes differences so you can
tell "this drone is misconfigured" from "this is just normal per-board calibration."

Usage:
    python diff_params.py drone1_baseline.params drone2_before.params
"""
import sys

# Params that are SUPPOSED to differ per physical board / over time — not a problem.
EXPECTED_DIFFER_PREFIX = ("CAL_",)
EXPECTED_DIFFER_EXACT = {
    "MPC_THR_HOVER",        # learned per-drone (mass/thrust)
    "COM_FLIGHT_UUID",
}
EXPECTED_DIFFER_CONTAINS = ("_FLIGHT_T", "FLIGHT_TIME", "_PRIME")

# Params we MOST care about matching on drone #2.
CRITICAL = {
    "SYS_AUTOSTART", "SENS_BOARD_ROT", "MC_AIRMODE",
    "MC_ROLLRATE_P","MC_PITCHRATE_P","MC_ROLLRATE_I","MC_PITCHRATE_I",
    "MC_ROLLRATE_D","MC_PITCHRATE_D","MC_ROLLRATE_MAX","MC_PITCHRATE_MAX",
    "MC_YAWRATE_P","MC_YAWRATE_I","MC_YAWRATE_MAX","MC_ROLL_P","MC_PITCH_P","MC_YAW_P",
    "MPC_MAN_TILT_MAX","BAT1_N_CELLS","EKF2_OF_CTRL","EKF2_HGT_REF",
    "MAV_0_CONFIG","UXRCE_DDS_CFG","MAV_2_CONFIG",
}

def load(path):
    hdr={}; params={}
    for line in open(path):
        if line.startswith('#'):
            if ':' in line:
                k,_,v = line[1:].partition(':'); hdr[k.strip()]=v.strip()
            continue
        parts=line.rstrip('\n').split('\t')
        if len(parts)>=4:
            params[parts[2]] = parts[3]
    return hdr, params

def is_expected(name):
    if name.startswith(EXPECTED_DIFFER_PREFIX): return True
    if name in EXPECTED_DIFFER_EXACT: return True
    if any(s in name for s in EXPECTED_DIFFER_CONTAINS): return True
    return False

def numeq(a,b):
    try: return abs(float(a)-float(b)) < 1e-5
    except Exception: return a==b

def main():
    if len(sys.argv)<3: sys.exit("usage: diff_params.py <ref.params> <new.params>")
    h1,p1 = load(sys.argv[1]); h2,p2 = load(sys.argv[2])

    print("=== VERSION ===")
    for k in ("flight_sw_version","board_version","flight_custom"):
        a,b = h1.get(k),h2.get(k)
        flag = "  <-- DIFFERENT" if a!=b else ""
        print(f"  {k:18s} ref={a}  new={b}{flag}")

    only1 = sorted(set(p1)-set(p2))
    only2 = sorted(set(p2)-set(p1))
    diff  = sorted(k for k in set(p1)&set(p2) if not numeq(p1[k],p2[k]))

    crit = [k for k in diff if k in CRITICAL]
    cfg  = [k for k in diff if k not in CRITICAL and not is_expected(k)]
    cal  = [k for k in diff if k not in CRITICAL and is_expected(k)]

    print(f"\n=== CRITICAL mismatches ({len(crit)}) — MUST review before flight ===")
    for k in crit: print(f"  {k:18s} ref={p1[k]:>10}  new={p2[k]:>10}")
    if not crit: print("  (none — critical config matches the golden drone)")

    print(f"\n=== Other config differences ({len(cfg)}) — review ===")
    for k in cfg: print(f"  {k:22s} ref={p1[k]:>10}  new={p2[k]:>10}")
    if not cfg: print("  (none)")

    print(f"\n=== Expected per-board diffs ({len(cal)}: calibration / learned) — normal ===")
    if cal: print("  " + ", ".join(cal[:25]) + (" ..." if len(cal)>25 else ""))

    if only1 or only2:
        print(f"\n=== Param set differs (fw mismatch?): only-in-ref={len(only1)} only-in-new={len(only2)} ===")
        if only2[:15]: print("  new-only:", ", ".join(only2[:15]))

if __name__ == "__main__":
    main()
