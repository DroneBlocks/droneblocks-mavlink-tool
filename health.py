#!/usr/bin/env python3
"""
CLI wrapper around telemetry.stream_health — prints a one-line
indoor-hover health summary at ~2 Hz and logs to CSV.

Usage:
    python health.py [host] [port]

Default: 192.168.68.61 : 14550
"""
import csv
import math
import signal
import sys
import time

from telemetry import HealthSample, stream_health


def tf(b: bool) -> str:
    return "T" if b else "F"


def main() -> None:
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.68.61"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 14550
    csv_path = f"/tmp/health_{int(time.time())}.csv"

    csv_file = open(csv_path, "w", newline="")
    writer: csv.DictWriter | None = None

    t0: float | None = None
    origin: list[float | None] = [None, None, None]  # pos_x, pos_y, pos_z when first valid

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

    def on_connect() -> None:
        print("Connected. Requesting streams. Fly when you see lines ticking.", flush=True)

    def on_sample(s: HealthSample) -> None:
        nonlocal t0, writer
        if t0 is None:
            t0 = s.t
            writer = csv.DictWriter(csv_file, fieldnames=list(s.to_dict().keys()))
            writer.writeheader()
        writer.writerow(s.to_dict())
        csv_file.flush()

        # Capture drift origin once we have a valid position
        if origin[0] is None and not math.isnan(s.pos_x):
            origin[0], origin[1], origin[2] = s.pos_x, s.pos_y, s.pos_z

        if origin[0] is not None and not math.isnan(s.pos_x):
            dx = s.pos_x - origin[0]
            dy = s.pos_y - origin[1]
            drift = math.hypot(dx, dy)
        else:
            drift = float("nan")

        q_str   = f"{s.flow_q:3d}" if s.flow_q is not None else " --"
        rng_str = f"{s.rng_m:4.2f}m" if s.rng_m is not None else "  -- "

        # Color-ish hint via '!' for out-of-spec values
        warn_cpm  = "!" if s.const_pos_mode   else " "
        warn_vib  = "!" if max(s.vib_x, s.vib_y, s.vib_z) > 3.0 else " "

        print(
            f"t={s.t - t0:5.1f}s  "
            f"CPM={tf(s.const_pos_mode)}{warn_cpm} "
            f"PRR={tf(s.pred_pos_horiz_rel)} "
            f"PHR={tf(s.pos_horiz_rel)}   "
            f"eph={s.eph:4.2f}  drift={drift:5.2f}m  "
            f"flow_q={q_str}  rng={rng_str}  "
            f"vib={s.vib_x:4.1f},{s.vib_y:4.1f},{s.vib_z:4.1f}{warn_vib}",
            flush=True,
        )

    print(f"Logging CSV -> {csv_path}")
    print(f"Connecting udpout:{host}:{port} ...", flush=True)
    try:
        stream_health(
            host=host, port=port,
            on_sample=on_sample,
            stop_flag=lambda: stop["flag"],
            on_connect=on_connect,
        )
    finally:
        csv_file.close()
        print(f"\nSaved {csv_path}")


if __name__ == "__main__":
    main()
