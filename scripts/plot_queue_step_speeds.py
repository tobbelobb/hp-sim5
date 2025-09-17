#!/usr/bin/env python3
"""
Parse Klipper MCU `queue_step` command sequences and export requested motor speeds per step.

Definition of "requested speed":
    speed (steps/s) = CLOCK_HZ / interval_ticks
Signed by the most recent `set_next_step_dir` for that stepper:
    dir=1 -> +speed; dir=0 -> -speed

Time base:
    Without explicit 'reset_step_clock clock=...' records for each stepper,
    queue_step timing is relative to the last step on that stepper (local timeline).
    This script reproduces the local-per-stepper timeline used in the CSV you saw.

Usage:
    python queue_step_speeds.py --src Hangprinter_logo2.shorter.txt \
                                --out-csv queue_step_speeds_per_step.csv \
                                --clock-hz 25000000 \
                                [--plot --out-dir plots]

Requires:
    Python 3.8+
    pandas, numpy, matplotlib
"""

import re
import sys
import argparse
from pathlib import Path
from typing import Dict, List
import pandas as pd
import numpy as np

# Optional plotting
try:
    import matplotlib.pyplot as plt
    HAVE_PLOT = True
except Exception:
    HAVE_PLOT = False


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Extract requested speeds from Klipper queue_step commands.")
    p.add_argument("--src", type=Path, required=True, help="Path to the MCU command log/text containing queue_step lines.")
    p.add_argument("--out-csv", type=Path, required=True, help="Where to write the per-step CSV.")
    p.add_argument("--clock-hz", type=float, default=25_000_000, help="MCU clock frequency in Hz (default: 25e6).")
    p.add_argument("--plot", action="store_true", help="If set, save one plot per stepper (requires matplotlib).")
    p.add_argument("--out-dir", type=Path, default=None, help="Directory to put plots into (default: alongside CSV).")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    text = args.src.read_text(encoding="utf-8", errors="ignore")

    # Regex patterns that match the lines emitted by Klipper MCU commands
    re_dir = re.compile(r"set_next_step_dir\s+oid=(\d+)\s+dir=(\d+)")
    re_qs  = re.compile(r"queue_step\s+oid=(\d+)\s+interval=(\d+)\s+count=(\d+)\s+add=(-?\d+)")

    dirs: Dict[int, int] = {}       # current direction sign per oid: +1 or -1
    last_tick: Dict[int, int] = {}  # last tick timestamp per oid (local timeline)
    records: List[dict] = []

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue

        m = re_dir.match(line)
        if m:
            oid = int(m.group(1))
            d   = int(m.group(2))
            dirs[oid] = 1 if d == 1 else -1
            last_tick.setdefault(oid, 0)
            continue

        m = re_qs.match(line)
        if m:
            oid = int(m.group(1))
            interval = int(m.group(2))
            count = int(m.group(3))
            add = int(m.group(4))

            sign = dirs.get(oid, 1)
            last = last_tick.get(oid, 0)

            for i in range(count):
                interval_i = interval + i * add
                if interval_i < 1:
                    interval_i = 1  # safety clamp
                tick = last + interval_i
                t = tick / args.clock_hz
                sps = args.clock_hz / interval_i
                records.append({
                    "oid": oid,
                    "step_index_in_qs": i,
                    "interval_ticks": interval_i,
                    "signed_speed_sps": sign * sps,
                    "speed_sps": sps,
                    "tick": tick,
                    "time_s": t,
                })
                last = tick

            last_tick[oid] = last

    if not records:
        print("No queue_step commands parsed. Check --src input.", file=sys.stderr)
        return 2

    df = pd.DataFrame.from_records(records)
    df.sort_values(["oid", "tick"], inplace=True)
    df["local_step_num"] = df.groupby("oid").cumcount()
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out_csv, index=False)
    print(f"Wrote per-step CSV with {len(df)} rows to: {args.out_csv}")

    # Summary to stdout
    def summarize(group: pd.DataFrame) -> dict:
        speeds = group["speed_sps"].to_numpy()
        signed = group["signed_speed_sps"].to_numpy()
        dur_s  = float(group["time_s"].max()) if len(group) else 0.0
        return {
            "steps": len(group),
            "duration_s": dur_s,
            "min_speed_sps": float(speeds.min()),
            "median_speed_sps": float(np.median(speeds)),
            "max_speed_sps": float(speeds.max()),
            "min_signed_sps": float(signed.min()),
            "max_signed_sps": float(signed.max()),
        }

    print("\nPer-stepper summary:")
    for oid, grp in df.groupby("oid"):
        s = summarize(grp)
        print(f"  oid={oid}: steps={s['steps']}, duration={s['duration_s']:.3f}s, "
              f"speed[min/med/max]=({s['min_speed_sps']:.1f}/{s['median_speed_sps']:.1f}/{s['max_speed_sps']:.1f}), "
              f"signed[min/max]=({s['min_signed_sps']:.1f}/{s['max_signed_sps']:.1f})")

    # Optional plots
    if args.plot:
        if not HAVE_PLOT:
            print("matplotlib not available; cannot plot.", file=sys.stderr)
        else:
            out_dir = args.out_dir or args.out_csv.parent
            out_dir.mkdir(parents=True, exist_ok=True)
            import matplotlib.pyplot as plt  # noqa: F401
            oids_sorted = sorted(df["oid"].unique())
            for oid in oids_sorted:
                g = df[df["oid"] == oid]
                fig = plt.figure(figsize=(10, 4.5))
                ax = fig.gca()
                ax.plot(g["time_s"].to_numpy(), g["signed_speed_sps"].to_numpy(), linewidth=0.8)
                ax.set_title(f"Stepper oid={oid} — requested speed vs local time\n(assumed CLOCK_HZ={int(args.clock_hz):,} Hz)")
                ax.set_xlabel("Local time (s)")
                ax.set_ylabel("Signed speed (steps/s)")
                ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
                fig.tight_layout()
                out_png = out_dir / f"speeds_oid{oid}.png"
                fig.savefig(out_png, dpi=150)
                plt.close(fig)
            print(f"Saved plots to: {out_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
