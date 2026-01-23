#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def _load_residuals(csv_path: Path, *, column: str) -> list[float]:
    values: list[float] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if column not in row:
                continue
            try:
                val = float(row[column])
            except (TypeError, ValueError):
                continue
            if val == val:
                values.append(val)
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plot a residual histogram from CSV output.")
    parser.add_argument("csv_path", type=Path, help="CSV file produced by --residuals-csv")
    parser.add_argument("--column", default="residual_mm", help="Column to plot (default: residual_mm)")
    parser.add_argument("--bins", type=int, default=60, help="Histogram bins (default: 60)")
    parser.add_argument("--max-mm", type=float, default=None, help="Optional max residual to include (mm)")
    parser.add_argument("--output", type=Path, default=None, help="Optional output PNG path")
    args = parser.parse_args(argv)

    values = _load_residuals(args.csv_path, column=args.column)
    if args.max_mm is not None:
        values = [v for v in values if v <= args.max_mm]

    if not values:
        raise SystemExit("No residuals to plot.")

    plt.hist(values, bins=int(args.bins))
    plt.xlabel("Residual error (mm)")
    plt.ylabel("Count of points")
    plt.title("Residual Histogram")
    plt.tight_layout()

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(args.output, dpi=150)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
