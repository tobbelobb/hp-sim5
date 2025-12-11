#!/usr/bin/env python3
from __future__ import annotations

"""CLI tool to fit ellipses to sweep calibration data."""

import argparse
import json
import sys
from pathlib import Path

from autocal.ellipse_fitting import fit_all_sweeps
from autocal.sweep_io import load_sweep_dataset


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit ellipses to sweep calibration data.")
    parser.add_argument("input", help="Input sweep data JSON file (absolute lengths expected)")
    parser.add_argument(
        "-o",
        "--output",
        help="Output JSON file (default: <input>_fits.json)",
    )
    parser.add_argument(
        "-t",
        "--threshold",
        type=float,
        default=0.01,
        help="RMS residual threshold for valid fit (default: 0.01)",
    )
    parser.add_argument(
        "--min-points",
        type=int,
        default=10,
        help="Minimum number of points required for fitting (default: 10)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print per-sweep status",
    )

    args = parser.parse_args()

    dataset = load_sweep_dataset(args.input)

    fitted_ellipses = fit_all_sweeps(
        dataset.sweeps, residual_threshold=args.threshold, min_points=args.min_points
    )

    valid_count = sum(1 for e in fitted_ellipses if e["valid"])
    total = len(fitted_ellipses)
    print(f"Fitted {total} sweeps: {valid_count} valid, {total - valid_count} rejected")

    if args.verbose:
        for fe in fitted_ellipses:
            status = "VALID" if fe["valid"] else f"REJECTED: {fe['rejection_reason']}"
            sweep_id = fe.get("sweep_id") or "<unnamed>"
            residual = fe.get("residual_rms", float("nan"))
            print(f"  {sweep_id}: RMS={residual:.6f} {status}")

    input_path = Path(args.input)
    output_path = Path(args.output) if args.output else input_path.with_name(input_path.stem + "_fits.json")
    output_payload = {
        "source": str(input_path),
        "residual_threshold": args.threshold,
        "min_points": args.min_points,
        "fitted_ellipses": fitted_ellipses,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(output_payload, f, indent=2)

    print(f"Saved to {output_path}")

    if valid_count == 0:
        print("ERROR: No valid ellipse fits!", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
