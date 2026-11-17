"""Fit-start sensitivity: is the two-sensor gain robust to the optimizer's basin?

The four-sweep bootstrap solve is weakly conditioned in the top-anchor height, so a
single fit can land in different basins. This script refits each policy variant from
several explicit starts and reports every result, not just the best one.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/pose_fit.py").exists())
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HERE))

import ext_fit  # noqa: E402
from analyze import nominal_anchors  # noqa: E402


def starts(truth: np.ndarray):
    centroid = truth.mean(axis=0)
    yield "truth", truth
    yield "truth_x1.05", truth * 1.05
    yield "truth_x0.95", truth * 0.95
    taller = truth.copy()
    taller[-1, 2] += 250.0
    yield "top_z+250", taller
    shorter = truth.copy()
    shorter[-1, 2] -= 250.0
    yield "top_z-250", shorter
    yield "truth_shifted", truth + np.array([15.0, -10.0, 20.0])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="*")
    parser.add_argument("--output", default=str(SESSION / "sensitivity.json"))
    args = parser.parse_args()
    dirs = [Path(r) for r in args.runs] or sorted(p for p in (SESSION / "runs").iterdir()
                                                  if (p / "sweeps.json").exists())
    output = {}
    for directory in dirs:
        dataset = json.loads((directory / "sweeps.json").read_text())
        invocation = json.loads((directory / "invocation.json").read_text())
        truth = nominal_anchors(dataset)
        mode = invocation.get("redundant_force_mode", "passive")
        rows = []
        for label, measure_extra in (("current", False), ("extra", True)):
            for start_label, start in starts(truth):
                offset = ext_fit.fit_extra(dataset, start, measure_extra=measure_extra,
                                           fit_home_offsets=True, redundant_force_mode=mode)
                shifted = ext_fit.shift_by_offsets(dataset, offset["home_offsets_mm"],
                                                   measure_extra=measure_extra)
                fit = ext_fit.fit_extra(shifted, offset["anchors"], measure_extra=measure_extra,
                                        fit_home_offsets=False, redundant_force_mode=mode)
                error = ext_fit.anchor_error(fit["anchors"], truth)
                rows.append(dict(variant=label, start=start_label, success=bool(fit["success"]),
                                 max_aligned_abs_coordinate_mm=error["max_aligned_abs_coordinate_mm"],
                                 max_abs_coordinate_mm=error["max_abs_coordinate_mm"],
                                 rms_mm=fit["rms_mm"]))
                print(f"{directory.name:22s} {label:8s} {start_label:14s} "
                      f"{error['max_aligned_abs_coordinate_mm']:9.2f} mm", flush=True)
        for label in ("current", "extra"):
            values = [row["max_aligned_abs_coordinate_mm"] for row in rows if row["variant"] == label]
            output.setdefault(directory.name, {})[label] = dict(
                starts=[row for row in rows if row["variant"] == label],
                median_max_aligned_mm=float(np.median(values)),
                worst_max_aligned_mm=float(np.max(values)),
                best_max_aligned_mm=float(np.min(values)))
    Path(args.output).write_text(json.dumps(output, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
