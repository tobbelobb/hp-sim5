"""Noise sensitivity at matched budget: perturb measured lengths and refit.

The four-sweep bootstrap solve is weakly conditioned in some anchor directions, so
one noise realisation can move the estimate by several millimetres. This compares
the current (4-line) and extra-sensor (5-line) fits over the same perturbations, on
the same poses, using a fixed seed list.
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--sigma-mm", type=float, default=0.02)
    parser.add_argument("--repeats", type=int, default=6)
    parser.add_argument("--output", default=str(SESSION / "noise-sensitivity.json"))
    args = parser.parse_args()
    output = {}
    for run in args.runs:
        directory = Path(run)
        dataset = json.loads((directory / "sweeps.json").read_text())
        invocation = json.loads((directory / "invocation.json").read_text())
        truth = nominal_anchors(dataset)
        mode = invocation.get("redundant_force_mode", "passive")
        results = {"current": [], "extra": []}
        for seed in range(args.repeats):
            rng = np.random.default_rng(seed)
            noisy = json.loads(json.dumps(dataset))
            for sweep in noisy["sweeps"]:
                for point in sweep["data_points"]:
                    for key in ("l_drive", "l_drive_mu", "l_sensor", "l_sensor_mu"):
                        if point.get(key) is not None:
                            point[key] += float(rng.normal(0.0, args.sigma_mm))
                    if point.get("mu") is not None:
                        point["mu"] = [float(v) + float(rng.normal(0.0, args.sigma_mm / 0.26))
                                       for v in point["mu"]]
            for label, measure_extra in (("current", False), ("extra", True)):
                offset = ext_fit.fit_extra(noisy, truth, measure_extra=measure_extra,
                                           fit_home_offsets=True, redundant_force_mode=mode)
                shifted = ext_fit.shift_by_offsets(noisy, offset["home_offsets_mm"],
                                                   measure_extra=measure_extra)
                fit = ext_fit.fit_extra(shifted, offset["anchors"], measure_extra=measure_extra,
                                        fit_home_offsets=False, redundant_force_mode=mode)
                error = ext_fit.anchor_error(fit["anchors"], truth)
                results[label].append(dict(seed=seed,
                                           max_aligned_mm=error["max_aligned_abs_coordinate_mm"],
                                           rms_mm=fit["rms_mm"]))
                print(f"{directory.name:22s} {label:8s} seed {seed} "
                      f"{error['max_aligned_abs_coordinate_mm']:8.2f} mm", flush=True)
        summary = {}
        for label, rows in results.items():
            values = np.array([row["max_aligned_mm"] for row in rows])
            summary[label] = dict(rows=rows, median_mm=float(np.median(values)),
                                  p95_mm=float(np.percentile(values, 95)),
                                  max_mm=float(values.max()))
        output[directory.name] = summary
    Path(args.output).write_text(json.dumps(output, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
