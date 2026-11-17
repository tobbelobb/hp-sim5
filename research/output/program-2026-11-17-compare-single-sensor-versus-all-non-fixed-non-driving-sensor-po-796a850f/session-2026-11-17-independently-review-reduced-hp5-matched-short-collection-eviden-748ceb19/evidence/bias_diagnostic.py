"""Where does the claimed -0.35..-0.86 mm extra-range 'bias' come from?

The report's tracking table compares the redundant encoder against a pose
trilaterated from three loaded lengths and nominal anchors. That pose is itself
the weakly determined quantity. Here the same extra range is checked against the
nominal anchors with the full five-line static solve, and the residual is split
by line and by sweep/direction.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import review_fit as R  # noqa: E402
import ext_fit  # noqa: E402
from analyze import geometry_table  # noqa: E402
from autocal.flex import FlexModel  # noqa: E402


def per_line_residuals(dataset, truth, measure_extra):
    table = ext_fit.line_lengths(dataset, measure_extra=measure_extra)
    lengths = table["lengths_mm"]
    measured = np.isfinite(lengths)
    config = dataset["config"]
    n = int(dataset["num_anchors"])
    model = FlexModel.from_m666(config["m666_before_data_collection"], num_axes=n)
    weight = float(config["m666_before_data_collection"].get("W", 0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    sensor_force = table["sensor_force_n"]
    redundant = table["redundant"]
    extra_force = sensor_force if measure_extra else sensor_force
    mode = "sensor"
    if mode == "sensor":
        extra_force = sensor_force
    force_lines = np.column_stack([ext_fit.point_sensor_anchors(dataset, table), redundant])
    forces = np.column_stack([sensor_force, extra_force])
    absolute = np.linalg.norm(truth, axis=1) + np.where(measured, lengths, 0.0)

    def tension_fn(positions):
        return ext_fit.corrected_lengths(truth, positions, absolute, force_lines, forces, weight)

    positions = ext_fit.solve_positions(truth, absolute, measured, model, tension_fn)
    residual = ext_fit.consistency_residuals(truth, positions, absolute, measured,
                                             tension_fn(positions), model)
    return residual, table, positions


def main():
    out = {}
    for run, mode in (("twosensor-1", "sensor"), ("ctl-1", "passive")):
        dataset, truth = R.load(run)
        out[run] = {}
        for measure_extra in (False, True):
            label = "five_line" if measure_extra else "four_line"
            residual, table, positions = per_line_residuals(dataset, truth, measure_extra)
            rows = []
            for index, sweep in enumerate(dataset["sweeps"]):
                used = set(int(a) for a in sweep["fixed_anchors"]) | {
                    int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])}
                redundant = next(a for a in range(int(dataset["num_anchors"])) if a not in used)
                mask = table["sweep_index"] == index
                values = residual[mask][:, redundant]
                rows.append(dict(
                    sweep=index, redundant=int(redundant),
                    mean_mm=float(np.mean(values)), std_mm=float(np.std(values)),
                    min_mm=float(np.min(values)), max_mm=float(np.max(values))))
            per_line = {}
            for axis in range(int(dataset["num_anchors"])):
                mask = np.isfinite(table["lengths_mm"][:, axis])
                if mask.any():
                    values = residual[mask][:, axis]
                    per_line[str(axis)] = dict(
                        n=int(mask.sum()), mean_mm=float(np.mean(values)),
                        rms_mm=float(np.sqrt(np.mean(values ** 2))),
                        max_abs_mm=float(np.max(np.abs(values))))
            out[run][label] = dict(redundant_by_sweep=rows, per_line=per_line)
    Path(HERE / "bias-diagnostic.json").write_text(json.dumps(out, indent=2) + "\n")
    for run, variants in out.items():
        print(run)
        for label, values in variants.items():
            print(" ", label, "redundant residual by sweep:",
                  [(row["sweep"], round(row["mean_mm"], 3), round(row["std_mm"], 3),
                    round(row["min_mm"], 3), round(row["max_mm"], 3)) for row in values["redundant_by_sweep"]])
            print("   per line:", {k: (round(v["mean_mm"], 3), round(v["rms_mm"], 3)) for k, v in values["per_line"].items()})
        # Tracking-table residual for comparison.
        geometry = geometry_table(dataset, truth)
        for label, measure_extra in (("four_line", False), ("five_line", True)):
            extra = np.array([g["encoder_minus_geometry"][g["row"].redundant] for g in geometry])
            print("   tracking residual", label, np.round(np.percentile(extra, [5, 50, 95]), 3))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
