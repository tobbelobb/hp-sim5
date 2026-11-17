"""Reviewer-side replays of the HP5 pulley-reduced two-sensor fit comparison.

Read-only imports from the original session's evidence. Nothing here modifies
the original artifacts or production code.

Adds three things the original analysis did not save:
  * a *general* home-offset model (free offset for every line), so the claimed
    gain can be tested against a less restrictive gauge;
  * explicit sensor-role ablation (physical `source_sensor_anchor` vs the
    canonical sweep sensor);
  * extra-range perturbations: constant bias, a perfect geometric placebo, and
    a point-shuffled placebo.
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
SOURCE = Path(
    "/home/torbjorn/repos/hp-sim5/research/output/"
    "program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/"
    "session-2026-11-17-hp5-pulley-reduced-independently-compare-the-current-single-acti-f359cbd6"
)
REPO = next(p for p in SESSION.parents if (p / "autocal/pose_fit.py").exists())
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(SOURCE / "evidence"))

import ext_fit  # noqa: E402
from analyze import nominal_anchors, geometry_table, tracking  # noqa: E402
from autocal.theoretical_ellipse import (  # noqa: E402
    anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec, get_anchor_opt_bounds,
)
from autocal.flex import FlexModel  # noqa: E402

MIN_SCALE_MM = 0.02
PRIOR_SIGMA_MM = 5.0
POSITION_ITERATIONS = 25


def load(run: str) -> tuple[dict, np.ndarray]:
    dataset = json.loads((SOURCE / "runs" / run / "sweeps.json").read_text())
    return dataset, nominal_anchors(dataset)


def strip_source_sensor(dataset: dict) -> dict:
    """Ablation: treat every point's canonical sweep sensor as the physical one."""
    out = copy.deepcopy(dataset)
    for sweep in out["sweeps"]:
        for point in sweep["data_points"]:
            point.pop("source_sensor_anchor", None)
    return out


def bias_extra(dataset: dict, bias_mm: float) -> dict:
    """Add a constant bias (mm) to the redundant line of every point."""
    out = copy.deepcopy(dataset)
    n = int(out["num_anchors"])
    for sweep in out["sweeps"]:
        used = set(int(a) for a in sweep["fixed_anchors"]) | {
            int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])}
        redundant = next((a for a in range(n) if a not in used), None)
        if redundant is None:
            continue
        scale = float(out["config"]["mm_per_degree"][redundant])
        for point in sweep["data_points"]:
            angles = point.get("mu") or point.get("raw_angles_deg")
            angles = list(angles)
            angles[redundant] = float(angles[redundant]) + bias_mm / scale
            point["mu"] = angles
            point["raw_angles_deg"] = angles
    return out


def replace_extra_with_geometry(dataset: dict, truth: np.ndarray) -> dict:
    """Placebo: replace the redundant range with the one predicted by the pose
    that the three position-controlled lines imply (nominal anchors)."""
    out = copy.deepcopy(dataset)
    table = ext_fit.line_lengths(out, measure_extra=False)
    n = int(out["num_anchors"])
    geometry = geometry_table(out, truth)
    for sweep_index, sweep in enumerate(out["sweeps"]):
        used = set(int(a) for a in sweep["fixed_anchors"]) | {
            int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])}
        redundant = next((a for a in range(n) if a not in used), None)
        if redundant is None:
            continue
        scale = float(out["config"]["mm_per_degree"][redundant])
        mask = np.where(table["sweep_index"] == sweep_index)[0]
        for point, index in zip(sweep["data_points"], mask):
            pose = geometry[index]["position"]
            predicted = float(np.linalg.norm(truth[redundant] - pose) - np.linalg.norm(truth[redundant]))
            angles = list(point.get("mu") or point.get("raw_angles_deg"))
            angles[redundant] = predicted / scale
            point["mu"] = angles
            point["raw_angles_deg"] = angles
    return out


def shuffle_extra(dataset: dict, seed: int) -> dict:
    """Placebo: keep the redundant ranges but destroy their point correspondence."""
    out = copy.deepcopy(dataset)
    n = int(out["num_anchors"])
    rng = np.random.default_rng(seed)
    entries = []  # (point, redundant axis, scale, original angle)
    for sweep in out["sweeps"]:
        used = set(int(a) for a in sweep["fixed_anchors"]) | {
            int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])}
        redundant = next((a for a in range(n) if a not in used), None)
        if redundant is None:
            continue
        scale = float(out["config"]["mm_per_degree"][redundant])
        for point in sweep["data_points"]:
            angles = point.get("mu") or point.get("raw_angles_deg")
            entries.append((point, redundant, scale, float(angles[redundant])))
    values = np.array([entry[3] for entry in entries])
    # Shuffle within each redundant axis: different axes have different gearing.
    for axis in sorted({entry[1] for entry in entries}):
        indices = [i for i, entry in enumerate(entries) if entry[1] == axis]
        permuted = rng.permutation([values[i] for i in indices])
        for i, value in zip(indices, permuted):
            point, redundant, scale, _ = entries[i]
            angles = list(point.get("mu") or point.get("raw_angles_deg"))
            angles[redundant] = float(value)
            point["mu"] = angles
            point["raw_angles_deg"] = angles
    return out


def fit_general(dataset: dict, initial_anchors: np.ndarray, *, measure_extra: bool,
                offset_lines: list[int], redundant_force_mode: str = "sensor",
                fit_offsets: bool = True,
                extra_initial_anchors=()) -> dict:
    """Same two-pass robust solve as ext_fit.fit_extra, but any set of lines may
    carry a free home offset."""
    machine_type = str(dataset["machine_type"])
    n, dimensions = int(dataset["num_anchors"]), int(dataset["dimensions"])
    table = ext_fit.line_lengths(dataset, measure_extra=measure_extra)
    lengths = table["lengths_mm"]
    measured = np.isfinite(lengths)
    config = dataset.get("config") or {}
    m666 = config.get("m666_before_data_collection") or {}
    model = FlexModel.from_m666(m666, num_axes=n)
    weight = float(m666.get("W", 0.0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    sensor_force = table["sensor_force_n"]
    redundant = table["redundant"]
    if redundant_force_mode == "sensor":
        extra_force = sensor_force
    elif redundant_force_mode == "passive":
        extra_force = np.full(len(sensor_force), passive)
    else:
        extra_force = np.full(len(sensor_force), float(redundant_force_mode))
    points = lengths.shape[0]
    force_lines = np.column_stack([ext_fit.point_sensor_anchors(dataset, table), redundant])
    forces = np.column_stack([sensor_force, extra_force])

    used_lines = list(offset_lines) if fit_offsets else []
    num_offsets = len(used_lines)

    def unpack(x):
        anchors = anchor_opt_vec_to_matrix(x[: x.size - num_offsets], machine_type, n, dimensions, None)
        offsets = np.zeros(n)
        if num_offsets:
            offsets[used_lines] = x[x.size - num_offsets:]
        return anchors, offsets

    def residuals_of(x, prior_weight):
        anchors, offsets = unpack(x)
        absolute = np.linalg.norm(anchors, axis=1) + offsets + np.where(measured, lengths, 0.0)

        def tension_fn(positions):
            return ext_fit.corrected_lengths(anchors, positions, absolute, force_lines, forces, weight)

        positions = ext_fit.solve_positions(anchors, absolute, measured, model, tension_fn)
        tensions = tension_fn(positions)
        return np.concatenate([
            ext_fit.consistency_residuals(anchors, positions, absolute, measured, tensions, model).ravel(),
            offsets[used_lines] * prior_weight if num_offsets else np.zeros(0),
        ])

    lb, ub = get_anchor_opt_bounds(machine_type, n, dimensions, None)
    bounds = (np.concatenate([lb, np.full(num_offsets, -np.inf)]),
              np.concatenate([ub, np.full(num_offsets, np.inf)]))

    def solve(start_anchors):
        anchor_start = anchors_matrix_to_opt_vec(np.asarray(start_anchors, dtype=float), machine_type, None)
        start = np.concatenate([np.clip(anchor_start, lb, ub), np.zeros(num_offsets)])
        x_scale = np.concatenate([np.full(start.size - num_offsets, 100.0), np.ones(num_offsets)])
        first = least_squares(residuals_of, start, args=(MIN_SCALE_MM / PRIOR_SIGMA_MM,),
                              bounds=bounds, x_scale=x_scale, loss="soft_l1", f_scale=1.0)
        point_residuals = first.fun[: lengths.size].reshape(lengths.shape)[measured]
        scale = max(1.4826 * float(np.median(np.abs(point_residuals - np.median(point_residuals)))),
                    MIN_SCALE_MM)
        return least_squares(residuals_of, first.x,
                             args=(scale / PRIOR_SIGMA_MM,), bounds=bounds,
                             x_scale=x_scale, loss="cauchy", f_scale=3.0 * scale,
                             xtol=1e-12, ftol=1e-12), scale

    solutions = [solve(start) for start in [initial_anchors, *extra_initial_anchors]]
    scale = min(s[1] for s in solutions)

    def cost(solution):
        return float(np.sum(np.log1p((residuals_of(solution[0].x, scale / PRIOR_SIGMA_MM)
                                      / (3.0 * scale)) ** 2)))

    converged = [s for s in solutions if s[0].success]
    best, _ = min(converged or solutions, key=cost)
    anchors, offsets = unpack(best.x)
    per_point = np.linalg.norm(best.fun[: lengths.size].reshape(lengths.shape), axis=1)
    return {
        "anchors": anchors, "home_offsets_mm": offsets,
        "rms_mm": float(np.sqrt(np.mean(per_point ** 2))),
        "robust_scale_mm": float(scale), "point_residuals_mm": per_point,
        "success": bool(best.success), "x": best.x,
        "lengths_mm": lengths, "measured": measured,
    }


def pipeline(dataset: dict, truth: np.ndarray, *, measure_extra: bool,
             offset_lines: list[int], redundant_force_mode: str,
             start: np.ndarray | None = None, extra_starts=()) -> dict:
    start = truth * 1.05 if start is None else start
    first = fit_general(dataset, start, measure_extra=measure_extra,
                        offset_lines=offset_lines, redundant_force_mode=redundant_force_mode,
                        fit_offsets=bool(offset_lines), extra_initial_anchors=extra_starts)
    if offset_lines:
        shifted = ext_fit.shift_by_offsets(dataset, first["home_offsets_mm"],
                                           measure_extra=measure_extra)
    else:
        shifted = dataset
    second = fit_general(shifted, first["anchors"], measure_extra=measure_extra,
                         offset_lines=[], redundant_force_mode=redundant_force_mode,
                         fit_offsets=False)
    error = ext_fit.anchor_error(second["anchors"], truth)
    return {
        "measure_extra": measure_extra,
        "offset_lines": list(offset_lines),
        "rms_mm": second["rms_mm"],
        "robust_scale_mm": second["robust_scale_mm"],
        "home_offsets_mm": first["home_offsets_mm"].tolist(),
        "max_aligned_abs_coordinate_mm": error["max_aligned_abs_coordinate_mm"],
        "max_abs_coordinate_mm": error["max_abs_coordinate_mm"],
        "per_anchor_aligned_mm": error["per_anchor_aligned_mm"],
        "anchors": second["anchors"].tolist(),
        "success": bool(first["success"] and second["success"]),
    }


def truth_residual(dataset: dict, truth: np.ndarray, *, measure_extra: bool,
                   redundant_force_mode: str) -> dict:
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
    if redundant_force_mode == "sensor":
        extra_force = sensor_force
    elif redundant_force_mode == "passive":
        extra_force = np.full(len(sensor_force), passive)
    else:
        extra_force = np.full(len(sensor_force), float(redundant_force_mode))
    force_lines = np.column_stack([ext_fit.point_sensor_anchors(dataset, table), redundant])
    forces = np.column_stack([sensor_force, extra_force])
    absolute = np.linalg.norm(truth, axis=1) + np.where(measured, lengths, 0.0)

    def tension_fn(positions):
        return ext_fit.corrected_lengths(truth, positions, absolute, force_lines, forces, weight)

    positions = ext_fit.solve_positions(truth, absolute, measured, model, tension_fn)
    residual = ext_fit.consistency_residuals(truth, positions, absolute, measured,
                                             tension_fn(positions), model)
    per_point = np.linalg.norm(np.where(measured, residual, 0.0), axis=1)
    return {"rms_mm": float(np.sqrt(np.mean(per_point ** 2))),
            "median_mm": float(np.median(per_point)),
            "max_mm": float(np.max(per_point))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(SESSION / "review-fits.json"))
    parser.add_argument("--skip-expensive", action="store_true")
    args = parser.parse_args()

    dataset_b, truth_b = load("twosensor-1")
    dataset_a, truth_a = load("ctl-1")
    out: dict = {"reference": {}, "sensor_role": {}, "offset_gauge": {},
                 "extra_perturbations": {}}

    # 1. Reference replay: production offsets on lines 3,4; physical sensor.
    for run, dataset, truth, mode in (("twosensor-1", dataset_b, truth_b, "sensor"),
                                      ("ctl-1", dataset_a, truth_a, "passive")):
        out["reference"][run] = {}
        for measure_extra in (False, True):
            key = "five_line" if measure_extra else "four_line"
            out["reference"][run][key] = pipeline(
                dataset, truth, measure_extra=measure_extra, offset_lines=[3, 4],
                redundant_force_mode=mode)

    # 2. Sensor-role ablation on arm B.
    canonical_b = strip_source_sensor(dataset_b)
    for label, dataset in (("physical", dataset_b), ("canonical", canonical_b)):
        out["sensor_role"][label] = {}
        for measure_extra in (False, True):
            key = "five_line" if measure_extra else "four_line"
            out["sensor_role"][label][key + "_truth_residual"] = truth_residual(
                dataset, truth_b, measure_extra=measure_extra, redundant_force_mode="sensor")
            out["sensor_role"][label][key + "_fit"] = pipeline(
                dataset, truth_b, measure_extra=measure_extra, offset_lines=[3, 4],
                redundant_force_mode="sensor")

    # 3. Offset gauge variants on arm B (physical sensor).
    for label, lines in (("production_offsets", [3, 4]), ("no_offsets", []),
                         ("all_line_offsets", [0, 1, 2, 3, 4])):
        out["offset_gauge"][label] = {}
        for measure_extra in (False, True):
            key = "five_line" if measure_extra else "four_line"
            out["offset_gauge"][label][key] = pipeline(
                dataset_b, truth_b, measure_extra=measure_extra, offset_lines=lines,
                redundant_force_mode="sensor", start=truth_b,
                extra_starts=(truth_b * 1.05,))

    # 4. Extra-range perturbations on arm B, production offsets.
    out["extra_perturbations"]["bias_mm"] = {}
    for bias in (-0.8, -0.4, -0.2, -0.1, 0.0, 0.1, 0.2, 0.4, 0.8, 1.2):
        perturbed = bias_extra(dataset_b, bias)
        out["extra_perturbations"]["bias_mm"][f"{bias:+.1f}"] = pipeline(
            perturbed, truth_b, measure_extra=True, offset_lines=[3, 4],
            redundant_force_mode="sensor")
    if not args.skip_expensive:
        out["extra_perturbations"]["geometric_placebo"] = pipeline(
            replace_extra_with_geometry(dataset_b, truth_b), truth_b,
            measure_extra=True, offset_lines=[3, 4], redundant_force_mode="sensor")
        out["extra_perturbations"]["shuffled_placebo"] = {}
        for seed in (0, 1, 2):
            out["extra_perturbations"]["shuffled_placebo"][str(seed)] = pipeline(
                shuffle_extra(dataset_b, seed), truth_b, measure_extra=True,
                offset_lines=[3, 4], redundant_force_mode="sensor")

    Path(args.output).write_text(json.dumps(out, indent=2) + "\n")
    for section, values in out.items():
        print(section)
        for key, value in values.items():
            summary = {}
            for label, fit in (value.items() if isinstance(value, dict) else []):
                if isinstance(fit, dict) and "max_aligned_abs_coordinate_mm" in fit:
                    summary[label] = round(fit["max_aligned_abs_coordinate_mm"], 3)
            print(" ", key, summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
