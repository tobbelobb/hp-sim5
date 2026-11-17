"""Experimental point fit that can add the redundant free line as a measured range.

This is a faithful copy of autocal.pose_fit's residual/optimizer structure. The two
deliberate differences are:

  * `measure_extra` also adds each point's redundant free line (the one that is
    neither fixed, drive nor sensor) to the geometry residual.
  * the static tension solve is written once for all lines: every line has either a
    position constraint (fixed/drive) or a known torque (sensor, and the redundant
    line in both policies). That is algebraically the production model, which
    solves the active d+1 lines and adds the redundant line's known force to the
    load vector.

`measure_extra=False` is the control: it should reproduce the production fit.
"""
from __future__ import annotations

from typing import Dict, Optional, Sequence

import numpy as np
from scipy.optimize import least_squares

from autocal.flex import FlexModel
from autocal.theoretical_ellipse import (
    anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec, get_anchor_opt_bounds,
)

HOME_OFFSET_PRIOR_SIGMA_MM = 5.0
_POSITION_ITERATIONS = 25
_MIN_SCALE_MM = 0.02


def _mean(point: dict, key: str, raw: str) -> float:
    value = point.get(key)
    if value is None:
        value = point.get(raw)
    return float(value)


def line_lengths(dataset: dict, *, measure_extra: bool) -> Dict[str, np.ndarray]:
    """Every point's length change on every line, NaN where it is not a measurement."""
    n = int(dataset["num_anchors"])
    mm_per_degree = (dataset.get("config") or {}).get("mm_per_degree")
    assert mm_per_degree is not None, "mm_per_degree is required to read the redundant encoder"
    rows, sweep_index, extras, forces = [], [], [], []
    for index, sweep in enumerate(dataset.get("sweeps", [])):
        fixed = [int(a) for a in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        used = set(fixed) | {drive, sensor}
        redundant = next((axis for axis in range(n) if axis not in used), None)
        for point in sweep.get("data_points", []):
            row = [float("nan")] * n
            errors = (point.get("fixed_error_angles_mu_deg")
                      if point.get("fixed_error_angles_mu_deg") is not None
                      else point.get("fixed_error_angles_deg"))
            for anchor, target in zip(fixed, sweep["fixed_lengths"]):
                row[anchor] = float(target)
                if errors is not None and errors[anchor] is not None:
                    row[anchor] += float(errors[anchor]) * float(mm_per_degree[anchor])
            row[drive] = _mean(point, "l_drive_mu", "l_drive")
            row[sensor] = _mean(point, "l_sensor_mu", "l_sensor")
            if measure_extra and redundant is not None:
                angles = point.get("mu") or point.get("raw_angles_deg")
                row[redundant] = float(angles[redundant]) * float(mm_per_degree[redundant])
            rows.append(row)
            sweep_index.append(index)
            extras.append(-1 if redundant is None else redundant)
            forces.append(float(point.get("sensor_force_n", np.nan)))
    return {
        "lengths_mm": np.asarray(rows, dtype=float).reshape(-1, n),
        "sweep_index": np.asarray(sweep_index, dtype=int),
        "redundant": np.asarray(extras, dtype=int),
        "sensor_force_n": np.asarray(forces, dtype=float),
    }


def shift_by_offsets(dataset: dict, offsets_mm: np.ndarray, *, measure_extra: bool) -> dict:
    """Shift every measured line by its fitted home offset (production's shift + extra)."""
    shifted = dict(dataset)
    shifted["sweeps"] = []
    mm_per_degree = (dataset.get("config") or {}).get("mm_per_degree")
    n = int(dataset["num_anchors"])
    for sweep in dataset.get("sweeps", []):
        fixed = [int(a) for a in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        redundant = next((a for a in range(n) if a not in set(fixed) | {drive, sensor}), None)
        out = dict(sweep)
        out["fixed_lengths"] = [float(v) + float(offsets_mm[a])
                                for a, v in zip(fixed, sweep["fixed_lengths"])]
        out["data_points"] = []
        for point in sweep.get("data_points", []):
            new = dict(point)
            for key, anchor in (("l_drive", drive), ("l_drive_mu", drive),
                                ("l_sensor", sensor), ("l_sensor_mu", sensor)):
                if new.get(key) is not None:
                    new[key] = float(new[key]) + float(offsets_mm[anchor])
            if measure_extra and redundant is not None:
                angles = new.get("mu") or new.get("raw_angles_deg")
                shifted_angle = list(angles)
                shifted_angle[redundant] = (float(angles[redundant])
                                            + float(offsets_mm[redundant]) / float(mm_per_degree[redundant]))
                new["mu"] = shifted_angle
                new["raw_angles_deg"] = shifted_angle
            out["data_points"].append(new)
        shifted["sweeps"].append(out)
    return shifted


def corrected_lengths(anchors: np.ndarray, positions: np.ndarray, absolute: np.ndarray,
                      force_lines: np.ndarray, forces: np.ndarray, weight_n: float) -> np.ndarray:
    """Static tensions for every line: 3 equilibrium rows plus one known-force row per
    torque-controlled line (the sensor, and the redundant free line in both policies).

    force_lines: (points, k) anchor indices with a commanded torque.
    forces: (points, k) those commanded forces in N.
    """
    points, n = absolute.shape
    to_anchor = anchors[None, :, :] - positions[:, None, :]
    distance = np.maximum(np.linalg.norm(to_anchor, axis=2), 1e-9)
    directions = (to_anchor / distance[:, :, None]).transpose(0, 2, 1)  # (p, 3, n)
    k = force_lines.shape[1]
    selector = np.zeros((points, k, n))
    for j in range(k):
        selector[np.arange(points), j, force_lines[:, j]] = 1.0
    matrix = np.concatenate([directions, selector], axis=1)  # (p, 3+k, n)
    load = np.zeros((points, 3 + k))
    load[:, 2] = -weight_n
    load[:, 3:] = forces
    # Tiny ridge keeps singular trial geometries finite, as production does.
    transpose = matrix.transpose(0, 2, 1)
    normal = transpose @ matrix + 1e-12 * np.eye(n)
    return np.linalg.solve(normal, (transpose @ load[:, :, None]))[:, :, 0]


def _corrected(anchors, positions, absolute, tensions, model):
    return np.column_stack([
        model.corrected_distance_mm(absolute[:, axis], tensions[:, axis], axis=axis)
        for axis in range(anchors.shape[0])
    ])


def consistency_residuals(anchors: np.ndarray, positions: np.ndarray, absolute: np.ndarray,
                          measured: np.ndarray, tensions: np.ndarray, model: FlexModel) -> np.ndarray:
    corrected = _corrected(anchors, positions, absolute, tensions, model)
    residual = np.linalg.norm(positions[:, None, :] - anchors[None, :, :], axis=2) - corrected
    return np.where(measured, residual, 0.0)


def solve_positions(anchors: np.ndarray, absolute: np.ndarray, measured: np.ndarray,
                    model: FlexModel, tension_fn) -> np.ndarray:
    """Production's position iteration: tensions are recomputed from the current
    positions before each stretch correction, exactly as pose_fit does."""
    points = absolute.shape[0]
    dimensions = anchors.shape[1]
    positions = np.zeros((points, dimensions))
    damping = 1e-9 * np.eye(dimensions)
    for _ in range(_POSITION_ITERATIONS):
        to_effector = positions[:, None, :] - anchors[None, :, :]
        distance = np.maximum(np.linalg.norm(to_effector, axis=2), 1e-9)
        direction = to_effector / distance[:, :, None] * measured[:, :, None]
        tensions = tension_fn(positions)
        corrected = _corrected(anchors, positions, absolute, tensions, model)
        misfit = np.where(measured, distance - corrected, 0.0)
        normal = np.einsum("pni,pnj->pij", direction, direction) + damping
        gradient = np.einsum("pni,pn->pi", direction, misfit)
        positions = positions - np.linalg.solve(normal, gradient[:, :, None])[:, :, 0]
    return positions


def point_sensor_anchors(dataset: dict, table: dict) -> np.ndarray:
    """Physical sensor line per point: `source_sensor_anchor` when the collector
    remapped a reversed sub-sweep into canonical drive/sensor order."""
    sensors = np.zeros(len(table["sweep_index"]), dtype=int)
    for index, sweep in enumerate(dataset.get("sweeps", [])):
        mask = table["sweep_index"] == index
        values = [point.get("source_sensor_anchor") for point in sweep.get("data_points", [])]
        sensors[mask] = [int(v) if v is not None else int(sweep["sensor_anchor"]) for v in values]
    return sensors


def fit_extra(dataset: dict, initial_anchors: np.ndarray, *, measure_extra: bool,
              fit_home_offsets: bool, use_noise_mean: bool = True,
              redundant_force_mode: str = "sensor",
              extra_initial_anchors: Sequence[np.ndarray] = ()) -> Dict[str, object]:
    machine_type = str(dataset["machine_type"])
    n, dimensions = int(dataset["num_anchors"]), int(dataset["dimensions"])
    table = line_lengths(dataset, measure_extra=measure_extra)
    lengths = table["lengths_mm"]
    measured = np.isfinite(lengths)
    if not np.all(np.sum(measured, axis=1) >= dimensions + 1):
        raise ValueError("need at least d+1 finite lengths per point")
    config = dataset.get("config") or {}
    m666 = config.get("m666_before_data_collection") or {}
    model = FlexModel.from_m666(m666, num_axes=n)
    weight = float(m666.get("W", 0.0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    sensor_force = table["sensor_force_n"]
    redundant = table["redundant"]
    # The redundant line's torque is a policy fact, not an inference:
    # "sensor" = two-sensor prototype (same measurement force),
    # "passive" = current collector (idle/compliant force).
    if redundant_force_mode == "sensor":
        extra_force = sensor_force
    elif redundant_force_mode == "passive":
        extra_force = np.full(len(sensor_force), passive)
    else:
        extra_force = np.full(len(sensor_force), float(redundant_force_mode))
    points = lengths.shape[0]
    force_lines = np.column_stack([point_sensor_anchors(dataset, table), redundant])
    forces = np.column_stack([sensor_force, extra_force])

    num_offsets = n - dimensions if fit_home_offsets else 0
    offset_lines = list(range(n - num_offsets, n))

    def unpack(x):
        anchors = anchor_opt_vec_to_matrix(x[: x.size - num_offsets], machine_type, n, dimensions, None)
        offsets = np.zeros(n)
        offsets[offset_lines] = x[x.size - num_offsets:]
        return anchors, offsets

    def residuals_of(x, prior_weight):
        anchors, offsets = unpack(x)
        absolute = np.linalg.norm(anchors, axis=1) + offsets + np.where(measured, lengths, 0.0)

        def tension_fn(positions):
            return corrected_lengths(anchors, positions, absolute, force_lines, forces, weight)

        positions = solve_positions(anchors, absolute, measured, model, tension_fn)
        tensions = tension_fn(positions)
        return np.concatenate([
            consistency_residuals(anchors, positions, absolute, measured, tensions, model).ravel(),
            offsets[offset_lines] * prior_weight,
        ])

    lb, ub = get_anchor_opt_bounds(machine_type, n, dimensions, None)
    bounds = (np.concatenate([lb, np.full(num_offsets, -np.inf)]),
              np.concatenate([ub, np.full(num_offsets, np.inf)]))

    def solve(start_anchors):
        anchor_start = anchors_matrix_to_opt_vec(np.asarray(start_anchors, dtype=float), machine_type, None)
        start = np.concatenate([np.clip(anchor_start, lb, ub), np.zeros(num_offsets)])
        x_scale = np.concatenate([np.full(start.size - num_offsets, 100.0), np.ones(num_offsets)])
        first = least_squares(residuals_of, start, args=(_MIN_SCALE_MM / HOME_OFFSET_PRIOR_SIGMA_MM,),
                              bounds=bounds, x_scale=x_scale, loss="soft_l1", f_scale=1.0)
        point_residuals = first.fun[: lengths.size].reshape(lengths.shape)[measured]
        scale = max(1.4826 * float(np.median(np.abs(point_residuals - np.median(point_residuals)))), _MIN_SCALE_MM)
        return least_squares(residuals_of, first.x,
                             args=(scale / HOME_OFFSET_PRIOR_SIGMA_MM,), bounds=bounds,
                             x_scale=x_scale, loss="cauchy", f_scale=3.0 * scale,
                             xtol=1e-12, ftol=1e-12), scale

    solutions = [solve(start) for start in [initial_anchors, *extra_initial_anchors]]
    scale = min(s[1] for s in solutions)

    def cost(solution):
        return float(np.sum(np.log1p((residuals_of(solution[0].x, scale / HOME_OFFSET_PRIOR_SIGMA_MM)
                                      / (3.0 * scale)) ** 2)))

    converged = [s for s in solutions if s[0].success]
    best, _ = min(converged or solutions, key=cost)
    anchors, offsets = unpack(best.x)
    per_point = np.linalg.norm(best.fun[: lengths.size].reshape(lengths.shape), axis=1)
    return {
        "anchors": anchors, "home_offsets_mm": offsets,
        "rms_mm": float(np.sqrt(np.mean(per_point ** 2))),
        "robust_scale_mm": float(scale), "point_residuals_mm": per_point,
        "num_points": int(lengths.shape[0]), "measured_count": int(measured.sum()),
        "success": bool(best.success), "x": best.x,
        "lengths_mm": lengths, "measured": measured,
    }


def anchor_error(fitted: np.ndarray, truth: np.ndarray) -> Dict[str, float]:
    diff = np.asarray(fitted) - np.asarray(truth)
    shift = diff.mean(axis=0)
    aligned = diff - shift
    return {
        "max_abs_coordinate_mm": float(np.max(np.abs(diff))),
        "max_aligned_abs_coordinate_mm": float(np.max(np.abs(aligned))),
        "per_anchor_mm": np.linalg.norm(diff, axis=1).tolist(),
        "per_anchor_aligned_mm": np.linalg.norm(aligned, axis=1).tolist(),
        "translation_mm": shift.tolist(),
    }
