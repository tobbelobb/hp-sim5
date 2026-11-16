"""Point-based least-squares anchor fit, kept beside the ellipse fitter for comparison.

Every data point holds one length change per line. A rigid machine with n lines
and d effector coordinates leaves n - d consistency conditions on those lengths.
This fit moves the anchors until all points satisfy them, without assuming that a
sweep traces an ellipse.

It can also fit home length offsets. The encoder origin is taken with some lines
slack while every measurement is taken under tension, so the lengths at the origin
do not describe one effector position. Of the n possible per-line offsets, d are
equivalent to moving the home position, which leaves n - d real ones. They are
placed on the last n - d lines.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import least_squares

from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec, get_anchor_opt_bounds

# Wide enough for the stretch of any realistic line, narrow enough to keep the offsets
# from trading against anchor moves when all poses lie on one surface.
HOME_OFFSET_PRIOR_SIGMA_MM = 5.0
_POSITION_ITERATIONS = 25
_MIN_SCALE_MM = 0.02


def _point_value(point: dict, mean_key: str, raw_key: str, use_noise_mean: bool) -> float:
    if use_noise_mean and point.get(mean_key) is not None:
        return float(point[mean_key])
    return float(point[raw_key])


def dataset_line_lengths(dataset: dict, *, use_noise_mean: bool = True) -> Dict[str, np.ndarray]:
    """Return every point's length change on every line, in mm, with its sweep index."""
    num_anchors = int(dataset["num_anchors"])
    mm_per_degree = (dataset.get("config") or {}).get("mm_per_degree")
    lengths: List[List[float]] = []
    sweep_index: List[int] = []
    for index, sweep in enumerate(dataset.get("sweeps", [])):
        fixed = [int(anchor) for anchor in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        for point in sweep.get("data_points", []):
            # Unused winches are compliant and can be slack. Their encoders
            # cannot constrain the effector pose; NaN marks an absent range.
            row = [float("nan")] * num_anchors
            errors = point.get("fixed_error_angles_mu_deg") if use_noise_mean else None
            if errors is None:
                errors = point.get("fixed_error_angles_deg")
            for anchor, target in zip(fixed, sweep["fixed_lengths"]):
                row[anchor] = float(target)
                if mm_per_degree is not None and errors is not None and errors[anchor] is not None:
                    row[anchor] += float(errors[anchor]) * float(mm_per_degree[anchor])
            row[drive] = _point_value(point, "l_drive_mu", "l_drive", use_noise_mean)
            row[sensor] = _point_value(point, "l_sensor_mu", "l_sensor", use_noise_mean)
            lengths.append(row)
            sweep_index.append(index)
    return {
        "lengths_mm": np.asarray(lengths, dtype=float).reshape(-1, num_anchors),
        "sweep_index": np.asarray(sweep_index, dtype=int),
    }


def consistency_residuals(anchors: np.ndarray, lengths_mm: np.ndarray, home_offsets_mm: np.ndarray) -> np.ndarray:
    """Per-point, per-line length misfit at each point's best effector position, in mm."""
    anchors = np.asarray(anchors, dtype=float)
    dimensions = anchors.shape[1]
    measured = np.isfinite(lengths_mm)
    absolute = (np.linalg.norm(anchors, axis=1) + np.asarray(home_offsets_mm, dtype=float)
                + np.where(measured, lengths_mm, 0.0))
    positions = np.zeros((lengths_mm.shape[0], dimensions))
    damping = 1e-9 * np.eye(dimensions)
    for _ in range(_POSITION_ITERATIONS):
        to_effector = positions[:, None, :] - anchors[None, :, :]
        distance = np.linalg.norm(to_effector, axis=2)
        direction = to_effector / distance[:, :, None] * measured[:, :, None]
        misfit = np.where(measured, distance - absolute, 0.0)
        normal = np.einsum("pni,pnj->pij", direction, direction) + damping
        gradient = np.einsum("pni,pn->pi", direction, misfit)
        positions = positions - np.linalg.solve(normal, gradient[:, :, None])[:, :, 0]
    return np.where(measured, np.linalg.norm(positions[:, None, :] - anchors[None, :, :], axis=2) - absolute, 0.0)


def fit_pose_least_squares(
    dataset: dict,
    initial_anchors: np.ndarray,
    *,
    fit_home_offsets: bool,
    home_offset_prior_sigma_mm: float,
    low_anchor_z: Optional[float] = None,
    use_noise_mean: bool = True,
    extra_initial_anchors: Sequence[np.ndarray] = (),
) -> Dict[str, object]:
    """Fit anchors, and optionally home length offsets, to all points of a sweep dataset.

    A soft-L1 pass sets the residual scale; a Cauchy pass at three times that scale
    then lets isolated bad points lose their influence. Offsets carry a Gaussian prior of
    `home_offset_prior_sigma_mm`, which keeps them near zero when the poses cannot
    tell an offset from an anchor move.
    """
    machine_type = str(dataset["machine_type"])
    num_anchors, dimensions = int(dataset["num_anchors"]), int(dataset["dimensions"])
    lengths = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    if not lengths.size or not np.all(np.sum(np.isfinite(lengths), axis=1) >= dimensions + 1):
        raise ValueError("Point fit requires finite measurements on at least one point")
    num_offsets = num_anchors - dimensions if fit_home_offsets else 0
    offset_lines = list(range(num_anchors - num_offsets, num_anchors))

    def unpack(x: np.ndarray):
        anchor_vec = x[: x.size - num_offsets]
        anchors = anchor_opt_vec_to_matrix(anchor_vec, machine_type, num_anchors, dimensions, low_anchor_z)
        offsets = np.zeros(num_anchors)
        offsets[offset_lines] = x[x.size - num_offsets:]
        return anchors, offsets

    def residuals(x: np.ndarray, prior_weight: float) -> np.ndarray:
        anchors, offsets = unpack(x)
        return np.concatenate([
            consistency_residuals(anchors, lengths, offsets).ravel(),
            offsets[offset_lines] * prior_weight,
        ])

    # The same anchor bounds as the ellipse fit: they exclude mirrored layouts that
    # fit a small set of sweeps equally well.
    anchor_lb, anchor_ub = get_anchor_opt_bounds(machine_type, num_anchors, dimensions, low_anchor_z)
    bounds = (np.concatenate([anchor_lb, np.full(num_offsets, -np.inf)]),
              np.concatenate([anchor_ub, np.full(num_offsets, np.inf)]))

    def solve(start_anchors: np.ndarray):
        anchor_start = ((anchor_lb + anchor_ub) / 2.0 if start_anchors is None else
                        anchors_matrix_to_opt_vec(np.asarray(start_anchors, dtype=float), machine_type, low_anchor_z))
        start = np.concatenate([
            np.clip(anchor_start, anchor_lb, anchor_ub),
            np.zeros(num_offsets),
        ])
        x_scale = np.concatenate([np.full(start.size - num_offsets, 100.0), np.ones(num_offsets)])
        first = least_squares(residuals, start, args=(_MIN_SCALE_MM / home_offset_prior_sigma_mm,),
                              bounds=bounds, x_scale=x_scale, loss="soft_l1", f_scale=1.0)
        point_residuals = first.fun[: lengths.size].reshape(lengths.shape)[np.isfinite(lengths)]
        scale = max(1.4826 * float(np.median(np.abs(point_residuals - np.median(point_residuals)))), _MIN_SCALE_MM)
        return least_squares(residuals, first.x, args=(scale / home_offset_prior_sigma_mm,),
                             bounds=bounds, x_scale=x_scale, loss="cauchy", f_scale=3.0 * scale, xtol=1e-12, ftol=1e-12), scale

    solutions = [solve(start) for start in [initial_anchors, *extra_initial_anchors]]
    # Starts can estimate different scales. Compare them with one common Cauchy
    # objective, rather than rewarding a solution for choosing a smaller scale.
    scale = min(solution[1] for solution in solutions)
    def common_cost(solution):
        values = residuals(solution[0].x, scale / home_offset_prior_sigma_mm)
        return float(np.sum(np.log1p((values / (3.0 * scale)) ** 2)))
    converged = [solution for solution in solutions if solution[0].success]
    best, _ = min(converged or solutions, key=common_cost)
    anchors, offsets = unpack(best.x)
    per_point = np.linalg.norm(best.fun[: lengths.size].reshape(lengths.shape), axis=1)
    return {
        "anchors": anchors,
        "home_offsets_mm": offsets,
        "rms_mm": float(np.sqrt(np.mean(per_point ** 2))),
        "robust_scale_mm": float(scale),
        "point_residuals_mm": per_point,
        "num_points": int(lengths.shape[0]),
        "fit_home_offsets": bool(fit_home_offsets),
        "success": bool(best.success),
        "status": int(best.status),
        "message": str(best.message),
    }


def shift_dataset_home_offsets(dataset: dict, home_offsets_mm: Sequence[float]) -> dict:
    """Return a copy of the dataset whose length changes include the home length offsets."""
    offsets = [float(value) for value in home_offsets_mm]
    shifted = dict(dataset)
    shifted["sweeps"] = []
    for sweep in dataset.get("sweeps", []):
        fixed = [int(anchor) for anchor in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        new_sweep = dict(sweep)
        new_sweep["fixed_lengths"] = [float(value) + offsets[anchor] for anchor, value in zip(fixed, sweep["fixed_lengths"])]
        new_sweep["data_points"] = []
        for point in sweep.get("data_points", []):
            new_point = dict(point)
            for key, anchor in (("l_drive", drive), ("l_drive_mu", drive), ("l_sensor", sensor), ("l_sensor_mu", sensor)):
                if new_point.get(key) is not None:
                    new_point[key] = float(new_point[key]) + offsets[anchor]
            new_sweep["data_points"].append(new_point)
        shifted["sweeps"].append(new_sweep)
    return shifted
