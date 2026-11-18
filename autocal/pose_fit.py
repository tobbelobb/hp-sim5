"""Point-based least-squares anchor fit, kept beside the ellipse fitter for comparison.

Every data point holds one length change per line. A rigid machine with n lines
and d effector coordinates leaves n - d consistency conditions on those lengths.
This fit moves the anchors until all points satisfy them, without assuming that a
sweep traces an ellipse.

It can also fit home length offsets. The encoder origin is taken with some lines
slack while every measurement is taken under tension, so the lengths at the origin
do not describe one effector position. Of the n possible per-line offsets, d are
equivalent to moving the home position, which leaves n - d real ones. They are
placed on the last n - d lines. HP4 equal-strain data can also use configured
compliance to estimate the ABC offsets that set the physical home reference.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import least_squares

from autocal.flex import FlexModel

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


def uses_range_fit(dataset: dict) -> bool:
    return (str(dataset.get("machine_type", "")).startswith("hangprinter_")
            and int(dataset["num_anchors"]) > int(dataset["dimensions"]) + 1)


def has_extra_sensor_data(dataset: dict) -> bool:
    return any(point.get("extra_sensor_ranges") for sweep in dataset.get("sweeps", [])
               for point in sweep.get("data_points", []))


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
            for extra in point.get("extra_sensor_ranges", []):
                if extra.get("accepted") is True:
                    row[int(extra["anchor"])] = _point_value(extra, "length_mu_mm", "length_mm", use_noise_mean)
            lengths.append(row)
            sweep_index.append(index)
    return {
        "lengths_mm": np.asarray(lengths, dtype=float).reshape(-1, num_anchors),
        "sweep_index": np.asarray(sweep_index, dtype=int),
    }


def dataset_stretch(dataset: dict):
    """Recorded sensor loads plus firmware compliance for per-pose range correction.

    Static equilibrium and every commanded torque determine the line loads.
    A rejected extra range still contributes its commanded force.
    Historical constant-force records without extra load evidence retain the rigid fit.
    """
    from autocal.equal_strain import is_equal_strain
    # Only redundant machines use this range model as their primary objective.
    # Ellipse fits still rely on equal-strain collection and rigid home offsets.
    if (int(dataset["num_anchors"]) <= int(dataset["dimensions"]) + 1
            or not (is_equal_strain(dataset) or has_extra_sensor_data(dataset))):
        return None
    config = dataset.get("config") or {}
    m666 = config.get("m666_before_data_collection") or {}
    model = FlexModel.from_m666(m666, num_axes=int(dataset["num_anchors"]))
    points = [(sweep, point) for sweep in dataset.get("sweeps", []) for point in sweep.get("data_points", [])]
    if model is None or not points or not all(isinstance(point.get("sensor_force_n"), (int, float))
                                              and np.isfinite(point["sensor_force_n"]) for _, point in points):
        return None
    sensors = np.array([int(point.get("source_sensor_anchor", sweep["sensor_anchor"])) for sweep, point in points])
    known = np.full((len(points), int(dataset["num_anchors"])), np.nan)
    for index, (_, point) in enumerate(points):
        for extra in point.get("extra_sensor_ranges", []):
            force = float(extra["force_n"])
            if not np.isfinite(force) or force <= 0:
                raise ValueError("Extra sensor requires a positive finite recorded force")
            known[index, int(extra["anchor"])] = force
    return dict(model=model, sensors=sensors, known=known,
                forces=np.array([point["sensor_force_n"] for _, point in points]),
                weight=float(m666.get("W", 0.)) * 9.81,
                passive=float((config.get("force_tuning") or {}).get("force_low_n", 0.)))


def stretch_corrected_lengths(anchors, positions, absolute, measured, stretch):
    directions = anchors[None, :, :] - positions[:, None, :]
    directions /= np.maximum(np.linalg.norm(directions, axis=2), 1e-9)[:, :, None]
    known = np.where(measured, np.nan, stretch["passive"])
    known = np.where(np.isfinite(stretch["known"]), stretch["known"], known)
    known[np.arange(len(positions)), stretch["sensors"]] = stretch["forces"]
    selectors = np.eye(anchors.shape[0])[None, :, :] * np.isfinite(known)[:, :, None]
    matrix = np.concatenate([directions.transpose(0, 2, 1), selectors], axis=1)
    load = np.zeros((len(positions), anchors.shape[1]))
    load[:, -1] = stretch["weight"]
    # Trial geometries can make the static system singular during fitting.
    # A tiny ridge keeps their residual finite without changing reachable loads.
    transpose = matrix.transpose(0, 2, 1)
    normal = transpose @ matrix + 1e-12 * np.eye(matrix.shape[-1])
    rhs = transpose @ np.concatenate([load, np.nan_to_num(known)], axis=1)[:, :, None]
    tensions = np.linalg.solve(normal, rhs)[:, :, 0]
    return np.column_stack([stretch["model"].corrected_distance_mm(absolute[:, axis], tensions[:, axis], axis=axis)
                            for axis in range(anchors.shape[0])])


def consistency_residuals(anchors: np.ndarray, lengths_mm: np.ndarray, home_offsets_mm: np.ndarray, *, stretch=None) -> np.ndarray:
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
        corrected = (absolute if stretch is None else
                     stretch_corrected_lengths(anchors, positions, absolute, measured, stretch))
        misfit = np.where(measured, distance - corrected, 0.0)
        normal = np.einsum("pni,pnj->pij", direction, direction) + damping
        gradient = np.einsum("pni,pn->pi", direction, misfit)
        positions = positions - np.linalg.solve(normal, gradient[:, :, None])[:, :, 0]
    if stretch is not None:
        absolute = stretch_corrected_lengths(anchors, positions, absolute, measured, stretch)
    return np.where(measured, np.linalg.norm(positions[:, None, :] - anchors[None, :, :], axis=2) - absolute, 0.0)


def _hp4_home_strain(dataset: dict) -> Optional[tuple[float, np.ndarray]]:
    """ABC hang slack at encoder zero. Approximate their measurement strain by T/EA."""
    from autocal.equal_strain import is_equal_strain

    if dataset["machine_type"] != "hangprinter_4" or not is_equal_strain(dataset):
        return None
    m666 = (dataset.get("config") or {}).get("m666_before_data_collection") or {}
    targets = [(s.get("metadata") or {}).get("strand_tension_n") for s in dataset["sweeps"]]
    try:
        targets = np.asarray(targets, dtype=float)
        stiffness = float(m666["S"])
        advantage = np.broadcast_to(np.asarray(m666["U"], dtype=float), (4,))
        guy = np.broadcast_to(np.asarray(m666["Y"], dtype=float), (4,))
    except (KeyError, TypeError, ValueError):
        return None
    if not (np.all(np.isfinite(targets) & (targets > 0)) and np.allclose(targets, targets[0])
            and np.isfinite(stiffness) and stiffness > 0
            and np.all(np.isfinite(advantage) & (advantage > 0))
            and np.all(np.isfinite(guy) & (guy >= 0))):
        return None
    return targets[0] / stiffness, guy[:3] / advantage[:3]


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
    stretch = dataset_stretch(dataset)
    num_offsets = num_anchors - dimensions if fit_home_offsets else 0
    offset_lines = list(range(num_anchors - num_offsets, num_anchors))

    home_strain = _hp4_home_strain(dataset) if fit_home_offsets and low_anchor_z is None else None

    def unpack(x: np.ndarray):
        anchor_vec = x[: x.size - num_offsets]
        anchors = anchor_opt_vec_to_matrix(anchor_vec, machine_type, num_anchors, dimensions, low_anchor_z)
        offsets = np.zeros(num_anchors)
        if home_strain is not None:
            strain, extra_length = home_strain
            offsets[:3] = strain * (np.linalg.norm(anchors[:3], axis=1) + extra_length)
        offsets[offset_lines] = x[x.size - num_offsets:]
        return anchors, offsets

    def residuals(x: np.ndarray, prior_weight: float, correct_stretch: bool = True) -> np.ndarray:
        anchors, offsets = unpack(x)
        return np.concatenate([
            consistency_residuals(anchors, lengths, offsets, stretch=stretch if correct_stretch else None).ravel(),
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
        if stretch is not None:
            # Far trial geometries can have singular static force systems. First
            # solve the well-behaved rigid problem, then refine its nearby fit.
            rigid = least_squares(residuals, start,
                args=(_MIN_SCALE_MM / home_offset_prior_sigma_mm, False),
                bounds=bounds, x_scale=x_scale, loss="soft_l1", f_scale=1.0)
            start = rigid.x
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
        "stretch_corrected": stretch is not None,
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
            if "extra_sensor_ranges" in point:
                new_point["extra_sensor_ranges"] = [dict(extra) for extra in point["extra_sensor_ranges"]]
                for extra in new_point["extra_sensor_ranges"]:
                    for key in ("length_mm", "length_mu_mm"):
                        if extra.get(key) is not None:
                            extra[key] = float(extra[key]) + offsets[int(extra["anchor"])]
            new_sweep["data_points"].append(new_point)
        shifted["sweeps"].append(new_sweep)
    return shifted
