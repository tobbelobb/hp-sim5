"""Fit periodic sensor-force error in HP3 equal-strain point data."""
from __future__ import annotations

from copy import deepcopy

import numpy as np

from autocal.dataset_roles import mesh_metadata
from autocal.equal_strain import PROFILE_STRAIN_MODEL
from autocal.flex import FlexModel
from autocal.pose_fit import (_home_strain, consistency_residuals, dataset_line_lengths,
                              fit_point_positions, stretch_corrected_lengths)
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec

# Integer spatial harmonics of motor angle. Require agreement and a large gain
# on omitted sweeps because choosing the best of 400 periods can fit noise.
_PERIODS = np.arange(1, 401)
_MIN_CV_GAIN = 0.75


def _scan_period(residuals, geometry, harmonics):
    """Profile the local geometry columns out of a bank of sinusoidal force terms."""
    u, singular, _ = np.linalg.svd(geometry, full_matrices=False)
    if singular.size != geometry.shape[1] or singular[-1] <= 1e-6 * singular[0]:
        raise ValueError("Force scan geometry is underdetermined")
    flat = harmonics.reshape(len(residuals), -1)
    projected = (flat - u @ (u.T @ flat)).reshape(harmonics.shape)
    target = residuals - u @ (u.T @ residuals)
    normal = np.einsum("nhi,nhj->hij", projected, projected)
    rhs = np.einsum("nhi,n->hi", projected, target)
    # Ridge on force terms (1 mN-equivalent residual penalty per N).
    beta = np.linalg.solve(normal + 1e-6 * np.eye(normal.shape[-1]), rhs[..., None])[..., 0]
    return int(np.argmax(np.einsum("hi,hi->h", rhs, beta)))


def _force_period(residuals, geometry, harmonics, groups):
    full = _scan_period(residuals, geometry, harmonics)
    design = np.c_[geometry, harmonics[:, full]]
    baseline_sse = corrected_sse = 0.0
    for group in np.unique(groups):
        train = groups != group
        if _scan_period(residuals[train], geometry[train], harmonics[train]) != full:
            return None
        baseline = np.linalg.lstsq(geometry[train], -residuals[train], rcond=None)[0]
        fit, _, rank, _ = np.linalg.lstsq(design[train], -residuals[train], rcond=1e-6)
        if rank != design.shape[1]:
            return None
        baseline_sse += np.sum((residuals[~train] + geometry[~train] @ baseline)**2)
        corrected_sse += np.sum((residuals[~train] + design[~train] @ fit)**2)
    if baseline_sse <= 0 or corrected_sse > (1 - _MIN_CV_GAIN) * baseline_sse:
        return None
    design = np.c_[geometry, harmonics[:, full]]
    fit, _, rank, _ = np.linalg.lstsq(design, -residuals, rcond=1e-6)
    if rank != design.shape[1]:
        return None
    return _PERIODS[full], fit[geometry.shape[1]:].reshape(-1, 2), 1 - corrected_sse / baseline_sse


def correct_hp3_force_ripple(dataset, baseline, home_offsets_mm, *, use_noise_mean):
    """Return fit-only ranges and ripple diagnostics when the measured sweeps support a correction."""
    if (dataset["machine_type"] != "hangprinter_3" or baseline["anchors"].shape != (4, 3)
            or len(dataset["sweeps"]) < 5):
        return None
    home = _home_strain(dataset)
    if home is None or any(
        mesh_metadata(sweep) is not None
        or (sweep.get("metadata") or {}).get("sensor_force_profile_model") != PROFILE_STRAIN_MODEL
        for sweep in dataset["sweeps"]
    ):
        return None
    config = dataset["config"]
    m666 = config["m666_before_data_collection"]
    mm_per_degree = np.asarray(config.get("mm_per_degree"), dtype=float)
    if (m666.get("Q") != 0 or mm_per_degree.shape != (4,)
            or not np.all(np.isfinite(mm_per_degree) & (mm_per_degree != 0))):
        return None
    strands = np.asarray(m666.get("N"), dtype=float)
    weight = float(m666["W"]) * 9.81
    if (strands.shape != (4,) or not np.all(np.isfinite(strands) & (strands > 0))
            or not np.isfinite(weight) or weight < 0):
        return None

    points = [(sweep, point) for sweep in dataset["sweeps"] for point in sweep["data_points"]]
    if any(p.get("extra_sensor_ranges") for _, p in points):
        return None
    # The physical sensor can differ from the canonical sweep sensor on the reverse pass.
    sensors = np.array([int(p.get("source_sensor_anchor", s["sensor_anchor"])) for s, p in points])
    forces = np.array([p.get("sensor_force_n", np.nan) for _, p in points], dtype=float)
    angles = np.asarray([p.get("mu", p.get("raw_angles_deg")) if use_noise_mean and p.get("mu") is not None
                         else p.get("raw_angles_deg") for _, p in points], dtype=float)
    if (angles.shape != (len(points), 4) or not np.all((sensors >= 0) & (sensors < 3))
            or not np.all(np.isfinite(forces) & (forces > 0))):
        return None
    theta = np.deg2rad(angles[np.arange(len(points)), sensors] % 360.)
    if not np.all(np.isfinite(theta)):
        return None

    measured = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)
    lengths, groups = measured["lengths_mm"], measured["sweep_index"]
    kept = np.ones(len(lengths), dtype=bool)
    kept[baseline["rejected_points"]] = False
    if any(np.count_nonzero(kept & (sensors == motor)) < 12
           or len(np.unique(groups[kept & (sensors == motor)])) < 3 for motor in range(3)):
        return None
    anchors = baseline["anchors"]
    if len(np.unique(groups[kept])) < 5 or not np.all(np.isfinite(lengths)):
        return None

    positions, absolute = fit_point_positions(anchors, lengths, home_offsets_mm)
    ranges = positions[:, None, :] - anchors[None, :, :]
    directions = ranges / np.linalg.norm(ranges, axis=2, keepdims=True)
    # Four range measurements leave one consistency residual per 3D pose.
    normal = np.linalg.svd(directions.transpose(0, 2, 1), full_matrices=True)[2][:, -1]
    residuals = np.sum(normal * (np.linalg.norm(ranges, axis=2) - absolute), axis=1)

    model = FlexModel.from_m666(m666, num_axes=4)
    stretch = dict(model=model, forces=forces, sensors=sensors, known=np.full_like(lengths, np.nan),
                   passive=0., weight=weight)
    all_measured = np.ones(lengths.shape, dtype=bool)
    commanded = stretch_corrected_lengths(anchors, positions, absolute, all_measured, stretch)
    raised = stretch_corrected_lengths(anchors, positions, absolute, all_measured,
                                       dict(stretch, forces=forces + .01))
    derivative = -np.sum(normal * (raised - commanded), axis=1) / .01

    x = anchors_matrix_to_opt_vec(anchors, dataset["machine_type"])
    # Keep the production home-strain correction, with only the top home offset free.
    strain, extra = home
    base_low = strain * (np.linalg.norm(anchors[:3], axis=1) + extra)

    def geometry_column(step):
        moved = anchor_opt_vec_to_matrix(x + step[:-1], dataset["machine_type"], 4, 3)
        offsets = home_offsets_mm.copy()
        offsets[:3] += strain * (np.linalg.norm(moved[:3], axis=1) + extra) - base_low
        offsets[3] += step[-1]
        return (np.sum(normal * consistency_residuals(moved, lengths, offsets), axis=1) - residuals) / .01

    geometry = np.column_stack([geometry_column(step) for step in .01 * np.eye(len(x) + 1)])
    phase = theta[:, None] * _PERIODS
    trig = np.stack([np.sin(phase), np.cos(phase)], axis=-1)
    harmonics = (derivative[:, None, None, None] * np.eye(3)[sensors, None, :, None]
                 * trig[:, :, None, :]).reshape(len(points), len(_PERIODS), 6)
    fit = _force_period(residuals[kept], geometry[kept], harmonics[kept], groups[kept])
    if fit is None:
        return None
    period, coefficients, gain = fit
    delta_force = np.sum(trig[:, period - 1] * coefficients[sensors], axis=1)
    actual_forces = forces + delta_force
    if not np.all(np.isfinite(actual_forces) & (actual_forces > 0)):
        return None
    corrected = stretch_corrected_lengths(anchors, positions, absolute, all_measured,
                                          dict(stretch, forces=actual_forces))
    delta = corrected - commanded
    if not np.all(np.isfinite(delta)):
        return None
    return _adjust_ranges(dataset, delta, mm_per_degree), {
        "periods_per_turn": int(period), "cv_sse_gain": float(gain),
        "amplitude_n": np.linalg.norm(coefficients, axis=1).tolist(),
    }


def _adjust_ranges(dataset, deltas, mm_per_degree):
    """Add point-dependent flex to a copy, leaving angles and commanded forces intact."""
    corrected = deepcopy(dataset)
    changes = iter(deltas)
    for sweep in corrected["sweeps"]:
        for point in sweep["data_points"]:
            delta = next(changes)
            for role in ("drive", "sensor"):
                for key in (f"l_{role}", f"l_{role}_mu"):
                    if point.get(key) is not None:
                        point[key] += float(delta[sweep[f"{role}_anchor"]])
            raw = point.get("fixed_error_angles_deg")
            raw = [None] * len(delta) if raw is None else raw
            mean = point.get("fixed_error_angles_mu_deg")
            for anchor in sweep["fixed_anchors"]:
                correction = float(delta[anchor] / mm_per_degree[anchor])
                raw[anchor] = (raw[anchor] or 0.) + correction
                if mean is not None:
                    mean[anchor] = (mean[anchor] or 0.) + correction
            point["fixed_error_angles_deg"] = raw
    return corrected
