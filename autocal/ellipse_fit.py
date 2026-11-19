"""Ellipse reference fit for fixed-line circle sweeps and sphere meshes on Hangprinters.

With two lines held, the effector moves on a circle. Along it, the squared lengths
of the drive line and of any other measured line trace an ellipse that the anchors
predict. Each point's distance to its predicted ellipses, in mm, is its misfit:
drive against the sensor line and, on HP5, drive against every accepted extra
range. Held lines are taken at their commanded lengths.

A sphere mesh holds one line, so the effector moves on a sphere and the squared
lengths of its two drives and any third measured line trace an ellipsoid; each
mesh point's distance in mm to that ellipsoid is its misfit.

Lengths are first corrected for line stretch with the force-aware model, at the
start anchors and again at the fitted ones. The robust loss and point rejection
are the point fit's (see pose_fit.py), so the two fits differ only in the misfit.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np
from scipy.optimize import least_squares

from autocal.dataset_roles import measurement_groups, mesh_metadata
from autocal.pose_fit import (REJECT_SCALES, _MIN_SCALE_MM, _REJECT_ROUNDS, _at_far_bound, _hp4_home_strain,
                              dataset_line_lengths, dataset_stretch, fit_point_positions, robust_loss)
from autocal.theoretical_ellipse import (anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec,
                                         compute_constraint_circle_3d, get_anchor_opt_bounds,
                                         parametric_to_algebraic_ellipse, squared_length_coefficients)

# A trial geometry where a sweep's held lengths cannot meet has no circle. Its
# points then cost this much each, which keeps the objective bounded.
_NO_CIRCLE_MM = 50.0
_STRETCH_PASSES = 2


def _sweep_ellipses(dataset: dict, lengths: np.ndarray) -> List[dict]:
    """Per collection unit, the conics its points must lie on.

    A sweep: its held lines and one (drive, other) pair per ellipse. A sphere
    mesh: its held line, its two drives and one other measured line per ellipsoid.
    """
    first_row = np.cumsum([0] + [len(sweep["data_points"]) for sweep in dataset["sweeps"]])
    units = []
    for group in measurement_groups(dataset):
        rows = np.concatenate([np.arange(first_row[index], first_row[index + 1]) for index in group])
        sweep = dataset["sweeps"][group[0]]
        mesh = mesh_metadata(sweep)
        if mesh is not None:
            # Rows hold different level lines, and spare ranges can be missing, so
            # points are grouped by which lines they measured: the first two free
            # lines act as drives, each further one adds an ellipsoid.
            held = int(mesh["held_anchor"])
            free = [anchor for anchor in range(lengths.shape[1]) if anchor != held]
            measured = np.isfinite(lengths[np.ix_(rows, free)]) & np.isfinite(lengths[rows, held])[:, None]
            for pattern in {tuple(row) for row in measured}:
                lines = [anchor for anchor, used in zip(free, pattern) if used]
                members = rows[np.all(measured == pattern, axis=1)]
                if len(lines) >= 3 and len(members) >= 4:
                    units.append(dict(kind="ellipsoid", rows=members, held=held, drives=lines[:2], others=lines[2:]))
            continue
        drive = int(sweep["drive_anchor"])
        others = [anchor for anchor in range(lengths.shape[1])
                  if anchor != drive and anchor not in sweep["fixed_anchors"]
                  and np.sum(np.isfinite(lengths[rows, anchor])) >= 3]
        units.append(dict(kind="ellipse", rows=rows, fixed=[int(a) for a in sweep["fixed_anchors"]],
                          drive=drive, others=others))
    return units


def _ellipsoid_misfits(anchors: np.ndarray, unit: dict, lengths: np.ndarray, norms: np.ndarray):
    """Held line h at radius R: squared lengths y_j of lines j are affine in the
    effector position p, y_j = R^2 - |A_h|^2 + |A_j|^2 + 2 p.(A_h - A_j). With
    p = A_h + R u, |u| = 1, three such lines satisfy g = |M^-1 (y - c)|^2 - R^2 = 0,
    c = R^2 + |A_j|^2 - |A_h|^2 + M A_h. R is each point's own measured held length,
    so the first-order distance divides g by its gradient over all four lengths."""
    rows, held = unit["rows"], unit["held"]
    for other in unit["others"]:
        lines = [*unit["drives"], other]
        mask = np.ones(len(rows), dtype=bool)
        radius = lengths[rows[mask], held] + norms[held]
        absolute = lengths[np.ix_(rows[mask], lines)] + norms[lines]
        matrix = 2.0 * (anchors[held] - anchors[lines])
        try:
            inverse = np.linalg.inv(matrix)
        except np.linalg.LinAlgError:
            yield rows[mask], np.full(int(mask.sum()), _NO_CIRCLE_MM)
            continue
        offset = (radius ** 2)[:, None] + (norms[lines] ** 2 - norms[held] ** 2 + matrix @ anchors[held])[None, :]
        unit_vectors = (absolute ** 2 - offset) @ inverse.T
        value = np.sum(unit_vectors ** 2, axis=1) - radius ** 2
        weights = 2.0 * unit_vectors @ inverse           # dg/dy for each line's squared length
        gradient_lines = weights * 2.0 * absolute
        # c moves with R^2 in every line: dg/dR = -2 R (sum of dg/dy) - 2 R.
        gradient_held = -2.0 * radius * np.sum(weights, axis=1) - 2.0 * radius
        gradient = np.hypot(np.linalg.norm(gradient_lines, axis=1), gradient_held)
        yield rows[mask], value / np.maximum(gradient, 1e-12)


def ellipse_misfits(anchors: np.ndarray, sweeps: List[dict], lengths: np.ndarray):
    """Signed misfit in mm of every (point, ellipse) pair, with the point index of each."""
    norms = np.linalg.norm(anchors, axis=1)
    values, points = [], []
    for sweep in sweeps:
        if sweep["kind"] == "ellipsoid":
            for rows, misfit in _ellipsoid_misfits(anchors, sweep, lengths, norms):
                points.append(rows)
                values.append(misfit)
            continue
        rows, drive = sweep["rows"], sweep["drive"]
        # Held lengths are commanded once per sweep; their per-point stretch varies by
        # hundredths of a millimetre, so the sweep mean stands for all points.
        held = [norms[a] + float(np.nanmean(lengths[rows, a])) for a in sweep["fixed"]]
        circle = compute_constraint_circle_3d(anchors, sweep["fixed"], held)
        for other in sweep["others"]:
            mask = np.isfinite(lengths[rows, other])
            points.append(rows[mask])
            if circle is None:
                values.append(np.full(int(mask.sum()), _NO_CIRCLE_MM))
                continue
            a, b, c, d, e, f = parametric_to_algebraic_ellipse(
                *squared_length_coefficients(circle, anchors[drive]),
                *squared_length_coefficients(circle, anchors[other]))
            drive_mm = lengths[rows[mask], drive] + norms[drive]
            other_mm = lengths[rows[mask], other] + norms[other]
            x, y = drive_mm ** 2, other_mm ** 2
            value = a * x * x + b * x * y + c * y * y + d * x + e * y + f
            # First-order distance in length space: divide by the gradient there.
            gradient = np.hypot((2 * a * x + b * y + d) * 2 * drive_mm, (b * x + 2 * c * y + e) * 2 * other_mm)
            values.append(value / np.maximum(gradient, 1e-12))
    return np.concatenate(values), np.concatenate(points)


def _corrected_lengths(anchors: np.ndarray, lengths: np.ndarray, offsets: np.ndarray, stretch) -> np.ndarray:
    """Recorded lengths with the line stretch at the poses these anchors and offsets imply removed."""
    if stretch is None:
        return lengths
    _, absolute = fit_point_positions(anchors, lengths, offsets, stretch=stretch)
    return np.where(np.isfinite(lengths), absolute - np.linalg.norm(anchors, axis=1) - offsets, np.nan)


def fit_ellipses(
    dataset: dict,
    initial_anchors: np.ndarray,
    *,
    low_anchor_z: Optional[float] = None,
    use_noise_mean: bool = True,
    fit_home_offsets: bool = False,
    home_offset_prior_sigma_mm: float = 5.0,
    correct_constant_force: bool = False,
) -> Dict[str, object]:
    """Fit anchors to every sweep's predicted ellipses and every mesh's ellipsoids.

    Without `fit_home_offsets` the offsets must already be applied. With it, the
    offsets the point fit would estimate (the lines beyond the dimension count,
    and HP4's equal-strain hang slack) are fitted here under the same Gaussian
    prior and the same guards: dropped when the data leaves them undetermined
    (posterior std above half the prior), implausible (beyond two prior sigmas)
    or when they push an anchor onto a far bound.

    `correct_constant_force` corrects line stretch on constant-force data too,
    which the point fit leaves to equal-strain collection on HP3/HP4.
    """
    options = (low_anchor_z, use_noise_mean, home_offset_prior_sigma_mm, correct_constant_force)
    rigid = _fit(dataset, initial_anchors, False, *options)
    if not fit_home_offsets:
        return rigid
    fit = _fit(dataset, rigid["anchors"], True, *options)
    sigma = home_offset_prior_sigma_mm
    if np.any(fit["home_offset_std_mm"] > 0.5 * sigma):
        return {**rigid, "home_offsets_skipped": "undetermined"}
    if np.any(np.abs(fit["fitted_offsets_mm"]) > 2.0 * sigma):
        return {**rigid, "home_offsets_skipped": "implausible"}
    if _at_far_bound(fit["anchors"], dataset, low_anchor_z) and not _at_far_bound(rigid["anchors"], dataset, low_anchor_z):
        return {**rigid, "home_offsets_skipped": "anchor at bound"}
    return fit


def _fit(dataset, initial_anchors, fit_home_offsets, low_anchor_z, use_noise_mean, home_offset_prior_sigma_mm,
         correct_constant_force):
    machine_type = str(dataset["machine_type"])
    num_anchors, dimensions = int(dataset["num_anchors"]), int(dataset["dimensions"])
    if not machine_type.startswith("hangprinter_") or dimensions != 3:
        raise ValueError("The ellipse reference fit supports three-dimensional Hangprinters")
    if any(len(sweep["fixed_anchors"]) != 2 for sweep in dataset["sweeps"] if mesh_metadata(sweep) is None):
        raise ValueError("The ellipse reference fit needs sweeps with exactly two held lines")
    recorded = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    stretch = dataset_stretch(dataset, constant_force=correct_constant_force)
    sweeps = _sweep_ellipses(dataset, recorded)
    lower, upper = get_anchor_opt_bounds(machine_type, num_anchors, dimensions, low_anchor_z)
    num_params = lower.size
    num_offsets = num_anchors - dimensions if fit_home_offsets else 0
    offset_lines = list(range(num_anchors - num_offsets, num_anchors))
    home_strain = _hp4_home_strain(dataset) if fit_home_offsets and low_anchor_z is None else None
    sigma = home_offset_prior_sigma_mm

    def unpack(x):
        anchors = anchor_opt_vec_to_matrix(x[:num_params], machine_type, num_anchors, dimensions, low_anchor_z)
        offsets = np.zeros(num_anchors)
        if home_strain is not None:
            strain, extra_length = home_strain
            offsets[:3] = strain * (np.linalg.norm(anchors[:3], axis=1) + extra_length)
        offsets[offset_lines] = x[num_params:]
        return anchors, offsets

    lower = np.concatenate([lower, np.full(num_offsets, -np.inf)])
    upper = np.concatenate([upper, np.full(num_offsets, np.inf)])
    x_scale = np.concatenate([np.full(num_params, 100.0), np.ones(num_offsets)])
    start = anchors_matrix_to_opt_vec(np.asarray(initial_anchors, dtype=float), machine_type, low_anchor_z)
    x = np.concatenate([np.clip(start, lower[:num_params], upper[:num_params]), np.zeros(num_offsets)])
    scale = _MIN_SCALE_MM
    for _ in range(_STRETCH_PASSES if stretch is not None else 1):
        anchors, offsets = unpack(x)
        lengths = _corrected_lengths(anchors, recorded, offsets, stretch)
        _, owner = ellipse_misfits(unpack(x)[0], sweeps, lengths)

        def residuals(vector, prior_weight, kept=None):
            anchors, offsets = unpack(vector)
            values, _ = ellipse_misfits(anchors, sweeps, lengths + offsets)
            values = values if kept is None else np.where(kept[owner], values, 0.0)
            return np.concatenate([values, offsets[offset_lines] * prior_weight])

        def point_misfit(vector):
            anchors, offsets = unpack(vector)
            values, _ = ellipse_misfits(anchors, sweeps, lengths + offsets)
            return np.sqrt(np.bincount(owner, values ** 2, minlength=len(lengths)))

        fit = least_squares(residuals, x, args=(_MIN_SCALE_MM / sigma,), bounds=(lower, upper), x_scale=x_scale,
                            loss=robust_loss("soft_l1", num_offsets), f_scale=1.0)
        kept = np.ones(len(lengths), dtype=bool)
        for _ in range(_REJECT_ROUNDS):
            used = fit.fun[:len(owner)][kept[owner]]
            scale = max(1.4826 * float(np.median(np.abs(used - np.median(used)))), _MIN_SCALE_MM)
            fit = least_squares(residuals, fit.x, args=(scale / sigma, kept), bounds=(lower, upper), x_scale=x_scale,
                                loss=robust_loss("arctan", num_offsets), f_scale=3.0 * scale)
            within = point_misfit(fit.x) <= REJECT_SCALES * scale
            if np.array_equal(within, kept) or within.mean() < 0.5:
                break
            kept = within
        x = fit.x
    anchors, offsets = unpack(x)
    per_point = point_misfit(x)
    # Posterior offset spread from the final Jacobian, priors included.
    offset_std = (np.sqrt(np.diag(np.linalg.pinv(fit.jac.T @ fit.jac)))[num_params:] * scale
                  if num_offsets else np.zeros(0))
    return {
        "anchors": anchors,
        "home_offsets_mm": offsets,
        "fitted_offsets_mm": offsets[offset_lines],
        "home_offset_std_mm": offset_std,
        "home_offsets_skipped": None,
        "rms_mm": float(np.sqrt(np.mean(per_point[kept] ** 2))),
        "robust_scale_mm": float(scale),
        "point_residuals_mm": per_point,
        "rejected_points": np.flatnonzero(~kept),
        "num_points": int(len(lengths)),
        "num_ellipses": int(sum(len(sweep["others"]) for sweep in sweeps if sweep["kind"] == "ellipse")),
        "num_ellipsoids": int(sum(len(sweep["others"]) for sweep in sweeps if sweep["kind"] == "ellipsoid")),
        "stretch_corrected": stretch is not None,
        "success": bool(fit.success),
        "message": str(fit.message),
    }
