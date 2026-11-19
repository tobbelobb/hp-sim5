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
from autocal.pose_fit import (REJECT_SCALES, _MIN_SCALE_MM, _REJECT_ROUNDS, dataset_line_lengths,
                              dataset_stretch, fit_point_positions, robust_loss)
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
            held, drives = int(mesh["held_anchor"]), [int(axis) for axis in mesh["drive_anchors"]]
            others = [anchor for anchor in range(lengths.shape[1]) if anchor != held and anchor not in drives
                      and np.sum(np.isfinite(lengths[rows, anchor])) >= 4]
            units.append(dict(kind="ellipsoid", rows=rows, held=held, drives=drives, others=others))
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
        mask = np.all(np.isfinite(lengths[np.ix_(rows, [held, *lines])]), axis=1)
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


def _corrected_lengths(dataset: dict, anchors: np.ndarray, lengths: np.ndarray, stretch) -> np.ndarray:
    if stretch is None:
        return lengths
    _, absolute = fit_point_positions(anchors, lengths, np.zeros(anchors.shape[0]), stretch=stretch)
    return np.where(np.isfinite(lengths), absolute - np.linalg.norm(anchors, axis=1), np.nan)


def fit_ellipses(
    dataset: dict,
    initial_anchors: np.ndarray,
    *,
    low_anchor_z: Optional[float] = None,
    use_noise_mean: bool = True,
) -> Dict[str, object]:
    """Fit anchors to every sweep's predicted ellipses. Home offsets must already be applied."""
    machine_type = str(dataset["machine_type"])
    num_anchors, dimensions = int(dataset["num_anchors"]), int(dataset["dimensions"])
    if not machine_type.startswith("hangprinter_") or dimensions != 3:
        raise ValueError("The ellipse reference fit supports three-dimensional Hangprinters")
    if any(len(sweep["fixed_anchors"]) != 2 for sweep in dataset["sweeps"] if mesh_metadata(sweep) is None):
        raise ValueError("The ellipse reference fit needs sweeps with exactly two held lines")
    recorded = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    stretch = dataset_stretch(dataset)
    sweeps = _sweep_ellipses(dataset, recorded)
    lower, upper = get_anchor_opt_bounds(machine_type, num_anchors, dimensions, low_anchor_z)
    matrix = lambda x: anchor_opt_vec_to_matrix(x, machine_type, num_anchors, dimensions, low_anchor_z)
    x = np.clip(anchors_matrix_to_opt_vec(np.asarray(initial_anchors, dtype=float), machine_type, low_anchor_z),
                lower, upper)
    for _ in range(_STRETCH_PASSES if stretch is not None else 1):
        lengths = _corrected_lengths(dataset, matrix(x), recorded, stretch)
        _, owner = ellipse_misfits(matrix(x), sweeps, lengths)

        def residuals(vector, kept=None):
            values, _ = ellipse_misfits(matrix(vector), sweeps, lengths)
            return values if kept is None else np.where(kept[owner], values, 0.0)

        def point_misfit(vector):
            values, _ = ellipse_misfits(matrix(vector), sweeps, lengths)
            return np.sqrt(np.bincount(owner, values ** 2, minlength=len(lengths)))

        fit = least_squares(residuals, x, bounds=(lower, upper), x_scale=100.0,
                            loss=robust_loss("soft_l1", 0), f_scale=1.0)
        kept = np.ones(len(lengths), dtype=bool)
        for _ in range(_REJECT_ROUNDS):
            used = fit.fun[kept[owner]]
            scale = max(1.4826 * float(np.median(np.abs(used - np.median(used)))), _MIN_SCALE_MM)
            fit = least_squares(residuals, fit.x, args=(kept,), bounds=(lower, upper), x_scale=100.0,
                                loss=robust_loss("arctan", 0), f_scale=3.0 * scale)
            within = point_misfit(fit.x) <= REJECT_SCALES * scale
            if np.array_equal(within, kept) or within.mean() < 0.5:
                break
            kept = within
        x = fit.x
    per_point = point_misfit(x)
    return {
        "anchors": matrix(x),
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
