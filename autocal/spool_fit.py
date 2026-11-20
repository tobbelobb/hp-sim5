"""Spool radii and buildup factors fitted with the point fit, from raw motor angles.

Every length is recomputed from the recorded motor angle with the firmware spool
formula, so the point fit's consistency residuals judge the spool model directly.
A uniform scale of anchors and radii changes the ranges only through the buildup
curvature, so a global radius is first profiled on a grid with anchors refitted at
each value, and then refined jointly with the anchors.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, _fit_points, dataset_line_lengths
from autocal.spool_model import build_spool_model_params, dataset_with_modeled_lengths

_RADIUS_GRID_POINTS = 13


@dataclass(frozen=True)
class SpoolTerms:
    """Spool parameters inside the point fit: start, bounds, scale, lengths and priors."""
    start: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    x_scale: np.ndarray
    lengths: Callable[[np.ndarray], np.ndarray]
    prior: Callable[[np.ndarray], np.ndarray]
    num_prior: int


def spool_lengths_mm(theta_deg, radius_mm, buildup, gearing, advantage, lines_per_spool):
    """Firmware line-position change for each motor angle; see spool_model.WinchSpoolModel."""
    degrees_per_unit_times_r = gearing * advantage * 360.0 / (2.0 * np.pi)
    k2 = -(advantage * lines_per_spool) * buildup
    curved = np.abs(k2) > 1e-12
    k2_safe = np.where(curved, k2, 1.0)
    term = theta_deg * k2_safe / (2.0 * degrees_per_unit_times_r) + radius_mm
    return np.where(curved, (term * term - radius_mm * radius_mm) / k2_safe,
                    theta_deg * radius_mm / degrees_per_unit_times_r)


def _motor_angles(dataset: dict, use_noise_mean: bool) -> np.ndarray:
    num_anchors = int(dataset["num_anchors"])
    rows = []
    for sweep in dataset["sweeps"]:
        for point in sweep["data_points"]:
            mean = point.get("mu") if use_noise_mean else None
            angles = mean if isinstance(mean, list) and len(mean) == num_anchors else point["raw_angles_deg"]
            rows.append(np.asarray(angles, dtype=float))
    return np.asarray(rows)


def fit_spools(
    dataset: dict,
    initial_anchors: np.ndarray,
    *,
    fit_radii: str,
    fit_buildup: str,
    base_radii_mm: Sequence[float],
    buildup_factor: Sequence[float],
    gearing: Sequence[float],
    mechanical_advantage: Sequence[float],
    lines_per_spool: Sequence[float],
    radius_bounds: Tuple[np.ndarray, np.ndarray],
    buildup_bounds: Tuple[np.ndarray, np.ndarray],
    radius_prior_sigma_mm: Optional[float] = None,
    buildup_prior_sigma: Optional[float] = None,
    initial_radii_mm: Optional[Sequence[float]] = None,
    initial_buildup_factor: Optional[Sequence[float]] = None,
    low_anchor_z: Optional[float] = None,
    use_noise_mean: bool = True,
) -> Dict[str, object]:
    """Fit anchors with spool radii ("global"/"per-anchor") and/or buildup factors.

    Returns the anchors, radii, buildup factors, a SpoolModelParams and the dataset
    with lengths rewritten by the fitted spool model.
    """
    num_anchors = int(dataset["num_anchors"])
    base_r = np.asarray(base_radii_mm, dtype=float)
    base_b = np.asarray(buildup_factor, dtype=float)
    gearing, advantage, lines = (np.asarray(v, dtype=float) for v in (gearing, mechanical_advantage, lines_per_spool))
    theta = _motor_angles(dataset, use_noise_mean)
    measured = np.isfinite(dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"])
    num_r = {"off": 0, "global": 1}.get(fit_radii, num_anchors)
    num_b = {"off": 0, "global": 1}.get(fit_buildup, num_anchors)
    r_lower, r_upper = (np.asarray(v, dtype=float).reshape(-1)[:num_r] for v in radius_bounds)
    b_lower, b_upper = (np.asarray(v, dtype=float).reshape(-1)[:num_b] for v in buildup_bounds)
    r_start = np.asarray(initial_radii_mm if initial_radii_mm is not None else base_r, dtype=float)
    b_start = np.asarray(initial_buildup_factor if initial_buildup_factor is not None else base_b, dtype=float)
    r_centre = np.full(num_r, np.max(base_r)) if num_r == 1 else base_r[:num_r]
    b_centre = np.full(num_b, np.median(base_b)) if num_b == 1 else base_b[:num_b]

    def spool_values(x: np.ndarray, radii=None):
        r = np.broadcast_to(x[:num_r], (num_anchors,)) if num_r else base_r
        b = np.broadcast_to(x[num_r:], (num_anchors,)) if num_b else base_b
        return (np.asarray(r if radii is None else radii, dtype=float), np.asarray(b, dtype=float))

    def terms(fixed_radius: Optional[float] = None) -> SpoolTerms:
        free_r = 0 if fixed_radius is not None else num_r
        def lengths(x):
            r, b = spool_values(np.concatenate([np.full(num_r, fixed_radius), x]) if fixed_radius is not None else x)
            return np.where(measured, spool_lengths_mm(theta, r[None], b[None], gearing, advantage, lines), np.nan)
        priors = []
        if free_r and radius_prior_sigma_mm:
            priors.append(lambda x: (x[:num_r] - r_centre) / radius_prior_sigma_mm)
        if num_b and buildup_prior_sigma:
            priors.append(lambda x: (x[free_r:] - b_centre) / buildup_prior_sigma)
        start_r = (np.full(num_r, np.max(r_start)) if num_r == 1 else r_start[:num_r])[:free_r]
        start_b = (np.full(num_b, np.median(b_start)) if num_b == 1 else b_start[:num_b])
        return SpoolTerms(
            start=np.concatenate([start_r, start_b]),
            lower=np.concatenate([r_lower[:free_r], b_lower]), upper=np.concatenate([r_upper[:free_r], b_upper]),
            x_scale=np.concatenate([np.ones(free_r), np.full(num_b, 0.1)]),
            lengths=lengths, prior=lambda x: np.concatenate([p(x) for p in priors]) if priors else np.zeros(0),
            num_prior=(num_r if free_r and radius_prior_sigma_mm else 0) + (num_b if num_b and buildup_prior_sigma else 0))

    options = dict(fit_home_offsets=False, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
                   low_anchor_z=low_anchor_z, use_noise_mean=use_noise_mean, extra_initial_anchors=())
    start = np.asarray(initial_anchors, dtype=float)
    if num_r == 1:
        # Anchors scale with the radius; profile it before refining everything jointly.
        reference = float(np.max(r_start))
        base_fit = _fit_points(dataset, start, spool=terms(reference), **options)
        profile = []
        for radius in np.linspace(r_lower[0], r_upper[0], _RADIUS_GRID_POINTS):
            fit = _fit_points(dataset, base_fit["anchors"] * radius / reference, spool=terms(float(radius)), **options)
            # The median point misfit needs no common scale and ignores disturbed points.
            profile.append((float(np.median(fit["point_residuals_mm"])), float(radius), fit["anchors"]))
        _, best_radius, start = min(profile, key=lambda item: item[0])
        r_start = np.full(num_anchors, best_radius)
    fit = _fit_points(dataset, start, spool=terms(), **options)
    radii, buildup = spool_values(fit["spool_x"])
    params = build_spool_model_params(
        dataset, base_radii_mm=base_r, modeled_radii_mm=radii, modeled_buildup_factor=buildup,
        spool_to_motor_gearing_factor=gearing, mechanical_advantage=advantage, lines_per_spool=lines,
        base_buildup_factor=np.zeros(num_anchors), theta0_mode="zero")
    return {**fit, "radii_mm": radii, "buildup_factor": buildup, "spool_params": params,
            "dataset": dataset_with_modeled_lengths(dataset, params)}
