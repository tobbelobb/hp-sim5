"""Check rough geometry before using it to control measurement forces.

This is a local observability check, not an accuracy certificate: the conservative
0.5 mm line-error floor and independent starts detect weak fits without claiming
to bound arbitrary systematic friction or stretch errors.
"""
from __future__ import annotations

import numpy as np

from autocal.active_learning import dataset_sweep_configs, generate_candidate_sweeps
from autocal.calibrate import build_anchor_initial_guess
from autocal.pose_fit import (
    HOME_OFFSET_PRIOR_SIGMA_MM, consistency_residuals, dataset_line_lengths,
    fit_pose_least_squares,
)
from autocal.pose_planning import rank_candidates_pose_information
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec

MAX_EXTRA_ROUGH_SWEEPS = 9
MAX_ROUGH_ANCHOR_STD_MM = 25.0
ROUGH_LINE_SIGMA_MM = 0.5


def assess_rough_geometry(dataset, *, low_anchor_z=None, use_noise_mean=True):
    machine = dataset["machine_type"]
    n, d = int(dataset["num_anchors"]), int(dataset["dimensions"])
    seed = build_anchor_initial_guess(dataset, machine_type=machine, base_radii_mm=None,
                                      low_anchor_z=low_anchor_z)
    if seed is None:
        raise ValueError("Rough bootstrap needs raw encoder travel to construct independent geometry seeds")
    options = dict(fit_home_offsets=False, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
                   low_anchor_z=low_anchor_z, use_noise_mean=use_noise_mean)
    fits = [fit_pose_least_squares(dataset, start, **options) for start in (seed, seed * 1.15)]
    fit = min(fits, key=lambda result: result["rms_mm"])
    anchors = fit["anchors"]
    vector = anchors_matrix_to_opt_vec(anchors, machine, low_anchor_z)
    lengths = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    inliers = fit["point_residuals_mm"] <= max(1.0, 6.0 * fit["robust_scale_mm"])
    lengths = lengths[inliers]

    def residuals(x):
        moved = anchor_opt_vec_to_matrix(x, machine, n, d, low_anchor_z)
        return consistency_residuals(moved, lengths, np.zeros(n)).ravel()

    steps = np.eye(vector.size) * 0.1
    jacobian = np.column_stack([(residuals(vector + step) - residuals(vector - step)) / 0.2
                                for step in steps])
    rank = int(np.linalg.matrix_rank(jacobian)) if jacobian.size else 0
    sigma = max(ROUGH_LINE_SIGMA_MM, 3.0 * float(fit["robust_scale_mm"]))
    max_std = (float(np.max(np.linalg.norm(np.linalg.pinv(jacobian), axis=1))) * sigma
               if rank == vector.size else float("inf"))
    restart_difference = float(np.max(np.abs(fits[0]["anchors"] - fits[1]["anchors"])))
    ready = (all(result["success"] for result in fits) and rank == vector.size
             and max_std <= MAX_ROUGH_ANCHOR_STD_MM and restart_difference <= MAX_ROUGH_ANCHOR_STD_MM
             and np.mean(inliers) >= 0.8 and fit["robust_scale_mm"] <= ROUGH_LINE_SIGMA_MM)
    return anchors, {
        "ready": bool(ready), "sweeps": len(dataset["sweeps"]), "rank": rank,
        "parameters": int(vector.size), "assumed_line_sigma_mm": sigma,
        "max_anchor_std_mm": max_std if np.isfinite(max_std) else None,
        "restart_difference_mm": restart_difference,
        "inlier_fraction": float(np.mean(inliers)), "rms_mm": float(fit["rms_mm"]),
        "converged": all(result["success"] for result in fits),
    }


def next_rough_sweep(dataset, anchors, *, low_anchor_z=None):
    travel = float(dataset["config"]["max_travel_mm"])
    if not np.isfinite(travel) or travel <= 0:
        raise ValueError("Rough bootstrap requires a positive measured max_travel_mm")
    observed = dataset_sweep_configs(dataset)
    seen = {cfg.normalized_key(tol_mm=1.0) for cfg in observed}
    candidates = generate_candidate_sweeps(
        num_anchors=int(dataset["num_anchors"]), dimensions=int(dataset["dimensions"]),
        fixed_delta_values_mm=np.linspace(-travel, travel, 9),
        machine_type=dataset["machine_type"], max_total_fixed_delta_mm=travel,
    )
    ranked = rank_candidates_pose_information(
        anchors, machine_type=dataset["machine_type"], low_anchor_z=low_anchor_z,
        observed_lengths_mm=dataset_line_lengths(dataset)["lengths_mm"],
        candidates=[cfg for cfg in candidates if cfg.normalized_key(tol_mm=1.0) not in seen],
        points_per_sweep=20, sigma_mm=ROUGH_LINE_SIGMA_MM, regularization=1e-6, top_k=1,
    )
    if not ranked:
        raise ValueError("No informative reachable rough-bootstrap sweep remains")
    return ranked[0][1]
