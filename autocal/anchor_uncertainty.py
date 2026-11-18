"""Anchor uncertainty in mm, from refits that each leave out one whole sweep.

Fit cost cannot rank calibrations by accuracy. More poses expose more model
mismatch, so cost grows while anchors improve. In 69 distinct recorded headless
runs (HP3, HP4, HP5), the final fit cost did not track the true anchor error
(Spearman +0.05), while this spread did (+0.64). Its median ratio to the
true worst-coordinate error was 0.9-1.2 on HP3 and HP5, and every run off
by more than 10 mm reported more than 5 mm.

Bias that every sweep shares does not move a jackknife estimate. Examples are
unmodelled line stretch with uniform tension and wrong reeving geometry. HP4,
for example, keeps D.z about 4.6 mm low while its jackknife reports 1-2 mm.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares

MIN_SWEEPS = 3


def sweep_jackknife_mm(dataset: dict, anchors, *, fit_home_offsets: bool = True,
                       low_anchor_z: Optional[float] = None) -> Optional[float]:
    """Largest anchor-coordinate jackknife standard deviation, in mm.

    None means too few sweeps to leave one out. Infinity means a refit failed,
    so at least one sweep is needed to locate the anchors.
    """
    sweeps = dataset.get("sweeps") or []
    if len(sweeps) < MIN_SWEEPS:
        return None
    estimates = []
    for left_out in range(len(sweeps)):
        subset = dict(dataset, sweeps=[s for i, s in enumerate(sweeps) if i != left_out])
        try:
            fit = fit_pose_least_squares(
                subset, np.asarray(anchors, dtype=float), fit_home_offsets=fit_home_offsets,
                home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM, low_anchor_z=low_anchor_z,
            )
        except (ValueError, np.linalg.LinAlgError):
            return float("inf")
        if not fit["success"] or not np.all(np.isfinite(fit["anchors"])):
            return float("inf")
        estimates.append(fit["anchors"])
    estimates = np.stack(estimates)
    n = len(estimates)
    std = np.sqrt((n - 1) / n * np.sum((estimates - estimates.mean(axis=0)) ** 2, axis=0))
    return float(np.max(std))


def plan_anchor_uncertainty_mm(plan: dict) -> Optional[float]:
    """Jackknife uncertainty of a fitted plan, on the lengths its fit used."""
    dataset = plan.get("dataset_for_estimation") or plan.get("dataset")
    if not isinstance(dataset, dict) or plan.get("anchors") is None:
        return None
    return sweep_jackknife_mm(
        dataset, plan["anchors"],
        # The estimation dataset already includes fitted home offsets; refit them
        # per subset so each fold carries the same nuisance freedom as the full fit.
        fit_home_offsets=bool(plan.get("home_offsets_fitted", False)),
        low_anchor_z=dataset.get("_low_anchor_z"),
    )


def uncertainty_label(uncertainty_mm: Optional[float]) -> str:
    if uncertainty_mm is None or not np.isfinite(uncertainty_mm):
        return "unknown"
    if uncertainty_mm <= 1.0:
        return "good"
    if uncertainty_mm <= 3.0:
        return "usable"
    return "poor"
