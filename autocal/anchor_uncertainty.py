"""Anchor uncertainty in mm, from refits that each leave out one whole sweep.

Fit cost cannot rank calibrations by accuracy. More poses expose more model
mismatch, so cost grows while anchors improve. In 69 distinct recorded headless
runs (HP3, HP4, HP5), the final fit cost did not track the true anchor error
(Spearman +0.05), while this spread did (+0.64), and every run off by more
than 10 mm reported more than 5 mm. Those runs fitted one shared low-anchor z.

With every low z free, leaving out one of HP5's five sweeps can remove the
only constraint on one anchor coordinate (a low anchor's height or radius, or
the top anchor's height). That single fold then moves the coordinate by
several mm for structural reasons, and the spread called sub-mm calibrations
"poor". Which folds lose a coordinate is known from the poses alone: the
information the kept points carry. A fold whose information-implied standard
deviation for a coordinate exceeds SINGLE_UNIT_RATIO times the full data's,
while every other fold keeps that coordinate, is left out of that coordinate's
spread, and the coordinate is reported as resting on a single unit: that unit is
not cross-checked, because no statistic over these folds can tell its corruption
from its structural loss. Everything else keeps the classical spread. A median of
the folds was tried and rejected: it also discards the one clean fold when a
single sweep is corrupted, and called two injected 5-6 mm failures "usable". In the research session (2026-11-20
uneven low-anchor heights, E12) this rule removed 3 of 4 false "poor" labels
on 9 sub-mm HP5/HP5r runs, kept "poor" on every one of 56 single-sweep
corruptions the classical spread called poor, and changed no patience stop
or applied fit in an 84-run corpus replay.

Bias that every sweep shares does not move a jackknife estimate. Examples are
unmodelled line stretch with uniform tension and wrong reeving geometry. HP4,
for example, keeps D.z about 4.6 mm low while its jackknife reports 1-2 mm.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, dataset_line_lengths, fit_point_positions, fit_pose_least_squares

MIN_SWEEPS = 3
# Leaving out an average one of n units raises a coordinate's information-implied std
# by about sqrt(n / (n - 1)); on recorded HP5/HP5r runs ordinary folds mostly stay below
# 3.7, and a fold that alone loses a coordinate's only constraint reaches 4.3-14.
SINGLE_UNIT_RATIO = 3.0
# A newest fit more uncertain than both of these has diverged. A ratio of 10 changed
# no choice in the recorded corpus; 2-5 picked worse early fits. The floor keeps a
# near-zero early value from rejecting every later fit that is still usable.
DIVERGED_UNCERTAINTY_RATIO = 10.0
DIVERGED_UNCERTAINTY_FLOOR_MM = 3.0


def sweep_jackknife_mm(dataset: dict, anchors, *, fit_home_offsets: bool = True,
                       use_noise_mean: bool = True) -> Optional[float]:
    """Largest anchor-coordinate jackknife standard deviation, in mm (see sweep_jackknife)."""
    return sweep_jackknife(dataset, anchors, fit_home_offsets=fit_home_offsets, use_noise_mean=use_noise_mean)[0]


def sweep_jackknife(dataset: dict, anchors, *, fit_home_offsets: bool = True,
                    use_noise_mean: bool = True) -> Tuple[Optional[float], List[Tuple[int, int]]]:
    """Largest anchor-coordinate jackknife standard deviation in mm, and single-unit coordinates.

    Each refit leaves out one collection unit: a sweep, or a whole sphere mesh.
    None means too few units to leave one out. Infinity means a refit failed,
    so at least one unit is needed to locate the anchors.
    """
    sweeps = dataset.get("sweeps") or []
    groups = measurement_groups(dataset)
    if len(groups) < MIN_SWEEPS:
        return None, []
    estimates = []
    for left_out in groups:
        subset = dict(dataset, sweeps=[s for i, s in enumerate(sweeps) if i not in left_out])
        try:
            fit = fit_pose_least_squares(
                subset, np.asarray(anchors, dtype=float), fit_home_offsets=fit_home_offsets,
                home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
                use_noise_mean=use_noise_mean,
            )
        except (ValueError, np.linalg.LinAlgError):
            return float("inf"), []
        if not fit["success"] or not np.all(np.isfinite(fit["anchors"])):
            return float("inf"), []
        estimates.append(fit["anchors"])
    ratios = fold_information_ratios(dataset, anchors, groups, use_noise_mean=use_noise_mean)
    return jackknife_spread_mm(np.stack(estimates), ratios)


def jackknife_spread_mm(estimates: np.ndarray, ratios: np.ndarray) -> Tuple[float, List[Tuple[int, int]]]:
    """Classical jackknife std per coordinate, without the one fold that alone lost it.

    `ratios[i]` is fold i's information-implied std over the full data's, per coordinate.
    A coordinate that exactly one fold loses (ratio > SINGLE_UNIT_RATIO) is measured on the
    other folds when at least MIN_SWEEPS remain; it is returned as resting on one unit.
    """
    n = len(estimates)
    def classical(folds):
        return np.sqrt((len(folds) - 1) / len(folds) * np.sum((folds - folds.mean(axis=0)) ** 2, axis=0))
    spread = classical(estimates)
    single = []
    for index in np.ndindex(*estimates.shape[1:]):
        lost = ratios[(slice(None), *index)] > SINGLE_UNIT_RATIO
        if lost.sum() == 1 and n - 1 >= MIN_SWEEPS:
            spread[index] = classical(estimates[(slice(None), *index)][~lost])
            single.append(tuple(int(v) for v in index))
    return float(np.max(spread)), single


def fold_information_ratios(dataset: dict, anchors, groups, *, use_noise_mean: bool = True) -> np.ndarray:
    """Per fold and anchor coordinate: information-implied std without the unit over with it.

    Rigid range information at the full fit's poses, with the fit's home-offset prior.
    Gauge-fixed coordinates get ratio 1.
    """
    from autocal.pose_planning import point_informations
    from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec
    anchors = np.asarray(anchors, dtype=float)
    machine, (num_anchors, dimensions) = str(dataset["machine_type"]), anchors.shape
    lengths = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)
    positions, _ = fit_point_positions(anchors, lengths["lengths_mm"], np.zeros(num_anchors))
    infos = point_informations(anchors, lengths["lengths_mm"], positions, machine_type=machine, sigma_mm=1.0)
    num_offsets = num_anchors - dimensions
    num_params = infos.shape[1] - num_offsets
    prior = np.zeros(infos.shape[1:])
    prior[num_params:, num_params:] = np.eye(num_offsets) / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
    vector = anchors_matrix_to_opt_vec(anchors, machine)
    mapping = np.stack([(anchor_opt_vec_to_matrix(vector + step, machine, num_anchors, dimensions)
                         - anchor_opt_vec_to_matrix(vector - step, machine, num_anchors, dimensions)).ravel() / 2e-3
                        for step in np.eye(num_params) * 1e-3], axis=1)

    def coordinate_std(keep):
        covariance = np.linalg.pinv(infos[keep].sum(axis=0) + prior, rcond=1e-12)[:num_params, :num_params]
        return np.sqrt(np.clip(np.diag(mapping @ covariance @ mapping.T), 0.0, None)).reshape(anchors.shape)

    full = coordinate_std(np.ones(len(positions), dtype=bool))
    fixed = full <= 1e-9
    ratios = []
    for group in groups:
        ratio = coordinate_std(~np.isin(lengths["sweep_index"], list(group))) / np.where(fixed, 1.0, full)
        ratios.append(np.where(fixed, 1.0, ratio))
    return np.stack(ratios)


def plan_anchor_uncertainty(plan: dict) -> Tuple[Optional[float], List[Tuple[int, int]]]:
    """Jackknife uncertainty of a fitted plan, on the lengths its fit used, and its single-unit coordinates."""
    # Home offsets are refitted per fold from the unshifted lengths, so a left-out
    # sweep cannot influence them. Spool-corrected lengths keep the full-data spool fit.
    fit_home_offsets = bool(plan.get("home_offsets_fitted", False))
    dataset = plan.get("dataset") if fit_home_offsets else plan.get("dataset_for_estimation")
    if not isinstance(dataset, dict) or plan.get("anchors") is None:
        return None, []
    return sweep_jackknife(
        dataset, plan["anchors"], fit_home_offsets=fit_home_offsets, use_noise_mean=bool(plan.get("use_noise_mean", True)),
    )


def coordinate_names(coordinates, num_anchors: int) -> List[str]:
    """Readable names such as 'A.z' or 'I.z' for (anchor, axis) pairs."""
    letters = "ABCDI" if num_anchors == 5 else "ABCDEFGH"[:num_anchors]
    return [f"{letters[anchor]}.{'xyz'[axis]}" for anchor, axis in coordinates]


def newest_trusted_index(uncertainties_mm) -> Optional[int]:
    """Index of the newest fit to apply, given each fit's uncertainty in fitting order.

    None (too few sweeps) is acceptable; a failed jackknife (non-finite) never is.
    Returns None when no fit qualifies.
    """
    finite = [value for value in uncertainties_mm if value is not None and np.isfinite(value)]
    limit = max(DIVERGED_UNCERTAINTY_FLOOR_MM, DIVERGED_UNCERTAINTY_RATIO * min(finite)) if finite else None
    for index in reversed(range(len(uncertainties_mm))):
        value = uncertainties_mm[index]
        if value is None or (limit is not None and np.isfinite(value) and value <= limit):
            return index
    return None


def uncertainty_label(uncertainty_mm: Optional[float]) -> str:
    if uncertainty_mm is None or not np.isfinite(uncertainty_mm):
        return "unknown"
    if uncertainty_mm <= 1.0:
        return "good"
    if uncertainty_mm <= 3.0:
        return "usable"
    return "poor"
