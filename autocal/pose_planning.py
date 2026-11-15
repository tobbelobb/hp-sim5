"""Sweep ranking by the information each candidate adds to the point-based fit.

A candidate sweep is scored by the effector positions it would visit, not by the
ellipse it would trace. Home length offsets are part of the parameter set, so a
candidate that leaves them indistinguishable from an anchor move scores low. For
a Hangprinter that favours sweeps with a shortened D line: with D at one length
every position lies on a single sphere around the D anchor.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from autocal.active_learning import SweepConfig
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, consistency_residuals
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec

_CIRCLE_SAMPLES = 360
# A position counts as reachable while each of the sweep's lines carries at least
# this share of the largest pretension. 0.25 reproduces where recorded HP3 sweeps turn.
_MIN_PRETENSION_SHARE = 0.25
_FD_STEP_MM = 0.5


def _fixed_length_circle(anchors: np.ndarray, cfg: SweepConfig) -> np.ndarray:
    """Positions that keep the sweep's fixed lines at their lengths, as a closed curve."""
    dimensions = anchors.shape[1]
    centres = anchors[list(cfg.fixed_anchors)]
    radii = np.linalg.norm(centres, axis=1) + np.asarray(cfg.fixed_deltas_mm, dtype=float)
    angles = np.linspace(-np.pi, np.pi, _CIRCLE_SAMPLES, endpoint=False)
    if dimensions == 2:
        if radii[0] <= 0.0:
            return np.zeros((0, 2))
        return centres[0] + radii[0] * np.stack([np.cos(angles), np.sin(angles)], axis=1)
    between = centres[1] - centres[0]
    distance = float(np.linalg.norm(between))
    axis = between / distance
    along = (distance ** 2 + radii[0] ** 2 - radii[1] ** 2) / (2.0 * distance)
    radius_squared = radii[0] ** 2 - along ** 2
    if np.any(radii <= 0.0) or radius_squared <= 0.0:
        return np.zeros((0, 3))
    first = np.cross(axis, np.eye(3)[int(np.argmin(np.abs(axis)))])
    first /= np.linalg.norm(first)
    second = np.cross(axis, first)
    return (centres[0] + along * axis
            + np.sqrt(radius_squared) * (np.cos(angles)[:, None] * first + np.sin(angles)[:, None] * second))


def predict_sweep_positions(anchors: np.ndarray, cfg: SweepConfig, points: int) -> np.ndarray:
    """Effector positions a sweep would visit: where its fixed, drive and sensor lines can all pull."""
    anchors = np.asarray(anchors, dtype=float)
    curve = _fixed_length_circle(anchors, cfg)
    if curve.shape[0] == 0:
        return curve
    lines = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
    to_anchor = anchors[lines][None, :, :] - curve[:, None, :]
    directions = to_anchor / np.linalg.norm(to_anchor, axis=2)[:, :, None]
    # The pretension is the one combination of line pulls that cancels out.
    pretension = np.linalg.svd(np.transpose(directions, (0, 2, 1)))[2][:, -1, :]
    pretension *= np.sign(np.sum(pretension, axis=1))[:, None]
    reachable = np.min(pretension, axis=1) > _MIN_PRETENSION_SHARE * np.max(pretension, axis=1)
    # Start in the reachable stretch nearest home and stay on it.
    order = np.roll(np.arange(curve.shape[0]), -int(np.argmin(np.where(reachable, np.inf, 0.0))))
    reachable_ordered = reachable[order]
    if not np.any(reachable_ordered):
        return np.zeros((0, anchors.shape[1]))
    home_distance = np.where(reachable_ordered, np.linalg.norm(curve[order], axis=1), np.inf)
    nearest = int(np.argmin(home_distance))
    start, end = nearest, nearest
    while start > 0 and reachable_ordered[start - 1]:
        start -= 1
    while end < reachable_ordered.size - 1 and reachable_ordered[end + 1]:
        end += 1
    stretch = order[start:end + 1]
    if stretch.size < points:
        return np.zeros((0, anchors.shape[1]))
    return curve[stretch[np.round(np.linspace(0, stretch.size - 1, points)).astype(int)]]


def pose_information(
    anchors: np.ndarray,
    lengths_mm: np.ndarray,
    *,
    machine_type: str,
    low_anchor_z: Optional[float],
    sigma_mm: float,
) -> np.ndarray:
    """Information the given length changes carry about the anchors and the home length offsets."""
    anchors = np.asarray(anchors, dtype=float)
    num_anchors, dimensions = anchors.shape
    num_offsets = num_anchors - dimensions
    anchor_vec = anchors_matrix_to_opt_vec(anchors, machine_type, low_anchor_z)

    def residuals(step: np.ndarray) -> np.ndarray:
        moved = anchor_opt_vec_to_matrix(anchor_vec + step[:anchor_vec.size], machine_type, num_anchors,
                                         dimensions, low_anchor_z)
        offsets = np.zeros(num_anchors)
        offsets[num_anchors - num_offsets:] = step[anchor_vec.size:]
        return consistency_residuals(moved, lengths_mm, offsets).ravel()

    size = anchor_vec.size + num_offsets
    jacobian = np.zeros((lengths_mm.size, size))
    for index in range(size):
        step = np.zeros(size)
        step[index] = _FD_STEP_MM
        jacobian[:, index] = (residuals(step) - residuals(-step)) / (2.0 * _FD_STEP_MM)
    return jacobian.T @ jacobian / sigma_mm ** 2


def rank_candidates_pose_information(
    anchors: np.ndarray,
    *,
    machine_type: str,
    low_anchor_z: Optional[float],
    observed_lengths_mm: np.ndarray,
    candidates: Sequence[SweepConfig],
    points_per_sweep: int,
    sigma_mm: float,
    regularization: float,
    top_k: int,
) -> List[Tuple[float, SweepConfig]]:
    """Rank candidates by the gain in log-determinant of the point-fit information."""
    anchors = np.asarray(anchors, dtype=float)
    num_anchors, dimensions = anchors.shape
    num_offsets = num_anchors - dimensions
    home_lengths = np.linalg.norm(anchors, axis=1)
    options = dict(machine_type=machine_type, low_anchor_z=low_anchor_z, sigma_mm=sigma_mm)
    base = pose_information(anchors, observed_lengths_mm, **options)
    prior = np.full(base.shape[0], float(regularization))
    prior[base.shape[0] - num_offsets:] += 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
    base = base + np.diag(prior)
    base_logdet = np.linalg.slogdet(base)[1]

    scored: List[Tuple[float, SweepConfig]] = []
    for cfg in candidates:
        positions = predict_sweep_positions(anchors, cfg, points_per_sweep)
        if positions.shape[0] == 0:
            continue
        lengths = np.linalg.norm(anchors[None, :, :] - positions[:, None, :], axis=2) - home_lengths
        gain = np.linalg.slogdet(base + pose_information(anchors, lengths, **options))[1] - base_logdet
        if np.isfinite(gain):
            scored.append((float(gain), cfg))
    scored.sort(key=lambda item: item[0], reverse=True)
    return scored[:top_k]
