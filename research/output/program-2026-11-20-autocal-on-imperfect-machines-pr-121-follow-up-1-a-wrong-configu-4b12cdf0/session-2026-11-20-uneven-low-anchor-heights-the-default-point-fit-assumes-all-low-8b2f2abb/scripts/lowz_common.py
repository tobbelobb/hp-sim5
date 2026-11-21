"""Shared helpers: perturb low-anchor heights in recorded datasets and score fits.

Delta resynthesis: every recorded range keeps its simulated content (stretch, noise,
settling) and gains only the geometric change |a'-p|-|a'| - (|a-p|-|a|), with p each
point's position fitted at the true anchors. So only the anchor change is new.
"""
import copy
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from autocal._autocal_common import _normalize_dataset_point_roles, build_anchor_initial_guess
from autocal.pose_fit import (HOME_OFFSET_PRIOR_SIGMA_MM, dataset_line_lengths, dataset_stretch,
                              consistency_residuals, fit_point_positions, fit_pose_least_squares, shift_dataset_home_offsets)
from autocal.tools.regress_calibration_logs import BENCHMARK_MACHINES

REPO = Path(__file__).resolve().parents[5]
REFERENCES = REPO / "autocal" / "data" / "references"
TRUTH = {m: np.asarray(a, dtype=float) for m, a in BENCHMARK_MACHINES}


def load(name):
    return json.loads((REFERENCES / f"{name}.json").read_text())


def fit_default(dataset):
    """The sweep autocal's two-stage point fit (test_disturbed_robustness._fit), in either code version."""
    dataset = copy.deepcopy(dataset)
    _normalize_dataset_point_roles(dataset)
    seed = build_anchor_initial_guess(dataset, machine_type=dataset["machine_type"])
    opts = dict(home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    offsets = fit_pose_least_squares(dataset, seed, fit_home_offsets=True, **opts)
    shifted = shift_dataset_home_offsets(dataset, offsets["home_offsets_mm"])
    fit = fit_pose_least_squares(shifted, offsets["anchors"], fit_home_offsets=False, **opts)
    return fit, shifted


def point_positions(dataset, anchors):
    lengths = dataset_line_lengths(dataset)["lengths_mm"]
    pos, _ = fit_point_positions(anchors, lengths, np.zeros(len(anchors)), stretch=dataset_stretch(dataset))
    return pos


def resynthesise(dataset, old_anchors, new_anchors, positions=None, noise_mm=0.0, seed=0):
    """Return a copy whose ranges describe new_anchors (delta resynthesis, optional noise)."""
    old_anchors, new_anchors = np.asarray(old_anchors, float), np.asarray(new_anchors, float)
    positions = point_positions(dataset, old_anchors) if positions is None else positions
    ranges = lambda a: np.linalg.norm(a[None] - positions[:, None], axis=2) - np.linalg.norm(a, axis=1)[None]
    delta = ranges(new_anchors) - ranges(old_anchors)
    rng = np.random.default_rng(seed)
    if noise_mm:
        delta = delta + rng.normal(0.0, noise_mm, delta.shape)
    out = copy.deepcopy(dataset)
    mm_per_degree = out["config"]["mm_per_degree"]
    index = 0
    for sweep in out["sweeps"]:
        fixed = [int(a) for a in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        for point in sweep["data_points"]:
            d = delta[index]
            for key in ("fixed_error_angles_deg", "fixed_error_angles_mu_deg"):
                errors = point.get(key)
                if errors is None:
                    continue
                errors = list(errors)
                for anchor in fixed:
                    errors[anchor] = (errors[anchor] or 0.0) + d[anchor] / mm_per_degree[anchor]
                point[key] = errors
            for key, anchor in (("l_drive", drive), ("l_drive_mu", drive), ("l_sensor", sensor), ("l_sensor_mu", sensor)):
                if point.get(key) is not None:
                    point[key] = float(point[key]) + d[anchor]
            for extra in point.get("extra_sensor_ranges", []):
                for key in ("length_mm", "length_mu_mm"):
                    if extra.get(key) is not None:
                        extra[key] = float(extra[key]) + d[int(extra["anchor"])]
            index += 1
    return out


def level_frame(anchors):
    """The machine rotated about home (origin) so its first three low anchors share z, A.x = 0.

    For three low anchors this is exactly what an equal-height fit of rigid data returns.
    """
    a = np.asarray(anchors, float)
    low = a[:-1]
    normal = np.cross(low[1] - low[0], low[2] - low[0])
    normal /= np.linalg.norm(normal)
    normal *= np.sign(normal[2])
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(normal, z), float(normal @ z)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rot = np.eye(3) + vx + vx @ vx / (1.0 + c)
    out = a @ rot.T
    yaw = np.arctan(out[0, 0] / out[0, 1])  # rotate about z so A has x = 0 (A.y < 0)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return out @ rz.T


def rotation_aligned(fitted, truth):
    """Fitted anchors after the best rotation about home onto truth (Kabsch, no translation)."""
    u, _, vt = np.linalg.svd(np.asarray(fitted).T @ np.asarray(truth))
    d = np.sign(np.linalg.det(u @ vt))
    rot = u @ np.diag([1, 1, d]) @ vt
    return np.asarray(fitted) @ rot


def anchor_error(fitted, truth):
    return float(np.max(np.linalg.norm(np.asarray(fitted) - np.asarray(truth), axis=1)))


def tilt_deg(anchors):
    """Angle between the low-anchor best plane normal and z."""
    low = np.asarray(anchors)[:-1]
    centred = low - low.mean(axis=0)
    normal = np.linalg.svd(centred)[2][-1]
    return float(np.degrees(np.arccos(abs(normal[2]))))


def level_frame_ls(anchors):
    """Rotate about home so the low anchors' least-squares plane is horizontal, A.x = 0."""
    a = np.asarray(anchors, float); low = a[:-1]
    normal = np.linalg.svd(low - low.mean(0))[2][-1]; normal *= np.sign(normal[2])
    z = np.array([0, 0, 1.0]); v, c = np.cross(normal, z), normal[2]
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    out = a @ (np.eye(3) + vx + vx @ vx / (1 + c)).T
    yaw = np.arctan(out[0, 0] / out[0, 1]); cz, sz = np.cos(yaw), np.sin(yaw)
    return out @ np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]).T


def warp_basis(anchors):
    low = np.asarray(anchors)[:-1]
    m = np.column_stack([np.ones(len(low)), low[:, 0], low[:, 1]])
    q = np.linalg.svd(m, full_matrices=True)[0][:, 3:]  # orthogonal complement
    return q  # (n_low, k)


def refit(dataset, start, free, offset_sigma_mm=5.0):
    """Plain least-squares refit in the level gauge; `free` adds the warp modes (warp_basis).

    Both models also correct the last n-d home offsets (Gaussian prior), so neither inherits
    offsets that the equal-z fit bent to absorb a wrong height.
    Returns anchors, range cost (prior excluded), number of ranges, number of parameters, k.
    """
    n, d = start.shape; lengths = dataset_line_lengths(dataset)["lengths_mm"]; stretch = dataset_stretch(dataset)
    q = warp_basis(start) if free else np.zeros((n - 1, 0)); k = q.shape[1]; no = n - d
    na = 5 + 2 * (n - 2)

    def unpack(x):
        a = np.zeros((n, 3)); a[0, 1] = x[0]; a[:-1, 2] = x[1]
        a[1:-1, :2] = x[2:2 + 2 * (n - 2)].reshape(-1, 2); a[-1] = x[2 + 2 * (n - 2):na]
        a[:-1, 2] += q @ x[na:na + k]
        off = np.zeros(n); off[n - no:] = x[na + k:]
        return a, off

    def res(x):
        a, off = unpack(x)
        return np.r_[consistency_residuals(a, lengths, off, stretch=stretch).ravel(), off[n - no:] * 0.02 / offset_sigma_mm]

    x0 = np.r_[start[0, 1], start[:-1, 2].mean(), start[1:-1, :2].ravel(), start[-1], np.zeros(k + no)]
    sol = least_squares(res, x0, x_scale=np.r_[np.full(na + k, 100.0), np.ones(no)])
    fun = sol.fun[:-no]
    return unpack(sol.x)[0], float(fun @ fun), int(np.isfinite(lengths).sum()), sol.x.size, k
