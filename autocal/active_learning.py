from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations, product
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from autocal.ellipse_cost import canonicalize_geometry
from autocal.sweep_types import MachineConfig, MachineType
from autocal.theoretical_ellipse import (
    anchors_matrix_to_vec,
    anchors_vec_to_matrix,
    get_anchor_bounds,
    predict_ellipse_geometry,
)

GeometryWeights = Tuple[float, float, float]


@dataclass(frozen=True)
class SweepConfig:
    """Minimal sweep config for planning."""

    fixed_anchors: Tuple[int, ...]
    fixed_deltas_mm: Tuple[float, ...]
    drive_anchor: int
    sensor_anchor: int

    def normalized_key(self, *, tol_mm: float = 1e-3) -> Tuple[object, ...]:
        tol = float(tol_mm)
        if not np.isfinite(tol) or tol <= 0.0:
            tol = 1e-3
        fixed_pairs = tuple(
            (int(idx), float(np.round(float(delta) / tol) * tol))
            for idx, delta in zip(self.fixed_anchors, self.fixed_deltas_mm)
        )
        return (*fixed_pairs, ("d", int(self.drive_anchor)), ("s", int(self.sensor_anchor)))


def _wrap_angle_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    a = float(angle)
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def l2_scale_for_machine(machine_type: str, num_anchors: int, dimensions: int) -> float:
    """Scale factor for squared-length plane normalization (matches ellipse_cost.py)."""
    _lb, ub = get_anchor_bounds(machine_type)
    ub_mat = np.asarray(ub, dtype=float).reshape(int(num_anchors), int(dimensions))
    max_anchor_norm = float(np.max(np.linalg.norm(ub_mat, axis=1))) if ub_mat.size else 1.0
    return float((2.0 * max_anchor_norm) ** 2)


def _canonical_drive_sensor_pair(
    anchor_a: int, anchor_b: int, forbidden_sensors: Iterable[int]
) -> Optional[Tuple[int, int]]:
    forbidden = set(int(x) for x in forbidden_sensors or [])
    if anchor_a in forbidden and anchor_b in forbidden:
        return None
    if anchor_a in forbidden and anchor_b not in forbidden:
        return int(anchor_a), int(anchor_b)
    if anchor_b in forbidden and anchor_a not in forbidden:
        return int(anchor_b), int(anchor_a)
    if anchor_a <= anchor_b:
        return int(anchor_a), int(anchor_b)
    return int(anchor_b), int(anchor_a)


def dataset_sweep_configs(dataset: Dict[str, object]) -> List[SweepConfig]:
    sweeps = dataset.get("sweeps")
    if not isinstance(sweeps, list):
        return []
    out: List[SweepConfig] = []
    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        fixed = tuple(int(x) for x in sweep.get("fixed_anchors", []) or [])
        fixed_deltas = tuple(float(x) for x in sweep.get("fixed_lengths", []) or [])
        drive_raw = sweep.get("drive_anchor")
        sensor_raw = sweep.get("sensor_anchor")
        if drive_raw is None or sensor_raw is None:
            continue
        drive = int(drive_raw)
        sensor = int(sensor_raw)
        if len(fixed) != len(fixed_deltas):
            continue
        out.append(
            SweepConfig(
                fixed_anchors=fixed,
                fixed_deltas_mm=fixed_deltas,
                drive_anchor=drive,
                sensor_anchor=sensor,
            )
        )
    return out


def _predict_measurement_vec4_theta(
    anchors: np.ndarray,
    cfg: SweepConfig,
    *,
    dimensions: int,
    l2_scale: float,
    geometry_weights: GeometryWeights,
    eps_len_mm: float = 1.0,
) -> Optional[Tuple[np.ndarray, float]]:
    anchors = np.asarray(anchors, dtype=float)
    fixed_lengths_abs: List[float] = []
    eps_len = float(eps_len_mm)
    if not np.isfinite(eps_len) or eps_len <= 0.0:
        eps_len = 1.0
    for idx, delta in zip(cfg.fixed_anchors, cfg.fixed_deltas_mm):
        base = float(np.linalg.norm(anchors[int(idx)]))
        length_abs = base + float(delta)
        if not np.isfinite(length_abs) or length_abs < eps_len:
            return None
        fixed_lengths_abs.append(float(length_abs))

    geom = predict_ellipse_geometry(
        anchors,
        list(cfg.fixed_anchors),
        fixed_lengths_abs,
        int(cfg.drive_anchor),
        int(cfg.sensor_anchor),
        int(dimensions),
    )
    if geom is None:
        return None

    center, axes, theta = canonicalize_geometry(*geom)
    w_center, w_axes, _w_theta = geometry_weights
    s_center = math.sqrt(float(w_center)) / float(max(l2_scale, 1.0))
    s_axes = math.sqrt(float(w_axes)) / float(max(l2_scale, 1.0))
    vec4 = np.array(
        [
            float(center[0]) * s_center,
            float(center[1]) * s_center,
            float(axes[0]) * s_axes,
            float(axes[1]) * s_axes,
        ],
        dtype=float,
    )
    return vec4, float(theta)


def sweep_information_matrix(
    anchors: np.ndarray,
    cfg: SweepConfig,
    *,
    machine_type: str,
    num_anchors: int,
    dimensions: int,
    l2_scale: float,
    geometry_weights: GeometryWeights = (1.0, 1.0, 0.2),
    fd_eps_mm: float = 1.0,
    eps_len_mm: float = 1.0,
) -> Optional[np.ndarray]:
    """
    Approximate sweep Fisher information using a finite-difference Jacobian.

    Measurement vector is canonical ellipse geometry in the squared-length plane:
      [center_x, center_y, a, b, theta]
    with center/axes normalized by l2_scale and weighted by sqrt(geometry_weights).
    """
    anchors = np.asarray(anchors, dtype=float)
    num_anchors = int(num_anchors)
    dimensions = int(dimensions)

    x0 = anchors_matrix_to_vec(anchors)
    n_params = int(x0.size)
    m = 5

    eps = float(fd_eps_mm)
    if not np.isfinite(eps) or eps <= 0.0:
        eps = 1.0

    lb, ub = get_anchor_bounds(machine_type)
    if lb.size != n_params or ub.size != n_params:
        lb = np.full(n_params, -np.inf, dtype=float)
        ub = np.full(n_params, np.inf, dtype=float)

    def _eval(x: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        x = np.asarray(x, dtype=float).reshape(n_params)
        x = np.clip(x, lb, ub)
        a = anchors_vec_to_matrix(x, num_anchors, dimensions)
        return _predict_measurement_vec4_theta(
            a,
            cfg,
            dimensions=dimensions,
            l2_scale=l2_scale,
            geometry_weights=geometry_weights,
            eps_len_mm=eps_len_mm,
        )

    base = _eval(x0)
    if base is None:
        return None

    w_theta = float(geometry_weights[2])
    if not np.isfinite(w_theta) or w_theta < 0.0:
        w_theta = 0.0
    s_theta = math.sqrt(w_theta)

    J = np.zeros((m, n_params), dtype=float)
    for i in range(n_params):
        xp = x0.copy()
        xm = x0.copy()
        xp[i] += eps
        xm[i] -= eps
        fp = _eval(xp)
        fm = _eval(xm)
        if fp is None or fm is None:
            return None
        vec4_p, theta_p = fp
        vec4_m, theta_m = fm
        J[:4, i] = (vec4_p - vec4_m) / (2.0 * eps)
        J[4, i] = s_theta * _wrap_angle_pi(theta_p - theta_m) / (2.0 * eps)

    info = J.T @ J
    info = 0.5 * (info + info.T)
    return info


def total_information_matrix(
    anchors: np.ndarray,
    sweep_configs: Sequence[SweepConfig],
    *,
    machine_type: str,
    num_anchors: int,
    dimensions: int,
    l2_scale: float,
    geometry_weights: GeometryWeights = (1.0, 1.0, 0.2),
    fd_eps_mm: float = 1.0,
) -> np.ndarray:
    anchors = np.asarray(anchors, dtype=float)
    x0 = anchors_matrix_to_vec(anchors)
    out = np.zeros((int(x0.size), int(x0.size)), dtype=float)
    for cfg in sweep_configs:
        info = sweep_information_matrix(
            anchors,
            cfg,
            machine_type=machine_type,
            num_anchors=num_anchors,
            dimensions=dimensions,
            l2_scale=l2_scale,
            geometry_weights=geometry_weights,
            fd_eps_mm=fd_eps_mm,
        )
        if info is None:
            continue
        out += info
    out = 0.5 * (out + out.T)
    return out


def generate_candidate_sweeps(
    *,
    num_anchors: int,
    dimensions: int,
    fixed_delta_values_mm: Sequence[float],
    machine_type: Optional[str] = None,
    forbidden_sensors: Optional[Sequence[int]] = None,
) -> List[SweepConfig]:
    """
    Generate sweep candidates compatible with position sweep collection.

    Drive/sensor are emitted in a canonical order that keeps sensor anchors out of
    the forbidden set (carrying anchors). This matches autocal/control/cli/collect_sweep_data.mjs sweep output.
    """
    n = int(num_anchors)
    dims = int(dimensions)
    fixed_count = max(0, dims - 1)
    if fixed_count <= 0:
        return []

    forbidden = set(int(x) for x in (forbidden_sensors or []))
    if machine_type is not None:
        try:
            mt = MachineType(machine_type)
            cfg = MachineConfig.from_type(mt)
            forbidden.update(int(x) for x in cfg.carrying_anchors or [])
        except ValueError:
            pass

    deltas = [float(v) for v in fixed_delta_values_mm]
    deltas = [v for v in deltas if np.isfinite(v)]
    if not deltas:
        return []

    indices = list(range(n))
    out: List[SweepConfig] = []
    for fixed in combinations(indices, fixed_count):
        fixed = tuple(int(x) for x in fixed)
        free = [idx for idx in indices if idx not in fixed]
        for pair in combinations(free, 2):
            canonical = _canonical_drive_sensor_pair(int(pair[0]), int(pair[1]), forbidden)
            if canonical is None:
                continue
            drive, sensor = canonical
            if sensor in forbidden:
                continue
            for fixed_deltas in product(deltas, repeat=fixed_count):
                out.append(
                    SweepConfig(
                        fixed_anchors=fixed,
                        fixed_deltas_mm=tuple(float(x) for x in fixed_deltas),
                        drive_anchor=int(drive),
                        sensor_anchor=int(sensor),
                    )
                )
    return out


def _safe_logdet(matrix: np.ndarray) -> float:
    sign, logdet = np.linalg.slogdet(np.asarray(matrix, dtype=float))
    if sign <= 0 or not np.isfinite(logdet):
        return float("-inf")
    return float(logdet)


def rank_candidates_d_optimal(
    anchors: np.ndarray,
    *,
    machine_type: str,
    num_anchors: int,
    dimensions: int,
    l2_scale: float,
    observed: Sequence[SweepConfig],
    candidates: Sequence[SweepConfig],
    geometry_weights: GeometryWeights = (1.0, 1.0, 0.2),
    fd_eps_mm: float = 1.0,
    regularization: float = 1e-6,
    exclude_existing: bool = True,
    existing_tol_mm: float = 1e-3,
    top_k: int = 10,
) -> List[Tuple[float, SweepConfig]]:
    anchors = np.asarray(anchors, dtype=float)
    base_info = total_information_matrix(
        anchors,
        observed,
        machine_type=machine_type,
        num_anchors=num_anchors,
        dimensions=dimensions,
        l2_scale=l2_scale,
        geometry_weights=geometry_weights,
        fd_eps_mm=fd_eps_mm,
    )
    reg = float(regularization)
    if not np.isfinite(reg) or reg < 0.0:
        reg = 0.0
    if reg:
        base_info = base_info + reg * np.eye(base_info.shape[0], dtype=float)
    base_logdet = _safe_logdet(base_info)

    existing = set()
    if exclude_existing:
        for cfg in observed:
            existing.add(cfg.normalized_key(tol_mm=existing_tol_mm))

    scored: List[Tuple[float, SweepConfig]] = []
    for cfg in candidates:
        if exclude_existing and cfg.normalized_key(tol_mm=existing_tol_mm) in existing:
            continue
        info = sweep_information_matrix(
            anchors,
            cfg,
            machine_type=machine_type,
            num_anchors=num_anchors,
            dimensions=dimensions,
            l2_scale=l2_scale,
            geometry_weights=geometry_weights,
            fd_eps_mm=fd_eps_mm,
        )
        if info is None:
            continue
        trial = base_info + info
        score = _safe_logdet(trial) - base_logdet
        if not np.isfinite(score):
            continue
        scored.append((float(score), cfg))

    scored.sort(key=lambda item: item[0], reverse=True)
    if top_k and len(scored) > int(top_k):
        return scored[: int(top_k)]
    return scored
