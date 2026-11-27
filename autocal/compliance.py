"""Line stiffness / compliance estimation from fixed-pose force-step probes.

At the end of a sweep pass, all held lines (fixed + drive + forbidden-force) stay
in position mode while the sensor motor steps its commanded force by dF (and any
other force-mode lines keep constant force, dT = 0). With per-axis compliance
    c_i = (U_i * d_i + Y_i) / (EA_cfg * U_i^2 * N_i),
and scaled compliance factor k = EA_cfg / EA_true, each held line obeys
    u_h . dp = -dm_h - k * c_h * dT_h  =>  dT_h = -(1 / (k * c_h)) * (u_h . dp + dm_h).
Substituting into static equilibrium (sum_h u_h * dT_h + u_s * dF = 0) yields the
exact d x d elastic compatibility system (valid for both determinate and redundant
held lines):
    K_sys @ dp = k * dF * u_s - sum_h (1 / c_h) * dm_h * u_h,
    K_sys = sum_h (1 / c_h) * (u_h u_h^T),
so dp = P + k * Q is strictly affine in k, and the sensor's unstretched material
length change satisfies
    dm_s_rest = -u_s . dp - k * c_s * dF = a + k * b.
Collecting (a, b, dm_s_rest) across well-conditioned probe steps gives a robust
least-squares estimate of k and the measured axial stiffness EA_meas = EA_cfg / k.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from autocal.flex import FlexModel
from autocal.swivel_pulley import home_line_lengths, line_geometry, swivel_geometry_from_m669

_MIN_PLAUSIBLE_K = 0.2
_MAX_PLAUSIBLE_K = 5.0
_MIN_FORCE_STEP_N = 0.05
_MAX_STIFFNESS_COND = 1e4
_MIN_STEP_SENSITIVITY_MM = 1e-3
_MIN_SINGLE_STEP_SENSITIVITY_MM = 0.05
_MAX_RELATIVE_K_STD = 0.35


def has_compliance_probes(dataset: dict) -> bool:
    """Whether any sweep in `dataset` recorded fixed-pose compliance probes."""
    for sweep in dataset.get("sweeps", []):
        probes = (sweep.get("metadata") or {}).get("compliance_probes")
        if isinstance(probes, list) and any(
            isinstance(probe, dict) and len(probe.get("levels") or []) >= 2
            for probe in probes
        ):
            return True
    return False


def _line_description(dataset: dict) -> Optional[Tuple[float, np.ndarray, np.ndarray, np.ndarray]]:
    config = dataset.get("config") or {}
    m666 = config.get("m666_before_data_collection") or {}
    num_anchors = int(dataset.get("num_anchors", 0))
    if num_anchors <= 0:
        return None
    try:
        stiffness = float(m666["S"])
        strands = np.broadcast_to(np.asarray(m666.get("N", 1.0), dtype=float), (num_anchors,)).copy()
        advantage = np.broadcast_to(np.asarray(m666.get("U", 1.0), dtype=float), (num_anchors,)).copy()
        extra = np.broadcast_to(np.asarray(m666.get("Y", 0.0), dtype=float), (num_anchors,)).copy()
    except (KeyError, TypeError, ValueError):
        return None
    if (
        not np.isfinite(stiffness)
        or stiffness <= 0.0
        or not np.all(np.isfinite(strands) & (strands > 0.0))
        or not np.all(np.isfinite(advantage) & (advantage > 0.0))
        or not np.all(np.isfinite(extra) & (extra >= 0.0))
    ):
        return None
    return stiffness, strands, advantage, extra


def _default_forbidden_anchors(machine_type: str, num_anchors: int) -> List[int]:
    if machine_type in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced") and num_anchors >= 4:
        return [num_anchors - 1]
    return []


def _probe_rows(dataset: dict):
    config = dataset.get("config") or {}
    mm_raw = config.get("mm_per_degree")
    num_anchors = int(dataset.get("num_anchors", 0))
    machine_type = str(dataset.get("machine_type", ""))
    if mm_raw is None or len(mm_raw) != num_anchors:
        return
    mm_per_deg = np.asarray(mm_raw, dtype=float)
    if not np.all(np.isfinite(mm_per_deg) & (mm_per_deg != 0.0)):
        return
    default_forbidden = _default_forbidden_anchors(machine_type, num_anchors)
    for sweep in dataset.get("sweeps", []):
        probes = (sweep.get("metadata") or {}).get("compliance_probes") or []
        for probe in probes:
            if not isinstance(probe, dict):
                continue
            levels = probe.get("levels") or []
            if len(levels) < 2:
                continue
            try:
                lengths = np.array(
                    [np.asarray(level["mu_deg"], dtype=float) * mm_per_deg for level in levels],
                    dtype=float,
                )
                forces = np.array([float(level["force_n"]) for level in levels], dtype=float)
            except (KeyError, TypeError, ValueError):
                continue
            if lengths.ndim != 2 or lengths.shape[1] != num_anchors:
                continue
            if not (np.all(np.isfinite(lengths)) and np.all(np.isfinite(forces))):
                continue
            sensor = int(probe["sensor_anchor"])
            recorded_held = probe.get("held_anchors")
            if isinstance(recorded_held, list) and recorded_held:
                held = sorted({int(a) for a in recorded_held if 0 <= int(a) < num_anchors and int(a) != sensor})
            else:
                held = sorted(
                    a for a in (
                        {int(a) for a in sweep["fixed_anchors"]}
                        | {int(probe["drive_anchor"])}
                        | set(default_forbidden)
                    )
                    if 0 <= a < num_anchors and a != sensor
                )
            yield sensor, held, forces, lengths


def _position_at_lengths(
    anchors: np.ndarray,
    length_delta_mm: np.ndarray,
    home_offsets_mm: np.ndarray,
    *,
    swivel: Optional[dict] = None,
) -> np.ndarray:
    """Solve effector position from one point's length deltas."""
    anchors = np.asarray(anchors, dtype=float)
    dims = anchors.shape[1]
    absolute = home_line_lengths(anchors, swivel=swivel) + np.asarray(home_offsets_mm, dtype=float) + length_delta_mm
    pos = np.zeros((1, dims), dtype=float)
    damping = 1e-9 * np.eye(dims)
    for _ in range(25):
        dist, dir_to_anchor = line_geometry(anchors, pos, swivel=swivel)
        direction = -dir_to_anchor[0]
        misfit = dist[0] - absolute
        normal = direction.T @ direction + damping
        grad = direction.T @ misfit
        pos = pos - np.linalg.solve(normal, grad)[None, :]
    return pos[0]


def _step_terms(
    anchors: np.ndarray,
    position: np.ndarray,
    sensor: int,
    held: Sequence[int],
    d_force: float,
    d_lengths: np.ndarray,
    stiffness: float,
    strands: np.ndarray,
    advantage: np.ndarray,
    extra: np.ndarray,
    *,
    swivel: Optional[dict] = None,
    flex_model: Optional[FlexModel] = None,
    force_high: Optional[float] = None,
) -> Optional[Tuple[float, float]]:
    """Compute `(a, b)` in `dm_s_rest = a + k * b` for one force step at one pose.

    Uses exact elastic compatibility across all held lines (`K_sys = sum_h (1/c_h) u_h u_h^T`),
    rejects poorly conditioned geometries, and accounts for spool flattening / strained storage
    material transfer on moving lines when `flex_model` has spool physics enabled.
    """
    dims = anchors.shape[1]
    held_idx = list(held)
    if len(held_idx) < dims or sensor in held_idx:
        return None
    dist, dir_to_anchor = line_geometry(anchors, position[None, :], swivel=swivel)
    distance = dist[0]
    unit = dir_to_anchor[0]  # (N, D) unit vectors from effector toward anchor/exit
    compliance = (advantage * distance + extra) / (stiffness * (advantage ** 2) * strands)
    if not np.all(np.isfinite(compliance) & (compliance > 0.0)):
        return None

    # Exact elastic compatibility:
    #   K_sys @ dp = k * dF * u_s - sum_h (1 / c_h) * dm_h * u_h
    inv_c_held = 1.0 / compliance[held_idx]
    u_held = unit[held_idx]  # (H, D)
    k_sys = (u_held.T * inv_c_held[None, :]) @ u_held
    eigs = np.linalg.eigvalsh(k_sys)
    if not np.all(np.isfinite(eigs)) or eigs[0] <= 0.0 or (eigs[-1] / eigs[0]) > _MAX_STIFFNESS_COND:
        return None

    dm_held = np.asarray(d_lengths[held_idx], dtype=float)
    if flex_model is not None and flex_model.has_spool_physics and force_high is not None:
        # Scale any tiny held-line winding motion by nominal spool storage ratio.
        rho_h = np.array(
            [float(flex_model.spool_rho(np.array([max(force_high, 0.0)]), axis=h)[0]) for h in held_idx],
            dtype=float,
        )
        dm_held = np.where(dm_held < 0.0, dm_held * rho_h, dm_held)

    rhs_p = -(u_held.T @ (inv_c_held * dm_held))
    rhs_q = d_force * unit[sensor]
    try:
        p_vec = np.linalg.solve(k_sys, rhs_p)
        q_vec = np.linalg.solve(k_sys, rhs_q)
    except np.linalg.LinAlgError:
        return None

    a = float(-unit[sensor] @ p_vec)
    b = float(-unit[sensor] @ q_vec - compliance[sensor] * d_force)
    if not (np.isfinite(a) and np.isfinite(b)) or abs(b) < _MIN_STEP_SENSITIVITY_MM:
        return None
    return a, b


def _default_level_pairs(forces: np.ndarray) -> List[Tuple[int, int]]:
    """Step pairs from level 0 to every higher force level (skipping return-to-base repeats)."""
    pairs: List[Tuple[int, int]] = []
    f0 = float(forces[0])
    for j in range(1, len(forces)):
        if abs(float(forces[j]) - f0) >= _MIN_FORCE_STEP_N:
            pairs.append((0, j))
    return pairs


def estimate_compliance_scale(
    dataset: dict,
    anchors: np.ndarray,
    home_offsets_mm: Optional[Sequence[float]] = None,
    *,
    swivel: Optional[dict] = None,
    pairs: Optional[Sequence[Tuple[int, int]]] = None,
) -> Optional[Dict[str, object]]:
    """Estimate compliance scale `k = EA_cfg / EA_true` from recorded compliance probes.

    Returns a diagnostics dict with `k_hat`, `stiffness_n` (`EA_cfg / k_hat`), `k_std`,
    `num_steps`, and `source = "compliance_probe"`, or None if insufficient/implausible.
    """
    desc = _line_description(dataset)
    if desc is None:
        return None
    stiffness, strands, advantage, extra = desc
    anchors_arr = np.asarray(anchors, dtype=float)
    num_anchors = anchors_arr.shape[0]
    if home_offsets_mm is None:
        offsets = np.zeros(num_anchors, dtype=float)
    else:
        offsets = np.asarray(home_offsets_mm, dtype=float)
    if swivel is None:
        m669 = (dataset.get("config") or {}).get("m669")
        swivel = swivel_geometry_from_m669(m669, num_anchors)
    m666_cfg = (dataset.get("config") or {}).get("m666_before_data_collection") or {}
    flex_model = FlexModel.from_m666(m666_cfg, num_axes=num_anchors)

    a_vals: List[float] = []
    b_vals: List[float] = []
    m_vals: List[float] = []
    step_k: List[float] = []
    for sensor, held, forces, lengths in _probe_rows(dataset):
        level_pairs = list(pairs) if pairs is not None else _default_level_pairs(forces)
        for i, j in level_pairs:
            if i >= len(forces) or j >= len(forces):
                continue
            d_force = float(forces[j] - forces[i])
            if abs(d_force) < _MIN_FORCE_STEP_N:
                continue
            position = _position_at_lengths(anchors_arr, lengths[i], offsets, swivel=swivel)
            terms = _step_terms(
                anchors_arr,
                position,
                sensor,
                held,
                d_force,
                lengths[j] - lengths[i],
                stiffness,
                strands,
                advantage,
                extra,
                swivel=swivel,
                flex_model=flex_model,
                force_high=max(float(forces[i]), float(forces[j])),
            )
            if terms is None:
                continue
            a, b = terms
            raw_dm_s = float(lengths[j, sensor] - lengths[i, sensor])
            if flex_model is not None and flex_model.has_spool_physics and raw_dm_s < 0.0:
                # Winding onto the drum under tension stores rho(T_avg) rest cable per mm of drum arc.
                f_wind = 0.5 * (float(forces[i]) + float(forces[j]))
                rho_s = float(flex_model.spool_rho(np.array([f_wind]), axis=sensor)[0])
                measured = raw_dm_s * rho_s
            else:
                measured = raw_dm_s
            a_vals.append(a)
            b_vals.append(b)
            m_vals.append(measured)
            step_k.append((measured - a) / b)

    if not b_vals:
        return None

    a_arr = np.asarray(a_vals, dtype=float)
    b_arr = np.asarray(b_vals, dtype=float)
    m_arr = np.asarray(m_vals, dtype=float)
    y_arr = m_arr - a_arr

    # Robust Cauchy IRLS initialized at the step-wise median protects the slope against
    # slipped or noisy probe steps while staying accurate on clean data.
    denom = float(np.sum(b_arr * b_arr))
    if denom <= 1e-18:
        return None
    if len(b_arr) == 1 and abs(float(b_arr[0])) < _MIN_SINGLE_STEP_SENSITIVITY_MM:
        return None
    k_ols = float(np.sum(b_arr * y_arr) / denom)
    weights = np.ones_like(b_arr, dtype=float)
    if len(b_arr) >= 4:
        k_hat = float(np.median(y_arr / b_arr))
        for _ in range(3):
            resid = y_arr - k_hat * b_arr
            mad = 1.4826 * float(np.median(np.abs(resid - np.median(resid))))
            scale = max(mad, 0.01)
            weights = 1.0 / (1.0 + (resid / (2.5 * scale)) ** 2)
            w_denom = float(np.sum(weights * b_arr * b_arr))
            if w_denom > 1e-18:
                k_hat = float(np.sum(weights * b_arr * y_arr) / w_denom)
    else:
        k_hat = k_ols

    if not np.isfinite(k_hat) or not (_MIN_PLAUSIBLE_K <= k_hat <= _MAX_PLAUSIBLE_K):
        return None

    resid = y_arr - k_hat * b_arr
    w_denom = float(np.sum(weights * b_arr * b_arr))
    eff_n = float(np.sum(weights))
    if len(b_arr) >= 4 and eff_n < 0.6 * len(b_arr):
        return None
    if len(b_arr) >= 2 and w_denom > 1e-18 and eff_n > 1.0:
        k_std = float(np.sqrt(float(np.sum(weights * resid ** 2)) / ((eff_n - 1.0) * w_denom)))
        if not np.isfinite(k_std) or (k_std / k_hat) > _MAX_RELATIVE_K_STD:
            return None
    else:
        k_std = float("nan")
    ea_measured = float(stiffness / k_hat)
    return {
        "source": "compliance_probe",
        "k_hat": float(k_hat),
        "k_std": k_std,
        "stiffness_cfg_n": float(stiffness),
        "stiffness_n": ea_measured,
        "spring_k_multiplier": float(1.0 / k_hat),
        "num_steps": int(len(b_arr)),
        "step_k_median": float(np.median(step_k)),
    }


def resolve_measured_stiffness(
    dataset: dict,
    anchors: np.ndarray,
    home_offsets_mm: Optional[Sequence[float]] = None,
    *,
    varpro: Optional[dict] = None,
    swivel: Optional[dict] = None,
) -> Optional[Dict[str, object]]:
    """Resolve the best independently measured axial stiffness `EA` (N) for `M666 S`.

    Prefers fixed-pose compliance probes (which isolate span compliance on every machine).
    Falls back to VarPro gravity-separated stiffness when no compliance probes were collected
    and VarPro confirmed gravity identifiability.
    """
    probe = estimate_compliance_scale(dataset, anchors, home_offsets_mm, swivel=swivel)
    if probe is not None:
        return probe

    if isinstance(varpro, dict) and varpro.get("k_grav_identifiable"):
        ea_est = varpro.get("ea_estimated_n")
        k_grav = varpro.get("k_grav_hat")
        if (
            isinstance(ea_est, (int, float))
            and np.isfinite(ea_est)
            and float(ea_est) > 0.0
            and isinstance(k_grav, (int, float))
            and np.isfinite(k_grav)
            and _MIN_PLAUSIBLE_K <= float(k_grav) <= _MAX_PLAUSIBLE_K
        ):
            ea_cfg = float(varpro.get("ea_cfg_n", float(ea_est) * float(k_grav)))
            return {
                "source": "varpro",
                "k_hat": float(k_grav),
                "k_std": float(varpro.get("ea_grav_std", float("nan"))),
                "stiffness_cfg_n": ea_cfg,
                "stiffness_n": float(ea_est),
                "spring_k_multiplier": float(1.0 / float(k_grav)),
                "num_steps": 0,
            }
    return None
