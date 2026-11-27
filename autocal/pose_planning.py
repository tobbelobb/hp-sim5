"""Sweep ranking by the information each candidate adds to the point-based fit.

A candidate sweep is scored by the effector positions it would visit, not by the
ellipse it would trace. Home length offsets are part of the parameter set, so a
candidate that leaves them indistinguishable from an anchor move scores low. For
a Hangprinter that favours sweeps with a shortened D line: with D at one length
every position lies on a single sphere around the D anchor.
"""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from autocal.active_learning import SweepConfig
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, consistency_residuals
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec

_CIRCLE_SAMPLES = 360
# A position counts as reachable while each of the sweep's lines carries at least
# this share of the largest pretension. 0.25 reproduces where recorded HP3 sweeps turn.
SWEEP_PRETENSION_SHARE = 0.25
# A position is planned only this share of each anchor's home distance away from it, or further.
ANCHOR_CLEARANCE_SHARE = 0.10
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


def predict_sweep_positions(
    anchors: np.ndarray, cfg: SweepConfig, points: int, min_pretension_share: float,
) -> np.ndarray:
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
    reachable = np.min(pretension, axis=1) > min_pretension_share * np.max(pretension, axis=1)
    clearance = np.linalg.norm(anchors[None, :, :] - curve[:, None, :], axis=2)
    reachable &= np.all(clearance >= ANCHOR_CLEARANCE_SHARE * np.linalg.norm(anchors, axis=1), axis=1)
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


def point_informations(
    anchors: np.ndarray,
    lengths_mm: np.ndarray,
    positions: np.ndarray,
    *,
    machine_type: str,
    sigma_mm: float,
) -> np.ndarray:
    """Information of each point, at known positions, about the anchors and home offsets.

    Each point's d pose coordinates are eliminated analytically. NaN lengths are
    unmeasured lines.
    """
    anchors = np.asarray(anchors, dtype=float)
    num_anchors, dimensions = anchors.shape
    num_offsets = num_anchors - dimensions
    anchor_vec = anchors_matrix_to_opt_vec(anchors, machine_type)
    # Candidate poses are already known exactly. Eliminate their d nuisance
    # coordinates analytically rather than re-solving them from home for
    # every parameter perturbation (which can select another sphere branch).
    mapping = np.empty((num_anchors, dimensions, anchor_vec.size))
    for index in range(anchor_vec.size):
        step = np.zeros(anchor_vec.size)
        step[index] = _FD_STEP_MM
        plus = anchor_opt_vec_to_matrix(anchor_vec + step, machine_type, num_anchors, dimensions)
        minus = anchor_opt_vec_to_matrix(anchor_vec - step, machine_type, num_anchors, dimensions)
        mapping[:, :, index] = (plus - minus) / (2 * _FD_STEP_MM)
    vector = np.asarray(positions)[:, None, :] - anchors[None, :, :]
    direction = vector / np.linalg.norm(vector, axis=2)[:, :, None]
    home_direction = anchors / np.linalg.norm(anchors, axis=1)[:, None]
    jacobian = np.einsum('pnd,ndk->pnk', -direction - home_direction, mapping)
    offset_jacobian = -np.eye(num_anchors)[:, -num_offsets:]
    jacobian = np.concatenate([jacobian, np.broadcast_to(offset_jacobian, (*lengths_mm.shape, num_offsets))], axis=2)
    measured = np.isfinite(lengths_mm)[:, :, None]
    direction = direction * measured
    jacobian *= measured
    normal = np.einsum('pni,pnj->pij', direction, direction) + 1e-9*np.eye(dimensions)
    pose_step = np.linalg.solve(normal, np.einsum('pni,pnk->pik', direction, jacobian))
    jacobian -= np.einsum('pni,pik->pnk', direction, pose_step)
    return np.einsum('pnk,pnl->pkl', jacobian, jacobian) / sigma_mm**2


def pose_information(
    anchors: np.ndarray,
    lengths_mm: np.ndarray,
    *,
    machine_type: str,
    sigma_mm: float,
    positions: Optional[np.ndarray] = None,
    stretch=None,
    swivel=None,
) -> np.ndarray:
    """Information the given length changes carry about the anchors and the home length offsets."""
    anchors = np.asarray(anchors, dtype=float)
    num_anchors, dimensions = anchors.shape
    num_offsets = num_anchors - dimensions
    anchor_vec = anchors_matrix_to_opt_vec(anchors, machine_type)

    if positions is not None:
        return np.sum(point_informations(anchors, lengths_mm, positions, machine_type=machine_type, sigma_mm=sigma_mm), axis=0)

    def residuals(step: np.ndarray) -> np.ndarray:
        moved = anchor_opt_vec_to_matrix(anchor_vec + step[:anchor_vec.size], machine_type, num_anchors,
                                         dimensions)
        offsets = np.zeros(num_anchors)
        offsets[num_anchors - num_offsets:] = step[anchor_vec.size:]
        return consistency_residuals(moved, lengths_mm, offsets, stretch=stretch, swivel=swivel).ravel()

    size = anchor_vec.size + num_offsets
    jacobian = np.zeros((lengths_mm.size, size))
    for index in range(size):
        step = np.zeros(size)
        step[index] = _FD_STEP_MM
        jacobian[:, index] = (residuals(step) - residuals(-step)) / (2.0 * _FD_STEP_MM)
    return jacobian.T @ jacobian / sigma_mm ** 2


def schur_anchor_information(
    joint_info: np.ndarray,
    num_anchor_params: int,
    nuisance_prior_diag: Sequence[float],
) -> np.ndarray:
    """Schur-complement effective anchor information marginalizing out linear stretch nuisances.

    I_eff(a) = I_aa - I_ap (I_pp + Lambda_prior)^{-1} I_pa.
    Automatically zeroes out apparent z_D information that can be mimicked by rescaling stretch
    or motor force.
    """
    joint_info = np.asarray(joint_info, dtype=float)
    k = int(num_anchor_params)
    i_aa = joint_info[:k, :k]
    if joint_info.shape[0] <= k:
        return i_aa
    i_ap = joint_info[:k, k:]
    i_pp = joint_info[k:, k:] + np.diag(np.asarray(nuisance_prior_diag, dtype=float))
    return i_aa - i_ap @ np.linalg.solve(i_pp, i_ap.T)


def _stretch_basis_from_stretch(stretch: dict, num_anchors: int) -> Optional[dict]:
    """Adapt a pose_fit.dataset_stretch dict into the format expected by static_tension_superposition."""
    if not isinstance(stretch, dict) or stretch.get("model") is None:
        return None
    model = stretch["model"]
    ma = np.array([float(model.mechanical_advantage[a]) for a in range(num_anchors)], dtype=float)
    guy = np.array([float(model.guy_wire_lengths[a]) for a in range(num_anchors)], dtype=float)
    strands = np.array(
        [float(model.lines_per_axis[a]) if a < len(model.lines_per_axis) else 1.0 for a in range(num_anchors)],
        dtype=float,
    )
    return {
        "model": model,
        "sensors": np.asarray(stretch["sensors"], dtype=int),
        "known": np.asarray(stretch["known"], dtype=float),
        "forces": np.asarray(stretch["forces"], dtype=float),
        "weight": float(stretch.get("weight", 0.0)),
        "passive": float(stretch.get("passive", 0.0)),
        "ma": ma,
        "guy": guy,
        "strands": strands,
        "ea_cfg": float(model.effective_stiffness_n(0)),
        "spool_history": stretch.get("spool_history"),
        "swivel": stretch.get("swivel"),
    }


def _candidate_stretch_basis(
    stretch_basis: dict,
    cfg: SweepConfig,
    num_points: int,
    num_anchors: int,
    *,
    force_scale: float = 1.0,
    base_sensor_force_n: Optional[float | np.ndarray] = None,
    extra_sensor_force_n: Optional[float] = None,
    max_extra_force_n: Optional[float] = None,
) -> dict:
    """Build a synthetic per-point stretch basis for a candidate (sweep, force_scale) action."""
    if base_sensor_force_n is None:
        base_sensor_force_n = float(np.median(stretch_basis["forces"])) if len(stretch_basis["forces"]) else 2.0
    base_arr = np.broadcast_to(np.asarray(base_sensor_force_n, dtype=float), (num_anchors,))
    sensors = np.full(num_points, int(cfg.sensor_anchor), dtype=int)
    # Match the two-direction collector: second half reverses drive/sensor roles.
    if num_points >= 2:
        sensors[num_points // 2:] = int(cfg.drive_anchor)
    forces = base_arr[sensors] * float(force_scale)
    known = np.full((num_points, num_anchors), np.nan)
    active = {*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor}
    if extra_sensor_force_n is not None and np.isfinite(extra_sensor_force_n) and extra_sensor_force_n > 0.0:
        ext_f = float(extra_sensor_force_n) * float(force_scale)
        if max_extra_force_n is not None and np.isfinite(max_extra_force_n):
            ext_f = min(float(max_extra_force_n), ext_f)
        for axis in range(num_anchors):
            if axis not in active:
                known[:, axis] = ext_f
    return {
        **stretch_basis,
        "sensors": sensors,
        "known": known,
        "forces": forces,
    }


def point_joint_stretch_information(
    anchors: np.ndarray,
    lengths_mm: np.ndarray,
    positions: np.ndarray,
    *,
    machine_type: str,
    sigma_mm: float,
    stretch_basis: dict,
) -> np.ndarray:
    """Analytic joint Fisher information over (anchors, home_offsets, dk_grav, dalpha_sensor).

    Costs zero extra finite differences: the geometric and linear stretch nuisance columns
    (gravity compliance 1/EA and sensor-force compliance alpha/EA) are computed analytically
    and pose-eliminated together.
    """
    from autocal.stretch_varpro import project_pose_orthogonal, static_tension_superposition

    anchors = np.asarray(anchors, dtype=float)
    lengths_mm = np.asarray(lengths_mm, dtype=float)
    positions = np.asarray(positions, dtype=float)
    num_anchors, dimensions = anchors.shape
    num_offsets = num_anchors - dimensions
    measured = np.isfinite(lengths_mm)
    anchor_vec = anchors_matrix_to_opt_vec(anchors, machine_type)
    mapping = np.empty((num_anchors, dimensions, anchor_vec.size))
    for index in range(anchor_vec.size):
        step = np.zeros(anchor_vec.size)
        step[index] = _FD_STEP_MM
        plus = anchor_opt_vec_to_matrix(anchor_vec + step, machine_type, num_anchors, dimensions)
        minus = anchor_opt_vec_to_matrix(anchor_vec - step, machine_type, num_anchors, dimensions)
        mapping[:, :, index] = (plus - minus) / (2.0 * _FD_STEP_MM)
    vector = positions[:, None, :] - anchors[None, :, :]
    dist = np.maximum(np.linalg.norm(vector, axis=2), 1e-9)
    direction = (vector / dist[:, :, None]) * measured[:, :, None]
    home_direction = anchors / np.maximum(np.linalg.norm(anchors, axis=1), 1e-9)[:, None]
    j_geom = np.einsum("pnd,ndk->pnk", -direction - home_direction[None, :, :] * measured[:, :, None], mapping)
    offset_jac = np.broadcast_to(-np.eye(num_anchors)[:, -num_offsets:], (*lengths_mm.shape, num_offsets))
    j_anchor = np.concatenate([j_geom, offset_jac], axis=2) * measured[:, :, None]

    _, _, t_grav, _, _, t_nom, g = static_tension_superposition(anchors, positions, measured, stretch_basis)
    t_cmd = t_nom - t_grav
    b_psi = np.stack([g * t_grav, g * t_cmd], axis=2) * measured[:, :, None]
    j_full = np.concatenate([j_anchor, -b_psi], axis=2)
    j_proj = project_pose_orthogonal(direction, measured, j_full)
    return np.einsum("pnk,pnl->kl", j_proj, j_proj) / (sigma_mm ** 2)


def rank_candidates_force_aware(
    anchors: np.ndarray,
    *,
    machine_type: str,
    observed_lengths_mm: np.ndarray,
    candidates: Sequence[SweepConfig],
    points_per_sweep: int,
    sigma_mm: float,
    regularization: float,
    top_k: int,
    measure_extra_ranges: bool = False,
    observed_stretch=None,
    candidate_force_scales: Sequence[float] = (1.0,),
    base_sensor_force_n: Optional[float | np.ndarray] = None,
    base_extra_force_n: Optional[float] = None,
    max_extra_force_n: Optional[float] = None,
    effective_scale_fn: Optional[Callable[[SweepConfig, float], Optional[float]]] = None,
    nuisance_prior_sigma: Tuple[float, float] = (0.35, 0.35),
) -> List[Tuple[float, SweepConfig, float]]:
    """Rank (sweep, force_scale) candidates by Schur-complement Ds-optimal anchor information gain.

    When `observed_stretch` is provided, marginalizes out both gravity compliance (1/EA) and
    sensor-force compliance (alpha/EA) via the Schur complement so confounded single-force
    sweeps on the z_D <-> alpha/EA ridge no longer receive inflated scores. When
    `base_sensor_force_n` (and `base_extra_force_n`) are provided, candidates are evaluated at
    those exact intended Newtons rather than inferring a baseline from past recorded forces.
    When `effective_scale_fn` is provided, candidates are pre-ranked analytically and feasibility
    / force capping is evaluated lazily in ranked order with branch-and-bound early stopping.
    """
    from autocal.pose_fit import fit_point_positions

    anchors = np.asarray(anchors, dtype=float)
    num_anchors, dimensions = anchors.shape
    num_offsets = num_anchors - dimensions
    home_lengths = np.linalg.norm(anchors, axis=1)
    options = dict(machine_type=machine_type, sigma_mm=sigma_mm)
    basis = _stretch_basis_from_stretch(observed_stretch, num_anchors)
    force_scales = tuple(float(s) for s in candidate_force_scales) or (1.0,)

    def _resolve_lazy(
        pre_ranked: List[Tuple[float, SweepConfig, float]],
        rescore_fn: Callable[[SweepConfig, float], Optional[float]],
    ) -> List[Tuple[float, SweepConfig, float]]:
        if effective_scale_fn is None:
            return pre_ranked[:top_k]
        verified: List[Tuple[float, SweepConfig, float]] = []
        seen_eff: dict[int, List[float]] = {}
        min_eval = max(int(top_k), 8)
        for upper_gain, cfg, req in pre_ranked:
            if len(verified) >= min_eval:
                verified.sort(key=lambda item: (round(item[0], 6), -item[2]), reverse=True)
                if verified[min(int(top_k), len(verified)) - 1][0] >= upper_gain + 1e-6:
                    break
            eff = effective_scale_fn(cfg, req)
            if eff is None or not np.isfinite(eff) or eff <= 0.0:
                continue
            eff_f = float(eff)
            prev_list = seen_eff.setdefault(id(cfg), [])
            if any(abs(eff_f - prev) <= 1e-3 for prev in prev_list):
                continue
            prev_list.append(eff_f)
            if abs(eff_f - req) <= 1e-4:
                gain = upper_gain
            else:
                gain = rescore_fn(cfg, eff_f)
                if gain is None or not np.isfinite(gain):
                    continue
            verified.append((float(gain), cfg, eff_f))
        verified.sort(key=lambda item: (round(item[0], 6), -item[2]), reverse=True)
        return verified[:top_k]

    if basis is None:
        base = pose_information(anchors, observed_lengths_mm, stretch=observed_stretch, **options)
        prior = np.full(base.shape[0], float(regularization))
        prior[base.shape[0] - num_offsets:] += 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
        base = base + np.diag(prior)
        base_logdet = np.linalg.slogdet(base)[1]
        gain_by_cfg: dict[int, float] = {}
        scored: List[Tuple[float, SweepConfig, float]] = []
        for cfg in candidates:
            positions = predict_sweep_positions(anchors, cfg, points_per_sweep, SWEEP_PRETENSION_SHARE)
            if positions.shape[0] == 0:
                continue
            lengths = np.linalg.norm(anchors[None, :, :] - positions[:, None, :], axis=2) - home_lengths
            active = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
            if not measure_extra_ranges:
                lengths[:, [axis for axis in range(num_anchors) if axis not in active]] = np.nan
            info_c = pose_information(anchors, lengths, positions=positions, **options)
            gain = float(np.linalg.slogdet(base + info_c)[1] - base_logdet)
            if np.isfinite(gain):
                gain_by_cfg[id(cfg)] = gain
                for scale in force_scales:
                    scored.append((gain, cfg, scale))
        scored.sort(key=lambda item: (round(item[0], 6), -item[2]), reverse=True)
        return _resolve_lazy(scored, lambda cfg, _s: gain_by_cfg.get(id(cfg)))

    # Joint observed information over (anchors, home_offsets, dk_grav, dalpha_sensor).
    obs_positions, _ = fit_point_positions(
        anchors, observed_lengths_mm, np.zeros(num_anchors), stretch=observed_stretch,
    )
    base_joint = point_joint_stretch_information(
        anchors, observed_lengths_mm, obs_positions,
        machine_type=machine_type, sigma_mm=sigma_mm, stretch_basis=basis,
    )
    num_geom = base_joint.shape[0] - 2
    prior_geom = np.full(num_geom, float(regularization))
    prior_geom[num_geom - num_offsets:] += 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
    base_joint[:num_geom, :num_geom] += np.diag(prior_geom)
    nuis_prior = [1.0 / (float(s) ** 2) for s in nuisance_prior_sigma]
    base_eff = schur_anchor_information(base_joint, num_geom, nuis_prior)
    base_logdet = float(np.linalg.slogdet(base_eff)[1])

    extra_known = basis["known"][np.isfinite(basis["known"])]
    if base_extra_force_n is not None:
        extra_force = float(base_extra_force_n) if measure_extra_ranges else None
    else:
        extra_force = float(np.median(extra_known)) if (measure_extra_ranges and extra_known.size) else None
    if base_sensor_force_n is not None:
        base_sensor_force: float | np.ndarray = base_sensor_force_n
    else:
        base_sensor_force = float(np.min(basis["forces"])) if basis["forces"].size else 2.0

    geom_cache: dict[int, Tuple[np.ndarray, np.ndarray]] = {}

    def _score_candidate(cfg: SweepConfig, scale: float) -> Optional[float]:
        cached = geom_cache.get(id(cfg))
        if cached is None:
            return None
        positions, lengths = cached
        cand_basis = _candidate_stretch_basis(
            basis, cfg, positions.shape[0], num_anchors,
            force_scale=scale,
            base_sensor_force_n=base_sensor_force,
            extra_sensor_force_n=extra_force,
            max_extra_force_n=max_extra_force_n,
        )
        info_c = point_joint_stretch_information(
            anchors, lengths, positions,
            machine_type=machine_type, sigma_mm=sigma_mm, stretch_basis=cand_basis,
        )
        eff = schur_anchor_information(base_joint + info_c, num_geom, nuis_prior)
        gain = float(np.linalg.slogdet(eff)[1] - base_logdet)
        return gain if np.isfinite(gain) else None

    scored = []
    for cfg in candidates:
        positions = predict_sweep_positions(anchors, cfg, points_per_sweep, SWEEP_PRETENSION_SHARE)
        if positions.shape[0] == 0:
            continue
        lengths = np.linalg.norm(anchors[None, :, :] - positions[:, None, :], axis=2) - home_lengths
        active = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
        if not measure_extra_ranges:
            lengths[:, [axis for axis in range(num_anchors) if axis not in active]] = np.nan
        geom_cache[id(cfg)] = (positions, lengths)
        for scale in force_scales:
            gain = _score_candidate(cfg, scale)
            if gain is not None:
                scored.append((gain, cfg, scale))
    # Prefer lower force on near-ties so hysteresis does not pre-stretch lines unnecessarily.
    scored.sort(key=lambda item: (round(item[0], 6), -item[2]), reverse=True)
    return _resolve_lazy(scored, _score_candidate)


def rank_candidates_pose_information(
    anchors: np.ndarray,
    *,
    machine_type: str,
    observed_lengths_mm: np.ndarray,
    candidates: Sequence[SweepConfig],
    points_per_sweep: int,
    sigma_mm: float,
    regularization: float,
    top_k: int,
    measure_extra_ranges: bool = False,
    observed_stretch=None,
) -> List[Tuple[float, SweepConfig]]:
    """Rank candidates by the gain in log-determinant of the point-fit information.

    Uses Schur-complement Ds-optimal scoring automatically when `observed_stretch` is supplied.
    """
    ranked = rank_candidates_force_aware(
        anchors,
        machine_type=machine_type,
        observed_lengths_mm=observed_lengths_mm,
        candidates=candidates,
        points_per_sweep=points_per_sweep,
        sigma_mm=sigma_mm,
        regularization=regularization,
        top_k=top_k,
        measure_extra_ranges=measure_extra_ranges,
        observed_stretch=observed_stretch,
        candidate_force_scales=(1.0,),
    )
    return [(score, cfg) for score, cfg, _ in ranked]
