"""Golub-Pereyra Variable Projection (VarPro) for quasi-static cable stretch and real-world nuisances.

Static equilibrium is linear in cable tensions, so elastic stretch splits cleanly into a
gravity-driven part (proportional to 1/EA) and a sensor-force-driven part (proportional to
alpha_m / EA, F_{0,m} / EA, directional friction f_{dir,m} / EA, and periodic cogging / EA).
Additional real-world effects — retained peak-load line compliance hysteresis and permanent
per-line creep across sweeps — also enter the pose-eliminated range residual linearly.

Eliminating these nuisance parameters analytically in a closed-form inner linear solve inside
the outer geometric least-squares loop:
  1. Adds zero finite-difference dimensions to the outer optimizer.
  2. Exposes whether true EA (1/EA via gravity mg) actually separated from motor force scale
     (alpha / EA) before exporting stiffness to config.g.
  3. Protects anchor geometry against wrong configured stiffness, motor cogging, line
     compliance hysteresis/creep, directional friction, and unpredictable stiction spikes.
"""
from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares

from autocal.flex import FlexModel, extract_spool_history
from autocal.swivel_pulley import home_line_lengths, line_geometry, swivel_geometry_from_m669
from autocal.pose_fit import (
    HOME_OFFSET_PRIOR_SIGMA_MM,
    REJECT_SCALES,
    _at_far_bound,
    dataset_line_lengths,
    fit_point_positions,
    robust_loss,
)
from autocal.theoretical_ellipse import (
    anchor_opt_vec_to_matrix,
    anchors_matrix_to_opt_vec,
    get_anchor_opt_bounds,
)

# Prior sigmas for dimensionless/physical nuisance deviations around nominal config.g values.
PRIOR_K_SIGMA = 0.35
PRIOR_ALPHA_SPLIT_SIGMA = 0.35
PRIOR_RIPPLE_N_SIGMA = 1.5
PRIOR_DIR_FRIC_N_SIGMA = 1.5
PRIOR_HYST_SIGMA = 0.6
PRIOR_CREEP_MM_SIGMA = 3.0
PRIOR_RW_STEP_MM_SIGMA = 0.35

# Safety gates: skip VarPro when the nominal fit is already near-exact, and only enable extra
# hysteresis/friction/creep blocks when residual exceeds the noise floor and drops substantially.
_EXACT_BASELINE_RMS_MM = 0.015
_EXTRA_NUISANCE_RMS_FLOOR_MM = 0.015
_MIN_NUIS_SCALE_MM = 0.003
_MIN_ROBUST_SCALE_MM = 0.02
_K_BOUNDS = (0.45, 2.2)
_MAX_K_POSTERIOR_RATIO = 0.70
_PERIODS = np.arange(1, 401)


def dataset_stretch_basis(
    dataset: dict,
    lengths_mm: Optional[np.ndarray] = None,
    *,
    use_noise_mean: bool = True,
    spring_k_multiplier: float = 1.0,
) -> Optional[Dict[str, object]]:
    """Extract static-equilibrium and nuisance inputs needed by VarPro and Ds-optimal planning."""
    n = int(dataset["num_anchors"])
    config = dataset.get("config") or {}
    m666 = config.get("m666_before_data_collection") or {}
    model = FlexModel.from_m666(m666, num_axes=n, spring_k_multiplier=spring_k_multiplier)
    if model is None:
        return None
    points = [
        (s_idx, sweep, point)
        for s_idx, sweep in enumerate(dataset.get("sweeps", []))
        for point in sweep.get("data_points", [])
    ]
    if not points or not all(
        isinstance(point.get("sensor_force_n"), (int, float)) and np.isfinite(point["sensor_force_n"])
        for _, _, point in points
    ):
        return None

    if lengths_mm is None:
        lengths_mm = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    else:
        lengths_mm = np.asarray(lengths_mm, dtype=float)
    num_points = len(points)
    sensors = np.array([int(point.get("source_sensor_anchor", sweep["sensor_anchor"])) for _, sweep, point in points], dtype=int)
    known = np.full((num_points, n), np.nan)
    for idx, (_, _, point) in enumerate(points):
        for extra in point.get("extra_sensor_ranges", []):
            force = float(extra["force_n"])
            if not np.isfinite(force) or force <= 0.0:
                raise ValueError("Extra sensor requires a positive finite recorded force")
            known[idx, int(extra["anchor"])] = force
    forces = np.array([float(point["sensor_force_n"]) for _, _, point in points], dtype=float)
    weight = float(m666.get("W", 0.0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    sweep_idx = np.array([s_idx for s_idx, _, _ in points], dtype=int)

    # Approach motion sign (+1 paying out / lengthening, -1 reeling in) along each sweep pass.
    direction = np.zeros((num_points, n))
    for i in range(num_points):
        prev_ok = i > 0 and sweep_idx[i - 1] == sweep_idx[i] and sensors[i - 1] == sensors[i]
        next_ok = i + 1 < num_points and sweep_idx[i + 1] == sweep_idx[i] and sensors[i + 1] == sensors[i]
        if prev_ok:
            dl = lengths_mm[i] - lengths_mm[i - 1]
        elif next_ok:
            dl = lengths_mm[i + 1] - lengths_mm[i]
        else:
            dl = np.zeros(n)
        direction[i] = np.where(np.isfinite(dl) & (np.abs(dl) > 1e-4), np.sign(dl), 0.0)

    angles = np.full((num_points, n), np.nan)
    for idx, (_, _, point) in enumerate(points):
        raw_ang = point.get("mu") if (use_noise_mean and point.get("mu") is not None) else point.get("raw_angles_deg")
        if raw_ang is not None and len(raw_ang) == n:
            angles[idx] = np.deg2rad(np.asarray(raw_ang, dtype=float))

    ma = np.array([float(model.mechanical_advantage[a]) for a in range(n)], dtype=float)
    guy = np.array([float(model.guy_wire_lengths[a]) for a in range(n)], dtype=float)
    strands = np.array(
        [float(model.lines_per_axis[a]) if a < len(model.lines_per_axis) else 1.0 for a in range(n)],
        dtype=float,
    )
    ea_cfg = float(model.effective_stiffness_n(0))
    swivel = swivel_geometry_from_m669(config.get("m669"), n)
    spool_history = extract_spool_history(dataset) if model.has_spool_physics else None
    return {
        "model": model,
        "sensors": sensors,
        "known": known,
        "forces": forces,
        "weight": weight,
        "passive": passive,
        "direction": direction,
        "angles": angles,
        "sweep_idx": sweep_idx,
        "ma": ma,
        "guy": guy,
        "strands": strands,
        "ea_cfg": ea_cfg,
        "spool_history": spool_history,
        "swivel": swivel,
    }


def static_tension_superposition(
    anchors: np.ndarray,
    positions: np.ndarray,
    measured: np.ndarray,
    basis: dict,
    *,
    force_scale: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Exact linear static-equilibrium decomposition of cable tensions and compliance factors.

    Returns (dist, dirs_to_anchor, T_grav, T_unit, known, T_nom, g) where:
      - T_grav (P, N): line tensions produced by gravity mg alone
      - T_unit (P, N, N): line tensions per 1 N applied on known/sensor motor m
      - T_nom  (P, N): total nominal tension under gravity + known commanded/passive forces
      - g      (P, N): compliance geometry factor (U*d + Y) / (EA_cfg * U^2 * N) in mm/N
    """
    anchors = np.asarray(anchors, dtype=float)
    positions = np.asarray(positions, dtype=float)
    measured = np.asarray(measured, dtype=bool)
    num_points, n = measured.shape
    d = anchors.shape[1]

    dist, dirs_to_anchor = line_geometry(anchors, positions, swivel=basis.get("swivel"))

    known = np.where(measured, np.nan, basis["passive"])
    known = np.where(np.isfinite(basis["known"]), basis["known"] * force_scale, known)
    known[np.arange(num_points), basis["sensors"]] = basis["forces"] * force_scale
    is_known = np.isfinite(known)

    selectors = np.eye(n)[None, :, :] * is_known[:, :, None]
    matrix = np.concatenate([dirs_to_anchor.transpose(0, 2, 1), selectors], axis=1)
    transpose = matrix.transpose(0, 2, 1)
    normal = transpose @ matrix + 1e-12 * np.eye(n)
    pinv = np.linalg.solve(normal, transpose)

    t_grav = pinv[:, :, d - 1] * float(basis["weight"])
    t_unit = pinv[:, :, d:] * is_known[:, None, :]
    t_cmd = np.einsum("pnm,pm->pn", t_unit, np.nan_to_num(known, nan=0.0))
    t_nom = t_grav + t_cmd

    ma = np.asarray(basis["ma"], dtype=float)
    guy = np.asarray(basis["guy"], dtype=float)
    strands = np.asarray(basis["strands"], dtype=float)
    ea_cfg = float(basis["ea_cfg"])
    g = (ma[None, :] * dist + guy[None, :]) / (ea_cfg * (ma[None, :] ** 2) * strands[None, :])
    return dist, dirs_to_anchor, t_grav, t_unit, known, t_nom, g


def project_pose_orthogonal(direction: np.ndarray, measured: np.ndarray, columns: np.ndarray) -> np.ndarray:
    """Eliminate each point's d effector coordinates analytically from nuisance/geometric columns."""
    d = direction.shape[2]
    normal = np.einsum("pni,pnj->pij", direction, direction) + 1e-9 * np.eye(d)
    step = np.linalg.solve(normal, np.einsum("pni,pnq->piq", direction, columns))
    return (columns - np.einsum("pni,piq->pnq", direction, step)) * measured[:, :, None]


def _geometric_jacobian(
    anchors: np.ndarray,
    positions: np.ndarray,
    measured: np.ndarray,
    machine_type: str,
    num_offsets: int,
    *,
    swivel: Optional[dict] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Analytic pose-eliminated Jacobian of range residuals w.r.t. reduced anchors and home offsets."""
    n, d = anchors.shape
    anchor_vec = anchors_matrix_to_opt_vec(anchors, machine_type)
    _, dir_to_anchor = line_geometry(anchors, positions, swivel=swivel)
    direction = -dir_to_anchor * measured[:, :, None]
    if swivel is None:
        mapping = np.empty((n, d, anchor_vec.size))
        for k in range(anchor_vec.size):
            step = np.zeros(anchor_vec.size)
            step[k] = 0.5
            mapping[:, :, k] = (
                anchor_opt_vec_to_matrix(anchor_vec + step, machine_type, n, d)
                - anchor_opt_vec_to_matrix(anchor_vec - step, machine_type, n, d)
            )
        home_dir = anchors / np.maximum(np.linalg.norm(anchors, axis=1), 1e-9)[:, None]
        j_anchor = np.einsum("pnd,ndk->pnk", -direction - home_dir[None, :, :] * measured[:, :, None], mapping)
    else:
        j_anchor = np.empty((len(positions), n, anchor_vec.size))
        for k in range(anchor_vec.size):
            step = np.zeros(anchor_vec.size)
            step[k] = 0.5
            ap = anchor_opt_vec_to_matrix(anchor_vec + step, machine_type, n, d)
            am = anchor_opt_vec_to_matrix(anchor_vec - step, machine_type, n, d)
            dp = line_geometry(ap, positions, swivel=swivel)[0] - home_line_lengths(ap, swivel=swivel)[None, :]
            dm = line_geometry(am, positions, swivel=swivel)[0] - home_line_lengths(am, swivel=swivel)[None, :]
            j_anchor[:, :, k] = (dp - dm) * measured
    if num_offsets:
        j_off = np.broadcast_to(-np.eye(n)[:, -num_offsets:], (len(positions), n, num_offsets)) * measured[:, :, None]
        j_geom = np.concatenate([j_anchor, j_off], axis=2)
    else:
        j_geom = j_anchor
    return project_pose_orthogonal(direction, measured, j_geom), direction


def _detect_ripple_period(
    r0: np.ndarray,
    j_geom: np.ndarray,
    t_unit: np.ndarray,
    known: np.ndarray,
    g: np.ndarray,
    angles: np.ndarray,
    direction: np.ndarray,
    measured: np.ndarray,
    groups: np.ndarray,
    kept: np.ndarray,
    sensor_motors: np.ndarray,
) -> Optional[Tuple[int, float]]:
    """Fast blind spatial-harmonic scan (1..400 periods/turn) validated by leave-one-sweep-out CV."""
    num_points, n = measured.shape
    num_motors = len(sensor_motors)
    if num_motors == 0 or not np.all(np.isfinite(angles[kept][:, sensor_motors])):
        return None
    if len(np.unique(groups[kept])) < 4:
        return None

    b_force = np.stack(
        [t_unit[:, :, m] * np.isfinite(known[:, m])[:, None] * g for m in sensor_motors],
        axis=2,
    )
    a_force = project_pose_orthogonal(direction, measured, b_force)
    mask = (kept[:, None] & measured).ravel()
    y = r0.ravel()[mask]
    g_mat = j_geom.reshape(-1, j_geom.shape[2])[mask]
    a_f = a_force.reshape(-1, num_motors)[mask]
    pt_idx = np.repeat(np.arange(num_points), n)[mask]
    grp = groups[pt_idx]
    theta = (angles[pt_idx][:, sensor_motors]) % (2.0 * np.pi)

    mad = float(np.median(np.abs(y - np.median(y))))
    sc = max(1.4826 * mad, _MIN_ROBUST_SCALE_MM)
    w = 1.0 / np.sqrt(1.0 + (y / (3.0 * sc)) ** 2)

    def scan_candidates(sel: np.ndarray, cand_periods: np.ndarray) -> np.ndarray:
        gw = g_mat[sel] * w[sel, None]
        yw = y[sel] * w[sel]
        u, sv, _ = np.linalg.svd(gw, full_matrices=False)
        u = u[:, sv > 1e-6 * sv[0]] if sv.size else u
        yr = yw - u @ (u.T @ yw)
        af_w = a_f[sel] * w[sel, None]
        th_s = theta[sel]
        gains = np.empty(len(cand_periods))
        for start in range(0, len(cand_periods), 50):
            ps = cand_periods[start:start + 50]
            ph = th_s[:, :, None] * ps[None, None, :]
            h_blk = np.concatenate(
                [af_w[:, :, None] * np.sin(ph), af_w[:, :, None] * np.cos(ph)],
                axis=1,
            ).transpose(0, 2, 1)
            flat = h_blk.reshape(len(yr), -1)
            flat = flat - u @ (u.T @ flat)
            proj = flat.reshape(h_blk.shape)
            lhs = np.einsum("rhi,rhj->hij", proj, proj) + 1e-6 * np.eye(2 * num_motors)
            rhs = np.einsum("rhi,r->hi", proj, yr)
            beta = np.linalg.solve(lhs, rhs[..., None])[..., 0]
            gains[start:start + 50] = np.einsum("hi,hi->h", rhs, beta)
        return gains, float(yr @ yr)

    all_sel = np.ones(len(y), dtype=bool)
    scores_all, denom_all = scan_candidates(all_sel, _PERIODS)
    top_idx = np.argsort(scores_all)[-8:][::-1]
    full_period = int(_PERIODS[top_idx[0]])
    train_r2 = float(scores_all[top_idx[0]]) / max(denom_all, 1e-12)
    if train_r2 < 0.35:
        return None

    top_cands = _PERIODS[top_idx]
    uniq = np.unique(grp)
    ph_full = theta * full_period
    h_full = np.concatenate([a_f * np.sin(ph_full), a_f * np.cos(ph_full)], axis=1)
    base_sse = corr_sse = 0.0
    agree = 0
    for g_id in uniq:
        tr = grp != g_id
        te = ~tr
        fold_gains, _ = scan_candidates(tr, top_cands)
        if int(top_cands[int(np.argmax(fold_gains))]) == full_period:
            agree += 1
        gw = g_mat[tr] * w[tr, None]
        yw = y[tr] * w[tr]
        b0 = np.linalg.lstsq(gw, yw, rcond=1e-6)[0]
        x_tr = np.hstack([g_mat[tr], h_full[tr]]) * w[tr, None]
        reg = np.r_[np.full(g_mat.shape[1], 1e-10), np.full(2 * num_motors, 1e-5)]
        b1 = np.linalg.solve(x_tr.T @ x_tr + np.diag(reg), x_tr.T @ yw)
        base_sse += float(np.sum((y[te] - g_mat[te] @ b0) ** 2))
        corr_sse += float(np.sum((y[te] - np.hstack([g_mat[te], h_full[te]]) @ b1) ** 2))

    cv_gain = 1.0 - corr_sse / max(base_sse, 1e-12)
    required_agree = len(uniq) if len(uniq) <= 10 else max(len(uniq) - 1, int(np.ceil(0.85 * len(uniq))))
    if agree >= required_agree and cv_gain >= 0.65:
        return full_period, float(cv_gain)
    return None


def stretch_identifiability_diagnostics(
    anchors: np.ndarray,
    positions: np.ndarray,
    measured: np.ndarray,
    basis: dict,
    *,
    machine_type: str,
    num_offsets: int,
    sigma_mm: float,
    kept: Optional[np.ndarray] = None,
    active_a: Optional[np.ndarray] = None,
    active_prior_inv: Optional[np.ndarray] = None,
    k_grav_hat: Optional[float] = 1.0,
    k_sensor_hat: Optional[float] = None,
    has_gravity: bool = True,
    split_alpha: bool = True,
    lev_weights: Optional[np.ndarray] = None,
) -> Dict[str, object]:
    """Diagnose whether true EA (via gravity mg) separated from motor force scale alpha.

    Specifically checks the config.g transfer trap: multiplying alpha_i, F_{0,i}, c_i, and EA
    together leaves every sensor-force stretch term invariant; only gravity mg fixes absolute EA.
    `ea_estimated_n` is derived strictly from the gravity compliance `k_grav_hat` when the
    gravity-vs-sensor split is active, never from the sensor-force scale `k_sensor_hat`.
    """
    sigma = max(float(sigma_mm), _MIN_NUIS_SCALE_MM)
    mask_pts = np.ones(len(positions), dtype=bool) if kept is None else np.asarray(kept, dtype=bool)
    mask = (mask_pts[:, None] & measured).ravel()
    row_sqrt_w = (
        np.repeat(np.sqrt(np.maximum(np.asarray(lev_weights, dtype=float), 0.0)), anchors.shape[0])[mask]
        if lev_weights is not None
        else None
    )

    _, dirs_to_a, t_grav, _, _, t_nom, g = static_tension_superposition(anchors, positions, measured, basis)
    direction = -dirs_to_a * measured[:, :, None]
    j_geom, _ = _geometric_jacobian(
        anchors, positions, measured, machine_type, num_offsets, swivel=basis.get("swivel")
    )
    num_geom = j_geom.shape[2]
    num_anchor_params = num_geom - num_offsets

    # Gravity-vs-sensor split matrix: column 0 = dk_grav (1/EA), column 1 = dalpha_sensor (alpha/EA).
    pos_mask = (t_nom > 0.0).astype(float)
    t_grav_eff = t_grav * pos_mask
    t_cmd_eff = (t_nom - t_grav) * pos_mask
    b_split = np.stack([g * t_grav_eff, g * t_cmd_eff], axis=2) * measured[:, :, None]
    a_split = project_pose_orthogonal(direction, measured, b_split)

    j_split = np.hstack([j_geom.reshape(-1, num_geom)[mask], -a_split.reshape(-1, 2)[mask]])
    if row_sqrt_w is not None:
        j_split = j_split * row_sqrt_w[:, None]
    i_split = (j_split.T @ j_split) / (sigma ** 2)
    prior_split = np.r_[
        np.full(num_anchor_params, 1e-6),
        np.full(num_offsets, 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2),
        1.0 / PRIOR_K_SIGMA ** 2,
        1.0 / PRIOR_ALPHA_SPLIT_SIGMA ** 2,
    ]
    cov_split = np.linalg.pinv(i_split + np.diag(prior_split))
    ea_grav_std = float(np.sqrt(max(cov_split[num_geom, num_geom], 0.0)))
    alpha_sensor_std = float(np.sqrt(max(cov_split[num_geom + 1, num_geom + 1], 0.0)))

    # Joint posterior uncertainty of the active fitted stretch basis on kept measurements.
    if active_a is not None and active_a.shape[2] > 0:
        x_nuis = active_a.reshape(-1, active_a.shape[2])[mask]
        j_act = np.hstack([j_geom.reshape(-1, num_geom)[mask], -x_nuis])
        if row_sqrt_w is not None:
            j_act = j_act * row_sqrt_w[:, None]
        i_act = (j_act.T @ j_act) / (sigma ** 2)
        p_inv = np.asarray(active_prior_inv, dtype=float)
        prior_act = np.r_[
            np.full(num_anchor_params, 1e-6),
            np.full(num_offsets, 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2),
            p_inv ** 2,
        ]
        cov_act = np.linalg.pinv(i_act + np.diag(prior_act))
        if split_alpha and has_gravity and active_a.shape[2] >= 2:
            # Active columns 0, 1 are [dk_grav, dalpha_scale] where k_sensor = 1 + dk_grav + dalpha_scale.
            c_grav = cov_act[num_geom, num_geom]
            c_sens = (
                cov_act[num_geom, num_geom]
                + cov_act[num_geom + 1, num_geom + 1]
                + 2.0 * cov_act[num_geom, num_geom + 1]
            )
            ea_grav_std = max(ea_grav_std, float(np.sqrt(max(c_grav, 0.0))))
            k_joint_std = float(np.sqrt(max(c_sens, 0.0)))
        else:
            k_joint_std = float(np.sqrt(max(cov_act[num_geom, num_geom], 0.0)))
        svals = np.linalg.svd(x_nuis, compute_uv=False).tolist()
    else:
        k_joint_std = alpha_sensor_std
        svals = np.linalg.svd(a_split.reshape(-1, 2)[mask], compute_uv=False).tolist()

    k_sensor_eff = float(1.0 if (k_sensor_hat is None and k_grav_hat is None) else (k_grav_hat if k_sensor_hat is None else k_sensor_hat))
    # True EA is separated for config.g only when the gravity-vs-sensor split is active and gravity
    # shrinks the 1/EA posterior std well below both the prior and the 10% transfer threshold.
    ea_separated = bool(split_alpha and has_gravity and ea_grav_std < min(0.5 * PRIOR_K_SIGMA, 0.10))
    k_grav_out = float(k_grav_hat) if (split_alpha and has_gravity and k_grav_hat is not None) else None
    ea_estimated_n = float(basis["ea_cfg"] / k_grav_out) if (k_grav_out is not None and k_grav_out > 0.0) else None
    alpha_hat = float(k_sensor_eff / k_grav_out) if (k_grav_out is not None and k_grav_out > 0.0) else None
    return {
        "k_hat": k_sensor_eff,
        "k_grav_hat": k_grav_out,
        "k_sensor_hat": k_sensor_eff,
        "alpha_hat": alpha_hat,
        "k_joint_std": k_joint_std,
        "ea_estimated_n": ea_estimated_n,
        "ea_grav_std": ea_grav_std,
        "alpha_sensor_std": alpha_sensor_std,
        "ea_separated_for_config_g": ea_separated,
        "nuisance_singular_values": [float(v) for v in svals],
    }


def _estimate_force_leverage_weights(
    r_corr: np.ndarray,
    leverage: np.ndarray,
    kept: np.ndarray,
) -> Optional[np.ndarray]:
    """Return heteroscedastic weights downweighting force-amplified poses under random friction."""
    if int(np.sum(kept)) < 20:
        return None
    lev_k = leverage[kept]
    if float(np.percentile(lev_k, 80)) <= 1.4 * max(float(np.percentile(lev_k, 20)), 1e-9):
        return None
    pt_misfit = np.linalg.norm(r_corr, axis=1)
    hi = kept & (leverage >= np.percentile(lev_k, 70))
    lo = kept & (leverage <= np.percentile(lev_k, 30))
    s_hi = float(np.median(pt_misfit[hi]))
    s_lo = float(np.median(pt_misfit[lo]))
    l_hi = float(np.median(leverage[hi]))
    l_lo = float(np.median(leverage[lo]))
    # Only activate when high-leverage points actually show materially larger random residuals.
    if s_hi <= 1.35 * max(s_lo, _MIN_ROBUST_SCALE_MM) or l_hi <= 1.2 * max(l_lo, 1e-9):
        return None
    sigma_f_sq = max(0.0, (s_hi ** 2 - s_lo ** 2) / max(l_hi ** 2 - l_lo ** 2, 1e-9))
    sigma_e_sq = max(_MIN_ROBUST_SCALE_MM ** 2, s_lo ** 2 - sigma_f_sq * (l_lo ** 2))
    var_p = sigma_e_sq + sigma_f_sq * (leverage ** 2)
    return float(np.median(var_p[kept])) / var_p


def _solve_varpro_inner(
    r0: np.ndarray,
    a_mat: np.ndarray,
    p_inv: np.ndarray,
    nuis_scale: float,
    w_pt: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Solve the regularized weighted VarPro inner problem:
        min_psi sum_p w_pt[p] * ||r0[p] - A[p] @ psi||^2 + ||(nuis_scale * p_inv) * psi||^2.
    Uses sqrt(w_pt) so the normal equations Xw.T @ Xw apply statistical weights `w_pt`
    (matching the outer objective's sqrt(lev_weights) residual scaling).
    """
    q = a_mat.shape[2]
    if q == 0:
        return r0, np.zeros(0), np.zeros(0)
    n = r0.shape[1]
    y = r0.ravel()
    x_mat = a_mat.reshape(-1, q)
    if w_pt is not None:
        sqrt_w = np.sqrt(np.maximum(np.repeat(w_pt, n), 0.0))
        xw, yw = x_mat * sqrt_w[:, None], y * sqrt_w
    else:
        xw, yw = x_mat, y
    lam = (nuis_scale * p_inv) ** 2
    psi = np.linalg.solve(xw.T @ xw + np.diag(lam), xw.T @ yw)
    r_corr = (y - x_mat @ psi).reshape(r0.shape)
    prior_res = (nuis_scale * p_inv) * psi
    return r_corr, psi, prior_res


def fit_pose_varpro(
    dataset: dict,
    baseline: dict,
    *,
    use_noise_mean: bool = True,
    allow_ripple: bool = True,
    fixed_ripple_period: Optional[int] = None,
    allow_real_world_nuisance: bool = True,
    home_offset_prior_sigma_mm: float = HOME_OFFSET_PRIOR_SIGMA_MM,
    spring_k_multiplier: float = 1.0,
) -> Optional[Dict[str, object]]:
    """Refine a baseline point fit with guarded Golub-Pereyra VarPro over stretch & nuisances.

    Returns an upgraded fit dict (same keys as fit_pose_least_squares plus `varpro` diagnostics),
    or None if the baseline is already exact or VarPro is not supported / fails safety guards.
    """
    if not baseline.get("success") or not baseline.get("stretch_corrected"):
        return None
    rms_base = float(baseline.get("rms_mm", np.nan))
    if not np.isfinite(rms_base) or rms_base < _EXACT_BASELINE_RMS_MM:
        return None

    machine_type = str(dataset["machine_type"])
    n, d = int(dataset["num_anchors"]), int(dataset["dimensions"])
    lengths = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    measured = np.isfinite(lengths)
    num_points = len(lengths)
    basis = dataset_stretch_basis(
        dataset, lengths, use_noise_mean=use_noise_mean, spring_k_multiplier=spring_k_multiplier
    )
    if basis is None or num_points < n + d + 2:
        return None

    num_sweeps = len(dataset.get("sweeps", []))
    sweep_idx = basis["sweep_idx"]
    progress = sweep_idx.astype(float) / max(1, num_sweeps - 1)
    cmd_mat = basis["known"].copy()
    cmd_mat[np.arange(num_points), basis["sensors"]] = basis["forces"]
    sensor_motors = np.flatnonzero(np.isfinite(cmd_mat).any(axis=0))

    fit_home_offsets = bool(baseline.get("fit_home_offsets", False)) and not baseline.get("home_offsets_skipped")
    num_offsets = n - d if fit_home_offsets else 0
    offset_lines = list(range(n - num_offsets, n))
    anchor_lb, anchor_ub = get_anchor_opt_bounds(machine_type, n, d)
    num_anchor_params = anchor_lb.size

    kept_init = np.ones(num_points, dtype=bool)
    rejected_base = baseline.get("rejected_points")
    if rejected_base is not None and len(rejected_base):
        kept_init[np.asarray(rejected_base, dtype=int)] = False

    stretch_nom = {
        "model": basis["model"],
        "sensors": basis["sensors"],
        "known": basis["known"],
        "forces": basis["forces"],
        "weight": basis["weight"],
        "passive": basis["passive"],
        "spool_history": basis.get("spool_history"),
        "swivel": basis.get("swivel"),
    }
    a0 = np.asarray(baseline["anchors"], dtype=float)
    o0 = np.asarray(baseline["home_offsets_mm"], dtype=float)
    pos0, abs0 = fit_point_positions(a0, lengths, o0, stretch=stretch_nom)
    dist0, dirs_to_a0, t_grav0, t_unit0, known0, t_nom0, g0 = static_tension_superposition(a0, pos0, measured, basis)
    r0_base = np.where(measured, dist0 - abs0, 0.0)
    j_geom0, dir0 = _geometric_jacobian(a0, pos0, measured, machine_type, num_offsets, swivel=basis.get("swivel"))
    has_gravity = bool(basis["weight"] > 0.0 and np.max(np.abs(t_grav0)) > 1e-6)
    pos_mask0 = (t_nom0 > 0.0).astype(float)
    if has_gravity:
        b_k0 = np.stack([g0 * t_nom0 * pos_mask0, g0 * (t_nom0 - t_grav0) * pos_mask0], axis=2) * measured[:, :, None]
    else:
        b_k0 = (g0 * np.maximum(t_nom0, 0.0) * measured)[:, :, None]
    a_k0 = project_pose_orthogonal(dir0, measured, b_k0)
    j_geom_k0 = np.concatenate([j_geom0, -a_k0], axis=2)

    ripple_period: Optional[int] = fixed_ripple_period
    ripple_cv_gain: Optional[float] = None
    if ripple_period is None and allow_ripple:
        detected = _detect_ripple_period(
            r0_base, j_geom_k0, t_unit0, known0, g0,
            basis["angles"], dir0, measured, sweep_idx, kept_init, sensor_motors,
        )
        if detected is not None:
            ripple_period, ripple_cv_gain = detected

    def build_blocks(anchors: np.ndarray, offsets: np.ndarray, kept_mask: Optional[np.ndarray], active_blocks: Tuple[str, ...]):
        used = lengths if kept_mask is None else np.where(kept_mask[:, None], lengths, np.nan)
        meas = np.isfinite(used)
        positions, abs_corr = fit_point_positions(anchors, used, offsets, stretch=stretch_nom)
        dist, dirs_to_a, t_grav, t_unit, known, t_nom, g = static_tension_superposition(anchors, positions, meas, basis)
        direction = -dirs_to_a * meas[:, :, None]
        r0 = np.where(meas, dist - abs_corr, 0.0)

        t_sens = t_unit[np.arange(num_points), :, basis["sensors"]]
        sens_perp = project_pose_orthogonal(direction, meas, (t_sens * g * meas)[:, :, None])[:, :, 0]
        leverage = np.linalg.norm(sens_perp, axis=1)

        cols, p_inv, names = [], [], []
        if "k" in active_blocks:
            pos_mask = (t_nom > 0.0).astype(float)
            if has_gravity and "alpha_split" in active_blocks:
                # Hierarchical split: dk_grav scales (T_grav + T_cmd), dalpha_scale scales T_cmd only,
                # so k_grav = 1 + dk_grav = EA_cfg/EA_true and k_sensor = 1 + dk_grav + dalpha_scale = alpha * EA_cfg/EA_true.
                cols.append(g * t_nom * pos_mask)
                p_inv.append(1.0 / PRIOR_K_SIGMA)
                names.append("dk_grav")
                cols.append(g * (t_nom - t_grav) * pos_mask)
                p_inv.append(1.0 / PRIOR_ALPHA_SPLIT_SIGMA)
                names.append("dalpha_scale")
            else:
                cols.append(g * np.maximum(t_nom, 0.0))
                p_inv.append(1.0 / PRIOR_K_SIGMA)
                names.append("dk_common")
        if "ripple" in active_blocks and ripple_period is not None:
            ph = (basis["angles"] % (2.0 * np.pi)) * ripple_period
            for m in sensor_motors:
                act = np.isfinite(known[:, m]).astype(float)
                base_col = t_unit[:, :, m] * act[:, None] * g
                cols.append(base_col * np.sin(ph[:, m])[:, None])
                p_inv.append(1.0 / PRIOR_RIPPLE_N_SIGMA)
                names.append(f"rip_s_m{m}")
                cols.append(base_col * np.cos(ph[:, m])[:, None])
                p_inv.append(1.0 / PRIOR_RIPPLE_N_SIGMA)
                names.append(f"rip_c_m{m}")
        if "dir_fric" in active_blocks:
            for m in sensor_motors:
                act = np.isfinite(known[:, m]).astype(float) * basis["direction"][:, m]
                cols.append(t_unit[:, :, m] * act[:, None] * g)
                p_inv.append(1.0 / PRIOR_DIR_FRIC_N_SIGMA)
                names.append(f"fdir_m{m}")
        if "hyst" in active_blocks:
            t_pos = np.maximum(t_nom, 0.0)
            t_peak = np.maximum.accumulate(t_pos, axis=0)
            cols.append(g * (t_peak - t_pos))
            p_inv.append(1.0 / PRIOR_HYST_SIGMA)
            names.append("dhyst_ret")
        if "creep" in active_blocks and num_sweeps >= 4:
            for a_i in range(n):
                col = np.zeros((num_points, n))
                col[:, a_i] = progress
                cols.append(col)
                p_inv.append(1.0 / PRIOR_CREEP_MM_SIGMA)
                names.append(f"creep_a{a_i}")
        if "rw_creep" in active_blocks and num_sweeps >= 4:
            for s_i in range(1, num_sweeps):
                after = (sweep_idx >= s_i).astype(float)
                for a_i in range(n):
                    col = np.zeros((num_points, n))
                    col[:, a_i] = after
                    cols.append(col)
                    p_inv.append(1.0 / PRIOR_RW_STEP_MM_SIGMA)
                    names.append(f"rw_s{s_i}_a{a_i}")

        if not cols:
            return r0, np.zeros((num_points, n, 0)), np.zeros(0), names, positions, meas, leverage
        b_mat = np.stack(cols, axis=2) * meas[:, :, None]
        a_mat = project_pose_orthogonal(direction, meas, b_mat)
        return r0, a_mat, np.asarray(p_inv, dtype=float), names, positions, meas, leverage

    active_list = ["k"]
    if ripple_period is not None:
        active_list.append("ripple")

    mask_f = (kept_init[:, None] & measured).ravel()
    y_lin = r0_base.ravel()[mask_f]
    g_lin = j_geom0.reshape(-1, j_geom0.shape[2])[mask_f]
    sc0 = max(1.4826 * float(np.median(np.abs(y_lin - np.median(y_lin)))), _MIN_ROBUST_SCALE_MM)

    def _lin_fit(blocks: Sequence[str]) -> Tuple[float, float, np.ndarray]:
        _, a_c, p_c, _, _, _, _ = build_blocks(a0, o0, None, tuple(blocks))
        ac_m = a_c.reshape(-1, a_c.shape[2])[mask_f]
        x_mat = np.hstack([g_lin, ac_m])
        reg = np.r_[np.full(g_lin.shape[1], 1e-10), (sc0 * p_c) ** 2]
        beta = np.linalg.solve(x_mat.T @ x_mat + np.diag(reg), x_mat.T @ y_lin)
        res = y_lin - x_mat @ beta
        sse = float(np.sum(res ** 2))
        return float(np.sqrt(sse / max(int(np.sum(kept_init)), 1))), sse, res

    # Gate real-world nuisance blocks (hysteresis, directional friction, gradual/step creep)
    # strictly when residual exceeds the noise floor and drops materially on linearized fit.
    if allow_real_world_nuisance and num_sweeps >= 4:
        cur_rms, cur_sse, _ = _lin_fit(active_list)
        if cur_rms > _EXTRA_NUISANCE_RMS_FLOOR_MM:
            # Forward-select the strongest genuine nuisance block first (highest SSE drop)
            # rather than a fixed order so true friction/creep isn't shadowed by hysteresis.
            remaining = [("hyst", 0.25), ("dir_fric", 0.35), ("creep", 0.45)]
            while remaining and cur_rms > _EXTRA_NUISANCE_RMS_FLOOR_MM and cur_sse > 0.0:
                best = None
                for blk, min_drop in remaining:
                    cand_rms, cand_sse, _ = _lin_fit([*active_list, blk])
                    drop = 1.0 - cand_sse / cur_sse
                    if drop >= min_drop and (best is None or drop > best[0]):
                        best = (drop, blk, cand_rms, cand_sse)
                if best is None:
                    break
                _, blk, cur_rms, cur_sse = best
                active_list.append(blk)
                remaining = [(b, t) for b, t in remaining if b != blk]
            # Escalate to per-sweep random-walk creep for sudden mid-run permanent stretch jumps
            # when residual is still large (>0.12 mm) and RW-creep explains >55% of remaining SSE.
            if cur_rms > 0.12:
                trial_list = [b for b in active_list if b != "creep"] + ["rw_creep"]
                rw_rms, rw_sse, _ = _lin_fit(trial_list)
                if cur_sse > 0.0 and (1.0 - rw_sse / cur_sse) >= 0.55:
                    active_list = trial_list
                    cur_rms, cur_sse = rw_rms, rw_sse

    # Gate the gravity-vs-sensor split (`dk_grav` vs `dalpha_scale`) on either gravity identifiability
    # or strong direct residual evidence of alpha != 1, avoiding weak-gravity collinearity on single-force runs.
    ea_sep_threshold = min(0.5 * PRIOR_K_SIGMA, 0.10)
    if has_gravity:
        cur_rms, cur_sse, res_lin = _lin_fit(active_list)
        mad_lin = 1.4826 * float(np.median(np.abs(res_lin - np.median(res_lin))))
        sigma_lin = max(mad_lin, _MIN_NUIS_SCALE_MM)
        diag0 = stretch_identifiability_diagnostics(
            a0, pos0, measured, basis,
            machine_type=machine_type, num_offsets=num_offsets,
            sigma_mm=sigma_lin, kept=kept_init, has_gravity=True, split_alpha=True,
        )
        split_identifiable = bool(diag0["ea_grav_std"] < ea_sep_threshold)
        split_needed = False
        if not split_identifiable and cur_rms > _EXTRA_NUISANCE_RMS_FLOOR_MM and cur_sse > 0.0:
            _, split_sse, _ = _lin_fit([*active_list, "alpha_split"])
            split_needed = (1.0 - split_sse / cur_sse) >= 0.25
        if split_identifiable or split_needed:
            active_list.append("alpha_split")

    active_blocks = tuple(active_list)

    def unpack(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        anchors = anchor_opt_vec_to_matrix(x[:num_anchor_params], machine_type, n, d)
        offsets = np.zeros(n)
        offsets[offset_lines] = x[num_anchor_params:num_anchor_params + num_offsets]
        return anchors, offsets

    start = np.concatenate([np.clip(anchors_matrix_to_opt_vec(a0, machine_type), anchor_lb, anchor_ub), o0[offset_lines]])
    bounds = (
        np.concatenate([anchor_lb, np.full(num_offsets, -np.inf)]),
        np.concatenate([anchor_ub, np.full(num_offsets, np.inf)]),
    )
    x_scale = np.concatenate([np.full(num_anchor_params, 100.0), np.ones(num_offsets)])

    def _run_varpro_solve(blocks: Tuple[str, ...], x_start: np.ndarray):
        _, _, _, c_names, _, _, _ = build_blocks(a0, o0, None, blocks)
        num_prior = num_offsets + len(c_names)
        has_creep = ("creep" in blocks) or ("rw_creep" in blocks)
        split_alpha = bool(has_gravity and "alpha_split" in blocks)

        def residuals_fn(x, prior_weight, nuis_scale, kept=None, pt_weights=None, lev_weights=None):
            anchors, offsets = unpack(x)
            r0, a_mat, p_inv, _, _, _, _ = build_blocks(anchors, offsets, kept, blocks)
            w_in = pt_weights
            if lev_weights is not None:
                w_in = lev_weights if w_in is None else w_in * lev_weights
            r_corr, _, prior_res = _solve_varpro_inner(r0, a_mat, p_inv, nuis_scale, w_pt=w_in)
            r_out = r_corr * np.sqrt(lev_weights)[:, None] if lev_weights is not None else r_corr
            return np.concatenate([r_out.ravel(), offsets[offset_lines] * prior_weight, prior_res])

        fit_res = least_squares(
            residuals_fn,
            x_start,
            args=(_MIN_ROBUST_SCALE_MM / home_offset_prior_sigma_mm, 0.05),
            bounds=bounds,
            x_scale=x_scale,
            loss=robust_loss("soft_l1", num_prior),
            f_scale=1.0,
            max_nfev=80,
        )

        kept = np.ones(num_points, dtype=bool)
        scale = 0.05
        nuis_scale = 0.05
        pt_weights = np.ones(num_points)
        lev_weights: Optional[np.ndarray] = None
        for _ in range(3):
            ac, oc = unpack(fit_res.x)
            r0, a_mat, p_inv, _, _, _, lev = build_blocks(ac, oc, kept, blocks)
            r_corr, _, _ = _solve_varpro_inner(r0, a_mat, p_inv, nuis_scale, w_pt=pt_weights * kept)
            lev_weights = _estimate_force_leverage_weights(r_corr, lev, kept) if allow_real_world_nuisance else None
            eff_r = r_corr * np.sqrt(lev_weights)[:, None] if lev_weights is not None else r_corr
            lr = eff_r[kept][measured[kept]]
            mad = 1.4826 * float(np.median(np.abs(lr - np.median(lr))))
            scale = max(mad, _MIN_ROBUST_SCALE_MM)
            nuis_scale = max(mad, 0.01 if has_creep else _MIN_NUIS_SCALE_MM)
            eff_pt = np.linalg.norm(eff_r, axis=1)
            pt_weights = 1.0 / (1.0 + (eff_pt / (3.0 * scale)) ** 2)
            fit_res = least_squares(
                residuals_fn,
                fit_res.x,
                args=(scale / home_offset_prior_sigma_mm, nuis_scale, kept, pt_weights * kept, lev_weights),
                bounds=bounds,
                x_scale=x_scale,
                loss=robust_loss("arctan", num_prior),
                f_scale=3.0 * scale,
                max_nfev=60,
            )
            ac, oc = unpack(fit_res.x)
            r0, a_mat, p_inv, _, _, _, _ = build_blocks(ac, oc, None, blocks)
            w_in = pt_weights * kept * (lev_weights if lev_weights is not None else 1.0)
            r_corr, _, _ = _solve_varpro_inner(r0, a_mat, p_inv, nuis_scale, w_pt=w_in)
            eff_pt = np.linalg.norm(r_corr * (np.sqrt(lev_weights)[:, None] if lev_weights is not None else 1.0), axis=1)
            within = eff_pt <= REJECT_SCALES * scale
            if np.array_equal(within, kept) or within.mean() < 0.5:
                break
            kept = within

        anchors_final, offsets_final = unpack(fit_res.x)
        r0, a_mat, p_inv, col_names, pos_f, meas_f, _ = build_blocks(anchors_final, offsets_final, None, blocks)
        w_in = pt_weights * kept * (lev_weights if lev_weights is not None else 1.0)
        r_corr, psi, _ = _solve_varpro_inner(r0, a_mat, p_inv, nuis_scale, w_pt=w_in)
        per_point = np.linalg.norm(r_corr, axis=1)
        rms_vp = float(np.sqrt(np.mean(per_point[kept] ** 2)))
        psi_dict = dict(zip(col_names, psi.tolist()))
        if split_alpha:
            k_grav_hat: Optional[float] = 1.0 + float(psi_dict.get("dk_grav", 0.0))
            k_sensor_hat = float(k_grav_hat) + float(psi_dict.get("dalpha_scale", 0.0))
        else:
            k_grav_hat = None
            k_sensor_hat = 1.0 + float(psi_dict.get("dk_common", 0.0))

        diag = stretch_identifiability_diagnostics(
            anchors_final,
            pos_f,
            meas_f,
            basis,
            machine_type=machine_type,
            num_offsets=num_offsets,
            sigma_mm=nuis_scale,
            kept=kept,
            active_a=a_mat,
            active_prior_inv=p_inv,
            k_grav_hat=k_grav_hat,
            k_sensor_hat=k_sensor_hat,
            has_gravity=has_gravity,
            split_alpha=split_alpha,
            lev_weights=lev_weights,
        )
        diag.update({
            "active_blocks": list(blocks),
            "ripple_period": ripple_period,
            "ripple_cv_gain": ripple_cv_gain,
            "force_leverage_weighted": lev_weights is not None,
            "psi": psi_dict,
        })
        ok = not (
            not fit_res.success
            or not (_K_BOUNDS[0] <= k_sensor_hat <= _K_BOUNDS[1])
            or (split_alpha and k_grav_hat is not None and not (_K_BOUNDS[0] <= k_grav_hat <= _K_BOUNDS[1]))
            or diag["k_joint_std"] >= _MAX_K_POSTERIOR_RATIO * PRIOR_K_SIGMA
            or rms_vp > 1.05 * rms_base
            or (_at_far_bound(anchors_final, dataset) and not _at_far_bound(a0, dataset))
        )
        return ok, fit_res, anchors_final, offsets_final, rms_vp, scale, per_point, kept, diag

    ok, fit, anchors_final, offsets_final, rms_vp, scale, per_point, kept, diag = _run_varpro_solve(active_blocks, start)
    # If gravity separation emerged after anchor convergence (e.g. initial residual inflated sigma_lin),
    # perform one warm-started polish with `alpha_split` so `ea_estimated_n` is separated from alpha.
    if ok and has_gravity and "alpha_split" not in active_blocks and diag["ea_grav_std"] < ea_sep_threshold:
        split_res = _run_varpro_solve((*active_blocks, "alpha_split"), fit.x)
        if split_res[0]:
            ok, fit, anchors_final, offsets_final, rms_vp, scale, per_point, kept, diag = split_res

    if not ok:
        return None

    return {
        **baseline,
        "anchors": anchors_final,
        "home_offsets_mm": offsets_final,
        "rms_mm": rms_vp,
        "robust_scale_mm": float(scale),
        "point_residuals_mm": per_point,
        "rejected_points": np.flatnonzero(~kept),
        "success": bool(fit.success),
        "status": int(fit.status),
        "message": str(fit.message),
        "varpro": diag,
    }
