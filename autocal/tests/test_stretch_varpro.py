"""Unit tests for VarPro stretch/nuisance elimination, config.g EA separation diagnostics,
real-world hysteresis/creep/friction robustness, and Schur-complement Ds-optimal planning."""
from __future__ import annotations

import numpy as np
import pytest

from autocal.active_learning import SweepConfig, generate_candidate_sweeps
from autocal.pose_fit import (
    HOME_OFFSET_PRIOR_SIGMA_MM,
    dataset_line_lengths,
    dataset_stretch,
    fit_pose_least_squares,
    has_force_contrast,
)
from autocal.pose_planning import (
    SWEEP_PRETENSION_SHARE,
    point_joint_stretch_information,
    predict_sweep_positions,
    rank_candidates_force_aware,
    schur_anchor_information,
)
from autocal.stretch_varpro import (
    dataset_stretch_basis,
    fit_pose_varpro,
    static_tension_superposition,
    stretch_identifiability_diagnostics,
)

HP5_ANCHORS = np.array([
    [0.0, -1900.0, -280.0],
    [1900.0, 0.0, -280.0],
    [0.0, 1900.0, -280.0],
    [-1900.0, 0.0, -280.0],
    [0.0, 0.0, 1900.0],
])
HP3_ANCHORS = np.array([
    [0.0, -1900.0, -280.0],
    [1645.4, 950.0, -280.0],
    [-1645.4, 950.0, -280.0],
    [0.0, 0.0, 1900.0],
])


def _clean_hp5_dataset(
    *,
    true_ea_scale: float = 1.0,
    hyst_retained_frac: float = 0.0,
    dir_coulomb_n: float = 0.0,
    creep_per_sweep_mm: float = 0.0,
    ripple_amp_n: float = 0.0,
    ripple_period: int = 50,
    force_levels: tuple[float, ...] = (4.0,),
) -> dict:
    """Clean exact-geometry HP5r dataset with consistent fixed-line deltas and physical nuisances."""
    anchors = HP5_ANCHORS
    ma = np.array([2.0, 2.0, 2.0, 2.0, 4.0])
    strands = np.array([2.0, 2.0, 2.0, 2.0, 4.0])
    guy = np.array([429.1, 429.1, 429.1, 429.1, 352.3])
    ea_cfg = 20000.0
    ea_true = ea_cfg * true_ea_scale
    weight_n = 0.5 * 9.81
    home = np.linalg.norm(anchors, axis=1)
    mm_per_deg = [0.25] * 5

    sweeps = []
    running_peak_t = np.zeros(5)
    creep_line = np.array([0.0, 0.8, 0.4, 0.2, 1.0]) * creep_per_sweep_mm

    sweep_specs = [
        (low, z_mid, f_cmd)
        for f_cmd in force_levels
        for z_mid in (100.0, 500.0)
        for low in range(4)
    ]

    for s_idx, (low, z_mid, f_cmd) in enumerate(sweep_specs):
        drive, sensor, passive = (low + 1) % 4, (low + 3) % 4, (low + 2) % 4
        pts = []
        tangent = anchors[drive] - anchors[sensor]
        tangent = tangent / np.linalg.norm(tangent)
        base_xy = 0.06 * anchors[low][:2]
        for pass_dir, actual_sensor in ((1.0, sensor), (-1.0, drive)):
            # Asymmetric arc so every step has a well-defined nonzero motion sign on every line.
            t_vals = np.linspace(-130.0, 210.0, 5) * pass_dir
            for idx_t, t_val in enumerate(t_vals):
                pos = np.array([base_xy[0], base_xy[1], z_mid]) + t_val * tangent
                pos[2] += 0.06 * (t_val ** 2) / 180.0
                vec = anchors - pos
                dist = np.linalg.norm(vec, axis=1)
                dirs = (vec / dist[:, None]).T
                raw_angles = (dist - home) / np.asarray(mm_per_deg)
                th = np.deg2rad(raw_angles)
                other_t = t_vals[idx_t - 1] if idx_t > 0 else t_vals[idx_t + 1]
                other_pos = np.array([base_xy[0], base_xy[1], z_mid]) + other_t * tangent
                other_pos[2] += 0.06 * (other_t ** 2) / 180.0
                other_dist = np.linalg.norm(anchors - other_pos, axis=1)
                dl_sens = (dist[actual_sensor] - other_dist[actual_sensor]) if idx_t > 0 else (other_dist[actual_sensor] - dist[actual_sensor])
                move_sign = float(np.sign(dl_sens)) if abs(dl_sens) > 1e-3 else 0.0
                dl_extra = (dist[passive] - other_dist[passive]) if idx_t > 0 else (other_dist[passive] - dist[passive])
                extra_sign = float(np.sign(dl_extra)) if abs(dl_extra) > 1e-3 else 0.0

                f_real = f_cmd + dir_coulomb_n * move_sign + ripple_amp_n * np.sin(ripple_period * th[actual_sensor])
                extra_f_cmd = 0.5 * f_cmd
                extra_f_real = extra_f_cmd + dir_coulomb_n * extra_sign
                mat = np.vstack([dirs, np.eye(5)[actual_sensor], np.eye(5)[passive]])
                tensions = np.linalg.solve(mat, [0.0, 0.0, weight_n, f_real, extra_f_real])
                assert np.all(tensions > 0.0)

                running_peak_t = np.maximum(running_peak_t, tensions)
                g_true = (ma * dist + guy) / (ea_true * (ma ** 2) * strands)
                ext = tensions * g_true
                if hyst_retained_frac != 0.0:
                    ext = ext + hyst_retained_frac * (running_peak_t - tensions) * g_true
                if creep_per_sweep_mm != 0.0:
                    ext = ext + creep_line * s_idx

                delta = dist - ext - home
                pts.append({
                    "l_drive": float(delta[drive]),
                    "l_sensor": float(delta[sensor]),
                    "sensor_force_n": float(f_cmd),
                    "source_sensor_anchor": int(actual_sensor),
                    "raw_angles_deg": raw_angles.tolist(),
                    "fixed_error_angles_deg": [
                        float(delta[a] / mm_per_deg[a]) if a in (low, 4) else None
                        for a in range(5)
                    ],
                    "extra_sensor_ranges": [{
                        "anchor": int(passive),
                        "force_n": float(extra_f_cmd),
                        "length_mm": float(delta[passive]),
                        "accepted": True,
                    }],
                })
        sweeps.append({
            "fixed_anchors": [low, 4],
            "fixed_lengths": [0.0, 0.0],
            "drive_anchor": drive,
            "sensor_anchor": sensor,
            "metadata": {"sensor_force_profile": True, "strand_tension_n": float(f_cmd / 2.0)},
            "data_points": pts,
        })

    return {
        "machine_type": "hangprinter_5_pulley_reduced",
        "num_anchors": 5,
        "dimensions": 3,
        "config": {
            "mm_per_degree": mm_per_deg,
            "m666_before_data_collection": {
                "S": ea_cfg, "W": weight_n / 9.81, "U": ma.tolist(), "N": strands.tolist(), "Y": guy.tolist(),
            },
            "force_tuning": {"force_low_n": 0.05},
        },
        "sweeps": sweeps,
    }


def test_static_tension_superposition_is_exact():
    dataset = _clean_hp5_dataset(true_ea_scale=1.0)
    lengths = dataset_line_lengths(dataset)["lengths_mm"]
    basis = dataset_stretch_basis(dataset, lengths)
    assert basis is not None
    measured = np.isfinite(lengths)
    positions = np.zeros((len(lengths), 3))
    positions[:, 2] = 300.0
    _, dirs_to_a, t_grav, t_unit, known, t_nom, _ = static_tension_superposition(
        HP5_ANCHORS, positions, measured, basis,
    )
    # Check static force balance sum_i T_i u_i = [0, 0, W] and exact linearity in known forces.
    net_force = np.einsum("pn,pnd->pd", t_nom, dirs_to_a)
    expected = np.zeros_like(net_force)
    expected[:, 2] = basis["weight"]
    np.testing.assert_allclose(net_force, expected, atol=1e-7)
    t_double = static_tension_superposition(HP5_ANCHORS, positions, measured, basis, force_scale=2.0)[5]
    np.testing.assert_allclose(t_double - t_grav, 2.0 * (t_nom - t_grav), atol=1e-8)


def test_varpro_recovers_wrong_stiffness_and_reports_config_g_separation():
    # True EA is 0.75x configured (k_true = 1.333) with two force levels (4 N and 10 N).
    dataset = _clean_hp5_dataset(true_ea_scale=0.75, force_levels=(4.0, 10.0))
    assert has_force_contrast(dataset)
    seed = HP5_ANCHORS + np.array([[0, 20, 10], [-15, 10, -10], [10, -15, 5], [-10, -10, 8], [0, 0, 30]])
    base = fit_pose_least_squares(dataset, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    vp = fit_pose_least_squares(
        dataset, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
        fit_stretch_varpro=True,
    )
    base_err = float(np.max(np.abs(base["anchors"] - HP5_ANCHORS)))
    vp_err = float(np.max(np.abs(vp["anchors"] - HP5_ANCHORS)))
    assert vp.get("varpro") is not None
    assert vp_err < 0.2 * base_err
    assert vp_err < 0.15
    assert vp["varpro"]["k_hat"] == pytest.approx(1.0 / 0.75, abs=0.04)
    assert "ea_separated_for_config_g" in vp["varpro"]
    assert "nuisance_singular_values" in vp["varpro"]


def test_varpro_handles_compliance_hysteresis_creep_and_directional_friction():
    seed = HP5_ANCHORS + np.array([[0, 15, 8], [-12, 8, -8], [8, -12, 4], [-8, -8, 6], [0, 0, 20]])

    # 1. Retained compliance hysteresis + wrong EA
    ds_hyst = _clean_hp5_dataset(true_ea_scale=0.8, hyst_retained_frac=0.45, force_levels=(4.0, 9.0))
    base_h = fit_pose_least_squares(ds_hyst, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    vp_h = fit_pose_least_squares(ds_hyst, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM, fit_stretch_varpro=True)
    assert vp_h.get("varpro") is not None
    assert float(np.max(np.abs(vp_h["anchors"] - HP5_ANCHORS))) < float(np.max(np.abs(base_h["anchors"] - HP5_ANCHORS)))

    # 2. Gradual permanent line creep across sweeps
    ds_creep = _clean_hp5_dataset(true_ea_scale=0.85, creep_per_sweep_mm=0.18)
    base_c = fit_pose_least_squares(ds_creep, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    vp_c = fit_pose_least_squares(ds_creep, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM, fit_stretch_varpro=True)
    assert vp_c.get("varpro") is not None
    assert "creep" in vp_c["varpro"]["active_blocks"] or "rw_creep" in vp_c["varpro"]["active_blocks"]
    assert float(np.max(np.abs(vp_c["anchors"] - HP5_ANCHORS))) < 0.35 * float(np.max(np.abs(base_c["anchors"] - HP5_ANCHORS)))

    # 3. Directional Coulomb friction + wrong EA across two force levels (separates friction offset from k)
    ds_fric = _clean_hp5_dataset(true_ea_scale=0.85, dir_coulomb_n=1.2, force_levels=(4.0, 9.0))
    base_f = fit_pose_least_squares(ds_fric, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    vp_f = fit_pose_least_squares(ds_fric, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM, fit_stretch_varpro=True)
    assert vp_f.get("varpro") is not None
    assert "dir_fric" in vp_f["varpro"]["active_blocks"]
    assert float(np.max(np.abs(vp_f["anchors"] - HP5_ANCHORS))) < 0.3 * float(np.max(np.abs(base_f["anchors"] - HP5_ANCHORS)))


def test_schur_ds_optimal_planning_prefers_force_contrast_on_z_d_ridge():
    """Single-force low-z sweeps sit on the z_D <-> alpha/EA ridge; Schur Ds-optimal planning
    automatically selects an elevated high-force sweep to collapse the ridge."""
    from autocal.flex import FlexModel

    anchors = HP3_ANCHORS
    home = np.linalg.norm(anchors, axis=1)
    obs_cfgs = [
        SweepConfig((0, 3), (390.0, 0.0), 1, 2),
        SweepConfig((1, 3), (390.0, 0.0), 2, 0),
        SweepConfig((2, 3), (390.0, 0.0), 0, 1),
    ]
    obs_pos = np.concatenate([predict_sweep_positions(anchors, cfg, 12, SWEEP_PRETENSION_SHARE) for cfg in obs_cfgs])
    obs_len = np.linalg.norm(anchors[None, :, :] - obs_pos[:, None, :], axis=2) - home
    obs_sensors = np.concatenate([np.full(12, cfg.sensor_anchor, dtype=int) for cfg in obs_cfgs])
    model = FlexModel(spring_k_per_unit_length=20000.0, mechanical_advantage=[1.0] * 4, guy_wire_lengths=[0.0] * 4)
    obs_stretch = {
        "model": model,
        "sensors": obs_sensors,
        "known": np.full_like(obs_len, np.nan),
        "forces": np.full(len(obs_pos), 2.0),
        "weight": 0.5 * 9.81,
        "passive": 0.0,
    }
    candidates = generate_candidate_sweeps(
        num_anchors=4, dimensions=3, machine_type="hangprinter_3",
        fixed_delta_values_mm=[-900.0, -500.0, 0.0, 300.0],
    )
    ranked = rank_candidates_force_aware(
        anchors,
        machine_type="hangprinter_3",
        observed_lengths_mm=obs_len,
        candidates=candidates,
        points_per_sweep=12,
        sigma_mm=0.05,
        regularization=1e-6,
        top_k=5,
        observed_stretch=obs_stretch,
        candidate_force_scales=(1.0, 2.5),
    )
    assert ranked
    top_gain, top_cfg, top_scale = ranked[0]
    assert top_gain > 0.0
    # Must pick an elevated sweep (shortened D line) at the high force scale to break the ridge!
    assert top_cfg.fixed_deltas_mm[top_cfg.fixed_anchors.index(3)] < 0.0
    assert top_scale == pytest.approx(2.5)


def test_schur_anchor_information_never_exceeds_unmarginalized_information():
    rng = np.random.default_rng(7)
    jac = rng.normal(size=(40, 10))
    joint = jac.T @ jac
    eff = schur_anchor_information(joint, 8, [1.0, 1.0])
    # Schur complement must be positive semidefinite and <= I_aa in Loewner order.
    diff_eigs = np.linalg.eigvalsh(joint[:8, :8] - eff)
    eff_eigs = np.linalg.eigvalsh(eff)
    assert np.all(diff_eigs >= -1e-10)
    assert np.all(eff_eigs >= -1e-10)


def test_hp3_force_contrast_dataset_enables_per_pose_stretch():
    ds = {
        "machine_type": "hangprinter_3",
        "num_anchors": 4,
        "dimensions": 3,
        "config": {"m666_before_data_collection": {"S": 20000.0, "W": 0.5, "U": [1] * 4, "N": [1] * 4, "Y": [0] * 4}},
        "sweeps": [
            {"fixed_anchors": [0, 3], "fixed_lengths": [0.0, 0.0], "drive_anchor": 1, "sensor_anchor": 2,
             "metadata": {"sensor_force_profile": True, "strand_tension_n": 2.0},
             "data_points": [{"l_drive": 0.0, "l_sensor": 0.0, "sensor_force_n": 2.0}]},
            {"fixed_anchors": [0, 3], "fixed_lengths": [0.0, -300.0], "drive_anchor": 1, "sensor_anchor": 2,
             "metadata": {"sensor_force_profile": True, "strand_tension_n": 4.0},
             "data_points": [{"l_drive": 0.0, "l_sensor": 0.0, "sensor_force_n": 4.0}]},
        ],
    }
    assert has_force_contrast(ds)
    assert dataset_stretch(ds) is not None
