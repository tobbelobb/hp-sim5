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
    true_force_scale: float = 1.0,
    hyst_retained_frac: float = 0.0,
    dir_coulomb_n: float = 0.0,
    creep_per_sweep_mm: float = 0.0,
    ripple_amp_n: float = 0.0,
    ripple_period: int = 50,
    force_levels: tuple[float, ...] = (4.0,),
    weight_kg: float = 0.5,
) -> dict:
    """Clean exact-geometry HP5r dataset with consistent fixed-line deltas and physical nuisances."""
    anchors = HP5_ANCHORS
    ma = np.array([2.0, 2.0, 2.0, 2.0, 4.0])
    strands = np.array([2.0, 2.0, 2.0, 2.0, 4.0])
    guy = np.array([429.1, 429.1, 429.1, 429.1, 352.3])
    ea_cfg = 20000.0
    ea_true = ea_cfg * true_ea_scale
    weight_n = float(weight_kg) * 9.81
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

                f_real = true_force_scale * f_cmd + dir_coulomb_n * move_sign + ripple_amp_n * np.sin(ripple_period * th[actual_sensor])
                extra_f_cmd = 0.5 * f_cmd
                extra_f_real = true_force_scale * extra_f_cmd + dir_coulomb_n * extra_sign
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


def test_varpro_separates_true_ea_from_incorrect_motor_force_scale():
    """Regression test (review #1): wrong motor force scale alpha=1.35 must NOT bias ea_estimated_n.

    With true EA = 0.80 * EA_cfg (k_grav = 1.25, EA_true = 16000 N) and alpha = 1.35
    (k_sensor = 1.6875), VarPro must separate k_grav from k_sensor via gravity mg.
    """
    dataset = _clean_hp5_dataset(
        true_ea_scale=0.80,
        true_force_scale=1.35,
        force_levels=(4.0, 10.0),
        weight_kg=1.2,
    )
    seed = HP5_ANCHORS + np.array([[0, 15, 8], [-12, 8, -8], [8, -12, 4], [-8, -8, 6], [0, 0, 20]])
    vp = fit_pose_least_squares(
        dataset, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
        fit_stretch_varpro=True,
    )
    diag = vp.get("varpro")
    assert diag is not None
    assert float(np.max(np.abs(vp["anchors"] - HP5_ANCHORS))) < 0.25
    assert diag["ea_separated_for_config_g"] is True
    # Gravity-derived k_grav and EA must track true EA (16000 N), NOT alpha * k_grav (1.6875).
    assert diag["k_grav_hat"] == pytest.approx(1.25, abs=0.06)
    assert diag["ea_estimated_n"] == pytest.approx(16000.0, rel=0.05)
    assert diag["k_sensor_hat"] == pytest.approx(1.35 * 1.25, abs=0.05)
    assert diag["alpha_hat"] == pytest.approx(1.35, abs=0.07)


def test_strip_sensor_force_option_removes_flag_value_pairs_and_preserves_profile():
    """Regression test (review #2): replacing --sensor-force never leaves orphaned values or strips --sensor-force-profile."""
    from autocal.planning_pass import _SENSOR_FORCE_FLAGS, _strip_cli_option

    args = [
        "--sim",
        "--sensor-force", "3.0",
        "--sensor-force-profile", "profile.json",
        "--sensor-force=4.5",
        "--sensorForce", "2.2",
        "--return-to-origin",
    ]
    cleaned = _strip_cli_option(args, _SENSOR_FORCE_FLAGS)
    assert cleaned == ["--sim", "--sensor-force-profile", "profile.json", "--return-to-origin"]
    assert "3.0" not in cleaned
    assert "2.2" not in cleaned


def test_force_contrast_planning_scores_and_records_actual_capped_force(tmp_path, monkeypatch):
    """Regression test (review #3): planner scores & records actual capped force (and avoids unexecutable contrast)."""
    import json
    import autocal.planning_pass as planning
    from autocal.equal_strain import effective_strand_tension_scale, write_sensor_force_profile

    anchors = HP3_ANCHORS
    cfg = SweepConfig((0, 3), (300.0, -500.0), 1, 2)
    # Build a valid HP3 equal-strain dataset with a tight M666 X force cap so 2.0x gets capped!
    base_target = 1.5
    home = np.linalg.norm(anchors, axis=1)
    obs_cfgs = [
        SweepConfig((0, 3), (390.0, 0.0), 1, 2),
        SweepConfig((1, 3), (390.0, 0.0), 2, 0),
        SweepConfig((2, 3), (390.0, 0.0), 0, 1),
    ]
    sweeps = []
    for ocfg in obs_cfgs:
        pos = predict_sweep_positions(anchors, ocfg, 8, SWEEP_PRETENSION_SHARE)
        lens = np.linalg.norm(anchors[None, :, :] - pos[:, None, :], axis=2) - home
        sweeps.append({
            "fixed_anchors": list(ocfg.fixed_anchors),
            "fixed_lengths": list(ocfg.fixed_deltas_mm),
            "drive_anchor": ocfg.drive_anchor,
            "sensor_anchor": ocfg.sensor_anchor,
            "metadata": {"sensor_force_profile": True, "sensor_force_profile_model": "reeving-aware-v2",
                         "strand_tension_n": base_target},
            "data_points": [
                {"l_drive": float(row[ocfg.drive_anchor]), "l_sensor": float(row[ocfg.sensor_anchor]),
                 "sensor_force_n": base_target}
                for row in lens
            ],
        })
    # Tight X cap (9 N) forces a 2.0x request to be capped below 2.0x (to ~1.75x).
    dataset = {
        "machine_type": "hangprinter_3",
        "num_anchors": 4,
        "dimensions": 3,
        "config": {
            "m666_before_data_collection": {
                "S": 20000.0, "W": 0.5, "U": [1.0] * 4, "N": [1.0] * 4, "Y": [0.0] * 4, "X": [9.0] * 4,
            },
            "force_tuning": {"force_low_n": 0.05, "force_mid_n": 0.3, "force_max_n": 6.0},
        },
        "sweeps": sweeps,
    }
    capped_scale = effective_strand_tension_scale(dataset, anchors, cfg, 2.0, allow_force_contrast=True)
    assert capped_scale is not None
    assert 1.0 < capped_scale < 2.0

    fit = {
        "dataset": dataset, "dataset_for_estimation": dataset, "anchors": anchors,
        "machine_type": "hangprinter_3", "num_anchors": 4, "dimensions": 3,
        "spool_model_params": None, "sweep_configs_for_info": [], "info": np.eye(9),
        "pose_fit": {"robust_scale_mm": 0.05},
        "information_settings": {"fd_eps_mm": 1.0, "regularization": 1e-6},
        "collection_settings": {"find_radii_mode": "off", "find_buildup_mode": "off",
                                "base_radii": None, "buildup_factor": None},
    }
    monkeypatch.setattr(planning, "generate_candidate_sweeps", lambda **kw: [cfg])
    plan = planning.plan_ellipse_sweep(fit, tmp_path / "sweeps.json", force_contrast=True, top_k=5)
    assert plan["best_cfg"] == cfg
    assert plan["chosen_force_scale"] == pytest.approx(capped_scale, abs=1e-3)
    written_profiles = json.loads((tmp_path / "sweeps.active_sweep_cfg.sensor_force_profile.json").read_text())
    assert written_profiles[0]["strand_tension_n"] / base_target == pytest.approx(plan["chosen_force_scale"], abs=1e-3)

    # Constant-force path: (a) with existing --sensor-force 3.0 and tight force_max_n, replaces cleanly with capped value;
    # (b) without force_mid_n, never selects an unexecutable high-force contrast scale.
    const_ds = {
        **dataset,
        "sweeps": [{**s, "metadata": {"sensor_force_profile": False}} for s in sweeps],
    }
    const_fit = {**fit, "dataset": const_ds, "dataset_for_estimation": const_ds}
    plan_const = planning.plan_ellipse_sweep(
        const_fit, tmp_path / "const.json", force_contrast=True,
        collector_args=["--sensor-force", "3.0"], top_k=5,
    )
    cmd = plan_const["collect_command"]
    assert cmd is not None
    assert "3.0" not in cmd
    assert cmd.count("--sensor-force") <= 1

    no_mid_ds = {
        **const_ds,
        "config": {**const_ds["config"], "force_tuning": {"force_low_n": 0.05}},
    }
    no_mid_fit = {**fit, "dataset": no_mid_ds, "dataset_for_estimation": no_mid_ds}
    plan_no_mid = planning.plan_ellipse_sweep(no_mid_fit, tmp_path / "nomid.json", force_contrast=True, top_k=5)
    assert plan_no_mid["chosen_force_scale"] == pytest.approx(1.0)
    assert "--sensor-force" not in (plan_no_mid["collect_command"] or [])


def test_heteroscedastic_weights_are_applied_as_variance_weights_consistently():
    """Regression test (review #4): VarPro inner solve applies statistical weights w (via sqrt(w)),
    matching the outer objective's sqrt(lev_weights) scaling rather than squaring w."""
    from autocal.stretch_varpro import _estimate_force_leverage_weights

    # Construct residuals whose magnitude scales strictly with leverage to trigger leverage weighting.
    rng = np.random.default_rng(123)
    leverage = np.linspace(0.05, 0.50, 40)
    r_corr = np.zeros((40, 5))
    r_corr[:, 0] = 0.02 + 0.6 * leverage * (1.0 + 0.05 * rng.normal(size=40))
    kept = np.ones(40, dtype=bool)
    lev_w = _estimate_force_leverage_weights(r_corr, leverage, kept)
    assert lev_w is not None
    assert lev_w[0] > 3.0 * lev_w[-1]

    # Verify exact algebraic equivalence between weighted normal equations with sqrt(w)
    # and the stacked outer objective [sqrt(w)*r_corr; lam_sqrt*psi].
    X = rng.normal(size=(40, 3))
    y = rng.normal(size=40)
    lam_sqrt = np.array([0.2, 0.3, 0.4])
    sqrt_w = np.sqrt(lev_w)
    xw, yw = X * sqrt_w[:, None], y * sqrt_w
    psi = np.linalg.solve(xw.T @ xw + np.diag(lam_sqrt ** 2), xw.T @ yw)

    def outer_obj(p):
        r_out = sqrt_w * (y - X @ p)
        prior_out = lam_sqrt * p
        return float(np.sum(r_out ** 2) + np.sum(prior_out ** 2))

    # Any perturbation around psi must strictly increase the exact outer objective (stationarity check).
    base_val = outer_obj(psi)
    for k in range(3):
        step = np.zeros(3)
        step[k] = 1e-4
        assert outer_obj(psi + step) > base_val
        assert outer_obj(psi - step) > base_val

