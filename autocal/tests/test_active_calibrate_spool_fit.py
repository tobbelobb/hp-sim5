from types import SimpleNamespace

import numpy as np
import pytest

from autocal import active_calibrate as ac


def _patch_spool_runtime(monkeypatch, *, target_radii: np.ndarray, target_buildup: np.ndarray):
    target_r = np.asarray(target_radii, dtype=float).reshape(-1)
    target_b = np.asarray(target_buildup, dtype=float).reshape(-1)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        out = dict(dataset)
        out["_spool_radii_mm"] = np.asarray(spool_params["radii_mm"], dtype=float).reshape(-1).tolist()
        out["_spool_buildup_factor"] = np.asarray(spool_params["buildup_factor"], dtype=float).reshape(-1).tolist()
        return out

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = kwargs
        if isinstance(dataset_or_path, dict):
            num_anchors = int(dataset_or_path.get("num_anchors", 3))
        else:
            num_anchors = 3
        anchors = np.zeros((num_anchors, 2), dtype=float)
        for axis in range(num_anchors):
            anchors[axis, 0] = float(axis)
        return {"anchors": anchors, "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = (anchors, kwargs)
        radii = np.asarray(dataset.get("_spool_radii_mm", target_r), dtype=float).reshape(-1)
        buildup = np.asarray(dataset.get("_spool_buildup_factor", target_b), dtype=float).reshape(-1)
        return float(np.sum((radii - target_r) ** 2.0) + np.sum((buildup - target_b) ** 2.0))

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)


def test_spool_fit_buildup_only_keeps_radii_fixed(monkeypatch):
    base = np.array([10.0, 11.0, 12.0], dtype=float)
    target_k = np.array([0.12, 0.12, 0.12], dtype=float)
    _patch_spool_runtime(monkeypatch, target_radii=base, target_buildup=target_k)

    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    eff_r, fit_anchors, spool_params, transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="off",
        find_buildup_mode="global",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=(0.0, 0.2),
        r0_prior_sigma_mm=None,
        b_prior_sigma=1e9,
        spool_outer_iters=1,
        spool_inner_iters=40,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    assert np.allclose(eff_r, base)
    assert fit_anchors.shape == (3, 2)
    assert isinstance(spool_params, dict)
    assert isinstance(transformed, dict)
    fitted_k = np.asarray(fit_info["best_modeled_buildup_factor"], dtype=float)
    assert np.allclose(fitted_k, target_k, atol=2e-2)
    assert "spool_info_matrix" in fit_info
    assert fit_info["spool_info_matrix"] is None or isinstance(fit_info["spool_info_matrix"], list)


def test_spool_fit_radius_similarity_prior_reduces_spread(monkeypatch):
    base = np.array([10.0, 14.0, 18.0], dtype=float)
    target_r = np.array([10.0, 14.0, 18.0], dtype=float)
    target_k = np.zeros(3, dtype=float)
    _patch_spool_runtime(monkeypatch, target_radii=target_r, target_buildup=target_k)

    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    eff_r, _fit_anchors, _spool_params, _transformed, _fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="per-anchor",
        find_buildup_mode="off",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=(1.0, 30.0),
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=60,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    assert np.std(eff_r) < np.std(base)
    assert "spool_info_matrix" in _fit_info


def test_spool_fit_uses_default_b_prior_when_not_provided(monkeypatch):
    base = np.array([10.0, 10.0, 10.0], dtype=float)
    target_k = np.array([0.02, 0.02, 0.02], dtype=float)
    _patch_spool_runtime(monkeypatch, target_radii=base, target_buildup=target_k)

    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    _eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="off",
        find_buildup_mode="global",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=(-0.5, 0.5),
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=4,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    assert fit_info.get("b_prior_sigma") == ac._DEFAULT_B_PRIOR_SIGMA


def test_spool_fit_anchor_step_uses_fast_solver_settings(monkeypatch):
    base = np.array([10.0, 11.0, 12.0], dtype=float)
    target_k = np.array([0.05, 0.05, 0.05], dtype=float)
    _patch_spool_runtime(monkeypatch, target_radii=base, target_buildup=target_k)

    calls = []

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = dataset_or_path
        calls.append(dict(kwargs))
        return {"anchors": np.zeros((3, 2), dtype=float), "cost": 0.0}

    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)

    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="per-anchor",
        find_buildup_mode="global",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=(1.0, 20.0),
        b_bounds=(-0.2, 0.2),
        r0_prior_sigma_mm=2.0,
        b_prior_sigma=0.1,
        spool_outer_iters=2,
        spool_inner_iters=4,
        theta0_mode="zero",
        solve_restarts=7,
        solve_iterations=999,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        sweep_metric="outlier_ratio",
        use_noise_mean=True,
        sigma_source="auto",
        robust_debug=True,
    )

    assert len(calls) == 3
    assert np.allclose(np.asarray(calls[0].get("initial_guess"), dtype=float), np.zeros((3, 2), dtype=float))
    for kwargs in calls:
        assert int(kwargs.get("num_restarts", 0)) <= 2
        assert int(kwargs.get("max_iterations", 0)) <= 160
        assert kwargs.get("pointwise_filtering") is True
        assert kwargs.get("sweep_wise_filtering") is True
        assert kwargs.get("robust_debug") is False


def test_coordinate_descent_shrinks_non_improving_axes():
    x0 = np.array([5.0, 0.25], dtype=float)
    lo = np.array([-10.0, -1.0], dtype=float)
    hi = np.array([10.0, 1.0], dtype=float)
    kinds = ["r", "k"]
    initial_steps = ac._initial_spool_steps(x0, lo, hi, kinds)

    def objective(x):
        vec = np.asarray(x, dtype=float).reshape(-1)
        return float((vec[0] - 1.0) ** 2.0)

    _x_opt, info = ac._coordinate_descent_spool(
        x0,
        lo=lo,
        hi=hi,
        kinds=kinds,
        max_iters=4,
        objective=objective,
    )
    final_steps = np.asarray(info.get("step_final", []), dtype=float)
    assert final_steps.size == 2
    assert final_steps[1] < initial_steps[1]


def test_format_m666_from_length_model_includes_r_and_q():
    length_model = {
        "effective_radii_mm": [30.0, 29.5, 30.25],
        "modeled_buildup_factor": [0.012, 0.012, 0.012],
    }
    cmd = ac._format_m666_from_length_model(length_model)
    assert cmd == "M666 R30:29.5:30.25 Q0.012"


def test_format_m666_from_length_model_handles_per_anchor_q():
    length_model = {
        "effective_radii_mm": [30.0, 30.0, 30.0],
        "modeled_buildup_factor": [0.01, 0.02, 0.03],
    }
    cmd = ac._format_m666_from_length_model(length_model)
    assert cmd == "M666 R30:30:30 Q0.01:0.02:0.03"


def test_plan_hits_underconstrained_penalty_when_cost_is_sentinel():
    plan = {
        "cost_noise_normalized": 100.0,
        "calibration": {
            "details": {
                "noise_metrics": {
                    "chi2_red": 1.2e5,
                }
            }
        },
    }
    assert ac._plan_hits_underconstrained_penalty(plan) is True


def test_plan_hits_underconstrained_penalty_ignores_non_sentinel_cost():
    plan = {
        "cost_noise_normalized": 99.0,
        "calibration": {
            "details": {
                "noise_metrics": {
                    "chi2_red": 1.2e5,
                }
            }
        },
    }
    assert ac._plan_hits_underconstrained_penalty(plan) is False


def test_compute_tau_mad_rescore_from_rows_uses_inliers_and_rescores():
    rows = [
        {
            "sweep_id": "sweep_a",
            "residual_mm": 1.0,
            "residual_mm_signed": -1.0,
            "residual_z_signed": -0.5,
            "cutoff_mm": 2.0,
            "l_drive_mm": 100.0,
            "l_sensor_mm": 100.0,
        },
        {
            "sweep_id": "sweep_a",
            "residual_mm": 2.0,
            "residual_mm_signed": 2.0,
            "residual_z_signed": 1.0,
            "cutoff_mm": 2.0,
            "l_drive_mm": 200.0,
            "l_sensor_mm": 200.0,
        },
        {
            "sweep_id": "sweep_b",
            "residual_mm": 3.0,
            "residual_mm_signed": 3.0,
            "residual_z_signed": 1.5,
            "cutoff_mm": 4.0,
            "l_drive_mm": 300.0,
            "l_sensor_mm": 300.0,
        },
        {
            "sweep_id": "sweep_b",
            "residual_mm": 10.0,
            "residual_mm_signed": 10.0,
            "residual_z_signed": 5.0,
            "cutoff_mm": 2.0,
            "l_drive_mm": 400.0,
            "l_sensor_mm": 400.0,
        },
    ]
    out = ac._compute_tau_mad_rescore_from_rows(
        rows,
        cost_noise_normalized_old=74.0,
        chi2_red_old=50.0,
        sigma_model_mm=2.0,
        params_count=2.0,
    )

    signed_inliers = np.asarray([-1.0, 2.0, 3.0], dtype=float)
    expected_tau = 1.4826 * np.median(np.abs(signed_inliers - np.median(signed_inliers)))
    expected_scale = (2.0 * 2.0) / ((2.0 * 2.0) + (expected_tau * expected_tau))
    assert np.isclose(float(out["tau_mad_mm"]), expected_tau, atol=1e-9)
    assert int(out["tau_mad_inlier_points"]) == 3
    assert int(out["tau_mad_total_points"]) == 4
    assert np.isclose(float(out["cost_noise_normalized_old"]), 74.0, atol=1e-9)
    assert np.isclose(float(out["cost_noise_normalized_rescored"]), 74.0 * expected_scale, atol=1e-9)
    assert np.isclose(float(out["chi2_red_old"]), 50.0, atol=1e-9)
    assert np.isclose(float(out["chi2_red_rescored"]), 50.0 * expected_scale, atol=1e-9)
    assert np.isclose(float(out["residual_vs_distance_slope"]), 0.01, atol=1e-9)
    per_sweep = out["per_sweep_residual_summary"]
    assert isinstance(per_sweep, dict)
    assert set(per_sweep.keys()) == {"sweep_a", "sweep_b"}
    assert np.isclose(float(per_sweep["sweep_a"]["median_residual_mm"]), 0.5, atol=1e-9)
    assert int(per_sweep["sweep_b"]["clipped_points"]) == 1
    demean = out["per_sweep_demean"]
    assert isinstance(demean, dict)
    assert np.isclose(float(demean["cost_noise_normalized_demeaned"]), 0.375, atol=1e-9)
    assert np.isclose(float(demean["chi2_red_demeaned"]), 1.125, atol=1e-9)
    assert np.isclose(float(demean["sweep_bias_span_mm"]), 2.5, atol=1e-9)


def test_normalize_dataset_point_roles_swaps_unswapped_reversed_points():
    dataset = {
        "sweeps": [
            {
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                {
                    "l_drive": 10.0,
                    "l_sensor": -100.0,
                    "l_drive_mu": 11.0,
                    "l_sensor_mu": -101.0,
                    "assumed_tension_drive_n": 4.0,
                    "assumed_tension_sensor_n": 2.0,
                    "drive_setpoint_mm": 9.95,
                    "source_drive_anchor": 2,
                    "source_sensor_anchor": 1,
                    "raw_angles_deg": [0.0, 1.0, 2.0],
                }
            ],
        }
        ]
    }
    changed = ac._normalize_dataset_point_roles(dataset)
    point = dataset["sweeps"][0]["data_points"][0]
    assert changed == 1
    assert point["l_drive"] == -100.0
    assert point["l_sensor"] == 10.0
    assert point["l_drive_mu"] == -101.0
    assert point["l_sensor_mu"] == 11.0
    assert point["assumed_tension_drive_n"] == 2.0
    assert point["assumed_tension_sensor_n"] == 4.0
    assert point["raw_angles_deg"][1] == 1.0
    assert point["raw_angles_deg"][2] == 2.0


def test_normalize_dataset_point_roles_keeps_already_canonical_points():
    dataset = {
        "sweeps": [
            {
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {
                        "l_drive": -100.0,
                        "l_sensor": 10.0,
                        "drive_setpoint_mm": 9.95,
                        "source_drive_anchor": 2,
                        "source_sensor_anchor": 1,
                    }
                ],
            }
        ]
    }
    changed = ac._normalize_dataset_point_roles(dataset)
    point = dataset["sweeps"][0]["data_points"][0]
    assert changed == 0
    assert point["l_drive"] == -100.0
    assert point["l_sensor"] == 10.0


def test_normalize_dataset_point_roles_does_not_double_swap_canonical_reversed_metadata():
    # Reversed source_* is valid when collection has already remapped l_* fields to
    # sweep-level canonical orientation.
    dataset = {
        "sweeps": [
            {
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {
                        "l_drive": -100.0,
                        "l_sensor": 10.0,
                        "l_drive_mu": -101.0,
                        "l_sensor_mu": 11.0,
                        "assumed_tension_drive_n": 2.0,
                        "assumed_tension_sensor_n": 4.0,
                        "drive_setpoint_mm": 9.95,
                        "source_drive_anchor": 2,
                        "source_sensor_anchor": 1,
                        "raw_angles_deg": [0.0, 1.0, 2.0],
                    }
                ],
            }
        ]
    }
    changed = ac._normalize_dataset_point_roles(dataset)
    point = dataset["sweeps"][0]["data_points"][0]
    assert changed == 0
    assert point["l_drive"] == -100.0
    assert point["l_sensor"] == 10.0
    assert point["l_drive_mu"] == -101.0
    assert point["l_sensor_mu"] == 11.0
    assert point["assumed_tension_drive_n"] == 2.0
    assert point["assumed_tension_sensor_n"] == 4.0
    assert point["raw_angles_deg"][1] == 1.0
    assert point["raw_angles_deg"][2] == 2.0


def test_normalize_dataset_point_roles_keeps_reversed_points_without_setpoint():
    dataset = {
        "sweeps": [
            {
                "drive_anchor": 0,
                "sensor_anchor": 1,
                "data_points": [
                    {
                        "l_drive": -12.0,
                        "l_sensor": 34.0,
                        "source_drive_anchor": 1,
                        "source_sensor_anchor": 0,
                    }
                ],
            }
        ]
    }
    changed = ac._normalize_dataset_point_roles(dataset)
    point = dataset["sweeps"][0]["data_points"][0]
    assert changed == 0
    assert point["l_drive"] == -12.0
    assert point["l_sensor"] == 34.0


def test_default_r0_bounds_per_anchor_are_base_to_1p5x_base():
    base = np.array([30.0, 40.0, 50.0], dtype=float)
    lo, hi = ac._resolve_r0_bounds(base, find_radii_mode="per-anchor", r0_bounds=None)
    assert np.allclose(lo, base)
    assert np.allclose(hi, base * 1.5)


def test_default_r0_bounds_global_start_at_largest_base():
    base = np.array([30.0, 40.0, 50.0], dtype=float)
    lo, hi = ac._resolve_r0_bounds(base, find_radii_mode="global", r0_bounds=None)
    assert lo.shape == (1,)
    assert hi.shape == (1,)
    assert np.isclose(float(lo[0]), 50.0)
    assert np.isclose(float(hi[0]), 75.0)


def test_default_b_bounds_global_are_zero_to_one():
    modeled_b = np.array([0.0, 0.0, 0.0], dtype=float)
    lo, hi = ac._resolve_b_bounds(modeled_b, find_buildup_mode="global", b_bounds=None)
    assert lo.shape == (1,)
    assert hi.shape == (1,)
    assert np.isclose(float(lo[0]), 0.0)
    assert np.isclose(float(hi[0]), 1.0)


def test_default_b_bounds_per_anchor_are_zero_to_one():
    modeled_b = np.array([0.0, 0.0, 0.0], dtype=float)
    lo, hi = ac._resolve_b_bounds(modeled_b, find_buildup_mode="per-anchor", b_bounds=None)
    assert np.allclose(lo, np.zeros(3, dtype=float))
    assert np.allclose(hi, np.ones(3, dtype=float))


def test_default_b_prior_sigma_is_relaxed_for_global_k_fits():
    assert np.isclose(float(ac._DEFAULT_B_PRIOR_SIGMA), 0.1)


def test_inject_spool_collection_args_adds_preserve_when_search_enabled():
    args, applied = ac._inject_spool_collection_args(
        ["--speedup", "25"],
        find_radii_mode="per-anchor",
        find_buildup_mode="off",
    )
    assert applied is True
    assert "--preserve-buildup-factor" in args


def test_inject_spool_collection_args_respects_explicit_buildup_override():
    args, applied = ac._inject_spool_collection_args(
        ["--force-buildup-factor", "0.2"],
        find_radii_mode="per-anchor",
        find_buildup_mode="global",
    )
    assert applied is False
    assert args == ["--force-buildup-factor", "0.2"]


def test_effective_hp_sim_reset_enables_for_sim_spool_search():
    enabled = ac._effective_hp_sim_reset(
        sim=True,
        hp_sim_reset=False,
        find_radii_mode="per-anchor",
        find_buildup_mode="off",
    )
    assert enabled is True


def test_effective_hp_sim_reset_stays_off_without_sim_or_spool_search():
    assert (
        ac._effective_hp_sim_reset(
            sim=False,
            hp_sim_reset=False,
            find_radii_mode="per-anchor",
            find_buildup_mode="global",
        )
        is False
    )
    assert (
        ac._effective_hp_sim_reset(
            sim=True,
            hp_sim_reset=False,
            find_radii_mode="off",
            find_buildup_mode="off",
        )
        is False
    )


def test_effective_hp_sim_reset_respects_explicit_flag():
    enabled = ac._effective_hp_sim_reset(
        sim=True,
        hp_sim_reset=True,
        find_radii_mode="off",
        find_buildup_mode="off",
    )
    assert enabled is True


def test_resolve_sim_config_prefers_line_layer_config_for_spool_search(monkeypatch):
    monkeypatch.delenv("AUTOCAL_RRF_SIM_CONFIG", raising=False)
    candidate = ac.REPO_ROOT / ac.RRF_SIM_VSD_PATH / ac.RRF_SIM_LINE_LAYER_CONFIG
    if not candidate.exists():
        pytest.skip(f"missing simulator config: {candidate}")
    cfg = ac._resolve_sim_config(
        machine_type="slideprinter",
        find_radii_mode="per-anchor",
        find_buildup_mode="global",
    )
    assert cfg == ac.RRF_SIM_LINE_LAYER_CONFIG


def test_resolve_sim_config_uses_env_override(monkeypatch):
    monkeypatch.setenv("AUTOCAL_RRF_SIM_CONFIG", "sys/custom_config.g")
    cfg = ac._resolve_sim_config(
        machine_type="slideprinter",
        find_radii_mode="off",
        find_buildup_mode="off",
    )
    assert cfg == "sys/custom_config.g"


def test_mm_per_degree_for_axis_ignores_lines_per_spool():
    r = 30.0
    gear = np.asarray([1.2], dtype=float)
    ma = np.asarray([2.0], dtype=float)
    mm_l1 = ac._mm_per_degree_for_axis(
        r,
        0,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=ma,
        lines_per_spool=np.asarray([1.0], dtype=float),
    )
    mm_l3 = ac._mm_per_degree_for_axis(
        r,
        0,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=ma,
        lines_per_spool=np.asarray([3.0], dtype=float),
    )
    assert np.isclose(float(mm_l1), float(mm_l3), atol=1e-12)


def test_global_b_prior_penalty_is_not_multiplied_by_anchor_count(monkeypatch):
    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        out = dict(dataset)
        out["_spool_radii_mm"] = np.asarray(spool_params["radii_mm"], dtype=float).reshape(-1).tolist()
        out["_spool_buildup_factor"] = np.asarray(spool_params["buildup_factor"], dtype=float).reshape(-1).tolist()
        return out

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = kwargs
        if isinstance(dataset_or_path, dict):
            num_anchors = int(dataset_or_path.get("num_anchors", 3))
        else:
            num_anchors = 3
        return {"anchors": np.zeros((num_anchors, 2), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = (dataset, anchors, kwargs)
        return 0.0

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    base = np.array([30.0, 30.0, 30.0], dtype=float)
    dataset = {"num_anchors": 3, "sweeps": []}
    _eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        np.zeros((3, 2), dtype=float),
        find_radii_mode="off",
        find_buildup_mode="global",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=(0.5, 0.500001),
        r0_prior_sigma_mm=None,
        b_prior_sigma=0.1,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    history = fit_info.get("history")
    assert isinstance(history, list) and history
    start_cost = float(history[0].get("start_cost"))
    assert np.isclose(start_cost, 25.0, atol=1e-6)


def test_spool_seed_candidates_include_mid_and_quartiles():
    x = np.array([30.0, 0.0], dtype=float)
    lo = np.array([30.0, 0.0], dtype=float)
    hi = np.array([45.0, 1.0], dtype=float)
    seeds = ac._spool_seed_candidates(x, lo, hi)
    assert len(seeds) == 4
    expected = [
        np.array([30.0, 0.0], dtype=float),
        np.array([37.5, 0.5], dtype=float),
        np.array([41.25, 0.75], dtype=float),
        np.array([33.75, 0.25], dtype=float),
    ]
    for got, exp in zip(seeds, expected):
        assert np.allclose(got, exp)


def test_spool_prefit_seed_candidates_add_global_radius_grid():
    x = np.array([30.0], dtype=float)
    lo = np.array([30.0], dtype=float)
    hi = np.array([45.0], dtype=float)
    seeds = ac._spool_prefit_seed_candidates(x, lo, hi, kinds=["r"])
    assert len(seeds) > 4
    assert any(np.allclose(seed, np.array([39.375], dtype=float)) for seed in seeds)


def test_spool_fit_seed_selection_can_move_off_current(monkeypatch):
    base = np.array([10.0, 10.0, 10.0], dtype=float)
    target_r = np.array([19.0, 19.0, 19.0], dtype=float)
    target_k = np.zeros(3, dtype=float)
    _patch_spool_runtime(monkeypatch, target_radii=target_r, target_buildup=target_k)

    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    _eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="global",
        find_buildup_mode="off",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=(10.0, 20.0),
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    history = fit_info.get("history")
    assert isinstance(history, list) and history
    assert history[0].get("seed_choice") != "current"


def test_spool_prefit_grid_avoids_false_basin_for_low_base_radius(monkeypatch):
    target_r = 39.2

    def landscape_cost(r_val: float) -> float:
        r = float(r_val)
        if abs(r - target_r) <= 0.35:
            return float((r - target_r) ** 2.0)
        if abs(r - 30.0) <= 0.35:
            return float(4.0 + (r - 30.0) ** 2.0)
        return float(25.0 + 0.1 * (r - 37.0) ** 2.0)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        out = {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
        }
        r = float(np.median(np.asarray(spool_params["radii_mm"], dtype=float)))
        residual = float(landscape_cost(r))
        out["_spool_r"] = r
        out["sweeps"].append(
            {
                "drive_anchor": 0,
                "sensor_anchor": 1,
                "data_points": [{"l_drive": residual, "l_sensor": 0.0} for _ in range(6)],
            }
        )
        return out

    def fake_fit_ellipse_from_sweep(l_drive, l_sensor, **kwargs):
        _ = kwargs
        drive = np.asarray(l_drive, dtype=float).reshape(-1)
        sensor = np.asarray(l_sensor, dtype=float).reshape(-1)
        rms = abs(float(np.mean(drive))) + abs(float(np.mean(sensor)))
        return SimpleNamespace(residual_rms=rms)

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = (dataset_or_path, kwargs)
        return {"anchors": np.zeros((3, 2), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = (anchors, kwargs)
        r = float(dataset.get("_spool_r", 30.0))
        return float(landscape_cost(r))

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "fit_ellipse_from_sweep", fake_fit_ellipse_from_sweep)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    dataset = {"num_anchors": 3, "sweeps": [{}]}
    seed_anchors = np.zeros((3, 2), dtype=float)

    def _run(base_radius: float):
        eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
            dataset,
            seed_anchors,
            find_radii_mode="global",
            find_buildup_mode="off",
            base_radii_mm=np.full(3, float(base_radius), dtype=float),
            modeled_buildup_factor=np.zeros(3, dtype=float),
            spool_to_motor_gearing_factor=np.ones(3, dtype=float),
            mechanical_advantage=np.ones(3, dtype=float),
            lines_per_spool=np.ones(3, dtype=float),
            r0_bounds=None,
            b_bounds=None,
            r0_prior_sigma_mm=None,
            b_prior_sigma=None,
            spool_outer_iters=1,
            spool_inner_iters=2,
            theta0_mode="zero",
            solve_restarts=1,
            solve_iterations=10,
            solve_optimizer="L-BFGS-B",
            residual_threshold=1.0,
            spring_k_multiplier=1.0,
            use_flex=False,
            pointwise_residual_mode="sampson",
            pointwise_filtering=False,
            pointwise_global_mad=False,
            sweep_wise_filtering=False,
            sweep_metric="mad",
            use_noise_mean=False,
            sigma_source="auto",
            robust_debug=False,
        )
        return float(np.median(np.asarray(eff_r, dtype=float))), fit_info

    r_low, fit_low = _run(30.0)
    r_high, fit_high = _run(38.7)

    assert abs(r_low - target_r) < 0.35
    assert abs(r_high - target_r) < 0.35
    assert abs(r_low - r_high) < 0.25

    prefit_low = fit_low.get("prefit", {})
    prefit_high = fit_high.get("prefit", {})
    assert prefit_low.get("enabled") is True
    assert prefit_high.get("enabled") is True
    assert prefit_low.get("seed_choice") != "current"


def test_spool_prefit_monotonic_boundary_is_guarded_for_base30_vs_38p7(monkeypatch):
    prefit_square_inputs = []
    noise_norm_flags = []

    def data_landscape(r_val: float) -> float:
        return float(r_val)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        out = {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
        }
        r = float(np.median(np.asarray(spool_params["radii_mm"], dtype=float)))
        out["_spool_r"] = r
        # Monotonic-in-radius proxy landscape for prefit scoring.
        prefit_val = float(r)
        out["sweeps"].append(
            {
                "drive_anchor": 0,
                "sensor_anchor": 1,
                "data_points": [{"l_drive": prefit_val, "l_sensor": 0.0} for _ in range(6)],
            }
        )
        return out

    def fake_fit_ellipse_from_sweep(l_drive, l_sensor, **kwargs):
        prefit_square_inputs.append(bool(kwargs.get("square_inputs", True)))
        drive = np.asarray(l_drive, dtype=float).reshape(-1)
        sensor = np.asarray(l_sensor, dtype=float).reshape(-1)
        rms = abs(float(np.mean(drive))) + abs(float(np.mean(sensor)))
        return SimpleNamespace(residual_rms=rms)

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = (dataset_or_path, kwargs)
        return {"anchors": np.zeros((3, 2), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = anchors
        noise_norm_flags.append(bool(kwargs.get("noise_normalized", False)))
        r = float(dataset.get("_spool_r", 30.0))
        return float(data_landscape(r))

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "fit_ellipse_from_sweep", fake_fit_ellipse_from_sweep)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    dataset = {"num_anchors": 3, "sweeps": [{}]}
    seed_anchors = np.zeros((3, 2), dtype=float)

    def _run(base_radius: float):
        eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
            dataset,
            seed_anchors,
            find_radii_mode="global",
            find_buildup_mode="off",
            base_radii_mm=np.full(3, float(base_radius), dtype=float),
            modeled_buildup_factor=np.zeros(3, dtype=float),
            spool_to_motor_gearing_factor=np.ones(3, dtype=float),
            mechanical_advantage=np.ones(3, dtype=float),
            lines_per_spool=np.ones(3, dtype=float),
            r0_bounds=None,
            b_bounds=None,
            r0_prior_sigma_mm=None,
            b_prior_sigma=None,
            spool_outer_iters=1,
            spool_inner_iters=12,
            theta0_mode="zero",
            solve_restarts=1,
            solve_iterations=10,
            solve_optimizer="L-BFGS-B",
            residual_threshold=1.0,
            spring_k_multiplier=1.0,
            use_flex=False,
            pointwise_residual_mode="sampson",
            pointwise_filtering=False,
            pointwise_global_mad=False,
            sweep_wise_filtering=False,
            sweep_metric="mad",
            use_noise_mean=False,
            sigma_source="auto",
            robust_debug=False,
        )
        return float(np.median(np.asarray(eff_r, dtype=float))), fit_info

    r_low, fit_low = _run(30.0)
    r_high, fit_high = _run(38.7)

    prefit_low = fit_low.get("prefit", {})
    prefit_high = fit_high.get("prefit", {})
    assert prefit_low.get("enabled") is True
    assert prefit_high.get("enabled") is True
    assert prefit_low.get("guarded") is True
    assert prefit_high.get("guarded") is True
    assert "uninformative" in str(prefit_low.get("message", "")).lower()
    assert "uninformative" in str(prefit_high.get("message", "")).lower()

    assert prefit_square_inputs and all(flag is False for flag in prefit_square_inputs)
    assert noise_norm_flags and all(flag is True for flag in noise_norm_flags)
    assert float(prefit_low.get("start_total_cost")) == pytest.approx(
        float(prefit_low.get("start_data_cost")) + float(prefit_low.get("start_prior_cost"))
    )
    assert float(prefit_high.get("start_total_cost")) == pytest.approx(
        float(prefit_high.get("start_data_cost")) + float(prefit_high.get("start_prior_cost"))
    )
    assert float(prefit_low.get("fitted_total_cost")) == pytest.approx(
        float(prefit_low.get("fitted_data_cost")) + float(prefit_low.get("fitted_prior_cost"))
    )
    assert float(prefit_high.get("fitted_total_cost")) == pytest.approx(
        float(prefit_high.get("fitted_data_cost")) + float(prefit_high.get("fitted_prior_cost"))
    )

    history_low = fit_low.get("history", [])
    history_high = fit_high.get("history", [])
    assert isinstance(history_low, list) and history_low
    assert isinstance(history_high, list) and history_high
    for hist in (history_low[-1], history_high[-1]):
        assert float(hist.get("cal_total_cost")) == pytest.approx(
            float(hist.get("cal_data_cost")) + float(hist.get("cal_prior_cost"))
        )

    assert r_low == pytest.approx(30.0, abs=1e-6)
    assert r_high == pytest.approx(38.7, abs=1e-6)


def test_spool_prefit_ellipse_can_reseed_before_anchor_step(monkeypatch):
    target_r = 15.0

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        out = {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
        }
        r = float(np.median(np.asarray(spool_params["radii_mm"], dtype=float)))
        out["_spool_r"] = r
        residual_drive = r - target_r
        sweep = {
            "drive_anchor": 0,
            "sensor_anchor": 1,
            "data_points": [
                {
                    "l_drive": float(residual_drive),
                    "l_sensor": 0.0,
                }
                for _ in range(6)
            ],
        }
        out["sweeps"].append(sweep)
        return out

    def fake_fit_ellipse_from_sweep(l_drive, l_sensor, **kwargs):
        _ = kwargs
        drive = np.asarray(l_drive, dtype=float).reshape(-1)
        sensor = np.asarray(l_sensor, dtype=float).reshape(-1)
        residual = abs(float(np.mean(drive))) + abs(float(np.mean(sensor)))
        return SimpleNamespace(residual_rms=float(residual))

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = (dataset_or_path, kwargs)
        return {"anchors": np.zeros((3, 2), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = (anchors, kwargs)
        r = float(dataset.get("_spool_r", 0.0))
        return float((r - target_r) ** 2.0)

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "fit_ellipse_from_sweep", fake_fit_ellipse_from_sweep)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    base = np.array([10.0, 10.0, 10.0], dtype=float)
    dataset = {"num_anchors": 3, "sweeps": [{}]}
    seed_anchors = np.zeros((3, 2), dtype=float)
    eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="global",
        find_buildup_mode="off",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=(10.0, 20.0),
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=2,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    prefit = fit_info.get("prefit", {})
    assert isinstance(prefit, dict)
    assert prefit.get("enabled") is True
    assert prefit.get("success") is True
    assert float(prefit.get("fitted_cost")) < float(prefit.get("start_cost"))
    assert abs(float(np.mean(eff_r)) - target_r) < abs(float(np.mean(base)) - target_r)


def test_bootstrap_anchor_refresh_runs_before_first_spool_radius_step(monkeypatch):
    target_r_with_refreshed_anchors = 41.25

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        return {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
            "_spool_r": float(np.median(np.asarray(spool_params["radii_mm"], dtype=float))),
        }

    cal_calls = []

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = dataset_or_path
        cal_calls.append(dict(kwargs))
        if len(cal_calls) == 1:
            return {"anchors": np.ones((3, 2), dtype=float), "cost": 0.0}
        return {"anchors": np.asarray(kwargs.get("initial_guess"), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = kwargs
        r = float(dataset.get("_spool_r", 30.0))
        anchor_mean = float(np.mean(np.asarray(anchors, dtype=float)))
        if anchor_mean < 0.5:
            return float((r - 30.0) ** 2.0 + 200.0)
        return float((r - target_r_with_refreshed_anchors) ** 2.0)

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    base = np.array([30.0, 30.0, 30.0], dtype=float)
    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="global",
        find_buildup_mode="off",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    bootstrap = fit_info.get("bootstrap_anchor_refresh", {})
    assert bootstrap.get("attempted") is True
    assert bootstrap.get("success") is True
    assert bootstrap.get("accepted") is True
    assert float(bootstrap.get("accepted_alpha")) == pytest.approx(1.0, abs=1e-9)
    history = fit_info.get("history", [])
    assert isinstance(history, list) and history
    assert float(history[0].get("current_data_cost")) > 100.0
    assert float(np.median(np.asarray(eff_r, dtype=float))) == pytest.approx(
        target_r_with_refreshed_anchors,
        abs=1e-6,
    )


def test_scale_fix_1_scales_anchor_seed_for_first_anchor_step(monkeypatch):
    base = np.array([30.0, 30.0, 30.0], dtype=float)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        return {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
            "_spool_r": float(np.median(np.asarray(spool_params["radii_mm"], dtype=float))),
        }

    cal_calls = []

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = dataset_or_path
        cal_calls.append(np.asarray(kwargs.get("initial_guess"), dtype=float))
        return {"anchors": np.asarray(kwargs.get("initial_guess"), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = kwargs
        r = float(dataset.get("_spool_r", 30.0))
        anchor_mean = float(np.mean(np.asarray(anchors, dtype=float)))
        return float((r - 40.0) ** 2.0 + 0.01 * (anchor_mean - (r / 30.0)) ** 2.0)

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    seed_anchors = np.ones((3, 2), dtype=float)
    _eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        {"num_anchors": 3, "sweeps": []},
        seed_anchors,
        find_radii_mode="global",
        find_buildup_mode="off",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
        scale_fix_levels=(1,),
    )

    assert len(cal_calls) >= 2
    assert float(np.mean(cal_calls[1])) > 1.2
    history = fit_info.get("history")
    assert isinstance(history, list) and history
    assert history[0].get("scale_fix_1_applied") is True


def test_scale_fix_2_runs_final_uniform_scale_polish(monkeypatch):
    base = np.array([30.0, 30.0, 30.0], dtype=float)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        return {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
            "_spool_r": float(np.median(np.asarray(spool_params["radii_mm"], dtype=float))),
        }

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = dataset_or_path
        return {"anchors": np.asarray(kwargs.get("initial_guess"), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = kwargs
        r = float(dataset.get("_spool_r", 30.0))
        anchor_mean = float(np.mean(np.asarray(anchors, dtype=float)))
        return float((r / max(anchor_mean, 1e-9) - 40.0) ** 2.0 + 5.0 * (anchor_mean - 1.05) ** 2.0)

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    seed_anchors = np.ones((3, 2), dtype=float)

    def _run(scale_fix_levels):
        eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
            {"num_anchors": 3, "sweeps": []},
            seed_anchors,
            find_radii_mode="global",
            find_buildup_mode="off",
            base_radii_mm=base,
            modeled_buildup_factor=np.zeros(3, dtype=float),
            spool_to_motor_gearing_factor=np.ones(3, dtype=float),
            mechanical_advantage=np.ones(3, dtype=float),
            lines_per_spool=np.ones(3, dtype=float),
            r0_bounds=None,
            b_bounds=None,
            r0_prior_sigma_mm=None,
            b_prior_sigma=None,
            spool_outer_iters=1,
            spool_inner_iters=1,
            theta0_mode="zero",
            solve_restarts=1,
            solve_iterations=10,
            solve_optimizer="L-BFGS-B",
            residual_threshold=1.0,
            spring_k_multiplier=1.0,
            use_flex=False,
            pointwise_residual_mode="sampson",
            pointwise_filtering=False,
            pointwise_global_mad=False,
            sweep_wise_filtering=False,
            sweep_metric="mad",
            use_noise_mean=False,
            sigma_source="auto",
            robust_debug=False,
            scale_fix_levels=scale_fix_levels,
        )
        return float(np.median(np.asarray(eff_r, dtype=float))), fit_info

    r_no_fix, _ = _run(tuple())
    r_fix2, fit_fix2 = _run((2,))

    assert r_fix2 > r_no_fix + 1.0
    final_polish = fit_fix2.get("final_scale_polish", {})
    assert final_polish.get("attempted") is True
    assert final_polish.get("accepted") is True


def test_scale_fix_3_runs_per_iteration_scale_polish(monkeypatch):
    base = np.array([30.0, 30.0, 30.0], dtype=float)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = prefer_zero_tension_angles
        return {
            "num_anchors": int(dataset.get("num_anchors", 3)),
            "sweeps": [],
            "_spool_r": float(np.median(np.asarray(spool_params["radii_mm"], dtype=float))),
        }

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = dataset_or_path
        return {"anchors": np.asarray(kwargs.get("initial_guess"), dtype=float), "cost": 0.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = kwargs
        r = float(dataset.get("_spool_r", 30.0))
        anchor_mean = float(np.mean(np.asarray(anchors, dtype=float)))
        return float((r / max(anchor_mean, 1e-9) - 40.0) ** 2.0 + 5.0 * (anchor_mean - 1.05) ** 2.0)

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    eff_r, _fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        {"num_anchors": 3, "sweeps": []},
        np.ones((3, 2), dtype=float),
        find_radii_mode="global",
        find_buildup_mode="off",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
        scale_fix_levels=(3,),
    )

    assert float(np.median(np.asarray(eff_r, dtype=float))) > 41.0
    history = fit_info.get("history")
    assert isinstance(history, list) and history
    assert history[0].get("accepted_update") == "anchor_scale_polish"
    scale_fix3 = history[0].get("scale_fix_3", {})
    assert isinstance(scale_fix3, dict)
    assert scale_fix3.get("attempted") is True
    assert scale_fix3.get("accepted") is True


def test_spool_block_update_rolls_back_when_not_improving(monkeypatch):
    base = np.array([10.0, 10.0, 10.0], dtype=float)

    def fake_build_spool_model_params(
        dataset,
        *,
        base_radii_mm,
        modeled_radii_mm,
        modeled_buildup_factor,
        spool_to_motor_gearing_factor,
        mechanical_advantage,
        lines_per_spool,
        base_buildup_factor=None,
        theta0_mode="zero",
        prefer_zero_tension_angles=False,
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
            theta0_mode,
            prefer_zero_tension_angles,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(
        dataset,
        spool_params,
        *,
        prefer_zero_tension_angles=False,
    ):
        _ = (spool_params, prefer_zero_tension_angles)
        return {"num_anchors": int(dataset.get("num_anchors", 3)), "sweeps": []}

    def fake_calibrate_elliptical(dataset_or_path, **kwargs):
        _ = (dataset_or_path, kwargs)
        anchors = np.ones((3, 2), dtype=float)
        return {"anchors": anchors, "cost": 1.0}

    def fake_evaluate_cost_at_anchors(dataset, anchors, **kwargs):
        _ = (dataset, kwargs)
        arr = np.asarray(anchors, dtype=float)
        return float(np.sum(arr**2.0))

    monkeypatch.setattr(ac, "build_spool_model_params", fake_build_spool_model_params)
    monkeypatch.setattr(ac, "dataset_with_modeled_lengths", fake_dataset_with_modeled_lengths)
    monkeypatch.setattr(ac, "calibrate_elliptical", fake_calibrate_elliptical)
    monkeypatch.setattr(ac, "_evaluate_cost_at_anchors", fake_evaluate_cost_at_anchors)

    dataset = {"num_anchors": 3, "sweeps": []}
    seed_anchors = np.zeros((3, 2), dtype=float)
    _eff_r, fit_anchors, _spool_params, _transformed, fit_info = ac._estimate_effective_radii_with_spool_model(
        dataset,
        seed_anchors,
        find_radii_mode="off",
        find_buildup_mode="global",
        base_radii_mm=base,
        modeled_buildup_factor=np.zeros(3, dtype=float),
        spool_to_motor_gearing_factor=np.ones(3, dtype=float),
        mechanical_advantage=np.ones(3, dtype=float),
        lines_per_spool=np.ones(3, dtype=float),
        r0_bounds=None,
        b_bounds=(0.0, 0.5),
        r0_prior_sigma_mm=None,
        b_prior_sigma=1e9,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        solve_restarts=1,
        solve_iterations=10,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
    )

    assert np.allclose(fit_anchors, seed_anchors)
    history = fit_info.get("history")
    assert isinstance(history, list) and history
    assert history[0].get("accepted_update") == "rollback"
