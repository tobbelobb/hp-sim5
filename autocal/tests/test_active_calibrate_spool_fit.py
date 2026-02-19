import numpy as np

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
    ):
        _ = (
            dataset,
            base_radii_mm,
            spool_to_motor_gearing_factor,
            mechanical_advantage,
            lines_per_spool,
            base_buildup_factor,
        )
        return {
            "radii_mm": np.asarray(modeled_radii_mm, dtype=float).reshape(-1),
            "buildup_factor": np.asarray(modeled_buildup_factor, dtype=float).reshape(-1),
        }

    def fake_dataset_with_modeled_lengths(dataset, spool_params):
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

    assert len(calls) == 2
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
