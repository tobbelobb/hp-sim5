import json

import numpy as np

import autocal.planning_pass as pp


def _write_dataset(path) -> None:
    payload = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0],
                "fixed_lengths": [10.0],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {
                        "l_drive": 1.0,
                        "l_sensor": 2.0,
                    }
                ],
            }
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_plan_next_ellipse_sweep_uses_explicit_warm_start_seeds(monkeypatch, tmp_path):
    dataset_path = tmp_path / "dataset.json"
    _write_dataset(dataset_path)

    initial_guess = np.asarray([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=float)
    initial_radii = np.asarray([39.1, 39.2, 39.3], dtype=float)
    initial_buildup = np.asarray([0.61, 0.62, 0.63], dtype=float)
    calibrate_calls = []
    spool_calls = []

    monkeypatch.setattr(pp, "_normalize_dataset_point_roles", lambda dataset: 0)
    monkeypatch.setattr(pp, "_require_machine_type", lambda dataset, **_kwargs: "slideprinter")
    monkeypatch.setattr(pp, "_annotate_dataset_noise_model", lambda *args, **kwargs: None)
    monkeypatch.setattr(pp, "_resolve_buildup_factor_seed", lambda *_args, **_kwargs: 0.636619)
    monkeypatch.setattr(pp, "validate_dataset_has_raw_angles", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        pp,
        "_resolve_length_model_base_params",
        lambda *_args, **_kwargs: {
            "base_radii_mm": [30.0, 30.0, 30.0],
            "spool_to_motor_gearing_factor": [1.0, 1.0, 1.0],
            "mechanical_advantage": [1.0, 1.0, 1.0],
            "lines_per_spool": [1.0, 1.0, 1.0],
        },
    )
    monkeypatch.setattr(
        pp,
        "_default_modeled_buildup_values",
        lambda num_anchors, **_kwargs: np.full(int(num_anchors), 0.636619, dtype=float),
    )
    monkeypatch.setattr(
        pp,
        "build_anchor_initial_guess",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("explicit warm start should bypass build_anchor_initial_guess")
        ),
    )

    def fake_calibrate(dataset_in, **kwargs):
        guess = np.asarray(kwargs.get("initial_guess"), dtype=float)
        calibrate_calls.append(guess)
        return {
            "anchors": guess,
            "cost": 1.0,
            "details": {"noise_metrics": {"chi2_red": 1.0, "J": 1.0}},
        }

    def fake_estimate(dataset_in, anchors, **kwargs):
        spool_calls.append(
            {
                "anchors": np.asarray(anchors, dtype=float),
                "initial_radii_mm": np.asarray(kwargs.get("initial_radii_mm"), dtype=float),
                "initial_buildup_factor": np.asarray(
                    kwargs.get("initial_buildup_factor"),
                    dtype=float,
                ),
            }
        )
        return (
            np.asarray([40.0, 40.0, 40.0], dtype=float),
            np.asarray(anchors, dtype=float) + 1.0,
            object(),
            dict(dataset_in),
            {"best_modeled_buildup_factor": np.asarray(initial_buildup, dtype=float)},
        )

    monkeypatch.setattr(pp, "calibrate_elliptical", fake_calibrate)
    monkeypatch.setattr(pp, "estimate_effective_radii_with_spool_model", fake_estimate)
    monkeypatch.setattr(pp, "dataset_sweep_configs", lambda dataset: [])
    monkeypatch.setattr(pp, "sweep_configs_with_modeled_lengths", lambda cfgs, _sp: list(cfgs))
    monkeypatch.setattr(pp, "total_information_matrix", lambda *args, **kwargs: np.eye(6, dtype=float))
    monkeypatch.setattr(pp, "_evaluate_cost_at_anchors", lambda *args, **kwargs: 1.0)
    monkeypatch.setattr(pp, "generate_candidate_sweeps", lambda *args, **kwargs: [])
    monkeypatch.setattr(pp, "_filter_candidates_by_spacing", lambda candidates, *_args, **_kwargs: candidates)
    monkeypatch.setattr(pp, "rank_candidates_d_optimal", lambda *args, **kwargs: [])

    plan = pp.plan_next_ellipse_sweep(
        dataset_path,
        solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
        residual_threshold=1.0,
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        sweep_metric="mad",
        use_noise_mean=False,
        sigma_source="auto",
        robust_debug=False,
        residuals_csv=None,
        generate_report=False,
        find_radii="global",
        find_buildup_factor="global",
        base_radii=[30.0, 30.0, 30.0],
        buildup_factor=0.636619,
        r0_bounds=None,
        b_bounds=None,
        r0_prior_sigma_mm=None,
        b_prior_sigma=None,
        spool_outer_iters=1,
        spool_inner_iters=1,
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        sigma_used_mm=None,
        candidate_deltas=None,
        candidate_count=4,
        delta_min=None,
        delta_max=None,
        fd_eps_mm=1.0,
        regularization=0.0,
        exclude_existing=True,
        existing_tol_mm=1.0,
        min_fixed_delta_spacing_mm=0.0,
        top_k=5,
        write_cfg=None,
        collector_output=tmp_path / "next.json",
        collector_args=[],
        initial_guess=initial_guess,
        initial_radii_mm=initial_radii,
        initial_buildup_factor=initial_buildup,
    )

    assert len(calibrate_calls) >= 1
    assert np.allclose(calibrate_calls[0], initial_guess)
    assert len(spool_calls) == 1
    assert np.allclose(spool_calls[0]["anchors"], initial_guess)
    assert np.allclose(spool_calls[0]["initial_radii_mm"], initial_radii)
    assert np.allclose(spool_calls[0]["initial_buildup_factor"], initial_buildup)
    assert np.allclose(np.asarray(plan["anchors"], dtype=float), initial_guess)
