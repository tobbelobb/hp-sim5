import numpy as np

from autocal import active_calibrate as ac


def _valid_sweep(idx: int) -> dict:
    return {
        "id": f"sweep_{idx:03d}",
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


def _write_dataset(path, *, sweeps: int) -> None:
    payload = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [_valid_sweep(i) for i in range(1, sweeps + 1)],
    }
    ac._write_json(path, payload)


def _layered_plan(
    *,
    primary_cost: float,
    cost_raw: float,
    chi2_layered: float,
    tau_mad_mm: float,
    n_trim: float,
    chi2_trimmed_direct=None,
    max_std_mm=None,
    workspace_diag_mm=None,
) -> dict:
    noise_metrics = {
        "chi2_red_rescored_tau_3bin_debiased": float(chi2_layered),
        "chi2_red_rescored": float(chi2_layered),
        "chi2_red_trimmed": float(chi2_layered),
        "tau_mad_mm": float(tau_mad_mm),
        "n_obs_trimmed": float(n_trim),
        "chi2_red": float(primary_cost),
        "J": float(primary_cost),
    }
    if chi2_trimmed_direct is not None:
        noise_metrics["chi2_red_tau_d_trimmed_direct"] = float(chi2_trimmed_direct)

    plan = {
        "anchors": np.asarray([[0.0, 0.0], [1645.0, 950.0], [-1645.0, 950.0]], dtype=float),
        "machine_type": "slideprinter",
        "cost": float(primary_cost),
        "cost_noise_normalized": float(primary_cost),
        "cost_raw": float(cost_raw),
        "covariance_scaled": np.eye(6, dtype=float),
        "collect_command": ["node", "autocal/control/cli/collect_sweep_data.mjs"],
        "length_model": {
            "find_radii_mode": "global",
            "find_buildup_factor_mode": "off",
            "effective_radii_mm": [39.2, 39.2, 39.2],
            "modeled_buildup_factor": [0.636619, 0.636619, 0.636619],
            "buildup_factor_k": 0.636619,
        },
        "calibration": {
            "details": {
                "noise_metrics": noise_metrics
            }
        },
    }
    if max_std_mm is not None:
        plan["confidence_intervals"] = {"max_std_mm": float(max_std_mm)}
    if workspace_diag_mm is not None:
        plan["workspace_diag_mm"] = float(workspace_diag_mm)
    return plan


def test_plan_score_ui_layered_keeps_good_fit_below_two_even_with_high_primary_cost():
    plan = _layered_plan(
        primary_cost=74.11,
        cost_raw=0.2557,
        chi2_layered=64.0,
        tau_mad_mm=0.8,
        n_trim=41.0,
    )

    score_ui, rank_score, basis = ac._plan_score_ui(plan)
    score_ui_recomputed, m_layered = ac._compute_score_ui_layered(plan)
    expected_rank = ac._layered_rank_score_from_internal_metric(
        m_layered,
        cost_raw=float(plan["cost_raw"]),
    )
    assert basis == "layered-calibrated"
    assert score_ui < 2.0
    assert np.isclose(score_ui, score_ui_recomputed, atol=1e-12)
    assert np.isclose(rank_score, expected_rank, atol=1e-12)
    assert ac._solution_quality_label(score_ui) == "ideal"
    assert "near-perfect fit" in ac._solution_quality_message(score_ui)


def test_plan_score_ui_layered_prefers_trimmed_direct_risk_metric():
    plan = _layered_plan(
        primary_cost=74.11,
        cost_raw=0.3,
        chi2_layered=140.0,
        tau_mad_mm=1.2,
        n_trim=35.0,
        chi2_trimmed_direct=40.0,
        max_std_mm=2.0,
        workspace_diag_mm=100.0,
    )

    expected_m = 40.0 * (2.0 / 100.0)
    expected_m *= (
        1.0
        + ac._SCORE_UI_LAYERED_N_TRIM_WEIGHT
        * max(0.0, (ac._SCORE_UI_LAYERED_N_TRIM_REF - 35.0) / 10.0)
    )
    expected_m *= (
        1.0
        + ac._SCORE_UI_LAYERED_TAU_MAD_WEIGHT
        * max(0.0, 1.2 / ac._SCORE_UI_LAYERED_TAU_MAD_REF_MM - 1.0)
    )
    expected_rank = ac._layered_rank_score_from_internal_metric(
        expected_m,
        cost_raw=float(plan["cost_raw"]),
    )

    noise_metrics = plan["calibration"]["details"]["noise_metrics"]
    fallback_m = ac._layered_internal_metric_from_noise_metrics(
        noise_metrics,
        cost_raw=float(plan["cost_raw"]),
    )

    score_ui, rank_score, basis = ac._plan_score_ui(plan)
    score_ui_recomputed, m_layered = ac._compute_score_ui_layered(plan)
    assert basis == "layered-calibrated"
    assert fallback_m is not None
    assert np.isclose(ac._plan_trimmed_risk_metric(plan), expected_m, atol=1e-12)
    assert np.isclose(m_layered, expected_m, atol=1e-12)
    assert not np.isclose(m_layered, fallback_m, atol=1e-12)
    assert np.isclose(score_ui, score_ui_recomputed, atol=1e-12)
    assert np.isclose(rank_score, expected_rank, atol=1e-12)


def test_plan_score_ui_layered_falls_back_when_trimmed_direct_risk_missing_rel_std():
    plan = _layered_plan(
        primary_cost=74.11,
        cost_raw=0.2557,
        chi2_layered=64.0,
        tau_mad_mm=0.8,
        n_trim=41.0,
        chi2_trimmed_direct=40.0,
    )

    noise_metrics = plan["calibration"]["details"]["noise_metrics"]
    expected_m = ac._layered_internal_metric_from_noise_metrics(
        noise_metrics,
        cost_raw=float(plan["cost_raw"]),
    )
    assert expected_m is not None
    assert ac._plan_trimmed_risk_metric(plan) is None

    score_ui, rank_score, basis = ac._plan_score_ui(plan)
    score_ui_recomputed, m_layered = ac._compute_score_ui_layered(plan)
    expected_rank = ac._layered_rank_score_from_internal_metric(
        expected_m,
        cost_raw=float(plan["cost_raw"]),
    )

    assert basis == "layered-calibrated"
    assert np.isclose(m_layered, expected_m, atol=1e-12)
    assert np.isclose(score_ui, score_ui_recomputed, atol=1e-12)
    assert np.isclose(rank_score, expected_rank, atol=1e-12)


def test_plan_score_ui_layered_prefers_full_data_ranking_metrics_and_cost_raw():
    baseline_plan = _layered_plan(
        primary_cost=74.11,
        cost_raw=0.3,
        chi2_layered=64.0,
        tau_mad_mm=0.8,
        n_trim=41.0,
    )
    baseline_score_ui, _, _ = ac._plan_score_ui(baseline_plan)
    assert baseline_score_ui < ac._SCORE_UI_HARD_FAIL

    plan = _layered_plan(
        primary_cost=74.11,
        cost_raw=0.3,
        chi2_layered=64.0,
        tau_mad_mm=0.8,
        n_trim=41.0,
    )
    plan["ranking_noise_metrics"] = {
        "chi2_red_rescored_tau_3bin_debiased": 180.0,
        "chi2_red_rescored": 180.0,
        "chi2_red_trimmed": 180.0,
        "tau_mad_mm": 1.6,
        "n_obs_trimmed": 24.0,
        "chi2_red": 180.0,
        "J": 180.0,
    }
    plan["ranking_cost_raw"] = 6.5

    score_ui, rank_score, basis = ac._plan_score_ui(plan)
    score_ui_recomputed, m_layered = ac._compute_score_ui_layered(plan)
    expected_rank = ac._layered_rank_score_from_internal_metric(
        m_layered,
        cost_raw=float(plan["ranking_cost_raw"]),
    )

    assert basis == "layered-calibrated"
    assert score_ui >= ac._SCORE_UI_HARD_FAIL
    assert np.isclose(score_ui, score_ui_recomputed, atol=1e-12)
    assert np.isclose(rank_score, expected_rank, atol=1e-12)


def test_plan_score_ui_layered_uses_hard_fail_for_bad_raw_geometry():
    plan = _layered_plan(
        primary_cost=74.11,
        cost_raw=6.0,
        chi2_layered=64.0,
        tau_mad_mm=0.8,
        n_trim=41.0,
    )

    score_ui, _, _ = ac._plan_score_ui(plan)
    assert score_ui >= 50.0
    assert ac._solution_quality_label(score_ui) == "concerning"


def test_plan_score_ui_non_layered_matches_primary_cost():
    plan = {
        "anchors": np.asarray([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
        "machine_type": "slideprinter",
        "cost": 4.2,
        "cost_noise_normalized": 4.2,
        "covariance_scaled": np.eye(6, dtype=float),
    }

    score_ui, rank_score, basis = ac._plan_score_ui(plan)
    assert basis == "standard-noise"
    assert np.isclose(score_ui, 4.2, atol=1e-12)
    assert np.isclose(rank_score, 4.2, atol=1e-12)
    assert ac._solution_quality_label(score_ui) == "good"


def test_full_auto_loop_ranks_variants_by_score_ui_not_primary_cost(
    tmp_path,
    monkeypatch,
    capsys,
):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=3)

    low_cost_bad_score = _layered_plan(
        primary_cost=10.0,
        cost_raw=0.3,
        chi2_layered=145.0,
        tau_mad_mm=1.2,
        n_trim=35.0,
    )
    high_cost_good_score = _layered_plan(
        primary_cost=20.0,
        cost_raw=0.3,
        chi2_layered=64.0,
        tau_mad_mm=0.8,
        n_trim=41.0,
    )
    planned_runs = [low_cost_bad_score, high_cost_good_score]

    def fake_plan_next(*_args, **_kwargs):
        assert planned_runs
        return planned_runs.pop(0)

    monkeypatch.setattr(ac, "_plan_next_ellipse_sweep", fake_plan_next)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_start_rrf_simulator", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_wait_for_rrf_server", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac.subprocess, "run", lambda *_args, **_kwargs: None)

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=1,
        stop_cost=None,
        stop_std_mm=None,
        solve_restarts=1,
        solve_iterations=1,
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
        residuals_csv=None,
        generate_report=False,
        find_radii="global",
        find_buildup_factor="off",
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
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        fd_eps_mm=1.0,
        regularization=0.0,
        exclude_existing=True,
        existing_tol_mm=1.0,
        min_fixed_delta_spacing_mm=0.0,
        top_k=5,
        write_cfg=None,
        collector_args=[],
        sim=True,
        keep_sim_alive=False,
        hp_sim_reset=False,
        sweep_points=None,
        output_with_explanations=False,
        full_auto_runs=["--solve-iterations 1", "--solve-iterations 2"],
        full_auto_log=None,
        patience=20,
        full_auto_verbose=False,
        no_collect=True,
    )

    assert rc == 0
    out = capsys.readouterr().out
    assert "; selected run=run_02" in out
    assert "Fit quality score (lower is better):" in out
    assert "Quality score below 2 indicates a near-perfect fit." in out


def test_full_auto_loop_prefers_full_data_ranking_metrics(
    tmp_path,
    monkeypatch,
    capsys,
):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=3)

    low_calibration_cost = _layered_plan(
        primary_cost=10.0,
        cost_raw=0.3,
        chi2_layered=40.0,
        tau_mad_mm=0.7,
        n_trim=45.0,
    )
    low_calibration_cost["ranking_noise_metrics"] = {
        "chi2_red_rescored_tau_3bin_debiased": 220.0,
        "chi2_red_rescored": 220.0,
        "chi2_red_trimmed": 220.0,
        "tau_mad_mm": 1.5,
        "n_obs_trimmed": 24.0,
        "chi2_red": 220.0,
        "J": 220.0,
    }
    low_calibration_cost["ranking_cost_raw"] = 0.3

    high_calibration_cost = _layered_plan(
        primary_cost=20.0,
        cost_raw=0.3,
        chi2_layered=100.0,
        tau_mad_mm=1.2,
        n_trim=32.0,
    )
    high_calibration_cost["ranking_noise_metrics"] = {
        "chi2_red_rescored_tau_3bin_debiased": 36.0,
        "chi2_red_rescored": 36.0,
        "chi2_red_trimmed": 36.0,
        "tau_mad_mm": 0.6,
        "n_obs_trimmed": 52.0,
        "chi2_red": 36.0,
        "J": 36.0,
    }
    high_calibration_cost["ranking_cost_raw"] = 0.3

    planned_runs = [low_calibration_cost, high_calibration_cost]

    def fake_plan_next(*_args, **_kwargs):
        assert planned_runs
        return planned_runs.pop(0)

    monkeypatch.setattr(ac, "_plan_next_ellipse_sweep", fake_plan_next)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_start_rrf_simulator", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_wait_for_rrf_server", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac.subprocess, "run", lambda *_args, **_kwargs: None)

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=1,
        stop_cost=None,
        stop_std_mm=None,
        solve_restarts=1,
        solve_iterations=1,
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
        residuals_csv=None,
        generate_report=False,
        find_radii="global",
        find_buildup_factor="off",
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
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        fd_eps_mm=1.0,
        regularization=0.0,
        exclude_existing=True,
        existing_tol_mm=1.0,
        min_fixed_delta_spacing_mm=0.0,
        top_k=5,
        write_cfg=None,
        collector_args=[],
        sim=True,
        keep_sim_alive=False,
        hp_sim_reset=False,
        sweep_points=None,
        output_with_explanations=False,
        full_auto_runs=["--solve-iterations 1", "--solve-iterations 2"],
        full_auto_log=None,
        patience=20,
        full_auto_verbose=False,
        no_collect=True,
    )

    assert rc == 0
    out = capsys.readouterr().out
    assert "; selected run=run_02" in out
