import shlex

import numpy as np

import autocal.autocal as ac
from autocal.sweep_types import DataPoint, MachineConfig, MachineType, Sweep


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


def _fake_plan() -> dict:
    return {
        "anchors": np.asarray([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
        "machine_type": "slideprinter",
        "cost": 1.0,
        "cost_noise_normalized": 1.0,
        "covariance_scaled": np.eye(6, dtype=float),
        "collect_command": ["node", "autocal/control/cli/collect_sweep_data.mjs"],
    }


def test_build_semi_auto_parser_accepts_no_collect():
    parser = ac.build_semi_auto_parser()
    args = parser.parse_args(
        [
            "--machine-type",
            "slideprinter",
            "--no-collect",
        ]
    )
    assert args.no_collect is True


def test_default_delta_range_with_max_travel_is_bidirectional_even_if_observed_positive_only():
    lo, hi = ac._default_delta_range(
        max_travel_mm=600.0,
        observed_deltas=[120.0, 240.0, 360.0],
    )
    assert np.isclose(float(lo), -600.0, atol=1e-9)
    assert np.isclose(float(hi), 600.0, atol=1e-9)


def test_default_delta_range_without_max_travel_uses_observed_span_bidirectionally():
    lo, hi = ac._default_delta_range(
        max_travel_mm=None,
        observed_deltas=[80.0, 140.0, 220.0],
    )
    assert np.isclose(float(lo), -220.0, atol=1e-9)
    assert np.isclose(float(hi), 220.0, atol=1e-9)


def test_print_ellipse_plan_includes_noise_rescore_line(capsys):
    plan = _fake_plan()
    plan["calibration"] = {
        "details": {
            "noise_metrics": {
                "J": 74.11,
                "chi2_red": 91.2,
                "tau_mad_mm": 2.5,
                "cost_noise_normalized_old": 74.11,
                "cost_noise_normalized_rescored": 10.4,
                "chi2_red_old": 91.2,
                "chi2_red_rescored": 12.8,
                "residual_vs_distance_slope_inliers": 0.02,
                "residual_abs_vs_distance_slope": 0.01,
                "residual_sq_vs_distance_slope": 0.04,
                "inlier_cutoff_mm": 2.4,
                "n_inlier_rows_used_for_tau": 17,
                "tau_d_tau0_mm": 2.0,
                "tau_d_tau1_per_mm": 0.001,
                "cost_noise_normalized_tau_d": 8.0,
                "chi2_red_tau_d": 9.0,
                "cost_noise_normalized_tau_d_trimmed_direct": 6.2,
                "chi2_red_tau_d_trimmed_direct": 7.3,
                "bias_vs_distance_intercept_mm": -0.3,
                "bias_vs_distance_slope_inliers": 0.002,
                "cost_after_bias_diagnostic": 6.5,
                "chi2_red_after_bias_diagnostic": 7.5,
                "distance_bin_mad_mm": {
                    "near": 0.4,
                    "mid": 0.8,
                    "far": 1.6,
                },
                "distance_bin_mad_debiased_mm": {
                    "near": 0.2,
                    "mid": 0.3,
                    "far": 0.4,
                },
                "distance_bin_counts": {
                    "near": 4,
                    "mid": 5,
                    "far": 3,
                },
                "tau_3bin_debiased_mm": {
                    "near": 0.5,
                    "mid": 0.7,
                    "far": 1.1,
                },
                "cost_noise_normalized_rescored_tau_3bin_debiased": 5.8,
                "chi2_red_rescored_tau_3bin_debiased": 6.8,
                "chi2_red_tau_3bin_debiased_trimmed_direct": 6.8,
                "trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased": 7.3 / 6.8,
                "per_sweep_residual_summary": {
                    "sweep_001": {
                        "median_residual_mm": 1.0,
                        "mad_residual_mm": 0.5,
                        "p95_abs_residual_mm": 2.0,
                        "mean_abs_z": 1.2,
                        "clipped_points": 1,
                        "total_points": 5,
                    }
                },
                "per_sweep_demean": {
                    "cost_noise_normalized_demeaned": 5.5,
                    "chi2_red_demeaned": 6.0,
                    "tail_ratio": 3.4,
                    "sweep_bias_span_mm": 2.1,
                    "n_obs_demeaned": 12,
                },
            }
        }
    }

    ac._print_ellipse_plan(plan, print_command=False)
    out = capsys.readouterr().out
    assert "noise_rescore:" in out
    assert "tau_MAD=2.5mm" in out
    assert "cost_noise_normalized_old=74.11" in out
    assert "cost_noise_normalized_new=10.4" in out
    assert "chi2_red_old=91.2" in out
    assert "chi2_red_new=12.8" in out
    assert "residual_vs_distance_slope_inliers=0.02" in out
    assert "slope(|r|,d)=0.01" in out
    assert "slope(r^2,d)=0.04" in out
    assert "inlier_cutoff_mm=2.4mm" in out
    assert "n_inlier_rows_used_for_tau=17" in out
    assert "tau_d_tau0=2mm" in out
    assert "tau_d_tau1=0.001" in out
    assert "cost_noise_normalized_tau_d=8" in out
    assert "chi2_red_tau_d=9" in out
    assert "cost_noise_normalized_tau_d_trimmed_direct=6.2" in out
    assert "chi2_red_tau_d_trimmed_direct=7.3" in out
    assert "chi2_red_tau_3bin_debiased_trimmed_direct=6.8" in out
    assert "trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased=1.074" in out
    assert "noise_bias_diagnostic:" in out
    assert "bias_vs_distance_intercept=-0.3mm" in out
    assert "bias_vs_distance_slope_inliers=0.002" in out
    assert "cost_after_bias_diagnostic=6.5" in out
    assert "chi2_red_after_bias_diagnostic=7.5" in out
    assert "noise_rescore_bins:" in out
    assert "near_MAD=0.4mm" in out
    assert "mid_MAD=0.8mm" in out
    assert "far_MAD=1.6mm" in out
    assert "N=[4,5,3]" in out
    assert "noise_rescore_bins_debiased:" in out
    assert "near_MAD=0.2mm" in out
    assert "mid_MAD=0.3mm" in out
    assert "far_MAD=0.4mm" in out
    assert "noise_rescore_tau_3bin_debiased:" in out
    assert "tau_near=0.5mm" in out
    assert "tau_mid=0.7mm" in out
    assert "tau_far=1.1mm" in out
    assert "cost_noise_normalized_tau_3bin_debiased=5.8" in out
    assert "chi2_red_tau_3bin_debiased=6.8" in out
    assert "chi2_red_tau_3bin_debiased_trimmed_direct=6.8" in out
    assert "sweep_residual: id=sweep_001" in out
    assert "median=1mm" in out
    assert "MAD=0.5mm" in out
    assert "p95_abs=2mm" in out
    assert "mean|z|=1.2" in out
    assert "clipped=1/5" in out
    assert "noise_demean_by_sweep:" in out
    assert "cost_noise_normalized_demeaned=5.5" in out
    assert "chi2_red_demeaned=6" in out
    assert "tail_ratio=3.4" in out
    assert "sweep_bias_span=2.1mm" in out


def test_print_ellipse_plan_logs_plain_anchors_each_iteration_for_hangprinter(capsys):
    plan = {
        "anchors": np.asarray(
            [
                [0.0, -1900.004, -280.0],
                [1645.004, 950.005, -280.0],
                [-1645.006, 950.994, -280.0],
                [0.0, 0.0, 1900.444],
            ],
            dtype=float,
        ),
        "machine_type": "hangprinter_4",
        "cost": 1.0,
        "cost_noise_normalized": 1.0,
        "covariance_scaled": np.eye(12, dtype=float),
        "collect_command": ["node", "autocal/control/cli/collect_sweep_data.mjs"],
        "calibration": {
            "gcode": (
                "M669 A0.00:-1900.00:-280.00 B1645.00:950.00:-280.00 "
                "C-1645.00:950.00:-280.00 D0.00:0.00:1900.00"
            )
        },
    }

    ac._print_ellipse_plan(plan, print_command=False)

    out = capsys.readouterr().out
    assert (
        "; Anchors: [[0.00, -1900.00, -280.00], [1645.00, 950.00, -280.00], "
        "[-1645.01, 950.99, -280.00], [0.00, 0.00, 1900.44]]"
    ) in out
    assert "M669 A0.00:-1900.00:-280.00" in out


def test_full_auto_loop_no_collect_exits_when_replay_depletes(tmp_path, monkeypatch):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=5)

    sent = []

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    def fail_run(*_args, **_kwargs):
        raise AssertionError("subprocess.run should not be called with --no-collect")

    def fail_start(*_args, **_kwargs):
        raise AssertionError("rrf_simulator should not be started with --sim --no-collect")

    def fail_wait(*_args, **_kwargs):
        raise AssertionError("rrf_simulator wait should not run with --sim --no-collect")

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", lambda *_args, **_kwargs: _fake_plan())
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_send_rrf_gcode", fake_send)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_start_rrf_simulator", fail_start)
    monkeypatch.setattr(ac, "_wait_for_rrf_server", fail_wait)
    monkeypatch.setattr(ac.subprocess, "run", fail_run)

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=8,
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
        find_radii="off",
        find_buildup_factor="off",
        base_radii=None,
        buildup_factor=None,
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
        full_auto_runs=None,
        full_auto_log=None,
        patience=20,
        verbose=False,
        no_collect=True,
    )

    assert rc == 0
    assert len(sent) == 0


def test_full_auto_loop_logs_invoked_command_near_top(tmp_path, monkeypatch):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=5)

    sent = []
    fake_argv = [
        "python",
        "autocal/autocal.py",
        "--sim",
        "--dataset",
        str(dataset),
    ]
    monkeypatch.setattr(ac.sys, "argv", fake_argv)

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    def fail_run(*_args, **_kwargs):
        raise AssertionError("subprocess.run should not be called with --no-collect")

    def fail_start(*_args, **_kwargs):
        raise AssertionError("rrf_simulator should not be started with --sim --no-collect")

    def fail_wait(*_args, **_kwargs):
        raise AssertionError("rrf_simulator wait should not run with --sim --no-collect")

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", lambda *_args, **_kwargs: _fake_plan())
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_send_rrf_gcode", fake_send)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_start_rrf_simulator", fail_start)
    monkeypatch.setattr(ac, "_wait_for_rrf_server", fail_wait)
    monkeypatch.setattr(ac.subprocess, "run", fail_run)

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=8,
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
        find_radii="off",
        find_buildup_factor="off",
        base_radii=None,
        buildup_factor=None,
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
        full_auto_runs=None,
        full_auto_log=None,
        patience=20,
        verbose=False,
        no_collect=True,
    )

    assert rc == 0
    assert len(sent) == 0
    log_text = (tmp_path / "full_dataset.full_auto.log").read_text(encoding="utf-8")
    first_two_lines = log_text.splitlines()[:2]
    assert f"; command: {shlex.join(fake_argv)}" in first_two_lines


def test_write_bootstrap_sweep_config_uses_machine_specific_constraints(tmp_path):
    cfg_path = tmp_path / "hangprinter_4.bootstrap_cfg.txt"

    written = ac._write_bootstrap_sweep_config(cfg_path, machine_type="hangprinter_4")

    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    lines = [line for line in cfg_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    assert written == 3
    assert len(lines) == 3

    for idx, line in enumerate(lines, start=1):
        fixed_token, drive_token, sensor_token = line.split()
        fixed_anchors = [int(value) for value in fixed_token.strip("[]").split(",") if value]
        assert 3 in fixed_anchors
        assert int(drive_token) != 3
        assert int(sensor_token) != 3
        sweep = Sweep(
            id=f"sweep_{idx:03d}",
            fixed_anchors=fixed_anchors,
            fixed_lengths=[10.0] * len(fixed_anchors),
            drive_anchor=int(drive_token),
            sensor_anchor=int(sensor_token),
            data_points=[DataPoint(0.0, 0.0) for _ in range(5)],
        )
        assert sweep.validate(config) == []


def test_full_auto_accepts_best_historical_replay_plan_after_patience(tmp_path, monkeypatch, capsys):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=5)

    sent = []
    plan_queue = [
        {
            "marker": "iter_1",
            "anchors": np.asarray([[10.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.0,
            "cost_raw": 1.0,
            "cost_noise_normalized": 1.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
        {
            "marker": "iter_2",
            "anchors": np.asarray([[20.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 2.0,
            "cost_raw": 2.0,
            "cost_noise_normalized": 2.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
        {
            "marker": "iter_3",
            "anchors": np.asarray([[30.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 3.0,
            "cost_raw": 3.0,
            "cost_noise_normalized": 3.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
    ]

    def fake_plan(*_args, **_kwargs):
        assert plan_queue, "expected one replay plan per iteration"
        return plan_queue.pop(0)

    def fake_score(plan):
        score = float(plan["cost"])
        return score, score, "standard-noise"

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "_plan_score_ui", fake_score)
    monkeypatch.setattr(ac, "_plan_primary_cost", fake_cost)
    monkeypatch.setattr(ac, "_plan_covariance_summary", fake_cov_summary)
    monkeypatch.setattr(ac, "_plan_data_quality_warnings", lambda _plan: [])
    monkeypatch.setattr(ac, "_plan_noise_metrics", lambda _plan: {"chi2_red": 1.0, "J": 1.0})
    monkeypatch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_m669_from_plan", lambda plan: f"M669 {plan['marker']}")
    monkeypatch.setattr(ac, "_m666_from_plan", lambda _plan: "")
    monkeypatch.setattr(ac, "_send_rrf_gcode", fake_send)

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=8,
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
        find_radii="off",
        find_buildup_factor="off",
        base_radii=None,
        buildup_factor=None,
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
        full_auto_runs=None,
        full_auto_log=None,
        patience=2,
        verbose=False,
        no_collect=True,
    )

    assert rc == 0
    assert sent == []
    out = capsys.readouterr().out
    assert "M669 iter_1" in out
    assert "M669 iter_3" not in out
    assert "Rank score:" not in out
    assert "Fit score:" not in out
    assert "try so far." in out
    assert "Replaying next sweep to try and beat it." in out
    assert "Still has patience for 2 more attempts." in out
    assert "Patience reset." in out


def test_full_auto_history_selector_can_prefer_mid_late_replay_plan(tmp_path, monkeypatch, capsys):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=7)

    sent = []
    plan_queue = [
        {
            "marker": "iter_1",
            "anchors": np.asarray([[10.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.0,
            "cost_raw": 1.0,
            "cost_noise_normalized": 1.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
        {
            "marker": "iter_2",
            "anchors": np.asarray([[20.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.12,
            "cost_raw": 1.12,
            "cost_noise_normalized": 1.12,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
        {
            "marker": "iter_3",
            "anchors": np.asarray([[30.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.10,
            "cost_raw": 1.10,
            "cost_noise_normalized": 1.10,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
        {
            "marker": "iter_4",
            "anchors": np.asarray([[40.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.40,
            "cost_raw": 1.40,
            "cost_noise_normalized": 1.40,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
        {
            "marker": "iter_5",
            "anchors": np.asarray([[50.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.55,
            "cost_raw": 1.55,
            "cost_noise_normalized": 1.55,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
    ]

    def fake_plan(*_args, **_kwargs):
        assert plan_queue, "expected one replay plan per iteration"
        return plan_queue.pop(0)

    def fake_score(plan):
        score = float(plan["cost"])
        return score, score, "standard-noise"

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "_plan_score_ui", fake_score)
    monkeypatch.setattr(ac, "_plan_primary_cost", fake_cost)
    monkeypatch.setattr(ac, "_plan_covariance_summary", fake_cov_summary)
    monkeypatch.setattr(ac, "_plan_data_quality_warnings", lambda _plan: [])
    monkeypatch.setattr(ac, "_plan_noise_metrics", lambda _plan: {"chi2_red": 1.0, "J": 1.0})
    monkeypatch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_m669_from_plan", lambda plan: f"M669 {plan['marker']}")
    monkeypatch.setattr(ac, "_m666_from_plan", lambda _plan: "")
    monkeypatch.setattr(ac, "_send_rrf_gcode", fake_send)

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=8,
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
        find_radii="off",
        find_buildup_factor="off",
        base_radii=None,
        buildup_factor=None,
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
        full_auto_runs=None,
        full_auto_log=None,
        patience=3,
        verbose=False,
        no_collect=True,
    )

    assert rc == 0
    assert sent == []
    out = capsys.readouterr().out
    assert "M669 iter_3" in out


def test_full_auto_underconstrained_recovery_reuses_warm_start_and_keeps_collect_decision(
    tmp_path,
    monkeypatch,
    capsys,
):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=3)

    sent = []
    call_kwargs = []

    def _plan(marker: str, anchors: np.ndarray, cost: float) -> dict:
        return {
            "marker": marker,
            "anchors": np.asarray(anchors, dtype=float),
            "machine_type": "slideprinter",
            "cost": float(cost),
            "cost_raw": float(cost),
            "cost_noise_normalized": float(cost),
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
            "length_model": {
                "effective_radii_mm": [39.1, 39.2, 39.3],
                "modeled_buildup_factor": [0.61, 0.62, 0.63],
            },
        }

    base_under = _plan(
        "under_1",
        np.asarray([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=float),
        100.0,
    )
    sweep_recovery_under = _plan(
        "under_2",
        np.asarray([[2.0, 0.0], [0.0, 2.0], [2.0, 2.0]], dtype=float),
        100.0,
    )
    constrained = _plan(
        "recovered",
        np.asarray([[3.0, 0.0], [0.0, 3.0], [3.0, 3.0]], dtype=float),
        2.0,
    )
    planned_runs = [base_under, sweep_recovery_under, constrained]

    def fake_plan_next(*_args, **kwargs):
        call_kwargs.append(dict(kwargs))
        assert planned_runs, "expected a recovery plan for each attempt"
        return planned_runs.pop(0)

    def fake_score(plan):
        score = float(plan["cost"])
        return score, score, "standard-noise"

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan_next)
    monkeypatch.setattr(ac, "_plan_score_ui", fake_score)
    monkeypatch.setattr(ac, "_plan_primary_cost", fake_cost)
    monkeypatch.setattr(ac, "_plan_covariance_summary", fake_cov_summary)
    monkeypatch.setattr(ac, "_plan_data_quality_warnings", lambda _plan: [])
    monkeypatch.setattr(
        ac,
        "_plan_noise_metrics",
        lambda plan: {"chi2_red": float(plan["cost"]), "J": float(plan["cost"])},
    )
    monkeypatch.setattr(
        ac,
        "_plan_hits_underconstrained_penalty",
        lambda plan, *_args, **_kwargs: float(plan["cost"]) == 100.0,
    )
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_m669_from_plan", lambda plan: f"M669 {plan['marker']}")
    monkeypatch.setattr(ac, "_m666_from_plan", lambda _plan: "")
    monkeypatch.setattr(ac, "_send_rrf_gcode", fake_send)
    monkeypatch.setattr(
        ac.subprocess,
        "run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("subprocess.run should not be called after recovery succeeds")
        ),
    )

    rc = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=3,
        stop_cost=None,
        stop_std_mm=None,
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
        full_auto_runs=None,
        full_auto_log=None,
        patience=5,
        verbose=False,
        filter_schedule=["dynamic", "constant"],
        objective_schedule=[1, 2],
        no_collect=True,
    )

    assert rc == 0
    assert sent == []
    assert len(call_kwargs) == 3

    assert call_kwargs[0]["sweep_wise_filtering"] is True
    assert call_kwargs[0]["pointwise_filtering"] is True
    assert call_kwargs[0]["initial_guess"] is None

    assert call_kwargs[1]["sweep_wise_filtering"] is False
    assert call_kwargs[1]["pointwise_filtering"] is True
    assert call_kwargs[1]["filter_schedule"] == ["dynamic", "constant"]
    assert np.allclose(
        np.asarray(call_kwargs[1]["initial_guess"], dtype=float),
        np.asarray(base_under["anchors"], dtype=float),
    )
    assert np.allclose(
        np.asarray(call_kwargs[1]["initial_radii_mm"], dtype=float),
        np.asarray([39.1, 39.2, 39.3], dtype=float),
    )
    assert np.allclose(
        np.asarray(call_kwargs[1]["initial_buildup_factor"], dtype=float),
        np.asarray([0.61, 0.62, 0.63], dtype=float),
    )

    assert call_kwargs[2]["sweep_wise_filtering"] is False
    assert call_kwargs[2]["pointwise_filtering"] is False
    assert call_kwargs[2]["filter_schedule"] == ["warmup"]
    assert call_kwargs[2]["objective_schedule"] == [2]
    assert np.allclose(
        np.asarray(call_kwargs[2]["initial_guess"], dtype=float),
        np.asarray(sweep_recovery_under["anchors"], dtype=float),
    )

    out = capsys.readouterr().out
    assert "underconstrained recovery succeeded" in out
    assert "selected run hit underconstrained sentinel" not in out
    assert "; --no-collect set; stopping before live collection." in out


def test_full_auto_underconstrained_recovery_still_replays_next_sweep(
    tmp_path,
    monkeypatch,
    capsys,
):
    dataset = tmp_path / "full_dataset.json"
    _write_dataset(dataset, sweeps=5)

    plan_queue = [
        {
            "marker": "under_1",
            "anchors": np.asarray([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 100.0,
            "cost_raw": 100.0,
            "cost_noise_normalized": 100.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
            "length_model": {
                "effective_radii_mm": [39.1, 39.2, 39.3],
                "modeled_buildup_factor": [0.61, 0.62, 0.63],
            },
        },
        {
            "marker": "recovered",
            "anchors": np.asarray([[2.0, 0.0], [0.0, 2.0], [2.0, 2.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 2.0,
            "cost_raw": 2.0,
            "cost_noise_normalized": 2.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
            "length_model": {
                "effective_radii_mm": [39.1, 39.2, 39.3],
                "modeled_buildup_factor": [0.61, 0.62, 0.63],
            },
        },
    ]

    def fake_plan(*_args, **_kwargs):
        assert plan_queue, "expected replay recovery plans"
        return plan_queue.pop(0)

    def fake_score(plan):
        score = float(plan["cost"])
        return score, score, "standard-noise"

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "_plan_score_ui", fake_score)
    monkeypatch.setattr(ac, "_plan_primary_cost", fake_cost)
    monkeypatch.setattr(ac, "_plan_covariance_summary", fake_cov_summary)
    monkeypatch.setattr(ac, "_plan_data_quality_warnings", lambda _plan: [])
    monkeypatch.setattr(
        ac,
        "_plan_noise_metrics",
        lambda plan: {"chi2_red": float(plan["cost"]), "J": float(plan["cost"])},
    )
    monkeypatch.setattr(
        ac,
        "_plan_hits_underconstrained_penalty",
        lambda plan, *_args, **_kwargs: float(plan["cost"]) == 100.0,
    )
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_append_jsonl", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_send_rrf_gcode", lambda *_args, **_kwargs: "ok")

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
        full_auto_runs=None,
        full_auto_log=None,
        patience=5,
        verbose=False,
        no_collect=True,
    )

    assert rc == 0
    out = capsys.readouterr().out
    assert "underconstrained recovery succeeded" in out
    assert "Replaying next sweep to try and beat it." in out
