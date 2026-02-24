import shlex

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
                "residual_vs_distance_slope": 0.01,
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
    assert "residual_vs_distance_slope=0.01" in out
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


def test_ellipse_loop_no_collect_accepts_without_collection(tmp_path, monkeypatch):
    dataset = tmp_path / "semi_dataset.json"
    _write_dataset(dataset, sweeps=3)

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

    monkeypatch.setattr(ac, "_plan_next_ellipse_sweep", lambda *_args, **_kwargs: _fake_plan())
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_send_rrf_gcode", fake_send)
    monkeypatch.setattr(ac, "_start_rrf_simulator", fail_start)
    monkeypatch.setattr(ac, "_wait_for_rrf_server", fail_wait)
    monkeypatch.setattr(ac.subprocess, "run", fail_run)

    rc = ac.ellipse_loop(
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
        plot_residual_histogram=False,
        sweep_points=None,
        output_with_explanations=False,
        no_collect=True,
    )

    assert rc == 0
    assert len(sent) == 0


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

    monkeypatch.setattr(ac, "_plan_next_ellipse_sweep", lambda *_args, **_kwargs: _fake_plan())
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
        full_auto_verbose=False,
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
        "autocal/active_calibrate.py",
        "--sim",
        "--full-auto",
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

    monkeypatch.setattr(ac, "_plan_next_ellipse_sweep", lambda *_args, **_kwargs: _fake_plan())
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
        full_auto_verbose=False,
        no_collect=True,
    )

    assert rc == 0
    assert len(sent) == 0
    log_text = (tmp_path / "full_dataset.full_auto.log").read_text(encoding="utf-8")
    first_two_lines = log_text.splitlines()[:2]
    assert f"; command: {shlex.join(fake_argv)}" in first_two_lines
