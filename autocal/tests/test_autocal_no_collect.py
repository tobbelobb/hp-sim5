import copy
import subprocess

import numpy as np
import pytest

import autocal.autocal as ac
from autocal.sweep_types import DataPoint, MachineConfig, MachineType, Sweep
from autocal.stopping import StopDecision


def _worse_warm_duplicate(cold_plans: list) -> dict:
    """Mocked planner reply for the loop's history-seeded (warm-start) variant.

    The full-auto loop refits the current data once more, seeded from the best
    estimate so far. These tests pin the cold-run selection, so the warm variant
    answers with a strictly worse duplicate of the last cold plan.
    """
    assert cold_plans, "warm-start variant must follow a cold plan"
    worse = dict(cold_plans[-1])
    for key in ("cost", "cost_raw", "cost_noise_normalized"):
        worse[key] = float(worse[key]) * 10.0
    return worse


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


@pytest.mark.parametrize('transport_failure', [False, True])
def test_resolved_boundary_replans_without_refitting_but_transport_failure_aborts(tmp_path, monkeypatch, transport_failure):
    path = tmp_path / 'sweeps.json'
    _write_dataset(path, sweeps=3)
    fits, collects, replans, sent = [], [], [], []
    collectors = set()
    def fitted(data_path, **kwargs):
        data = ac._load_json(data_path)
        fits.append(len(data['sweeps']))
        plan = _fake_plan()
        plan.update(dataset=data, cfg_path=tmp_path/'cfg.txt', collect_command=['node', 'collector', '--output-file', str(kwargs['collector_output'])])
        return plan
    def replan(frozen, data_path, **kwargs):
        assert fits == [3]
        assert len(frozen['dataset']['sweeps']) == 3
        assert len(frozen['dataset']['acquisition_rejections']) == 1
        replans.append(frozen)
        return {**frozen, 'collect_command': ['node', 'collector', '--output-file', str(kwargs['collector_output'])]}
    def collect(self, command):
        collectors.add(self)
        collects.append(command)
        if transport_failure:
            raise subprocess.CalledProcessError(1, command)
        output = ac.Path(command[command.index('--output-file')+1])
        if len(collects) == 1:
            rejected = ac._load_json(path)
            rejected['sweeps'] = []
            rejected['acquisition_rejections'] = [{**_valid_sweep(1), 'reason': 'No coupled interval'}]
            ac._write_json(output, rejected)
        else:
            _write_dataset(output, sweeps=1)
    monkeypatch.setattr(ac, 'plan_next_ellipse_sweep', fitted)
    monkeypatch.setattr(ac, 'plan_ellipse_sweep', replan)
    monkeypatch.setattr(ac, '_print_ellipse_plan', lambda *a, **kw: None)
    monkeypatch.setattr(ac, '_plan_hits_underconstrained_penalty', lambda *a: False)
    monkeypatch.setattr(ac.StoppingPolicy, 'update', lambda *a, **kw: StopDecision(1, True, False, None, 'accept' if 4 in fits else 'collect'))
    monkeypatch.setattr(ac.BrowserCollector, 'collect', collect)
    monkeypatch.setattr(ac.RRFFirmwareProvider, 'send_gcode', lambda *a: sent.append(a) or '')
    code = ac.main(['--sim', '--machine-type', 'slideprinter', '--dataset', str(path), '--max-steps', '4',
                    '--collector-args', '--server', 'http://127.0.0.1:12345'])
    data = ac._load_json(path)
    if transport_failure:
        assert code == 2 and not sent and not replans
        assert len(data['sweeps']) == 3
    else:
        assert code == 0 and len(sent) == 1 and len(collects) == 2 and len(replans) == 1
        assert len(collectors) == 1
        assert fits[0] == 3 and all(count == 4 for count in fits[1:])
        assert len(data['sweeps']) == 4 and len(data['acquisition_rejections']) == 1
        assert path.with_name('sweeps.new_001.json').exists()


@pytest.mark.parametrize("mode", ["apply_interrupt", "stop_during_fit"])
def test_final_application_does_not_retry_and_preserves_stop_reason(tmp_path, monkeypatch, capsys, mode):
    dataset = tmp_path / "sweeps.json"
    _write_dataset(dataset, sweeps=3)
    sent = []

    def fit(path, **kwargs):
        if mode == "stop_during_fit":
            ac._full_auto_stop_path(dataset).touch()
        plan = _fake_plan()
        plan["dataset"] = ac._load_json(path)
        return plan

    def send(self, server, gcode):
        sent.append(gcode)
        if mode == "apply_interrupt":
            raise KeyboardInterrupt
        return ""

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fit)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *a, **kw: None)
    monkeypatch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *a: False)
    monkeypatch.setattr(ac.StoppingPolicy, "update", lambda *a, **kw: StopDecision(1, True, False, None, "accept"))
    monkeypatch.setattr(ac.RRFFirmwareProvider, "send_gcode", send)
    code = ac.main([
        "--sim", "--no-collect", "--machine-type", "slideprinter", "--dataset", str(dataset),
        "--max-steps", "3", "--collector-args", "--server", "http://127.0.0.1:12345",
    ])
    assert len(sent) == 1
    output = capsys.readouterr().out
    if mode == "apply_interrupt":
        assert code == 130
        assert "stopping without retry" in output
    else:
        assert code == 0
        assert "stop requested (stop-file)" in output


def test_full_auto_applies_the_newest_fit_with_its_own_radius(tmp_path, monkeypatch, capsys):

    from autocal.spool_model import build_spool_model_params, dataset_with_modeled_lengths

    anchors = np.array([[0.0, -400.0], [400.0, 0.0], [-300.0, 300.0]])
    phi = np.linspace(0.0, np.pi, 40)
    positions = anchors[0] + 600.0 * np.column_stack([np.cos(phi), np.sin(phi)])
    deltas = np.linalg.norm(positions[:, None, :] - anchors, axis=2) - np.linalg.norm(anchors, axis=1)
    deltas[:, 0] = 200.0
    sweep = {
        "fixed_anchors": [0], "fixed_lengths": [150.0], "drive_anchor": 1,
        "sensor_anchor": 2,
        "data_points": [
            {"l_drive": row[1] * 0.75, "l_sensor": row[2] * 0.75,
             "raw_angles_deg": (row * 180.0 / (np.pi * 40.0)).tolist()}
            for row in deltas
        ],
    }
    data = {"version": "1.0", "machine_type": "slideprinter", "num_anchors": 3, "dimensions": 2,
            "sweeps": [dict(copy.deepcopy(sweep), id=f"sweep_{i:03d}") for i in range(1, 6)]}
    path = tmp_path / "radii.json"
    ac._write_json(path, data)

    def fake_plan(path, **_kwargs):
        raw = ac._load_json(path)
        radius = 35.0 if len(raw["sweeps"]) == 3 else 40.0
        params = build_spool_model_params(
            raw, base_radii_mm=[30.0] * 3, modeled_radii_mm=[radius] * 3,
            modeled_buildup_factor=[0.0] * 3, spool_to_motor_gearing_factor=[1.0] * 3,
            mechanical_advantage=[1.0] * 3, lines_per_spool=[1.0] * 3,
        )
        plan = _fake_plan()
        plan.update(anchors=anchors, radius=radius, dataset=raw, spool_model_params=params,
                    dataset_for_estimation=dataset_with_modeled_lengths(raw, params))
        return plan

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *_args: False)
    monkeypatch.setattr(ac, "_m666_from_plan", lambda plan: f"M666 R{plan['radius']}")
    rc = ac.main([
        "--sim", "--machine-type", "slideprinter", "--dataset", str(path),
        "--no-collect", "--max-steps", "3",
    ])

    assert rc == 0
    output = capsys.readouterr().out
    assert "M666 R40.0" in output
    assert "M666 R35.0" not in output


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
        solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
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
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        regularization=0.0,
        min_fixed_delta_spacing_mm=0.0,
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


def test_full_auto_loop_closes_collection_and_firmware_on_bootstrap_interrupt(
    tmp_path,
    monkeypatch,
):
    dataset = tmp_path / "missing_dataset.json"

    fake_proc = object()
    stopped = []

    def fake_wait(*_args, **_kwargs):
        return None

    def interrupting_run(*_args, **_kwargs):
        raise KeyboardInterrupt

    def close_collector(self):
        if "collector" not in stopped:
            stopped.append("collector")

    monkeypatch.setattr(ac.RRFFirmwareProvider, "start_simulator", lambda *args, **kwargs: fake_proc)
    monkeypatch.setattr(ac.RRFFirmwareProvider, "wait_for_ready", fake_wait)
    monkeypatch.setattr(ac.BrowserCollector, "collect", interrupting_run)
    monkeypatch.setattr(ac.BrowserCollector, "close", close_collector)
    monkeypatch.setattr(ac, "_stop_process", lambda proc: stopped.append(proc))

    result = ac.full_auto_loop(
        work_dataset=dataset,
        machine_type="slideprinter",
        max_steps=8,
      solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
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
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        regularization=0.0,
        min_fixed_delta_spacing_mm=0.0,
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
        no_collect=False,
    )
    assert result == 130

    assert stopped == ["collector", fake_proc]


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


def test_full_auto_applies_newest_fit_after_patience(tmp_path, monkeypatch, capsys):
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

    cold_plans = []

    def fake_plan(*_args, **_kwargs):
        if _kwargs.get("initial_guess") is not None:
            return _worse_warm_duplicate(cold_plans)
        assert plan_queue, "expected one replay plan per iteration"
        plan = plan_queue.pop(0)
        cold_plans.append(plan)
        return plan

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "plan_anchor_uncertainty_mm", fake_cost)
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
        solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
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
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        regularization=0.0,
        min_fixed_delta_spacing_mm=0.0,
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
    assert "M669 iter_3" in out
    assert "Anchor uncertainty 1 mm, the lowest so far." in out
    assert "the 3rd lowest so far." in out
    assert "Replaying next sweep to try and beat it." in out
    assert "Still has patience for 2 more attempts." in out
    assert "Patience reset." in out


def test_full_auto_skips_a_newest_fit_whose_uncertainty_diverged(tmp_path, monkeypatch, capsys):
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
            "cost": 30.0,
            "cost_raw": 30.0,
            "cost_noise_normalized": 30.0,
            "covariance": np.eye(6, dtype=float),
            "covariance_scaled": np.eye(6, dtype=float),
            "collect_command": ["node", "collect"],
        },
    ]

    cold_plans = []

    def fake_plan(*_args, **_kwargs):
        if _kwargs.get("initial_guess") is not None:
            return _worse_warm_duplicate(cold_plans)
        assert plan_queue, "expected one replay plan per iteration"
        plan = plan_queue.pop(0)
        cold_plans.append(plan)
        return plan

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "plan_anchor_uncertainty_mm", fake_cost)
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
        solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
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
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        regularization=0.0,
        min_fixed_delta_spacing_mm=0.0,
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
    out = capsys.readouterr().out
    assert "newest fit diverged" in out
    assert "M669 iter_2" in out


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

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    def fake_send(_server: str, gcode: str) -> str:
        sent.append(gcode)
        return "ok"

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan_next)
    monkeypatch.setattr(ac, "plan_anchor_uncertainty_mm", fake_cost)
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
        solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
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
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        regularization=0.0,
        min_fixed_delta_spacing_mm=0.0,
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

    def fake_cost(plan):
        return float(plan["cost"])

    def fake_cov_summary(_plan):
        return 1.0, 1.0, True

    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "plan_anchor_uncertainty_mm", fake_cost)
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
        solve_restarts=1,
        solve_iterations=1,
        solve_optimizer="L-BFGS-B",
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
        theta0_mode="zero",
        line_width=0.4,
        sigma_floor_mm=None,
        candidate_count=16,
        delta_min=None,
        delta_max=None,
        regularization=0.0,
        min_fixed_delta_spacing_mm=0.0,
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
