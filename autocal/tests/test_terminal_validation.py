import copy

import numpy as np
import pytest

import autocal.terminal_validation as terminal
from autocal.pose_fit import shift_dataset_home_offsets
from autocal.stage_artifacts import read_stage_artifact, write_stage_artifact
from autocal.tools.replay_stage import main as replay
from autocal.tests.test_predictive_validation import _dataset
import autocal.autocal as ac
from autocal.stopping import StopDecision
from autocal.tests.test_autocal_no_collect import _write_dataset, _fake_plan


def _case():
    data, anchors = _dataset()
    offsets = [0., 0., 1.5]
    data = shift_dataset_home_offsets(data, -np.asarray(offsets))
    data["sweeps"][0]["id"] = "terminal_001"
    candidates = []
    for iteration, geometry in [(1, anchors + [30., 20.]), (2, anchors)]:
        candidates.append({"iteration": iteration, "run_id": "default", "rank_score": float(iteration),
            "training_sweep_ids": ["training"] * iteration,
            "plan": {"anchors": geometry, "home_offsets_mm": offsets}, "summary_meta": {}})
    return data, candidates


def test_newest_fit_can_win_on_common_fresh_data_with_frozen_offsets():
    data, candidates = _case()
    before = copy.deepcopy(candidates)
    scored = terminal.evaluate_terminal_candidates(candidates, data)
    plan, meta, ranked = terminal.select_terminal_plan(scored)
    assert meta["iteration"] == 2
    assert meta["terminal_validation"]
    assert all(item[2]["prediction_sweeps"] == 1 for item in scored)
    assert len(ranked) == 2
    np.testing.assert_array_equal(plan["anchors"], candidates[1]["plan"]["anchors"])
    for original, current in zip(before, candidates):
        np.testing.assert_array_equal(original["plan"]["anchors"], current["plan"]["anchors"])
        assert original["plan"]["home_offsets_mm"] == current["plan"]["home_offsets_mm"]


def test_training_overlap_and_failed_validation_cannot_apply_a_model(monkeypatch):
    data, candidates = _case()
    candidates[1]["training_sweep_ids"].append("terminal_001")
    with pytest.raises(ValueError, match="overlap"):
        terminal.evaluate_terminal_candidates(candidates, data)
    candidates[1]["training_sweep_ids"].pop()
    monkeypatch.setattr(terminal, "score_future_sweeps", lambda *a, **kw: (float("inf"), 1))
    scored = terminal.evaluate_terminal_candidates(candidates, data)
    assert terminal.select_terminal_plan(scored) == (None, None, [])


def test_terminal_artifact_replays_without_fitting(tmp_path, monkeypatch):
    import autocal.tools.replay_stage as stages
    data, candidates = _case()
    def forbidden(*args, **kwargs):
        raise AssertionError("Held-out data must never be fitted")
    monkeypatch.setattr(stages, "fit_ellipse_dataset", forbidden)
    artifact = tmp_path / "terminal.json"
    write_stage_artifact(artifact, "terminal_validation", {
        "candidates": candidates, "validation_dataset": data, "validation_options": {},
    })
    output = tmp_path / "selected.json"
    assert replay(["terminal", str(artifact), "--output", str(output)]) == 0
    assert read_stage_artifact(output)["payload"]["selected"]["iteration"] == 2


@pytest.mark.parametrize("mode", ["success", "collection_failure", "interrupt", "no_collect", "budget", "stop_file", "replay"])
def test_terminal_collection_keeps_models_frozen_and_honors_stops(tmp_path, monkeypatch, mode):
    import subprocess
    path = tmp_path / "sweeps.json"
    _write_dataset(path, sweeps=5 if mode == "replay" else 3)
    fit_calls, collections, applied = [], [], []
    real_run = ac.subprocess.run
    def fit(data_path, **kwargs):
        fit_calls.append(data_path)
        plan = _fake_plan()
        plan["dataset"] = ac._load_json(data_path)
        plan["collect_command"] += ["--output-file", str(tmp_path / "planned.json")]
        return plan
    def collect(command, **kwargs):
        if command[0] != "node":
            return real_run(command, **kwargs)
        collections.append(command)
        if mode == "collection_failure":
            raise subprocess.CalledProcessError(1, command)
        if mode == "interrupt":
            raise KeyboardInterrupt
        _write_dataset(command[command.index("--output-file") + 1], sweeps=1)
    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fit)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *a, **kw: None)
    monkeypatch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *a: False)
    monkeypatch.setattr(ac.StoppingPolicy, "update", lambda *a, **kw: StopDecision(1, True, False, None, None, "accept"))
    monkeypatch.setattr(ac.subprocess, "run", collect)
    monkeypatch.setattr(ac.RRFFirmwareProvider, "send_gcode", lambda *a: applied.append(a) or "")
    monkeypatch.setattr(terminal, "score_future_sweeps", lambda *a, **kw: (.1, 1))
    if mode == "stop_file":
        ac._full_auto_stop_path(path).touch()
    argv = ["--sim", "--machine-type", "slideprinter", "--dataset", str(path),
            "--terminal-validation", "--max-steps", "1" if mode == "budget" else "3"]
    if mode == "no_collect":
        argv.append("--no-collect")
    argv.extend(["--collector-args", "--server", "http://127.0.0.1:12345", "--no-ws"])
    code = ac.main(argv)
    if mode in ("success", "collection_failure", "interrupt"):
        assert len(collections) == 1 and len(fit_calls) == 1
    else:
        assert not collections
    if mode in ("collection_failure", "interrupt"):
        assert not applied
        assert code == (130 if mode == "interrupt" else 2)
    if mode == "success":
        assert code == 0 and len(applied) == 1
        assert len(ac._load_json(path)["sweeps"]) == 3
        artifact = read_stage_artifact(path.with_suffix(".terminal_validation.stage.json"))
        assert artifact["payload"]["validation_dataset"]["sweeps"][0]["id"] == "terminal_001"
