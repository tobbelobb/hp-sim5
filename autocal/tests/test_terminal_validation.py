import copy

import numpy as np
import pytest

import autocal.terminal_validation as terminal
from autocal.pose_fit import shift_dataset_home_offsets
from autocal.stage_artifacts import read_stage_artifact, write_stage_artifact
from autocal.tools.replay_stage import main as replay
from autocal.tests.test_predictive_validation import _dataset


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
