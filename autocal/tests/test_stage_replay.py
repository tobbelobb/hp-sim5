"""Stage contracts: frozen models, exact reuse, and failed-prediction vetoes."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

import autocal.active_learning as al
import autocal.fit_stage as fs
from autocal.history_selection import rank_history_candidates
from autocal.planning_pass import plan_ellipse_sweep
from autocal.spool_model import build_spool_model_params, sweep_configs_with_modeled_lengths
from autocal.stage_artifacts import read_stage_artifact, write_stage_artifact
from autocal.tools.replay_stage import main


def _frozen_fit(spool):
    anchors = np.array([[0.0, -1900.0], [1645.448, 950.0], [-1645.448, 950.0]])
    configs = al.generate_candidate_sweeps(
        num_anchors=3, dimensions=2, fixed_delta_values_mm=[0.0], machine_type="slideprinter")
    dataset = {"version": "2.0", "machine_type": "slideprinter", "num_anchors": 3,
               "dimensions": 2, "sweeps": [
                   {"id": str(i), "fixed_anchors": list(cfg.fixed_anchors),
                    "fixed_lengths": list(cfg.fixed_deltas_mm), "drive_anchor": cfg.drive_anchor,
                    "sensor_anchor": cfg.sensor_anchor, "data_points": []}
                   for i, cfg in enumerate(configs)]}
    params = build_spool_model_params(
        dataset, base_radii_mm=[30.0]*3, modeled_radii_mm=[39.184]*3,
        modeled_buildup_factor=[0.636619]*3, spool_to_motor_gearing_factor=[1.0]*3,
        mechanical_advantage=[1.0]*3, lines_per_spool=[1.0]*3,
    ) if spool else None
    observed = sweep_configs_with_modeled_lengths(configs, params) if spool else configs
    info = al.total_information_matrix(
        anchors, observed, machine_type="slideprinter", num_anchors=3, dimensions=2,
        l2_scale=al.l2_scale_for_machine("slideprinter", 3, 2), fd_eps_mm=1.0)
    return {"dataset": dataset, "anchors": anchors, "machine_type": "slideprinter",
            "num_anchors": 3, "dimensions": 2, "spool_model_params": params,
            "sweep_configs_for_info": observed, "info": info,
            "information_settings": {"fd_eps_mm": 1.0, "regularization": 1e-6},
            "collection_settings": {"find_radii_mode": "global" if spool else "off",
                                    "find_buildup_mode": "off", "base_radii": [30.0]*3,
                                    "buildup_factor": 0.636619}}


@pytest.mark.parametrize("spool", [False, True])
def test_frozen_plan_roundtrip_skips_fitting_and_reuses_exact_information(tmp_path, monkeypatch, spool):
    fit = _frozen_fit(spool)
    before = copy.deepcopy(fit)
    options = dict(candidate_deltas=[100.0, 200.0], top_k=4)
    first = plan_ellipse_sweep(fit, tmp_path / "sweeps.json", **options)
    artifact_path = tmp_path / "fit.json"
    write_stage_artifact(artifact_path, "fit", {"fit": fit, "planning_options": options,
                                               "dataset_path": tmp_path / "sweeps.json"})
    restored = read_stage_artifact(artifact_path, expected_kind="fit")["payload"]

    def forbidden(*_args, **_kwargs):
        raise AssertionError("planning must skip fitting and observed-information recomputation")

    monkeypatch.setattr(fs, "calibrate_elliptical", forbidden)
    monkeypatch.setattr(al, "total_information_matrix", forbidden)
    assert main(["plan", str(artifact_path), "--output", str(tmp_path / "plan.json")]) == 0
    replay = read_stage_artifact(tmp_path / "plan.json", expected_kind="plan")["payload"]["plan"]
    assert replay["ranked"] == first["ranked"]
    assert replay["best_cfg"] == first["best_cfg"]
    assert (tmp_path / "plan.cfg.txt").exists()
    np.testing.assert_array_equal(restored["fit"]["anchors"], before["anchors"])
    np.testing.assert_array_equal(fit["info"], before["info"])
    assert fit["dataset"] == before["dataset"]


@pytest.mark.parametrize("regularization", [0.0, 1e-6])
def test_observed_information_reuse_matches_recompute_and_does_not_mutate(regularization):
    fit = _frozen_fit(True)
    candidates = al.generate_candidate_sweeps(num_anchors=3, dimensions=2,
                                              fixed_delta_values_mm=[100.0, 200.0])
    kwargs = dict(machine_type="slideprinter", num_anchors=3, dimensions=2,
                  l2_scale=al.l2_scale_for_machine("slideprinter", 3, 2),
                  observed=fit["sweep_configs_for_info"], candidates=candidates,
                  regularization=regularization)
    before = fit["info"].copy()
    direct = al.rank_candidates_d_optimal(fit["anchors"], **kwargs)
    cached = al.rank_candidates_d_optimal(fit["anchors"], observed_information=fit["info"], **kwargs)
    assert direct == cached
    np.testing.assert_array_equal(before, fit["info"])
    with pytest.raises(ValueError, match="parameter count"):
        al.rank_candidates_d_optimal(fit["anchors"], observed_information=np.eye(2), **kwargs)


def test_planning_recomputes_information_when_difference_step_changes(tmp_path, monkeypatch):
    fit = _frozen_fit(False)
    calls = []
    original = al.total_information_matrix

    def count(*args, **kwargs):
        calls.append(kwargs["fd_eps_mm"])
        return original(*args, **kwargs)

    monkeypatch.setattr(al, "total_information_matrix", count)
    plan_ellipse_sweep(fit, tmp_path / "sweeps.json", candidate_deltas=[100.0], fd_eps_mm=2.0)
    assert calls == [2.0]


@pytest.mark.parametrize("buildup,enabled", [([0.0]*3, False), ([0.0, 0.6, 0.0], True)])
def test_planning_enables_layering_from_fitted_buildup(tmp_path, buildup, enabled):
    fit = _frozen_fit(False)
    fit["collection_settings"]["buildup_factor"] = None
    fit["length_model"] = {"modeled_buildup_factor": buildup}
    plan = plan_ellipse_sweep(fit, tmp_path / "sweeps.json", candidate_deltas=[100.0], collector_args=["--sim"])
    assert ("--line-layering" in plan["collect_command"]) is enabled


def test_history_ranking_preserves_veto_and_unvalidated_fallback(tmp_path):
    def item(iteration, prediction, key):
        return ((key,), {"iteration": iteration}, {"prediction_score": prediction})

    unvalidated = item(3, None, 0.0)
    failed = item(1, float("inf"), 1.0)
    valid = item(2, 2.0, 2.0)
    assert rank_history_candidates([unvalidated, failed, valid]) == [valid]
    assert rank_history_candidates([unvalidated, failed]) == []
    assert rank_history_candidates([unvalidated]) == [unvalidated]
    path = tmp_path / "history.json"
    write_stage_artifact(path, "history", {"evaluated": [unvalidated, failed, valid]})
    main(["rank", str(path), "--output", str(tmp_path / "rank.json")])
    payload = read_stage_artifact(tmp_path / "rank.json")["payload"]
    assert payload["chosen_iteration"] == 2
    assert payload["failed_predictions"] is False
    assert rank_history_candidates([]) == []


def test_artifacts_reject_wrong_stage_and_version(tmp_path):
    path = tmp_path / "data.json"
    write_stage_artifact(path, "history", {})
    with pytest.raises(ValueError, match="Expected fit"):
        read_stage_artifact(path, expected_kind="fit")
    content = json.loads(path.read_text())
    content["format_version"] = 999
    path.write_text(json.dumps(content))
    with pytest.raises(ValueError, match="version"):
        read_stage_artifact(path)
