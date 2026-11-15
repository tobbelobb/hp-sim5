import numpy as np
import pytest
import json
import subprocess

import autocal.rough_bootstrap as bootstrap
import autocal.autocal as ac
from autocal.active_learning import SweepConfig
from autocal.pose_planning import predict_sweep_positions, SWEEP_PRETENSION_SHARE

ANCHORS = np.array([[0., -1900., -280.], [1645.4, 950., -280.],
                    [-1645.4, 950., -280.], [0., 0., 1900.]])


def _sweep(cfg):
    positions = predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE)
    lengths = np.linalg.norm(ANCHORS[None] - positions[:, None], axis=2) - np.linalg.norm(ANCHORS, axis=1)
    return {"fixed_anchors": list(cfg.fixed_anchors), "fixed_lengths": list(cfg.fixed_deltas_mm),
            "drive_anchor": cfg.drive_anchor, "sensor_anchor": cfg.sensor_anchor,
            "data_points": [{"l_drive": row[cfg.drive_anchor], "l_sensor": row[cfg.sensor_anchor]} for row in lengths]}


def _dataset():
    return {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3,
            "config": {"max_travel_mm": 390.0},
            "sweeps": [_sweep(SweepConfig((low, 3), (390., 0.), *[a for a in range(3) if a != low]))
                       for low in range(3)]}


def test_weak_bootstrap_is_rejected_despite_exact_residuals_and_improves_with_height(monkeypatch):
    monkeypatch.setattr(bootstrap, "build_anchor_initial_guess", lambda *a, **kw: ANCHORS * 1.05)
    dataset = _dataset()
    anchors, check = bootstrap.assess_rough_geometry(dataset)
    assert check["rms_mm"] < 1e-6
    assert not check["ready"]
    assert check["max_anchor_std_mm"] > bootstrap.MAX_ROUGH_ANCHOR_STD_MM
    cfg = bootstrap.next_rough_sweep(dataset, anchors)
    assert cfg.fixed_deltas_mm[cfg.fixed_anchors.index(3)] < 0.0
    dataset["sweeps"].append(_sweep(cfg))
    _, check = bootstrap.assess_rough_geometry(dataset)
    assert check["ready"]


def test_no_encoder_travel_cannot_silently_produce_a_force_profile():
    with pytest.raises(ValueError, match="raw encoder travel"):
        bootstrap.assess_rough_geometry(_dataset())


def test_repeating_one_sweep_does_not_hide_unobservable_geometry(monkeypatch):
    monkeypatch.setattr(bootstrap, "build_anchor_initial_guess", lambda *a, **kw: ANCHORS)
    dataset = _dataset()
    dataset["sweeps"] = [dataset["sweeps"][0]] * 6
    _, check = bootstrap.assess_rough_geometry(dataset)
    assert not check["ready"]


@pytest.mark.parametrize("failure", ["initial_collection", "refinement_collection", "rejected_geometry"])
def test_interrupted_or_rejected_bootstrap_cannot_bypass_checks_on_restart(tmp_path, monkeypatch, failure):
    path = tmp_path / "sweeps.json"
    dataset = _dataset()
    dataset["version"] = "2.0"
    for index, sweep in enumerate(dataset["sweeps"]):
        sweep["id"] = f"sweep_{index}"
    dataset["config"].update(m666_before_data_collection={"W": .081, "N": [2, 2, 2, 3], "Y": [500]*4},
                             force_tuning={"force_low_n": .02, "force_mid_n": .4, "force_max_n": 2.0})
    calls = []
    def collect(command, **_kwargs):
        calls.append(command)
        if failure == "initial_collection" or len(calls) > 1:
            raise subprocess.CalledProcessError(1, command)
        ac._write_json(command[command.index("--output-file") + 1], dataset)
    monkeypatch.setattr(ac.subprocess, "run", collect)
    monkeypatch.setattr(ac, "assess_rough_geometry", lambda *a, **kw: (ANCHORS, {"ready": failure != "rejected_geometry"}))
    monkeypatch.setattr(ac, "MAX_EXTRA_ROUGH_SWEEPS", 0)
    argv = ["--machine-type", "hangprinter_4", "--dataset", str(path)]
    assert ac.main(argv) == 2
    state = json.loads(path.with_suffix(".bootstrap_state.json").read_text())
    assert state["stage"] == "failed"
    count = len(calls)
    with pytest.raises(SystemExit) as error:
        ac.main(argv)
    assert error.value.code == 2
    assert len(calls) == count


@pytest.mark.parametrize("stage", ["collecting_rough", "checking_rough", "remeasuring", "failed"])
def test_pending_stage_is_refused_even_without_a_dataset(tmp_path, stage):
    path = tmp_path / "sweeps.json"
    bootstrap.write_bootstrap_state(path, stage)
    assert "Incomplete bootstrap" in bootstrap.bootstrap_state_error(path)


def test_complete_marker_requires_the_saved_dataset(tmp_path):
    path = tmp_path / "sweeps.json"
    path.with_suffix(".bootstrap_state.json").write_text('{"stage":"complete"}')
    assert "missing" in bootstrap.bootstrap_state_error(path)
    dataset = _dataset()
    dataset["version"] = "2.0"
    for index, sweep in enumerate(dataset["sweeps"]):
        sweep.update(id=f"sweep_{index}", metadata={"sensor_force_profile": True})
    path.write_text(json.dumps(dataset))
    bootstrap.write_bootstrap_state(path, "complete")
    assert bootstrap.bootstrap_state_error(path) is None
    # Additional adaptive sweeps preserve the authenticated bootstrap prefix.
    dataset["sweeps"].append({**dataset["sweeps"][0], "id": "extra"})
    path.write_text(json.dumps(dataset))
    assert bootstrap.bootstrap_state_error(path) is None
    dataset["sweeps"][0]["data_points"][0]["l_sensor"] += 1.
    path.write_text(json.dumps(dataset))
    assert "no longer matches" in bootstrap.bootstrap_state_error(path)


def test_corrupt_complete_dataset_is_rejected_before_headless_context_entry(tmp_path, monkeypatch):
    from autocal.headless_sim import HeadlessSimulation
    path = tmp_path / "sweeps.json"
    path.write_text("{")
    path.with_suffix(".bootstrap_state.json").write_text('{"stage":"complete"}')
    def forbidden(*args):
        raise AssertionError("Headless context must not start")
    monkeypatch.setattr(HeadlessSimulation, "__enter__", forbidden)
    with pytest.raises(SystemExit) as error:
        ac.main(["--headless-sim", "--machine-type", "hangprinter_3", "--dataset", str(path)])
    assert error.value.code == 2


@pytest.mark.parametrize('mixed', [False, True])
def test_legacy_profile_policy_is_refused_before_headless_context_entry(tmp_path, monkeypatch, mixed):
    from autocal.headless_sim import HeadlessSimulation
    path = tmp_path / "legacy.json"
    data = _dataset()
    data['version'] = '2.0'
    data['config']['m666_before_data_collection'] = {'U': [2,2,2,4]}
    for i, sweep in enumerate(data['sweeps']):
        sweep['id'] = f'sweep_{i:03d}'
        sweep['metadata'] = {'sensor_force_profile': True}
    if mixed:
        data['sweeps'][0]['metadata']['sensor_force_profile'] = False
    path.write_text(json.dumps(data))
    def forbidden(*args):
        raise AssertionError('Headless context must not start')
    monkeypatch.setattr(HeadlessSimulation, '__enter__', forbidden)
    with pytest.raises(SystemExit) as error:
        ac.main(['--headless-sim', '--machine-type', 'hangprinter_4', '--dataset', str(path)])
    assert error.value.code == 2
