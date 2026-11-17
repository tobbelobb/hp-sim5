"""Regression tests for the experiment-only fifth-range integration.

Run with:
  PYTHONPATH=<worktree> .venv/bin/python -m pytest <session>/tests -q
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np
import pytest

SESSION = Path(__file__).resolve().parent.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/pose_fit.py").exists())
sys.path.insert(0, str(SESSION / "evidence"))
sys.path.insert(0, str(REPO))

import extra_range_fit as xr  # noqa: E402
import autocal.fit_stage as fit_stage  # noqa: E402
import autocal.pose_fit as pose_fit  # noqa: E402

WORKER_RUNS = Path(
    "/home/torbjorn/repos/hp-sim5/research/output/"
    "program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/"
    "session-2026-11-17-hp5-pulley-reduced-independently-compare-the-current-single-acti-f359cbd6/runs"
)
MANUAL = Path("/home/torbjorn/repos/hp-sim5/output/first-manual-autocal-of-hp5-pulley-reduced/sweeps.json")

FIT_OPTIONS = dict(
    solve_restarts=3, solve_iterations=20, solve_optimizer="lbfgsb+trf-fallback",
    pointwise_residual_mode="sampson", pointwise_filtering=False, pointwise_global_mad=False,
    sweep_wise_filtering=False, sweep_metric="median", use_noise_mean=True, sigma_source="auto",
    robust_debug=False, residuals_csv=None, generate_report=False, find_radii="off",
    find_buildup_factor="off", base_radii=None, buildup_factor=None, r0_bounds=None,
    b_bounds=None, r0_prior_sigma_mm=None, b_prior_sigma=None, spool_outer_iters=3,
    theta0_mode="zero", line_width=0.5, sigma_floor_mm=None, fd_eps_mm=1.0,
    regularization=1e-6, collector_args=(), low_anchor_z=None, fit_home_offsets=True,
    filter_schedule=None, objective_schedule=None, fit_structure=None,
)


def _inject_policy(dataset: dict, *, force_mode: str = "sensor", source: str | None = None) -> dict:
    data = copy.deepcopy(dataset)
    data["extra_sensor_policy"] = {"mode": xr.POLICY_MODE, "version": 1, "source": source}
    n = int(data["num_anchors"])
    for sweep in data["sweeps"]:
        used = set(int(a) for a in sweep["fixed_anchors"]) | {int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])}
        extra = [axis for axis in range(n) if axis not in used][0]
        sweep["metadata"]["extra_sensor_anchors"] = [extra]
        sweep["metadata"]["extra_sensor_policy"] = xr.POLICY_MODE
        for point in sweep["data_points"]:
            point["extra_sensor_anchor"] = extra
            point["extra_sensor_force_n"] = (
                float(point["sensor_force_n"]) if force_mode == "sensor" else 0.05159146026151931
            )
            point["extra_sensor_force_mode"] = xr.POLICY_MODE if force_mode == "sensor" else "passive"
    return data


def _fake_dataset(extra_values: np.ndarray, *, drive=None, sensor=None) -> dict:
    drive = np.asarray(drive if drive is not None else np.linspace(-150.0, 150.0, 10), dtype=float)
    sensor = np.asarray(sensor if sensor is not None else 80.0 + 0.2 * drive, dtype=float)
    extra_angles = np.asarray(extra_values, dtype=float) / 0.2617993877991494
    points = []
    for index, (drive_mm, sensor_mm, angle) in enumerate(zip(drive, sensor, extra_angles)):
        raw = [-1000.0, drive_mm / 0.2617993877991494, angle, sensor_mm / 0.2617993877991494, 0.0]
        points.append({
            "l_drive": float(drive_mm),
            "l_sensor": float(sensor_mm),
            "raw_angles_deg": raw,
            "mu": list(raw),
            "step_index": index,
            "sensor_force_n": 4.991419941498644,
        })
    return {
        "num_anchors": 5,
        "dimensions": 3,
        "machine_type": "hangprinter_5_pulley_reduced",
        "config": {
            "mm_per_degree": [0.2617993877991494, 0.2617993877991494, 0.2617993877991494,
                              0.2617993877991494, 0.1308996938995747],
            "m666_before_data_collection": {
                "S": 20000, "U": [2, 2, 2, 2, 4], "N": [2, 2, 2, 2, 4],
                "Y": [429.1, 429.1, 429.1, 429.1, 352.3], "W": 0.081,
                "R": [30, 30, 30, 30, 30],
            },
            "force_tuning": {"force_low_n": 0.05159146026151931},
        },
        "sweeps": [{
            "id": "sweep_001",
            "fixed_anchors": [0, 4],
            "fixed_lengths": [-320.0, 0.0],
            "drive_anchor": 1,
            "sensor_anchor": 3,
            "data_points": points,
            "metadata": {"extra_sensor_anchors": [2], "extra_sensor_policy": xr.POLICY_MODE},
        }],
        "extra_sensor_policy": {"mode": xr.POLICY_MODE, "version": 1},
    }


def test_force_role_uses_explicit_metadata_not_path():
    dataset = _fake_dataset(20.0 + 0.15 * np.linspace(-150.0, 150.0, 10))
    dataset = _inject_policy(dataset, force_mode="sensor", source="/tmp/prototype-control/collect.mjs")
    dataset["_extra_range_force_stretch"] = True
    stretch = xr.stretch(dataset)
    assert stretch is not None
    assert np.allclose(stretch["force_values"][:, 1], 4.991419941498644)
    passive = _inject_policy(dataset, force_mode="passive", source="/tmp/prototype/collect.mjs")
    passive["_extra_range_force_stretch"] = True
    stretch_passive = xr.stretch(passive)
    assert np.allclose(stretch_passive["force_values"][:, 1], 0.05159146026151931)


def test_home_offset_shift_includes_extra_angle():
    dataset = _fake_dataset(20.0 + 0.15 * np.linspace(-150.0, 150.0, 10))
    dataset = _inject_policy(dataset)
    before = [list(point["mu"]) for point in dataset["sweeps"][0]["data_points"]]
    offsets = np.array([0.0, 0.0, 2.6179938779914944, 0.0, 0.0])
    shifted = xr.shift(dataset, offsets)
    after = [point["mu"] for point in shifted["sweeps"][0]["data_points"]]
    assert np.allclose(after[0][2] - before[0][2], 10.0)
    for axis in (0, 1, 3, 4):
        assert after[0][axis] == before[0][axis]


def test_gate_synthetic_valid_frozen_insensitive_and_slack():
    x = np.linspace(-150.0, 150.0, 10)
    valid = 20.0 + 0.15 * x
    mask, evidence = xr.evaluate_validity(_fake_dataset(valid))
    assert evidence["accepted"] == 1, evidence
    mask, evidence = xr.evaluate_validity(_fake_dataset(np.full(10, 20.0)))
    assert evidence["rejected"] == 1 and mask["sweep_001"]["status"] == "rejected_frozen_or_slack"
    mask, evidence = xr.evaluate_validity(_fake_dataset(np.full(10, 20.0), sensor=np.linspace(80.0, 85.0, 10)))
    assert evidence["accepted"] == 1 and mask["sweep_001"]["status"] == "accepted_untestable"
    slack = valid.copy()
    slack[5] += 5.0
    mask, evidence = xr.evaluate_validity(_fake_dataset(slack))
    assert evidence["rejected"] == 1, evidence


@pytest.mark.skipif(not (WORKER_RUNS / "twosensor-1/sweeps.json").exists(), reason="saved worker data absent")
def test_gate_saved_frozen_and_taut_cases():
    taut = _inject_policy(json.loads((WORKER_RUNS / "twosensor-1/sweeps.json").read_text()))
    _, evidence = xr.evaluate_validity(taut)
    assert evidence["accepted"] == 4 and evidence["rejected"] == 0
    frozen = _inject_policy(json.loads((WORKER_RUNS / "ctl-1/sweeps.json").read_text()), force_mode="passive")
    _, evidence = xr.evaluate_validity(frozen)
    assert evidence["accepted"] == 0 and evidence["rejected"] == 4
    insensitive = _inject_policy(json.loads((WORKER_RUNS / "ctl-low1p0/sweeps.json").read_text()), force_mode="passive")
    _, evidence = xr.evaluate_validity(insensitive)
    assert evidence["accepted"] == 4 and evidence["rejected"] == 0


@pytest.mark.skipif(not MANUAL.exists(), reason="manual reduced dataset absent")
def test_patched_fitter_is_inert_on_baseline_data():
    production = fit_stage.fit_ellipse_dataset(MANUAL, **FIT_OPTIONS)
    xr.install()
    patched = fit_stage.fit_ellipse_dataset(MANUAL, **FIT_OPTIONS)
    assert np.allclose(np.asarray(patched["anchors"]), np.asarray(production["anchors"]))
    assert np.allclose(np.asarray(patched["home_offsets_mm"]), np.asarray(production["home_offsets_mm"]))
    assert patched["pose_fit"]["rms_mm"] == pytest.approx(production["pose_fit"]["rms_mm"])


@pytest.mark.skipif(not (WORKER_RUNS / "twosensor-1/sweeps.json").exists(), reason="saved worker data absent")
def test_saved_five_range_fit_improves_and_uses_two_torques(tmp_path):
    dataset = _inject_policy(json.loads((WORKER_RUNS / "twosensor-1/sweeps.json").read_text()))
    dataset["_extra_range_force_stretch"] = True
    path = tmp_path / "sweeps.json"
    path.write_text(json.dumps(dataset))
    xr.install()
    fit = xr.fit_ellipse_dataset(path, **FIT_OPTIONS)
    assert fit["pose_fit"]["stretch_corrected"] is True
    validity = json.loads(path.with_suffix(".extra_range_validity.json").read_text())
    assert validity["gate"]["accepted"] == 4
    truth = np.asarray([dataset["config"]["m669"][axis] for axis in "ABCDI"], dtype=float)
    diff = np.asarray(fit["anchors"]) - truth
    aligned = np.max(np.abs(diff - diff.mean(axis=0)))
    assert aligned < 4.0
    assert fit["pose_fit"]["rms_mm"] < 0.05
