"""Pinned analytical and mocked-loop checks. No services, physics, or hardware.

Run from root with PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python <file>.
"""
from __future__ import annotations

import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import types

import numpy as np
import pytest

import autocal.autocal as ac
import autocal.equal_strain as es
import autocal.history_selection as history
from autocal.active_learning import SweepConfig
from autocal.pose_planning import predict_sweep_positions
from autocal.workspace import exploration_grid


DIRECTORY = Path(__file__).resolve().parent
REVISION = "dfc0e3e3f47be4e7ae915211a42645d40e5e8746"
MANIFEST = json.loads((DIRECTORY / "source-manifest.json").read_text())
for filename, digest in MANIFEST["files"].items():
    assert hashlib.sha256(Path(filename).read_bytes()).hexdigest() == digest, filename


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(item) for item in value]
    return value


# Nominal geometry is an analytical fixture, never a scene read by a planner.
anchors = np.array([[0., -1900., -280.], [1645.4, 950., -280.],
                    [-1645.4, 950., -280.], [0., 0., 1900.]])
strands = np.array([2., 2., 2., 3.])
extra = np.array([528.3, 528.3, 528.3, 398.9])
advantage = np.array([2., 2., 2., 4.])
weight_n, minimum_n, target_n = .081 * 9.81, .23, .2875
options = dict(strand_tension_n=target_n, mover_weight_n=weight_n,
               strands=strands, extra_line_length_mm=extra,
               mechanical_advantage=advantage, minimum_force_n=minimum_n)
old = types.ModuleType("review_equal_strain_v1")
exec(compile(subprocess.check_output(["git", "show", "8338be8f:autocal/equal_strain.py"],
                                    text=True), "8338be8f:autocal/equal_strain.py", "exec"), old.__dict__)


def profile_case(fixed):
    cfg = SweepConfig((0, 3), tuple(fixed), 1, 2)
    positions = predict_sweep_positions(anchors, cfg, 60, .05)
    both = {}
    for label, module in (("v1", old), ("v2", es)):
        profiles = module.sensor_force_profiles(anchors, cfg, **options)
        direction_results = []
        for profile in profiles:
            drive, sensor = profile["drive_anchor"], profile["sensor_anchor"]
            predicted = np.linalg.norm(anchors[drive] - positions, axis=1) - np.linalg.norm(anchors[drive])
            rows = []
            for drive_mm, sensor_n in zip(profile["drive_mm"], profile["force_n"]):
                index = int(np.argmin(np.abs(predicted - drive_mm)))
                assert abs(predicted[index] - drive_mm) < 1e-8
                to_anchor = anchors - positions[index]
                length = np.linalg.norm(to_anchor, axis=1)
                directions = (to_anchor / length[:, None]).T
                carrying = np.linalg.lstsq(directions, [0., 0., weight_n], rcond=None)[0]
                null = np.linalg.svd(directions)[2][-1]
                actual_tensions = carrying + null * (sensor_n - carrying[sensor]) / null[sensor]
                q = length + extra / advantage
                achieved = np.sum(null * q * actual_tensions / (strands * advantage)) / np.sum(null * q)
                rows.append(dict(position_index=index, drive_mm=drive_mm, sensor_force_n=sensor_n,
                                 achieved_target_n=achieved, target_error_n=achieved-target_n,
                                 minimum_tension_n=np.min(actual_tensions)))
            direction_results.append(dict(drive=drive, sensor=sensor, row_count=len(rows),
                max_abs_target_error_n=max(abs(row["target_error_n"]) for row in rows),
                support_mm=[profile["drive_mm"][0], profile["drive_mm"][-1]], rows=rows))
        both[label] = direction_results
    assert max(result["max_abs_target_error_n"] for result in both["v2"]) < 1e-12
    assert all(row["minimum_tension_n"] > 0 for result in both["v2"] for row in result["rows"])
    assert all(row["sensor_force_n"] >= minimum_n for result in both["v2"] for row in result["rows"])
    if fixed[1] == -1170:
        assert max(result["max_abs_target_error_n"] for result in both["v1"]) > .2
        assert all(new["row_count"] < prior["row_count"] for new, prior in zip(both["v2"], both["v1"]))
    return dict(fixed_lengths_mm=fixed, predicted_pose_count=len(positions), comparisons=both)


profile_results = [profile_case([390., -1170.]), profile_case([390., -100.])]

# Mechanism test for an infeasible interior gap: only one contiguous feasible run
# survives. These artificial forces test selection, not physical feasibility.
gap_positions = np.array([[float(i), 0., 0.] for i in range(8)])
with pytest.MonkeyPatch.context() as patch:
    patch.setattr(es, "predict_sweep_positions", lambda *_: gap_positions)
    patch.setattr(es, "equal_strain_tensions", lambda _, position, lines, **kw:
                  np.full(len(lines), -.1 if int(position[0]) in (2, 6) else 1.))
    gap_profiles = es.sensor_force_profiles(anchors, SweepConfig((0, 3), (0., 0.), 1, 2), **options)
    expected_positions = gap_positions[3:6]
    for profile in gap_profiles:
        drive = profile["drive_anchor"]
        expected = np.sort(np.linalg.norm(anchors[drive]-expected_positions, axis=1) - np.linalg.norm(anchors[drive]))
        np.testing.assert_allclose(profile["drive_mm"], expected)
gap_result = dict(artificial_infeasible_indices=[2, 6], selected_pose_indices=[3, 4, 5],
                  rows_per_direction=[len(profile["drive_mm"]) for profile in gap_profiles])


def valid_sweep(index):
    return dict(id=f"sweep_{index:03d}", fixed_anchors=[0], fixed_lengths=[10.],
                drive_anchor=1, sensor_anchor=2, data_points=[dict(l_drive=1., l_sensor=2.)])


def base_data():
    return dict(version="2.0", machine_type="slideprinter", num_anchors=3, dimensions=2,
                config={"max_travel_mm": 100., "force_tuning": {"force_mid_n": .2}},
                sweeps=[valid_sweep(index) for index in range(1, 4)])


base = base_data()
boundary = dict(base, sweeps=[], config={"force_tuning": {"force_mid_n": 99.}},
                acquisition_rejections=[dict(fixed_anchors=[0], fixed_lengths=[1000.],
                    drive_anchor=1, sensor_anchor=2, reason="offline fixture")])
merged = ac._merge_sweep_datasets(copy.deepcopy(base), boundary)
assert merged["sweeps"] == base["sweeps"] and merged["config"] == base["config"]
assert exploration_grid(merged, 9) == exploration_grid(base, 9)
assert len(merged["acquisition_rejections"]) == 1
merge_result = dict(before_bounds=exploration_grid(base, 9)[1],
                    after_bounds=exploration_grid(merged, 9)[1],
                    retained_measurements=len(merged["sweeps"]),
                    retained_force_reference=merged["config"]["force_tuning"])


def loop_case(name, *, next_action="reject", initial_command=True, inject_output=None):
    work = DIRECTORY / "tmp" / name
    work.mkdir(parents=True, exist_ok=True)
    dataset_path = work / "dataset.json"
    local_base = base_data()
    if inject_output is not None:
        local_base = json.loads(Path(inject_output).read_text())
        local_base["sweeps"] = [dict(copy.deepcopy(local_base["sweeps"][0]), id=f"sweep_{index:03d}")
                                for index in range(1, 4)]
    local_machine = local_base["machine_type"]
    local_anchors = anchors if local_machine == "hangprinter_3" else np.array([[0., 0.], [1., 0.], [0., 1.]])
    write_json(dataset_path, local_base)
    fits, stopping_updates, replans, collections, applications = [], [], [], [], []
    output = io.StringIO()
    real_run = ac.subprocess.run
    real_stopping_update = ac.StoppingPolicy.update

    def fit(path, **kwargs):
        data = ac._load_json(path)
        fits.append(len(data["sweeps"]))
        return dict(anchors=local_anchors, machine_type=local_machine,
                    dataset=data, cost=1., cost_noise_normalized=1., covariance_scaled=np.eye(local_anchors.size),
                    cfg_path=work / "cfg.txt", collect_command=["node", "offline-collector", "--output-file",
                    str(kwargs["collector_output"])] if initial_command else None)

    def replan(frozen, path, **kwargs):
        replans.append(len(frozen["dataset"]["acquisition_rejections"]))
        assert fits == [3]
        command = None if next_action == "exhaust" else ["node", "offline-collector", "--output-file", str(kwargs["collector_output"])]
        return dict(frozen, collect_command=command)

    def collect(command, **kwargs):
        if command[0] != "node":
            return real_run(command, **kwargs)
        collections.append(list(command))
        if next_action == "fail" and len(collections) > 1:
            raise subprocess.CalledProcessError(1, command)
        if next_action == "fail-first":
            raise subprocess.CalledProcessError(1, command)
        path = Path(command[command.index("--output-file") + 1])
        if inject_output is not None:
            write_json(path, json.loads(Path(inject_output).read_text()))
            return
        if next_action == "empty":
            write_json(path, dict(base_data(), sweeps=[]))
            return
        rejection = dict(valid_sweep(1), fixed_lengths=[1000.+len(collections)], data_points=[], reason="offline resolved boundary")
        write_json(path, dict(base_data(), sweeps=[], acquisition_rejections=[rejection]))

    def stopping_update(self, history_items, **kwargs):
        stopping_updates.append(dict(iteration=kwargs["iteration"], history_size=len(history_items)))
        return real_stopping_update(self, history_items, **kwargs)

    def forbidden_service(*args, **kwargs):
        raise AssertionError("Offline review must never start or wait for services")

    with pytest.MonkeyPatch.context() as patch, contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
        patch.setattr(ac, "plan_next_ellipse_sweep", fit)
        patch.setattr(ac, "plan_ellipse_sweep", replan)
        patch.setattr(ac, "_print_ellipse_plan", lambda *args, **kwargs: None)
        patch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *args: False)
        patch.setattr(ac.StoppingPolicy, "update", stopping_update)
        patch.setattr(ac.subprocess, "run", collect)
        patch.setattr(ac, "_start_rrf_simulator", forbidden_service)
        patch.setattr(ac, "_wait_for_rrf_server", forbidden_service)
        patch.setattr(ac.extended_reference, "enabled", lambda: False)
        patch.setattr(history, "score_future_sweeps", lambda *args, **kwargs: (.1, 1))
        patch.setattr(ac.RRFFirmwareProvider, "send_gcode", lambda *args: applications.append(args[2]) or "")
        code = ac.main(["--sim", "--machine-type", local_machine, "--dataset", str(dataset_path),
                        "--max-steps", "4", "--patience", "1", "--collector-args", "--server", "http://127.0.0.1:12345"])
    write_json(work / "trace.json", dict(code=code, fit_sweep_counts=fits, stopping_updates=stopping_updates,
               replans=replans, collections=collections, applications=applications))
    (work / "console.txt").write_text(output.getvalue())
    return dict(code=code, fit_sweep_counts=fits, stopping_update_count=len(stopping_updates),
                replan_count=len(replans), collection_count=len(collections), application_count=len(applications),
                max_steps_stop="stop requested (max-steps)" in output.getvalue(),
                workspace_stop="stop requested (workspace-exhausted)" in output.getvalue(),
                artifact_directory=str(work.relative_to(DIRECTORY)))


repeated = loop_case("repeated-rejections")
assert repeated["code"] == 0 and repeated["fit_sweep_counts"] == [3]
assert repeated["stopping_update_count"] == 1 and repeated["collection_count"] == 3
assert repeated["replan_count"] == 2 and repeated["application_count"] == 1
assert repeated["max_steps_stop"] and not repeated["workspace_stop"]
failed = loop_case("rejection-then-transport-failure", next_action="fail")
assert failed["code"] == 2 and failed["application_count"] == 0 and failed["stopping_update_count"] == 1
failed_first = loop_case("immediate-transport-failure", next_action="fail-first")
assert failed_first["code"] == 2 and failed_first["application_count"] == 0
empty = loop_case("empty-collector", next_action="empty")
assert empty["code"] == 2 and empty["application_count"] == 0
exhausted = loop_case("resolved-workspace-exhaustion", next_action="exhaust")
assert exhausted["code"] == 0 and exhausted["stopping_update_count"] == 1 and exhausted["workspace_stop"]

# Chain the successful collector result despite a final movement exception into
# the unchanged production parent loop (fitting and firmware sends mocked).
# This only establishes that the parent's exit-status contract cannot detect it.
written_after_failure = DIRECTORY / "tmp" / "final-return-failure.json"
propagated_cleanup_failure = loop_case("swallowed-return-reaches-parent", inject_output=written_after_failure)
assert propagated_cleanup_failure["application_count"] == 1

result = dict(revision=REVISION, evidence_kind="analytical fixtures and mocked control flow",
    inputs=dict(anchors_mm=anchors, strands=strands, extra_line_length_mm=extra,
                mechanical_advantage=advantage, mover_weight_n=weight_n,
                minimum_force_n=minimum_n, target_strand_tension_n=target_n),
    force_profile_comparisons=profile_results, artificial_gap=gap_result, rejection_merge=merge_result,
    loops=dict(repeated_rejections=repeated, rejection_then_failure=failed, immediate_failure=failed_first,
               empty_collection=empty, resolved_exhaustion=exhausted,
               swallowed_cleanup_failure=propagated_cleanup_failure),
    limits="No simulation/hardware accuracy or actual motion/force claim. Model rows are static ideal equilibrium; loop fit/application are stubs.")
write_json(DIRECTORY / "offline-review.json", jsonable(result))
print(json.dumps(dict(assertion_status="passed", loops=result["loops"],
                     output=str(DIRECTORY / "offline-review.json"))))
