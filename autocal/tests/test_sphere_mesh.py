"""Sphere-sector meshes: planning, records, grouping and fitting."""
import copy
import json

import numpy as np
import pytest

from autocal._autocal_common import _suggested_mesh_command
from autocal.anchor_uncertainty import sweep_jackknife_mm
from autocal.dataset_roles import measurement_groups, normalize_dataset_point_roles
from autocal.ellipse_fit import fit_ellipses
from autocal.equal_strain import force_planning_parameters
from autocal.mesh_planning import MIN_MESH_POINTS, mesh_key, plan_sphere_mesh
from autocal.pose_fit import dataset_line_lengths, dataset_stretch, fit_pose_least_squares, has_extra_sensor_data
from autocal.tests.test_disturbed_robustness import TRUTH, _load
from autocal.tools.disturb_dataset import _solve_position


def _plan(machine, dataset=None):
    dataset = dataset or _load(f"benchmark_923c2baa_{machine}")
    normalize_dataset_point_roles(dataset)
    anchors = np.asarray(TRUTH[machine], dtype=float)
    plan = plan_sphere_mesh(anchors, dataset, dataset, machine_type=machine, home_offsets_mm=np.zeros(len(anchors)),
                            low_anchor_z=None, sigma_mm=0.05, regularization=1e-6,
                            measure_extra_ranges=has_extra_sensor_data(dataset), observed_stretch=dataset_stretch(dataset))
    return dataset, anchors, plan


def _mesh_records(plan, anchors, mesh_id, noise=0.0, rng=None):
    """Row records as the collector writes them, measured at the true geometry."""
    home = np.linalg.norm(anchors, axis=1)
    held, (level_drive, step_drive), sensor = plan["held_anchor"], plan["drive_anchors"], plan["sensor_anchor"]
    rows = sorted({point["row"] for point in plan["points"]}, key=[p["row"] for p in plan["points"]].index)
    records = []
    for row in rows:
        planned_row = [point for point in plan["points"] if point["row"] == row]
        points = []
        for step, planned in enumerate(planned_row):
            lines = np.array([held, level_drive, step_drive])
            commanded = np.array([plan["held_length_mm"], *planned["drive_mm"]])
            position = _solve_position(anchors, lines, home[lines] + commanded, np.asarray(planned["position_mm"]))
            reading = lambda axis: float(np.linalg.norm(position - anchors[axis]) - home[axis]
                                         + (rng.normal(0, noise) if rng is not None else 0.0))
            point = {"l_drive": float(commanded[2]), "l_drive_mu": float(commanded[2]),
                     "l_sensor": reading(sensor), "sensor_force_n": float("nan"), "step_index": step}
            point["l_sensor_mu"] = point["l_sensor"]
            if plan["extra_anchors"]:
                point["extra_sensor_ranges"] = [{"anchor": axis, "force_n": plan["extra_sensor_force_n"],
                                                 "length_mm": reading(axis), "accepted": True}
                                                for axis in plan["extra_anchors"]]
                for extra in point["extra_sensor_ranges"]:
                    extra["length_mu_mm"] = extra["length_mm"]
            points.append(point)
        records.append({"id": f"{mesh_id}_r{row}", "fixed_anchors": [held, level_drive],
                        "fixed_lengths": [plan["held_length_mm"], planned_row[0]["drive_mm"][0]],
                        "drive_anchor": step_drive, "sensor_anchor": sensor, "data_points": points,
                        "metadata": {"sweep_method": "sphere-mesh-row", "mesh": {
                            "id": mesh_id, "row": row, "held_anchor": held, "held_length_mm": plan["held_length_mm"],
                            "drive_anchors": [level_drive, step_drive], "sensor_anchor": sensor,
                            "extra_anchors": plan["extra_anchors"]}}})
    return records


def test_measurement_groups_treat_a_mesh_as_one_unit():
    dataset = {"sweeps": [{}, {"metadata": {"mesh": {"id": "m"}}}, {}, {"metadata": {"mesh": {"id": "m"}}}]}
    assert measurement_groups(dataset) == [[0], [1, 3], [2]]


@pytest.mark.parametrize("machine", sorted(TRUTH))
def test_planned_mesh_stays_in_the_measured_workspace(machine):
    dataset, anchors, plan = _plan(machine)
    assert plan is not None and len(plan["points"]) >= MIN_MESH_POINTS
    json.dumps(plan)
    roles = [plan["held_anchor"], *plan["drive_anchors"], plan["sensor_anchor"], *plan["extra_anchors"]]
    assert len(set(roles)) == len(roles)
    assert plan["held_anchor"] == len(anchors) - 1 and plan["held_length_mm"] <= 0.0
    raw = dataset_line_lengths(dataset)["lengths_mm"]
    low, high = np.nanmin(raw, axis=0), np.nanmax(raw, axis=0)
    checked = [plan["held_anchor"], *plan["drive_anchors"], plan["sensor_anchor"], *plan["extra_anchors"]]
    for point in plan["points"]:
        predicted = np.asarray(point["predicted_mm"])
        assert np.all(predicted[checked] >= low[checked] - 1e-9) and np.all(predicted[checked] <= high[checked] + 1e-9)
        assert point["drive_mm"] == pytest.approx(predicted[plan["drive_anchors"]].tolist())
        assert point["sensor_force_n"] >= force_planning_parameters(dataset)["minimum_force_n"]
    # A sweep of sweeps: each row holds the level drive at one length and is visited in one run.
    rows = [point["row"] for point in plan["points"]]
    assert rows == sorted(rows) and len(set(rows)) >= 2
    for row in set(rows):
        levels = {point["drive_mm"][0] for point in plan["points"] if point["row"] == row}
        assert len(levels) == 1


def test_rejected_mesh_is_not_planned_again():
    dataset, anchors, plan = _plan("hangprinter_4")
    dataset["acquisition_rejections"] = [{"mesh": {key: plan[key] for key in
                                                   ("held_anchor", "held_length_mm", "drive_anchors", "sensor_anchor")}}]
    _, _, replanned = _plan("hangprinter_4", dataset)
    assert replanned is None or mesh_key(replanned) != mesh_key(plan)


def test_no_mesh_before_the_first_sweeps():
    dataset = _load("benchmark_923c2baa_hangprinter_4")
    dataset["sweeps"] = dataset["sweeps"][:2]
    assert _plan("hangprinter_4", dataset)[2] is None


@pytest.mark.parametrize("machine", ["hangprinter_4", "hangprinter_5"])
def test_point_and_ellipsoid_fits_recover_exact_mesh_geometry(machine):
    source, anchors, plan = _plan(machine)
    # A second, different mesh: the next best once the first counts as rejected.
    source["acquisition_rejections"] = [{"mesh": {key: plan[key] for key in
                                                  ("held_anchor", "held_length_mm", "drive_anchors", "sensor_anchor")}}]
    _, _, other = _plan(machine, source)
    dataset = {"machine_type": machine, "num_anchors": len(anchors), "dimensions": 3, "config": {},
               "sweeps": [*_mesh_records(plan, anchors, "mesh_001"), *_mesh_records(other, anchors, "mesh_002")]}
    start = anchors + np.random.default_rng(0).normal(0, 15, anchors.shape)
    point_fit = fit_pose_least_squares(dataset, start, fit_home_offsets=False, home_offset_prior_sigma_mm=5)
    assert np.max(np.linalg.norm(point_fit["anchors"] - anchors, axis=1)) < 0.05
    ellipsoids = fit_ellipses(dataset, point_fit["anchors"])
    assert ellipsoids["num_ellipsoids"] == 2 + len(plan["extra_anchors"]) + len(other["extra_anchors"])
    assert ellipsoids["num_ellipses"] == 0
    assert np.max(np.linalg.norm(ellipsoids["anchors"] - anchors, axis=1)) < 0.05


def test_jackknife_leaves_out_whole_meshes():
    _, anchors, plan = _plan("hangprinter_4")
    sweeps = _load("benchmark_923c2baa_hangprinter_4")["sweeps"][:3]
    dataset = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {},
               "sweeps": [*sweeps, *_mesh_records(plan, anchors, "mesh_004")]}
    assert len(measurement_groups(dataset)) == 4
    assert sweep_jackknife_mm(dataset, anchors, fit_home_offsets=False) is not None


def test_mesh_command_owns_geometry_and_forces(tmp_path):
    cmd = _suggested_mesh_command(tmp_path / "plan.json", machine_type="hangprinter_4", output_file=tmp_path / "out.json",
                                  completed_sweeps=4, extra_args=["--sensor-force-profile", "p.json", "--fixed-targets",
                                                                  "1,2", "--force-low", "0.1", "--hp-sim-reset"])
    assert cmd[cmd.index("--mesh-plan") + 1] == str(tmp_path / "plan.json")
    assert "--sensor-force-profile" not in cmd and "--fixed-targets" not in cmd and "--hp-sim-reset" not in cmd
    assert cmd[cmd.index("--force-low") + 1] == "0.1" and cmd[cmd.index("--completed-sweeps") + 1] == "4"


@pytest.mark.parametrize("machine", sorted(TRUTH))
def test_planned_forces_stay_below_the_mesh_ceiling(machine):
    from autocal.mesh_planning import MESH_MAX_SENSOR_FORCE_N
    dataset, _, plan = _plan(machine)
    assert plan["max_sensor_force_n"] == MESH_MAX_SENSOR_FORCE_N
    assert all(point["sensor_force_n"] <= MESH_MAX_SENSOR_FORCE_N for point in plan["points"])
    assert plan["probe_radius_mm"] > 0 and 1 <= plan["min_points"] <= len(plan["points"])


def test_merging_meshes_with_the_same_id_keeps_them_separate():
    from autocal._autocal_common import _merge_sweep_datasets
    _, anchors, plan = _plan("hangprinter_4")
    base = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {},
            "sweeps": _mesh_records(plan, anchors, "mesh_001")}
    new = dict(base, sweeps=_mesh_records(plan, anchors, "mesh_001"))
    merged = _merge_sweep_datasets(base, new)
    groups = measurement_groups(merged)
    rows = len({point["row"] for point in plan["points"]})
    assert len(groups) == 2 and [len(group) for group in groups] == [rows] * 2
    assert {merged["sweeps"][group[0]]["metadata"]["mesh"]["id"] for group in groups} == {"mesh_001", "mesh_001_2"}


def test_exploration_ignores_mesh_records():
    from autocal.workspace import exploration_grid
    dataset = _load("benchmark_923c2baa_hangprinter_4")
    _, anchors, plan = _plan("hangprinter_4")
    with_mesh = dict(dataset, sweeps=[*dataset["sweeps"], *_mesh_records(plan, anchors, "mesh_009")])
    assert exploration_grid(with_mesh, 41) == exploration_grid(dataset, 41)


@pytest.mark.heavy
def test_replay_does_not_duplicate_recorded_rejections(tmp_path):
    import subprocess, sys
    source = _load("mesh_benchmark_ac9000fe_hangprinter_4")
    assert len(source.get("acquisition_rejections", [])) >= 1
    path = tmp_path / "mesh.json"
    path.write_text(json.dumps(source))
    subprocess.run([sys.executable, "-m", "autocal.autocal", "--sim", "--machine-type", "hangprinter_4", "--dataset", str(path),
                    "--find-radii", "off", "--base-radii", "30", "--buildup-factor", "0", "--no-collect"],
                   check=True, capture_output=True, cwd=str(__import__("pathlib").Path(__file__).resolve().parents[2]))
    replayed = json.loads(path.with_suffix(".replay_tmp.json").read_text())
    assert not replayed.get("acquisition_rejections")
    assert len(measurement_groups(replayed)) == len(measurement_groups(source))
