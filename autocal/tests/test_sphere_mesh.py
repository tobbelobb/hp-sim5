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
    """Records as the collector writes them, from the true geometry."""
    home = np.linalg.norm(anchors, axis=1)
    held, (first, second), sensor = plan["held_anchor"], plan["drive_anchors"], plan["sensor_anchor"]
    records = []
    for index, planned in enumerate(plan["points"]):
        lines = np.array([held, first, second])
        commanded = np.array([plan["held_length_mm"], *planned["drive_mm"]])
        position = _solve_position(anchors, lines, home[lines] + commanded, np.asarray(planned["position_mm"]))
        reading = lambda axis: float(np.linalg.norm(position - anchors[axis]) - home[axis]
                                     + (rng.normal(0, noise) if rng is not None else 0.0))
        point = {"l_drive": float(commanded[2]), "l_drive_mu": float(commanded[2]),
                 "l_sensor": reading(sensor), "sensor_force_n": float("nan"), "step_index": index}
        point["l_sensor_mu"] = point["l_sensor"]
        if plan["extra_anchors"]:
            point["extra_sensor_ranges"] = [{"anchor": axis, "force_n": plan["extra_sensor_force_n"],
                                             "length_mm": reading(axis), "accepted": True}
                                            for axis in plan["extra_anchors"]]
            for extra in point["extra_sensor_ranges"]:
                extra["length_mu_mm"] = extra["length_mm"]
        records.append({"id": f"{mesh_id}_p{index:02d}", "fixed_anchors": [held, first],
                        "fixed_lengths": [plan["held_length_mm"], planned["drive_mm"][0]],
                        "drive_anchor": second, "sensor_anchor": sensor, "data_points": [point],
                        "metadata": {"sweep_method": "sphere-mesh", "mesh": {
                            "id": mesh_id, "held_anchor": held, "held_length_mm": plan["held_length_mm"],
                            "drive_anchors": [first, second], "sensor_anchor": sensor,
                            "extra_anchors": plan["extra_anchors"], "index": index, "count": len(plan["points"])}}})
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
    # The path starts at the planned point nearest home.
    positions = np.array([point["position_mm"] for point in plan["points"]])
    assert np.argmin(np.linalg.norm(positions, axis=1)) == 0


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
    _, anchors, plan = _plan(machine)
    dataset = {"machine_type": machine, "num_anchors": len(anchors), "dimensions": 3, "config": {},
               "sweeps": [*_mesh_records(plan, anchors, "mesh_001"), *_mesh_records(plan, anchors, "mesh_002")]}
    # A second held length gives the meshes depth; reuse the plan on a shorter held line.
    shorter = copy.deepcopy(plan)
    shorter["held_length_mm"] -= 150.0
    home = np.linalg.norm(anchors, axis=1)
    for point in shorter["points"]:
        position = np.asarray(point["position_mm"])
        towards = position - anchors[plan["held_anchor"]]
        position = position - 150.0 * towards / np.linalg.norm(towards)
        point["position_mm"] = position.tolist()
        point["drive_mm"] = [float(np.linalg.norm(position - anchors[axis]) - home[axis]) for axis in plan["drive_anchors"]]
    dataset["sweeps"][len(plan["points"]):] = _mesh_records(shorter, anchors, "mesh_002")
    start = anchors + np.random.default_rng(0).normal(0, 15, anchors.shape)
    point_fit = fit_pose_least_squares(dataset, start, fit_home_offsets=False, home_offset_prior_sigma_mm=5)
    assert np.max(np.linalg.norm(point_fit["anchors"] - anchors, axis=1)) < 0.05
    ellipsoids = fit_ellipses(dataset, point_fit["anchors"])
    assert ellipsoids["num_ellipsoids"] == 2 * (1 + len(plan["extra_anchors"])) and ellipsoids["num_ellipses"] == 0
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
