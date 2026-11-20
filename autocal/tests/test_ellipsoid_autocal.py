"""Ellipsoid autocal: sphere-mesh rows, planned heights, fits with home offsets, the replay loop."""
import json

import numpy as np
import pytest

from autocal.dataset_roles import measurement_groups
from autocal.ellipse_fit import fit_ellipses
from autocal.ellipsoid_autocal import fit, highest_held_mm, level_offsets_mm, mesh_rows, next_held_mm, run_ellipsoid_autocal, tag_mesh, visits
from autocal.tests.test_disturbed_robustness import TRUTH, _load
from autocal.theoretical_ellipse import compute_constraint_circle_3d


def test_mesh_rows_hold_each_free_line_once():
    assert mesh_rows("hangprinter_4", None) == ["[0,3] 1 2", "[1,3] 0 2", "[2,3] 0 1"]
    assert mesh_rows("hangprinter_4", -800.0, {0: -430.0, 1: -20.0, 2: 0.0}) == [
        "[0,3] 1 2 fixed=n-430.000,-800.000", "[1,3] 0 2 fixed=n-20.000,-800.000", "[2,3] 0 1 fixed=n+0.000,-800.000"]
    hp5 = mesh_rows("hangprinter_5", -150.0)
    assert len(hp5) == 4 and all(row.endswith("fixed=n+0.000,-150.000") for row in hp5)


def test_level_lines_shorten_half_way_to_their_shortest():
    anchors = np.asarray(TRUTH["hangprinter_4"], dtype=float)
    offsets = level_offsets_mm(anchors, 3, -850.0)
    radius = 1900.0 - 850.0
    hanging = np.array([0.0, 0.0, 1900.0 - radius])
    idle = np.linalg.norm(anchors[0] - hanging)
    shortest = np.linalg.norm(anchors[0] - anchors[3]) - radius
    assert offsets[0] == pytest.approx(-0.5 * (idle - shortest))
    assert set(offsets) == {0, 1, 2} and all(offset < 0 for offset in offsets.values())


def test_heights_go_low_middle_highest_then_split_the_largest_gap():
    anchors = np.asarray(TRUTH["hangprinter_4"], dtype=float)
    top = highest_held_mm(anchors, 3)
    # The effector stops a tenth of the anchor's home distance below it.
    assert top == pytest.approx(-0.9 * 1900.0)
    assert next_held_mm(0, None, 3, []) is None
    assert next_held_mm(1, anchors, 3, [0.0]) == pytest.approx(0.5 * top)
    assert next_held_mm(2, anchors, 3, [0.0, 0.5 * top]) == pytest.approx(top)
    assert next_held_mm(3, anchors, 3, [0.0, -900.0, -1700.0]) == pytest.approx(-450.0)
    assert next_held_mm(4, anchors, 3, [0.0, -900.0, -1700.0, -450.0]) == pytest.approx(-1300.0)


def _blind_mesh(anchors, offsets, held_length, level_length, mesh_id, visit=1):
    """Rows as the collector records them, measured exactly at the true geometry."""
    norms = np.linalg.norm(anchors, axis=1)
    held = len(anchors) - 1
    records = []
    for level in range(held):
        drive, sensor = [axis for axis in range(held) if axis != level]
        recorded = {held: held_length, level: level_length}
        circle = compute_constraint_circle_3d(anchors, [level, held], [norms[a] + offsets[a] + recorded[a]
                                                                      for a in (level, held)])
        center, radius, u, v = circle.center, circle.radius, circle.u, circle.v
        lowest = np.arctan2(-v[2], -u[2])
        points = []
        for angle in lowest + np.linspace(-0.6, 0.6, 10):
            position = center + radius * (np.cos(angle) * u + np.sin(angle) * v)
            reading = lambda axis: float(np.linalg.norm(position - anchors[axis]) - norms[axis] - offsets[axis])
            points.append({"l_drive": reading(drive), "l_drive_mu": reading(drive), "l_sensor": reading(sensor),
                           "l_sensor_mu": reading(sensor), "step_index": len(points)})
        records.append({"fixed_anchors": [level, held], "fixed_lengths": [level_length, held_length],
                        "drive_anchor": drive, "sensor_anchor": sensor, "data_points": points})
    tag_mesh(records, mesh_id, held, visit)
    return records


def test_ellipsoid_fit_recovers_anchors_and_home_offsets():
    anchors = np.asarray(TRUTH["hangprinter_4"], dtype=float)
    offsets = np.array([0.0, 0.0, 0.0, 0.4])
    sweeps = [record for index, (held, level) in enumerate([(0, 400), (-400, 200), (-200, 400), (-600, 0)])
              for record in _blind_mesh(anchors, offsets, held, level, f"mesh_{index}")]
    dataset = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {}, "sweeps": sweeps}
    start = anchors + np.random.default_rng(0).normal(0, 20, anchors.shape)
    fit = fit_ellipses(dataset, start, fit_home_offsets=True)
    assert fit["home_offsets_skipped"] is None and fit["num_ellipses"] == 0
    assert np.max(np.abs(fit["anchors"] - anchors)) < 0.05
    assert fit["home_offsets_mm"][3] == pytest.approx(0.4, abs=0.02)
    # Ignoring the offset biases the anchors by far more than the offset itself.
    assert np.max(np.abs(fit_ellipses(dataset, start)["anchors"] - anchors)) > 1.0


def test_backfilled_spheres_share_a_mesh_and_replay_by_visit():
    anchors = np.asarray(TRUTH["hangprinter_4"], dtype=float)
    offsets = np.array([0.0, 0.0, 0.0, 0.4])
    schedule = [(0, 400), (-200, 200), (-400, 0), (0, 200), (-200, 0)]
    sweeps = [record for visit, (held, level) in enumerate(schedule, start=1)
              for record in _blind_mesh(anchors, offsets, held, level, f"sphere_{held}", visit)]
    dataset = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {}, "sweeps": sweeps}
    assert len(measurement_groups(dataset)) == 3
    assert visits(dataset) == [list(range(3 * k, 3 * k + 3)) for k in range(len(schedule))]
    start = anchors + np.random.default_rng(1).normal(0, 20, anchors.shape)
    for model in ("ellipsoid", "point"):
        result = fit(dataset, [start], None, True, model)
        assert np.max(np.abs(result["anchors"] - anchors)) < 0.05
        assert result["home_offsets_mm"][3] == pytest.approx(0.4, abs=0.02)


@pytest.mark.heavy
def test_replay_fits_spheres_of_a_sweep_dataset(tmp_path):
    path = tmp_path / "sweeps.json"
    path.write_text(json.dumps(_load("benchmark_923c2baa_hangprinter_4")))
    rows = []
    accepted = run_ellipsoid_autocal(
        path, machine_type="hangprinter_4", collector_args=[], run_collection=None, log_line=lambda _: None,
        log_console=lambda _: None, append_log=rows.append, no_collect=True, low_anchor_z=None,
        use_noise_mean=True, stable_step_mm=0.5, stop_file=tmp_path / "stop")
    truth = np.asarray(TRUTH["hangprinter_4"], dtype=float)
    assert np.max(np.abs(accepted["plan"]["anchors"] - truth)) < 1.5
    assert [row["iteration"] for row in rows] == list(range(1, len(rows) + 1))
    assert all(row["selected_run"] == "ellipsoid" for row in rows)


def test_measurement_groups_treat_a_mesh_as_one_unit():
    dataset = {"sweeps": [{}, {"metadata": {"mesh": {"id": "m"}}}, {}, {"metadata": {"mesh": {"id": "m"}}}]}
    assert measurement_groups(dataset) == [[0], [1, 3], [2]]
