import json

import numpy as np
import pytest

from autocal.active_learning import SweepConfig
from autocal.equal_strain import (
    GRAVITY_M_PER_S2,
    equal_strain_tensions,
    is_equal_strain,
    sensor_force_profiles,
    write_sensor_force_profile,
)

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])
STRANDS = np.array([2.0, 2.0, 2.0, 3.0])
EXTRA = np.array([528.3, 528.3, 528.3, 398.9])
WEIGHT = 0.081 * GRAVITY_M_PER_S2
CFG = SweepConfig((0, 3), (390.0, -100.0), 1, 2)
OPTIONS = dict(strand_tension_n=1.0, mover_weight_n=WEIGHT, strands=STRANDS, extra_line_length_mm=EXTRA)


@pytest.mark.parametrize("position", [[0.0, 0.0, 0.0], [300.0, -100.0, 250.0], [-400.0, 350.0, 600.0]])
def test_tensions_balance_the_mover_weight_at_the_requested_mean_strand_tension(position):
    lines = [0, 3, 1, 2]
    tensions = equal_strain_tensions(ANCHORS, np.array(position), lines, **OPTIONS)
    to_anchor = ANCHORS[lines] - np.array(position)
    distance = np.linalg.norm(to_anchor, axis=1)
    directions = to_anchor / distance[:, None]
    assert directions.T @ tensions == pytest.approx([0.0, 0.0, WEIGHT], abs=1e-9)
    pretension = np.linalg.svd(directions.T)[2][-1]
    weight = pretension * (distance + EXTRA[lines])
    assert np.sum(weight * tensions / STRANDS[lines]) / np.sum(weight) == pytest.approx(1.0)


def test_profiles_cover_both_sweep_directions_with_ascending_drive_lengths():
    forward, backward = sensor_force_profiles(ANCHORS, CFG, minimum_force_n=0.4, **OPTIONS)
    assert (forward["drive_anchor"], forward["sensor_anchor"]) == (1, 2)
    assert (backward["drive_anchor"], backward["sensor_anchor"]) == (2, 1)
    for profile in (forward, backward):
        assert np.all(np.diff(profile["drive_mm"]) > 0.0)
        assert min(profile["force_n"]) >= 0.4
        assert max(profile["force_n"]) < 5.0
    # The sweep is mirror symmetric, so both directions see the same force table.
    assert forward["force_n"] == pytest.approx(backward["force_n"], abs=1e-6)


def test_profile_file_needs_the_line_description_and_tuned_forces(tmp_path):
    path = tmp_path / "profile.json"
    assert not write_sensor_force_profile(path, {"config": {"force_tuning": {"force_mid_n": 0.4}}}, ANCHORS, [CFG])
    assert not path.exists()
    config = {"m666_before_data_collection": {"W": 0.081, "N": [2, 2, 2, 3], "Y": EXTRA.tolist()},
              "force_tuning": {"force_mid_n": 0.4}}
    assert write_sensor_force_profile(path, {"config": config}, ANCHORS, [CFG])
    profiles = json.loads(path.read_text())
    expected = sensor_force_profiles(ANCHORS, CFG, minimum_force_n=0.4, **OPTIONS)
    assert profiles[0]["force_n"] == pytest.approx(expected[0]["force_n"])


def test_dataset_is_equal_strain_only_when_every_sweep_used_a_profile():
    profiled = {"metadata": {"sensor_force_profile": True}}
    assert is_equal_strain({"sweeps": [profiled, profiled]})
    assert not is_equal_strain({"sweeps": [profiled, {"metadata": {"sensor_force_profile": False}}]})
    assert not is_equal_strain({"sweeps": [profiled, {}]})
    assert not is_equal_strain({"sweeps": []})


def test_historical_projected_profile_markers_do_not_describe_tensioned_measurements():
    sweep = {"metadata": {"sensor_force_profile": True},
             "data_points": [{"raw_angles_zero_tension_deg": [0.0]*4}]}
    assert not is_equal_strain({"num_anchors": 4, "sweeps": [sweep]})
