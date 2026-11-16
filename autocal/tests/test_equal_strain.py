import json

import numpy as np
import pytest

from autocal.active_learning import SweepConfig
from autocal.pose_planning import predict_sweep_positions
from autocal.equal_strain import (
    GRAVITY_M_PER_S2,
    equal_strain_tensions,
    is_equal_strain,
    sensor_force_profiles,
    write_sensor_force_profile,
    profile_policy_error,
    PROFILE_STRAIN_MODEL,
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


def test_redundant_compliant_lines_contribute_to_the_load():
    anchors = np.array([[0., -1900., -280.], [1900., 0., -280.], [0., 1900., -280.],
                        [-1900., 0., -280.], [0., 0., 1900.]])
    lines, passive = [0, 4, 1, 3], 2
    position = np.array([100., -400., 200.])
    tensions = equal_strain_tensions(anchors, position, lines, strand_tension_n=1.,
        mover_weight_n=WEIGHT, strands=np.array([2,2,2,2,4]),
        extra_line_length_mm=np.array([429.1]*4 + [352.3]),
        mechanical_advantage=np.array([2,2,2,2,4]), compliant_force_n=.05)
    vectors = anchors - position
    directions = vectors / np.linalg.norm(vectors, axis=1)[:, None]
    assert directions[lines].T @ tensions + .05*directions[passive] == pytest.approx([0.,0.,WEIGHT], abs=1e-9)
    with pytest.raises(ValueError, match='d\\+1 active lines'):
        equal_strain_tensions(anchors, position, range(5), strand_tension_n=1.,
            mover_weight_n=WEIGHT, strands=np.ones(5), extra_line_length_mm=np.zeros(5))


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
    config = {"m666_before_data_collection": {"W": 0.081, "N": [2, 2, 2, 3], "Y": EXTRA.tolist(), "U": [1]*4},
              "force_tuning": {"force_mid_n": 0.4}}
    assert write_sensor_force_profile(path, {"config": config}, ANCHORS, [CFG])
    profiles = json.loads(path.read_text())
    expected = sensor_force_profiles(ANCHORS, CFG, minimum_force_n=0.4, **OPTIONS)
    assert profiles[0]["force_n"] == pytest.approx(expected[0]["force_n"])


def test_tabulated_sensor_forces_preserve_the_firmware_strain_invariant():
    advantage = np.array([2., 2., 2., 4.])
    profiles = sensor_force_profiles(ANCHORS, CFG, minimum_force_n=.4,
                                      mechanical_advantage=advantage, **OPTIONS)
    positions = predict_sweep_positions(ANCHORS, CFG, 60, .05)
    for profile in profiles:
        drive, sensor = profile['drive_anchor'], profile['sensor_anchor']
        predicted_drive = np.linalg.norm(ANCHORS[drive] - positions, axis=1) - np.linalg.norm(ANCHORS[drive])
        assert len(profile['drive_mm']) < len(positions)  # infeasible tails are removed, never clamped
        for drive_mm, force in zip(profile['drive_mm'], profile['force_n']):
            index = np.argmin(np.abs(predicted_drive - drive_mm))
            assert predicted_drive[index] == pytest.approx(drive_mm)
            vectors = ANCHORS - positions[index]
            lengths = np.linalg.norm(vectors, axis=1)
            directions = (vectors / lengths[:, None]).T
            carrying = np.linalg.lstsq(directions, [0., 0., WEIGHT], rcond=None)[0]
            null = np.linalg.svd(directions)[2][-1]
            tensions = carrying + null * (force - carrying[sensor]) / null[sensor]
            flex_times_ea = tensions * (advantage * lengths + EXTRA) / (advantage**2 * STRANDS)
            assert null @ (flex_times_ea - (lengths + EXTRA / advantage)) == pytest.approx(0., abs=1e-8)


def test_hp4_invariant_matches_the_firmware_flex_equation_with_unequal_reeving():
    position = np.array([250., -150., 800.])
    advantage = np.array([2., 2., 2., 4.])
    lengths = np.linalg.norm(ANCHORS - position, axis=1)
    directions = (ANCHORS - position) / lengths[:, None]
    null = np.linalg.svd(directions.T)[2][-1]
    def projected_flex_error(tensions):
        # SpringK=EA/(U*distance+Y), flex=F/(SpringK*U²*N).
        flex_times_ea = tensions * (advantage*lengths + EXTRA) / (advantage**2 * STRANDS)
        harmless_scale_plus_reference = lengths + EXTRA / advantage
        return null @ (flex_times_ea - harmless_scale_plus_reference)
    corrected = equal_strain_tensions(ANCHORS, position, range(4), mechanical_advantage=advantage, **OPTIONS)
    old = equal_strain_tensions(ANCHORS, position, range(4), **OPTIONS)
    assert directions.T @ corrected == pytest.approx([0., 0., WEIGHT], abs=1e-9)
    assert projected_flex_error(corrected) == pytest.approx(0., abs=1e-9)
    assert abs(projected_flex_error(old)) > 1.


def test_profile_file_refuses_unknown_mechanical_advantage(tmp_path):
    config = {"m666_before_data_collection": {"W": .081, "N": [2,2,2,3], "Y": EXTRA.tolist()},
              "force_tuning": {"force_mid_n": .4}}
    assert not write_sensor_force_profile(tmp_path / 'unknown.json', {"config": config}, ANCHORS, [CFG])


def test_corrected_reeving_profiles_cannot_continue_legacy_data():
    sweep = {"metadata": {"sensor_force_profile": True}}
    dataset = {"config": {"m666_before_data_collection": {"U": [2,2,2,4]}}, "sweeps": [sweep]}
    assert "fresh dataset" in profile_policy_error(dataset)
    sweep["metadata"]["sensor_force_profile_model"] = PROFILE_STRAIN_MODEL
    assert profile_policy_error(dataset) is None
    sweep["metadata"].pop("sensor_force_profile_model")
    dataset["config"]["m666_before_data_collection"]["U"] = [1]*4
    assert "fresh dataset" in profile_policy_error(dataset)


def test_force_retuning_does_not_change_the_existing_datasets_strain_target(tmp_path):
    config = {"m666_before_data_collection": {"W": .081, "N": [2,2,2,3], "Y": EXTRA.tolist(), "U": [1]*4},
              "force_tuning": {"force_mid_n": .2}}
    dataset = {"config": config, "sweeps": [{"metadata": {"strand_tension_n": 1.}}]}
    path = tmp_path / 'preserved.json'
    assert write_sensor_force_profile(path, dataset, ANCHORS, [CFG])
    assert all(profile['strand_tension_n'] == 1. for profile in json.loads(path.read_text()))


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
