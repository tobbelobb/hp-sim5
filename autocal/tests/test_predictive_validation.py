import copy

import numpy as np
import pytest

from autocal.predictive_validation import score_future_sweeps
from autocal.spool_model import WinchSpoolModel, build_spool_model_params


def _dataset():
    anchors = np.array([[-400.0, 0.0], [400.0, 0.0], [0.0, 500.0]])
    phi = np.linspace(0.0, np.pi, 40)
    positions = anchors[0] + 600.0 * np.column_stack([np.cos(phi), np.sin(phi)])
    points = [
        {
            "l_drive": float(d - np.linalg.norm(anchors[1])),
            "l_sensor": float(s - np.linalg.norm(anchors[2])),
        }
        for d, s in zip(
            np.linalg.norm(positions - anchors[1], axis=1),
            np.linalg.norm(positions - anchors[2], axis=1),
        )
    ]
    sweep = {
        "id": "heldout",
        "fixed_anchors": [0],
        "fixed_lengths": [200.0],
        "drive_anchor": 1,
        "sensor_anchor": 2,
        "data_points": points,
    }
    rng = np.random.default_rng(12)
    for point in points:
        point["l_drive"] += float(rng.normal(0.0, 0.2))
        point["l_sensor"] += float(rng.normal(0.0, 0.2))
    points[9]["l_sensor"] += 20.0
    return {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [sweep],
    }, anchors


def test_score_future_sweeps_prefers_geometry_that_predicts_sensor_lengths():
    dataset, anchors = _dataset()

    exact = score_future_sweeps(dataset, anchors, training_sweep_ids=[])
    wrong = score_future_sweeps(
        dataset,
        anchors + np.array([[0.0, 0.0], [35.0, -20.0], [-15.0, 25.0]]),
        training_sweep_ids=[],
    )

    assert exact is not None and exact[1] == 1
    assert wrong is not None and wrong[1] == 1
    assert exact[0] < wrong[0]


def test_score_future_sweeps_requires_a_whole_sweep_outside_training():
    dataset, anchors = _dataset()

    assert score_future_sweeps(dataset, anchors, training_sweep_ids=["heldout"]) is None


@pytest.mark.parametrize("buildup", [0.0, 0.636619])
def test_validation_scores_the_candidates_own_radius_and_winding(buildup):
    dataset, anchors = _dataset()
    spool = WinchSpoolModel.from_firmware(
        base_radius=40.0, buildup_factor=buildup,
        spool_to_motor_gearing_factor=1.0, mechanical_advantage=1.0,
        lines_per_spool=1.0,
    )
    for point in dataset["sweeps"][0]["data_points"]:
        point["raw_angles_deg"] = [
            spool.linepos_mm_to_theta_deg(delta)
            for delta in (200.0, point["l_drive"], point["l_sensor"])
        ]
    before = copy.deepcopy(dataset)

    def score(radius, winding):
        params = build_spool_model_params(
            dataset, base_radii_mm=[30.0] * 3,
            modeled_radii_mm=[radius] * 3, modeled_buildup_factor=[winding] * 3,
            spool_to_motor_gearing_factor=[1.0] * 3,
            mechanical_advantage=[1.0] * 3, lines_per_spool=[1.0] * 3,
        )
        return score_future_sweeps(dataset, anchors, [], spool_params=params)

    exact = score(40.0, buildup)
    wrong_radius = score(35.0, buildup)
    wrong_winding = score(40.0, buildup + 2.0)
    assert exact[0] < wrong_radius[0]
    assert exact[0] < wrong_winding[0]
    assert dataset == before


def test_validation_rejects_finite_underconstrained_sentinel():
    dataset, _ = _dataset()
    dataset.update(machine_type="hangprinter_4", num_anchors=4, dimensions=3)
    dataset["sweeps"][0].update(fixed_anchors=[0, 3], fixed_lengths=[200.0, 200.0])
    score = score_future_sweeps(dataset, np.zeros((4, 3)), [])
    assert score == (float("inf"), 1)


def test_validation_requires_every_observation_in_every_sweep():
    dataset, anchors = _dataset()
    second = copy.deepcopy(dataset["sweeps"][0])
    second["id"] = "second"
    second["data_points"][0]["l_sensor"] = float("nan")
    dataset["sweeps"].append(second)
    assert score_future_sweeps(dataset, anchors, []) == (float("inf"), 2)


def test_invalid_raw_encoder_cannot_fall_back_to_reported_lengths():
    dataset, anchors = _dataset()
    for point in dataset["sweeps"][0]["data_points"]:
        point["raw_angles_deg"] = [0.0, 0.0, 0.0]
    params = build_spool_model_params(
        dataset, base_radii_mm=[30.0] * 3, modeled_radii_mm=[40.0] * 3,
        modeled_buildup_factor=[0.0] * 3, spool_to_motor_gearing_factor=[1.0] * 3,
        mechanical_advantage=[1.0] * 3, lines_per_spool=[1.0] * 3,
    )
    del dataset["sweeps"][0]["data_points"][0]["raw_angles_deg"]
    assert score_future_sweeps(dataset, anchors, [], spool_params=params) == (float("inf"), 1)


def test_validation_does_not_reinfer_encoder_reference_from_future_observations():
    dataset, anchors = _dataset()
    sweep = dataset["sweeps"][0]
    sweep["fixed_lengths"] = [150.0]
    for point in sweep["data_points"]:
        point["raw_angles_deg"] = [
            delta * 180.0 / (np.pi * 40.0)
            for delta in (200.0, point["l_drive"], point["l_sensor"])
        ]
        point["l_drive"] *= 0.75
        point["l_sensor"] *= 0.75
    params = build_spool_model_params(
        dataset, base_radii_mm=[30.0] * 3, modeled_radii_mm=[40.0] * 3,
        modeled_buildup_factor=[0.0] * 3, spool_to_motor_gearing_factor=[1.0] * 3,
        mechanical_advantage=[1.0] * 3, lines_per_spool=[1.0] * 3,
        theta0_mode="infer",
    )
    good = score_future_sweeps(dataset, anchors, [], spool_params=params)
    for point in sweep["data_points"]:
        point["raw_angles_deg"][2] += 20.0
    shifted = score_future_sweeps(dataset, anchors, [], spool_params=params)

    assert np.allclose(params.theta0_deg, 0.0)
    assert shifted[0] > good[0]
