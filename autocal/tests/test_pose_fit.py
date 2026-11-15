import numpy as np
import pytest

from autocal.pose_fit import (
    HOME_OFFSET_PRIOR_SIGMA_MM,
    dataset_line_lengths,
    fit_pose_least_squares,
    shift_dataset_home_offsets,
)

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])
SEED = ANCHORS + np.array([[0.0, 40.0, 15.0], [-30.0, 20.0, 15.0], [25.0, -20.0, 15.0], [10.0, -10.0, 40.0]])


def _dataset(home_offsets=(0.0, 0.0, 0.0, 0.0)):
    """Sweeps at several heights; every length change is exact geometry minus a per-line home offset."""
    home = np.linalg.norm(ANCHORS, axis=1)
    sweeps = []
    for height in (0.0, 400.0, 800.0):
        for low in range(3):
            drive, sensor = [anchor for anchor in range(3) if anchor != low]
            start = np.array([0.0, 0.0, height]) + 250.0 * ANCHORS[low] / home[low] * np.array([1.0, 1.0, 0.0])
            direction = np.cross(ANCHORS[low] - start, ANCHORS[3] - start)
            direction /= np.linalg.norm(direction)
            points = []
            for step in np.linspace(-500.0, 500.0, 9):
                position = start + step * direction
                lengths = np.linalg.norm(ANCHORS - position, axis=1) - home - np.asarray(home_offsets)
                points.append({"l_drive": lengths[drive], "l_sensor": lengths[sensor],
                               "fixed": [lengths[low], lengths[3]]})
            # Each point lies on its own pair of fixed lengths, so every point is its own one-point sweep.
            for point in points:
                sweeps.append({"fixed_anchors": [low, 3], "fixed_lengths": point.pop("fixed"),
                               "drive_anchor": drive, "sensor_anchor": sensor, "data_points": [point]})
    return {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {}, "sweeps": sweeps}


def _fit(dataset, fit_home_offsets):
    return fit_pose_least_squares(dataset, SEED, fit_home_offsets=fit_home_offsets,
                                  home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)


def test_rigid_data_recovers_the_anchors_from_a_distant_seed():
    result = _fit(_dataset(), fit_home_offsets=False)
    assert np.max(np.abs(result["anchors"] - ANCHORS)) < 1e-3
    assert result["rms_mm"] < 1e-6
    assert result["num_points"] == 81


def test_home_offset_biases_the_rigid_fit_and_is_recovered_when_fitted():
    dataset = _dataset(home_offsets=(0.0, 0.0, 0.0, 1.5))
    assert np.max(np.abs(_fit(dataset, fit_home_offsets=False)["anchors"] - ANCHORS)) > 1.0
    result = _fit(dataset, fit_home_offsets=True)
    assert np.max(np.abs(result["anchors"] - ANCHORS)) < 0.05
    assert result["home_offsets_mm"] == pytest.approx([0.0, 0.0, 0.0, 1.5], abs=0.01)


def test_one_bad_point_does_not_move_the_anchors():
    dataset = _dataset()
    dataset["sweeps"][40]["data_points"][0]["l_sensor"] += 5.0
    result = _fit(dataset, fit_home_offsets=False)
    assert np.max(np.abs(result["anchors"] - ANCHORS)) < 0.1
    assert int(np.argmax(result["point_residuals_mm"])) == 40


def test_shifted_dataset_contains_the_offsets_on_every_line_role():
    dataset = _dataset()
    shifted = shift_dataset_home_offsets(dataset, [1.0, 2.0, 3.0, 4.0])
    difference = dataset_line_lengths(shifted)["lengths_mm"] - dataset_line_lengths(dataset)["lengths_mm"]
    assert difference == pytest.approx(np.tile([1.0, 2.0, 3.0, 4.0], (81, 1)))
    assert dataset_line_lengths(dataset)["lengths_mm"][0, 3] == dataset["sweeps"][0]["fixed_lengths"][1]


def test_fixed_line_lengths_include_the_measured_holding_error():
    dataset = _dataset()
    dataset["config"]["mm_per_degree"] = [0.5, 0.5, 0.5, 0.5]
    sweep = dataset["sweeps"][0]
    sweep["data_points"][0]["fixed_error_angles_mu_deg"] = [0.2, None, None, -0.4]
    lengths = dataset_line_lengths(dataset)["lengths_mm"][0]
    assert lengths[0] == pytest.approx(sweep["fixed_lengths"][0] + 0.1)
    assert lengths[3] == pytest.approx(sweep["fixed_lengths"][1] - 0.2)
