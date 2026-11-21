import numpy as np

from autocal.anchor_uncertainty import (
    coordinate_names, jackknife_spread_mm, newest_trusted_index, plan_anchor_uncertainty, sweep_jackknife_mm, uncertainty_label,
)

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])


def _circle(low, start, angles):
    """Positions that keep the low line and D at their lengths at `start`."""
    a, d = ANCHORS[low], ANCHORS[3]
    axis = (d - a) / np.linalg.norm(d - a)
    centre = a + np.dot(start - a, axis) * axis
    radial = start - centre
    other = np.cross(axis, radial)
    return [centre + np.cos(t) * radial + np.sin(t) * other for t in angles]


def _dataset(sweep_bias_mm=0.0):
    """Six constant-fixed-length sweeps; the first may carry a sensor error the others contradict."""
    home = np.linalg.norm(ANCHORS, axis=1)
    sweeps = []
    for index, (height, low) in enumerate((h, low) for h in (0.0, 600.0) for low in range(3)):
        drive, sensor = [anchor for anchor in range(3) if anchor != low]
        start = np.array([0.0, 0.0, height]) - 300.0 * ANCHORS[low] / home[low] * np.array([1.0, 1.0, 0.0])
        points = []
        for position in _circle(low, start, np.linspace(-0.4, 0.4, 7)):
            lengths = np.linalg.norm(ANCHORS - position, axis=1) - home
            fixed = [lengths[low], lengths[3]]
            points.append({"l_drive": lengths[drive],
                           "l_sensor": lengths[sensor] + (sweep_bias_mm if index == 0 else 0.0)})
        sweeps.append({"fixed_anchors": [low, 3], "fixed_lengths": fixed, "drive_anchor": drive,
                       "sensor_anchor": sensor, "data_points": points})
    return {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {}, "sweeps": sweeps}


def test_consistent_sweeps_have_no_jackknife_spread():
    assert sweep_jackknife_mm(_dataset(), ANCHORS, fit_home_offsets=False) < 1e-3


def test_one_contradicting_sweep_shows_up_as_uncertainty():
    assert sweep_jackknife_mm(_dataset(sweep_bias_mm=0.5), ANCHORS, fit_home_offsets=False) > 0.1


def test_too_few_sweeps_leave_the_uncertainty_unknown():
    dataset = _dataset()
    dataset["sweeps"] = dataset["sweeps"][:2]
    assert sweep_jackknife_mm(dataset, ANCHORS) is None
    assert uncertainty_label(None) == "unknown"


def test_plan_uncertainty_uses_the_estimation_dataset():
    plan = {"anchors": ANCHORS, "dataset": {"sweeps": []}, "dataset_for_estimation": _dataset(),
            "home_offsets_fitted": False}
    assert plan_anchor_uncertainty(plan)[0] < 1e-3


def test_plan_with_fitted_home_offsets_refits_them_from_unshifted_lengths():
    plan = {"anchors": ANCHORS, "dataset": _dataset(), "dataset_for_estimation": {"sweeps": []},
            "home_offsets_fitted": True}
    assert plan_anchor_uncertainty(plan)[0] < 1e-2


def test_failed_jackknife_is_never_applied():
    assert newest_trusted_index([1.0, float("inf")]) == 0
    assert newest_trusted_index([float("inf"), float("inf")]) is None
    assert newest_trusted_index([float("nan")]) is None


def test_diverged_newest_fit_falls_back_to_the_newest_trusted_one():
    assert newest_trusted_index([1.0, 4.0, 25.0]) == 1
    assert newest_trusted_index([None, 1.0, None]) == 2


def test_near_zero_early_uncertainty_does_not_lock_selection():
    assert newest_trusted_index([0.0, 0.2]) == 1
    assert newest_trusted_index([0.01, 2.9]) == 1


def test_labels_follow_millimetre_thresholds():
    assert [uncertainty_label(value) for value in (0.8, 2.0, 4.0, float("inf"))] == \
        ["good", "usable", "poor", "unknown"]


def _five_folds_with_one_loose(loose_mm):
    folds = np.zeros((5, 2, 1)) + np.array([0.1, -0.1, 0.05, -0.05, 0.0])[:, None, None]
    folds[3, 0, 0] = loose_mm
    return folds


def test_a_fold_that_alone_lost_a_coordinate_is_left_out_of_its_spread():
    folds, ratios = _five_folds_with_one_loose(8.0), np.ones((5, 2, 1))
    ratios[3, 0, 0] = 8.0
    spread, single = jackknife_spread_mm(folds, ratios)
    assert spread < 0.2 and single == [(0, 0)]
    assert coordinate_names(single, 5) == ["A.x"]


def test_a_deviating_fold_that_kept_its_constraints_still_counts():
    # One corrupted sweep: the fold without it is the only clean one, and it deviates.
    spread, single = jackknife_spread_mm(_five_folds_with_one_loose(8.0), np.ones((5, 2, 1)))
    assert spread > 3.0 and single == []


def test_a_coordinate_several_folds_lose_keeps_the_classical_spread():
    folds, ratios = _five_folds_with_one_loose(8.0), np.ones((5, 2, 1))
    ratios[3, 0, 0] = ratios[1, 0, 0] = 8.0
    assert jackknife_spread_mm(folds, ratios)[0] > 3.0
