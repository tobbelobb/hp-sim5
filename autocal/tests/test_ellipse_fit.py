import copy

import numpy as np
import pytest

from autocal._autocal_common import _normalize_dataset_point_roles, build_anchor_initial_guess
from autocal.ellipse_fit import fit_ellipses
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares, shift_dataset_home_offsets
from autocal.tests.test_disturbed_robustness import TRUTH, _disturbed_points, _load
from autocal.tools.disturb_dataset import _solve_position

# Max anchor error on the disturbed benchmark sweeps: the clean result plus 0.5 mm.
LIMIT_MM = {"hangprinter_3": 1.45, "hangprinter_4": 1.5,
            "hangprinter_5": 1.6, "hangprinter_5_pulley_reduced": 2.6}


def _rigid(dataset, anchors):
    """Exact lengths at the recorded poses: no stretch, offsets or noise."""
    dataset = copy.deepcopy(dataset)
    home = np.linalg.norm(anchors, axis=1)
    for sweep in dataset["sweeps"]:
        held = [int(a) for a in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        for point in sweep["data_points"]:
            lines = np.array(held + [drive])
            lengths = np.array([home[a] + v for a, v in zip(held, sweep["fixed_lengths"])] + [home[drive] + point["l_drive"]])
            position = _solve_position(anchors, lines, lengths, np.zeros(3))
            point["l_drive_mu"] = point["l_drive"]
            point["l_sensor"] = point["l_sensor_mu"] = float(np.linalg.norm(position - anchors[sensor]) - home[sensor])
            point["fixed_error_angles_deg"] = point["fixed_error_angles_mu_deg"] = None
            point["sensor_force_n"] = float("nan")
            for extra in point.get("extra_sensor_ranges", []):
                extra["length_mm"] = extra["length_mu_mm"] = float(
                    np.linalg.norm(position - anchors[extra["anchor"]]) - home[extra["anchor"]])
    return dataset


@pytest.mark.parametrize("machine", ["hangprinter_5", "hangprinter_5_pulley_reduced"])
def test_extra_range_ellipses_recover_exact_hp5_geometry(machine):
    anchors = np.asarray(TRUTH[machine], dtype=float)
    dataset = _rigid(_load(f"benchmark_923c2baa_{machine}"), anchors)
    _normalize_dataset_point_roles(dataset)
    start = anchors + np.array([[0, 15, 5], [-10, 0, 5], [0, -10, 5], [10, 0, 5], [5, -5, -10.]])
    fit = fit_ellipses(dataset, start)
    assert fit["num_ellipses"] > len(dataset["sweeps"])
    assert np.max(np.linalg.norm(fit["anchors"] - anchors, axis=1)) < 0.05


@pytest.mark.parametrize("machine", sorted(TRUTH))
def test_ellipse_fit_rejects_disturbances_and_keeps_accuracy(machine):
    dataset = _load(f"disturbed_benchmark_923c2baa_{machine}")
    disturbed = _disturbed_points(dataset)
    _normalize_dataset_point_roles(dataset)
    seed = build_anchor_initial_guess(dataset, machine_type=machine)
    offsets = fit_pose_least_squares(dataset, seed, fit_home_offsets=True,
                                     home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    fit = fit_ellipses(shift_dataset_home_offsets(dataset, offsets["home_offsets_mm"]), offsets["anchors"])
    assert np.max(np.linalg.norm(fit["anchors"] - np.asarray(TRUTH[machine]), axis=1)) <= LIMIT_MM[machine]
    assert set(fit["rejected_points"].tolist()) == disturbed
