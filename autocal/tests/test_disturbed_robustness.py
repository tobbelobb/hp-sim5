"""Anchor fits on benchmark sweeps with pushed lines and outliers."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from autocal._autocal_common import _normalize_dataset_point_roles, build_anchor_initial_guess
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares, shift_dataset_home_offsets
from autocal.tools.disturb_dataset import PULL_PROFILE, apply_events, disturb_point
from autocal.tools.regress_calibration_logs import BENCHMARK_MACHINES

REFERENCES = Path(__file__).resolve().parents[1] / "data" / "references"
TRUTH = dict(BENCHMARK_MACHINES)
# Max anchor error of the same fit on the undisturbed benchmark sweeps, plus 0.5 mm.
# HP4 carries its known D.z bias.
LIMIT_MM = {"hangprinter_3": 1.5, "hangprinter_4": 5.7,
            "hangprinter_5": 1.2, "hangprinter_5_pulley_reduced": 2.3}


def _load(name):
    return json.loads((REFERENCES / f"{name}.json").read_text())


def _disturbed_points(dataset):
    """Flat indices of points the recorded events changed."""
    starts = np.cumsum([0] + [len(sweep["data_points"]) for sweep in dataset["sweeps"]])
    disturbed = set()
    for event in dataset["disturbances"]:
        profile = PULL_PROFILE if event["kind"] == "pull" else (1.0,)
        disturbed.update(int(starts[event["sweep"]]) + event["point"] + offset
                         for offset, fraction in enumerate(profile) if fraction > 0)
    return disturbed


def _fit(dataset):
    dataset = copy.deepcopy(dataset)
    _normalize_dataset_point_roles(dataset)
    seed = build_anchor_initial_guess(dataset, machine_type=dataset["machine_type"])
    offsets = fit_pose_least_squares(dataset, seed, fit_home_offsets=True,
                                     home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    return fit_pose_least_squares(shift_dataset_home_offsets(dataset, offsets["home_offsets_mm"]),
                                  offsets["anchors"], fit_home_offsets=False,
                                  home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)


@pytest.mark.parametrize("machine", sorted(TRUTH))
def test_point_fit_rejects_disturbances_and_keeps_accuracy(machine):
    dataset = _load(f"disturbed_benchmark_0623060e22_{machine}")
    fit = _fit(dataset)
    error = np.max(np.linalg.norm(fit["anchors"] - np.asarray(TRUTH[machine]), axis=1))
    assert error <= LIMIT_MM[machine]
    assert set(fit["rejected_points"].tolist()) == _disturbed_points(dataset)


@pytest.mark.parametrize("prefix", ["disturbed", "noisy_0p05mm", "noisy_0p2mm", "noisy_0p5mm"])
@pytest.mark.parametrize("machine", sorted(TRUTH))
def test_stored_disturbed_dataset_matches_generator(machine, prefix):
    stored = _load(f"{prefix}_benchmark_0623060e22_{machine}")
    regenerated = apply_events(_load(f"benchmark_0623060e22_{machine}"), TRUTH[machine], stored["disturbances"])
    assert regenerated["disturbances"] == stored["disturbances"]
    for regenerated_sweep, stored_sweep in zip(regenerated["sweeps"], stored["sweeps"], strict=True):
        for new, old in zip(regenerated_sweep["data_points"], stored_sweep["data_points"], strict=True):
            records = [(new, old, key) for key in ("l_drive", "l_drive_mu", "l_sensor", "l_sensor_mu")]
            records += [(a, b, "length_mm") for a, b in zip(new.get("extra_sensor_ranges", []),
                                                           old.get("extra_sensor_ranges", []), strict=True)]
            # The stored HP3/HP4 disturbed sets used 1645.448 for the rounded 1645.45 anchor X.
            assert all(a[key] == pytest.approx(b[key], abs=1e-3) for a, b, key in records)


def _hp5_point():
    anchors = np.asarray(TRUTH["hangprinter_5"], dtype=float)
    position = np.array([120.0, -80.0, 300.0])
    ranges = np.linalg.norm(anchors - position, axis=1) - np.linalg.norm(anchors, axis=1)
    sweep = {"fixed_anchors": [0, 4], "fixed_lengths": [ranges[0], ranges[4]],
             "drive_anchor": 1, "sensor_anchor": 3}
    point = {"l_drive": ranges[1], "l_sensor": ranges[3],
             "extra_sensor_ranges": [{"anchor": 2, "length_mm": ranges[2]}]}
    return anchors, sweep, point


def test_held_line_push_moves_effector_and_tension_lines_record_it():
    anchors, sweep, point = _hp5_point()
    before = copy.deepcopy(point)
    disturb_point(sweep, point, anchors, line=4, detour_mm=10.0)
    assert point["l_drive"] == before["l_drive"]
    moved = [point["l_sensor"] - before["l_sensor"],
             point["extra_sensor_ranges"][0]["length_mm"] - before["extra_sensor_ranges"][0]["length_mm"]]
    assert all(abs(change) > 0.1 for change in moved)
    # The tension lines describe the moved effector, whose top line is 10 mm short.
    home = np.linalg.norm(anchors, axis=1)
    lines = [0, 1, 2, 3]
    ranges = [sweep["fixed_lengths"][0], point["l_drive"], point["extra_sensor_ranges"][0]["length_mm"],
              point["l_sensor"]]
    from autocal.tools.disturb_dataset import _solve_position
    position = _solve_position(anchors, np.array(lines), home[lines] + np.array(ranges), np.zeros(3))
    assert np.linalg.norm(position - anchors[4]) == pytest.approx(home[4] + sweep["fixed_lengths"][1] - 10.0, abs=1e-6)


def test_sensor_line_push_changes_only_that_record():
    anchors, sweep, point = _hp5_point()
    before = copy.deepcopy(point)
    disturb_point(sweep, point, anchors, line=3, detour_mm=4.0)
    assert point["l_sensor"] == pytest.approx(before["l_sensor"] + 4.0)
    assert point["l_drive"] == before["l_drive"]
    assert point["extra_sensor_ranges"] == before["extra_sensor_ranges"]


@pytest.mark.parametrize("machine", sorted(TRUTH))
def test_point_fit_keeps_points_with_ordinary_encoder_noise(machine):
    # Noise is not a disturbance: no point of a 0.2 mm noise set may be rejected.
    assert _fit(_load(f"noisy_0p2mm_benchmark_0623060e22_{machine}"))["rejected_points"].size == 0
