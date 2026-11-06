import copy

import pytest

from autocal.ellipse_cost import EllipseCostFunction
from autocal.fit_stage import _uses_projected_angles
from autocal.projection_mode import resolve_projection_mode


def _dataset(projected=True):
    points = [
        {"l_drive": 10.0, "l_sensor": 20.0, "l_drive_mu": 100.0, "l_sensor_mu": 200.0}
        for _ in range(2)
    ]
    if projected:
        for point in points:
            point["raw_angles_zero_tension_deg"] = [1.0, 2.0, 3.0]
    return {
        "machine_type": "slideprinter", "num_anchors": 3, "dimensions": 2,
        "sweeps": [{"id": "sweep_001", "fixed_anchors": [0], "fixed_lengths": [1.0],
                    "drive_anchor": 1, "sensor_anchor": 2, "data_points": points}],
    }


def test_legacy_unmarked_projected_records_use_projected_lengths():
    dataset = _dataset()
    assert resolve_projection_mode(dataset) is True
    assert _uses_projected_angles(dataset, [])
    cost = EllipseCostFunction(dataset)
    assert cost._packed_sweeps[0].l_drive.tolist() == [10.0, 10.0]
    assert cost._packed_sweeps[0].l_sensor.tolist() == [20.0, 20.0]


def test_unmarked_ordinary_records_keep_noise_means():
    dataset = _dataset(projected=False)
    assert resolve_projection_mode(dataset) is False
    cost = EllipseCostFunction(dataset)
    assert cost._packed_sweeps[0].l_drive.tolist() == [100.0, 100.0]


@pytest.mark.parametrize("declared,point_mode", [(False, True), (True, False)])
def test_contradictory_metadata_is_rejected(declared, point_mode):
    dataset = _dataset(projected=point_mode)
    dataset["config"] = {"noise_model": {"project_zero_tension": declared}}
    with pytest.raises(ValueError, match="metadata contradicts"):
        EllipseCostFunction(dataset)
    with pytest.raises(ValueError, match="metadata contradicts"):
        _uses_projected_angles(dataset, [])


def test_mixed_points_are_rejected_even_if_metadata_is_present():
    dataset = _dataset()
    del dataset["sweeps"][0]["data_points"][1]["raw_angles_zero_tension_deg"]
    dataset["config"] = {"noise_model": {"project_zero_tension": True}}
    with pytest.raises(ValueError, match="mixed projected"):
        EllipseCostFunction(dataset)


@pytest.mark.parametrize("angles", [[1.0, 2.0], [1.0, 2.0, float('nan')], None])
def test_malformed_projected_angles_are_rejected(angles):
    dataset = _dataset()
    dataset["sweeps"][0]["data_points"][0]["raw_angles_zero_tension_deg"] = angles
    with pytest.raises(ValueError, match="raw_angles_zero_tension_deg"):
        resolve_projection_mode(dataset)


@pytest.mark.parametrize("flag", ["--project-zero-tension", "--projectZeroTension"])
def test_explicit_projection_rejects_ordinary_records(flag):
    dataset = _dataset(projected=False)
    before = copy.deepcopy(dataset)
    with pytest.raises(ValueError, match="Projection requested"):
        _uses_projected_angles(dataset, [flag])
    assert dataset == before
