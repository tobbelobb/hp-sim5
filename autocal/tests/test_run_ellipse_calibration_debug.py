import json
from pathlib import Path

import numpy as np

from scripts.run_ellipse_calibration_debug import (
    _parse_anchors_2d,
    inflate_dataset_to_absolute,
)


def _tiny_delta_dataset() -> dict:
    phi = np.linspace(0, np.pi, 10)
    l_drive = np.sqrt(10000 + 1000 * np.cos(phi))
    l_sensor = np.sqrt(12000 + 800 * np.cos(phi))
    return {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0],
                "fixed_lengths": [10.0],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {"l_drive": float(ld), "l_sensor": float(ls)}
                    for ld, ls in zip(l_drive, l_sensor)
                ],
            }
        ],
    }


def test_parse_anchors_2d():
    anchors = _parse_anchors_2d("0,-1900;1645.4,950;-1645.4,950")
    assert anchors.shape == (3, 2)
    assert anchors[0, 1] == -1900


def test_inflate_with_base_length():
    dataset = _tiny_delta_dataset()
    abs_ds = inflate_dataset_to_absolute(dataset, base_length=100.0)
    sweep = abs_ds["sweeps"][0]
    assert sweep["fixed_lengths"][0] == 110.0
    assert sweep["data_points"][0]["l_drive"] > dataset["sweeps"][0]["data_points"][0]["l_drive"]


def test_inflate_with_anchors():
    dataset = _tiny_delta_dataset()
    anchors = np.array([[0.0, -1900.0], [1645.0, 950.0], [-1645.0, 950.0]])
    abs_ds = inflate_dataset_to_absolute(dataset, anchors=anchors)
    sweep = abs_ds["sweeps"][0]
    baselines = np.linalg.norm(anchors, axis=1)
    assert np.isclose(sweep["fixed_lengths"][0], baselines[0] + 10.0)
    first_point = sweep["data_points"][0]
    assert np.isclose(first_point["l_drive"], dataset["sweeps"][0]["data_points"][0]["l_drive"] + baselines[1])

