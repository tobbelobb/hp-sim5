import numpy as np
import pytest

from autocal.ellipse_cost import (
    EllipseCostFunction,
    canonicalize_geometry,
)


def _synthetic_dataset():
    """
    Build a simple Slideprinter-style sweep where the fixed cable length
    defines a clean circle and recorded lengths are origin-relative deltas.
    """
    anchors = np.array(
        [
            [-400.0, 0.0],
            [400.0, 0.0],
            [0.0, 500.0],
        ]
    )
    fixed_anchor = 0
    fixed_length_abs = 600.0
    drive_idx = 1
    sensor_idx = 2

    phi = np.linspace(0, np.pi, 60)
    center = anchors[fixed_anchor]
    radius = fixed_length_abs
    positions = center + radius * np.column_stack([np.cos(phi), np.sin(phi)])

    l_drive_abs = np.linalg.norm(positions - anchors[drive_idx], axis=1)
    l_sensor_abs = np.linalg.norm(positions - anchors[sensor_idx], axis=1)

    l_drive_delta = l_drive_abs - np.linalg.norm(anchors[drive_idx])
    l_sensor_delta = l_sensor_abs - np.linalg.norm(anchors[sensor_idx])
    fixed_delta = fixed_length_abs - np.linalg.norm(anchors[fixed_anchor])

    sweep = {
        "id": "sweep_001",
        "fixed_anchors": [fixed_anchor],
        "fixed_lengths": [fixed_delta],
        "drive_anchor": drive_idx,
        "sensor_anchor": sensor_idx,
        "data_points": [
            {"l_drive": float(ld), "l_sensor": float(ls)}
            for ld, ls in zip(l_drive_delta, l_sensor_delta)
        ],
    }

    dataset = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [sweep],
    }

    return dataset, anchors


def test_canonicalize_geometry_enforces_order_and_wrap():
    center_can, axes_can, theta_can = canonicalize_geometry((10.0, -5.0), (5.0, 10.0), 0.9 * np.pi)

    assert axes_can[0] >= axes_can[1]
    assert -np.pi / 2 <= theta_can <= np.pi / 2
    assert np.isclose(np.linalg.norm(center_can - np.array([10.0, -5.0])), 0.0)


def test_cost_is_low_for_true_anchors():
    dataset, anchors = _synthetic_dataset()
    cost_fn = EllipseCostFunction(dataset, residual_threshold=0.01)

    cost = cost_fn.evaluate(anchors.ravel())
    detailed = cost_fn.evaluate_detailed(anchors.ravel())

    assert cost < 1e-3
    assert detailed.num_invalid_sweeps == 0
    assert detailed.num_valid_sweeps == 1


def test_pointwise_euclidean_cost_is_low_for_true_anchors():
    dataset, anchors = _synthetic_dataset()
    cost_fn = EllipseCostFunction(
        dataset,
        residual_threshold=0.01,
        pointwise_residual_mode="euclidean",
    )
    cost = cost_fn.evaluate(anchors.ravel())
    assert cost < 1e-6


def test_invalid_sweep_is_ignored_in_cost():
    dataset, anchors = _synthetic_dataset()
    # Trim to force an invalid fit (min_points default is 3)
    dataset["sweeps"][0]["data_points"] = dataset["sweeps"][0]["data_points"][:2]

    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_filtering=True,
        pointwise_filter_stage=2,
    )
    result = cost_fn.evaluate_detailed(anchors.ravel())

    assert result.num_invalid_sweeps == 1
    assert result.total_cost == pytest.approx(0.0)
