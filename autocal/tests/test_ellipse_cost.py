import copy
from typing import Optional

import numpy as np
from autocal.ellipse_cost import (
    EllipseCostFunction,
    canonicalize_geometry,
)


def _synthetic_dataset(num_sweeps: int = 3):
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

    base_points = [
        {"l_drive": float(ld), "l_sensor": float(ls)}
        for ld, ls in zip(l_drive_delta, l_sensor_delta)
    ]
    sweeps = []
    for idx in range(num_sweeps):
        sweeps.append(
            {
                "id": f"sweep_{idx + 1:03d}",
                "fixed_anchors": [fixed_anchor],
                "fixed_lengths": [fixed_delta],
                "drive_anchor": drive_idx,
                "sensor_anchor": sensor_idx,
                "data_points": [dict(point) for point in base_points],
            }
        )

    dataset = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": sweeps,
    }

    return dataset, anchors


def _with_noise_model(
    dataset: dict,
    *,
    find_radii_mode: str,
    find_buildup_mode: str,
    line_width_mm: float,
    sigma_floor_mm: float = 0.05,
    sigma_used_mm: Optional[float] = None,
) -> dict:
    out = copy.deepcopy(dataset)
    noise_model = {
        "line_width_mm": float(line_width_mm),
        "find_radii_mode": str(find_radii_mode),
        "find_buildup_mode": str(find_buildup_mode),
        "layered": bool(str(find_radii_mode) != "off" or str(find_buildup_mode) != "off"),
        "sigma_floor_mm": float(sigma_floor_mm),
    }
    if sigma_used_mm is not None:
        noise_model["sigma_used_mm"] = float(sigma_used_mm)
    out["config"] = {
        "m666": {
            "R": [40.0, 40.0, 40.0],
            "Q": 0.0,
            "W": 0.01,
        },
        "mm_per_degree": [1.0, 1.0, 1.0],
        "encoder_noise_origin_mm": [0.02, 0.02, 0.02],
        "force_tuning": {
            "force_low_n": 0.03,
            "force_mid_n": 0.23,
            "force_max_n": 1.03,
        },
        "noise_model": noise_model,
    }
    return out


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
    assert detailed.num_valid_sweeps == 3


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
    dataset, anchors = _synthetic_dataset(num_sweeps=4)
    # Trim to force an invalid fit (min_points default is 3)
    dataset["sweeps"][0]["data_points"] = dataset["sweeps"][0]["data_points"][:2]

    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_filtering=True,
        pointwise_filter_stage=2,
    )
    result = cost_fn.evaluate_detailed(anchors.ravel())

    assert result.num_invalid_sweeps == 1
    assert result.total_cost < 1e-3


def test_noise_mean_lengths_reduce_cost():
    dataset, anchors = _synthetic_dataset()
    points = dataset["sweeps"][0]["data_points"]
    for point in points:
        point["l_drive_mu"] = point["l_drive"]
        point["l_sensor_mu"] = point["l_sensor"]
        point["l_drive"] = float(point["l_drive"]) + 25.0
        point["l_sensor"] = float(point["l_sensor"]) - 25.0

    cost_raw = EllipseCostFunction(dataset, use_noise_mean=False).evaluate(anchors.ravel())
    cost_mu = EllipseCostFunction(dataset, use_noise_mean=True).evaluate(anchors.ravel())

    assert cost_mu < cost_raw


def test_constructor_packs_sweeps_once_with_mu_and_sigma_replacements():
    dataset, anchors = _synthetic_dataset()
    dataset["config"] = {
        "mm_per_degree": [1.0, 1.0, 1.0],
        "encoder_noise_origin_mm": [0.02, 0.02, 0.02],
    }

    point = dataset["sweeps"][0]["data_points"][0]
    original_drive = float(point["l_drive"])
    original_sensor = float(point["l_sensor"])
    point["l_drive"] = original_drive + 25.0
    point["l_sensor"] = original_sensor - 25.0
    point["mu"] = [0.0, original_drive, original_sensor]
    point["sigma"] = [0.1, 0.2, 0.3]

    reference_dataset = copy.deepcopy(dataset)
    expected_cost = EllipseCostFunction(
        reference_dataset,
        use_noise_mean=True,
        sigma_source="point",
        noise_normalized=True,
    ).evaluate(anchors.ravel())

    cost_fn = EllipseCostFunction(
        dataset,
        use_noise_mean=True,
        sigma_source="point",
        noise_normalized=True,
    )
    packed = cost_fn._packed_sweeps[0]
    assert packed.fixed_indices == (0,)
    assert np.isclose(packed.fixed_deltas[0], dataset["sweeps"][0]["fixed_lengths"][0], atol=1e-12)
    assert packed.drive_idx == 1
    assert packed.sensor_idx == 2
    assert np.isclose(float(packed.l_drive[0]), original_drive, atol=1e-12)
    assert np.isclose(float(packed.l_sensor[0]), original_sensor, atol=1e-12)
    assert packed.sigma_drive_mm is not None
    assert packed.sigma_sensor_mm is not None
    assert np.isclose(float(packed.sigma_drive_mm[0]), 0.2, atol=1e-12)
    assert np.isclose(float(packed.sigma_sensor_mm[0]), 0.3, atol=1e-12)

    sweep = dataset["sweeps"][0]
    sweep["fixed_lengths"][0] = float(sweep["fixed_lengths"][0]) + 1234.0
    sweep["drive_anchor"] = 0
    sweep["sensor_anchor"] = 1
    point["l_drive"] = original_drive + 5000.0
    point["l_sensor"] = original_sensor - 5000.0
    point["l_drive_mu"] = original_drive + 900.0
    point["l_sensor_mu"] = original_sensor - 900.0
    point["mu"] = [0.0, 999.0, 999.0]
    point["sigma"] = [9.0, 9.0, 9.0]

    packed_cost = cost_fn.evaluate(anchors.ravel())
    assert np.isclose(packed_cost, expected_cost, rtol=0.0, atol=1e-12)


def test_sigma_components_non_layered_quadrature():
    dataset, anchors = _synthetic_dataset()
    dataset = _with_noise_model(
        dataset,
        find_radii_mode="off",
        find_buildup_mode="off",
        line_width_mm=1.0,
    )
    cost_fn = EllipseCostFunction(dataset)
    diag = cost_fn.robustness_diagnostics(anchors.ravel())
    pw = diag["pointwise_filtering"]
    assert pw["sigma_layered_enabled"] is False
    assert np.isclose(float(pw["sigma_layer_changes_mm"]), 0.0, atol=1e-12)
    assert np.isclose(float(pw["sigma_mode_addition_mm"]), 0.0, atol=1e-12)
    sigma_min = float(pw["sigma_min_mm"])
    assert sigma_min > 0.0

    expected = np.sqrt(0.02**2 + 0.3**2 + 0.1**2 + sigma_min**2)
    assert np.isclose(float(pw["sigma_non_layered_mm"]), expected, atol=1e-6)
    assert np.isclose(float(pw["sigma_total_mm"]), expected, atol=1e-6)
    assert np.isclose(float(pw["sigma_model_mm"]), expected, atol=1e-6)
    assert np.isclose(float(pw["sigma_used_mm"]), expected, atol=1e-6)
    assert pw["sigma_floor_source"] == "model"


def test_sigma_components_layered_include_mode_and_linewidth_terms():
    dataset, anchors = _synthetic_dataset()
    dataset = _with_noise_model(
        dataset,
        find_radii_mode="global",
        find_buildup_mode="global",
        line_width_mm=1.0,
    )
    cost_fn = EllipseCostFunction(dataset)
    diag = cost_fn.robustness_diagnostics(anchors.ravel())
    pw = diag["pointwise_filtering"]
    assert pw["sigma_layered_enabled"] is True
    assert pw["sigma_solver_mode"] == "global/global"
    assert np.isclose(float(pw["sigma_solver_mode_factor"]), 5.0, atol=1e-12)

    sigma_non_layered = float(pw["sigma_non_layered_mm"])
    sigma_layer_changes = float(pw["sigma_layer_changes_mm"])
    sigma_mode_add = float(pw["sigma_mode_addition_mm"])
    sigma_total = float(pw["sigma_total_mm"])
    assert sigma_layer_changes > 0.0
    assert np.isclose(sigma_mode_add, 5.0 * sigma_layer_changes, atol=1e-6)

    expected = np.sqrt(sigma_non_layered**2 + sigma_layer_changes**2 + sigma_mode_add**2)
    assert np.isclose(sigma_total, expected, atol=1e-6)
    assert np.isclose(float(pw["sigma_model_mm"]), expected, atol=1e-6)
    assert np.isclose(float(pw["sigma_used_mm"]), expected, atol=1e-6)
    assert sigma_total > sigma_non_layered


def test_sigma_used_override_sets_final_normalization_sigma():
    dataset, anchors = _synthetic_dataset()
    dataset = _with_noise_model(
        dataset,
        find_radii_mode="global",
        find_buildup_mode="off",
        line_width_mm=1.0,
        sigma_floor_mm=0.05,
        sigma_used_mm=0.9,
    )
    cost_fn = EllipseCostFunction(dataset)
    diag = cost_fn.robustness_diagnostics(anchors.ravel())
    pw = diag["pointwise_filtering"]
    assert np.isclose(float(pw["sigma_min_mm"]), 0.05, atol=1e-12)
    assert float(pw["sigma_model_mm"]) > 0.05
    assert np.isclose(float(pw["sigma_used_mm"]), 0.9, atol=1e-12)
    assert np.isclose(float(pw["sigma_floor_mm"]), 0.9, atol=1e-12)
    assert np.isclose(float(pw["sigma_used_override_mm"]), 0.9, atol=1e-12)
    assert pw["sigma_floor_source"] == "override"
