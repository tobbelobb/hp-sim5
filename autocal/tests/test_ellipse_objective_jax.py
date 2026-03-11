import numpy as np
import pytest

from autocal.ellipse_cost import EllipseCostFunction
from autocal.ellipse_objective_jax import _JAX_AVAILABLE, build_compiled_value_and_grad
from autocal.theoretical_ellipse import (
    anchors_matrix_to_opt_vec,
    compute_constraint_circle_3d,
    get_anchor_bounds,
    get_anchor_opt_bounds,
)


def _synthetic_slideprinter_dataset():
    anchors = np.asarray(
        [
            [-400.0, 0.0],
            [400.0, 0.0],
            [0.0, 500.0],
        ],
        dtype=float,
    )
    fixed_anchor = 0
    drive_idx = 1
    sensor_idx = 2
    fixed_len_abs = 600.0

    phi = np.linspace(0.0, np.pi, 24)
    center = anchors[fixed_anchor]
    positions = center + fixed_len_abs * np.column_stack([np.cos(phi), np.sin(phi)])
    l_drive_abs = np.linalg.norm(positions - anchors[drive_idx], axis=1)
    l_sensor_abs = np.linalg.norm(positions - anchors[sensor_idx], axis=1)

    l_drive_delta = l_drive_abs - np.linalg.norm(anchors[drive_idx])
    l_sensor_delta = l_sensor_abs - np.linalg.norm(anchors[sensor_idx])
    fixed_delta = fixed_len_abs - np.linalg.norm(anchors[fixed_anchor])

    points = []
    for ld, ls in zip(l_drive_delta, l_sensor_delta):
        points.append(
            {
                "l_drive": float(ld),
                "l_sensor": float(ls),
                "l_drive_mu": float(ld),
                "l_sensor_mu": float(ls),
                "sigma": [0.0, 0.0, 0.0],
            }
        )

    dataset = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "config": {
            "mm_per_degree": [1.0, 1.0, 1.0],
            "encoder_noise_origin_mm": [0.02, 0.02, 0.02],
        },
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [fixed_anchor],
                "fixed_lengths": [float(fixed_delta)],
                "drive_anchor": drive_idx,
                "sensor_anchor": sensor_idx,
                "data_points": points,
            },
            {
                "id": "sweep_002",
                "fixed_anchors": [fixed_anchor],
                "fixed_lengths": [float(fixed_delta)],
                "drive_anchor": drive_idx,
                "sensor_anchor": sensor_idx,
                "data_points": points,
            },
            {
                "id": "sweep_003",
                "fixed_anchors": [fixed_anchor],
                "fixed_lengths": [float(fixed_delta)],
                "drive_anchor": drive_idx,
                "sensor_anchor": sensor_idx,
                "data_points": points,
            },
        ],
    }
    return dataset, anchors


def _hangprinter_4_sweep(
    anchors: np.ndarray,
    *,
    sweep_id: str,
    fixed_anchors: tuple[int, int],
    drive_anchor: int,
    sensor_anchor: int,
    seed_position: np.ndarray,
) -> dict:
    fixed_lengths_abs = [
        float(np.linalg.norm(np.asarray(seed_position, dtype=float) - anchors[idx]))
        for idx in fixed_anchors
    ]
    circle = compute_constraint_circle_3d(anchors, list(fixed_anchors), fixed_lengths_abs)
    assert circle is not None

    phi = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    positions = (
        circle.center[None, :]
        + circle.radius * np.cos(phi)[:, None] * circle.u[None, :]
        + circle.radius * np.sin(phi)[:, None] * circle.v[None, :]
    )
    l_drive_abs = np.linalg.norm(positions - anchors[drive_anchor], axis=1)
    l_sensor_abs = np.linalg.norm(positions - anchors[sensor_anchor], axis=1)

    l_drive_delta = l_drive_abs - np.linalg.norm(anchors[drive_anchor])
    l_sensor_delta = l_sensor_abs - np.linalg.norm(anchors[sensor_anchor])
    fixed_deltas = [
        float(length_abs - np.linalg.norm(anchors[idx]))
        for idx, length_abs in zip(fixed_anchors, fixed_lengths_abs)
    ]

    points = []
    for ld, ls in zip(l_drive_delta, l_sensor_delta):
        points.append(
            {
                "l_drive": float(ld),
                "l_sensor": float(ls),
                "l_drive_mu": float(ld),
                "l_sensor_mu": float(ls),
                "sigma": [0.0, 0.0, 0.0, 0.0],
            }
        )

    return {
        "id": str(sweep_id),
        "fixed_anchors": list(fixed_anchors),
        "fixed_lengths": fixed_deltas,
        "drive_anchor": int(drive_anchor),
        "sensor_anchor": int(sensor_anchor),
        "data_points": points,
    }


def _synthetic_hangprinter_4_dataset(*, low_anchor_z: float | None = None):
    low_z = -120.0 if low_anchor_z is None else float(low_anchor_z)
    anchors = np.asarray(
        [
            [0.0, -1900.0, low_z],
            [1645.0, 950.0, low_z],
            [-1645.0, 950.0, low_z],
            [0.0, 0.0, 2050.0],
        ],
        dtype=float,
    )

    dataset = {
        "version": "1.0",
        "machine_type": "hangprinter_4",
        "num_anchors": 4,
        "dimensions": 3,
        "config": {
            "mm_per_degree": [1.0, 1.0, 1.0, 1.0],
            "encoder_noise_origin_mm": [0.02, 0.02, 0.02, 0.02],
        },
        "sweeps": [
            _hangprinter_4_sweep(
                anchors,
                sweep_id="sweep_001",
                fixed_anchors=(0, 1),
                drive_anchor=2,
                sensor_anchor=3,
                seed_position=np.asarray([0.0, -200.0, 600.0], dtype=float),
            ),
            _hangprinter_4_sweep(
                anchors,
                sweep_id="sweep_002",
                fixed_anchors=(0, 2),
                drive_anchor=1,
                sensor_anchor=3,
                seed_position=np.asarray([120.0, -80.0, 500.0], dtype=float),
            ),
            _hangprinter_4_sweep(
                anchors,
                sweep_id="sweep_003",
                fixed_anchors=(0, 3),
                drive_anchor=1,
                sensor_anchor=2,
                seed_position=np.asarray([80.0, 160.0, 400.0], dtype=float),
            ),
        ],
    }
    if low_anchor_z is not None:
        dataset["_low_anchor_z"] = float(low_anchor_z)
    return dataset, anchors


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX is not installed")
def test_jax_objective_matches_numpy_cost():
    dataset, anchors = _synthetic_slideprinter_dataset()
    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
        noise_normalized=True,
    )
    lb, ub = get_anchor_bounds("slideprinter")
    objective = build_compiled_value_and_grad(cost_fn, np.asarray(lb, dtype=float), np.asarray(ub, dtype=float))
    assert objective is not None

    x = np.asarray(anchors, dtype=float).reshape(-1)
    value_jax, grad_jax = objective(x)
    value_np = float(cost_fn.evaluate(x))

    assert np.isfinite(value_jax)
    assert np.isfinite(value_np)
    assert np.all(np.isfinite(np.asarray(grad_jax, dtype=float)))
    assert np.asarray(grad_jax, dtype=float).shape == x.shape
    assert np.isclose(value_jax, value_np, rtol=1e-6, atol=1e-8)


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX is not installed")
def test_jax_underconstrained_penalty_is_finite():
    dataset, anchors = _synthetic_slideprinter_dataset()
    dataset["sweeps"] = list(dataset["sweeps"][:2])

    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
        noise_normalized=True,
    )
    lb, ub = get_anchor_bounds("slideprinter")
    objective = build_compiled_value_and_grad(cost_fn, np.asarray(lb, dtype=float), np.asarray(ub, dtype=float))
    assert objective is not None

    x1 = np.asarray(anchors, dtype=float).reshape(-1)
    x2 = x1.copy()
    x2[0] += 20.0

    value_1, grad_1 = objective(x1)
    value_2, grad_2 = objective(x2)

    assert np.isfinite(float(value_1))
    assert np.isfinite(float(value_2))
    assert np.all(np.isfinite(np.asarray(grad_1, dtype=float)))
    assert np.all(np.isfinite(np.asarray(grad_2, dtype=float)))
    assert float(value_1) >= 0.0
    assert float(value_2) >= 0.0


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX is not installed")
def test_jax_stage2_filtering_value_remains_finite_with_outliers():
    dataset, anchors = _synthetic_slideprinter_dataset()
    first_sweep = dataset["sweeps"][0]
    points = first_sweep.get("data_points", [])
    assert isinstance(points, list) and points
    points[0]["l_drive"] = float(points[0]["l_drive"]) + 80.0
    points[0]["l_sensor"] = float(points[0]["l_sensor"]) - 60.0
    points[0]["l_drive_mu"] = float(points[0]["l_drive_mu"]) + 80.0
    points[0]["l_sensor_mu"] = float(points[0]["l_sensor_mu"]) - 60.0

    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_filter_stage=2,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
        noise_normalized=True,
    )
    lb, ub = get_anchor_bounds("slideprinter")
    objective = build_compiled_value_and_grad(cost_fn, np.asarray(lb, dtype=float), np.asarray(ub, dtype=float))
    assert objective is not None

    x = np.asarray(anchors, dtype=float).reshape(-1)
    value_0, grad_0 = objective(x)
    value_minus, _ = objective(x - 1e-3)
    value_plus, _ = objective(x + 1e-3)

    assert np.isfinite(float(value_0))
    assert np.isfinite(float(value_minus))
    assert np.isfinite(float(value_plus))
    assert np.asarray(grad_0, dtype=float).shape == x.shape


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX is not installed")
def test_jax_objective_accepts_reduced_slideprinter_optimizer_vector():
    dataset, anchors = _synthetic_slideprinter_dataset()
    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
        noise_normalized=True,
    )
    lb_opt, ub_opt = get_anchor_opt_bounds("slideprinter", 3, 2)
    objective = build_compiled_value_and_grad(cost_fn, np.asarray(lb_opt, dtype=float), np.asarray(ub_opt, dtype=float))
    assert objective is not None

    x_full = np.asarray(anchors, dtype=float).reshape(-1)
    x_opt = anchors_matrix_to_opt_vec(anchors, "slideprinter")
    value_opt, grad_opt = objective(x_opt)
    value_full = float(cost_fn.evaluate(x_full))
    value_np = float(cost_fn.evaluate(x_opt))

    assert np.isfinite(float(value_opt))
    assert np.all(np.isfinite(np.asarray(grad_opt, dtype=float)))
    assert np.asarray(grad_opt, dtype=float).shape == x_opt.shape
    assert np.isclose(value_opt, value_np, rtol=1e-6, atol=1e-8)
    assert np.isclose(value_opt, value_full, rtol=1e-6, atol=1e-8)


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX is not installed")
def test_jax_hangprinter_4_objective_matches_numpy_cost():
    dataset, anchors = _synthetic_hangprinter_4_dataset()
    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
        noise_normalized=True,
    )
    lb_opt, ub_opt = get_anchor_opt_bounds("hangprinter_4", 4, 3)
    objective = build_compiled_value_and_grad(cost_fn, np.asarray(lb_opt, dtype=float), np.asarray(ub_opt, dtype=float))
    assert objective is not None

    x_opt = anchors_matrix_to_opt_vec(anchors, "hangprinter_4")
    value_jax, grad_jax = objective(x_opt)
    value_np = float(cost_fn.evaluate(x_opt))

    assert np.isfinite(float(value_jax))
    assert np.isfinite(value_np)
    assert np.all(np.isfinite(np.asarray(grad_jax, dtype=float)))
    assert np.asarray(grad_jax, dtype=float).shape == x_opt.shape
    assert np.isclose(value_jax, value_np, rtol=1e-6, atol=1e-8)


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX is not installed")
def test_jax_hangprinter_4_objective_accepts_fixed_low_anchor_z():
    low_anchor_z = -120.0
    dataset, anchors = _synthetic_hangprinter_4_dataset(low_anchor_z=low_anchor_z)
    cost_fn = EllipseCostFunction(
        dataset,
        pointwise_residual_mode="sampson",
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
        noise_normalized=True,
    )
    lb_opt, ub_opt = get_anchor_opt_bounds("hangprinter_4", 4, 3, low_anchor_z)
    objective = build_compiled_value_and_grad(cost_fn, np.asarray(lb_opt, dtype=float), np.asarray(ub_opt, dtype=float))
    assert objective is not None

    x_opt = anchors_matrix_to_opt_vec(anchors, "hangprinter_4", low_anchor_z)
    value_opt, grad_opt = objective(x_opt)
    value_np = float(cost_fn.evaluate(x_opt))

    assert np.isfinite(float(value_opt))
    assert np.all(np.isfinite(np.asarray(grad_opt, dtype=float)))
    assert np.asarray(grad_opt, dtype=float).shape == x_opt.shape
    assert np.isclose(value_opt, value_np, rtol=1e-6, atol=1e-8)
