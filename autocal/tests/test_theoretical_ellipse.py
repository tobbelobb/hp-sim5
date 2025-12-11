import numpy as np
import pytest

from autocal.ellipse_fitting import fit_ellipse_from_sweep
from autocal.sweep_types import MachineType
from autocal.theoretical_ellipse import (
    anchors_matrix_to_vec,
    anchors_vec_to_matrix,
    compute_constraint_circle_2d,
    compute_constraint_circle_3d,
    get_anchor_bounds,
    parametric_to_algebraic_ellipse,
    predict_ellipse_coefficients,
    predict_ellipse_geometry,
    squared_length_coefficients,
)


def test_compute_constraint_circle_2d_basic():
    anchors = np.array([[0, 0], [100, 0], [0, 100]], dtype=float)
    circle = compute_constraint_circle_2d(anchors, fixed_anchor_idx=1, fixed_length=50.0)

    assert np.allclose(circle.center, [100, 0])
    assert circle.radius == pytest.approx(50.0)
    assert np.allclose(circle.u, [1.0, 0.0])
    assert np.allclose(circle.v, [0.0, 1.0])


def test_compute_constraint_circle_3d_intersection():
    anchors = np.array(
        [
            [0, 0, 0],
            [100, 0, 0],
            [50, 50, 0],
            [50, 0, 50],
        ],
        dtype=float,
    )

    circle = compute_constraint_circle_3d(anchors, [0, 1], [60.0, 60.0])

    assert circle is not None
    assert np.allclose(circle.center, [50, 0, 0])
    assert circle.radius == pytest.approx(np.sqrt(60.0**2 - 50.0**2))
    assert np.allclose(circle.normal, [1.0, 0.0, 0.0])
    assert np.isclose(np.linalg.norm(circle.u), 1.0)
    assert np.isclose(np.linalg.norm(circle.v), 1.0)
    assert np.isclose(np.dot(circle.u, circle.v), 0.0, atol=1e-12)


def test_compute_constraint_circle_3d_no_intersection_returns_none():
    anchors = np.array([[0, 0, 0], [200, 0, 0], [0, 200, 0]], dtype=float)
    circle = compute_constraint_circle_3d(anchors, [0, 1], [50.0, 50.0])
    assert circle is None


def test_squared_length_coefficients_consistency():
    anchors = np.array([[100, 0], [0, 100], [-100, 0]], dtype=float)
    circle = compute_constraint_circle_2d(anchors, fixed_anchor_idx=0, fixed_length=50.0)

    K, M, N = squared_length_coefficients(circle, anchors[1])

    L_sq_phi0 = K + M  # cos(0)=1, sin(0)=0
    expected_phi0 = 150**2 + 100**2
    assert L_sq_phi0 == pytest.approx(expected_phi0)

    L_sq_phi90 = K + N  # cos(pi/2)=0, sin(pi/2)=1
    expected_phi90 = 100**2 + 50**2
    assert L_sq_phi90 == pytest.approx(expected_phi90)


def test_parametric_to_algebraic_produces_ellipse():
    coeffs = parametric_to_algebraic_ellipse(
        K_d=10000.0,
        M_d=2000.0,
        N_d=1500.0,
        K_s=12000.0,
        M_s=1800.0,
        N_s=-1200.0,
    )

    A, B, C = coeffs[:3]
    discriminant = B**2 - 4 * A * C
    assert discriminant < 0


def simulate_sweep_lengths(circle, anchors, drive_idx, sensor_idx):
    phi = np.linspace(0, np.pi, 80)
    positions = circle.center + circle.radius * (
        np.outer(np.cos(phi), circle.u) + np.outer(np.sin(phi), circle.v)
    )

    l_drive = np.linalg.norm(positions - anchors[drive_idx], axis=1)
    l_sensor = np.linalg.norm(positions - anchors[sensor_idx], axis=1)
    return l_drive, l_sensor


def test_prediction_matches_fitted_ellipse_2d():
    anchors = np.array([[-500, 400], [500, 400], [0, -500]], dtype=float)
    fixed_indices = [0]
    fixed_lengths = [600.0]
    drive_idx = 1
    sensor_idx = 2

    circle = compute_constraint_circle_2d(anchors, fixed_indices[0], fixed_lengths[0])
    l_drive, l_sensor = simulate_sweep_lengths(circle, anchors, drive_idx, sensor_idx)

    fit_result = fit_ellipse_from_sweep(l_drive, l_sensor)
    pred_coeffs = predict_ellipse_coefficients(
        anchors, fixed_indices, fixed_lengths, drive_idx, sensor_idx, dimensions=2
    )

    assert pred_coeffs is not None
    dot = abs(np.dot(fit_result.coefficients, pred_coeffs))
    norm_prod = np.linalg.norm(fit_result.coefficients) * np.linalg.norm(pred_coeffs)
    assert dot / norm_prod > 0.999


def test_prediction_matches_fitted_ellipse_3d():
    anchors = np.array(
        [
            [-1000, -500, -100],
            [1000, -500, -100],
            [0, 1000, -100],
            [0, 0, 2000],
        ],
        dtype=float,
    )
    fixed_indices = [0, 1]
    fixed_lengths = [1200.0, 1200.0]
    drive_idx = 2
    sensor_idx = 3

    circle = compute_constraint_circle_3d(anchors, fixed_indices, fixed_lengths)
    assert circle is not None

    l_drive, l_sensor = simulate_sweep_lengths(circle, anchors, drive_idx, sensor_idx)
    fit_result = fit_ellipse_from_sweep(l_drive, l_sensor)

    pred_coeffs = predict_ellipse_coefficients(
        anchors, fixed_indices, fixed_lengths, drive_idx, sensor_idx, dimensions=3
    )
    assert pred_coeffs is not None

    dot = abs(np.dot(fit_result.coefficients, pred_coeffs))
    norm_prod = np.linalg.norm(fit_result.coefficients) * np.linalg.norm(pred_coeffs)
    assert dot / norm_prod > 0.999

    geom = predict_ellipse_geometry(
        anchors, fixed_indices, fixed_lengths, drive_idx, sensor_idx, dimensions=3
    )
    assert geom is not None
    center, semi_axes, theta = geom
    assert semi_axes[0] > 0 and semi_axes[1] > 0
    assert isinstance(theta, float)
    assert len(center) == 2


def test_anchor_bounds_and_reshaping_helpers():
    lb, ub = get_anchor_bounds(MachineType.SLIDEPRINTER)
    assert lb.shape == (6,)
    assert ub.shape == (6,)
    assert np.all(lb < ub)

    vec = np.arange(6, dtype=float)
    mat = anchors_vec_to_matrix(vec, n_anchors=3, dims=2)
    assert mat.shape == (3, 2)
    assert np.allclose(anchors_matrix_to_vec(mat), vec)
