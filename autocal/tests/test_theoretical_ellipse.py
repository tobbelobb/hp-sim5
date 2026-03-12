import numpy as np
import pytest

from autocal.ellipse_fitting import fit_ellipse_from_sweep
from autocal.sweep_types import MachineType
from autocal.theoretical_ellipse import (
    anchor_opt_vec_to_matrix,
    anchor_opt_vector_size,
    anchors_matrix_to_vec,
    anchors_matrix_to_opt_vec,
    anchors_vec_to_matrix,
    canonicalize_anchor_gauge,
    compute_constraint_circle_2d,
    compute_constraint_circle_3d,
    get_anchor_bounds,
    get_anchor_opt_bounds,
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


def test_parametric_to_algebraic_degenerate_det_produces_nontrivial_line():
    # When det≈0, the sweep collapses to a line in (x=L_d^2, y=L_s^2) space.
    # Regression test: the previous fallback only used M terms, producing all-zero
    # coefficients when M_d=M_s=0 even if N terms were present.
    K_d = 10.0
    M_d = 0.0
    N_d = 4.0
    K_s = 20.0
    M_s = 0.0
    N_s = 0.0

    coeffs = parametric_to_algebraic_ellipse(K_d, M_d, N_d, K_s, M_s, N_s)
    assert np.linalg.norm(coeffs) > 0.0
    assert np.linalg.norm(coeffs[:3]) == pytest.approx(0.0)

    phi = np.linspace(0.0, 2.0 * np.pi, 17)
    x = K_d + M_d * np.cos(phi) + N_d * np.sin(phi)
    y = K_s + M_s * np.cos(phi) + N_s * np.sin(phi)

    vals = coeffs[3] * x + coeffs[4] * y + coeffs[5]
    assert np.max(np.abs(vals)) < 1e-9

    # A point off the line must violate the constraint.
    off = coeffs[3] * K_d + coeffs[4] * (K_s + 1.0) + coeffs[5]
    assert abs(off) > 1e-6


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


def test_hangprinter_4_bounds_encode_requested_half_space():
    lb, ub = get_anchor_bounds(MachineType.HANGPRINTER_4)
    lb_mat = lb.reshape(4, 3)
    ub_mat = ub.reshape(4, 3)

    assert ub_mat[0, 2] < 0.0
    assert ub_mat[1, 2] < 0.0
    assert ub_mat[2, 2] < 0.0
    assert ub_mat[0, 1] < 0.0
    assert lb_mat[1, 0] > 0.0
    assert lb_mat[1, 1] > 0.0
    assert ub_mat[2, 0] < 0.0
    assert lb_mat[2, 1] > 0.0
    assert lb_mat[3, 2] > 0.0

    lb_opt, ub_opt = get_anchor_opt_bounds(MachineType.HANGPRINTER_4, 4, 3)
    assert ub_opt[0] < 0.0
    assert ub_opt[1] < 0.0
    assert lb_opt[2] > 0.0
    assert lb_opt[3] > 0.0
    assert ub_opt[4] < 0.0
    assert lb_opt[5] > 0.0
    assert lb_opt[8] > 0.0

    fixed_low_z = -120.0
    lb_fixed, ub_fixed = get_anchor_opt_bounds(MachineType.HANGPRINTER_4, 4, 3, fixed_low_z)
    assert ub_fixed[0] < 0.0
    assert lb_fixed[1] > 0.0
    assert lb_fixed[2] > 0.0
    assert ub_fixed[3] < 0.0
    assert lb_fixed[4] > 0.0
    assert lb_fixed[7] > 0.0


def test_hangprinter_4_fixed_low_anchor_z_must_stay_negative():
    with pytest.raises(ValueError, match="low_anchor_z"):
        get_anchor_opt_bounds(MachineType.HANGPRINTER_4, 4, 3, 10.0)


def test_slideprinter_optimizer_helpers_drop_anchor_a_x_without_changing_geometry():
    anchors = np.array([[-400.0, 0.0], [400.0, 0.0], [0.0, 500.0]], dtype=float)

    opt_size = anchor_opt_vector_size(MachineType.SLIDEPRINTER, 3, 2)
    lb_opt, ub_opt = get_anchor_opt_bounds(MachineType.SLIDEPRINTER, 3, 2)
    assert opt_size == 5
    assert lb_opt.shape == (5,)
    assert ub_opt.shape == (5,)

    opt_vec = anchors_matrix_to_opt_vec(anchors, MachineType.SLIDEPRINTER)
    reduced = anchor_opt_vec_to_matrix(opt_vec, MachineType.SLIDEPRINTER, 3, 2)
    canonical = canonicalize_anchor_gauge(MachineType.SLIDEPRINTER, anchors)

    assert opt_vec.shape == (5,)
    assert reduced.shape == (3, 2)
    assert reduced[0, 0] == pytest.approx(0.0, abs=1e-12)
    assert np.allclose(reduced, canonical)
    assert np.allclose(np.linalg.norm(reduced, axis=1), np.linalg.norm(anchors, axis=1))
    assert np.isclose(np.linalg.norm(reduced[0] - reduced[1]), np.linalg.norm(anchors[0] - anchors[1]))
    assert np.isclose(np.linalg.norm(reduced[0] - reduced[2]), np.linalg.norm(anchors[0] - anchors[2]))
    assert np.isclose(np.linalg.norm(reduced[1] - reduced[2]), np.linalg.norm(anchors[1] - anchors[2]))


def test_hangprinter_4_optimizer_helpers_share_low_anchor_z_without_changing_geometry():
    anchors = np.array(
        [
            [0.0, -1900.0, -120.0],
            [1645.0, 950.0, -120.0],
            [-1645.0, 950.0, -120.0],
            [0.0, 0.0, 2050.0],
        ],
        dtype=float,
    )
    angle = np.deg2rad(37.0)
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    rmat = np.array([[c, -s], [s, c]], dtype=float)
    rotated = anchors.copy()
    rotated[:, :2] = rotated[:, :2] @ rmat.T

    opt_size = anchor_opt_vector_size(MachineType.HANGPRINTER_4, 4, 3)
    lb_opt, ub_opt = get_anchor_opt_bounds(MachineType.HANGPRINTER_4, 4, 3)
    opt_vec = anchors_matrix_to_opt_vec(rotated, MachineType.HANGPRINTER_4)
    reduced = anchor_opt_vec_to_matrix(opt_vec, MachineType.HANGPRINTER_4, 4, 3)
    canonical = canonicalize_anchor_gauge(MachineType.HANGPRINTER_4, rotated)

    assert opt_size == 9
    assert lb_opt.shape == (9,)
    assert ub_opt.shape == (9,)
    assert opt_vec.shape == (9,)
    assert reduced.shape == (4, 3)
    assert reduced[0, 0] == pytest.approx(0.0, abs=1e-12)
    assert np.allclose(reduced[:3, 2], reduced[0, 2])
    assert np.allclose(reduced, canonical)

    for i in range(4):
        for j in range(i + 1, 4):
            assert np.isclose(
                np.linalg.norm(reduced[i] - reduced[j]),
                np.linalg.norm(rotated[i] - rotated[j]),
            )


def test_hangprinter_4_optimizer_helpers_accept_fixed_low_anchor_z():
    low_anchor_z = -120.0
    anchors = np.array(
        [
            [0.0, -1900.0, low_anchor_z],
            [1645.0, 950.0, low_anchor_z],
            [-1645.0, 950.0, low_anchor_z],
            [0.0, 0.0, 2050.0],
        ],
        dtype=float,
    )

    opt_size = anchor_opt_vector_size(MachineType.HANGPRINTER_4, 4, 3, low_anchor_z)
    lb_opt, ub_opt = get_anchor_opt_bounds(MachineType.HANGPRINTER_4, 4, 3, low_anchor_z)
    opt_vec = anchors_matrix_to_opt_vec(anchors, MachineType.HANGPRINTER_4, low_anchor_z)
    reduced = anchor_opt_vec_to_matrix(opt_vec, MachineType.HANGPRINTER_4, 4, 3, low_anchor_z)

    assert opt_size == 8
    assert lb_opt.shape == (8,)
    assert ub_opt.shape == (8,)
    assert opt_vec.shape == (8,)
    assert reduced.shape == (4, 3)
    assert reduced[0, 0] == pytest.approx(0.0, abs=1e-12)
    assert np.allclose(reduced[:3, 2], low_anchor_z)
    assert np.allclose(reduced[:, :2], anchors[:, :2])
    assert np.allclose(reduced[3, 2], anchors[3, 2])
