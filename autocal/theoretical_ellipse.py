from __future__ import annotations

"""Forward model for predicting ellipses from anchor geometry and sweep roles."""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Union

import numpy as np
from numpy.linalg import norm

from autocal.ellipse_fitting import ellipse_geometric_params
from autocal.sweep_types import MachineType, Sweep


@dataclass
class ConstraintCircle:
    """A circle in 2D or 3D space representing the effector's constrained path."""

    center: np.ndarray
    radius: float
    u: np.ndarray
    v: np.ndarray
    normal: np.ndarray


def compute_constraint_circle_2d(
    anchors: np.ndarray, fixed_anchor_idx: int, fixed_length: float
) -> ConstraintCircle:
    """
    Compute the constraint circle for the 2D (Slideprinter) case.

    With one cable fixed, the effector moves on a circle centered at that anchor.
    """
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] < 2:
        raise ValueError("anchors must be of shape (N, 2) or wider for 2D computation")

    center = anchors[fixed_anchor_idx][:2].copy()
    radius = float(fixed_length)

    u = np.array([1.0, 0.0])
    v = np.array([0.0, 1.0])
    normal = np.array([0.0, 0.0, 1.0])

    return ConstraintCircle(center=center, radius=radius, u=u, v=v, normal=normal)


def compute_constraint_circle_3d(
    anchors: np.ndarray, fixed_anchor_indices: List[int], fixed_lengths: List[float]
) -> Optional[ConstraintCircle]:
    """
    Compute the constraint circle for the 3D case.

    With two cables fixed, the effector moves on the intersection of two spheres.
    """
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 3:
        raise ValueError("anchors must be of shape (N, 3) for 3D computation")

    if len(fixed_anchor_indices) != 2 or len(fixed_lengths) != 2:
        raise ValueError("3D constraint requires exactly two fixed anchors and lengths")

    j, k = fixed_anchor_indices
    L_j, L_k = float(fixed_lengths[0]), float(fixed_lengths[1])

    A_j = anchors[j]
    A_k = anchors[k]

    d_vec = A_k - A_j
    d = norm(d_vec)

    if d < 1e-10:
        return None

    if d > L_j + L_k:
        return None
    if d < abs(L_j - L_k):
        return None

    h = (d**2 + L_j**2 - L_k**2) / (2 * d)

    r_sq = L_j**2 - h**2
    if r_sq < -1e-10:
        return None
    radius = float(np.sqrt(max(r_sq, 0.0)))

    d_hat = d_vec / d
    center = A_j + h * d_hat

    normal = d_hat

    temp = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, temp)
    u_norm = norm(u)
    if u_norm < 1e-12:
        temp = np.array([0.0, 0.0, 1.0])
        u = np.cross(normal, temp)
        u_norm = norm(u)
    if u_norm < 1e-12:
        return None
    u = u / u_norm
    v = np.cross(normal, u)
    v_norm = norm(v)
    if v_norm < 1e-12:
        return None
    v = v / v_norm

    return ConstraintCircle(center=center, radius=radius, u=u, v=v, normal=normal)


def squared_length_coefficients(circle: ConstraintCircle, anchor: np.ndarray) -> Tuple[float, float, float]:
    """Compute coefficients K, M, N for L²(φ) = K + M·cos(φ) + N·sin(φ)."""
    anchor = np.asarray(anchor, dtype=float)

    if circle.center.shape[0] == 2:
        C = circle.center
        u = circle.u[:2]
        v = circle.v[:2]
        anchor_vec = anchor[:2]
    else:
        C = circle.center
        u = circle.u
        v = circle.v
        anchor_vec = anchor

    delta = C - anchor_vec

    K = float(np.dot(delta, delta) + circle.radius**2)
    M = float(2 * circle.radius * np.dot(u, delta))
    N = float(2 * circle.radius * np.dot(v, delta))

    return K, M, N


def parametric_to_algebraic_ellipse(
    K_d: float,
    M_d: float,
    N_d: float,
    K_s: float,
    M_s: float,
    N_s: float,
) -> np.ndarray:
    """
    Convert parametric L² expressions to algebraic ellipse coefficients.

    Returns coefficients normalized so ||[A, B, C]|| = 1. Degenerate
    configurations (det ≈ 0) fall back to a line-like representation.
    """
    det = M_d * N_s - N_d * M_s

    if abs(det) < 1e-10:
        return np.array([0.0, 0.0, 0.0, M_d, M_s, -K_d * M_d - K_s * M_s], dtype=float)

    A_xy = N_s**2 + M_s**2
    B_xy = -2 * (N_s * N_d + M_s * M_d)
    C_xy = N_d**2 + M_d**2
    F_xy = -det**2

    A = A_xy
    B = B_xy
    C = C_xy
    D = -2 * A_xy * K_d - B_xy * K_s
    E = -2 * C_xy * K_s - B_xy * K_d
    F = A_xy * K_d**2 + B_xy * K_d * K_s + C_xy * K_s**2 + F_xy

    coeffs = np.array([A, B, C, D, E, F], dtype=float)

    abc_norm = norm(coeffs[:3])
    if abc_norm > 1e-10:
        coeffs = coeffs / abc_norm

    return coeffs


def predict_ellipse_coefficients(
    anchors: np.ndarray,
    fixed_anchor_indices: List[int],
    fixed_lengths: List[float],
    drive_anchor_idx: int,
    sensor_anchor_idx: int,
    dimensions: int = 3,
) -> Optional[np.ndarray]:
    """Predict algebraic ellipse coefficients for a sweep configuration."""
    anchors = np.asarray(anchors, dtype=float)

    if dimensions == 2:
        if len(fixed_anchor_indices) != 1:
            raise ValueError("2D case requires exactly one fixed anchor")
        circle = compute_constraint_circle_2d(
            anchors, fixed_anchor_indices[0], fixed_lengths[0]
        )
    else:
        circle = compute_constraint_circle_3d(anchors, fixed_anchor_indices, fixed_lengths)

    if circle is None:
        return None

    K_d, M_d, N_d = squared_length_coefficients(circle, anchors[drive_anchor_idx])
    K_s, M_s, N_s = squared_length_coefficients(circle, anchors[sensor_anchor_idx])

    return parametric_to_algebraic_ellipse(K_d, M_d, N_d, K_s, M_s, N_s)


def predict_ellipse_geometry(
    anchors: np.ndarray,
    fixed_anchor_indices: List[int],
    fixed_lengths: List[float],
    drive_anchor_idx: int,
    sensor_anchor_idx: int,
    dimensions: int = 3,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float], float]]:
    """Predict canonical (center, (a, b), θ) tuple for a sweep configuration."""
    coeffs = predict_ellipse_coefficients(
        anchors,
        fixed_anchor_indices,
        fixed_lengths,
        drive_anchor_idx,
        sensor_anchor_idx,
        dimensions,
    )
    if coeffs is None:
        return None

    return ellipse_geometric_params(coeffs)


def predict_ellipses_for_dataset(
    anchors: np.ndarray, sweeps: Iterable[Union[Sweep, Dict]], dimensions: int
) -> List[Dict[str, object]]:
    """
    Predict ellipse coefficients for all sweeps in a dataset.

    Sweeps may be `Sweep` instances or plain dicts. Fixed lengths are reconstructed
    by adding the current anchor guess baseline (||A_i||) to origin-relative deltas.
    """
    anchors = np.asarray(anchors, dtype=float)
    predictions: List[Dict[str, object]] = []

    for sweep in sweeps:
        if isinstance(sweep, Sweep):
            sweep_id = sweep.id
            fixed_indices = list(sweep.fixed_anchors)
            fixed_deltas = list(sweep.fixed_lengths)
            drive_idx = sweep.drive_anchor
            sensor_idx = sweep.sensor_anchor
        else:
            sweep_id = sweep.get("id", "")
            fixed_indices = list(sweep.get("fixed_anchors", []))
            fixed_deltas = list(sweep.get("fixed_lengths", []))
            drive_idx = sweep.get("drive_anchor")
            sensor_idx = sweep.get("sensor_anchor")

        fixed_lengths = [
            norm(anchors[idx]) + delta for idx, delta in zip(fixed_indices, fixed_deltas)
        ]

        coeffs = predict_ellipse_coefficients(
            anchors, fixed_indices, fixed_lengths, drive_idx, sensor_idx, dimensions
        )

        if coeffs is None:
            predictions.append(
                {"sweep_id": sweep_id, "coefficients": None, "valid": False}
            )
            continue

        predictions.append(
            {
                "sweep_id": sweep_id,
                "coefficients": {
                    "A": coeffs[0],
                    "B": coeffs[1],
                    "C": coeffs[2],
                    "D": coeffs[3],
                    "E": coeffs[4],
                    "F": coeffs[5],
                },
                "valid": True,
            }
        )

    return predictions


def get_anchor_bounds(machine_type: Union[str, MachineType]) -> Tuple[np.ndarray, np.ndarray]:
    """Return lower/upper anchor bounds for a machine type."""
    mt = machine_type.value if isinstance(machine_type, MachineType) else str(machine_type)

    bounds = {
        "slideprinter": {
            "n_anchors": 3,
            "dims": 2,
            "lb": [-2000, -2000] * 3,
            "ub": [2000, 2000] * 3,
        },
        "hangprinter_4": {
            "n_anchors": 4,
            "dims": 3,
            "lb": [-5000, -5000, -2000] * 4,
            "ub": [5000, 5000, 5000] * 4,
        },
        "hangprinter_5": {
            "n_anchors": 5,
            "dims": 3,
            "lb": [-5000, -5000, -2000] * 5,
            "ub": [5000, 5000, 5000] * 5,
        },
        "cubecorners": {
            "n_anchors": 8,
            "dims": 3,
            "lb": [-5000, -5000, -2000] * 8,
            "ub": [5000, 5000, 5000] * 8,
        },
        "skycam": {
            "n_anchors": 4,
            "dims": 3,
            "lb": [-10000, -10000, 2000] * 4,
            "ub": [10000, 10000, 10000] * 4,
        },
    }

    cfg = bounds.get(mt)
    if cfg is None:
        raise ValueError(f"Unknown machine type: {machine_type}")

    return np.array(cfg["lb"], dtype=float), np.array(cfg["ub"], dtype=float)


def anchors_vec_to_matrix(vec: np.ndarray, n_anchors: int, dims: int) -> np.ndarray:
    """Convert flat anchor vector to (N, D) matrix."""
    vec = np.asarray(vec, dtype=float)
    return vec.reshape(n_anchors, dims)


def anchors_matrix_to_vec(matrix: np.ndarray) -> np.ndarray:
    """Convert (N, D) anchor matrix to flat vector."""
    matrix = np.asarray(matrix, dtype=float)
    return matrix.ravel()
