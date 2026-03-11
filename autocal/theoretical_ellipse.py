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
        # Degenerate case: the (cos, sin) coefficient vectors are linearly dependent, so the
        # parametric curve in (x, y) space collapses to a line:
        #   P * (x - K_d) + Q * (y - K_s) = 0
        #
        # Any (P, Q) satisfying:
        #   P*M_d + Q*M_s = 0  and  P*N_d + Q*N_s = 0
        # defines the same line, up to scaling. Prefer a numerically stable nonzero choice.
        cand1 = np.array([M_s, -M_d], dtype=float)
        cand2 = np.array([N_s, -N_d], dtype=float)
        if float(np.dot(cand2, cand2)) > float(np.dot(cand1, cand1)):
            P, Q = float(cand2[0]), float(cand2[1])
        else:
            P, Q = float(cand1[0]), float(cand1[1])

        if not (np.isfinite(P) and np.isfinite(Q)) or (abs(P) + abs(Q) < 1e-12):
            # Fully degenerate: both anchors coincide with the circle center so both x and y are
            # constant in the forward model, but real sweep data will not satisfy all-zero coeffs.
            # Emit a simple nonzero line to avoid a misleading zero-cost objective.
            P, Q = 1.0, 0.0

        norm_pq = float(np.hypot(P, Q))
        if norm_pq > 1e-12:
            P /= norm_pq
            Q /= norm_pq
        F = -(P * float(K_d) + Q * float(K_s))
        return np.array([0.0, 0.0, 0.0, P, Q, F], dtype=float)

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


_ANCHOR_OPT_MODE_FULL = 0
_ANCHOR_OPT_MODE_SLIDEPRINTER_AX0 = 1
_ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z = 2
_ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z = 3


def _coerce_low_anchor_z(low_anchor_z: Optional[float]) -> Optional[float]:
    if low_anchor_z is None:
        return None
    try:
        val = float(low_anchor_z)
    except (TypeError, ValueError):
        return None
    return val if np.isfinite(val) else None


def _slideprinter_ax0_gauge_enabled(
    machine_type: Union[str, MachineType],
    n_anchors: int,
    dims: int,
) -> bool:
    mt = machine_type.value if isinstance(machine_type, MachineType) else str(machine_type)
    return str(mt) == "slideprinter" and int(n_anchors) == 3 and int(dims) == 2


def _hangprinter_4_ax0_gauge_enabled(
    machine_type: Union[str, MachineType],
    n_anchors: int,
    dims: int,
) -> bool:
    mt = machine_type.value if isinstance(machine_type, MachineType) else str(machine_type)
    return str(mt) == "hangprinter_4" and int(n_anchors) == 4 and int(dims) == 3


def anchor_opt_constraint_mode(
    machine_type: Union[str, MachineType],
    n_anchors: int,
    dims: int,
    low_anchor_z: Optional[float] = None,
) -> int:
    if _slideprinter_ax0_gauge_enabled(machine_type, n_anchors, dims):
        return _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0
    if _hangprinter_4_ax0_gauge_enabled(machine_type, n_anchors, dims):
        if _coerce_low_anchor_z(low_anchor_z) is not None:
            return _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z
        return _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z
    return _ANCHOR_OPT_MODE_FULL


def _canonicalize_xy_gauge(anchors: np.ndarray, *, orient_idx: int) -> np.ndarray:
    out = np.asarray(anchors, dtype=float).copy()
    if out.ndim != 2 or out.shape[1] < 2 or out.shape[0] <= 0:
        return out

    ref_xy = out[0, :2]
    if float(np.linalg.norm(ref_xy)) > 1e-12:
        ang = float(np.arctan2(float(ref_xy[1]), float(ref_xy[0])))
        rot = float(-np.pi / 2 - ang)
        c = float(np.cos(rot))
        s = float(np.sin(rot))
        rmat = np.array([[c, -s], [s, c]], dtype=float)
        out[:, :2] = out[:, :2] @ rmat.T

    if int(orient_idx) < out.shape[0] and out[int(orient_idx), 0] < 0.0:
        out[:, 0] *= -1.0
    return out


def canonicalize_anchor_gauge(
    machine_type: Union[str, MachineType],
    anchors: np.ndarray,
    low_anchor_z: Optional[float] = None,
) -> np.ndarray:
    """
    Fix unobservable rotation/reflection gauges for reduced optimizer layouts.

    Sweep-length data identifies some anchor layouts only up to a rigid rotation/reflection
    about the origin or Z axis. Canonicalizing before dropping constrained coordinates
    preserves an equivalent geometry while making the reduced parameterization deterministic.
    """
    anchors = np.asarray(anchors, dtype=float)
    mode = anchor_opt_constraint_mode(machine_type, anchors.shape[0], anchors.shape[1], low_anchor_z)
    if mode == _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0:
        return _canonicalize_xy_gauge(anchors, orient_idx=1)
    if mode in (_ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z, _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z):
        return _canonicalize_xy_gauge(anchors, orient_idx=1)
    return anchors.copy()


def anchor_opt_vector_size(
    machine_type: Union[str, MachineType],
    n_anchors: int,
    dims: int,
    low_anchor_z: Optional[float] = None,
) -> int:
    """Return the optimizer vector length for this anchor layout."""
    full_size = int(n_anchors) * int(dims)
    mode = anchor_opt_constraint_mode(machine_type, n_anchors, dims, low_anchor_z)
    if mode == _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0:
        return full_size - 1
    if mode == _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z:
        return full_size - 3
    if mode == _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z:
        return full_size - 4
    return full_size


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


def get_anchor_opt_bounds(
    machine_type: Union[str, MachineType],
    n_anchors: int,
    dims: int,
    low_anchor_z: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return optimizer bounds for the reduced anchor parameterization."""
    lb, ub = get_anchor_bounds(machine_type)
    mode = anchor_opt_constraint_mode(machine_type, n_anchors, dims, low_anchor_z)
    if mode == _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0:
        bound_mat = np.maximum(
            np.abs(np.asarray(lb, dtype=float).reshape(int(n_anchors), int(dims))),
            np.abs(np.asarray(ub, dtype=float).reshape(int(n_anchors), int(dims))),
        )
        max_anchor_norm = float(np.max(np.linalg.norm(bound_mat, axis=1))) if bound_mat.size else 1.0
        if not np.isfinite(max_anchor_norm) or max_anchor_norm <= 0.0:
            max_anchor_norm = float(np.max(bound_mat)) if bound_mat.size else 1.0
        opt_size = anchor_opt_vector_size(machine_type, n_anchors, dims, low_anchor_z)
        return (
            np.full(opt_size, -max_anchor_norm, dtype=float),
            np.full(opt_size, max_anchor_norm, dtype=float),
        )
    if mode in (_ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z, _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z):
        lb_mat = np.asarray(lb, dtype=float).reshape(int(n_anchors), int(dims))
        ub_mat = np.asarray(ub, dtype=float).reshape(int(n_anchors), int(dims))
        xy_bound_mat = np.maximum(np.abs(lb_mat[:, :2]), np.abs(ub_mat[:, :2]))
        max_xy_norm = float(np.max(np.linalg.norm(xy_bound_mat, axis=1))) if xy_bound_mat.size else 1.0
        if not np.isfinite(max_xy_norm) or max_xy_norm <= 0.0:
            max_xy_norm = float(np.max(xy_bound_mat)) if xy_bound_mat.size else 1.0
        z_lb = float(np.min(lb_mat[:, 2])) if lb_mat.shape[1] >= 3 else -max_xy_norm
        z_ub = float(np.max(ub_mat[:, 2])) if ub_mat.shape[1] >= 3 else max_xy_norm
        if mode == _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z:
            return (
                np.asarray(
                    [
                        -max_xy_norm,
                        z_lb,
                        -max_xy_norm,
                        -max_xy_norm,
                        -max_xy_norm,
                        -max_xy_norm,
                        -max_xy_norm,
                        -max_xy_norm,
                        z_lb,
                    ],
                    dtype=float,
                ),
                np.asarray(
                    [
                        max_xy_norm,
                        z_ub,
                        max_xy_norm,
                        max_xy_norm,
                        max_xy_norm,
                        max_xy_norm,
                        max_xy_norm,
                        max_xy_norm,
                        z_ub,
                    ],
                    dtype=float,
                ),
            )
        return (
            np.asarray(
                [
                    -max_xy_norm,
                    -max_xy_norm,
                    -max_xy_norm,
                    -max_xy_norm,
                    -max_xy_norm,
                    -max_xy_norm,
                    -max_xy_norm,
                    z_lb,
                ],
                dtype=float,
            ),
            np.asarray(
                [
                    max_xy_norm,
                    max_xy_norm,
                    max_xy_norm,
                    max_xy_norm,
                    max_xy_norm,
                    max_xy_norm,
                    max_xy_norm,
                    z_ub,
                ],
                dtype=float,
            ),
        )
    return np.asarray(lb, dtype=float), np.asarray(ub, dtype=float)


def anchors_vec_to_matrix(vec: np.ndarray, n_anchors: int, dims: int) -> np.ndarray:
    """Convert flat anchor vector to (N, D) matrix."""
    vec = np.asarray(vec, dtype=float)
    return vec.reshape(n_anchors, dims)


def anchors_matrix_to_vec(matrix: np.ndarray) -> np.ndarray:
    """Convert (N, D) anchor matrix to flat vector."""
    matrix = np.asarray(matrix, dtype=float)
    return matrix.ravel()


def anchor_opt_vec_to_matrix(
    vec: np.ndarray,
    machine_type: Union[str, MachineType],
    n_anchors: int,
    dims: int,
    low_anchor_z: Optional[float] = None,
) -> np.ndarray:
    """
    Convert an optimizer vector to a full `(N, D)` anchor matrix.

    Reduced optimizer layouts reconstruct constrained anchor coordinates here.
    Full vectors remain accepted for compatibility with existing callers/tests.
    """
    arr = np.asarray(vec, dtype=float)
    if arr.ndim == 2:
        if arr.shape != (int(n_anchors), int(dims)):
            raise ValueError(f"Expected anchor matrix shape {(n_anchors, dims)}, got {arr.shape}")
        return arr.copy()

    flat = arr.reshape(-1)
    full_size = int(n_anchors) * int(dims)
    mode = anchor_opt_constraint_mode(machine_type, n_anchors, dims, low_anchor_z)
    opt_size = anchor_opt_vector_size(machine_type, n_anchors, dims, low_anchor_z)

    if flat.size == full_size:
        return flat.reshape(int(n_anchors), int(dims))
    if flat.size != opt_size:
        raise ValueError(f"Expected anchor vector of size {opt_size} or {full_size}, got {flat.size}")
    if opt_size == full_size:
        return flat.reshape(int(n_anchors), int(dims))
    if mode == _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0:
        full = np.empty(full_size, dtype=float)
        full[0] = 0.0
        full[1:] = flat
        return full.reshape(int(n_anchors), int(dims))
    if mode == _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z:
        ay, low_z, bx, by, cx, cy, dx, dy, dz = [float(v) for v in flat.tolist()]
        return np.asarray(
            [
                [0.0, ay, low_z],
                [bx, by, low_z],
                [cx, cy, low_z],
                [dx, dy, dz],
            ],
            dtype=float,
        )
    if mode == _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z:
        low_z = _coerce_low_anchor_z(low_anchor_z)
        if low_z is None:
            raise ValueError("Reduced hangprinter_4 fixed-Z mode requires a finite low_anchor_z")
        ay, bx, by, cx, cy, dx, dy, dz = [float(v) for v in flat.tolist()]
        return np.asarray(
            [
                [0.0, ay, low_z],
                [bx, by, low_z],
                [cx, cy, low_z],
                [dx, dy, dz],
            ],
            dtype=float,
        )
    return flat.reshape(int(n_anchors), int(dims))


def anchors_matrix_to_opt_vec(
    matrix: np.ndarray,
    machine_type: Union[str, MachineType],
    low_anchor_z: Optional[float] = None,
) -> np.ndarray:
    """Convert an anchor matrix to optimizer coordinates."""
    anchors = np.asarray(matrix, dtype=float)
    if anchors.ndim != 2:
        raise ValueError(f"Expected anchor matrix, got shape {anchors.shape}")

    mode = anchor_opt_constraint_mode(machine_type, anchors.shape[0], anchors.shape[1], low_anchor_z)
    if mode == _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0:
        anchors = canonicalize_anchor_gauge(machine_type, anchors, low_anchor_z=low_anchor_z)
        return anchors.ravel()[1:]
    if mode == _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z:
        anchors = canonicalize_anchor_gauge(machine_type, anchors, low_anchor_z=low_anchor_z)
        low_z = float(np.mean(anchors[:3, 2]))
        return np.asarray(
            [
                anchors[0, 1],
                low_z,
                anchors[1, 0],
                anchors[1, 1],
                anchors[2, 0],
                anchors[2, 1],
                anchors[3, 0],
                anchors[3, 1],
                anchors[3, 2],
            ],
            dtype=float,
        )
    if mode == _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z:
        anchors = canonicalize_anchor_gauge(machine_type, anchors, low_anchor_z=low_anchor_z)
        return np.asarray(
            [
                anchors[0, 1],
                anchors[1, 0],
                anchors[1, 1],
                anchors[2, 0],
                anchors[2, 1],
                anchors[3, 0],
                anchors[3, 1],
                anchors[3, 2],
            ],
            dtype=float,
        )
    return anchors.ravel()
