from __future__ import annotations

"""Ellipse fitting utilities based on Maini & Stefano's enhanced direct method."""

import warnings
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple, Union

import numpy as np
from numpy.linalg import eig, inv

from autocal.sweep_types import Sweep, SweepConfigSnapshot


@dataclass
class EllipseFitResult:
    """Result of ellipse fitting."""

    coefficients: np.ndarray  # [A, B, C, D, E, F]
    center: Tuple[float, float]
    semi_axes: Tuple[float, float]  # (a, b) where a >= b
    rotation_angle: float  # radians, canonicalized to [-pi/2, pi/2]
    residual_rms: float
    residual_max: float
    num_points: int
    valid: bool
    rejection_reason: Optional[str] = None


def fit_ellipse_maini_stefano(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Fit ellipse using Maini & Stefano's Least Squares method.

    Re-center and scale data into a [-1, 1]² box (EDFE) to tame conditioning,
    solve the constrained eigenproblem, and fall back to perturb-and-resample
    when the scatter matrix is ill-conditioned.

    Parameters:
        x, y: Arrays of point coordinates (already squared for L² data)

    Returns:
        Coefficients [A, B, C, D, E, F] of Ax² + Bxy + Cy² + Dx + Ey + F = 0
    """
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()

    if len(x) != len(y):
        raise ValueError("x and y must have the same length")
    if len(x) < 5:
        raise ValueError("At least 5 points required for ellipse fitting")

    # Re-center and scale into [-1, 1]² per Maini & Stefano
    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)

    sx = (x_max - x_min) / 2.0
    sy = (y_max - y_min) / 2.0

    if sx <= 0 or sy <= 0:
        raise ValueError("Degenerate data range; cannot normalize for ellipse fitting")

    x_hat = (x - x_min) / sx - 1.0
    y_hat = (y - y_min) / sy - 1.0

    def _direct_fit(xn: np.ndarray, yn: np.ndarray) -> Tuple[np.ndarray, float]:
        """Direct constrained fit on normalized data."""
        D = np.column_stack([xn**2, xn * yn, yn**2, xn, yn, np.ones_like(xn)])

        S = D.T @ D
        cond_S = np.linalg.cond(S)

        C = np.zeros((6, 6))
        C[0, 2] = 2
        C[1, 1] = -1
        C[2, 0] = 2

        S1 = S[:3, :3]
        S2 = S[:3, 3:]
        S3 = S[3:, 3:]

        try:
            S3_inv = inv(S3)
        except np.linalg.LinAlgError:
            S3_inv = np.linalg.pinv(S3)

        T = -S3_inv @ S2.T
        M = S1 + S2 @ T

        C1 = C[:3, :3]

        try:
            C1_inv = inv(C1)
        except np.linalg.LinAlgError:
            C1_inv = np.linalg.pinv(C1)

        M_prime = C1_inv @ M

        eigenvalues, eigenvectors = eig(M_prime)

        valid_idx = None
        best_constraint = -np.inf
        for i, vec in enumerate(eigenvectors.T):
            a1, b1, c1 = vec.real[:3]
            constraint_val = 4 * a1 * c1 - b1**2
            if np.isreal(eigenvalues[i]) and constraint_val > 0:
                valid_idx = i
                break
            if constraint_val > best_constraint:
                best_constraint = constraint_val
                valid_idx = i

        v1 = eigenvectors[:, valid_idx].real
        v2 = T @ v1
        coeffs_hat = np.concatenate([v1, v2])
        return coeffs_hat, cond_S

    try:
        coeffs_hat, cond_val = _direct_fit(x_hat, y_hat)
        if not np.isfinite(cond_val) or cond_val > 1e12:
            raise np.linalg.LinAlgError("Ill-conditioned scatter matrix")
    except Exception as first_err:
        # Perturb-and-resample strategy when localization is critical
        replicates = 120
        noise_std = 2e-7
        rng = np.random.default_rng(0)
        coeffs_list = []
        for _ in range(replicates):
            xn = x_hat + rng.normal(0, noise_std, size=x_hat.shape)
            yn = y_hat + rng.normal(0, noise_std, size=y_hat.shape)
            try:
                coeffs_i, _ = _direct_fit(xn, yn)
                coeffs_list.append(coeffs_i)
            except Exception:
                continue

        if not coeffs_list:
            raise ValueError("Ellipse fitting failed after perturb-and-resample") from first_err

        coeffs_hat = np.mean(coeffs_list, axis=0)

    # Denormalize coefficients back to original coordinates
    ox = 1.0 / sx
    oy = 1.0 / sy
    px = -x_min * ox - 1.0
    py = -y_min * oy - 1.0

    Ah, Bh, Ch, Dh, Eh, Fh = coeffs_hat

    A = Ah * ox * ox
    B = Bh * ox * oy
    C = Ch * oy * oy
    D = 2 * Ah * ox * px + Bh * ox * py + Dh * ox
    E = 2 * Ch * oy * py + Bh * px * oy + Eh * oy
    F = Ah * px**2 + Bh * px * py + Ch * py**2 + Dh * px + Eh * py + Fh

    coeffs = np.array([A, B, C, D, E, F], dtype=float)

    norm_factor = np.linalg.norm(coeffs[:3])
    if norm_factor > 0:
        coeffs = coeffs / norm_factor

    return coeffs


def fit_ellipse_fitzgibbon(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Fit ellipse using Fitzgibbon's Least Squares method.

    Minimizes algebraic distance subject to constraint 4AC - B² = 1.

    Parameters:
        x, y: Arrays of point coordinates (already squared for L² data)

    Returns:
        Coefficients [A, B, C, D, E, F] of Ax² + Bxy + Cy² + Dx + Ey + F = 0
    """
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()

    if len(x) != len(y):
        raise ValueError("x and y must have the same length")
    if len(x) < 5:
        raise ValueError("At least 5 points required for ellipse fitting")

    # Normalize data for numerical stability
    mx, my = np.mean(x), np.mean(y)
    sx = np.std(x) if np.std(x) > 0 else 1.0
    sy = np.std(y) if np.std(y) > 0 else 1.0

    x_norm = (x - mx) / sx
    y_norm = (y - my) / sy

    D = np.column_stack(
        [x_norm**2, x_norm * y_norm, y_norm**2, x_norm, y_norm, np.ones_like(x_norm)]
    )

    S = D.T @ D

    C = np.zeros((6, 6))
    C[0, 2] = 2
    C[1, 1] = -1
    C[2, 0] = 2

    S1 = S[:3, :3]
    S2 = S[:3, 3:]
    S3 = S[3:, 3:]

    try:
        S3_inv = inv(S3)
    except np.linalg.LinAlgError:
        S3_inv = np.linalg.pinv(S3)

    T = -S3_inv @ S2.T
    M = S1 + S2 @ T

    C1 = C[:3, :3]

    try:
        C1_inv = inv(C1)
    except np.linalg.LinAlgError:
        C1_inv = np.linalg.pinv(C1)

    M_prime = C1_inv @ M

    eigenvalues, eigenvectors = eig(M_prime)

    valid_idx = None
    for i, (eigval, eigvec) in enumerate(zip(eigenvalues, eigenvectors.T)):
        if np.isreal(eigval) and eigval > 0:
            a1 = eigvec[0].real
            b1 = eigvec[1].real
            c1 = eigvec[2].real
            constraint_val = 4 * a1 * c1 - b1**2
            if constraint_val > 0:
                valid_idx = i
                break

    if valid_idx is None:
        best_constraint = -np.inf
        for i, eigvec in enumerate(eigenvectors.T):
            a1 = eigvec[0].real
            b1 = eigvec[1].real
            c1 = eigvec[2].real
            constraint_val = 4 * a1 * c1 - b1**2
            if constraint_val > best_constraint:
                best_constraint = constraint_val
                valid_idx = i

    v1 = eigenvectors[:, valid_idx].real
    v2 = T @ v1

    coeffs_norm = np.concatenate([v1, v2])

    coeffs = denormalize_ellipse_coeffs(coeffs_norm, mx, my, sx, sy)

    coeffs = coeffs / np.linalg.norm(coeffs[:3])

    return coeffs


def denormalize_ellipse_coeffs(
    coeffs: np.ndarray, mx: float, my: float, sx: float, sy: float
) -> np.ndarray:
    """
    Transform ellipse coefficients from normalized to original coordinates.

    If x' = (x - mx)/sx, y' = (y - my)/sy, then:
    A'x'² + B'x'y' + C'y'² + D'x' + E'y' + F' = 0

    becomes:
    Ax² + Bxy + Cy² + Dx + Ey + F = 0
    """
    A, B, C, D, E, F = coeffs

    A_new = A / (sx**2)
    B_new = B / (sx * sy)
    C_new = C / (sy**2)
    D_new = D / sx - 2 * A * mx / (sx**2) - B * my / (sx * sy)
    E_new = E / sy - 2 * C * my / (sy**2) - B * mx / (sx * sy)
    F_new = (
        F
        - D * mx / sx
        - E * my / sy
        + A * mx**2 / (sx**2)
        + B * mx * my / (sx * sy)
        + C * my**2 / (sy**2)
    )

    return np.array([A_new, B_new, C_new, D_new, E_new, F_new])


def ellipse_algebraic_distance(coeffs: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Compute algebraic distance of points to ellipse.

    Returns value of Ax² + Bxy + Cy² + Dx + Ey + F for each point.
    """
    A, B, C, D, E, F = coeffs
    return A * x**2 + B * x * y + C * y**2 + D * x + E * y + F


def ellipse_sampson_residuals(coeffs: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Compute Sampson approximation of geometric distance for an ellipse.

    This down-weights points where the implicit gradient is large, producing
    a closer proxy to true point-to-ellipse distance than raw algebraic error.
    """
    A, B, C, D, E, F = coeffs
    algebraic = ellipse_algebraic_distance(coeffs, x, y)
    grad_x = 2 * A * x + B * y + D
    grad_y = B * x + 2 * C * y + E
    denom = np.sqrt(grad_x**2 + grad_y**2)
    denom = np.where(denom < 1e-12, 1e-12, denom)
    return algebraic / denom


def ellipse_geometric_params(coeffs: np.ndarray) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    """
    Extract geometric parameters from algebraic coefficients.

    Returns:
        center: (cx, cy)
        semi_axes: (a, b) where a >= b
        rotation: angle in radians
    """
    A, B, C, D, E, F = coeffs

    delta = B**2 - 4 * A * C

    if delta >= 0:
        warnings.warn("Coefficients do not represent an ellipse (delta >= 0)")
        return (0.0, 0.0), (0.0, 0.0), 0.0

    cx = (2 * C * D - B * E) / delta
    cy = (2 * A * E - B * D) / delta

    if abs(B) < 1e-10 and abs(A - C) < 1e-10:
        theta = 0.0
    else:
        theta = 0.5 * np.arctan2(B, A - C)

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    A_rot = A * cos_t**2 + B * cos_t * sin_t + C * sin_t**2
    C_rot = A * sin_t**2 - B * cos_t * sin_t + C * cos_t**2

    F_c = A * cx**2 + B * cx * cy + C * cy**2 + D * cx + E * cy + F

    if abs(A_rot) < 1e-12 or abs(C_rot) < 1e-12:
        warnings.warn("Degenerate rotated ellipse coefficients")
        return (cx, cy), (0.0, 0.0), theta

    a_sq = -F_c / A_rot
    b_sq = -F_c / C_rot

    if a_sq <= 0 or b_sq <= 0:
        warnings.warn("Negative semi-axis squared")
        return (cx, cy), (0.0, 0.0), theta

    a = float(np.sqrt(a_sq))
    b = float(np.sqrt(b_sq))

    if a < b:
        a, b = b, a
        theta += np.pi / 2

    if theta > np.pi / 2:
        theta -= np.pi
    elif theta < -np.pi / 2:
        theta += np.pi

    return (float(cx), float(cy)), (a, b), float(theta)


def fit_ellipse_from_sweep(
    l_drive: np.ndarray,
    l_sensor: np.ndarray,
    residual_threshold: float = 0.01,
    min_points: int = 10,
    square_inputs: bool = True,
) -> EllipseFitResult:
    """
    Fit ellipse to sweep data (l_drive, l_sensor).

    By default squares the inputs and fits in (L², L²) space. Set
    `square_inputs=False` to fit directly on the provided values (useful when
    working with raw absolute lengths whose baselines are approximate).

    Parameters:
        l_drive: Drive cable lengths
        l_sensor: Sensor cable lengths
        residual_threshold: Maximum RMS residual for valid fit
        min_points: Minimum points required
        square_inputs: Whether to square inputs before fitting

    Returns:
        EllipseFitResult with fit parameters and quality metrics
    """
    l_drive = np.asarray(l_drive, dtype=float).ravel()
    l_sensor = np.asarray(l_sensor, dtype=float).ravel()

    num_points = len(l_drive)

    if num_points != len(l_sensor):
        return EllipseFitResult(
            coefficients=np.zeros(6),
            center=(0.0, 0.0),
            semi_axes=(0.0, 0.0),
            rotation_angle=0.0,
            residual_rms=np.inf,
            residual_max=np.inf,
            num_points=num_points,
            valid=False,
            rejection_reason="Drive and sensor arrays must have the same length",
        )

    if num_points < min_points:
        return EllipseFitResult(
            coefficients=np.zeros(6),
            center=(0.0, 0.0),
            semi_axes=(0.0, 0.0),
            rotation_angle=0.0,
            residual_rms=np.inf,
            residual_max=np.inf,
            num_points=num_points,
            valid=False,
            rejection_reason=f"Insufficient points ({num_points} < {min_points})",
        )

    if square_inputs:
        x = l_drive**2
        y = l_sensor**2
    else:
        x = l_drive
        y = l_sensor

    try:
        coeffs = fit_ellipse_maini_stefano(x, y)
    except Exception as exc:
        return EllipseFitResult(
            coefficients=np.zeros(6),
            center=(0.0, 0.0),
            semi_axes=(0.0, 0.0),
            rotation_angle=0.0,
            residual_rms=np.inf,
            residual_max=np.inf,
            num_points=num_points,
            valid=False,
        rejection_reason=f"Fitting failed: {exc}",
        )

    sampson_residuals = ellipse_sampson_residuals(coeffs, x, y)

    residual_rms = float(np.sqrt(np.mean(sampson_residuals**2)))
    residual_max = float(np.max(np.abs(sampson_residuals)))

    center, semi_axes, rotation = ellipse_geometric_params(coeffs)

    geom_valid = semi_axes[0] > 0 and semi_axes[1] > 0
    valid = residual_rms < residual_threshold and geom_valid

    if not geom_valid:
        rejection_reason = "Degenerate ellipse geometry"
    elif not valid:
        rejection_reason = f"RMS residual too high ({residual_rms:.4f} > {residual_threshold})"
    else:
        rejection_reason = None

    return EllipseFitResult(
        coefficients=coeffs,
        center=center,
        semi_axes=semi_axes,
        rotation_angle=rotation,
        residual_rms=residual_rms,
        residual_max=residual_max,
        num_points=num_points,
        valid=valid,
        rejection_reason=rejection_reason,
    )


def fit_all_sweeps(
    sweeps: Iterable[Union[Sweep, dict]],
    residual_threshold: float = 0.01,
    min_points: int = 10,
    square_inputs: bool = False,
) -> List[dict]:
    """
    Fit ellipses to all sweeps in a dataset.

    Parameters:
        sweeps: Iterable of sweeps whose lengths have already been reconstructed
                to absolute values for the current anchor guess
        residual_threshold: Maximum RMS residual for valid fit
        min_points: Minimum number of points required per sweep
        square_inputs: Whether to square lengths before fitting

    Returns:
        List of fitted ellipse dicts
    """
    results: List[dict] = []

    for sweep in sweeps:
        if isinstance(sweep, Sweep):
            sweep_id = sweep.id
            l_drive = np.array([p.l_drive for p in sweep.data_points], dtype=float)
            l_sensor = np.array([p.l_sensor for p in sweep.data_points], dtype=float)
            sweep_config = SweepConfigSnapshot.from_sweep(sweep).__dict__
        else:
            sweep_id = sweep.get("id", "")
            data_points = sweep.get("data_points", [])
            l_drive = np.array([p.get("l_drive") for p in data_points], dtype=float)
            l_sensor = np.array([p.get("l_sensor") for p in data_points], dtype=float)
            sweep_config = {
                "fixed_anchors": list(sweep.get("fixed_anchors", [])),
                "fixed_lengths": list(sweep.get("fixed_lengths", [])),
                "drive_anchor": sweep.get("drive_anchor"),
                "sensor_anchor": sweep.get("sensor_anchor"),
            }

        result = fit_ellipse_from_sweep(
            l_drive,
            l_sensor,
            residual_threshold=residual_threshold,
            min_points=min_points,
            square_inputs=square_inputs,
        )

        if result.valid:
            x_resid = l_drive**2 if square_inputs else l_drive
            y_resid = l_sensor**2 if square_inputs else l_sensor
            residual_series = ellipse_sampson_residuals(result.coefficients, x_resid, y_resid)
        else:
            residual_series = np.array([], dtype=float)

        results.append(
            {
                "sweep_id": sweep_id,
                "coefficients": {
                    "A": result.coefficients[0],
                    "B": result.coefficients[1],
                    "C": result.coefficients[2],
                    "D": result.coefficients[3],
                    "E": result.coefficients[4],
                    "F": result.coefficients[5],
                },
                "center": {"x": result.center[0], "y": result.center[1]},
                "semi_axes": {"a": result.semi_axes[0], "b": result.semi_axes[1]},
                "rotation_angle_rad": result.rotation_angle,
                "residual_rms": result.residual_rms,
                "residual_max": result.residual_max,
                "residual_series": residual_series.tolist(),
                "valid": result.valid,
                "num_points": result.num_points,
                "rejection_reason": result.rejection_reason,
                # Snapshot of the sweep roles so downstream consumers don't need
                # the raw sweep payload to remain unchanged.
                "sweep_config": sweep_config,
            }
        )

    return results
