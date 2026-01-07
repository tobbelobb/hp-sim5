# Substep 3: Ellipse Fitting Module

## Overview

Implement robust ellipse fitting using Maini, Eliseo Stefano's Direct Least Squares method. This module takes (l_drive, l_sensor) pairs, squares them to (L_drive², L_sensor²), and fits the algebraic ellipse equation. Quality control metrics determine whether the fit is valid. Downstream cost computations consume the canonical geometric tuple `(x0, y0, a, b, θ)` (with `a >= b` and θ wrapped to a stable interval) rather than raw `(A,B,C,D,E,F)` to avoid scale ambiguity.

Because the ellipse invariant is strictly about *absolute* squared lengths, fitting must happen inside the optimization loop after reconstructing absolute lengths from encoder deltas and the current anchor guess. Pre-fitting invariant ellipses on raw deltas alone is mathematically inconsistent; drop that flow entirely.

## Implementation Details

### 3.1 Numerical Method

See ai_docs/Enhanced_Direct_Least_Square_Fitting_of_Ellipses/Enhanced_Direct_Least_Square_Fitting_of_Ellipses.md for details.

### 3.2 Core Implementation (`ellipse_fitting.py`)

Fit results include a `sweep_config` snapshot (fixed/drive/sense roles and held lengths) so the optimizer in Substep 5 can match each observed ellipse to the correct forward-model prediction without having to rely on the raw sweep payload remaining unchanged.

```python
"""
ellipse_fitting.py

Robust ellipse fitting using Maini & Stefano's Least Squares method.
"""

import numpy as np
from numpy.linalg import eig, inv, svd
from dataclasses import dataclass
from typing import Tuple, Optional, List
import warnings


@dataclass
class EllipseFitResult:
    """Result of ellipse fitting."""
    coefficients: np.ndarray  # [A, B, C, D, E, F]
    center: Tuple[float, float]
    semi_axes: Tuple[float, float]  # (a, b) where a >= b
    rotation_angle: float  # radians, canonicalized to a stable interval (e.g. [-pi/2, pi/2])
    residual_rms: float
    residual_max: float
    num_points: int
    valid: bool
    rejection_reason: Optional[str] = None

def fit_ellipse_maini_stefano(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Fit ellipse using Maini & Stefano's Least Squares method.

    Re-center/scale data into a [-1, 1]² box (EDFE) to tame conditioning, solve
    the constrained eigenproblem, and fall back to perturb-and-resample if the
    scatter matrix is ill-conditioned.

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
        D = np.column_stack([
            xn**2,
            xn * yn,
            yn**2,
            xn,
            yn,
            np.ones_like(xn)
        ])

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
        replicates = 25
        noise_std = 1e-3
        coeffs_list = []
        for _ in range(replicates):
            xn = x_hat + np.random.normal(0, noise_std, size=x_hat.shape)
            yn = y_hat + np.random.normal(0, noise_std, size=y_hat.shape)
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


// This one is only included for comparison and tests.
// A useful test could be: we should have the same fitting for maini and fitzgibbon most of the time
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

    # Build design matrix
    # D = [x², xy, y², x, y, 1]
    D = np.column_stack([
        x_norm**2,
        x_norm * y_norm,
        y_norm**2,
        x_norm,
        y_norm,
        np.ones_like(x_norm)
    ])

    # Build scatter matrix S = D'D
    S = D.T @ D

    # Build constraint matrix C
    # Constraint: 4AC - B² = 1
    # In matrix form for [A,B,C,D,E,F]:
    # C = [[0, 0, 2, 0, 0, 0],
    #      [0, -1, 0, 0, 0, 0],
    #      [2, 0, 0, 0, 0, 0],
    #      [0, 0, 0, 0, 0, 0],
    #      [0, 0, 0, 0, 0, 0],
    #      [0, 0, 0, 0, 0, 0]]
    C = np.zeros((6, 6))
    C[0, 2] = 2
    C[1, 1] = -1
    C[2, 0] = 2

    # Partition matrices for efficiency
    S1 = S[:3, :3]
    S2 = S[:3, 3:]
    S3 = S[3:, 3:]

    # Solve reduced problem
    try:
        S3_inv = inv(S3)
    except np.linalg.LinAlgError:
        # S3 singular, use pseudo-inverse
        S3_inv = np.linalg.pinv(S3)

    T = -S3_inv @ S2.T
    M = S1 + S2 @ T

    # C1 is the upper-left 3x3 of C
    C1 = C[:3, :3]

    # Solve generalized eigenvalue problem M*v = λ*C1*v
    try:
        C1_inv = inv(C1)
    except np.linalg.LinAlgError:
        C1_inv = np.linalg.pinv(C1)

    M_prime = C1_inv @ M

    eigenvalues, eigenvectors = eig(M_prime)

    # Find the positive eigenvalue (constraint requires 4AC - B² > 0)
    # For normalized eigenvector v = [A, B, C], check sign of 4AC - B²
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
        # Fallback: pick eigenvector with largest positive constraint value
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

    # Combine into full coefficient vector
    coeffs_norm = np.concatenate([v1, v2])

    # Denormalize coefficients
    coeffs = denormalize_ellipse_coeffs(coeffs_norm, mx, my, sx, sy)

    # Normalize so coefficient vector has unit norm (for consistent comparison)
    coeffs = coeffs / np.linalg.norm(coeffs[:3])

    return coeffs


def denormalize_ellipse_coeffs(
    coeffs: np.ndarray,
    mx: float, my: float,
    sx: float, sy: float
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
    F_new = (F - D * mx / sx - E * my / sy
             + A * mx**2 / (sx**2)
             + B * mx * my / (sx * sy)
             + C * my**2 / (sy**2))

    return np.array([A_new, B_new, C_new, D_new, E_new, F_new])


def ellipse_algebraic_distance(
    coeffs: np.ndarray,
    x: np.ndarray,
    y: np.ndarray
) -> np.ndarray:
    """
    Compute algebraic distance of points to ellipse.

    Returns value of Ax² + Bxy + Cy² + Dx + Ey + F for each point.
    """
    A, B, C, D, E, F = coeffs
    return A * x**2 + B * x * y + C * y**2 + D * x + E * y + F


def ellipse_geometric_params(coeffs: np.ndarray) -> Tuple[Tuple, Tuple, float]:
    """
    Extract geometric parameters from algebraic coefficients.

    Returns:
        center: (cx, cy)
        semi_axes: (a, b) where a >= b
        rotation: angle in radians
    """
    A, B, C, D, E, F = coeffs

    # Discriminant
    delta = B**2 - 4*A*C

    if delta >= 0:
        warnings.warn("Coefficients do not represent an ellipse (delta >= 0)")
        return (0, 0), (0, 0), 0

    # Center
    cx = (2*C*D - B*E) / delta
    cy = (2*A*E - B*D) / delta

    # Rotation angle
    if abs(B) < 1e-10 and abs(A - C) < 1e-10:
        theta = 0
    else:
        theta = 0.5 * np.arctan2(B, A - C)

    # Semi-axes
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    A_rot = A * cos_t**2 + B * cos_t * sin_t + C * sin_t**2
    C_rot = A * sin_t**2 - B * cos_t * sin_t + C * cos_t**2

    # Value at center
    F_c = A * cx**2 + B * cx * cy + C * cy**2 + D * cx + E * cy + F

    if F_c == 0:
        return (cx, cy), (0, 0), theta

    a_sq = -F_c / A_rot
    b_sq = -F_c / C_rot

    if a_sq < 0 or b_sq < 0:
        warnings.warn("Negative semi-axis squared")
        return (cx, cy), (0, 0), theta

    a = np.sqrt(a_sq)
    b = np.sqrt(b_sq)

    # Ensure a >= b
    if a < b:
        a, b = b, a
        theta += np.pi / 2

    # Wrap rotation to a stable interval to keep comparisons well-conditioned
    if theta > np.pi / 2:
        theta -= np.pi
    elif theta < -np.pi / 2:
        theta += np.pi

    return (cx, cy), (a, b), theta


def fit_ellipse_from_sweep(
    l_drive: np.ndarray,
    l_sensor: np.ndarray,
    residual_threshold: float = 0.01,
    min_points: int = 10
) -> EllipseFitResult:
    """
    Fit ellipse to sweep data (l_drive, l_sensor).

    Squares the inputs and fits in (L², L²) space.

    Parameters:
        l_drive: Drive cable lengths
        l_sensor: Sensor cable lengths
        residual_threshold: Maximum RMS residual for valid fit
        min_points: Minimum points required

    Returns:
        EllipseFitResult with fit parameters and quality metrics
    """
    l_drive = np.asarray(l_drive, dtype=float)
    l_sensor = np.asarray(l_sensor, dtype=float)

    num_points = len(l_drive)

    # Validation
    if num_points < min_points:
        return EllipseFitResult(
            coefficients=np.zeros(6),
            center=(0, 0),
            semi_axes=(0, 0),
            rotation_angle=0,
            residual_rms=np.inf,
            residual_max=np.inf,
            num_points=num_points,
            valid=False,
            rejection_reason=f"Insufficient points ({num_points} < {min_points})"
        )

    # Square the lengths
    x = l_drive**2
    y = l_sensor**2

    # Fit ellipse
    try:
        coeffs = fit_ellipse_maini_stefano(x, y)
    except Exception as e:
        return EllipseFitResult(
            coefficients=np.zeros(6),
            center=(0, 0),
            semi_axes=(0, 0),
            rotation_angle=0,
            residual_rms=np.inf,
            residual_max=np.inf,
            num_points=num_points,
            valid=False,
            rejection_reason=f"Fitting failed: {str(e)}"
        )

    # Compute residuals
    algebraic_dist = ellipse_algebraic_distance(coeffs, x, y)

    # Normalize residuals by data scale
    scale = np.sqrt(np.mean(x**2 + y**2))
    if scale < 1e-10:
        scale = 1.0

    normalized_residuals = algebraic_dist / scale

    residual_rms = np.sqrt(np.mean(normalized_residuals**2))
    residual_max = np.max(np.abs(normalized_residuals))

    # Extract geometric parameters
    center, semi_axes, rotation = ellipse_geometric_params(coeffs)

    # Quality check
    valid = residual_rms < residual_threshold
    rejection_reason = None if valid else f"RMS residual too high ({residual_rms:.4f} > {residual_threshold})"

    return EllipseFitResult(
        coefficients=coeffs,
        center=center,
        semi_axes=semi_axes,
        rotation_angle=rotation,
        residual_rms=residual_rms,
        residual_max=residual_max,
        num_points=num_points,
        valid=valid,
        rejection_reason=rejection_reason
    )


def fit_all_sweeps(sweeps: List[dict], residual_threshold: float = 0.01) -> List[dict]:
    """
    Fit ellipses to all sweeps in a dataset.

    Parameters:
        sweeps: Iterable of sweep dicts whose lengths have already been
                reconstructed to absolute values for the current anchor guess
        residual_threshold: Maximum RMS residual for valid fit

    Returns:
        List of fitted ellipse dicts
    """
    results = []

    for sweep in sweeps:
        sweep_id = sweep['id']
        data_points = sweep['data_points']
        sweep_config = {
            'fixed_anchors': sweep['fixed_anchors'],
            'fixed_lengths': sweep['fixed_lengths'],
            'drive_anchor': sweep['drive_anchor'],
            'sensor_anchor': sweep['sensor_anchor'],
        }

        l_drive = np.array([p['l_drive'] for p in data_points])
        l_sensor = np.array([p['l_sensor'] for p in data_points])

        result = fit_ellipse_from_sweep(l_drive, l_sensor, residual_threshold)

        # Residuals per sample help visualize noise variation along the arc
        residual_series = ellipse_algebraic_distance(result.coefficients, l_drive**2, l_sensor**2) if result.valid else []

        results.append({
            'sweep_id': sweep_id,
            'coefficients': {
                'A': result.coefficients[0],
                'B': result.coefficients[1],
                'C': result.coefficients[2],
                'D': result.coefficients[3],
                'E': result.coefficients[4],
                'F': result.coefficients[5],
            },
            'center': {'x': result.center[0], 'y': result.center[1]},
            'semi_axes': {'a': result.semi_axes[0], 'b': result.semi_axes[1]},
            'rotation_angle_rad': result.rotation_angle,
            'residual_rms': result.residual_rms,
            'residual_max': result.residual_max,
            'residual_series': residual_series.tolist() if isinstance(residual_series, np.ndarray) else [],
            'valid': result.valid,
            'num_points': result.num_points,
            'rejection_reason': result.rejection_reason,
            # Keep a snapshot of the sweep roles so Phase 2 can consume this
            # fit even if the original sweep list is filtered/trimmed.
            'sweep_config': sweep_config,
        })

    return results
```

### 3.3 CLI Tool for Batch Fitting

```python
#!/usr/bin/env python3
"""
fit_ellipses.py

CLI tool to fit ellipses to sweep data.
"""

import argparse
import json
import sys
from pathlib import Path
from ellipse_fitting import fit_all_sweeps


def main():
    parser = argparse.ArgumentParser(
        description='Fit ellipses to sweep calibration data'
    )
    parser.add_argument('input', help='Input sweep data JSON file')
    parser.add_argument('-o', '--output', help='Output JSON file (default: modify input)')
    parser.add_argument('-t', '--threshold', type=float, default=0.01,
                        help='RMS residual threshold for valid fit (default: 0.01)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Print detailed output')

    args = parser.parse_args()

    # Load input
    with open(args.input, 'r') as f:
        dataset = json.load(f)

    # Fit ellipses (expects sweeps already expressed in absolute lengths, e.g. after
    # adding ||anchor_i - origin|| to all stored deltas for a particular anchor guess)
    sweeps = dataset.get('sweeps', [])
    fitted_ellipses = fit_all_sweeps(sweeps, args.threshold)

    # Summary
    valid_count = sum(1 for e in fitted_ellipses if e['valid'])
    print(f"Fitted {len(fitted_ellipses)} sweeps: {valid_count} valid, "
          f"{len(fitted_ellipses) - valid_count} rejected")

    if args.verbose:
        for fe in fitted_ellipses:
            status = "VALID" if fe['valid'] else f"REJECTED: {fe['rejection_reason']}"
            print(f"  {fe['sweep_id']}: RMS={fe['residual_rms']:.6f} {status}")

    # Save output sidecar (keep canonical dataset sweep-only)
    output_path = args.output or args.input.replace('.json', '_fits.json')
    with open(output_path, 'w') as f:
        json.dump({
            "source": args.input,
            "residual_threshold": args.threshold,
            "fitted_ellipses": fitted_ellipses,
        }, f, indent=2)

    print(f"Saved to {output_path}")

    # Exit with error if no valid fits
    if valid_count == 0:
        print("ERROR: No valid ellipse fits!", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
```

## Testing

### Unit Tests

```python
# test_ellipse_fitting.py
import pytest
import numpy as np
from ellipse_fitting import (
    fit_ellipse_maini_stefano,
    fit_ellipse_from_sweep,
    ellipse_geometric_params,
    EllipseFitResult
)


def generate_ellipse_points(
    center: tuple,
    semi_axes: tuple,
    rotation: float,
    n_points: int,
    noise_std: float = 0
) -> tuple:
    """Generate points on an ellipse with optional noise."""
    cx, cy = center
    a, b = semi_axes

    theta = np.linspace(0, 2*np.pi, n_points, endpoint=False)

    # Parametric ellipse
    x_ell = a * np.cos(theta)
    y_ell = b * np.sin(theta)

    # Rotate
    cos_r = np.cos(rotation)
    sin_r = np.sin(rotation)
    x = cx + x_ell * cos_r - y_ell * sin_r
    y = cy + x_ell * sin_r + y_ell * cos_r

    # Add noise
    if noise_std > 0:
        x += np.random.normal(0, noise_std, n_points)
        y += np.random.normal(0, noise_std, n_points)

    return x, y


class TestMainiStefanoFitting:
    def test_perfect_circle(self):
        """Fit a perfect circle (special case of ellipse)."""
        x, y = generate_ellipse_points(
            center=(100, 200),
            semi_axes=(50, 50),
            rotation=0,
            n_points=100
        )

        coeffs = fit_ellipse_maini_stefano(x, y)
        center, semi_axes, rotation = ellipse_geometric_params(coeffs)

        assert abs(center[0] - 100) < 1e-6
        assert abs(center[1] - 200) < 1e-6
        assert abs(semi_axes[0] - 50) < 1e-6
        assert abs(semi_axes[1] - 50) < 1e-6

    def test_rotated_ellipse(self):
        """Fit a rotated ellipse."""
        x, y = generate_ellipse_points(
            center=(500, 800),
            semi_axes=(100, 60),
            rotation=np.pi/6,  # 30 degrees
            n_points=100
        )

        coeffs = fit_ellipse_maini_stefano(x, y)
        center, semi_axes, rotation = ellipse_geometric_params(coeffs)

        assert abs(center[0] - 500) < 1e-3
        assert abs(center[1] - 800) < 1e-3
        assert abs(semi_axes[0] - 100) < 1e-3
        assert abs(semi_axes[1] - 60) < 1e-3

    def test_noisy_data(self):
        """Fit ellipse with Gaussian noise."""
        np.random.seed(42)
        x, y = generate_ellipse_points(
            center=(1000, 2000),
            semi_axes=(200, 100),
            rotation=0.5,
            n_points=200,
            noise_std=5.0
        )

        coeffs = fit_ellipse_maini_stefano(x, y)
        center, semi_axes, rotation = ellipse_geometric_params(coeffs)

        # Looser tolerances for noisy data
        assert abs(center[0] - 1000) < 10
        assert abs(center[1] - 2000) < 10
        assert abs(semi_axes[0] - 200) < 10
        assert abs(semi_axes[1] - 100) < 10

    def test_minimum_points(self):
        """Fitting with exactly 5 points (minimum)."""
        x = np.array([0, 1, 0, -1, 0.5])
        y = np.array([1, 0, -1, 0, 0.866])

        # Should not raise
        coeffs = fit_ellipse_maini_stefano(x, y)
        assert len(coeffs) == 6

    def test_insufficient_points(self):
        """Fitting with fewer than 5 points should raise."""
        x = np.array([0, 1, 0, -1])
        y = np.array([1, 0, -1, 0])

        with pytest.raises(ValueError, match="At least 5 points"):
            fit_ellipse_maini_stefano(x, y)


class TestSweepFitting:
    def test_valid_sweep(self):
        """Fit from simulated sweep data."""
        # Simulate L² = K + M*cos(phi) + N*sin(phi) for two cables
        phi = np.linspace(0, np.pi, 50)  # Half rotation

        K_d, M_d, N_d = 10000, 2000, 1500
        K_s, M_s, N_s = 12000, 1800, -1200

        L_drive_sq = K_d + M_d * np.cos(phi) + N_d * np.sin(phi)
        L_sensor_sq = K_s + M_s * np.cos(phi) + N_s * np.sin(phi)

        # Take sqrt to get lengths
        l_drive = np.sqrt(L_drive_sq)
        l_sensor = np.sqrt(L_sensor_sq)

        result = fit_ellipse_from_sweep(l_drive, l_sensor, residual_threshold=0.01)

        assert result.valid
        assert result.residual_rms < 0.01
        assert result.num_points == 50

    def test_insufficient_points_sweep(self):
        """Sweep with too few points should be rejected."""
        l_drive = np.array([100, 101, 102])
        l_sensor = np.array([95, 96, 97])

        result = fit_ellipse_from_sweep(l_drive, l_sensor, min_points=10)

        assert not result.valid
        assert "Insufficient points" in result.rejection_reason

    def test_high_noise_rejection(self):
        """Highly noisy data should be rejected."""
        np.random.seed(123)
        l_drive = np.random.uniform(80, 120, 50)
        l_sensor = np.random.uniform(90, 110, 50)

        result = fit_ellipse_from_sweep(l_drive, l_sensor, residual_threshold=0.001)

        assert not result.valid
        assert "RMS residual too high" in result.rejection_reason


class TestBatchFitting:
    def test_fit_all_sweeps(self):
        """Test batch fitting of multiple sweeps."""
        from ellipse_fitting import fit_all_sweeps

        # Create mock dataset
        phi = np.linspace(0, np.pi, 30)
        dataset = {
            'sweeps': [
                {
                    'id': 'sweep_001',
                    'fixed_anchors': [0],
                    'fixed_lengths': [500.0],
                    'drive_anchor': 1,
                    'sensor_anchor': 2,
                    'data_points': [
                        {
                            'l_drive': np.sqrt(10000 + 2000*np.cos(p)),
                            'l_sensor': np.sqrt(12000 + 1800*np.cos(p))
                        }
                        for p in phi
                    ]
                },
                {
                    'id': 'sweep_002',
                    'fixed_anchors': [1],
                    'fixed_lengths': [520.0],
                    'drive_anchor': 0,
                    'sensor_anchor': 2,
                    'data_points': [
                        {
                            'l_drive': np.sqrt(9000 + 1500*np.cos(p)),
                            'l_sensor': np.sqrt(11000 + 2000*np.cos(p))
                        }
                        for p in phi
                    ]
                }
            ]
        }

        results = fit_all_sweeps(dataset['sweeps'], residual_threshold=0.01)

        assert len(results) == 2
        assert results[0]['sweep_id'] == 'sweep_001'
        assert results[1]['sweep_id'] == 'sweep_002'
        assert results[0]['sweep_config']['drive_anchor'] == 1
        assert results[1]['sweep_config']['fixed_anchors'] == [1]
```

## Validation Criteria

1. **Perfect Data Recovery**: For synthetic ellipse points with zero noise, recovered parameters match true values within 1e-6
2. **Noise Tolerance**: With σ=1% noise, recovered parameters within 5% of true values
3. **QC Rejection**: Random noise data correctly rejected with appropriate reason
4. **Numerical Stability**: No NaN/Inf in outputs for any valid input
5. **Coefficient Normalization**: ||[A,B,C]|| = 1 for all outputs
6. **Ellipse Guarantee**: Some inequality holds for all valid fits

## Dependencies

- Python 3.8+
- numpy
- scipy (optional, for comparison methods)
- pytest (for testing)

## Estimated Complexity

**Effort**: Medium

The core algorithm is well-documented. Quality control thresholds may need tuning based on real data.

## Files to Create/Modify

| File                                    | Action       |
|-----------------------------------------|--------------|
| `autocal/ellipse_fitting.py`            | Create       |
| `autocal/fit_ellipses.py`               | Create (CLI) |
| `autocal/tests/test_ellipse_fitting.py` | Create       |
