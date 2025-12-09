# Substep 4: Theoretical Ellipse Projection

## Overview

Implement the forward mathematical model that predicts ellipse parameters from anchor positions and sweep configuration. Given a set of anchor positions and the specification of which cables are fixed/driven/sensed, compute the theoretical ellipse that the (L_drive², L_sensor²) relationship should trace.

## Implementation Details

### 4.1 Mathematical Foundation

#### The Constraint Circle

When D-1 cables are held fixed (D = dimensions), the effector is constrained to move on a circle:

**2D Case (Slideprinter, 1 fixed cable):**
- Fixing cable j at length L_j constrains the effector to a circle centered at anchor A_j with radius L_j

**3D Case (Hangprinter, 2 fixed cables):**
- Fixing cables j and k constrains the effector to the intersection of two spheres
- This intersection is a circle in 3D space
- The circle has center C, radius r, and normal vector n

#### Circle-Anchor Distance as Function of Angle

For any anchor A_i and a point P(φ) on the circle:
$$P(\phi) = C + r(\cos\phi \cdot u + \sin\phi \cdot v)$$

where u, v are orthonormal vectors in the plane of the circle.

The squared distance from A_i to P(φ) is:
$$L_i^2(\phi) = ||P(\phi) - A_i||^2 = K_i + M_i\cos\phi + N_i\sin\phi$$

where:
- $K_i = ||C - A_i||^2 + r^2$
- $M_i = 2r \cdot u \cdot (C - A_i)$
- $N_i = 2r \cdot v \cdot (C - A_i)$

### 4.2 Core Implementation (`theoretical_ellipse.py`)

```python
"""
theoretical_ellipse.py

Compute theoretical ellipse parameters from anchor geometry and sweep configuration.
"""

import numpy as np
from numpy.linalg import norm, svd
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass


@dataclass
class ConstraintCircle:
    """A circle in 2D or 3D space representing the effector's constrained path."""
    center: np.ndarray      # Circle center (2D or 3D)
    radius: float           # Circle radius
    u: np.ndarray           # First basis vector in circle plane
    v: np.ndarray           # Second basis vector in circle plane
    normal: np.ndarray      # Normal to circle plane (for 3D; [0,0,1] for 2D)


def compute_constraint_circle_2d(
    anchors: np.ndarray,
    fixed_anchor_idx: int,
    fixed_length: float
) -> ConstraintCircle:
    """
    Compute the constraint circle for 2D (Slideprinter) case.

    With 1 cable fixed, the effector moves on a circle centered at that anchor.

    Parameters:
        anchors: (N, 2) array of anchor positions
        fixed_anchor_idx: Index of the fixed anchor
        fixed_length: Length of the fixed cable

    Returns:
        ConstraintCircle in 2D
    """
    center = anchors[fixed_anchor_idx].copy()
    radius = fixed_length

    # Standard basis in 2D
    u = np.array([1.0, 0.0])
    v = np.array([0.0, 1.0])
    normal = np.array([0.0, 0.0, 1.0])  # z-axis

    return ConstraintCircle(center=center, radius=radius, u=u, v=v, normal=normal)


def compute_constraint_circle_3d(
    anchors: np.ndarray,
    fixed_anchor_indices: List[int],
    fixed_lengths: List[float]
) -> Optional[ConstraintCircle]:
    """
    Compute the constraint circle for 3D case.

    With 2 cables fixed, the effector moves on the intersection of two spheres.

    Parameters:
        anchors: (N, 3) array of anchor positions
        fixed_anchor_indices: Indices of the two fixed anchors [j, k]
        fixed_lengths: Lengths of the fixed cables [L_j, L_k]

    Returns:
        ConstraintCircle in 3D, or None if spheres don't intersect
    """
    if len(fixed_anchor_indices) != 2 or len(fixed_lengths) != 2:
        raise ValueError("3D constraint requires exactly 2 fixed anchors")

    j, k = fixed_anchor_indices
    L_j, L_k = fixed_lengths

    A_j = anchors[j]
    A_k = anchors[k]

    # Vector from A_j to A_k
    d_vec = A_k - A_j
    d = norm(d_vec)

    if d < 1e-10:
        return None  # Anchors coincident

    # Check if spheres intersect
    if d > L_j + L_k:
        return None  # Spheres don't touch
    if d < abs(L_j - L_k):
        return None  # One sphere inside the other

    # Distance from A_j along d_vec to the circle plane
    # Using: h = (d² + L_j² - L_k²) / (2d)
    h = (d**2 + L_j**2 - L_k**2) / (2 * d)

    # Circle radius
    r_sq = L_j**2 - h**2
    if r_sq < 0:
        return None  # Numerical issue
    radius = np.sqrt(r_sq)

    # Circle center
    d_hat = d_vec / d
    center = A_j + h * d_hat

    # Normal to the circle plane
    normal = d_hat

    # Build orthonormal basis for circle plane
    # Find a vector not parallel to normal
    if abs(normal[0]) < 0.9:
        temp = np.array([1.0, 0.0, 0.0])
    else:
        temp = np.array([0.0, 1.0, 0.0])

    u = np.cross(normal, temp)
    u = u / norm(u)
    v = np.cross(normal, u)

    return ConstraintCircle(center=center, radius=radius, u=u, v=v, normal=normal)


def squared_length_coefficients(
    circle: ConstraintCircle,
    anchor: np.ndarray
) -> Tuple[float, float, float]:
    """
    Compute the coefficients K, M, N for L²(φ) = K + M·cos(φ) + N·sin(φ).

    Parameters:
        circle: The constraint circle
        anchor: Position of the anchor to compute distance to

    Returns:
        (K, M, N) coefficients
    """
    C = circle.center
    r = circle.radius
    u = circle.u
    v = circle.v

    # For 2D, extend anchor to 2D if needed
    if len(anchor) == 2:
        anchor = anchor[:2]
        C = C[:2]
        u = u[:2]
        v = v[:2]

    # Vector from anchor to circle center
    delta = C - anchor

    # K = ||delta||² + r²
    K = np.dot(delta, delta) + r**2

    # M = 2r * (u · delta)
    M = 2 * r * np.dot(u, delta)

    # N = 2r * (v · delta)
    N = 2 * r * np.dot(v, delta)

    return K, M, N


def parametric_to_algebraic_ellipse(
    K_d: float, M_d: float, N_d: float,  # Drive coefficients
    K_s: float, M_s: float, N_s: float   # Sensor coefficients
) -> np.ndarray:
    """
    Convert parametric ellipse representation to algebraic form.

    Given:
        X(φ) = K_d + M_d·cos(φ) + N_d·sin(φ)
        Y(φ) = K_s + M_s·cos(φ) + N_s·sin(φ)

    Find coefficients [A, B, C, D, E, F] such that:
        Ax² + Bxy + Cy² + Dx + Ey + F = 0

    Returns:
        Array [A, B, C, D, E, F] normalized so ||[A,B,C]|| = 1
    """
    # Eliminate the parameter φ
    #
    # Let c = cos(φ), s = sin(φ), with c² + s² = 1
    #
    # X - K_d = M_d·c + N_d·s
    # Y - K_s = M_s·c + N_s·s
    #
    # Solving for c, s:
    # [M_d  N_d] [c]   [X - K_d]
    # [M_s  N_s] [s] = [Y - K_s]
    #
    # det = M_d·N_s - N_d·M_s

    det = M_d * N_s - N_d * M_s

    if abs(det) < 1e-10:
        # Degenerate case: the motion is essentially linear
        # Return a degenerate ellipse (line)
        return np.array([0, 0, 0, M_d, M_s, -K_d * M_d - K_s * M_s])

    # c = (N_s·(X - K_d) - N_d·(Y - K_s)) / det
    # s = (M_d·(Y - K_s) - M_s·(X - K_d)) / det
    #
    # Using c² + s² = 1:
    # (N_s·(X - K_d) - N_d·(Y - K_s))² + (M_d·(Y - K_s) - M_s·(X - K_d))² = det²

    # Expand:
    # Let x = X - K_d, y = Y - K_s
    # (N_s·x - N_d·y)² + (M_d·y - M_s·x)² = det²
    #
    # N_s²·x² - 2·N_s·N_d·x·y + N_d²·y² + M_s²·x² - 2·M_s·M_d·x·y + M_d²·y² = det²
    #
    # (N_s² + M_s²)·x² - 2·(N_s·N_d + M_s·M_d)·x·y + (N_d² + M_d²)·y² = det²

    # Coefficients in (x, y) space:
    A_xy = N_s**2 + M_s**2
    B_xy = -2 * (N_s * N_d + M_s * M_d)
    C_xy = N_d**2 + M_d**2
    F_xy = -det**2

    # Transform back to (X, Y) with X = x + K_d, Y = y + K_s
    # A_xy·x² + B_xy·x·y + C_xy·y² + F_xy = 0
    #
    # A_xy·(X-K_d)² + B_xy·(X-K_d)·(Y-K_s) + C_xy·(Y-K_s)² + F_xy = 0
    #
    # Expanding:
    A = A_xy
    B = B_xy
    C = C_xy
    D = -2 * A_xy * K_d - B_xy * K_s
    E = -2 * C_xy * K_s - B_xy * K_d
    F = A_xy * K_d**2 + B_xy * K_d * K_s + C_xy * K_s**2 + F_xy

    coeffs = np.array([A, B, C, D, E, F], dtype=float)

    # Normalize
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
    dimensions: int = 3
) -> Optional[np.ndarray]:
    """
    Predict the ellipse coefficients for a sweep configuration.

    Parameters:
        anchors: (N, D) array of anchor positions
        fixed_anchor_indices: Indices of fixed anchors
        fixed_lengths: Lengths of fixed cables
        drive_anchor_idx: Index of drive anchor
        sensor_anchor_idx: Index of sensor anchor
        dimensions: 2 for Slideprinter, 3 for others

    Returns:
        Array [A, B, C, D, E, F] or None if configuration is invalid
    """
    # Compute the constraint circle
    if dimensions == 2:
        if len(fixed_anchor_indices) != 1:
            raise ValueError("2D case requires exactly 1 fixed anchor")
        circle = compute_constraint_circle_2d(
            anchors, fixed_anchor_indices[0], fixed_lengths[0]
        )
    else:
        if len(fixed_anchor_indices) != 2:
            raise ValueError("3D case requires exactly 2 fixed anchors")
        circle = compute_constraint_circle_3d(
            anchors, fixed_anchor_indices, fixed_lengths
        )

    if circle is None:
        return None

    # Get coefficients for drive and sensor cables
    K_d, M_d, N_d = squared_length_coefficients(circle, anchors[drive_anchor_idx])
    K_s, M_s, N_s = squared_length_coefficients(circle, anchors[sensor_anchor_idx])

    # Convert to algebraic ellipse
    coeffs = parametric_to_algebraic_ellipse(K_d, M_d, N_d, K_s, M_s, N_s)

    return coeffs


def predict_ellipses_for_dataset(
    anchors: np.ndarray,
    dataset: dict
) -> List[dict]:
    """
    Predict ellipse coefficients for all sweeps in a dataset.

    Parameters:
        anchors: Current anchor position estimates (N, D)
        dataset: Sweep dataset dictionary

    Returns:
        List of predicted ellipse dicts matching sweep IDs
    """
    dimensions = dataset.get('dimensions', 3)
    predictions = []

    for sweep in dataset.get('sweeps', []):
        sweep_id = sweep['id']
        fixed_indices = sweep['fixed_anchors']
        fixed_lengths = sweep['fixed_lengths']
        drive_idx = sweep['drive_anchor']
        sensor_idx = sweep['sensor_anchor']

        coeffs = predict_ellipse_coefficients(
            anchors, fixed_indices, fixed_lengths,
            drive_idx, sensor_idx, dimensions
        )

        if coeffs is not None:
            predictions.append({
                'sweep_id': sweep_id,
                'coefficients': {
                    'A': coeffs[0],
                    'B': coeffs[1],
                    'C': coeffs[2],
                    'D': coeffs[3],
                    'E': coeffs[4],
                    'F': coeffs[5],
                },
                'valid': True
            })
        else:
            predictions.append({
                'sweep_id': sweep_id,
                'coefficients': None,
                'valid': False
            })

    return predictions


# Utility functions for working with different machine types

def get_anchor_bounds(machine_type: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get reasonable bounds for anchor positions by machine type.

    Returns:
        (lower_bounds, upper_bounds) arrays of shape (N*D,)
    """
    bounds = {
        'slideprinter': {
            'n_anchors': 3,
            'dims': 2,
            'lb': [-2000, -2000] * 3,
            'ub': [2000, 2000] * 3,
        },
        'hangprinter_4': {
            'n_anchors': 4,
            'dims': 3,
            'lb': [-5000, -5000, -2000] * 4,
            'ub': [5000, 5000, 5000] * 4,
        },
        'hangprinter_5': {
            'n_anchors': 5,
            'dims': 3,
            'lb': [-5000, -5000, -2000] * 5,
            'ub': [5000, 5000, 5000] * 5,
        },
        'cubecorners': {
            'n_anchors': 8,
            'dims': 3,
            'lb': [-5000, -5000, -2000] * 8,
            'ub': [5000, 5000, 5000] * 8,
        },
        'skycam': {
            'n_anchors': 4,
            'dims': 3,
            'lb': [-10000, -10000, 2000] * 4,  # All anchors high
            'ub': [10000, 10000, 10000] * 4,
        },
    }

    cfg = bounds.get(machine_type)
    if cfg is None:
        raise ValueError(f"Unknown machine type: {machine_type}")

    return np.array(cfg['lb']), np.array(cfg['ub'])


def anchors_vec_to_matrix(vec: np.ndarray, n_anchors: int, dims: int) -> np.ndarray:
    """Convert flat anchor vector to (N, D) matrix."""
    return vec.reshape(n_anchors, dims)


def anchors_matrix_to_vec(matrix: np.ndarray) -> np.ndarray:
    """Convert (N, D) anchor matrix to flat vector."""
    return matrix.ravel()
```

### 4.3 Validation Against Synthetic Data

```python
"""
test_theoretical_ellipse.py

Verify that theoretical predictions match numerical simulation.
"""

import numpy as np
from theoretical_ellipse import (
    compute_constraint_circle_2d,
    compute_constraint_circle_3d,
    predict_ellipse_coefficients,
    squared_length_coefficients
)
from ellipse_fitting import fit_ellipse_from_sweep


def simulate_sweep(
    anchors: np.ndarray,
    fixed_indices: list,
    fixed_lengths: list,
    drive_idx: int,
    sensor_idx: int,
    n_points: int = 100,
    dimensions: int = 3
) -> tuple:
    """
    Numerically simulate a sweep by computing exact positions on the constraint circle.

    Returns:
        (l_drive, l_sensor) arrays
    """
    if dimensions == 2:
        from theoretical_ellipse import compute_constraint_circle_2d
        circle = compute_constraint_circle_2d(anchors, fixed_indices[0], fixed_lengths[0])
    else:
        from theoretical_ellipse import compute_constraint_circle_3d
        circle = compute_constraint_circle_3d(anchors, fixed_indices, fixed_lengths)

    if circle is None:
        return None, None

    # Generate points on circle
    phi = np.linspace(0, np.pi, n_points)  # Half circle for typical sweep

    l_drive = []
    l_sensor = []

    for p in phi:
        # Position on circle
        pos = circle.center + circle.radius * (np.cos(p) * circle.u + np.sin(p) * circle.v)

        # Cable lengths
        if dimensions == 2:
            l_d = np.linalg.norm(pos[:2] - anchors[drive_idx][:2])
            l_s = np.linalg.norm(pos[:2] - anchors[sensor_idx][:2])
        else:
            l_d = np.linalg.norm(pos - anchors[drive_idx])
            l_s = np.linalg.norm(pos - anchors[sensor_idx])

        l_drive.append(l_d)
        l_sensor.append(l_s)

    return np.array(l_drive), np.array(l_sensor)


def test_prediction_matches_simulation_2d():
    """Verify theoretical prediction matches simulation for 2D case."""
    # Set up Slideprinter-like geometry
    anchors = np.array([
        [-500, 400],
        [500, 400],
        [0, -500]
    ], dtype=float)

    fixed_indices = [0]
    fixed_lengths = [600.0]
    drive_idx = 1
    sensor_idx = 2

    # Simulate sweep
    l_drive, l_sensor = simulate_sweep(
        anchors, fixed_indices, fixed_lengths,
        drive_idx, sensor_idx, n_points=100, dimensions=2
    )

    # Fit ellipse to simulated data
    fit_result = fit_ellipse_from_sweep(l_drive, l_sensor)

    # Get theoretical prediction
    pred_coeffs = predict_ellipse_coefficients(
        anchors, fixed_indices, fixed_lengths,
        drive_idx, sensor_idx, dimensions=2
    )

    # Compare coefficients (both normalized to ||ABC||=1)
    fit_coeffs = fit_result.coefficients

    # Check they're parallel (allow sign flip)
    dot = np.abs(np.dot(fit_coeffs, pred_coeffs))
    norm_prod = np.linalg.norm(fit_coeffs) * np.linalg.norm(pred_coeffs)

    assert dot / norm_prod > 0.999, f"Coefficient mismatch: {fit_coeffs} vs {pred_coeffs}"


def test_prediction_matches_simulation_3d():
    """Verify theoretical prediction matches simulation for 3D case."""
    # Set up Hangprinter-like geometry
    anchors = np.array([
        [-1000, -500, -100],
        [1000, -500, -100],
        [0, 1000, -100],
        [0, 0, 2000]
    ], dtype=float)

    fixed_indices = [0, 1]
    fixed_lengths = [1200.0, 1200.0]
    drive_idx = 2
    sensor_idx = 3

    # Simulate sweep
    l_drive, l_sensor = simulate_sweep(
        anchors, fixed_indices, fixed_lengths,
        drive_idx, sensor_idx, n_points=100, dimensions=3
    )

    if l_drive is None:
        print("Configuration doesn't produce valid intersection")
        return

    # Fit ellipse to simulated data
    fit_result = fit_ellipse_from_sweep(l_drive, l_sensor)

    # Get theoretical prediction
    pred_coeffs = predict_ellipse_coefficients(
        anchors, fixed_indices, fixed_lengths,
        drive_idx, sensor_idx, dimensions=3
    )

    # Compare
    fit_coeffs = fit_result.coefficients
    dot = np.abs(np.dot(fit_coeffs, pred_coeffs))
    norm_prod = np.linalg.norm(fit_coeffs) * np.linalg.norm(pred_coeffs)

    assert dot / norm_prod > 0.999, f"Coefficient mismatch: {fit_coeffs} vs {pred_coeffs}"


if __name__ == '__main__':
    test_prediction_matches_simulation_2d()
    print("2D test passed")

    test_prediction_matches_simulation_3d()
    print("3D test passed")
```

## Testing

### Unit Tests

```python
# test_theoretical_ellipse.py
import pytest
import numpy as np
from theoretical_ellipse import (
    compute_constraint_circle_2d,
    compute_constraint_circle_3d,
    squared_length_coefficients,
    parametric_to_algebraic_ellipse,
    predict_ellipse_coefficients
)


class TestConstraintCircle2D:
    def test_basic_circle(self):
        anchors = np.array([[0, 0], [100, 0], [50, 86.6]])
        circle = compute_constraint_circle_2d(anchors, 0, 50.0)

        assert np.allclose(circle.center, [0, 0])
        assert circle.radius == 50.0
        assert np.allclose(circle.u, [1, 0])
        assert np.allclose(circle.v, [0, 1])


class TestConstraintCircle3D:
    def test_intersecting_spheres(self):
        anchors = np.array([
            [0, 0, 0],
            [100, 0, 0],
            [50, 50, 0],
            [50, 0, 50]
        ])

        circle = compute_constraint_circle_3d(anchors, [0, 1], [60.0, 60.0])

        assert circle is not None
        # Center should be at x=50 (midpoint)
        assert abs(circle.center[0] - 50) < 1e-10
        # Normal should be along x-axis
        assert abs(abs(circle.normal[0]) - 1) < 1e-10

    def test_non_intersecting_spheres(self):
        anchors = np.array([
            [0, 0, 0],
            [200, 0, 0],
            [100, 100, 0],
            [100, 0, 100]
        ])

        circle = compute_constraint_circle_3d(anchors, [0, 1], [50.0, 50.0])
        assert circle is None  # Spheres too far apart


class TestSquaredLengthCoefficients:
    def test_consistency(self):
        """Verify K, M, N produce correct L² for known points."""
        anchors = np.array([[100, 0], [0, 100], [-100, 0]])
        circle = compute_constraint_circle_2d(anchors, 0, 50.0)

        K, M, N = squared_length_coefficients(circle, anchors[1])

        # Check at φ=0 and φ=π/2
        # At φ=0: point is at (100+50, 0) = (150, 0)
        # Distance to anchor[1]=(0,100) is sqrt(150² + 100²)
        L_sq_0 = K + M * 1 + N * 0
        expected_0 = 150**2 + 100**2
        assert abs(L_sq_0 - expected_0) < 1e-6

        # At φ=π/2: point is at (100, 50)
        # Distance to anchor[1]=(0,100) is sqrt(100² + 50²)
        L_sq_90 = K + M * 0 + N * 1
        expected_90 = 100**2 + 50**2
        assert abs(L_sq_90 - expected_90) < 1e-6


class TestParametricToAlgebraic:
    def test_known_ellipse(self):
        """Test conversion with a simple case."""
        # If X = 100 + 20*cos(φ) and Y = 80 + 15*cos(φ)
        # (same sin coefficient = 0), this is a degenerate line segment
        K_d, M_d, N_d = 100, 20, 0
        K_s, M_s, N_s = 80, 15, 0

        coeffs = parametric_to_algebraic_ellipse(K_d, M_d, N_d, K_s, M_s, N_s)

        # Should be degenerate (line)
        # Check B² - 4AC ≥ 0 (not an ellipse)
        A, B, C = coeffs[:3]
        discriminant = B**2 - 4*A*C
        # Degenerate case
        assert discriminant >= -1e-10

    def test_proper_ellipse(self):
        """Test conversion with non-degenerate case."""
        K_d, M_d, N_d = 10000, 2000, 1500
        K_s, M_s, N_s = 12000, 1800, -1200

        coeffs = parametric_to_algebraic_ellipse(K_d, M_d, N_d, K_s, M_s, N_s)

        # Check it's an ellipse (B² - 4AC < 0)
        A, B, C = coeffs[:3]
        discriminant = B**2 - 4*A*C
        assert discriminant < 0
```

## Validation Criteria

1. **Circle Computation**: Constraint circles match expected geometry
2. **Coefficient Consistency**: L² values at known angles match K + M·cos + N·sin
3. **Algebraic Conversion**: Parametric-to-algebraic produces valid ellipse (B²-4AC<0)
4. **Prediction-Simulation Match**: Theoretical coefficients match fitted coefficients from simulation to within 0.1%
5. **Boundary Cases**: Graceful handling of non-intersecting spheres, degenerate configurations

## Dependencies

- Python 3.8+
- numpy
- (ellipse_fitting.py from Substep 3 for validation)

## Estimated Complexity

**Effort**: High (4-6 hours)

The sphere-sphere intersection geometry requires careful implementation. The parametric-to-algebraic conversion has subtle numerical issues. Edge cases (degenerate configurations, near-parallel spheres) need handling.

## Files to Create/Modify

| File | Action |
|------|--------|
| `autocal/theoretical_ellipse.py` | Create |
| `autocal/tests/test_theoretical_ellipse.py` | Create |
