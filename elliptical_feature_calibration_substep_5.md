# Substep 5: Feature Cost Function

## Overview

Implement the cost function that, for each anchor guess, reconstructs absolute sweep lengths, fits ellipses on those absolute lengths, and compares the fitted **geometric** parameters `(x0, y0, a, b, θ)` (canonicalized with `a >= b` and θ wrapped) to theoretical ellipse geometry from the same anchor guess. This is the core of Phase 2 optimization—finding anchor positions that minimize the discrepancy between predicted and measured ellipses without suffering from the scale/conditioning ambiguity of raw `(A,B,C,D,E,F)` coefficients. No pre-fit ellipses are used; everything is fit per iteration. Residual thresholds/weights should use the Sampson (approximate geometric) distance produced by the fitter, not the raw algebraic residual.

Remember the data-collection constraint: all recorded lengths are deltas from the origin because anchors were unknown at logging time. When evaluating the cost, reconstruct absolute lengths from the current anchor guess (`L_abs = ||A_i - origin|| + ΔL_measured`), fit ellipses from those absolute lengths, and only then compare to theoretical ellipses so both sides live in the same coordinate system. The optimizer ignores any stored ellipse fits and re-fits per iteration.

## Implementation Details

### 5.1 Cost Function Design

Inputs carry sweep configurations (fixed anchors, held lengths, drive/sense roles). Per-iteration fits should attach the same snapshot so the forward model compares like with like.

The cost function computes:
$$\text{Cost} = \sum_{\text{sweeps}} w_i \cdot d\Big((x_0, y_0, a, b, \theta)_{\text{obs}}^{(i)}, (x_0, y_0, a, b, \theta)_{\text{pred}}^{(i)}\Big)^2$$

where:
- Observed tuples come from the per-iteration ellipse fitting (canonicalized with `a >= b`, θ in a stable interval such as [-π/2, π/2])
- Predicted tuples come from the theoretical projection (also canonicalized)
- $d(\cdot, \cdot)$ is a geometry-aware distance (see below)
- $w_i$ is a weight (inverse of fit residual, or uniform)
- Per-sweep QC should also retain per-point residuals and sampling density (e.g. histogram by drive delta or fitted φ) to expose where noise or sparse coverage might weaken observability.

### 5.2 Geometry Distance Metrics

Use geometry-aware comparisons instead of raw coefficients:

1. **Canonical Euclidean** (default): Weighted Euclidean distance in `(x0, y0, a, b, θ)` after wrapping θ to the shortest difference.
2. **Axis-Weighted**: Emphasize shape `(a, b, θ)` over translation `(x0, y0)` if desired.
3. **Hybrid Debug**: Keep `(A..F)` alongside geometry for diagnostics, but never drive the optimizer directly with them.

### 5.3 Core Implementation (`ellipse_cost.py`)

```python
"""
ellipse_cost.py

Cost function for elliptical feature calibration.
"""

import numpy as np
from numpy.linalg import norm
from typing import List, Dict, Tuple, Optional, Callable
from dataclasses import dataclass
import warnings

from theoretical_ellipse import (
    predict_ellipse_geometry,
    anchors_vec_to_matrix,
    anchors_matrix_to_vec
)


@dataclass
class CostResult:
    """Result of cost function evaluation."""
    total_cost: float
    per_sweep_costs: Dict[str, float]
    num_valid_sweeps: int
    num_invalid_sweeps: int
    anchor_estimate: np.ndarray


def canonicalize_geometry(center: Tuple[float, float], semi_axes: Tuple[float, float], theta: float) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Canonicalize geometry to (x0, y0, a >= b, θ in [-pi/2, pi/2]).
    """
    x0, y0 = center
    a, b = semi_axes
    theta_wrapped = ((theta + np.pi/2) % np.pi) - np.pi/2  # wrap to [-pi/2, pi/2]

    if a < b:
        a, b = b, a
        theta_wrapped += np.sign(theta_wrapped) * np.pi/2 if theta_wrapped != 0 else np.pi/2
        theta_wrapped = ((theta_wrapped + np.pi/2) % np.pi) - np.pi/2

    return np.array([x0, y0]), np.array([a, b]), theta_wrapped


def geometry_distance(
    obs_center: np.ndarray,
    obs_axes: np.ndarray,
    obs_theta: float,
    pred_center: np.ndarray,
    pred_axes: np.ndarray,
    pred_theta: float,
    weights: Tuple[float, float, float] = (1.0, 1.0, 0.2)
) -> float:
    """
    Weighted Euclidean distance in canonical geometry space.

    weights = (w_center, w_axes, w_theta) controls emphasis.
    """
    w_center, w_axes, w_theta = weights

    delta_center = obs_center - pred_center
    delta_axes = obs_axes - pred_axes

    # Smallest angular difference
    delta_theta = pred_theta - obs_theta
    delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi  # wrap to [-π, π]

    return np.sqrt(
        w_center * np.dot(delta_center, delta_center) +
        w_axes * np.dot(delta_axes, delta_axes) +
        w_theta * (delta_theta ** 2)
    )


class EllipseCostFunction:
    """
    Cost function for elliptical feature calibration.

    Evaluates how well a set of anchor positions explains the observed ellipses.
    """

    def __init__(
        self,
        dataset: dict,
        geometry_weights: Tuple[float, float, float] = (1.0, 1.0, 0.2),
        residual_threshold: float = 0.01,
        use_weights: bool = True,
        invalid_sweep_penalty: float = 1000.0
    ):
        """
        Initialize cost function.

        Parameters:
            dataset: Sweep dataset (sweeps only; lengths stored as deltas)
            geometry_weights: Tuple weighting (center, axes, theta) in geometry_distance
            residual_threshold: QC threshold passed to fit_ellipse_from_sweep
            use_weights: Weight sweeps by inverse residual
            invalid_sweep_penalty: Penalty for sweeps that can't be predicted
        """
        self.dataset = dataset
        self.machine_type = dataset.get('machine_type', 'hangprinter_4')
        self.dimensions = dataset.get('dimensions', 3)
        self.num_anchors = dataset.get('num_anchors', 4)

        self.invalid_penalty = invalid_sweep_penalty
        self.geometry_weights = geometry_weights
        self.residual_threshold = residual_threshold
        self.use_weights = use_weights

        self.sweeps = dataset.get('sweeps', [])

    def _reconstruct_lengths(
        self,
        sweep: dict,
        anchors: np.ndarray
    ) -> tuple[list[float], np.ndarray, np.ndarray]:
        """Add anchor-baseline lengths to the stored encoder deltas."""
        drive_idx = sweep['drive_anchor']
        sensor_idx = sweep['sensor_anchor']
        fixed_indices = sweep['fixed_anchors']

        l_drive = np.array([p['l_drive'] for p in sweep['data_points']], dtype=float)
        l_sensor = np.array([p['l_sensor'] for p in sweep['data_points']], dtype=float)

        fixed_lengths_abs = [
            np.linalg.norm(anchors[idx]) + dl
            for idx, dl in zip(fixed_indices, sweep['fixed_lengths'])
        ]
        l_drive_abs = l_drive + np.linalg.norm(anchors[drive_idx])
        l_sensor_abs = l_sensor + np.linalg.norm(anchors[sensor_idx])

        return fixed_lengths_abs, l_drive_abs, l_sensor_abs

    def _fit_observed_geometry(
        self,
        sweep: dict,
        anchors: np.ndarray
    ) -> tuple[Optional[Tuple[np.ndarray, np.ndarray, float]], float, list[float]]:
        """
        Fit an ellipse for a sweep given the current anchor guess.

        Returns:
            (canonical_geom or None, weight, fixed_lengths_abs)
        """
        fixed_lengths_abs, l_drive_abs, l_sensor_abs = self._reconstruct_lengths(sweep, anchors)
        fit = fit_ellipse_from_sweep(
            l_drive_abs,
            l_sensor_abs,
            residual_threshold=self.residual_threshold
        )

        weight = 1.0
        if self.use_weights and np.isfinite(fit.residual_rms):
            weight = 1.0 / max(fit.residual_rms, 0.001)

        if not fit.valid:
            return None, weight, fixed_lengths_abs

        geom = canonicalize_geometry(
            fit.center,
            fit.semi_axes,
            fit.rotation_angle
        )
        return geom, weight, fixed_lengths_abs

    def evaluate(self, anchor_vec: np.ndarray) -> float:
        """
        Evaluate cost for a given anchor configuration.

        Parameters:
            anchor_vec: Flat array of anchor positions (N*D,)

        Returns:
            Total cost (lower is better)
        """
        anchors = anchors_vec_to_matrix(
            anchor_vec, self.num_anchors, self.dimensions
        )

        weighted_costs = []
        weights = []

        for sweep in self.sweeps:
            obs_geom, weight, fixed_lengths_abs = self._fit_observed_geometry(
                sweep, anchors
            )
            sweep_id = sweep.get('id', '')
            weights.append(weight)

            # Predict ellipse for this configuration
            pred_geom = predict_ellipse_geometry(
                anchors,
                sweep['fixed_anchors'],
                fixed_lengths_abs,
                sweep['drive_anchor'],
                sweep['sensor_anchor'],
                self.dimensions
            )

            if obs_geom is None or pred_geom is None:
                weighted_costs.append(self.invalid_penalty)
                continue

            pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
            obs_center, obs_axes, obs_theta = obs_geom

            dist = geometry_distance(
                obs_center, obs_axes, obs_theta,
                pred_center, pred_axes, pred_theta,
                self.geometry_weights
            )
            weighted_costs.append(dist**2)

        # Normalize by total weight to keep scale consistent across sweeps
        weight_sum = sum(weights) or 1.0
        total_cost = sum(
            wc * (w / weight_sum) for wc, w in zip(weighted_costs, weights)
        )

        return total_cost

    def evaluate_detailed(self, anchor_vec: np.ndarray) -> CostResult:
        """
        Evaluate cost with detailed breakdown.
        """
        anchors = anchors_vec_to_matrix(
            anchor_vec, self.num_anchors, self.dimensions
        )

        per_sweep_costs = {}
        num_valid = 0
        num_invalid = 0
        weights = {}

        for sweep in self.sweeps:
            sweep_id = sweep.get('id', '')
            obs_geom, weight, fixed_lengths_abs = self._fit_observed_geometry(
                sweep, anchors
            )
            weights[sweep_id] = weight

            pred_geom = predict_ellipse_geometry(
                anchors,
                sweep['fixed_anchors'],
                fixed_lengths_abs,
                sweep['drive_anchor'],
                sweep['sensor_anchor'],
                self.dimensions
            )

            if obs_geom is None or pred_geom is None:
                per_sweep_costs[sweep_id] = self.invalid_penalty
                num_invalid += 1
                continue

            pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
            obs_center, obs_axes, obs_theta = obs_geom
            dist = geometry_distance(
                obs_center, obs_axes, obs_theta,
                pred_center, pred_axes, pred_theta,
                self.geometry_weights
            )
            per_sweep_costs[sweep_id] = dist**2
            num_valid += 1

        weight_sum = sum(weights.values()) or 1.0
        total_cost = sum(
            per_sweep_costs[sid] * (weights.get(sid, 1.0) / weight_sum)
            for sid in per_sweep_costs
        )

        return CostResult(
            total_cost=total_cost,
            per_sweep_costs=per_sweep_costs,
            num_valid_sweeps=num_valid,
            num_invalid_sweeps=num_invalid,
            anchor_estimate=anchors
        )

    def gradient_numerical(
        self,
        anchor_vec: np.ndarray,
        epsilon: float = 1e-6
    ) -> np.ndarray:
        """
        Compute numerical gradient of cost function.
        """
        grad = np.zeros_like(anchor_vec)
        f0 = self.evaluate(anchor_vec)

        for i in range(len(anchor_vec)):
            x_plus = anchor_vec.copy()
            x_plus[i] += epsilon
            f_plus = self.evaluate(x_plus)
            grad[i] = (f_plus - f0) / epsilon

        return grad


def create_optimization_objective(
    dataset: dict,
    **cost_kwargs
) -> Tuple[Callable, Callable]:
    """
    Create objective function and gradient for scipy.optimize.

    Returns:
        (objective_fn, gradient_fn) suitable for scipy.optimize.minimize
    """
    cost_fn = EllipseCostFunction(dataset, **cost_kwargs)

    def objective(x):
        return cost_fn.evaluate(x)

    def gradient(x):
        return cost_fn.gradient_numerical(x)

    return objective, gradient


# Integration with existing calibration framework

def combined_cost_function(
    anchor_vec: np.ndarray,
    ellipse_cost_fn: EllipseCostFunction,
    point_cost_fn: Optional[Callable] = None,
    ellipse_weight: float = 1.0,
    point_weight: float = 0.1
) -> float:
    """
    Combined cost function using both ellipse features and point measurements.

    Allows gradual transition from existing point-based calibration.
    """
    cost = ellipse_weight * ellipse_cost_fn.evaluate(anchor_vec)

    if point_cost_fn is not None:
        cost += point_weight * point_cost_fn(anchor_vec)

    return cost


# Regularization terms

def anchor_regularity_penalty(
    anchor_vec: np.ndarray,
    num_anchors: int,
    dimensions: int,
    target_symmetry: str = 'none'
) -> float:
    """
    Regularization penalty for anchor configurations.

    Encourages plausible anchor arrangements.
    """
    anchors = anchors_vec_to_matrix(anchor_vec, num_anchors, dimensions)

    penalty = 0.0

    # Penalty for anchors too close together
    min_dist = 100.0  # mm
    for i in range(num_anchors):
        for j in range(i+1, num_anchors):
            dist = norm(anchors[i] - anchors[j])
            if dist < min_dist:
                penalty += (min_dist - dist)**2

    # Optional: encourage symmetric arrangements
    if target_symmetry == 'triangular' and num_anchors >= 3:
        # Penalize deviation from 120° spacing in XY
        pass  # Implementation depends on specific geometry

    return penalty
```

### 5.4 Solver Integration (`ellipse_solver.py`)

```python
"""
ellipse_solver.py

Solver for elliptical feature calibration.
"""

import numpy as np
from scipy.optimize import minimize, differential_evolution
from typing import Dict, Optional, Tuple
import warnings
import concurrent.futures

from ellipse_cost import EllipseCostFunction, CostResult
from theoretical_ellipse import get_anchor_bounds, anchors_matrix_to_vec, anchors_vec_to_matrix


def solve_anchors(
    dataset: dict,
    initial_guess: Optional[np.ndarray] = None,
    method: str = 'SLSQP',
    max_iterations: int = 1000,
    num_restarts: int = 8,
    verbose: bool = False
) -> Dict:
    """
    Find anchor positions that best explain the observed ellipses.

    Parameters:
        dataset: Sweep dataset with fitted ellipses
        initial_guess: Initial anchor positions (N*D,) or None for random
        method: Optimization method ('SLSQP', 'L-BFGS-B', 'differential_evolution')
        max_iterations: Maximum iterations per restart
        num_restarts: Number of random restarts
        verbose: Print progress

    Returns:
        Dict with 'anchors', 'cost', 'success', 'details'
    """
    machine_type = dataset.get('machine_type', 'hangprinter_4')
    num_anchors = dataset.get('num_anchors', 4)
    dimensions = dataset.get('dimensions', 3)

    # Get bounds
    lb, ub = get_anchor_bounds(machine_type)
    bounds = list(zip(lb, ub))

    # Create cost function
    cost_fn = EllipseCostFunction(dataset, geometry_weights=(1.0, 1.0, 0.2), use_weights=True)

    if verbose:
        print(f"Solving for {num_anchors} anchors in {dimensions}D")
        print(f"Using {len(cost_fn.observed_ellipses)} valid ellipse observations")

    # Generate initial guesses
    if initial_guess is not None:
        initial_guesses = [initial_guess]
    else:
        initial_guesses = []

    # Add random initial guesses
    while len(initial_guesses) < num_restarts:
        random_guess = np.array([
            np.random.uniform(l, u) for l, u in bounds
        ])
        initial_guesses.append(random_guess)

    best_result = None
    best_cost = np.inf

    def optimize_single(x0):
        """Run optimization from single starting point."""
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore')

            if method == 'differential_evolution':
                result = differential_evolution(
                    cost_fn.evaluate,
                    bounds,
                    maxiter=max_iterations,
                    seed=42,
                    polish=True
                )
            else:
                result = minimize(
                    cost_fn.evaluate,
                    x0,
                    method=method,
                    bounds=bounds,
                    options={
                        'maxiter': max_iterations,
                        'ftol': 1e-9,
                        'disp': False
                    }
                )

        return result

    # Run optimizations in parallel
    with concurrent.futures.ProcessPoolExecutor() as executor:
        futures = [executor.submit(optimize_single, x0) for x0 in initial_guesses]
        results = [f.result() for f in concurrent.futures.as_completed(futures)]

    # Find best result
    for result in results:
        if result.fun < best_cost:
            best_cost = result.fun
            best_result = result

    if best_result is None:
        return {
            'anchors': None,
            'cost': np.inf,
            'success': False,
            'details': 'All optimizations failed'
        }

    # Get detailed cost breakdown
    final_eval = cost_fn.evaluate_detailed(best_result.x)
    anchors_matrix = anchors_vec_to_matrix(best_result.x, num_anchors, dimensions)

    if verbose:
        print(f"\nOptimization complete:")
        print(f"  Final cost: {best_cost:.6e}")
        print(f"  Valid sweeps: {final_eval.num_valid_sweeps}")
        print(f"  Invalid sweeps: {final_eval.num_invalid_sweeps}")
        print(f"\nAnchors:")
        for i, anchor in enumerate(anchors_matrix):
            print(f"  {i}: {anchor}")

    return {
        'anchors': anchors_matrix,
        'cost': best_cost,
        'success': best_result.success,
        'details': final_eval,
        'raw_result': best_result
    }


def format_anchors_gcode(anchors: np.ndarray, machine_type: str) -> str:
    """
    Format anchor positions as G-code M669 command.
    """
    if machine_type in ['hangprinter_4', 'hangprinter_5']:
        labels = ['A', 'B', 'C', 'D', 'I'][:len(anchors)]
        parts = [
            f"{label}{anchors[i, 0]:.2f}:{anchors[i, 1]:.2f}:{anchors[i, 2]:.2f}"
            for i, label in enumerate(labels)
        ]
        return f"M669 {' '.join(parts)}"
    elif machine_type == 'slideprinter':
        # Different format for Slideprinter
        return f"; Anchors: {anchors.tolist()}"
    else:
        return f"; Anchors for {machine_type}: {anchors.tolist()}"
```

## Testing

### Unit Tests

```python
# test_ellipse_cost.py
import pytest
import numpy as np
from ellipse_cost import (
    canonicalize_geometry,
    geometry_distance,
    EllipseCostFunction,
)
from ellipse_solver import solve_anchors


class TestGeometryCanonicalization:
    def test_axes_ordering_and_angle_wrap(self):
        center_can, axes_can, theta_can = canonicalize_geometry(
            (10.0, -5.0),
            (5.0, 10.0),
            3 * np.pi / 4,  # 135°
        )
        assert axes_can[0] >= axes_can[1]
        assert -np.pi/2 <= theta_can <= np.pi/2

    def test_identity_distance(self):
        obs_c = np.array([0.0, 0.0])
        obs_axes = np.array([10.0, 8.0])
        obs_theta = 0.1
        assert geometry_distance(obs_c, obs_axes, obs_theta, obs_c, obs_axes, obs_theta) == 0.0

    def test_geometry_distance_changes(self):
        obs_c = np.array([0.0, 0.0])
        obs_axes = np.array([10.0, 8.0])
        obs_theta = 0.0
        pred_c = np.array([1.0, 0.0])
        pred_axes = np.array([11.0, 8.0])
        pred_theta = 0.05
        assert geometry_distance(obs_c, obs_axes, obs_theta, pred_c, pred_axes, pred_theta) > 0


class TestCostFunction:
    @pytest.fixture
    def synthetic_dataset(self):
        """Create a synthetic dataset with known anchors."""
        # Known anchor positions (Slideprinter)
        true_anchors = np.array([
            [-500, 400],
            [500, 400],
            [0, -500]
        ])

        # Generate synthetic sweep deltas (absolute lengths rebuilt with anchor norms)
        phi = np.linspace(0, np.pi, 40)
        baseline_drive = np.linalg.norm(true_anchors[1])
        baseline_sensor = np.linalg.norm(true_anchors[2])
        l_drive_delta = 20.0 * np.cos(phi)  # encoder deltas from origin
        l_sensor_delta = 15.0 * np.sin(phi)

        return {
            'version': '1.0',
            'machine_type': 'slideprinter',
            'num_anchors': 3,
            'dimensions': 2,
            'sweeps': [
                {
                    'id': 'sweep_001',
                    'fixed_anchors': [0],
                    'fixed_lengths': [0.0],  # delta; absolute = ||A0|| + delta
                    'drive_anchor': 1,
                    'sensor_anchor': 2,
                    'data_points': [
                        {'l_drive': float(ld), 'l_sensor': float(ls)}
                        for ld, ls in zip(l_drive_delta, l_sensor_delta)
                    ]
                }
            ],
            '_true_anchors': true_anchors
        }

    def test_cost_function_creation(self, synthetic_dataset):
        cost_fn = EllipseCostFunction(synthetic_dataset)
        assert len(cost_fn.sweeps) == 1

    def test_cost_at_true_anchors(self, synthetic_dataset):
        """Cost should be low at true anchor positions."""
        # This test would need proper synthetic data generation
        pass

    def test_cost_gradient(self, synthetic_dataset):
        cost_fn = EllipseCostFunction(synthetic_dataset)
        x = np.zeros(6)  # Flat anchor vector

        grad = cost_fn.gradient_numerical(x)
        assert len(grad) == 6
        assert np.all(np.isfinite(grad))


class TestSolver:
    def test_synthetic_solve(self):
        """Solve with synthetic data and verify recovery."""
        # Generate synthetic dataset
        # ... (would need full simulation)
        pass
```

## Validation Criteria

1. **Zero Cost at True Anchors**: Synthetic data with known anchors gives cost ≈ 0
2. **Monotonic Cost Increase**: Perturbing anchors from true positions increases cost
3. **Geometry Canonicalization**: Swapping axes or adding ±π to θ leaves the cost unchanged after canonicalization
4. **Angle Wrapping**: Cost uses the shortest angular difference (no discontinuities at ±π)
5. **Gradient Consistency**: Numerical gradient matches finite differences
6. **Solver Convergence**: Solver finds near-optimal solution from random starts

## Dependencies

- Python 3.8+
- numpy
- scipy
- (theoretical_ellipse.py from Substep 4)
- (ellipse_fitting.py from Substep 3)

## Estimated Complexity

**Effort**: Medium-High (4-5 hours)

The cost function itself is straightforward, but ensuring numerical stability, proper normalization, and gradient computation requires care. Integration with scipy.optimize and testing with synthetic data adds complexity.

## Files to Create/Modify

| File | Action |
|------|--------|
| `autocal/ellipse_cost.py` | Create |
| `autocal/ellipse_solver.py` | Create |
| `autocal/tests/test_ellipse_cost.py` | Create |
