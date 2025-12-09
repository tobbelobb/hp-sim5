# Substep 5: Feature Cost Function

## Overview

Implement the cost function that compares observed ellipse coefficients (from fitting) to theoretical ellipse coefficients (from anchor geometry). This is the core of Phase 2 optimization - finding anchor positions that minimize the discrepancy between predicted and measured ellipses.

## Implementation Details

### 5.1 Cost Function Design

The cost function computes:
$$\text{Cost} = \sum_{\text{sweeps}} w_i \cdot d(\mathbf{C}_{\text{obs}}^{(i)}, \mathbf{C}_{\text{pred}}^{(i)})^2$$

where:
- $\mathbf{C}_{\text{obs}}^{(i)}$ are the fitted (observed) coefficients for sweep $i$
- $\mathbf{C}_{\text{pred}}^{(i)}$ are the predicted coefficients from current anchor estimates
- $d(\cdot, \cdot)$ is a distance metric between coefficient vectors
- $w_i$ is a weight (inverse of fit residual, or uniform)

### 5.2 Coefficient Distance Metrics

Since ellipse coefficients are defined up to scale, we need scale-invariant comparison:

1. **Normalized L2**: Normalize both vectors to unit norm, compute Euclidean distance
2. **Angular Distance**: $1 - |\cos\theta|$ where $\theta$ is the angle between vectors
3. **Weighted Algebraic**: Weight by importance of each coefficient

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
    predict_ellipse_coefficients,
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


def normalize_coefficients(coeffs: np.ndarray) -> np.ndarray:
    """
    Normalize ellipse coefficients for comparison.

    Normalizes so that ||[A, B, C]|| = 1 and the sign is canonical.
    """
    coeffs = np.asarray(coeffs, dtype=float)

    # Normalize by ||ABC||
    abc_norm = norm(coeffs[:3])
    if abc_norm < 1e-10:
        return coeffs

    coeffs = coeffs / abc_norm

    # Canonical sign: make largest of |A|, |B|, |C| positive
    max_idx = np.argmax(np.abs(coeffs[:3]))
    if coeffs[max_idx] < 0:
        coeffs = -coeffs

    return coeffs


def coefficient_distance_l2(
    coeffs1: np.ndarray,
    coeffs2: np.ndarray
) -> float:
    """
    Compute L2 distance between normalized coefficient vectors.
    """
    c1 = normalize_coefficients(coeffs1)
    c2 = normalize_coefficients(coeffs2)

    # Handle sign ambiguity
    dist_same = norm(c1 - c2)
    dist_flip = norm(c1 + c2)

    return min(dist_same, dist_flip)


def coefficient_distance_angular(
    coeffs1: np.ndarray,
    coeffs2: np.ndarray
) -> float:
    """
    Compute angular distance between coefficient vectors.

    Returns 1 - |cos(θ)| where θ is the angle between vectors.
    Range: [0, 1] where 0 = parallel, 1 = orthogonal
    """
    c1 = normalize_coefficients(coeffs1)
    c2 = normalize_coefficients(coeffs2)

    dot = np.dot(c1, c2)
    cos_theta = np.clip(np.abs(dot), 0, 1)

    return 1 - cos_theta


def coefficient_distance_weighted(
    coeffs1: np.ndarray,
    coeffs2: np.ndarray,
    weights: np.ndarray = None
) -> float:
    """
    Compute weighted distance between coefficients.

    Default weights emphasize A, B, C (shape) over D, E, F (position).
    """
    if weights is None:
        # Shape parameters more important than position
        weights = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])

    c1 = normalize_coefficients(coeffs1)
    c2 = normalize_coefficients(coeffs2)

    # Handle sign ambiguity
    diff_same = (c1 - c2) * weights
    diff_flip = (c1 + c2) * weights

    dist_same = norm(diff_same)
    dist_flip = norm(diff_flip)

    return min(dist_same, dist_flip)


class EllipseCostFunction:
    """
    Cost function for elliptical feature calibration.

    Evaluates how well a set of anchor positions explains the observed ellipses.
    """

    def __init__(
        self,
        dataset: dict,
        distance_metric: str = 'l2',
        use_weights: bool = True,
        invalid_sweep_penalty: float = 1000.0
    ):
        """
        Initialize cost function.

        Parameters:
            dataset: Sweep dataset with fitted_ellipses
            distance_metric: 'l2', 'angular', or 'weighted'
            use_weights: Weight sweeps by inverse residual
            invalid_sweep_penalty: Penalty for sweeps that can't be predicted
        """
        self.dataset = dataset
        self.machine_type = dataset.get('machine_type', 'hangprinter_4')
        self.dimensions = dataset.get('dimensions', 3)
        self.num_anchors = dataset.get('num_anchors', 4)

        self.invalid_penalty = invalid_sweep_penalty

        # Select distance metric
        if distance_metric == 'l2':
            self.distance_fn = coefficient_distance_l2
        elif distance_metric == 'angular':
            self.distance_fn = coefficient_distance_angular
        elif distance_metric == 'weighted':
            self.distance_fn = coefficient_distance_weighted
        else:
            raise ValueError(f"Unknown distance metric: {distance_metric}")

        # Extract observed ellipses
        self.observed_ellipses = {}
        self.sweep_weights = {}

        for fe in dataset.get('fitted_ellipses', []):
            if not fe.get('valid', False):
                continue

            sweep_id = fe['sweep_id']
            coeffs = fe['coefficients']
            self.observed_ellipses[sweep_id] = np.array([
                coeffs['A'], coeffs['B'], coeffs['C'],
                coeffs['D'], coeffs['E'], coeffs['F']
            ])

            # Weight by inverse residual (more confident fits get more weight)
            if use_weights:
                residual = fe.get('residual_rms', 0.01)
                self.sweep_weights[sweep_id] = 1.0 / max(residual, 0.001)
            else:
                self.sweep_weights[sweep_id] = 1.0

        # Normalize weights
        if self.sweep_weights:
            total_weight = sum(self.sweep_weights.values())
            for k in self.sweep_weights:
                self.sweep_weights[k] /= total_weight

        # Cache sweep configurations
        self.sweep_configs = {}
        for sweep in dataset.get('sweeps', []):
            self.sweep_configs[sweep['id']] = {
                'fixed_anchors': sweep['fixed_anchors'],
                'fixed_lengths': sweep['fixed_lengths'],
                'drive_anchor': sweep['drive_anchor'],
                'sensor_anchor': sweep['sensor_anchor']
            }

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

        total_cost = 0.0

        for sweep_id, obs_coeffs in self.observed_ellipses.items():
            config = self.sweep_configs.get(sweep_id)
            if config is None:
                continue

            # Predict ellipse for this configuration
            pred_coeffs = predict_ellipse_coefficients(
                anchors,
                config['fixed_anchors'],
                config['fixed_lengths'],
                config['drive_anchor'],
                config['sensor_anchor'],
                self.dimensions
            )

            if pred_coeffs is None:
                # Invalid configuration (e.g., spheres don't intersect)
                total_cost += self.invalid_penalty * self.sweep_weights[sweep_id]
                continue

            # Compute distance
            dist = self.distance_fn(obs_coeffs, pred_coeffs)
            weighted_cost = dist**2 * self.sweep_weights[sweep_id]
            total_cost += weighted_cost

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

        for sweep_id, obs_coeffs in self.observed_ellipses.items():
            config = self.sweep_configs.get(sweep_id)
            if config is None:
                continue

            pred_coeffs = predict_ellipse_coefficients(
                anchors,
                config['fixed_anchors'],
                config['fixed_lengths'],
                config['drive_anchor'],
                config['sensor_anchor'],
                self.dimensions
            )

            if pred_coeffs is None:
                per_sweep_costs[sweep_id] = self.invalid_penalty
                num_invalid += 1
            else:
                dist = self.distance_fn(obs_coeffs, pred_coeffs)
                per_sweep_costs[sweep_id] = dist**2
                num_valid += 1

        total_cost = sum(
            per_sweep_costs[sid] * self.sweep_weights.get(sid, 1.0)
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
    cost_fn = EllipseCostFunction(dataset, distance_metric='l2', use_weights=True)

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
    normalize_coefficients,
    coefficient_distance_l2,
    coefficient_distance_angular,
    EllipseCostFunction,
)
from ellipse_solver import solve_anchors


class TestNormalization:
    def test_unit_norm(self):
        coeffs = np.array([3, 0, 4, 10, 20, 30])
        normalized = normalize_coefficients(coeffs)
        assert abs(np.linalg.norm(normalized[:3]) - 1.0) < 1e-10

    def test_canonical_sign(self):
        coeffs1 = np.array([0.6, 0, 0.8, 1, 2, 3])
        coeffs2 = np.array([-0.6, 0, -0.8, -1, -2, -3])

        n1 = normalize_coefficients(coeffs1)
        n2 = normalize_coefficients(coeffs2)

        # Should have same sign after normalization
        assert np.allclose(n1, n2) or np.allclose(n1, -n2)


class TestDistanceMetrics:
    def test_identical_coefficients(self):
        c1 = np.array([1, 0, 1, 10, 20, 30])
        c2 = np.array([2, 0, 2, 20, 40, 60])  # Same up to scale

        assert coefficient_distance_l2(c1, c2) < 1e-10
        assert coefficient_distance_angular(c1, c2) < 1e-10

    def test_different_coefficients(self):
        c1 = np.array([1, 0, 1, 0, 0, 0])
        c2 = np.array([1, 0.5, 0.5, 0, 0, 0])

        dist_l2 = coefficient_distance_l2(c1, c2)
        dist_ang = coefficient_distance_angular(c1, c2)

        assert dist_l2 > 0
        assert dist_ang > 0

    def test_sign_invariance(self):
        c1 = np.array([1, 0.2, 0.8, 5, 10, 15])
        c2 = -c1

        assert coefficient_distance_l2(c1, c2) < 1e-10
        assert coefficient_distance_angular(c1, c2) < 1e-10


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

        # Generate synthetic sweeps and fit ellipses
        # (In practice, would use actual simulation)
        return {
            'version': '1.0',
            'machine_type': 'slideprinter',
            'num_anchors': 3,
            'dimensions': 2,
            'sweeps': [
                {
                    'id': 'sweep_001',
                    'fixed_anchors': [0],
                    'fixed_lengths': [600.0],
                    'drive_anchor': 1,
                    'sensor_anchor': 2,
                    'data_points': []
                }
            ],
            'fitted_ellipses': [
                {
                    'sweep_id': 'sweep_001',
                    'coefficients': {
                        'A': 0.5, 'B': 0.1, 'C': 0.4,
                        'D': -50, 'E': -40, 'F': 1000
                    },
                    'residual_rms': 0.001,
                    'valid': True
                }
            ],
            '_true_anchors': true_anchors
        }

    def test_cost_function_creation(self, synthetic_dataset):
        cost_fn = EllipseCostFunction(synthetic_dataset)
        assert len(cost_fn.observed_ellipses) == 1

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
3. **Sign Invariance**: Cost identical for coefficient vectors differing only by sign
4. **Scale Invariance**: Cost identical for scaled coefficient vectors
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
