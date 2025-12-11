from __future__ import annotations

"""Cost function for ellipse-based feature calibration."""

from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

from autocal.ellipse_fitting import fit_ellipse_from_sweep
from autocal.sweep_types import MachineConfig, MachineType, Sweep
from autocal.theoretical_ellipse import (
    anchors_vec_to_matrix,
    predict_ellipse_geometry,
)


@dataclass
class CostResult:
    """Breakdown of a cost function evaluation."""

    total_cost: float
    per_sweep_costs: Dict[str, float]
    num_valid_sweeps: int
    num_invalid_sweeps: int
    anchor_estimate: np.ndarray


def canonicalize_geometry(
    center: Tuple[float, float], semi_axes: Tuple[float, float], theta: float
) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Canonicalize ellipse geometry to a stable representation.

    Ensures a >= b and wraps theta into [-pi/2, pi/2] so equivalent
    parameterizations compare consistently.
    """
    x0, y0 = center
    a, b = semi_axes
    theta_wrapped = ((float(theta) + np.pi / 2) % np.pi) - np.pi / 2

    if a < b:
        a, b = b, a
        theta_wrapped += np.pi / 2
        theta_wrapped = ((theta_wrapped + np.pi / 2) % np.pi) - np.pi / 2

    return np.array([x0, y0], dtype=float), np.array([a, b], dtype=float), theta_wrapped


def geometry_distance(
    obs_center: np.ndarray,
    obs_axes: np.ndarray,
    obs_theta: float,
    pred_center: np.ndarray,
    pred_axes: np.ndarray,
    pred_theta: float,
    weights: Tuple[float, float, float] = (1.0, 1.0, 0.2),
) -> float:
    """
    Weighted Euclidean distance in canonical geometry space.

    Weights order: (center, axes, theta).
    """
    w_center, w_axes, w_theta = weights

    delta_center = obs_center - pred_center
    delta_axes = obs_axes - pred_axes

    delta_theta = (obs_theta - pred_theta + np.pi) % (2 * np.pi) - np.pi

    return float(
        np.sqrt(
            w_center * np.dot(delta_center, delta_center)
            + w_axes * np.dot(delta_axes, delta_axes)
            + w_theta * (delta_theta**2)
        )
    )


def _dataset_metadata(
    dataset: Union[dict, "SweepDataset"]
) -> Tuple[str, int, int, Iterable[Union[Sweep, dict]]]:
    """Extract machine metadata and sweeps from either dict or SweepDataset."""
    if hasattr(dataset, "machine_config"):
        config: MachineConfig = dataset.machine_config  # type: ignore[assignment]
        machine_type = config.machine_type.value
        num_anchors = config.num_anchors
        dimensions = config.dimensions
        sweeps = dataset.sweeps  # type: ignore[assignment]
    else:
        machine_type_raw = dataset.get("machine_type", "hangprinter_4")  # type: ignore[index]
        machine_type = (
            machine_type_raw.value if isinstance(machine_type_raw, MachineType) else str(machine_type_raw)
        )
        num_anchors = int(dataset.get("num_anchors", 4))  # type: ignore[index]
        dimensions = int(dataset.get("dimensions", 3))  # type: ignore[index]
        sweeps = dataset.get("sweeps", [])  # type: ignore[index]

    return machine_type, num_anchors, dimensions, sweeps


class EllipseCostFunction:
    """Evaluate how well anchor guesses explain observed ellipses."""

    def __init__(
        self,
        dataset: Union[dict, "SweepDataset"],
        geometry_weights: Tuple[float, float, float] = (1.0, 1.0, 0.2),
        residual_threshold: float = 0.01,
        min_points: int = 10,
        use_weights: bool = True,
        invalid_sweep_penalty: float = 1000.0,
        weight_floor: float = 1e-3,
    ) -> None:
        (
            self.machine_type,
            self.num_anchors,
            self.dimensions,
            self.sweeps,
        ) = _dataset_metadata(dataset)

        self.geometry_weights = geometry_weights
        self.residual_threshold = residual_threshold
        self.min_points = min_points
        self.use_weights = use_weights
        self.invalid_penalty = invalid_sweep_penalty
        self.weight_floor = weight_floor

    @staticmethod
    def _extract_sweep_arrays(
        sweep: Union[Sweep, dict]
    ) -> Tuple[List[int], List[float], int, int, np.ndarray, np.ndarray, str]:
        """Extract sweep role metadata and drive/sensor arrays."""
        if isinstance(sweep, Sweep):
            fixed_indices = list(sweep.fixed_anchors)
            fixed_deltas = list(sweep.fixed_lengths)
            drive_idx = sweep.drive_anchor
            sensor_idx = sweep.sensor_anchor
            l_drive = np.array([p.l_drive for p in sweep.data_points], dtype=float)
            l_sensor = np.array([p.l_sensor for p in sweep.data_points], dtype=float)
            sweep_id = sweep.id
        else:
            fixed_indices = list(sweep.get("fixed_anchors", []))
            fixed_deltas = list(sweep.get("fixed_lengths", []))
            drive_idx = int(sweep.get("drive_anchor"))
            sensor_idx = int(sweep.get("sensor_anchor"))
            data_points = sweep.get("data_points", [])
            l_drive = np.array([p.get("l_drive", 0.0) for p in data_points], dtype=float)
            l_sensor = np.array([p.get("l_sensor", 0.0) for p in data_points], dtype=float)
            sweep_id = sweep.get("id", "")

        return fixed_indices, fixed_deltas, drive_idx, sensor_idx, l_drive, l_sensor, sweep_id

    def _reconstruct_lengths(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Tuple[List[float], np.ndarray, np.ndarray, str]:
        """Add anchor baselines to encoder deltas to get absolute lengths."""
        (
            fixed_indices,
            fixed_deltas,
            drive_idx,
            sensor_idx,
            l_drive,
            l_sensor,
            sweep_id,
        ) = self._extract_sweep_arrays(sweep)

        fixed_lengths_abs = [
            float(np.linalg.norm(anchors[idx]) + delta) for idx, delta in zip(fixed_indices, fixed_deltas)
        ]
        l_drive_abs = l_drive + np.linalg.norm(anchors[drive_idx])
        l_sensor_abs = l_sensor + np.linalg.norm(anchors[sensor_idx])

        return fixed_lengths_abs, l_drive_abs, l_sensor_abs, sweep_id

    def _fit_observed_geometry(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Tuple[Optional[Tuple[np.ndarray, np.ndarray, float]], float, List[float], str]:
        """
        Fit ellipse geometry for a sweep given the current anchor guess.

        Returns:
            (canonical_geometry or None, weight, fixed_lengths_abs, sweep_id)
        """
        fixed_lengths_abs, l_drive_abs, l_sensor_abs, sweep_id = self._reconstruct_lengths(sweep, anchors)

        fit = fit_ellipse_from_sweep(
            l_drive_abs,
            l_sensor_abs,
            residual_threshold=self.residual_threshold,
            min_points=self.min_points,
            square_inputs=True,
        )

        if self.use_weights and np.isfinite(fit.residual_rms):
            weight = 1.0 / max(fit.residual_rms, self.weight_floor)
        else:
            weight = 1.0

        if not fit.valid:
            return None, weight, fixed_lengths_abs, sweep_id

        geom = canonicalize_geometry(fit.center, fit.semi_axes, fit.rotation_angle)
        return geom, weight, fixed_lengths_abs, sweep_id

    def evaluate(self, anchor_vec: np.ndarray) -> float:
        """Compute scalar cost for a flat anchor vector."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)

        if not self.sweeps:
            return 0.0

        weighted_costs: List[float] = []
        weights: List[float] = []

        for sweep in self.sweeps:
            obs_geom, weight, fixed_lengths_abs, _ = self._fit_observed_geometry(sweep, anchors)
            weights.append(weight)

            sweep_fields = self._extract_sweep_arrays(sweep)
            fixed_indices, _, drive_idx, sensor_idx, _, _, _ = sweep_fields

            pred_geom = predict_ellipse_geometry(
                anchors,
                fixed_indices,
                fixed_lengths_abs,
                drive_idx,
                sensor_idx,
                self.dimensions,
            )

            if obs_geom is None or pred_geom is None:
                weighted_costs.append(self.invalid_penalty)
                continue

            pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
            obs_center, obs_axes, obs_theta = obs_geom

            dist = geometry_distance(
                obs_center, obs_axes, obs_theta, pred_center, pred_axes, pred_theta, self.geometry_weights
            )
            weighted_costs.append(dist**2)

        weight_sum = float(sum(weights)) if weights else 1.0
        if weight_sum <= 0:
            weight_sum = 1.0

        total_cost = sum(wc * (w / weight_sum) for wc, w in zip(weighted_costs, weights))
        return float(total_cost)

    def evaluate_detailed(self, anchor_vec: np.ndarray) -> CostResult:
        """Evaluate cost with per-sweep breakdown."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)

        per_sweep_costs: Dict[str, float] = {}
        num_valid = 0
        num_invalid = 0
        weights: Dict[str, float] = {}

        for sweep in self.sweeps:
            obs_geom, weight, fixed_lengths_abs, sweep_id = self._fit_observed_geometry(sweep, anchors)
            weights[sweep_id] = weight

            fixed_indices, _, drive_idx, sensor_idx, _, _, _ = self._extract_sweep_arrays(sweep)
            pred_geom = predict_ellipse_geometry(
                anchors,
                fixed_indices,
                fixed_lengths_abs,
                drive_idx,
                sensor_idx,
                self.dimensions,
            )

            if obs_geom is None or pred_geom is None:
                per_sweep_costs[sweep_id] = self.invalid_penalty
                num_invalid += 1
                continue

            pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
            obs_center, obs_axes, obs_theta = obs_geom
            dist = geometry_distance(
                obs_center, obs_axes, obs_theta, pred_center, pred_axes, pred_theta, self.geometry_weights
            )
            per_sweep_costs[sweep_id] = dist**2
            num_valid += 1

        weight_sum = float(sum(weights.values())) if weights else 1.0
        if weight_sum <= 0:
            weight_sum = 1.0

        total_cost = sum(
            per_sweep_costs[sid] * (weights.get(sid, 1.0) / weight_sum) for sid in per_sweep_costs
        )

        return CostResult(
            total_cost=float(total_cost),
            per_sweep_costs=per_sweep_costs,
            num_valid_sweeps=num_valid,
            num_invalid_sweeps=num_invalid,
            anchor_estimate=anchors,
        )

    def gradient_numerical(self, anchor_vec: np.ndarray, epsilon: float = 1e-6) -> np.ndarray:
        """Finite-difference gradient of the cost."""
        anchor_vec = np.asarray(anchor_vec, dtype=float)
        grad = np.zeros_like(anchor_vec)
        f0 = self.evaluate(anchor_vec)

        for i in range(len(anchor_vec)):
            x_plus = anchor_vec.copy()
            x_plus[i] += epsilon
            f_plus = self.evaluate(x_plus)
            grad[i] = (f_plus - f0) / epsilon

        return grad


def create_optimization_objective(
    dataset: Union[dict, "SweepDataset"], **cost_kwargs: object
) -> Tuple[Callable[[np.ndarray], float], Callable[[np.ndarray], np.ndarray]]:
    """Return objective and gradient callables for scipy.optimize."""
    cost_fn = EllipseCostFunction(dataset, **cost_kwargs)

    def objective(x: np.ndarray) -> float:
        return cost_fn.evaluate(x)

    def gradient(x: np.ndarray) -> np.ndarray:
        return cost_fn.gradient_numerical(x)

    return objective, gradient


def combined_cost_function(
    anchor_vec: np.ndarray,
    ellipse_cost_fn: EllipseCostFunction,
    point_cost_fn: Optional[Callable[[np.ndarray], float]] = None,
    ellipse_weight: float = 1.0,
    point_weight: float = 0.1,
) -> float:
    """Blend ellipse-based cost with an optional point-based term."""
    cost = ellipse_weight * ellipse_cost_fn.evaluate(anchor_vec)

    if point_cost_fn is not None:
        cost += point_weight * point_cost_fn(anchor_vec)

    return float(cost)


def anchor_regularity_penalty(
    anchor_vec: np.ndarray,
    num_anchors: int,
    dimensions: int,
    target_symmetry: str = "none",
) -> float:
    """
    Regularization that discourages implausible anchor placements.

    Currently penalizes anchors that are closer together than a minimum spacing.
    """
    anchors = anchors_vec_to_matrix(anchor_vec, num_anchors, dimensions)

    penalty = 0.0
    min_dist = 100.0

    for i in range(num_anchors):
        for j in range(i + 1, num_anchors):
            dist = np.linalg.norm(anchors[i] - anchors[j])
            if dist < min_dist:
                penalty += float((min_dist - dist) ** 2)

    if target_symmetry == "triangular" and num_anchors >= 3:
        pass

    return penalty
