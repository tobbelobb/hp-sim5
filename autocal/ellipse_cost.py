from __future__ import annotations

"""Cost function for ellipse-based feature calibration."""

from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

from autocal.ellipse_fitting import ellipse_sampson_residuals, fit_ellipse_from_sweep
from autocal.flex import FlexModel
from autocal.sweep_types import MachineConfig, MachineType, Sweep
from autocal.theoretical_ellipse import (
    anchors_vec_to_matrix,
    get_anchor_bounds,
    predict_ellipse_coefficients,
    predict_ellipse_geometry,
)

_EPS_LEN_MM = 1.0  # Prevent squaring negative/near-zero lengths during reconstruction.


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

def geometry_distance_squared_normalized(
    obs_center: np.ndarray,
    obs_axes: np.ndarray,
    obs_theta: float,
    pred_center: np.ndarray,
    pred_axes: np.ndarray,
    pred_theta: float,
    weights: Tuple[float, float, float] = (1.0, 1.0, 0.2),
    *,
    center_scale: float,
    axes_scale: float,
) -> float:
    """
    Dimensionless squared distance in canonical geometry space.

    The raw ellipse geometry lives in the (L_drive^2, L_sensor^2) plane, so center/axes are O(1e7)
    for mm-scale machines. Comparing those directly produces O(1e10..1e14) costs that swamp
    gradients and make penalties meaningless. This function compares *relative* geometry error.
    """
    w_center, w_axes, w_theta = weights

    obs_center = np.asarray(obs_center, dtype=float).reshape(2)
    pred_center = np.asarray(pred_center, dtype=float).reshape(2)
    obs_axes = np.asarray(obs_axes, dtype=float).reshape(2)
    pred_axes = np.asarray(pred_axes, dtype=float).reshape(2)

    center_scale = float(max(center_scale, 1.0))
    axes_scale = float(max(axes_scale, 1.0))

    delta_center = (obs_center - pred_center) / center_scale
    delta_axes = (obs_axes - pred_axes) / axes_scale

    delta_theta = (obs_theta - pred_theta + np.pi) % (2 * np.pi) - np.pi

    return float(
        w_center * np.dot(delta_center, delta_center)
        + w_axes * np.dot(delta_axes, delta_axes)
        + w_theta * (delta_theta**2)
    )

def pseudo_huber_loss(residuals: np.ndarray, *, delta: float) -> np.ndarray:
    """
    Smooth robust loss (quadratic near zero, linear for outliers).

    Returns per-element loss for the given residuals.
    """
    r = np.asarray(residuals, dtype=float)
    d = float(delta)
    if not np.isfinite(d) or d <= 0.0:
        return np.square(r)
    scaled = r / d
    return 2.0 * (d**2) * (np.sqrt(1.0 + np.square(scaled)) - 1.0)


def pseudo_huber_from_squared_error(squared_error: float, *, delta: float) -> float:
    """Apply pseudo-Huber to a scalar squared error by interpreting it as r^2."""
    se = float(squared_error)
    if not np.isfinite(se) or se < 0.0:
        return float("inf")
    r = float(np.sqrt(se))
    return float(pseudo_huber_loss(np.array([r], dtype=float), delta=delta)[0])


def pointwise_sampson_cost_normalized(
    coeffs: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    *,
    l2_scale: float,
    weight: float = 1.0,
    robust_loss: bool = False,
    huber_delta: float = 1.0,
) -> Tuple[float, float, float]:
    """
    Normalized pointwise cost of samples to a predicted ellipse.

    Returns (cost, rms, max), where residual stats are in the squared-length plane's
    units and cost is dimensionless (scaled by `l2_scale`).
    """
    coeffs = np.asarray(coeffs, dtype=float).reshape(6)
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if x.size == 0 or y.size == 0 or x.size != y.size:
        return float("inf"), float("inf"), float("inf")

    residuals = ellipse_sampson_residuals(coeffs, x, y)
    if not np.all(np.isfinite(residuals)):
        return float("inf"), float("inf"), float("inf")

    rms = float(np.sqrt(np.mean(residuals**2)))
    max_abs = float(np.max(np.abs(residuals)))

    scale = float(max(l2_scale, 1.0))
    r_norm = residuals / scale
    if bool(robust_loss):
        cost = float(np.mean(pseudo_huber_loss(r_norm, delta=float(huber_delta))))
    else:
        cost = float(np.mean(r_norm**2))
    return float(weight) * cost, rms, max_abs


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
        cost_mode: str = "geometry",
        invalid_sweep_penalty: float = 1e6,
        weight_floor: float = 1e-3,
        min_weight: float = 0.2,
        max_weight: float = 1.0,
        residual_cost_weight: float = 0.1,
        pointwise_cost_weight: float = 1e8,
        spring_k_multiplier: float = 1.0,
        use_flex: bool = True,
        robust_loss: bool = False,
        huber_delta: float = 1.0,
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
        self.cost_mode = str(cost_mode or "geometry").strip().lower()
        self.invalid_penalty = invalid_sweep_penalty
        self.weight_floor = weight_floor
        self.min_weight = float(min_weight)
        self.max_weight = float(max_weight)
        self.residual_cost_weight = float(residual_cost_weight)
        self.pointwise_cost_weight = float(pointwise_cost_weight)
        self.robust_loss = bool(robust_loss)
        self.huber_delta = float(huber_delta)

        lb, ub = get_anchor_bounds(self.machine_type)
        ub_mat = np.asarray(ub, dtype=float).reshape(self.num_anchors, self.dimensions)
        max_anchor_norm = float(np.max(np.linalg.norm(ub_mat, axis=1))) if ub_mat.size else 1.0
        self._l2_scale = float((2.0 * max_anchor_norm) ** 2)

        self.flex_model: Optional[FlexModel] = None
        if bool(use_flex) and isinstance(dataset, dict):
            m666 = (dataset.get("config") or {}).get("m666")
            self.flex_model = FlexModel.from_m666(
                m666,
                num_axes=self.num_anchors,
                spring_k_multiplier=float(spring_k_multiplier),
            )

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

    @staticmethod
    def _extract_tension_arrays(
        sweep: Union[Sweep, dict]
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if isinstance(sweep, Sweep):
            return None, None

        data_points = sweep.get("data_points", [])
        if not isinstance(data_points, list) or not data_points:
            return None, None

        t_drive = np.array([p.get("assumed_tension_drive_n", np.nan) for p in data_points], dtype=float)
        t_sensor = np.array([p.get("assumed_tension_sensor_n", np.nan) for p in data_points], dtype=float)
        if t_drive.ndim != 1 or t_sensor.ndim != 1:
            return None, None
        return t_drive, t_sensor

    def _reconstruct_lengths(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Tuple[List[float], int, int, np.ndarray, np.ndarray, str, float]:
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

        # Baseline reconstruction is only valid when absolute lengths remain positive. If a guess
        # makes some lengths negative, we keep optimization stable by (a) adding a smooth penalty
        # proportional to the violation magnitude, and (b) clipping to a small epsilon before
        # squaring inside the ellipse fitter.
        fixed_lengths_abs = [float(np.linalg.norm(anchors[idx]) + delta) for idx, delta in zip(fixed_indices, fixed_deltas)]
        l_drive_abs = l_drive + float(np.linalg.norm(anchors[drive_idx]))
        l_sensor_abs = l_sensor + float(np.linalg.norm(anchors[sensor_idx]))

        penalty = 0.0
        if fixed_lengths_abs:
            fixed_arr = np.asarray(fixed_lengths_abs, dtype=float)
            neg = np.maximum(_EPS_LEN_MM - fixed_arr, 0.0)
            penalty += float(np.dot(neg, neg))
            fixed_lengths_abs = [float(max(v, _EPS_LEN_MM)) for v in fixed_arr]

        if l_drive_abs.size:
            neg = np.maximum(_EPS_LEN_MM - l_drive_abs, 0.0)
            penalty += float(np.dot(neg, neg))
            l_drive_abs = np.maximum(l_drive_abs, _EPS_LEN_MM)

        if l_sensor_abs.size:
            neg = np.maximum(_EPS_LEN_MM - l_sensor_abs, 0.0)
            penalty += float(np.dot(neg, neg))
            l_sensor_abs = np.maximum(l_sensor_abs, _EPS_LEN_MM)

        # Convert to a dimensionless scale by normalizing with typical squared-length magnitude.
        typical = float(
            max(
                np.mean(l_drive_abs) if l_drive_abs.size else 0.0,
                np.mean(l_sensor_abs) if l_sensor_abs.size else 0.0,
                np.mean(fixed_lengths_abs) if fixed_lengths_abs else 0.0,
                1.0,
            )
        )
        penalty = penalty / (typical * typical)
        return fixed_lengths_abs, drive_idx, sensor_idx, l_drive_abs, l_sensor_abs, sweep_id, penalty

    def _fit_observed_geometry(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Tuple[Optional[Tuple[np.ndarray, np.ndarray, float]], float, List[float], str, float, float]:
        """
        Fit ellipse geometry for a sweep given the current anchor guess.

        Returns:
            (canonical_geometry or None, weight, fixed_lengths_abs, sweep_id, residual_ratio, violation_penalty)
        """
        (
            fixed_lengths_abs,
            drive_idx,
            sensor_idx,
            l_drive_abs,
            l_sensor_abs,
            sweep_id,
            violation_penalty,
        ) = self._reconstruct_lengths(sweep, anchors)

        if self.flex_model is not None:
            t_drive, t_sensor = self._extract_tension_arrays(sweep)
            if (
                t_drive is not None
                and t_sensor is not None
                and t_drive.shape == l_drive_abs.shape
                and t_sensor.shape == l_sensor_abs.shape
            ):
                l_drive_abs = self.flex_model.corrected_distance_mm(l_drive_abs, t_drive, axis=drive_idx)
                l_sensor_abs = self.flex_model.corrected_distance_mm(l_sensor_abs, t_sensor, axis=sensor_idx)

        fit = fit_ellipse_from_sweep(
            l_drive_abs,
            l_sensor_abs,
            residual_threshold=float("inf"),
            min_points=self.min_points,
            square_inputs=True,
        )

        residual_ratio = float("inf")
        if np.isfinite(fit.residual_rms):
            denom = float(self.residual_threshold) if float(self.residual_threshold) > 0 else 1.0
            residual_ratio = float(fit.residual_rms) / denom

        # Keep weights conservative: never overweight a sweep; don't downweight too hard either.
        weight = 1.0
        if self.use_weights and np.isfinite(residual_ratio):
            weight = 1.0 / max(1.0, residual_ratio)
            weight = float(np.clip(weight, self.min_weight, self.max_weight))

        # Only reject when geometry cannot be extracted (non-ellipse / degeneracy).
        if not fit.valid:
            return None, 1.0, fixed_lengths_abs, sweep_id, residual_ratio, violation_penalty

        geom = canonicalize_geometry(fit.center, fit.semi_axes, fit.rotation_angle)
        return geom, weight, fixed_lengths_abs, sweep_id, residual_ratio, violation_penalty

    def _pointwise_predicted_cost(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Tuple[float, float, float, float, str]:
        """
        Pointwise cost of reconstructed samples against the predicted ellipse.

        Returns (cost, rms, max_abs, violation_penalty, sweep_id).
        """
        (
            fixed_lengths_abs,
            drive_idx,
            sensor_idx,
            l_drive_abs,
            l_sensor_abs,
            sweep_id,
            violation_penalty,
        ) = self._reconstruct_lengths(sweep, anchors)

        if self.flex_model is not None:
            t_drive, t_sensor = self._extract_tension_arrays(sweep)
            if (
                t_drive is not None
                and t_sensor is not None
                and t_drive.shape == l_drive_abs.shape
                and t_sensor.shape == l_sensor_abs.shape
            ):
                l_drive_abs = self.flex_model.corrected_distance_mm(l_drive_abs, t_drive, axis=drive_idx)
                l_sensor_abs = self.flex_model.corrected_distance_mm(l_sensor_abs, t_sensor, axis=sensor_idx)

        fixed_indices, _, drive_idx2, sensor_idx2, *_rest = self._extract_sweep_arrays(sweep)
        coeffs = predict_ellipse_coefficients(
            anchors,
            fixed_indices,
            fixed_lengths_abs,
            drive_idx2,
            sensor_idx2,
            dimensions=self.dimensions,
        )
        if coeffs is None:
            return float("inf"), float("inf"), float("inf"), float(violation_penalty), sweep_id

        x = l_drive_abs**2
        y = l_sensor_abs**2
        cost, rms, max_abs = pointwise_sampson_cost_normalized(
            coeffs,
            x,
            y,
            l2_scale=self._l2_scale,
            weight=self.pointwise_cost_weight,
            robust_loss=self.robust_loss,
            huber_delta=self.huber_delta,
        )
        return float(cost), float(rms), float(max_abs), float(violation_penalty), str(sweep_id)

    def evaluate(self, anchor_vec: np.ndarray) -> float:
        """Compute scalar cost for a flat anchor vector."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)

        if not self.sweeps:
            return 0.0

        weighted_costs: List[float] = []
        weights: List[float] = []

        for sweep in self.sweeps:
            if self.cost_mode in ("pointwise", "sampson"):
                cost, _rms, _max_abs, violation_penalty, _sid = self._pointwise_predicted_cost(
                    sweep, anchors
                )
                weights.append(1.0)
                if not np.isfinite(cost):
                    weighted_costs.append(float(self.invalid_penalty) + float(violation_penalty))
                else:
                    weighted_costs.append(float(cost + violation_penalty))
                continue

            obs_geom, weight, fixed_lengths_abs, _, residual_ratio, violation_penalty = self._fit_observed_geometry(
                sweep, anchors
            )
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
                weighted_costs.append(float(self.invalid_penalty) + float(violation_penalty))
                continue

            pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
            obs_center, obs_axes, obs_theta = obs_geom

            geom_cost = geometry_distance_squared_normalized(
                obs_center,
                obs_axes,
                obs_theta,
                pred_center,
                pred_axes,
                pred_theta,
                self.geometry_weights,
                center_scale=self._l2_scale,
                axes_scale=self._l2_scale,
            )
            if self.robust_loss:
                geom_cost = pseudo_huber_from_squared_error(geom_cost, delta=self.huber_delta)

            residual_cost = 0.0
            if np.isfinite(residual_ratio) and float(residual_ratio) > 1.0:
                dr = float(residual_ratio) - 1.0
                residual_cost = self.residual_cost_weight * float(dr * dr)
            weighted_costs.append(float(geom_cost + residual_cost + violation_penalty))

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
            if self.cost_mode in ("pointwise", "sampson"):
                cost, _rms, _max_abs, violation_penalty, sweep_id = self._pointwise_predicted_cost(
                    sweep, anchors
                )
                weights[sweep_id] = 1.0
                if not np.isfinite(cost):
                    per_sweep_costs[sweep_id] = float(self.invalid_penalty) + float(violation_penalty)
                    num_invalid += 1
                else:
                    per_sweep_costs[sweep_id] = float(cost + violation_penalty)
                    num_valid += 1
                continue

            obs_geom, weight, fixed_lengths_abs, sweep_id, residual_ratio, violation_penalty = self._fit_observed_geometry(
                sweep, anchors
            )
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
                per_sweep_costs[sweep_id] = float(self.invalid_penalty) + float(violation_penalty)
                num_invalid += 1
                continue

            pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
            obs_center, obs_axes, obs_theta = obs_geom

            geom_cost = geometry_distance_squared_normalized(
                obs_center,
                obs_axes,
                obs_theta,
                pred_center,
                pred_axes,
                pred_theta,
                self.geometry_weights,
                center_scale=self._l2_scale,
                axes_scale=self._l2_scale,
            )
            if self.robust_loss:
                geom_cost = pseudo_huber_from_squared_error(geom_cost, delta=self.huber_delta)
            residual_cost = 0.0
            if np.isfinite(residual_ratio) and float(residual_ratio) > 1.0:
                dr = float(residual_ratio) - 1.0
                residual_cost = self.residual_cost_weight * float(dr * dr)
            per_sweep_costs[sweep_id] = float(geom_cost + residual_cost + violation_penalty)
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
