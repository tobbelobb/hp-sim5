from __future__ import annotations

"""Cost function for ellipse-based feature calibration."""

from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

from autocal.ellipse_fitting import (
    ellipse_euclidean_residuals,
    ellipse_sampson_residuals,
    fit_ellipse_from_sweep,
)
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


def _circular_mean_pi(angles: np.ndarray) -> float:
    """Mean angle for periodicity pi (ellipse orientation)."""
    a = np.asarray(angles, dtype=float)
    if a.size == 0 or not np.all(np.isfinite(a)):
        return 0.0
    doubled = 2.0 * a
    mean_sin = float(np.mean(np.sin(doubled)))
    mean_cos = float(np.mean(np.cos(doubled)))
    return 0.5 * float(np.arctan2(mean_sin, mean_cos))


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


def pointwise_euclidean_cost_normalized(
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
    Normalized pointwise cost using Euclidean distance to the predicted ellipse.

    Returns (cost, rms, max) where residual stats are in the squared-length plane's
    units and cost is dimensionless (scaled by `l2_scale`).
    """
    coeffs = np.asarray(coeffs, dtype=float).reshape(6)
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if x.size == 0 or y.size == 0 or x.size != y.size:
        return float("inf"), float("inf"), float("inf")

    residuals = ellipse_euclidean_residuals(coeffs, x, y)
    if residuals.size == 0 or not np.all(np.isfinite(residuals)):
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
        pointwise_residual_mode: str = "sampson",
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
        ransac: bool = False,
        ransac_trials: int = 60,
        ransac_sample_size: int = 5,
        ransac_min_inlier_ratio: float = 0.5,
        ransac_threshold: Optional[float] = None,
        ransac_seed: Optional[int] = 0,
        mahalanobis_rejection: bool = False,
        mahalanobis_threshold: float = 3.0,
        mahalanobis_min_samples: int = 8,
        mahalanobis_regularization: float = 1e-6,
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
        self.pointwise_residual_mode = str(pointwise_residual_mode or "sampson").strip().lower()
        self.invalid_penalty = invalid_sweep_penalty
        self.weight_floor = weight_floor
        self.min_weight = float(min_weight)
        self.max_weight = float(max_weight)
        self.residual_cost_weight = float(residual_cost_weight)
        self.pointwise_cost_weight = float(pointwise_cost_weight)
        self.robust_loss = bool(robust_loss)
        self.huber_delta = float(huber_delta)
        self.ransac = bool(ransac)
        self.ransac_trials = int(ransac_trials)
        self.ransac_sample_size = int(ransac_sample_size)
        self.ransac_min_inlier_ratio = float(ransac_min_inlier_ratio)
        self.ransac_threshold = ransac_threshold if ransac_threshold is None else float(ransac_threshold)
        self.ransac_seed = ransac_seed if ransac_seed is None else int(ransac_seed)
        self.mahalanobis_rejection = bool(mahalanobis_rejection)
        self.mahalanobis_threshold = float(mahalanobis_threshold)
        self.mahalanobis_min_samples = int(mahalanobis_min_samples)
        self.mahalanobis_regularization = float(mahalanobis_regularization)

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
            ransac=self.ransac,
            ransac_trials=self.ransac_trials,
            ransac_sample_size=self.ransac_sample_size,
            ransac_min_inlier_ratio=self.ransac_min_inlier_ratio,
            ransac_threshold=self.ransac_threshold if self.ransac_threshold is not None else self.residual_threshold,
            ransac_seed=self.ransac_seed,
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

    def _mahalanobis_distances(
        self, obs_geoms: List[Tuple[np.ndarray, np.ndarray, float]]
    ) -> Tuple[Optional[np.ndarray], Optional[str]]:
        """Compute squared Mahalanobis distances for observed geometries."""
        if not self.mahalanobis_rejection:
            return None, "disabled"
        if not np.isfinite(self.mahalanobis_threshold) or self.mahalanobis_threshold <= 0.0:
            return None, "invalid-threshold"

        n = len(obs_geoms)
        min_samples = max(2, self.mahalanobis_min_samples)
        if n < min_samples:
            return None, f"insufficient-samples ({n} < {min_samples})"

        centers = np.array([g[0] for g in obs_geoms], dtype=float)
        axes = np.array([g[1] for g in obs_geoms], dtype=float)
        thetas = np.array([g[2] for g in obs_geoms], dtype=float)
        if not (np.all(np.isfinite(centers)) and np.all(np.isfinite(axes)) and np.all(np.isfinite(thetas))):
            return None, "non-finite-geometry"

        center_scale = float(max(self._l2_scale, 1.0))
        axes_scale = float(max(self._l2_scale, 1.0))
        center_scaled = centers / center_scale
        axes_scaled = axes / axes_scale
        theta_mean = _circular_mean_pi(thetas)
        theta_delta = ((thetas - theta_mean + np.pi / 2) % np.pi) - np.pi / 2

        X = np.column_stack([center_scaled, axes_scaled, theta_delta])
        mean = np.mean(X, axis=0)
        Xc = X - mean
        cov = np.cov(Xc, rowvar=False, bias=False)
        if cov.ndim != 2 or cov.shape[0] != X.shape[1]:
            return None, "covariance-shape"

        reg = float(max(self.mahalanobis_regularization, 0.0))
        cov = cov + reg * np.eye(cov.shape[0])
        inv_cov = np.linalg.pinv(cov)
        d2 = np.einsum("ij,jk,ik->i", Xc, inv_cov, Xc)
        if not np.all(np.isfinite(d2)):
            return None, "non-finite-distance"

        return d2, None

    def _mahalanobis_keep_mask(
        self, obs_geoms: List[Tuple[np.ndarray, np.ndarray, float]]
    ) -> Optional[np.ndarray]:
        d2, _reason = self._mahalanobis_distances(obs_geoms)
        if d2 is None:
            return None

        thresh2 = float(self.mahalanobis_threshold) ** 2
        keep = d2 <= thresh2
        if not np.any(keep):
            return None
        return keep

    def robustness_diagnostics(self, anchor_vec: np.ndarray, *, top_n: int = 5) -> Dict[str, object]:
        """Return per-sweep diagnostics for RANSAC and Mahalanobis filtering."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)
        diagnostics: Dict[str, object] = {"cost_mode": self.cost_mode}

        sweep_entries: List[Dict[str, object]] = []
        geom_entries: List[Tuple[str, Tuple[np.ndarray, np.ndarray, float]]] = []

        for sweep in self.sweeps:
            (
                _fixed_lengths_abs,
                drive_idx,
                sensor_idx,
                l_drive_abs,
                l_sensor_abs,
                sweep_id,
                _violation_penalty,
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
                ransac=self.ransac,
                ransac_trials=self.ransac_trials,
                ransac_sample_size=self.ransac_sample_size,
                ransac_min_inlier_ratio=self.ransac_min_inlier_ratio,
                ransac_threshold=self.ransac_threshold if self.ransac_threshold is not None else self.residual_threshold,
                ransac_seed=self.ransac_seed,
            )

            residual_ratio = float("inf")
            if np.isfinite(fit.residual_rms):
                denom = float(self.residual_threshold) if float(self.residual_threshold) > 0 else 1.0
                residual_ratio = float(fit.residual_rms) / denom

            inlier_ratio = None
            if fit.ransac_used and fit.num_inliers is not None and fit.num_points > 0:
                inlier_ratio = float(fit.num_inliers) / float(fit.num_points)

            sweep_entries.append(
                {
                    "sweep_id": str(sweep_id),
                    "num_points": int(fit.num_points),
                    "num_inliers": fit.num_inliers,
                    "inlier_ratio": inlier_ratio,
                    "ransac_used": bool(fit.ransac_used),
                    "valid": bool(fit.valid),
                    "residual_rms": float(fit.residual_rms),
                    "residual_ratio": residual_ratio,
                    "rejection_reason": fit.rejection_reason,
                }
            )

            if fit.valid:
                geom_entries.append(
                    (str(sweep_id), canonicalize_geometry(fit.center, fit.semi_axes, fit.rotation_angle))
                )

        ransac_used_entries = [e for e in sweep_entries if e["ransac_used"]]
        ransac_fallbacks = [e for e in sweep_entries if self.ransac and not e["ransac_used"]]
        inlier_ratios = [e["inlier_ratio"] for e in ransac_used_entries if e["inlier_ratio"] is not None]
        inlier_stats = None
        if inlier_ratios:
            inlier_stats = {
                "min": float(np.min(inlier_ratios)),
                "median": float(np.median(inlier_ratios)),
                "max": float(np.max(inlier_ratios)),
            }

        worst_inliers: List[Dict[str, object]] = []
        if ransac_used_entries:
            worst_sorted = sorted(
                ransac_used_entries,
                key=lambda e: (e["inlier_ratio"] if e["inlier_ratio"] is not None else float("inf")),
            )
            for entry in worst_sorted[: max(1, int(top_n))]:
                worst_inliers.append(
                    {
                        "sweep_id": entry["sweep_id"],
                        "num_inliers": entry["num_inliers"],
                        "num_points": entry["num_points"],
                        "inlier_ratio": entry["inlier_ratio"],
                        "residual_rms": entry["residual_rms"],
                        "valid": entry["valid"],
                    }
                )

        diagnostics["ransac"] = {
            "enabled": bool(self.ransac),
            "threshold": self.ransac_threshold,
            "threshold_mode": "auto" if self.ransac_threshold is None else "fixed",
            "trials": int(self.ransac_trials),
            "sample_size": int(self.ransac_sample_size),
            "min_inlier_ratio": float(self.ransac_min_inlier_ratio),
            "sweeps_total": int(len(sweep_entries)),
            "sweeps_used": int(len(ransac_used_entries)),
            "sweeps_fallback": int(len(ransac_fallbacks)),
            "inlier_ratio_stats": inlier_stats,
            "worst_inliers": worst_inliers,
            "fallback_sweeps": [e["sweep_id"] for e in ransac_fallbacks[: max(1, int(top_n))]],
        }

        mah_status = "disabled"
        mah_distances = None
        mah_keep = None
        mah_reason = None
        if self.mahalanobis_rejection and not geom_entries:
            mah_status = "no-valid-geometry"
        if geom_entries:
            obs_geoms = [entry[1] for entry in geom_entries]
            mah_distances, mah_reason = self._mahalanobis_distances(obs_geoms)
        if mah_distances is not None:
            mah_status = "ok"
            mah_keep = mah_distances <= float(self.mahalanobis_threshold) ** 2
        elif mah_reason is not None:
            mah_status = mah_reason

        mah_worst: List[Dict[str, object]] = []
        if mah_distances is not None:
            distances = np.sqrt(mah_distances)
            order = np.argsort(distances)[::-1]
            top = order[: max(1, int(top_n))]
            for idx in top:
                sweep_id = geom_entries[int(idx)][0]
                mah_worst.append(
                    {
                        "sweep_id": sweep_id,
                        "distance": float(distances[int(idx)]),
                        "keep": bool(mah_keep[int(idx)]) if mah_keep is not None else True,
                    }
                )

        diagnostics["mahalanobis"] = {
            "enabled": bool(self.mahalanobis_rejection),
            "threshold": float(self.mahalanobis_threshold),
            "min_samples": int(self.mahalanobis_min_samples),
            "regularization": float(self.mahalanobis_regularization),
            "status": mah_status,
            "valid_geometries": int(len(geom_entries)),
            "rejected": int(np.sum(~mah_keep)) if mah_keep is not None else 0,
            "worst_distances": mah_worst,
        }

        return diagnostics

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
        mode = self.pointwise_residual_mode
        if mode in ("euclidean", "exact", "distance"):
            cost, rms, max_abs = pointwise_euclidean_cost_normalized(
                coeffs,
                x,
                y,
                l2_scale=self._l2_scale,
                weight=self.pointwise_cost_weight,
                robust_loss=self.robust_loss,
                huber_delta=self.huber_delta,
            )
        else:
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

        if self.cost_mode in ("pointwise", "sampson"):
            for sweep in self.sweeps:
                cost, _rms, _max_abs, violation_penalty, _sid = self._pointwise_predicted_cost(
                    sweep, anchors
                )
                weights.append(1.0)
                if not np.isfinite(cost):
                    weighted_costs.append(float(self.invalid_penalty) + float(violation_penalty))
                else:
                    weighted_costs.append(float(cost + violation_penalty))
        else:
            entries: List[Dict[str, object]] = []
            for sweep in self.sweeps:
                obs_geom, weight, fixed_lengths_abs, _sid, residual_ratio, violation_penalty = self._fit_observed_geometry(
                    sweep, anchors
                )
                if obs_geom is None:
                    weighted_costs.append(float(self.invalid_penalty) + float(violation_penalty))
                    weights.append(1.0)
                    continue

                fixed_indices, _, drive_idx, sensor_idx, _, _, _ = self._extract_sweep_arrays(sweep)
                entries.append(
                    {
                        "obs_geom": obs_geom,
                        "weight": float(weight),
                        "fixed_lengths_abs": fixed_lengths_abs,
                        "fixed_indices": fixed_indices,
                        "drive_idx": drive_idx,
                        "sensor_idx": sensor_idx,
                        "residual_ratio": float(residual_ratio),
                        "violation_penalty": float(violation_penalty),
                    }
                )

            keep_mask = self._mahalanobis_keep_mask([e["obs_geom"] for e in entries]) if entries else None

            for idx, entry in enumerate(entries):
                if keep_mask is not None and not bool(keep_mask[idx]):
                    weights.append(0.0)
                    weighted_costs.append(0.0)
                    continue

                pred_geom = predict_ellipse_geometry(
                    anchors,
                    entry["fixed_indices"],
                    entry["fixed_lengths_abs"],
                    entry["drive_idx"],
                    entry["sensor_idx"],
                    self.dimensions,
                )

                if pred_geom is None:
                    weighted_costs.append(float(self.invalid_penalty) + float(entry["violation_penalty"]))
                    weights.append(1.0)
                    continue

                pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
                obs_center, obs_axes, obs_theta = entry["obs_geom"]

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
                if np.isfinite(entry["residual_ratio"]) and float(entry["residual_ratio"]) > 1.0:
                    dr = float(entry["residual_ratio"]) - 1.0
                    residual_cost = self.residual_cost_weight * float(dr * dr)
                weighted_costs.append(float(geom_cost + residual_cost + float(entry["violation_penalty"])))
                weights.append(float(entry["weight"]))

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

        if self.cost_mode in ("pointwise", "sampson"):
            for sweep in self.sweeps:
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
        else:
            entries: List[Dict[str, object]] = []
            for sweep in self.sweeps:
                obs_geom, weight, fixed_lengths_abs, sweep_id, residual_ratio, violation_penalty = self._fit_observed_geometry(
                    sweep, anchors
                )
                if obs_geom is None:
                    per_sweep_costs[sweep_id] = float(self.invalid_penalty) + float(violation_penalty)
                    weights[sweep_id] = 1.0
                    num_invalid += 1
                    continue

                fixed_indices, _, drive_idx, sensor_idx, _, _, _ = self._extract_sweep_arrays(sweep)
                entries.append(
                    {
                        "sweep_id": sweep_id,
                        "obs_geom": obs_geom,
                        "weight": float(weight),
                        "fixed_lengths_abs": fixed_lengths_abs,
                        "fixed_indices": fixed_indices,
                        "drive_idx": drive_idx,
                        "sensor_idx": sensor_idx,
                        "residual_ratio": float(residual_ratio),
                        "violation_penalty": float(violation_penalty),
                    }
                )

            keep_mask = self._mahalanobis_keep_mask([e["obs_geom"] for e in entries]) if entries else None

            for idx, entry in enumerate(entries):
                sweep_id = str(entry["sweep_id"])
                if keep_mask is not None and not bool(keep_mask[idx]):
                    per_sweep_costs[sweep_id] = float(self.invalid_penalty)
                    weights[sweep_id] = 0.0
                    num_invalid += 1
                    continue

                pred_geom = predict_ellipse_geometry(
                    anchors,
                    entry["fixed_indices"],
                    entry["fixed_lengths_abs"],
                    entry["drive_idx"],
                    entry["sensor_idx"],
                    self.dimensions,
                )

                if pred_geom is None:
                    per_sweep_costs[sweep_id] = float(self.invalid_penalty) + float(entry["violation_penalty"])
                    weights[sweep_id] = 1.0
                    num_invalid += 1
                    continue

                pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
                obs_center, obs_axes, obs_theta = entry["obs_geom"]

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
                if np.isfinite(entry["residual_ratio"]) and float(entry["residual_ratio"]) > 1.0:
                    dr = float(entry["residual_ratio"]) - 1.0
                    residual_cost = self.residual_cost_weight * float(dr * dr)
                per_sweep_costs[sweep_id] = float(
                    geom_cost + residual_cost + float(entry["violation_penalty"])
                )
                weights[sweep_id] = float(entry["weight"])
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
