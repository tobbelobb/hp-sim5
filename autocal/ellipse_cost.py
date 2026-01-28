from __future__ import annotations

"""Cost function for ellipse-based feature calibration."""

from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

from autocal.ellipse_fitting import (
    ellipse_euclidean_residuals,
    ellipse_sampson_residuals,
)
from autocal.flex import FlexModel
from autocal.sweep_types import MachineConfig, MachineType, Sweep
from autocal.theoretical_ellipse import (
    anchors_vec_to_matrix,
    get_anchor_bounds,
    predict_ellipse_coefficients,
)

_EPS_LEN_MM = 1.0  # Prevent squaring negative/near-zero lengths during reconstruction.
_MAD_SCALE = 1.4826
_POINTWISE_HUBER_MULTIPLIERS = (10.0, 3.0)
_POINTWISE_TRIM_K = 3.0
_POINTWISE_MIN_INLIERS = 5
_POINTWISE_SIGMA_MULT = 3
_POINTWISE_MIN_SIGMA_MM = 0.05
_SWEEP_WISE_K = 3.0
_SWEEP_WISE_MIN_SAMPLES = 5
_SWEEP_WISE_MIN_KEEP = 2
_SWEEP_WISE_MIN_KEEP_RATIO = 0.5


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
        residual_threshold: float = 0.01,
        min_points: int = 10,
        pointwise_residual_mode: str = "sampson",
        invalid_sweep_penalty: float = 1e6,
        pointwise_cost_weight: float = 1e8,
        spring_k_multiplier: float = 1.0,
        use_flex: bool = True,
        robust_loss: bool = False,
        huber_delta: float = 1.0,
        pointwise_filtering: bool = True,
        sweep_wise_filtering: bool = True,
        pointwise_filter_stage: int = 0,
        pointwise_global_mad: bool = True,
        sweep_metric: str = "outlier_ratio",
    ) -> None:
        (
            self.machine_type,
            self.num_anchors,
            self.dimensions,
            self.sweeps,
        ) = _dataset_metadata(dataset)

        self.residual_threshold = residual_threshold
        self.min_points = min_points
        self.pointwise_residual_mode = str(pointwise_residual_mode or "sampson").strip().lower()
        self.invalid_penalty = invalid_sweep_penalty
        self.pointwise_cost_weight = float(pointwise_cost_weight)
        self.robust_loss = bool(robust_loss)
        self.huber_delta = float(huber_delta)
        self.pointwise_filtering = bool(pointwise_filtering)
        self.sweep_wise_filtering = bool(sweep_wise_filtering)
        self.pointwise_filter_stage = int(pointwise_filter_stage)
        self.pointwise_global_mad = bool(pointwise_global_mad)
        self.sweep_metric = str(sweep_metric or "mad").strip().lower()

        lb, ub = get_anchor_bounds(self.machine_type)
        ub_mat = np.asarray(ub, dtype=float).reshape(self.num_anchors, self.dimensions)
        max_anchor_norm = float(np.max(np.linalg.norm(ub_mat, axis=1))) if ub_mat.size else 1.0
        self._length_scale = float(max_anchor_norm) if np.isfinite(max_anchor_norm) else 1.0
        self._l2_scale = float((2.0 * max_anchor_norm) ** 2)
        raw_noise_mm = self._infer_encoder_noise_mm(dataset)
        if raw_noise_mm is None or not np.isfinite(raw_noise_mm) or raw_noise_mm < 0.0:
            raw_noise_mm = None
        self._encoder_noise_mm = raw_noise_mm
        self._pointwise_sigma_mult = float(_POINTWISE_SIGMA_MULT)
        self._pointwise_sigma_min_mm = float(_POINTWISE_MIN_SIGMA_MM)
        sigma_scaled_mm = None
        if raw_noise_mm is not None:
            sigma_scaled_mm = float(self._pointwise_sigma_mult * raw_noise_mm)
        self._pointwise_sigma_scaled_mm = sigma_scaled_mm
        if sigma_scaled_mm is None:
            sigma_floor_mm = float(self._pointwise_sigma_min_mm)
            sigma_source = "min"
        elif sigma_scaled_mm >= self._pointwise_sigma_min_mm:
            sigma_floor_mm = float(sigma_scaled_mm)
            sigma_source = "noise"
        else:
            sigma_floor_mm = float(self._pointwise_sigma_min_mm)
            sigma_source = "min"
        denom = float(max(2.0 * self._length_scale, 1.0))
        self._pointwise_sigma_floor_mm = float(sigma_floor_mm)
        self._pointwise_sigma_floor_source = sigma_source
        self._pointwise_sigma_floor_norm = float(self._pointwise_sigma_floor_mm / denom)

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

    @staticmethod
    def _mean_finite(values: Iterable[float]) -> Optional[float]:
        arr = np.asarray(list(values), dtype=float).ravel()
        if arr.size == 0:
            return None
        arr = arr[np.isfinite(arr)]
        if arr.size == 0:
            return None
        return float(np.mean(arr))

    @staticmethod
    def _flatten_config_values(value: object) -> List[float]:
        if value is None:
            return []
        if isinstance(value, dict):
            values = list(value.values())
        elif isinstance(value, (list, tuple, np.ndarray)):
            values = list(value)
        else:
            values = [value]

        out: List[float] = []
        for v in values:
            try:
                f = float(v)
            except (TypeError, ValueError):
                continue
            if np.isfinite(f):
                out.append(f)
        return out

    @classmethod
    def _infer_encoder_noise_mm(cls, dataset: Union[dict, "SweepDataset"]) -> Optional[float]:
        if not isinstance(dataset, dict):
            return None
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return None

        def pick(*keys: str) -> Optional[float]:
            for key in keys:
                vals = cls._flatten_config_values(config.get(key))
                mean = cls._mean_finite(vals)
                if mean is not None:
                    return mean
            ft = config.get("force_tuning") if isinstance(config.get("force_tuning"), dict) else {}
            if isinstance(ft, dict):
                for key in keys:
                    vals = cls._flatten_config_values(ft.get(key))
                    mean = cls._mean_finite(vals)
                    if mean is not None:
                        return mean
            return None

        noise_mm = pick("encoder_noise_mm", "encoder_noise_mm_per_axis")
        if noise_mm is not None:
            return float(noise_mm)

        noise_deg = pick("encoder_noise_deg", "encoder_noise_deg_per_axis")
        if noise_deg is None:
            return None
        mm_per_deg = pick("mm_per_degree", "mm_per_degree_by_axis")
        if mm_per_deg is None:
            return None
        return float(noise_deg) * float(mm_per_deg)

    @staticmethod
    def _mad_scale(values: np.ndarray) -> float:
        v = np.asarray(values, dtype=float).ravel()
        if v.size == 0 or not np.all(np.isfinite(v)):
            return float("nan")
        med = float(np.median(v))
        mad = float(np.median(np.abs(v - med)))
        scale = _MAD_SCALE * mad
        if not np.isfinite(scale) or scale <= 0.0:
            abs_med = float(np.median(np.abs(v)))
            scale = abs_med
        return float(scale)

    def _pointwise_stage_settings(self) -> Tuple[int, str, float, bool]:
        stage = int(self.pointwise_filter_stage)
        if stage < 0:
            stage = 0
        if stage > 2:
            stage = 2
        if stage == 0:
            return stage, "wide-huber", _POINTWISE_HUBER_MULTIPLIERS[0], False
        if stage == 1:
            return stage, "tight-huber", _POINTWISE_HUBER_MULTIPLIERS[1], False
        return stage, "trim", _POINTWISE_HUBER_MULTIPLIERS[1], True

    def _pointwise_residuals(
        self, coeffs: np.ndarray, x: np.ndarray, y: np.ndarray
    ) -> np.ndarray:
        mode = self.pointwise_residual_mode
        if mode in ("euclidean", "exact", "distance"):
            return ellipse_euclidean_residuals(coeffs, x, y)
        return ellipse_sampson_residuals(coeffs, x, y)

    def _pointwise_scale(
        self, r_norm: np.ndarray, *, scale_override: Optional[float] = None
    ) -> float:
        if scale_override is None:
            scale = self._mad_scale(r_norm)
        else:
            scale = float(scale_override)

        floor = float(max(self._pointwise_sigma_floor_norm, 1e-12))
        if not np.isfinite(scale) or scale <= 0.0:
            return floor
        return float(max(scale, floor))

    def _pointwise_cost_filtered(
        self, residuals: np.ndarray, *, scale_override: Optional[float] = None
    ) -> Tuple[float, float, float, Optional[np.ndarray], float, float, float]:
        residuals = np.asarray(residuals, dtype=float).ravel()
        if residuals.size == 0 or not np.all(np.isfinite(residuals)):
            return float("inf"), float("inf"), float("inf"), None, float("nan"), float("nan"), float("nan")

        rms = float(np.sqrt(np.mean(residuals**2)))
        max_abs = float(np.max(np.abs(residuals)))

        r_norm = residuals / float(max(self._l2_scale, 1.0))
        scale = self._pointwise_scale(r_norm, scale_override=scale_override)

        stage, _stage_name, huber_mult, hard_cut = self._pointwise_stage_settings()
        huber_delta = float(max(huber_mult * scale, 1e-12))
        trim_threshold = float(max(_POINTWISE_TRIM_K * scale, 1e-12))

        inlier_mask = None
        inlier_ratio = 1.0
        if hard_cut:
            inlier_mask = np.abs(r_norm) <= trim_threshold
            inlier_count = int(np.sum(inlier_mask))
            if inlier_count < _POINTWISE_MIN_INLIERS:
                return float("inf"), rms, max_abs, inlier_mask, 0.0, scale, trim_threshold
            inlier_ratio = float(inlier_count) / float(r_norm.size)
            r_used = r_norm[inlier_mask]
        else:
            r_used = r_norm

        if stage >= 0:
            losses = pseudo_huber_loss(r_used, delta=huber_delta)
            cost = float(np.mean(losses))
        else:
            cost = float(np.mean(r_used**2))

        return cost, rms, max_abs, inlier_mask, inlier_ratio, scale, trim_threshold

    def _pointwise_sweep_residuals(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Dict[str, object]:
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
            return {
                "sweep_id": str(sweep_id),
                "residuals": None,
                "violation_penalty": float(violation_penalty),
                "num_points": int(l_drive_abs.size),
            }

        x = l_drive_abs**2
        y = l_sensor_abs**2
        residuals = self._pointwise_residuals(coeffs, x, y)
        if residuals.size == 0 or not np.all(np.isfinite(residuals)):
            residuals = None

        return {
            "sweep_id": str(sweep_id),
            "residuals": residuals,
            "violation_penalty": float(violation_penalty),
            "num_points": int(residuals.size) if residuals is not None else int(l_drive_abs.size),
        }

    def pointwise_residual_rows(self, anchor_vec: np.ndarray) -> List[Dict[str, object]]:
        """
        Return per-point residuals (approx mm) for the current pointwise model.

        Residuals are computed in the L^2 plane and converted to mm with a
        local linearization: |r_mm| ≈ |r_l2| / (2 * mean_length).
        """
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)
        rows: List[Dict[str, object]] = []
        l2_scale = float(max(self._l2_scale, 1.0))
        sigma_noise_mm = self._encoder_noise_mm
        sigma_mult = self._pointwise_sigma_mult
        sigma_scaled_mm = self._pointwise_sigma_scaled_mm
        sigma_min_mm = self._pointwise_sigma_min_mm
        sigma_floor_mm = self._pointwise_sigma_floor_mm
        sigma_floor_source = self._pointwise_sigma_floor_source
        sweep_cache: List[Dict[str, object]] = []
        residuals_list: List[np.ndarray] = []

        for sweep in self.sweeps:
            (
                fixed_lengths_abs,
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
                continue

            x = l_drive_abs**2
            y = l_sensor_abs**2
            residuals = self._pointwise_residuals(coeffs, x, y)
            if residuals.size == 0 or not np.all(np.isfinite(residuals)):
                continue

            sweep_cache.append(
                {
                    "sweep_id": str(sweep_id),
                    "drive_anchor": int(drive_idx2),
                    "sensor_anchor": int(sensor_idx2),
                    "l_drive_abs": l_drive_abs,
                    "l_sensor_abs": l_sensor_abs,
                    "residuals": residuals,
                }
            )
            residuals_list.append(residuals)

        scale_override = None
        if self.pointwise_filtering and self.pointwise_global_mad and residuals_list:
            scale_override = self._pointwise_global_scale(residuals_list)
            if scale_override is not None and not np.isfinite(scale_override):
                scale_override = None

        for entry in sweep_cache:
            residuals = np.asarray(entry["residuals"], dtype=float).ravel()
            l_drive_abs = np.asarray(entry["l_drive_abs"], dtype=float).ravel()
            l_sensor_abs = np.asarray(entry["l_sensor_abs"], dtype=float).ravel()
            sweep_id = str(entry["sweep_id"])
            drive_idx2 = int(entry["drive_anchor"])
            sensor_idx2 = int(entry["sensor_anchor"])

            l_mean = 0.5 * (l_drive_abs + l_sensor_abs)
            denom = np.maximum(2.0 * l_mean, 1e-9)
            residuals_abs = np.abs(residuals)
            residuals_mm = residuals_abs / denom

            r_norm = residuals / l2_scale
            scale = self._pointwise_scale(r_norm, scale_override=scale_override)
            trim_threshold_norm = float(max(_POINTWISE_TRIM_K * scale, 1e-12))
            cutoff_mm = (trim_threshold_norm * l2_scale) / denom

            for idx, (res_l2, res_mm, ld, ls, cut_mm) in enumerate(
                zip(residuals_abs, residuals_mm, l_drive_abs, l_sensor_abs, cutoff_mm)
            ):
                if not (np.isfinite(res_l2) and np.isfinite(res_mm)):
                    continue
                rows.append(
                    {
                        "sweep_id": str(sweep_id),
                        "point_idx": int(idx),
                        "drive_anchor": int(drive_idx2),
                        "sensor_anchor": int(sensor_idx2),
                        "l_drive_mm": float(ld),
                        "l_sensor_mm": float(ls),
                        "residual_l2": float(res_l2),
                        "residual_mm": float(res_mm),
                        "cutoff_mm": float(cut_mm) if np.isfinite(cut_mm) else None,
                        "sigma_noise_mm": float(sigma_noise_mm) if sigma_noise_mm is not None else None,
                        "sigma_mult": float(sigma_mult),
                        "sigma_scaled_mm": float(sigma_scaled_mm) if sigma_scaled_mm is not None else None,
                        "sigma_min_mm": float(sigma_min_mm),
                        "sigma_floor_mm": float(sigma_floor_mm),
                        "sigma_floor_source": str(sigma_floor_source),
                    }
                )

        return rows

    def _pointwise_cost_unfiltered(self, residuals: np.ndarray) -> Tuple[float, float, float]:
        residuals = np.asarray(residuals, dtype=float).ravel()
        if residuals.size == 0 or not np.all(np.isfinite(residuals)):
            return float("inf"), float("inf"), float("inf")
        rms = float(np.sqrt(np.mean(residuals**2)))
        max_abs = float(np.max(np.abs(residuals)))
        r_norm = residuals / float(max(self._l2_scale, 1.0))
        if self.robust_loss:
            cost_unit = float(np.mean(pseudo_huber_loss(r_norm, delta=self.huber_delta)))
        else:
            cost_unit = float(np.mean(r_norm**2))
        return float(cost_unit * self.pointwise_cost_weight), rms, max_abs

    def _sweep_metric_value(self, r_norm: np.ndarray, *, inlier_ratio: Optional[float] = None) -> float:
        r_norm = np.asarray(r_norm, dtype=float).ravel()
        if r_norm.size == 0 or not np.all(np.isfinite(r_norm)):
            return float("inf")

        mode = str(self.sweep_metric or "").strip().lower()
        if mode in ("median_abs", "median-abs", "median"):
            return float(np.median(np.abs(r_norm)))
        if mode in ("outlier_ratio", "outlier-ratio", "outliers"):
            if inlier_ratio is None or not np.isfinite(inlier_ratio):
                return float("inf")
            return float(max(0.0, 1.0 - float(inlier_ratio)))
        if mode in ("mad", "median_abs_dev", "median-abs-dev"):
            return self._mad_scale(r_norm)
        return float(np.median(np.abs(r_norm)))

    def _pointwise_sweep_info_from_entry(
        self, entry: Dict[str, object], *, scale_override: Optional[float] = None
    ) -> Dict[str, object]:
        sweep_id = str(entry.get("sweep_id", ""))
        violation_penalty = float(entry.get("violation_penalty", 0.0))
        residuals = entry.get("residuals")
        num_points = int(entry.get("num_points") or 0)

        if residuals is None:
            return {
                "sweep_id": sweep_id,
                "valid": False,
                "cost": float("inf"),
                "rms": float("inf"),
                "max_abs": float("inf"),
                "violation_penalty": float(violation_penalty),
                "num_points": int(num_points),
                "num_inliers": None,
                "inlier_ratio": None,
                "scale": None,
                "trim_threshold": None,
                "sweep_metric": float("inf"),
            }

        residuals = np.asarray(residuals, dtype=float).ravel()
        num_points = int(residuals.size)
        r_norm = residuals / float(max(self._l2_scale, 1.0))
        if not self.pointwise_filtering:
            sweep_metric = float(self._sweep_metric_value(r_norm))
            cost, rms, max_abs = self._pointwise_cost_unfiltered(residuals)
            return {
                "sweep_id": sweep_id,
                "valid": np.isfinite(cost),
                "cost": float(cost),
                "rms": float(rms),
                "max_abs": float(max_abs),
                "violation_penalty": float(violation_penalty),
                "num_points": int(num_points),
                "num_inliers": None,
                "inlier_ratio": None,
                "scale": None,
                "trim_threshold": None,
                "sweep_metric": sweep_metric,
            }

        cost_unit, rms, max_abs, inlier_mask, inlier_ratio, scale, trim_threshold = self._pointwise_cost_filtered(
            residuals, scale_override=scale_override
        )
        metric_residuals = r_norm
        if inlier_mask is not None and np.any(inlier_mask):
            metric_residuals = r_norm[inlier_mask]
        sweep_metric = float(self._sweep_metric_value(metric_residuals, inlier_ratio=inlier_ratio))
        if not np.isfinite(cost_unit):
            return {
                "sweep_id": sweep_id,
                "valid": False,
                "cost": float("inf"),
                "rms": float(rms),
                "max_abs": float(max_abs),
                "violation_penalty": float(violation_penalty),
                "num_points": int(num_points),
                "num_inliers": int(np.sum(inlier_mask)) if inlier_mask is not None else None,
                "inlier_ratio": float(inlier_ratio) if np.isfinite(inlier_ratio) else None,
                "scale": float(scale) if np.isfinite(scale) else None,
                "trim_threshold": float(trim_threshold) if np.isfinite(trim_threshold) else None,
                "sweep_metric": sweep_metric if np.isfinite(sweep_metric) else float("inf"),
            }

        cost = float(cost_unit * self.pointwise_cost_weight)
        return {
            "sweep_id": sweep_id,
            "valid": True,
            "cost": float(cost),
            "rms": float(rms),
            "max_abs": float(max_abs),
            "violation_penalty": float(violation_penalty),
            "num_points": int(num_points),
            "num_inliers": int(np.sum(inlier_mask)) if inlier_mask is not None else None,
            "inlier_ratio": float(inlier_ratio) if np.isfinite(inlier_ratio) else None,
            "scale": float(scale) if np.isfinite(scale) else None,
            "trim_threshold": float(trim_threshold) if np.isfinite(trim_threshold) else None,
            "sweep_metric": sweep_metric if np.isfinite(sweep_metric) else float("inf"),
        }

    def _pointwise_sweep_info(
        self,
        sweep: Union[Sweep, dict],
        anchors: np.ndarray,
        *,
        scale_override: Optional[float] = None,
    ) -> Dict[str, object]:
        entry = self._pointwise_sweep_residuals(sweep, anchors)
        return self._pointwise_sweep_info_from_entry(entry, scale_override=scale_override)

    def _pointwise_global_scale(self, residuals_list: List[np.ndarray]) -> Optional[float]:
        if not residuals_list:
            return None
        flat = [r.ravel() for r in residuals_list if r.size and np.all(np.isfinite(r))]
        if not flat:
            return None
        r_norm = np.concatenate([r / float(max(self._l2_scale, 1.0)) for r in flat])
        if r_norm.size == 0 or not np.all(np.isfinite(r_norm)):
            return None
        return self._pointwise_scale(r_norm)

    def _pointwise_entries(
        self, anchors: np.ndarray
    ) -> Tuple[List[Dict[str, object]], Optional[float]]:
        residual_entries = [self._pointwise_sweep_residuals(sweep, anchors) for sweep in self.sweeps]
        scale_override = None
        if self.pointwise_filtering and self.pointwise_global_mad:
            residuals_list = [
                entry["residuals"]
                for entry in residual_entries
                if isinstance(entry.get("residuals"), np.ndarray)
            ]
            scale_override = self._pointwise_global_scale(residuals_list)
            if scale_override is not None and not np.isfinite(scale_override):
                scale_override = None

        entries = [
            self._pointwise_sweep_info_from_entry(entry, scale_override=scale_override)
            for entry in residual_entries
        ]
        return entries, scale_override

    def _sweep_wise_keep_mask(
        self, sweep_metrics: List[float]
    ) -> Tuple[Optional[np.ndarray], Optional[float], Optional[str]]:
        if not self.sweep_wise_filtering:
            return None, None, "disabled"
        metrics = np.asarray(sweep_metrics, dtype=float)
        finite_mask = np.isfinite(metrics)
        count = int(np.sum(finite_mask))
        if count < max(2, _SWEEP_WISE_MIN_SAMPLES):
            return None, None, f"insufficient-samples ({count} < {_SWEEP_WISE_MIN_SAMPLES})"
        data = metrics[finite_mask]
        med = float(np.median(data))
        mad = float(np.median(np.abs(data - med)))
        scale = _MAD_SCALE * mad
        if not np.isfinite(scale) or scale <= 0.0:
            scale = max(med, 1e-12)
        scale = float(max(scale, self._pointwise_sigma_floor_norm, 1e-12))
        threshold = float(max(med + _SWEEP_WISE_K * scale, 1e-12))
        keep = metrics <= threshold
        keep_count = int(np.sum(keep & finite_mask))
        min_keep = int(max(_SWEEP_WISE_MIN_KEEP, np.ceil(_SWEEP_WISE_MIN_KEEP_RATIO * count)))
        reason = "ok"
        if keep_count < min_keep:
            sorted_vals = np.sort(data)
            idx = min(min_keep - 1, sorted_vals.size - 1)
            relaxed = float(sorted_vals[idx])
            if np.isfinite(relaxed):
                threshold = float(max(threshold, relaxed))
                keep = metrics <= threshold
                reason = "relaxed-min-keep"
        if not np.any(keep & finite_mask):
            return None, threshold, "all-rejected"
        return keep, threshold, reason

    def robustness_diagnostics(self, anchor_vec: np.ndarray, *, top_n: int = 5) -> Dict[str, object]:
        """Return per-sweep diagnostics for active robustness filters."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)
        diagnostics: Dict[str, object] = {}

        stage_idx, stage_name, huber_mult, hard_cut = self._pointwise_stage_settings()
        entries, global_scale = self._pointwise_entries(anchors)
        inlier_ratios = [
            e.get("inlier_ratio")
            for e in entries
            if isinstance(e.get("inlier_ratio"), (int, float)) and np.isfinite(e.get("inlier_ratio"))
        ]
        inlier_stats = None
        if inlier_ratios:
            inlier_stats = {
                "min": float(np.min(inlier_ratios)),
                "median": float(np.median(inlier_ratios)),
                "max": float(np.max(inlier_ratios)),
            }

        worst_inliers: List[Dict[str, object]] = []
        if inlier_ratios:
            sortable = [
                (e.get("inlier_ratio"), e)
                for e in entries
                if e.get("inlier_ratio") is not None
            ]
            sortable.sort(key=lambda pair: pair[0])
            for _ratio, entry in sortable[: max(1, int(top_n))]:
                worst_inliers.append(
                    {
                        "sweep_id": entry.get("sweep_id"),
                        "num_inliers": entry.get("num_inliers"),
                        "num_points": entry.get("num_points"),
                        "inlier_ratio": entry.get("inlier_ratio"),
                        "rms": entry.get("rms"),
                    }
                )

        sweep_metrics = [float(e.get("sweep_metric", float("inf"))) for e in entries]
        keep_mask, threshold, reason = self._sweep_wise_keep_mask(sweep_metrics)
        sweep_rejected = 0
        if keep_mask is not None:
            sweep_rejected = int(np.sum(~keep_mask))

        worst_sweeps: List[Dict[str, object]] = []
        if sweep_metrics:
            order = np.argsort(np.asarray(sweep_metrics, dtype=float))[::-1]
            for idx in order[: max(1, int(top_n))]:
                entry = entries[int(idx)]
                worst_sweeps.append(
                    {
                        "sweep_id": entry.get("sweep_id"),
                        "metric": float(entry.get("sweep_metric", float("nan"))),
                        "keep": bool(keep_mask[int(idx)]) if keep_mask is not None else True,
                    }
                )

        diagnostics["pointwise_filtering"] = {
            "enabled": bool(self.pointwise_filtering),
            "stage": int(stage_idx),
            "stage_name": stage_name,
            "huber_multiplier": float(huber_mult),
            "hard_cut": bool(hard_cut),
            "sweeps_total": int(len(entries)),
            "invalid_sweeps": int(len([e for e in entries if not e.get("valid", False)])),
            "inlier_ratio_stats": inlier_stats,
            "worst_inliers": worst_inliers,
            "scale_source": "global" if bool(self.pointwise_global_mad) else "per-sweep",
            "scale_global": float(global_scale) if global_scale is not None else None,
            "scale_floor": float(self._pointwise_sigma_floor_norm),
            "scale_floor_mm": float(self._pointwise_sigma_floor_mm),
            "sigma_noise_mm": float(self._encoder_noise_mm) if self._encoder_noise_mm is not None else None,
            "sigma_mult": float(self._pointwise_sigma_mult),
            "sigma_scaled_mm": float(self._pointwise_sigma_scaled_mm)
            if self._pointwise_sigma_scaled_mm is not None
            else None,
            "sigma_min_mm": float(self._pointwise_sigma_min_mm),
            "sigma_floor_mm": float(self._pointwise_sigma_floor_mm),
            "sigma_floor_source": str(self._pointwise_sigma_floor_source),
        }
        diagnostics["sweep_wise_filtering"] = {
            "enabled": bool(self.sweep_wise_filtering),
            "status": str(reason or "disabled"),
            "metric_mode": str(self.sweep_metric),
            "threshold": float(threshold) if threshold is not None else None,
            "rejected": int(sweep_rejected),
            "worst_sweeps": worst_sweeps,
        }
        return diagnostics

    def _pointwise_predicted_cost(
        self, sweep: Union[Sweep, dict], anchors: np.ndarray
    ) -> Tuple[float, float, float, float, str]:
        """
        Pointwise cost of reconstructed samples against the predicted ellipse.

        Returns (cost, rms, max_abs, violation_penalty, sweep_id).
        """
        info = self._pointwise_sweep_info(sweep, anchors)
        cost = float(info.get("cost", float("inf")))
        rms = float(info.get("rms", float("inf")))
        max_abs = float(info.get("max_abs", float("inf")))
        violation_penalty = float(info.get("violation_penalty", 0.0))
        sweep_id = str(info.get("sweep_id", ""))
        return cost, rms, max_abs, violation_penalty, sweep_id

    def evaluate(self, anchor_vec: np.ndarray) -> float:
        """Compute scalar cost for a flat anchor vector."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)

        if not self.sweeps:
            return 0.0

        weighted_costs: List[float] = []
        weights: List[float] = []

        entries, _global_scale = self._pointwise_entries(anchors)

        sweep_metrics = [float(e.get("sweep_metric", float("inf"))) for e in entries]
        keep_mask, _threshold, _reason = self._sweep_wise_keep_mask(sweep_metrics)

        for idx, entry in enumerate(entries):
            cost = float(entry.get("cost", float("inf")))
            violation_penalty = float(entry.get("violation_penalty", 0.0))
            valid = bool(entry.get("valid", False))

            if not valid or not np.isfinite(cost):
                weights.append(1.0)
                weighted_costs.append(float(self.invalid_penalty) + float(violation_penalty))
                continue

            if keep_mask is not None and not bool(keep_mask[idx]):
                weights.append(0.0)
                weighted_costs.append(0.0)
                continue

            weights.append(1.0)
            weighted_costs.append(float(cost + violation_penalty))

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

        entries, _global_scale = self._pointwise_entries(anchors)

        sweep_metrics = [float(e.get("sweep_metric", float("inf"))) for e in entries]
        keep_mask, _threshold, _reason = self._sweep_wise_keep_mask(sweep_metrics)

        for idx, entry in enumerate(entries):
            sweep_id = str(entry.get("sweep_id", ""))
            cost = float(entry.get("cost", float("inf")))
            violation_penalty = float(entry.get("violation_penalty", 0.0))
            valid = bool(entry.get("valid", False))

            if not valid or not np.isfinite(cost):
                weights[sweep_id] = 1.0
                per_sweep_costs[sweep_id] = float(self.invalid_penalty) + float(violation_penalty)
                num_invalid += 1
                continue

            if keep_mask is not None and not bool(keep_mask[idx]):
                weights[sweep_id] = 0.0
                per_sweep_costs[sweep_id] = 0.0
                num_invalid += 1
                continue

            weights[sweep_id] = 1.0
            per_sweep_costs[sweep_id] = float(cost + violation_penalty)
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
