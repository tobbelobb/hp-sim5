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
_POINTWISE_MIN_INLIERS = 3
_POINTWISE_SIGMA_MULT = 3
_POINTWISE_MIN_SIGMA_MM = 0.05
_SWEEP_WISE_K = 3.0
_SWEEP_WISE_MIN_SAMPLES = 5
_SWEEP_WISE_MIN_KEEP = 2
_SWEEP_WISE_MIN_KEEP_RATIO = 0.5
_UNDERCONSTRAINED_PENALTY = 100.0  # Soft-reject: good costs are ~0-5, so 100 is ~20x worse yet finite for optimizers.
_MIN_SWEEPS_AFTER_TRIM_DEFAULT = 3
_MIN_SWEEPS_AFTER_TRIM_BY_MACHINE = {
    "slideprinter": 3,
    "hangprinter_4": 3,
    "hangprinter_5": 3,
    "cubecorners": 3,
    "skycam": 3,
}
_NOISE_MODEL_CONFIG_KEY = "noise_model"
_DEFAULT_LAYER_LINE_WIDTH_MM = 1.0
_DEFAULT_FORCE_MIN_TO_SIGMA_MM_PER_N = 10.0
_DEFAULT_FORCE_SPAN_TO_SIGMA_MM_PER_N = 0.1
_MODE_SIGMA_FACTORS = {
    ("global", "off"): 2.0,
    ("global", "global"): 5.0,
    ("per-anchor", "global"): 10.0,
}


@dataclass
class CostResult:
    """Breakdown of a cost function evaluation."""

    total_cost: float
    per_sweep_costs: Dict[str, float]
    num_valid_sweeps: int
    num_invalid_sweeps: int
    anchor_estimate: np.ndarray
    noise_metrics: Optional[Dict[str, object]] = None


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
        use_noise_mean: bool = True,
        noise_normalized: bool = True,
        sigma_source: str = "auto",
    ) -> None:
        (
            self.machine_type,
            self.num_anchors,
            self.dimensions,
            self.sweeps,
        ) = _dataset_metadata(dataset)
        machine_key = str(self.machine_type).lower()
        self._min_sweeps_after_trim = int(
            _MIN_SWEEPS_AFTER_TRIM_BY_MACHINE.get(machine_key, _MIN_SWEEPS_AFTER_TRIM_DEFAULT)
        )

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
        self.use_noise_mean = bool(use_noise_mean)
        self.noise_normalized = bool(noise_normalized)
        self.sigma_source = str(sigma_source or "auto").strip().lower()
        if self.sigma_source not in ("auto", "point", "origin", "min"):
            self.sigma_source = "auto"
        self._mm_per_degree_by_axis = self._extract_mm_per_degree_by_axis(dataset, self.num_anchors)
        self._sigma_floor_deg = self._extract_sigma_floor_deg(dataset)
        self._sigma_floor_mm_by_axis = None
        if self._sigma_floor_deg is not None and self._mm_per_degree_by_axis is not None:
            self._sigma_floor_mm_by_axis = [
                float(self._sigma_floor_deg) * float(v) if np.isfinite(v) else float("nan")
                for v in self._mm_per_degree_by_axis
            ]
        self._has_point_sigma = self._dataset_has_sigma(dataset)

        lb, ub = get_anchor_bounds(self.machine_type)
        ub_mat = np.asarray(ub, dtype=float).reshape(self.num_anchors, self.dimensions)
        max_anchor_norm = float(np.max(np.linalg.norm(ub_mat, axis=1))) if ub_mat.size else 1.0
        self._length_scale = float(max_anchor_norm) if np.isfinite(max_anchor_norm) else 1.0
        self._l2_scale = float((2.0 * max_anchor_norm) ** 2)
        raw_noise_mm = self._infer_encoder_noise_mm(dataset)
        if raw_noise_mm is None or not np.isfinite(raw_noise_mm) or raw_noise_mm < 0.0:
            raw_noise_mm = None
        self._encoder_noise_mm = raw_noise_mm
        self._noise_norm_available = bool(self.noise_normalized)
        if self._noise_norm_available:
            self.pointwise_cost_weight = 1.0
        self._pointwise_sigma_mult = float(_POINTWISE_SIGMA_MULT)
        self._sigma_components = self._build_sigma_components(
            dataset,
            encoder_noise_mm=raw_noise_mm,
            default_floor_mm=float(_POINTWISE_MIN_SIGMA_MM),
        )
        sigma_min_floor = self._sigma_components.get("sigma_floor_term_mm")
        if not isinstance(sigma_min_floor, (int, float)) or not np.isfinite(float(sigma_min_floor)):
            sigma_min_floor = float(_POINTWISE_MIN_SIGMA_MM)
        self._pointwise_sigma_min_mm = float(max(float(sigma_min_floor), 1e-9))
        sigma_model_floor = self._sigma_components.get("sigma_total_mm")
        if not isinstance(sigma_model_floor, (int, float)) or not np.isfinite(float(sigma_model_floor)):
            sigma_model_floor = float(self._pointwise_sigma_min_mm)
        self._pointwise_sigma_model_mm = float(max(float(sigma_model_floor), self._pointwise_sigma_min_mm))
        sigma_scaled_mm = None
        if raw_noise_mm is not None:
            sigma_scaled_mm = float(self._pointwise_sigma_mult * raw_noise_mm)
        self._pointwise_sigma_scaled_mm = sigma_scaled_mm
        sigma_used_override_mm = self._sigma_components.get("sigma_used_override_mm")
        if (
            isinstance(sigma_used_override_mm, (int, float))
            and np.isfinite(float(sigma_used_override_mm))
            and float(sigma_used_override_mm) > 0.0
        ):
            sigma_floor_mm = float(sigma_used_override_mm)
            sigma_source = "override"
        elif sigma_scaled_mm is not None and sigma_scaled_mm >= self._pointwise_sigma_model_mm:
            sigma_floor_mm = float(sigma_scaled_mm)
            sigma_source = "noise"
        else:
            sigma_floor_mm = float(self._pointwise_sigma_model_mm)
            if sigma_floor_mm > self._pointwise_sigma_min_mm + 1e-12:
                sigma_source = "model"
            else:
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

    def _extract_sweep_arrays(
        self, sweep: Union[Sweep, dict]
    ) -> Tuple[
        List[int],
        List[float],
        int,
        int,
        np.ndarray,
        np.ndarray,
        str,
        Optional[np.ndarray],
        Optional[np.ndarray],
    ]:
        """Extract sweep role metadata and drive/sensor arrays."""
        if isinstance(sweep, Sweep):
            fixed_indices = list(sweep.fixed_anchors)
            fixed_deltas = list(sweep.fixed_lengths)
            drive_idx = sweep.drive_anchor
            sensor_idx = sweep.sensor_anchor
            data_points = list(sweep.data_points or [])
            sweep_id = sweep.id
        else:
            fixed_indices = list(sweep.get("fixed_anchors", []))
            fixed_deltas = list(sweep.get("fixed_lengths", []))
            drive_idx = int(sweep.get("drive_anchor"))
            sensor_idx = int(sweep.get("sensor_anchor"))
            data_points = sweep.get("data_points", [])
            if not isinstance(data_points, list):
                data_points = []
            sweep_id = sweep.get("id", "")

        l_drive, l_sensor, sigma_drive, sigma_sensor = self._extract_point_arrays(
            data_points, int(drive_idx), int(sensor_idx)
        )
        return (
            fixed_indices,
            fixed_deltas,
            int(drive_idx),
            int(sensor_idx),
            l_drive,
            l_sensor,
            str(sweep_id),
            sigma_drive,
            sigma_sensor,
        )

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
            sigma_drive_mm,
            sigma_sensor_mm,
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
        return (
            fixed_lengths_abs,
            drive_idx,
            sensor_idx,
            l_drive_abs,
            l_sensor_abs,
            sweep_id,
            penalty,
            sigma_drive_mm,
            sigma_sensor_mm,
        )

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

        noise_mm = pick("encoder_noise_origin_mm", "encoder_noise_origin_mm_per_axis")
        if noise_mm is not None:
            return float(noise_mm)

        noise_deg = pick("encoder_noise_origin_deg", "encoder_noise_origin_deg_per_axis")
        if noise_deg is None:
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

    @classmethod
    def _extract_mm_per_degree_by_axis(
        cls, dataset: Union[dict, "SweepDataset"], num_anchors: int
    ) -> Optional[List[float]]:
        if not isinstance(dataset, dict):
            return None
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return None
        raw = config.get("mm_per_degree")
        if raw is None:
            raw = config.get("mm_per_degree_by_axis")
        if raw is None:
            return None

        values: List[object]
        if isinstance(raw, dict):
            keyed: List[Tuple[int, object]] = []
            for key, val in raw.items():
                try:
                    idx = int(key)
                except (TypeError, ValueError):
                    continue
                keyed.append((idx, val))
            if keyed:
                keyed.sort(key=lambda pair: pair[0])
                values = [val for _idx, val in keyed]
            else:
                values = list(raw.values())
        elif isinstance(raw, (list, tuple, np.ndarray)):
            values = list(raw)
        else:
            values = [raw]

        out: List[float] = []
        for v in values:
            try:
                f = float(v)
            except (TypeError, ValueError):
                f = float("nan")
            out.append(f if np.isfinite(f) else float("nan"))

        if not out:
            return None

        count = int(num_anchors)
        if count <= 0:
            return None
        if len(out) < count:
            if len(out) == 1:
                out = out * count
            else:
                out = out + [out[-1]] * (count - len(out))
        if len(out) > count:
            out = out[:count]
        return out

    @classmethod
    def _extract_sigma_floor_deg(cls, dataset: Union[dict, "SweepDataset"]) -> Optional[float]:
        if not isinstance(dataset, dict):
            return None
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return None
        noise_cfg = config.get("encoder_noise")
        if not isinstance(noise_cfg, dict):
            return None
        floor_raw = noise_cfg.get("sigma_floor_deg")
        if isinstance(floor_raw, (int, float)) and np.isfinite(floor_raw):
            return float(floor_raw)
        return None

    @staticmethod
    def _normalize_find_mode(mode: object) -> str:
        out = str(mode or "off").strip().lower()
        if out in ("off", "global", "per-anchor"):
            return out
        return "off"

    @classmethod
    def _noise_model_config(cls, dataset: Union[dict, "SweepDataset"]) -> Dict[str, object]:
        if not isinstance(dataset, dict):
            return {}
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return {}
        raw = config.get(_NOISE_MODEL_CONFIG_KEY)
        if not isinstance(raw, dict):
            return {}
        return dict(raw)

    @classmethod
    def _infer_base_radius_mm(cls, dataset: Union[dict, "SweepDataset"]) -> Optional[float]:
        if not isinstance(dataset, dict):
            return None
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return None
        m666_before = config.get("m666_before")
        m666 = config.get("m666")
        for src in (m666_before, m666):
            if isinstance(src, dict):
                vals = cls._flatten_config_values(src.get("R"))
                mean = cls._mean_finite(vals)
                if mean is not None and np.isfinite(mean) and mean > 0.0:
                    return float(mean)
        return None

    @classmethod
    def _infer_layered_enabled(
        cls,
        dataset: Union[dict, "SweepDataset"],
        noise_model: Dict[str, object],
        *,
        mode_r: str,
        mode_b: str,
    ) -> bool:
        explicit = noise_model.get("layered")
        if isinstance(explicit, bool):
            return bool(explicit)
        if mode_r != "off" or mode_b != "off":
            return True
        if not isinstance(dataset, dict):
            return False
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return False

        def _q_abs(src_key: str) -> Optional[float]:
            src = config.get(src_key)
            if not isinstance(src, dict):
                return None
            vals = cls._flatten_config_values(src.get("Q"))
            mean = cls._mean_finite(vals)
            if mean is None:
                return None
            return abs(float(mean))

        for key in ("m666_before", "m666"):
            q_abs = _q_abs(key)
            if q_abs is not None and np.isfinite(q_abs) and q_abs > 1e-9:
                return True
        return False

    @classmethod
    def _infer_force_levels_n(
        cls, dataset: Union[dict, "SweepDataset"]
    ) -> Tuple[Optional[float], Optional[float]]:
        if not isinstance(dataset, dict):
            return None, None
        config = dataset.get("config", {}) or {}
        if not isinstance(config, dict):
            return None, None
        tuning = config.get("force_tuning")
        if not isinstance(tuning, dict):
            tuning = config.get("torque_tuning")
        if not isinstance(tuning, dict):
            return None, None

        def pick(*keys: str) -> Optional[float]:
            for key in keys:
                vals = cls._flatten_config_values(tuning.get(key))
                mean = cls._mean_finite(vals)
                if mean is not None and np.isfinite(mean):
                    return float(mean)
            return None

        force_low = pick("force_low_n", "torque_low_n")
        force_start = pick("force_start", "min_force_n")
        force_min = force_low if force_low is not None else force_start
        force_max = pick("force_max_n", "torque_max_n")
        force_mid = pick("force_mid_n", "torque_mid_n")
        force_span = None
        if force_max is not None and force_low is not None:
            force_span = max(float(force_max) - float(force_low), 0.0)
        elif force_min is not None and force_max is not None:
            force_span = max(float(force_max) - float(force_min), 0.0)
        elif force_min is not None and force_mid is not None:
            force_span = max(float(force_mid) - float(force_min), 0.0)
        return force_min, force_span

    def _infer_zero_tension_flex_sigma_mm(self, dataset: Union[dict, "SweepDataset"]) -> Optional[float]:
        if not isinstance(dataset, dict):
            return None
        if self._mm_per_degree_by_axis is None or not self._mm_per_degree_by_axis:
            return None
        sweeps = dataset.get("sweeps")
        if not isinstance(sweeps, list):
            return None
        diffs_mm: List[float] = []
        for sweep in sweeps:
            if not isinstance(sweep, dict):
                continue
            points = sweep.get("data_points")
            if not isinstance(points, list):
                continue
            for point in points:
                if not isinstance(point, dict):
                    continue
                raw = point.get("raw_angles_deg")
                zero = point.get("raw_angles_zero_tension_deg")
                if not isinstance(raw, list) or not isinstance(zero, list):
                    continue
                count = min(len(raw), len(zero), len(self._mm_per_degree_by_axis))
                for axis in range(count):
                    mm_per = self._mm_per_degree_by_axis[axis]
                    if not np.isfinite(mm_per):
                        continue
                    try:
                        delta_deg = float(zero[axis]) - float(raw[axis])
                    except (TypeError, ValueError):
                        continue
                    if not np.isfinite(delta_deg):
                        continue
                    diffs_mm.append(abs(float(delta_deg) * float(mm_per)))
        if not diffs_mm:
            return None
        arr = np.asarray(diffs_mm, dtype=float)
        arr = arr[np.isfinite(arr)]
        if arr.size == 0:
            return None
        return float(np.median(arr))

    @staticmethod
    def _coerce_nonnegative(value: object, *, default: float) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            out = float(default)
        if not np.isfinite(out) or out < 0.0:
            out = float(default)
        return float(out)

    def _build_sigma_components(
        self,
        dataset: Union[dict, "SweepDataset"],
        *,
        encoder_noise_mm: Optional[float],
        default_floor_mm: float,
    ) -> Dict[str, object]:
        noise_model = self._noise_model_config(dataset)
        mode_r = self._normalize_find_mode(noise_model.get("find_radii_mode"))
        mode_b = self._normalize_find_mode(noise_model.get("find_buildup_mode"))
        solver_mode = f"{mode_r}/{mode_b}"
        mode_factor = float(_MODE_SIGMA_FACTORS.get((mode_r, mode_b), 0.0))

        sigma_encoder = 0.0
        if encoder_noise_mm is not None and np.isfinite(encoder_noise_mm) and encoder_noise_mm > 0.0:
            sigma_encoder = float(encoder_noise_mm)

        force_min_n, force_span_n = self._infer_force_levels_n(dataset)
        friction_gain = self._coerce_nonnegative(
            noise_model.get("friction_sigma_mm_per_n"),
            default=_DEFAULT_FORCE_MIN_TO_SIGMA_MM_PER_N,
        )
        flex_gain = self._coerce_nonnegative(
            noise_model.get("flex_sigma_mm_per_n"),
            default=_DEFAULT_FORCE_SPAN_TO_SIGMA_MM_PER_N,
        )
        sigma_friction = 0.0
        if force_min_n is not None and np.isfinite(force_min_n):
            sigma_friction = float(max(float(force_min_n), 0.0) * float(friction_gain))

        sigma_flex = 0.0
        sigma_flex_source = "none"
        sigma_zero_tension = self._infer_zero_tension_flex_sigma_mm(dataset)
        if sigma_zero_tension is not None and np.isfinite(sigma_zero_tension) and sigma_zero_tension > 0.0:
            sigma_flex = float(sigma_zero_tension)
            sigma_flex_source = "zero_tension"
        elif force_span_n is not None and np.isfinite(force_span_n):
            sigma_flex = float(max(float(force_span_n), 0.0) * float(flex_gain))
            sigma_flex_source = "force_span"

        sigma_floor_override = noise_model.get("sigma_floor_mm")
        sigma_floor_term = self._coerce_nonnegative(
            sigma_floor_override,
            default=self._coerce_nonnegative(default_floor_mm, default=float(_POINTWISE_MIN_SIGMA_MM)),
        )
        if isinstance(sigma_floor_override, (int, float)) and (not np.isfinite(sigma_floor_override) or sigma_floor_override <= 0.0):
            sigma_floor_term = self._coerce_nonnegative(default_floor_mm, default=float(_POINTWISE_MIN_SIGMA_MM))
        sigma_non_layered = float(
            np.sqrt(
                sigma_encoder**2
                + sigma_friction**2
                + sigma_flex**2
                + sigma_floor_term**2
            )
        )
        sigma_used_override_mm: Optional[float] = None
        sigma_used_override = noise_model.get("sigma_used_mm")
        if isinstance(sigma_used_override, (int, float)):
            sigma_used_val = float(sigma_used_override)
            if np.isfinite(sigma_used_val) and sigma_used_val > 0.0:
                sigma_used_override_mm = float(sigma_used_val)

        if isinstance(dataset, dict):
            config = dataset.get("config", {}) or {}
        else:
            config = {}
        line_width_default = _DEFAULT_LAYER_LINE_WIDTH_MM
        if isinstance(config, dict):
            m666 = config.get("m666")
            if isinstance(m666, dict):
                raw_w = m666.get("W")
                if isinstance(raw_w, (int, float)) and np.isfinite(raw_w) and raw_w >= 0.0:
                    line_width_default = float(raw_w)
        line_width_mm = self._coerce_nonnegative(
            noise_model.get("line_width_mm"),
            default=float(line_width_default),
        )
        if not np.isfinite(line_width_mm):
            line_width_mm = float(_DEFAULT_LAYER_LINE_WIDTH_MM)

        base_radius_mm = self._infer_base_radius_mm(dataset)
        if base_radius_mm is None or not np.isfinite(base_radius_mm) or base_radius_mm <= 0.0:
            base_radius_mm = 1.0
        layered = self._infer_layered_enabled(
            dataset,
            noise_model,
            mode_r=mode_r,
            mode_b=mode_b,
        )

        sigma_layer_changes = 0.0
        if bool(layered) and line_width_mm > 0.0:
            ratio = float(line_width_mm / max(float(base_radius_mm), 1e-9))
            sigma_layer_changes = float(line_width_mm * (1.0 + ratio) / np.sqrt(12.0))

        sigma_mode_addition = float(mode_factor * sigma_layer_changes) if bool(layered) else 0.0
        sigma_layered = float(
            np.sqrt(
                sigma_non_layered**2
                + sigma_layer_changes**2
                + sigma_mode_addition**2
            )
        )
        sigma_total = float(sigma_layered if bool(layered) else sigma_non_layered)
        return {
            "sigma_encoder_mm": float(sigma_encoder),
            "sigma_friction_cogging_mm": float(sigma_friction),
            "sigma_flex_mm": float(sigma_flex),
            "sigma_flex_source": str(sigma_flex_source),
            "sigma_floor_term_mm": float(sigma_floor_term),
            "sigma_used_override_mm": sigma_used_override_mm,
            "sigma_non_layered_mm": float(sigma_non_layered),
            "sigma_layer_changes_mm": float(sigma_layer_changes),
            "sigma_mode_addition_mm": float(sigma_mode_addition),
            "sigma_layered_mm": float(sigma_layered),
            "sigma_total_mm": float(sigma_total),
            "sigma_layered_enabled": bool(layered),
            "sigma_solver_mode": str(solver_mode),
            "sigma_solver_mode_factor": float(mode_factor),
            "sigma_line_width_mm": float(line_width_mm),
            "sigma_base_radius_mm": float(base_radius_mm),
            "sigma_force_min_n": (
                None
                if force_min_n is None or not np.isfinite(force_min_n)
                else float(force_min_n)
            ),
            "sigma_force_span_n": (
                None
                if force_span_n is None or not np.isfinite(force_span_n)
                else float(force_span_n)
            ),
        }

    @staticmethod
    def _dataset_has_sigma(dataset: Union[dict, "SweepDataset"]) -> bool:
        sweeps: Iterable[object]
        if hasattr(dataset, "sweeps"):
            sweeps = getattr(dataset, "sweeps", []) or []
        elif isinstance(dataset, dict):
            sweeps = dataset.get("sweeps", []) or []
        else:
            return False
        for sweep in sweeps:
            if isinstance(sweep, dict):
                points = sweep.get("data_points", []) or []
            else:
                points = getattr(sweep, "data_points", []) or []
            for point in points:
                sigmas = point.get("sigma") if isinstance(point, dict) else getattr(point, "sigma", None)
                if isinstance(sigmas, np.ndarray):
                    sig_vals = sigmas.tolist()
                elif isinstance(sigmas, (list, tuple)):
                    sig_vals = list(sigmas)
                else:
                    sig_vals = []
                for val in sig_vals:
                    try:
                        f = float(val)
                    except (TypeError, ValueError):
                        continue
                    if np.isfinite(f):
                        return True
        return False

    @staticmethod
    def _point_field(point: Union[Sweep, dict, object], name: str) -> Optional[object]:
        if isinstance(point, dict):
            return point.get(name)
        return getattr(point, name, None)

    def _sigma_deg_for_point(self, point: Union[Sweep, dict, object], axis_idx: int) -> Optional[float]:
        sigma_raw = self._point_field(point, "sigma")
        if isinstance(sigma_raw, np.ndarray):
            sigma_vals = sigma_raw.tolist()
        elif isinstance(sigma_raw, (list, tuple)):
            sigma_vals = list(sigma_raw)
        else:
            sigma_vals = []
        if axis_idx < 0 or axis_idx >= len(sigma_vals):
            return None
        try:
            sigma_deg = float(sigma_vals[axis_idx])
        except (TypeError, ValueError):
            return None
        if not np.isfinite(sigma_deg):
            return None
        return sigma_deg

    def _sigma_source_for_point(self, point: Union[Sweep, dict, object], axis_idx: int) -> str:
        source = self.sigma_source
        if source == "min":
            return "min"
        if source == "origin":
            return "origin" if self._encoder_noise_mm is not None and np.isfinite(self._encoder_noise_mm) else "min"

        sigma_deg = self._sigma_deg_for_point(point, axis_idx)
        has_mm = (
            self._mm_per_degree_by_axis is not None
            and 0 <= axis_idx < len(self._mm_per_degree_by_axis)
            and np.isfinite(self._mm_per_degree_by_axis[axis_idx])
        )
        point_available = sigma_deg is not None and has_mm

        if source == "point":
            return "point" if point_available else "min"

        if point_available:
            return "point"
        if self._encoder_noise_mm is not None and np.isfinite(self._encoder_noise_mm):
            return "origin"
        return "min"

    def _length_from_mu(self, point: Union[Sweep, dict, object], axis_idx: int) -> Optional[float]:
        mu_raw = self._point_field(point, "mu")
        if isinstance(mu_raw, np.ndarray):
            mu_vals = mu_raw.tolist()
        elif isinstance(mu_raw, (list, tuple)):
            mu_vals = list(mu_raw)
        else:
            return None
        if axis_idx < 0 or axis_idx >= len(mu_vals):
            return None
        if self._mm_per_degree_by_axis is None or axis_idx >= len(self._mm_per_degree_by_axis):
            return None
        mm_per = self._mm_per_degree_by_axis[axis_idx]
        if not np.isfinite(mm_per):
            return None
        try:
            val = float(mu_vals[axis_idx]) * float(mm_per)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(val):
            return None
        return float(val)

    def _sigma_mm_for_point(self, point: Union[Sweep, dict, object], axis_idx: int) -> Optional[float]:
        source = self.sigma_source
        sigma_min = float(self._pointwise_sigma_min_mm)
        if source == "min":
            return sigma_min

        if source == "origin":
            if self._encoder_noise_mm is not None and np.isfinite(self._encoder_noise_mm):
                sigma_mm = float(self._encoder_noise_mm)
            else:
                sigma_mm = sigma_min
            return float(max(sigma_mm, sigma_min))

        sigma_deg = self._sigma_deg_for_point(point, axis_idx)

        floor_deg = self._sigma_floor_deg
        if sigma_deg is not None and floor_deg is not None and np.isfinite(floor_deg):
            sigma_deg = max(sigma_deg, float(floor_deg))

        if self._mm_per_degree_by_axis is not None and 0 <= axis_idx < len(self._mm_per_degree_by_axis):
            mm_per = self._mm_per_degree_by_axis[axis_idx]
            if np.isfinite(mm_per) and sigma_deg is not None:
                sigma_mm = float(sigma_deg) * float(mm_per)
                return float(max(sigma_mm, sigma_min))

        if source == "point":
            return sigma_min

        if self._encoder_noise_mm is not None and np.isfinite(self._encoder_noise_mm):
            sigma_mm = float(self._encoder_noise_mm)
        else:
            sigma_mm = sigma_min
        if not np.isfinite(sigma_mm):
            sigma_mm = sigma_min
        return float(max(sigma_mm, sigma_min))

    def _extract_point_arrays(
        self,
        data_points: Iterable[Union[dict, object]],
        drive_idx: int,
        sensor_idx: int,
    ) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
        l_drive_vals: List[float] = []
        l_sensor_vals: List[float] = []
        sigma_drive_vals: List[float] = []
        sigma_sensor_vals: List[float] = []

        for point in data_points:
            raw_drive = self._point_field(point, "l_drive")
            raw_sensor = self._point_field(point, "l_sensor")
            try:
                drive_len = float(raw_drive)
            except (TypeError, ValueError):
                drive_len = 0.0
            try:
                sensor_len = float(raw_sensor)
            except (TypeError, ValueError):
                sensor_len = 0.0

            if self.use_noise_mean:
                drive_mu = self._point_field(point, "l_drive_mu")
                sensor_mu = self._point_field(point, "l_sensor_mu")
                if drive_mu is None:
                    drive_mu = self._length_from_mu(point, drive_idx)
                if sensor_mu is None:
                    sensor_mu = self._length_from_mu(point, sensor_idx)
                try:
                    if drive_mu is not None and np.isfinite(float(drive_mu)):
                        drive_len = float(drive_mu)
                except (TypeError, ValueError):
                    pass
                try:
                    if sensor_mu is not None and np.isfinite(float(sensor_mu)):
                        sensor_len = float(sensor_mu)
                except (TypeError, ValueError):
                    pass

            l_drive_vals.append(float(drive_len))
            l_sensor_vals.append(float(sensor_len))

            sigma_drive = self._sigma_mm_for_point(point, drive_idx)
            sigma_sensor = self._sigma_mm_for_point(point, sensor_idx)
            sigma_drive_vals.append(float(sigma_drive) if sigma_drive is not None else float("nan"))
            sigma_sensor_vals.append(float(sigma_sensor) if sigma_sensor is not None else float("nan"))

        l_drive = np.asarray(l_drive_vals, dtype=float)
        l_sensor = np.asarray(l_sensor_vals, dtype=float)
        sigma_drive = np.asarray(sigma_drive_vals, dtype=float) if sigma_drive_vals else None
        sigma_sensor = np.asarray(sigma_sensor_vals, dtype=float) if sigma_sensor_vals else None
        return l_drive, l_sensor, sigma_drive, sigma_sensor

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

    def _sigma_l2_for_points(
        self,
        coeffs: np.ndarray,
        l_drive_abs: np.ndarray,
        l_sensor_abs: np.ndarray,
        sigma_drive_mm: np.ndarray,
        sigma_sensor_mm: np.ndarray,
    ) -> Optional[np.ndarray]:
        if not bool(self._noise_norm_available):
            return None
        if l_drive_abs.size == 0 or l_sensor_abs.size == 0:
            return None
        sigma_drive_mm = np.asarray(sigma_drive_mm, dtype=float).ravel()
        sigma_sensor_mm = np.asarray(sigma_sensor_mm, dtype=float).ravel()
        if sigma_drive_mm.size != l_drive_abs.size or sigma_sensor_mm.size != l_sensor_abs.size:
            return None
        if not np.all(np.isfinite(sigma_drive_mm)) or not np.all(np.isfinite(sigma_sensor_mm)):
            return None

        x = np.asarray(l_drive_abs, dtype=float).ravel() ** 2
        y = np.asarray(l_sensor_abs, dtype=float).ravel() ** 2
        A, B, C, D, E, _F = np.asarray(coeffs, dtype=float).reshape(6)
        grad_x = 2 * A * x + B * y + D
        grad_y = B * x + 2 * C * y + E
        grad_norm = np.sqrt(grad_x**2 + grad_y**2)
        grad_norm = np.where(grad_norm < 1e-12, 1e-12, grad_norm)
        n_x = grad_x / grad_norm
        n_y = grad_y / grad_norm

        sigma_x = 2.0 * np.asarray(l_drive_abs, dtype=float).ravel() * sigma_drive_mm
        sigma_y = 2.0 * np.asarray(l_sensor_abs, dtype=float).ravel() * sigma_sensor_mm
        sigma_l2 = np.sqrt((n_x**2) * (sigma_x**2) + (n_y**2) * (sigma_y**2))
        if not np.all(np.isfinite(sigma_l2)):
            return None
        sigma_l2 = np.where(sigma_l2 < 1e-12, 1e-12, sigma_l2)
        return sigma_l2

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
        self,
        residuals: np.ndarray,
        *,
        scale_override: Optional[float] = None,
        r_norm_override: Optional[np.ndarray] = None,
    ) -> Tuple[float, float, float, Optional[np.ndarray], float, float, float]:
        residuals = np.asarray(residuals, dtype=float).ravel()
        if residuals.size == 0 or not np.all(np.isfinite(residuals)):
            return float("inf"), float("inf"), float("inf"), None, float("nan"), float("nan"), float("nan")

        rms = float(np.sqrt(np.mean(residuals**2)))
        max_abs = float(np.max(np.abs(residuals)))

        if r_norm_override is None:
            r_norm = residuals / float(max(self._l2_scale, 1.0))
        else:
            r_norm = np.asarray(r_norm_override, dtype=float).ravel()
            if r_norm.size != residuals.size or not np.all(np.isfinite(r_norm)):
                return float("inf"), rms, max_abs, None, float("nan"), float("nan"), float("nan")
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
                # Fallback: keep cost finite even if trimming rejects everything.
                inlier_ratio = float(inlier_count) / float(r_norm.size) if r_norm.size else 0.0
                r_used = r_norm
            else:
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
            sigma_drive_mm,
            sigma_sensor_mm,
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

        sigma_l2 = None
        if (
            residuals is not None
            and bool(self._noise_norm_available)
            and sigma_drive_mm is not None
            and sigma_sensor_mm is not None
        ):
            sigma_l2 = self._sigma_l2_for_points(
                coeffs,
                l_drive_abs,
                l_sensor_abs,
                sigma_drive_mm,
                sigma_sensor_mm,
            )

        return {
            "sweep_id": str(sweep_id),
            "residuals": residuals,
            "sigma_l2": sigma_l2,
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
                sigma_drive_mm,
                sigma_sensor_mm,
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
            sigma_l2 = self._sigma_l2_for_points(
                coeffs,
                l_drive_abs,
                l_sensor_abs,
                sigma_drive_mm,
                sigma_sensor_mm,
            )

            sweep_cache.append(
                {
                    "sweep_id": str(sweep_id),
                    "drive_anchor": int(drive_idx2),
                    "sensor_anchor": int(sensor_idx2),
                    "l_drive_abs": l_drive_abs,
                    "l_sensor_abs": l_sensor_abs,
                    "residuals": residuals,
                    "sigma_l2": sigma_l2,
                }
            )
            residuals_list.append(residuals)

        scale_override = None
        if self.pointwise_filtering and self.pointwise_global_mad and residuals_list:
            norm_residuals: List[np.ndarray] = []
            for entry in sweep_cache:
                residuals = np.asarray(entry["residuals"], dtype=float).ravel()
                sigma_l2 = entry.get("sigma_l2")
                if bool(self._noise_norm_available) and isinstance(sigma_l2, np.ndarray):
                    sigma_arr = np.asarray(sigma_l2, dtype=float).ravel()
                    if sigma_arr.size == residuals.size and np.all(np.isfinite(sigma_arr)):
                        norm_residuals.append(residuals / sigma_arr)
                        continue
                norm_residuals.append(residuals / float(max(self._l2_scale, 1.0)))
            scale_override = self._pointwise_global_scale(norm_residuals)
            if scale_override is not None and not np.isfinite(scale_override):
                scale_override = None

        for entry in sweep_cache:
            residuals = np.asarray(entry["residuals"], dtype=float).ravel()
            l_drive_abs = np.asarray(entry["l_drive_abs"], dtype=float).ravel()
            l_sensor_abs = np.asarray(entry["l_sensor_abs"], dtype=float).ravel()
            sweep_id = str(entry["sweep_id"])
            drive_idx2 = int(entry["drive_anchor"])
            sensor_idx2 = int(entry["sensor_anchor"])
            sigma_l2 = entry.get("sigma_l2")

            l_mean = 0.5 * (l_drive_abs + l_sensor_abs)
            denom = np.maximum(2.0 * l_mean, 1e-9)
            residuals_mm_signed = residuals / denom
            residuals_abs = np.abs(residuals)
            residuals_mm = residuals_abs / denom

            use_sigma = False
            r_norm = residuals / l2_scale
            if bool(self._noise_norm_available) and isinstance(sigma_l2, np.ndarray):
                sigma_arr = np.asarray(sigma_l2, dtype=float).ravel()
                if sigma_arr.size == residuals.size and np.all(np.isfinite(sigma_arr)):
                    r_norm = residuals / sigma_arr
                    use_sigma = True

            scale = self._pointwise_scale(r_norm, scale_override=scale_override)
            trim_threshold = float(max(_POINTWISE_TRIM_K * scale, 1e-12))
            if use_sigma:
                cutoff_mm = (trim_threshold * sigma_arr) / denom
            else:
                cutoff_mm = (trim_threshold * l2_scale) / denom

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
                        "residual_l2_signed": float(residuals[idx]),
                        "residual_l2": float(res_l2),
                        "residual_mm_signed": float(residuals_mm_signed[idx]),
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

    def _pointwise_cost_unfiltered(
        self, residuals: np.ndarray, *, r_norm_override: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float]:
        residuals = np.asarray(residuals, dtype=float).ravel()
        if residuals.size == 0 or not np.all(np.isfinite(residuals)):
            return float("inf"), float("inf"), float("inf")
        rms = float(np.sqrt(np.mean(residuals**2)))
        max_abs = float(np.max(np.abs(residuals)))
        if r_norm_override is None:
            r_norm = residuals / float(max(self._l2_scale, 1.0))
        else:
            r_norm = np.asarray(r_norm_override, dtype=float).ravel()
            if r_norm.size != residuals.size or not np.all(np.isfinite(r_norm)):
                return float("inf"), float("inf"), float("inf")
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
        if num_points < int(self.min_points):
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
        sigma_l2 = entry.get("sigma_l2")
        norm_mode = "l2_scale"
        r_norm = residuals / float(max(self._l2_scale, 1.0))
        if bool(self._noise_norm_available) and isinstance(sigma_l2, np.ndarray):
            sigma_arr = np.asarray(sigma_l2, dtype=float).ravel()
            if sigma_arr.size == residuals.size and np.all(np.isfinite(sigma_arr)):
                r_norm = residuals / sigma_arr
                norm_mode = "sigma"
        if not self.pointwise_filtering:
            sweep_metric = float(self._sweep_metric_value(r_norm))
            cost, rms, max_abs = self._pointwise_cost_unfiltered(residuals, r_norm_override=r_norm)
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
                "norm_mode": norm_mode,
                "_r_norm": r_norm,
                "_r_norm_used": r_norm,
            }

        cost_unit, rms, max_abs, inlier_mask, inlier_ratio, scale, trim_threshold = self._pointwise_cost_filtered(
            residuals, scale_override=scale_override, r_norm_override=r_norm
        )
        r_norm_used = r_norm
        if inlier_mask is not None and np.any(inlier_mask):
            if int(np.sum(inlier_mask)) >= _POINTWISE_MIN_INLIERS:
                r_norm_used = r_norm[inlier_mask]
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
                "norm_mode": norm_mode,
                "_r_norm": r_norm,
                "_r_norm_used": r_norm_used,
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
            "norm_mode": norm_mode,
            "_r_norm": r_norm,
            "_r_norm_used": r_norm_used,
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
        r_norm = np.concatenate(flat)
        if r_norm.size == 0 or not np.all(np.isfinite(r_norm)):
            return None
        return self._pointwise_scale(r_norm)

    def _pointwise_entries(
        self, anchors: np.ndarray
    ) -> Tuple[List[Dict[str, object]], Optional[float]]:
        residual_entries = [self._pointwise_sweep_residuals(sweep, anchors) for sweep in self.sweeps]
        scale_override = None
        if self.pointwise_filtering and self.pointwise_global_mad:
            residuals_list: List[np.ndarray] = []
            for entry in residual_entries:
                residuals = entry.get("residuals")
                if not isinstance(residuals, np.ndarray):
                    continue
                sigma_l2 = entry.get("sigma_l2")
                if bool(self._noise_norm_available) and isinstance(sigma_l2, np.ndarray):
                    sigma_arr = np.asarray(sigma_l2, dtype=float).ravel()
                    if sigma_arr.size == residuals.size and np.all(np.isfinite(sigma_arr)):
                        residuals_list.append(residuals / sigma_arr)
                        continue
                residuals_list.append(residuals / float(max(self._l2_scale, 1.0)))
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
        if count <= 0:
            return None, None, "no-finite"
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
        if count < _SWEEP_WISE_MIN_SAMPLES:
            reason = f"few-samples ({count} < {_SWEEP_WISE_MIN_SAMPLES})"
        if keep_count < min_keep:
            sorted_vals = np.sort(data)
            idx = min(min_keep - 1, sorted_vals.size - 1)
            relaxed = float(sorted_vals[idx])
            if np.isfinite(relaxed):
                threshold = float(max(threshold, relaxed))
                keep = metrics <= threshold
                reason = f"{reason}; relaxed-min-keep" if reason != "ok" else "relaxed-min-keep"
        if not np.any(keep & finite_mask):
            return None, threshold, "all-rejected"
        return keep, threshold, reason

    def robustness_diagnostics(self, anchor_vec: np.ndarray, *, top_n: int = 5) -> Dict[str, object]:
        """Return per-sweep diagnostics for active robustness filters."""
        anchors = anchors_vec_to_matrix(anchor_vec, self.num_anchors, self.dimensions)
        diagnostics: Dict[str, object] = {}

        sigma_counts: Dict[str, int] = {"point": 0, "origin": 0, "min": 0, "mixed": 0}
        sigma_total = 0
        for sweep in self.sweeps:
            if isinstance(sweep, Sweep):
                drive_idx = int(sweep.drive_anchor)
                sensor_idx = int(sweep.sensor_anchor)
                points = sweep.data_points or []
            else:
                drive_idx = int(sweep.get("drive_anchor")) if sweep.get("drive_anchor") is not None else -1
                sensor_idx = int(sweep.get("sensor_anchor")) if sweep.get("sensor_anchor") is not None else -1
                points = sweep.get("data_points", []) if isinstance(sweep, dict) else []
            if drive_idx < 0 or sensor_idx < 0:
                continue
            for point in points:
                src_drive = self._sigma_source_for_point(point, drive_idx)
                src_sensor = self._sigma_source_for_point(point, sensor_idx)
                sigma_total += 1
                if src_drive == src_sensor:
                    sigma_counts[src_drive] = sigma_counts.get(src_drive, 0) + 1
                else:
                    sigma_counts["mixed"] = sigma_counts.get("mixed", 0) + 1

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
            "sigma_source": str(self.sigma_source),
            "sigma_source_counts": {**sigma_counts, "total": int(sigma_total)},
            "sigma_floor_deg": float(self._sigma_floor_deg)
            if self._sigma_floor_deg is not None and np.isfinite(self._sigma_floor_deg)
            else None,
            "sigma_noise_mm": float(self._encoder_noise_mm) if self._encoder_noise_mm is not None else None,
            "sigma_mult": float(self._pointwise_sigma_mult),
            "sigma_scaled_mm": float(self._pointwise_sigma_scaled_mm)
            if self._pointwise_sigma_scaled_mm is not None
            else None,
            "sigma_min_mm": float(self._pointwise_sigma_min_mm),
            "sigma_model_mm": float(self._pointwise_sigma_model_mm),
            "sigma_used_mm": float(self._pointwise_sigma_floor_mm),
            "sigma_floor_mm": float(self._pointwise_sigma_floor_mm),
            "sigma_floor_source": str(self._pointwise_sigma_floor_source),
            **dict(self._sigma_components),
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
        kept_count = 0

        entries, _global_scale = self._pointwise_entries(anchors)

        sweep_metrics = [float(e.get("sweep_metric", float("inf"))) for e in entries]
        keep_mask, _threshold, _reason = self._sweep_wise_keep_mask(sweep_metrics)

        for idx, entry in enumerate(entries):
            cost = float(entry.get("cost", float("inf")))
            violation_penalty = float(entry.get("violation_penalty", 0.0))
            valid = bool(entry.get("valid", False))

            if not valid or not np.isfinite(cost):
                # Treat invalid sweeps like rejected sweeps: no weight, no cost contribution.
                weights.append(0.0)
                weighted_costs.append(0.0)
                continue

            if keep_mask is not None and not bool(keep_mask[idx]):
                weights.append(0.0)
                weighted_costs.append(0.0)
                continue

            weights.append(1.0)
            weighted_costs.append(float(cost + violation_penalty))
            kept_count += 1

        if kept_count < self._min_sweeps_after_trim:
            return float(_UNDERCONSTRAINED_PENALTY)

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
                # Treat invalid sweeps like rejected sweeps: no weight, no cost contribution.
                weights[sweep_id] = 0.0
                per_sweep_costs[sweep_id] = 0.0
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

        if num_valid < self._min_sweeps_after_trim:
            total_cost = float(_UNDERCONSTRAINED_PENALTY)
        else:
            weight_sum = float(sum(weights.values())) if weights else 1.0
            if weight_sum <= 0:
                weight_sum = 1.0

            total_cost = sum(
                per_sweep_costs[sid] * (weights.get(sid, 1.0) / weight_sum) for sid in per_sweep_costs
            )

        noise_metrics = None
        z_values: List[np.ndarray] = []
        z_trim_values: List[np.ndarray] = []
        norm_modes: List[str] = []
        for idx, entry in enumerate(entries):
            sweep_id = str(entry.get("sweep_id", ""))
            if sweep_id not in per_sweep_costs:
                continue
            if not bool(entry.get("valid", False)):
                continue
            if keep_mask is not None and not bool(keep_mask[idx]):
                continue
            r_norm = entry.get("_r_norm")
            if isinstance(r_norm, np.ndarray):
                vals = np.asarray(r_norm, dtype=float).ravel()
                vals = vals[np.isfinite(vals)]
                if vals.size:
                    z_values.append(vals)
                    mode = entry.get("norm_mode")
                    if isinstance(mode, str) and mode:
                        norm_modes.append(mode)
            r_norm_used = entry.get("_r_norm_used")
            if isinstance(r_norm_used, np.ndarray):
                vals_used = np.asarray(r_norm_used, dtype=float).ravel()
                vals_used = vals_used[np.isfinite(vals_used)]
                if vals_used.size:
                    z_trim_values.append(vals_used)

        if z_values:
            z_all = np.concatenate(z_values)
            z_abs = np.abs(z_all)
            n_obs = int(z_all.size)
            if n_obs > 0:
                norm_mode = None
                if norm_modes:
                    unique_modes = sorted(set(norm_modes))
                    if len(unique_modes) == 1:
                        norm_mode = unique_modes[0]
                    else:
                        norm_mode = "mixed"
                p_count = int(self.num_anchors * self.dimensions)
                chi2_red = None
                if n_obs > p_count:
                    chi2_red = float(np.sum(z_all**2) / float(n_obs - p_count))
                trim_j = None
                trim_chi2 = None
                trim_n = None
                if z_trim_values:
                    z_trim = np.concatenate(z_trim_values)
                    trim_n = int(z_trim.size)
                    if trim_n > 0:
                        trim_j = float(np.mean(z_trim**2))
                        if trim_n > p_count:
                            trim_chi2 = float(np.sum(z_trim**2) / float(trim_n - p_count))
                noise_metrics = {
                    "J": float(np.mean(z_all**2)),
                    "chi2_red": chi2_red,
                    "n_obs": n_obs,
                    "params": p_count,
                    "J_trimmed": trim_j,
                    "chi2_red_trimmed": trim_chi2,
                    "n_obs_trimmed": trim_n,
                    "median_abs_z": float(np.median(z_abs)),
                    "p95_abs_z": float(np.percentile(z_abs, 95)),
                    "outlier_ratio": float(np.mean(z_abs > _POINTWISE_TRIM_K)),
                    "norm_mode": norm_mode,
                    "noise_normalized": bool(norm_mode == "sigma"),
                    "lengths_mode": "mu" if bool(self.use_noise_mean) else "raw",
                    "sigma_min_mm": float(self._pointwise_sigma_min_mm),
                    "sigma_model_mm": float(self._pointwise_sigma_model_mm),
                    "sigma_used_mm": float(self._pointwise_sigma_floor_mm),
                    "sigma_floor_deg": float(self._sigma_floor_deg)
                    if self._sigma_floor_deg is not None and np.isfinite(self._sigma_floor_deg)
                    else None,
                    "sigma_source": str(self.sigma_source),
                    **dict(self._sigma_components),
                }

        return CostResult(
            total_cost=float(total_cost),
            per_sweep_costs=per_sweep_costs,
            num_valid_sweeps=num_valid,
            num_invalid_sweeps=num_invalid,
            anchor_estimate=anchors,
            noise_metrics=noise_metrics,
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
