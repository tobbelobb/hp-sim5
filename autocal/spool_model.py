from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from autocal.active_learning import SweepConfig

_EPS = 1e-12


@dataclass(frozen=True)
class WinchSpoolModel:
    """
    Per-winch spool model with firmware-style coefficients.

    The model is defined in motor-angle space and line-position space:
      line_pos_mm = L(theta_deg)

    with coefficients:
      k2 = -(mechanical_advantage * lines_per_spool) * buildup_factor
      k1 = base_radius^2
      k0 = 2 * degrees_per_unit_times_r / k2

    Important: non-linear deltas must be computed as:
      delta = L(theta) - L(theta0)
    """

    base_radius: float
    degrees_per_unit_times_r: float
    k0: float
    k1: float
    k2: float
    use_constant_model: bool

    @classmethod
    def from_firmware(
        cls,
        *,
        base_radius: float,
        buildup_factor: float,
        spool_to_motor_gearing_factor: float,
        mechanical_advantage: float,
        lines_per_spool: float,
    ) -> "WinchSpoolModel":
        r = float(base_radius)
        gear = float(spool_to_motor_gearing_factor)
        ma = float(mechanical_advantage)
        lines = float(lines_per_spool)
        if not np.isfinite(r) or r <= 0.0:
            raise ValueError("base_radius must be finite and > 0")
        if not np.isfinite(gear) or abs(gear) <= _EPS:
            raise ValueError("spool_to_motor_gearing_factor must be finite and non-zero")
        if not np.isfinite(ma) or abs(ma) <= _EPS:
            raise ValueError("mechanical_advantage must be finite and non-zero")
        if not np.isfinite(lines) or abs(lines) <= _EPS:
            raise ValueError("lines_per_spool must be finite and non-zero")

        degrees_per_unit_times_r = (gear * ma * lines * 360.0) / (2.0 * np.pi)
        if not np.isfinite(degrees_per_unit_times_r) or abs(degrees_per_unit_times_r) <= _EPS:
            raise ValueError("invalid degrees_per_unit_times_r")

        k2 = -(ma * lines) * float(buildup_factor)
        if not np.isfinite(k2) or abs(k2) <= _EPS:
            return cls(
                base_radius=r,
                degrees_per_unit_times_r=degrees_per_unit_times_r,
                k0=0.0,
                k1=r * r,
                k2=0.0,
                use_constant_model=True,
            )

        k0 = 2.0 * degrees_per_unit_times_r / k2
        if not np.isfinite(k0) or abs(k0) <= _EPS:
            return cls(
                base_radius=r,
                degrees_per_unit_times_r=degrees_per_unit_times_r,
                k0=0.0,
                k1=r * r,
                k2=0.0,
                use_constant_model=True,
            )
        return cls(
            base_radius=r,
            degrees_per_unit_times_r=degrees_per_unit_times_r,
            k0=float(k0),
            k1=float(r * r),
            k2=float(k2),
            use_constant_model=False,
        )

    def theta_deg_to_linepos_mm(self, theta_deg: float) -> float:
        theta = float(theta_deg)
        if not np.isfinite(theta):
            return float("nan")
        if self.use_constant_model:
            return (theta * self.base_radius) / self.degrees_per_unit_times_r
        term = theta / self.k0 + self.base_radius
        return (term * term - self.k1) / self.k2

    def delta_linepos_mm(self, theta_deg: float, theta0_deg: float) -> float:
        return self.theta_deg_to_linepos_mm(theta_deg) - self.theta_deg_to_linepos_mm(theta0_deg)

    def linepos_mm_to_theta_deg(self, linepos_mm: float) -> float:
        linepos = float(linepos_mm)
        if not np.isfinite(linepos):
            return float("nan")
        if self.use_constant_model:
            return (linepos * self.degrees_per_unit_times_r) / self.base_radius
        radicand = self.k1 + linepos * self.k2
        if radicand < 0.0:
            radicand = 0.0
        return self.k0 * (float(np.sqrt(radicand)) - self.base_radius)


@dataclass(frozen=True)
class SpoolModelParams:
    base_models: Tuple[WinchSpoolModel, ...]
    modeled_models: Tuple[WinchSpoolModel, ...]
    theta0_deg: Tuple[float, ...]

    def _theta0(self, axis_idx: int) -> float:
        return float(self.theta0_deg[int(axis_idx)])

    def modeled_delta_from_theta(self, axis_idx: int, theta_deg: float) -> float:
        axis = int(axis_idx)
        model = self.modeled_models[axis]
        theta0 = self._theta0(axis)
        return model.delta_linepos_mm(float(theta_deg), theta0)

    def modeled_delta_from_base_delta(self, axis_idx: int, base_delta_mm: float) -> float:
        axis = int(axis_idx)
        theta_delta = self.base_models[axis].linepos_mm_to_theta_deg(float(base_delta_mm))
        theta_abs = self._theta0(axis) + float(theta_delta)
        return self.modeled_models[axis].delta_linepos_mm(theta_abs, self._theta0(axis))


def _extract_raw_angles(point: dict, num_anchors: int) -> List[float]:
    raw = point.get("raw_angles_deg")
    if not isinstance(raw, list) or len(raw) < int(num_anchors):
        raise ValueError("raw_angles_deg missing or wrong length")
    out: List[float] = []
    for idx in range(int(num_anchors)):
        try:
            val = float(raw[idx])
        except (TypeError, ValueError):
            raise ValueError("raw_angles_deg contains non-numeric values") from None
        if not np.isfinite(val):
            raise ValueError("raw_angles_deg contains non-finite values")
        out.append(val)
    return out


def _point_len_base(point: dict, primary_key: str, mu_key: str) -> Optional[float]:
    val = point.get(mu_key, point.get(primary_key))
    try:
        f = float(val)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(f):
        return None
    return float(f)


def validate_dataset_has_raw_angles(dataset: dict) -> None:
    num_anchors = int(dataset.get("num_anchors", 0))
    sweeps = dataset.get("sweeps")
    if num_anchors <= 0 or not isinstance(sweeps, list):
        raise ValueError("Dataset missing num_anchors/sweeps for raw angle validation")
    for sweep_idx, sweep in enumerate(sweeps):
        if not isinstance(sweep, dict):
            raise ValueError(f"sweep[{sweep_idx}] is not an object")
        points = sweep.get("data_points")
        if not isinstance(points, list):
            raise ValueError(f"sweep[{sweep_idx}] missing data_points list")
        for point_idx, point in enumerate(points):
            if not isinstance(point, dict):
                raise ValueError(f"sweep[{sweep_idx}] point[{point_idx}] is not an object")
            _extract_raw_angles(point, num_anchors)


def _infer_theta0_deg(dataset: dict, base_models: Sequence[WinchSpoolModel]) -> Tuple[float, ...]:
    num_anchors = int(dataset.get("num_anchors", 0))
    sweeps = dataset.get("sweeps")
    if num_anchors <= 0 or not isinstance(sweeps, list):
        raise ValueError("Dataset missing num_anchors/sweeps for theta0 inference")
    if len(base_models) != num_anchors:
        raise ValueError("base_models length mismatch")

    theta0_samples: List[List[float]] = [[] for _ in range(num_anchors)]

    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        drive_idx = int(sweep.get("drive_anchor", 0))
        sensor_idx = int(sweep.get("sensor_anchor", 0))
        fixed_anchors = [int(v) for v in (sweep.get("fixed_anchors", []) or [])]
        fixed_lengths = [float(v) for v in (sweep.get("fixed_lengths", []) or [])]
        points = sweep.get("data_points")
        if not isinstance(points, list):
            continue
        for point in points:
            if not isinstance(point, dict):
                continue
            raw = _extract_raw_angles(point, num_anchors)

            drive_base = _point_len_base(point, "l_drive", "l_drive_mu")
            if drive_base is not None:
                theta_delta = base_models[drive_idx].linepos_mm_to_theta_deg(drive_base)
                if np.isfinite(theta_delta):
                    theta0_samples[drive_idx].append(float(raw[drive_idx] - theta_delta))

            sensor_base = _point_len_base(point, "l_sensor", "l_sensor_mu")
            if sensor_base is not None:
                theta_delta = base_models[sensor_idx].linepos_mm_to_theta_deg(sensor_base)
                if np.isfinite(theta_delta):
                    theta0_samples[sensor_idx].append(float(raw[sensor_idx] - theta_delta))

            for axis, fixed_delta in zip(fixed_anchors, fixed_lengths):
                theta_delta = base_models[axis].linepos_mm_to_theta_deg(float(fixed_delta))
                if np.isfinite(theta_delta):
                    theta0_samples[axis].append(float(raw[axis] - theta_delta))

    out: List[float] = []
    for axis, vals in enumerate(theta0_samples):
        arr = np.asarray(vals, dtype=float)
        arr = arr[np.isfinite(arr)]
        if arr.size == 0:
            raise ValueError(
                f"Could not infer theta0 for axis {axis}; raw_angles_deg and base lengths are required."
            )
        out.append(float(np.median(arr)))
    return tuple(out)


def build_spool_model_params(
    dataset: dict,
    *,
    base_radii_mm: Sequence[float],
    modeled_radii_mm: Sequence[float],
    modeled_buildup_factor: Sequence[float],
    spool_to_motor_gearing_factor: Sequence[float],
    mechanical_advantage: Sequence[float],
    lines_per_spool: Sequence[float],
    base_buildup_factor: Optional[Sequence[float]] = None,
) -> SpoolModelParams:
    num_anchors = int(dataset.get("num_anchors", 0))
    if num_anchors <= 0:
        raise ValueError("Dataset missing num_anchors")

    base_r = np.asarray(base_radii_mm, dtype=float).reshape(-1)
    modeled_r = np.asarray(modeled_radii_mm, dtype=float).reshape(-1)
    modeled_b = np.asarray(modeled_buildup_factor, dtype=float).reshape(-1)
    spool_to_motor = np.asarray(spool_to_motor_gearing_factor, dtype=float).reshape(-1)
    mech_adv = np.asarray(mechanical_advantage, dtype=float).reshape(-1)
    lines = np.asarray(lines_per_spool, dtype=float).reshape(-1)

    if any(v.size != num_anchors for v in (base_r, modeled_r, modeled_b, spool_to_motor, mech_adv, lines)):
        raise ValueError("spool parameter array length mismatch")

    if base_buildup_factor is None:
        base_b = np.zeros(num_anchors, dtype=float)
    else:
        base_b = np.asarray(base_buildup_factor, dtype=float).reshape(-1)
        if base_b.size != num_anchors:
            raise ValueError("base_buildup_factor length mismatch")

    base_models = tuple(
        WinchSpoolModel.from_firmware(
            base_radius=float(base_r[i]),
            buildup_factor=float(base_b[i]),
            spool_to_motor_gearing_factor=float(spool_to_motor[i]),
            mechanical_advantage=float(mech_adv[i]),
            lines_per_spool=float(lines[i]),
        )
        for i in range(num_anchors)
    )
    modeled_models = tuple(
        WinchSpoolModel.from_firmware(
            base_radius=float(modeled_r[i]),
            buildup_factor=float(modeled_b[i]),
            spool_to_motor_gearing_factor=float(spool_to_motor[i]),
            mechanical_advantage=float(mech_adv[i]),
            lines_per_spool=float(lines[i]),
        )
        for i in range(num_anchors)
    )

    validate_dataset_has_raw_angles(dataset)
    theta0 = _infer_theta0_deg(dataset, base_models)
    return SpoolModelParams(
        base_models=base_models,
        modeled_models=modeled_models,
        theta0_deg=theta0,
    )


def dataset_with_modeled_lengths(dataset: dict, spool_params: SpoolModelParams) -> dict:
    out = copy.deepcopy(dataset)
    num_anchors = int(out.get("num_anchors", 0))
    sweeps = out.get("sweeps")
    if num_anchors <= 0 or not isinstance(sweeps, list):
        raise ValueError("Dataset missing num_anchors/sweeps for modeled length rewrite")

    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        drive_idx = int(sweep.get("drive_anchor", 0))
        sensor_idx = int(sweep.get("sensor_anchor", 0))
        fixed_anchors = [int(v) for v in (sweep.get("fixed_anchors", []) or [])]
        fixed_lengths_base = [float(v) for v in (sweep.get("fixed_lengths", []) or [])]
        sweep["fixed_lengths_base"] = list(fixed_lengths_base)

        points = sweep.get("data_points")
        if not isinstance(points, list):
            continue

        fixed_modeled: List[float] = []
        for axis, _fixed_delta_base in zip(fixed_anchors, fixed_lengths_base):
            vals: List[float] = []
            for point in points:
                if not isinstance(point, dict):
                    continue
                raw = _extract_raw_angles(point, num_anchors)
                vals.append(spool_params.modeled_delta_from_theta(axis, raw[axis]))
            arr = np.asarray(vals, dtype=float)
            arr = arr[np.isfinite(arr)]
            if arr.size == 0:
                raise ValueError(f"Could not model fixed length for axis {axis}")
            fixed_modeled.append(float(np.median(arr)))
        sweep["fixed_lengths"] = fixed_modeled

        for point in points:
            if not isinstance(point, dict):
                continue
            raw = _extract_raw_angles(point, num_anchors)

            if "l_drive" in point:
                point["l_drive_base"] = point.get("l_drive")
            if "l_sensor" in point:
                point["l_sensor_base"] = point.get("l_sensor")
            if "l_drive_mu" in point:
                point["l_drive_mu_base"] = point.get("l_drive_mu")
            if "l_sensor_mu" in point:
                point["l_sensor_mu_base"] = point.get("l_sensor_mu")

            drive_modeled = spool_params.modeled_delta_from_theta(drive_idx, raw[drive_idx])
            sensor_modeled = spool_params.modeled_delta_from_theta(sensor_idx, raw[sensor_idx])
            point["l_drive"] = float(drive_modeled)
            point["l_sensor"] = float(sensor_modeled)
            if "l_drive_mu_base" in point or "l_drive_mu" in point:
                point["l_drive_mu"] = float(drive_modeled)
            if "l_sensor_mu_base" in point or "l_sensor_mu" in point:
                point["l_sensor_mu"] = float(sensor_modeled)

    return out


def sweep_configs_with_modeled_lengths(
    sweep_configs: Sequence[SweepConfig], spool_params: SpoolModelParams
) -> List[SweepConfig]:
    out: List[SweepConfig] = []
    for cfg in sweep_configs:
        modeled_fixed = [
            float(spool_params.modeled_delta_from_base_delta(axis, delta))
            for axis, delta in zip(cfg.fixed_anchors, cfg.fixed_deltas_mm)
        ]
        out.append(
            SweepConfig(
                fixed_anchors=tuple(int(v) for v in cfg.fixed_anchors),
                fixed_deltas_mm=tuple(float(v) for v in modeled_fixed),
                drive_anchor=int(cfg.drive_anchor),
                sensor_anchor=int(cfg.sensor_anchor),
            )
        )
    return out

