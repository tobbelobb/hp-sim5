#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time
import urllib.request
import contextlib
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

DEFAULT_RRF_PORT = 8081
RRF_SIM_BINARY = REPO_ROOT / "RRF" / "build" / "rrf_simulator"
RRF_SIM_VSD_PATH = "RRF/run/vsd"
RRF_SIM_DEFAULT_CONFIG = "sys/config_slideprinter.g"
RRF_SIM_LINE_LAYER_CONFIG = "sys/config_slideprinter_w_line_layers.g"
DEFAULT_NOISE_SIGMA_FLOOR_DEG = 0.01
DEFAULT_NOISE_MIN_SAMPLES = 10
DEFAULT_LAYER_LINE_WIDTH_MM = 1.0
DEFAULT_FULL_AUTO_MIN_DELTA = 0.0
_MIN_RADIUS_SCALE = 0.2
_MAX_RADIUS_SCALE = 5.0
_DEFAULT_B_BOUNDS = (0.0, 1.0)
_DEFAULT_B_PRIOR_SIGMA = 0.1
_DEFAULT_RADIUS_PAIR_SIGMA_MM = 2.0
_COMPUTE_SPOOL_INFO_MATRIX = False
_SPOOL_FIND_MODE_CHOICES = ("off", "global", "per-anchor")
_THETA0_MODE_CHOICES = ("infer", "zero")
_NOISE_MODEL_CONFIG_KEY = "noise_model"

from autocal.active_learning import (
    SweepConfig,
    dataset_sweep_configs,
    generate_candidate_sweeps,
    l2_scale_for_machine,
    rank_candidates_d_optimal,
    total_information_matrix,
)
from autocal.calibrate import calibrate_elliptical
from autocal.dataset_roles import normalize_dataset_point_roles as _normalize_dataset_point_roles
from autocal.ellipse_cost import EllipseCostFunction
from autocal.json_schema import append_jsonl_line, load_json_file, write_json_file
from autocal.spool_model import (
    SpoolModelParams,
    build_spool_model_params,
    dataset_with_modeled_lengths,
    sweep_configs_with_modeled_lengths,
    validate_dataset_has_raw_angles,
)
from autocal.sweep_types import MachineType

GeometryWeights = Tuple[float, float, float]
MACHINE_TYPE_CHOICES = tuple(machine.value for machine in MachineType)
MACHINE_TYPE_CHOICES_STR = " | ".join(MACHINE_TYPE_CHOICES)


def _require_machine_type(
    dataset: dict,
    *,
    expected: Optional[str] = None,
    context: str = "dataset",
    mismatch: str = "error",
) -> str:
    raw_machine_type = dataset.get("machine_type")
    if not raw_machine_type:
        raise ValueError(
            f"{context} missing machine_type. Expected one of: {MACHINE_TYPE_CHOICES_STR}"
        )
    machine_type = str(raw_machine_type)
    if machine_type not in MACHINE_TYPE_CHOICES:
        raise ValueError(
            f"{context} machine_type '{machine_type}' is not supported. Expected one of: {MACHINE_TYPE_CHOICES_STR}"
        )
    if expected is not None and str(expected) != machine_type:
        message = (
            f"{context} machine_type '{machine_type}' does not match --machine-type '{expected}'. "
            f"Using '{machine_type}'. Expected one of: {MACHINE_TYPE_CHOICES_STR}"
        )
        if mismatch == "warn":
            print(f"; warning: {message}", file=sys.stderr)
        else:
            raise ValueError(message)
    return machine_type


def _load_json(path: Path) -> dict:
    return load_json_file(path, schema="sweep_dataset")


def _write_json(path: Path, payload: dict) -> None:
    write_json_file(path, payload, schema="sweep_dataset")



def _expand_to_num_axes(value: Any, num_axes: int, default: float) -> List[float]:
    if value is None:
        return [float(default)] * num_axes
    if isinstance(value, (int, float)):
        try:
            v = float(value)
        except Exception:
            v = float(default)
        return [v] * num_axes
    if isinstance(value, (list, tuple)):
        out: List[float] = []
        for item in value[:num_axes]:
            try:
                out.append(float(item))
            except Exception:
                out.append(float(default))
        if len(out) < num_axes:
            pad = out[0] if out else float(default)
            out.extend([float(pad)] * (num_axes - len(out)))
        return out
    return [float(default)] * num_axes


def _extract_m666(config: Optional[dict]) -> Optional[dict]:
    if not isinstance(config, dict):
        return None
    for key in ("m666", "m666_after", "m666_before"):
        val = config.get(key)
        if isinstance(val, dict):
            return val
    return None


def _resolve_length_model_base_params(
    dataset: dict,
    *,
    num_anchors: int,
    base_radii_override: Optional[Sequence[float]],
) -> Dict[str, np.ndarray]:
    config = dataset.get("config")
    if not isinstance(config, dict):
        config = {}
    m666 = _extract_m666(config)

    if base_radii_override is not None and len(base_radii_override) > 0:
        base_radii_list = _expand_to_num_axes(list(base_radii_override), int(num_anchors), float("nan"))
    elif isinstance(m666, dict):
        base_radii_list = _expand_to_num_axes(m666.get("R"), int(num_anchors), float("nan"))
    else:
        base_radii_list = [float("nan")] * int(num_anchors)

    base_radii = np.asarray(base_radii_list, dtype=float)
    if (
        base_radii.size != int(num_anchors)
        or not np.all(np.isfinite(base_radii))
        or np.any(base_radii <= 0.0)
    ):
        raise ValueError(
            "--find-radii requires positive base radii for all axes "
            "(use --base-radii or provide m666 R in dataset config)."
        )

    mech_adv = np.asarray(
        _expand_to_num_axes(m666.get("U") if isinstance(m666, dict) else None, int(num_anchors), 1.0),
        dtype=float,
    )
    lines_per_spool = np.asarray(
        _expand_to_num_axes(m666.get("O") if isinstance(m666, dict) else None, int(num_anchors), 1.0),
        dtype=float,
    )
    motor_gear = np.asarray(
        _expand_to_num_axes(m666.get("L") if isinstance(m666, dict) else None, int(num_anchors), 1.0),
        dtype=float,
    )
    spool_gear = np.asarray(
        _expand_to_num_axes(m666.get("H") if isinstance(m666, dict) else None, int(num_anchors), 1.0),
        dtype=float,
    )

    mech_adv = np.where(np.isfinite(mech_adv) & (np.abs(mech_adv) > 1e-12), mech_adv, 1.0)
    lines_per_spool = np.where(
        np.isfinite(lines_per_spool) & (np.abs(lines_per_spool) > 1e-12), lines_per_spool, 1.0
    )
    motor_gear = np.where(np.isfinite(motor_gear) & (np.abs(motor_gear) > 1e-12), motor_gear, 1.0)
    spool_gear = np.where(np.isfinite(spool_gear) & (np.abs(spool_gear) > 1e-12), spool_gear, 1.0)
    spool_to_motor = spool_gear / motor_gear

    return {
        "base_radii_mm": base_radii,
        "mechanical_advantage": mech_adv,
        "lines_per_spool": lines_per_spool,
        "spool_to_motor_gearing_factor": spool_to_motor,
    }


def _mm_per_degree_for_axis(
    radius_mm: float,
    axis_idx: int,
    *,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
) -> float:
    r = float(radius_mm)
    if not np.isfinite(r) or r <= 0.0:
        return float("nan")
    gear = float(spool_to_motor_gearing_factor[int(axis_idx)])
    ma = float(mechanical_advantage[int(axis_idx)])
    _ = float(lines_per_spool[int(axis_idx)])
    # Match firmware: lines_per_spool only affects the buildup slope (k2), not base mm/deg.
    denom = gear * ma * 360.0
    if not np.isfinite(denom) or abs(denom) <= 1e-12:
        return float("nan")
    return float((2.0 * np.pi * r) / denom)


def _delta_base_to_model(
    delta_base: object,
    axis_idx: int,
    *,
    base_radii_mm: np.ndarray,
    effective_radii_mm: np.ndarray,
    buildup_factor: float,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
) -> object:
    vals = np.asarray(delta_base, dtype=float)
    is_scalar = vals.ndim == 0
    vals_flat = vals.reshape(-1)
    out_flat = vals_flat.copy()
    finite = np.isfinite(vals_flat)
    if not np.any(finite):
        if is_scalar:
            return float(out_flat[0])
        return out_flat.reshape(vals.shape)

    axis = int(axis_idx)
    base_r = float(base_radii_mm[axis])
    eff_r = float(effective_radii_mm[axis])
    mm_per_deg_base = _mm_per_degree_for_axis(
        base_r,
        axis,
        spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
        mechanical_advantage=mechanical_advantage,
        lines_per_spool=lines_per_spool,
    )
    mm_per_deg_eff = _mm_per_degree_for_axis(
        eff_r,
        axis,
        spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
        mechanical_advantage=mechanical_advantage,
        lines_per_spool=lines_per_spool,
    )
    if (
        not np.isfinite(mm_per_deg_base)
        or abs(mm_per_deg_base) <= 1e-12
        or not np.isfinite(mm_per_deg_eff)
    ):
        if is_scalar:
            return float(out_flat[0])
        return out_flat.reshape(vals.shape)

    theta_deg = vals_flat[finite] / mm_per_deg_base
    k = float(buildup_factor)
    if not np.isfinite(k) or abs(k) <= 1e-12:
        out_flat[finite] = theta_deg * mm_per_deg_eff
    else:
        gear = float(spool_to_motor_gearing_factor[axis])
        ma = float(mechanical_advantage[axis])
        lines = float(lines_per_spool[axis])
        c1 = -ma * lines * k
        deg_per_unit_times_r = (gear * ma * 360.0) / (2.0 * np.pi)
        if (
            not np.isfinite(c1)
            or abs(c1) <= 1e-12
            or not np.isfinite(deg_per_unit_times_r)
            or abs(deg_per_unit_times_r) <= 1e-12
        ):
            out_flat[finite] = theta_deg * mm_per_deg_eff
        else:
            k0 = 2.0 * deg_per_unit_times_r / c1
            out_flat[finite] = (((theta_deg / k0) + eff_r) ** 2.0 - eff_r * eff_r) / c1

    if is_scalar:
        return float(out_flat[0])
    return out_flat.reshape(vals.shape)


def _transform_dataset_lengths_to_model(
    dataset: dict,
    *,
    base_radii_mm: np.ndarray,
    effective_radii_mm: np.ndarray,
    buildup_factor: float,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
) -> dict:
    out = dict(dataset)
    cfg_in = dataset.get("config")
    cfg_out = dict(cfg_in) if isinstance(cfg_in, dict) else {}
    out["config"] = cfg_out
    mm_per_degree_model = [
        _mm_per_degree_for_axis(
            float(effective_radii_mm[idx]),
            idx,
            spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
            mechanical_advantage=mechanical_advantage,
            lines_per_spool=lines_per_spool,
        )
        for idx in range(int(base_radii_mm.size))
    ]
    cfg_out["mm_per_degree"] = [float(v) if np.isfinite(v) else float("nan") for v in mm_per_degree_model]
    notes = cfg_out.get("notes")
    notes_out = dict(notes) if isinstance(notes, dict) else {}
    notes_out["length_model"] = {
        "coord_planning": "L_base_mm",
        "coord_estimation": "L_model_mm",
        "buildup_factor_k": float(buildup_factor),
    }
    cfg_out["notes"] = notes_out

    sweeps_in = dataset.get("sweeps")
    sweeps_out: List[dict] = []
    if isinstance(sweeps_in, list):
        for sweep in sweeps_in:
            if not isinstance(sweep, dict):
                continue
            sw = dict(sweep)
            fixed_anchors = [int(v) for v in (sweep.get("fixed_anchors", []) or [])]
            fixed_lengths = list(sweep.get("fixed_lengths", []) or [])
            transformed_fixed: List[float] = []
            for axis, delta in zip(fixed_anchors, fixed_lengths):
                transformed_fixed.append(
                    float(
                        _delta_base_to_model(
                            delta,
                            int(axis),
                            base_radii_mm=base_radii_mm,
                            effective_radii_mm=effective_radii_mm,
                            buildup_factor=float(buildup_factor),
                            spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                            mechanical_advantage=mechanical_advantage,
                            lines_per_spool=lines_per_spool,
                        )
                    )
                )
            sw["fixed_lengths"] = transformed_fixed

            drive_idx = int(sweep.get("drive_anchor", 0))
            sensor_idx = int(sweep.get("sensor_anchor", 0))
            points_in = sweep.get("data_points")
            if isinstance(points_in, list):
                points_out: List[dict] = []
                for point in points_in:
                    if not isinstance(point, dict):
                        continue
                    p = dict(point)
                    for key, axis in (
                        ("l_drive", drive_idx),
                        ("l_sensor", sensor_idx),
                        ("l_drive_mu", drive_idx),
                        ("l_sensor_mu", sensor_idx),
                    ):
                        if key in p:
                            p[key] = float(
                                _delta_base_to_model(
                                    p.get(key),
                                    int(axis),
                                    base_radii_mm=base_radii_mm,
                                    effective_radii_mm=effective_radii_mm,
                                    buildup_factor=float(buildup_factor),
                                    spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                                    mechanical_advantage=mechanical_advantage,
                                    lines_per_spool=lines_per_spool,
                                )
                            )
                    points_out.append(p)
                sw["data_points"] = points_out
            sweeps_out.append(sw)
    out["sweeps"] = sweeps_out
    return out


def _transform_sweep_configs_to_model(
    sweeps: Sequence[SweepConfig],
    *,
    base_radii_mm: np.ndarray,
    effective_radii_mm: np.ndarray,
    buildup_factor: float,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
) -> List[SweepConfig]:
    out: List[SweepConfig] = []
    for cfg in sweeps:
        transformed = []
        for axis, delta in zip(cfg.fixed_anchors, cfg.fixed_deltas_mm):
            transformed.append(
                float(
                    _delta_base_to_model(
                        float(delta),
                        int(axis),
                        base_radii_mm=base_radii_mm,
                        effective_radii_mm=effective_radii_mm,
                        buildup_factor=float(buildup_factor),
                        spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                        mechanical_advantage=mechanical_advantage,
                        lines_per_spool=lines_per_spool,
                    )
                )
            )
        out.append(
            SweepConfig(
                fixed_anchors=tuple(int(v) for v in cfg.fixed_anchors),
                fixed_deltas_mm=tuple(transformed),
                drive_anchor=int(cfg.drive_anchor),
                sensor_anchor=int(cfg.sensor_anchor),
            )
        )
    return out


def _estimate_effective_radii(
    dataset: dict,
    anchors: np.ndarray,
    *,
    base_radii_mm: np.ndarray,
    buildup_factor: float,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
) -> Tuple[np.ndarray, Dict[str, object]]:
    base = np.asarray(base_radii_mm, dtype=float).reshape(-1)
    if base.size == 0 or not np.all(np.isfinite(base)) or np.any(base <= 0.0):
        raise ValueError("Cannot estimate radii without finite positive base radii.")

    lo = np.maximum(base * float(_MIN_RADIUS_SCALE), 1e-6)
    hi = np.maximum(base * float(_MAX_RADIUS_SCALE), lo + 1e-6)
    x0 = np.clip(base, lo, hi)

    def _objective(radii_vec: np.ndarray) -> float:
        radii = np.clip(np.asarray(radii_vec, dtype=float).reshape(-1), lo, hi)
        transformed = _transform_dataset_lengths_to_model(
            dataset,
            base_radii_mm=base,
            effective_radii_mm=radii,
            buildup_factor=float(buildup_factor),
            spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
            mechanical_advantage=mechanical_advantage,
            lines_per_spool=lines_per_spool,
        )
        cost_val = _evaluate_cost_at_anchors(
            transformed,
            anchors,
            residual_threshold=float(residual_threshold),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            pointwise_residual_mode=str(pointwise_residual_mode),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            noise_normalized=True,
            sigma_source=str(sigma_source),
        )
        if not np.isfinite(cost_val):
            return 1e12
        return float(cost_val)

    start_cost = float(_objective(x0))
    try:
        result = minimize(
            _objective,
            x0,
            method="L-BFGS-B",
            bounds=list(zip(lo.tolist(), hi.tolist())),
            options={"maxiter": 60, "ftol": 1e-6},
        )
        if result is not None and np.isfinite(float(result.fun)):
            fitted = np.clip(np.asarray(result.x, dtype=float).reshape(-1), lo, hi)
            fitted_cost = float(result.fun)
            return fitted, {
                "success": bool(getattr(result, "success", False)),
                "message": str(getattr(result, "message", "")),
                "nfev": int(getattr(result, "nfev", 0) or 0),
                "nit": int(getattr(result, "nit", 0) or 0),
                "start_cost": float(start_cost),
                "fitted_cost": float(fitted_cost),
            }
    except Exception as exc:
        return x0, {
            "success": False,
            "message": f"radius optimization failed: {exc}",
            "nfev": 0,
            "nit": 0,
            "start_cost": float(start_cost),
            "fitted_cost": float(start_cost),
        }

    return x0, {
        "success": False,
        "message": "radius optimization did not produce a finite result",
        "nfev": 0,
        "nit": 0,
        "start_cost": float(start_cost),
        "fitted_cost": float(start_cost),
    }


def _estimate_effective_radii_with_spool_model(
    dataset: dict,
    anchors: np.ndarray,
    *,
    find_radii_mode: str,
    find_buildup_mode: str,
    base_radii_mm: np.ndarray,
    modeled_buildup_factor: np.ndarray,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    robust_debug: bool,
    prefer_zero_tension_angles: bool = False,
) -> Tuple[np.ndarray, np.ndarray, SpoolModelParams, dict, Dict[str, object]]:
    base = np.asarray(base_radii_mm, dtype=float).reshape(-1)
    num_anchors = int(base.size)
    if num_anchors <= 0 or not np.all(np.isfinite(base)) or np.any(base <= 0.0):
        raise ValueError("Cannot estimate radii without finite positive base radii.")
    mode_r = _normalize_spool_find_mode(find_radii_mode)
    mode_b = _normalize_spool_find_mode(find_buildup_mode)
    theta0_mode_norm = _normalize_theta0_mode(theta0_mode)
    search_r = _spool_mode_enabled(mode_r)
    search_b = _spool_mode_enabled(mode_b)
    if not (search_r or search_b):
        raise ValueError("spool estimation requires find_radii and/or find_buildup_factor")

    modeled_b = np.asarray(modeled_buildup_factor, dtype=float).reshape(-1)
    spool_to_motor = np.asarray(spool_to_motor_gearing_factor, dtype=float).reshape(-1)
    mech_adv = np.asarray(mechanical_advantage, dtype=float).reshape(-1)
    lines = np.asarray(lines_per_spool, dtype=float).reshape(-1)
    if any(arr.size != num_anchors for arr in (modeled_b, spool_to_motor, mech_adv, lines)):
        raise ValueError("spool parameter length mismatch for radii estimation")

    lo_parts: List[np.ndarray] = []
    hi_parts: List[np.ndarray] = []
    lo_r = np.zeros(0, dtype=float)
    hi_r = np.zeros(0, dtype=float)
    if search_r:
        lo_r, hi_r = _resolve_r0_bounds(base, find_radii_mode=mode_r, r0_bounds=r0_bounds)
        lo_parts.append(lo_r)
        hi_parts.append(hi_r)
    lo_b = np.zeros(0, dtype=float)
    hi_b = np.zeros(0, dtype=float)
    if search_b:
        lo_b, hi_b = _resolve_b_bounds(modeled_b, find_buildup_mode=mode_b, b_bounds=b_bounds)
        lo_parts.append(lo_b)
        hi_parts.append(hi_b)

    if lo_parts:
        lo = np.concatenate(lo_parts).astype(float)
        hi = np.concatenate(hi_parts).astype(float)
    else:
        lo = np.zeros(0, dtype=float)
        hi = np.zeros(0, dtype=float)

    sigma_r = None
    if r0_prior_sigma_mm is not None:
        try:
            sigma_raw = float(r0_prior_sigma_mm)
        except (TypeError, ValueError):
            sigma_raw = float("nan")
        if np.isfinite(sigma_raw) and sigma_raw > 0.0:
            sigma_r = sigma_raw

    sigma_b = None
    if b_prior_sigma is not None:
        try:
            sigma_raw = float(b_prior_sigma)
        except (TypeError, ValueError):
            sigma_raw = float("nan")
        if np.isfinite(sigma_raw) and sigma_raw > 0.0:
            sigma_b = sigma_raw
    elif search_b:
        sigma_b = float(_DEFAULT_B_PRIOR_SIGMA)

    sigma_rpair = None
    if search_r and mode_r == "per-anchor" and num_anchors > 1:
        if sigma_r is not None:
            sigma_rpair = float(max(sigma_r, 1e-6))
        else:
            sigma_rpair = float(_DEFAULT_RADIUS_PAIR_SIGMA_MM)
        if not np.isfinite(sigma_rpair) or sigma_rpair <= 0.0:
            sigma_rpair = float(_DEFAULT_RADIUS_PAIR_SIGMA_MM)

    anchors_current = np.asarray(anchors, dtype=float)
    if anchors_current.ndim != 2 or anchors_current.shape[0] != num_anchors:
        raise ValueError("anchor estimate shape mismatch")
    if not np.all(np.isfinite(anchors_current)):
        raise ValueError("anchor estimate contains non-finite values")

    radii_current = np.asarray(base, dtype=float).copy()
    buildup_current = np.asarray(modeled_b, dtype=float).copy()
    x_current = _pack_spool_opt_vector(
        radii_mm=radii_current,
        buildup_factor=buildup_current,
        find_radii_mode=mode_r,
        find_buildup_mode=mode_b,
    )
    if x_current.size > 0:
        x_current = np.clip(x_current, lo, hi)
        radii_current, buildup_current = _unpack_spool_opt_vector(
            x_current,
            num_anchors=num_anchors,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
            fixed_radii_mm=base,
            fixed_buildup_factor=modeled_b,
        )

    kinds = _spool_opt_kinds(
        num_anchors=num_anchors,
        find_radii_mode=mode_r,
        find_buildup_mode=mode_b,
    )
    outer_iters = max(1, int(spool_outer_iters))
    inner_iters = max(1, int(spool_inner_iters))
    history: List[Dict[str, object]] = []

    best = {
        "cost": float("inf"),
        "radii_mm": np.asarray(radii_current, dtype=float),
        "buildup_factor": np.asarray(buildup_current, dtype=float),
        "anchors": np.asarray(anchors_current, dtype=float),
        "spool_params": None,
        "dataset": None,
    }

    def _build_dataset_and_params(
        radii_mm: np.ndarray,
        buildup_factor: np.ndarray,
    ) -> Tuple[SpoolModelParams, dict]:
        spool_params = build_spool_model_params(
            dataset,
            base_radii_mm=base,
            modeled_radii_mm=radii_mm,
            modeled_buildup_factor=buildup_factor,
            spool_to_motor_gearing_factor=spool_to_motor,
            mechanical_advantage=mech_adv,
            lines_per_spool=lines,
            base_buildup_factor=np.zeros(num_anchors, dtype=float),
            theta0_mode=theta0_mode_norm,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        transformed = dataset_with_modeled_lengths(
            dataset,
            spool_params,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        return spool_params, transformed

    def _prior_cost(radii_mm: np.ndarray, buildup_factor: np.ndarray) -> float:
        prior = 0.0
        if search_r and sigma_r is not None:
            radii_arr = np.asarray(radii_mm, dtype=float).reshape(-1)
            if mode_r == "global":
                prior += float(((float(np.median(radii_arr)) - float(np.median(base))) / sigma_r) ** 2.0)
            else:
                prior += float(np.sum(((radii_arr - base) / sigma_r) ** 2.0))
        if search_b and sigma_b is not None:
            b_arr = np.asarray(buildup_factor, dtype=float).reshape(-1)
            if mode_b == "global":
                prior += float(((float(np.median(b_arr)) - float(np.median(modeled_b))) / sigma_b) ** 2.0)
            else:
                prior += float(np.sum(((b_arr - modeled_b) / sigma_b) ** 2.0))
        if sigma_rpair is not None:
            radii_arr = np.asarray(radii_mm, dtype=float).reshape(-1)
            r_mean = float(np.mean(radii_arr))
            prior += float(np.sum(((radii_arr - r_mean) / sigma_rpair) ** 2.0))
        return float(prior)

    # Keep spool-step data term aligned with the main optimization metric.
    spool_noise_normalized = True

    def _data_cost(transformed_dataset: dict, anchors_eval: np.ndarray) -> float:
        return float(
            _evaluate_cost_at_anchors(
                transformed_dataset,
                anchors_eval,
                residual_threshold=float(residual_threshold),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                pointwise_residual_mode=str(pointwise_residual_mode),
                pointwise_filtering=bool(pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                noise_normalized=bool(spool_noise_normalized),
                sigma_source=str(sigma_source),
            )
        )

    for outer_idx in range(outer_iters):
        spool_params_seed, transformed_seed = _build_dataset_and_params(radii_current, buildup_current)

        anchors_next = np.asarray(anchors_current, dtype=float)
        anchor_cost = float("nan")
        anchor_step_success = False
        anchor_step_restarts = max(1, min(2, int(solve_restarts)))
        anchor_step_iterations = max(40, min(160, int(solve_iterations)))
        try:
            cal_step = calibrate_elliptical(
                transformed_seed,
                output_path=None,
                residual_threshold=float(residual_threshold),
                num_restarts=int(anchor_step_restarts),
                max_iterations=int(anchor_step_iterations),
                method=str(solve_optimizer),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                verbose=False,
                use_parallel=False,
                pointwise_residual_mode=str(pointwise_residual_mode),
                robust_debug=False,
                pointwise_filtering=bool(pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                sigma_source=str(sigma_source),
                generate_report=False,
                residuals_csv=None,
            )
            cand_anchors = np.asarray(cal_step.get("anchors"), dtype=float)
            if cand_anchors.ndim == 2 and cand_anchors.shape == anchors_current.shape and np.all(np.isfinite(cand_anchors)):
                anchors_next = cand_anchors
                anchor_step_success = True
            anchor_cost = float(_data_cost(transformed_seed, anchors_next))
        except Exception:
            anchor_step_success = False

        eval_counter = {"count": 0}
        objective_cache: Dict[Tuple[float, ...], float] = {}

        def _objective(opt_vec: np.ndarray) -> float:
            try:
                if lo.size > 0:
                    clipped_vec = np.clip(np.asarray(opt_vec, dtype=float).reshape(-1), lo, hi)
                else:
                    clipped_vec = np.asarray(opt_vec, dtype=float).reshape(-1)
                key = tuple(float(v) for v in np.round(clipped_vec, decimals=9).tolist())
                cached = objective_cache.get(key)
                if cached is not None and np.isfinite(cached):
                    return float(cached)
                eval_counter["count"] += 1
                radii_try, buildup_try = _unpack_spool_opt_vector(
                    clipped_vec,
                    num_anchors=num_anchors,
                    find_radii_mode=mode_r,
                    find_buildup_mode=mode_b,
                    fixed_radii_mm=base,
                    fixed_buildup_factor=modeled_b,
                )
                _, transformed_try = _build_dataset_and_params(radii_try, buildup_try)
                data_cost = _data_cost(transformed_try, anchors_next)
                if not np.isfinite(data_cost):
                    return 1e12
                prior = _prior_cost(radii_try, buildup_try)
                score = float(data_cost + prior)
                if not np.isfinite(score):
                    objective_cache[key] = 1e12
                    return 1e12
                objective_cache[key] = float(score)
                return score
            except Exception:
                return 1e12

        x_seed = _pack_spool_opt_vector(
            radii_mm=radii_current,
            buildup_factor=buildup_current,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
        )
        if x_seed.size > 0:
            x_seed = np.clip(x_seed, lo, hi)
            seed_candidates = _spool_seed_candidates(x_seed, lo, hi)
            seed_choice = "current"
            seed_cost = float(_objective(x_seed))
            if len(seed_candidates) > 1:
                for idx, seed_try in enumerate(seed_candidates[1:], start=1):
                    score_try = float(_objective(seed_try))
                    if np.isfinite(score_try) and (
                        not np.isfinite(seed_cost) or score_try + 1e-12 < seed_cost
                    ):
                        x_seed = np.asarray(seed_try, dtype=float).reshape(-1)
                        seed_cost = float(score_try)
                        seed_choice = f"seed_{idx}"
            x_opt, opt_info = _coordinate_descent_spool(
                x_seed,
                lo=lo,
                hi=hi,
                kinds=kinds,
                max_iters=int(inner_iters),
                objective=_objective,
            )
            opt_info["seed_choice"] = seed_choice
            opt_info["seed_cost"] = float(seed_cost) if np.isfinite(seed_cost) else None
            if lo.size > 0:
                x_opt = np.clip(np.asarray(x_opt, dtype=float).reshape(-1), lo, hi)
        else:
            x_opt = x_seed
            fitted_cost = float(_objective(x_opt))
            opt_info = {
                "success": True,
                "message": "no spool parameters selected for fitting",
                "nfev": int(eval_counter["count"]),
                "nit": 0,
                "start_cost": float(fitted_cost),
                "fitted_cost": float(fitted_cost),
                "step_final": [],
                "seed_choice": "none",
                "seed_cost": float(fitted_cost) if np.isfinite(fitted_cost) else None,
            }

        radii_opt, buildup_opt = _unpack_spool_opt_vector(
            x_opt,
            num_anchors=num_anchors,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
            fixed_radii_mm=base,
            fixed_buildup_factor=modeled_b,
        )
        spool_params_opt, transformed_opt = _build_dataset_and_params(radii_opt, buildup_opt)
        model_cost = float(_data_cost(transformed_opt, anchors_next))

        if np.isfinite(model_cost) and model_cost < float(best["cost"]):
            best = {
                "cost": float(model_cost),
                "radii_mm": np.asarray(radii_opt, dtype=float),
                "buildup_factor": np.asarray(buildup_opt, dtype=float),
                "anchors": np.asarray(anchors_next, dtype=float),
                "spool_params": spool_params_opt,
                "dataset": transformed_opt,
            }

        start_cost = float(opt_info.get("start_cost", float("nan")))
        fitted_cost = float(opt_info.get("fitted_cost", float("nan")))
        history.append(
            {
                "outer_iter": int(outer_idx + 1),
                "success": bool(opt_info.get("success", False)),
                "message": str(opt_info.get("message", "")),
                "nfev": int(opt_info.get("nfev", eval_counter["count"])),
                "nit": int(opt_info.get("nit", 0)),
                "start_cost": float(start_cost),
                "fitted_cost": float(fitted_cost),
                "anchor_step_success": bool(anchor_step_success),
                "anchor_cost": float(anchor_cost) if np.isfinite(anchor_cost) else None,
                "cal_step_success": bool(anchor_step_success),
                "cal_cost": float(model_cost) if np.isfinite(model_cost) else None,
                "step_final": opt_info.get("step_final"),
                "seed_choice": str(opt_info.get("seed_choice", "")),
                "seed_cost": opt_info.get("seed_cost"),
                "radii_mm": [float(v) for v in np.asarray(radii_opt, dtype=float).tolist()],
                "buildup_factor": [float(v) for v in np.asarray(buildup_opt, dtype=float).tolist()],
            }
        )

        x_current = _pack_spool_opt_vector(
            radii_mm=radii_opt,
            buildup_factor=buildup_opt,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
        )
        if x_current.size > 0:
            x_current = np.clip(x_current, lo, hi)
        radii_current = np.asarray(radii_opt, dtype=float)
        buildup_current = np.asarray(buildup_opt, dtype=float)
        anchors_current = np.asarray(anchors_next, dtype=float)

    if best["dataset"] is None or best["spool_params"] is None:
        if x_current.size > 0:
            x_use = np.clip(x_current, lo, hi)
        else:
            x_use = x_current
        radii_fallback, buildup_fallback = _unpack_spool_opt_vector(
            x_use,
            num_anchors=num_anchors,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
            fixed_radii_mm=base,
            fixed_buildup_factor=modeled_b,
        )
        spool_params_fallback, transformed_fallback = _build_dataset_and_params(radii_fallback, buildup_fallback)
        best = {
            "cost": float("nan"),
            "radii_mm": np.asarray(radii_fallback, dtype=float),
            "buildup_factor": np.asarray(buildup_fallback, dtype=float),
            "anchors": np.asarray(anchors_current, dtype=float),
            "spool_params": spool_params_fallback,
            "dataset": transformed_fallback,
        }

    r0_bounds_info = (
        [float(np.min(lo_r)), float(np.max(hi_r))]
        if lo_r.size > 0 and hi_r.size > 0
        else None
    )
    b_bounds_info = (
        [float(np.min(lo_b)), float(np.max(hi_b))]
        if lo_b.size > 0 and hi_b.size > 0
        else None
    )
    spool_info_hessian = None
    spool_info_rank = None
    if bool(_COMPUTE_SPOOL_INFO_MATRIX) and lo.size > 0 and hi.size > 0:
        x_best = _pack_spool_opt_vector(
            radii_mm=np.asarray(best["radii_mm"], dtype=float),
            buildup_factor=np.asarray(best["buildup_factor"], dtype=float),
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
        )
        x_best = np.clip(np.asarray(x_best, dtype=float).reshape(-1), lo, hi)
        anchors_best = np.asarray(best["anchors"], dtype=float)

        def _objective_for_hessian(opt_vec: np.ndarray) -> float:
            if lo.size > 0:
                clipped_vec = np.clip(np.asarray(opt_vec, dtype=float).reshape(-1), lo, hi)
            else:
                clipped_vec = np.asarray(opt_vec, dtype=float).reshape(-1)
            radii_try, buildup_try = _unpack_spool_opt_vector(
                clipped_vec,
                num_anchors=num_anchors,
                find_radii_mode=mode_r,
                find_buildup_mode=mode_b,
                fixed_radii_mm=base,
                fixed_buildup_factor=modeled_b,
            )
            _, transformed_try = _build_dataset_and_params(radii_try, buildup_try)
            data_cost = _data_cost(transformed_try, anchors_best)
            if not np.isfinite(data_cost):
                return float("inf")
            return float(data_cost + _prior_cost(radii_try, buildup_try))

        hess = _numerical_hessian_symmetric(
            x_best,
            lo=lo,
            hi=hi,
            kinds=kinds,
            objective=_objective_for_hessian,
        )
        if isinstance(hess, np.ndarray) and hess.size:
            spool_info_hessian = hess
            spool_info_rank = int(np.linalg.matrix_rank(hess))

    fit_info: Dict[str, object] = {
        "mode": f"radii={mode_r},buildup={mode_b}",
        "radii_mode": mode_r,
        "buildup_mode": mode_b,
        "theta0_mode": theta0_mode_norm,
        "bounds_mm": r0_bounds_info,
        "r0_bounds_mm": r0_bounds_info,
        "b_bounds": b_bounds_info,
        "r0_prior_sigma_mm": (None if sigma_r is None else float(sigma_r)),
        "b_prior_sigma": (None if sigma_b is None else float(sigma_b)),
        "radii_pair_sigma_mm": (None if sigma_rpair is None else float(sigma_rpair)),
        "outer_iters": int(outer_iters),
        "inner_iters": int(inner_iters),
        "noise_normalized_data_term": bool(spool_noise_normalized),
        "history": history,
        "best_cost": (
            float(best["cost"])
            if isinstance(best.get("cost"), (int, float)) and np.isfinite(float(best["cost"]))
            else None
        ),
        "best_radii_mm": [float(v) for v in np.asarray(best["radii_mm"], dtype=float).tolist()],
        "best_modeled_buildup_factor": [
            float(v) for v in np.asarray(best["buildup_factor"], dtype=float).tolist()
        ],
        "spool_info_rank": spool_info_rank,
        "spool_info_matrix": (
            None
            if spool_info_hessian is None
            else np.asarray(spool_info_hessian, dtype=float).tolist()
        ),
    }

    return (
        np.asarray(best["radii_mm"], dtype=float),
        np.asarray(best["anchors"], dtype=float),
        best["spool_params"],
        best["dataset"],
        fit_info,
    )


def _arg_has_flag(args: Sequence[str], *flags: str) -> bool:
    for arg in args:
        if not isinstance(arg, str):
            continue
        for flag in flags:
            if arg == flag or arg.startswith(flag + "="):
                return True
    return False


def _arg_value(args: Sequence[str], *flags: str) -> Optional[str]:
    for idx, arg in enumerate(args):
        if not isinstance(arg, str):
            continue
        for flag in flags:
            if arg == flag:
                return str(args[idx + 1]) if idx + 1 < len(args) else None
            prefix = flag + "="
            if arg.startswith(prefix):
                return arg[len(prefix) :]
    return None


def _noise_settings_from_dataset(dataset: Optional[dict]) -> Tuple[float, int]:
    sigma_floor = float(DEFAULT_NOISE_SIGMA_FLOOR_DEG)
    min_samples = int(DEFAULT_NOISE_MIN_SAMPLES)
    if not isinstance(dataset, dict):
        return sigma_floor, min_samples
    config = dataset.get("config")
    if not isinstance(config, dict):
        return sigma_floor, min_samples
    noise_cfg = config.get("encoder_noise")
    if isinstance(noise_cfg, dict):
        floor_raw = noise_cfg.get("sigma_floor_deg")
        if isinstance(floor_raw, (int, float)) and np.isfinite(floor_raw):
            sigma_floor = float(floor_raw)
        min_raw = noise_cfg.get("min_samples")
        if isinstance(min_raw, (int, float)) and np.isfinite(min_raw):
            min_samples = int(min_raw)
    return sigma_floor, min_samples


def _summarize_encoder_noise(dataset: Optional[dict]) -> Optional[dict]:
    if not isinstance(dataset, dict):
        return None
    sweeps = dataset.get("sweeps")
    if not isinstance(sweeps, list):
        return None
    sigma_floor, min_samples = _noise_settings_from_dataset(dataset)
    total_points = 0
    points_with_sigma = 0
    low_samples = 0
    sigma_low = 0
    sigma_nonfinite = 0
    sigma_values: List[float] = []
    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        points = sweep.get("data_points")
        if not isinstance(points, list):
            continue
        for point in points:
            if not isinstance(point, dict):
                continue
            total_points += 1
            sigmas = point.get("sigma")
            if not isinstance(sigmas, list):
                continue
            points_with_sigma += 1
            if any(
                (not isinstance(v, (int, float)) or not np.isfinite(v))
                for v in sigmas
            ):
                sigma_nonfinite += 1
            if any(
                (isinstance(v, (int, float)) and np.isfinite(v) and v < sigma_floor)
                for v in sigmas
            ):
                sigma_low += 1
            sample_count = point.get("sample_count")
            if (
                isinstance(sample_count, (int, float))
                and np.isfinite(sample_count)
                and sample_count < min_samples
            ):
                low_samples += 1
            for v in sigmas:
                if isinstance(v, (int, float)) and np.isfinite(v):
                    sigma_values.append(float(v))
    if points_with_sigma == 0:
        if total_points > 0:
            return {
                "total_points": total_points,
                "points_with_sigma": 0,
                "sigma_floor": sigma_floor,
                "min_samples": min_samples,
            }
        return None
    median_sigma = float(np.median(sigma_values)) if sigma_values else float("nan")
    return {
        "total_points": total_points,
        "points_with_sigma": points_with_sigma,
        "median_sigma": median_sigma,
        "sigma_floor": sigma_floor,
        "min_samples": min_samples,
        "low_samples": low_samples,
        "sigma_low": sigma_low,
        "sigma_nonfinite": sigma_nonfinite,
    }


def _resolve_rrf_target(collector_args: Sequence[str]) -> Tuple[str, bool, Optional[int]]:
    env_server = os.environ.get("RRF_SERVER_URL")
    server = env_server or "http://localhost:8080"
    server_explicit = bool(env_server)

    server_arg = _arg_value(collector_args, "--server", "--rrf")
    if server_arg:
        server = server_arg
        server_explicit = True

    port = None
    port_arg = _arg_value(collector_args, "--port")
    if port_arg:
        try:
            parsed = int(port_arg)
        except ValueError:
            parsed = None
        if parsed is not None and parsed > 0:
            port = parsed

    if not server_explicit:
        port = port or DEFAULT_RRF_PORT
        server = f"http://localhost:{int(port)}"
    return server, server_explicit, port


def _apply_simulation_defaults(collector_args: Sequence[str], *, sim: bool) -> List[str]:
    args = list(collector_args)
    if sim:
        if not _arg_has_flag(args, "--no-spawn-rrf-simulator"):
            args.append("--no-spawn-rrf-simulator")
    else:
        if not _arg_has_flag(args, "--no-ws"):
            args.append("--no-ws")
        if not _arg_has_flag(args, "--no-spawn-rrf-simulator"):
            args.append("--no-spawn-rrf-simulator")
    return args


def _collector_has_buildup_override(args: Sequence[str]) -> bool:
    override_flags = {
        "--preserve-buildup-factor",
        "--preserveBuildupFactor",
        "--force-buildup-factor",
        "--forceBuildupFactor",
    }
    for arg in args:
        if not isinstance(arg, str):
            continue
        if arg in override_flags:
            return True
        if arg.startswith("--force-buildup-factor=") or arg.startswith("--forceBuildupFactor="):
            return True
    return False


def _inject_spool_collection_args(
    collector_args: Sequence[str],
    *,
    find_radii_mode: str,
    find_buildup_mode: str,
) -> Tuple[List[str], bool]:
    args = list(collector_args)
    search_spool = _spool_mode_enabled(find_radii_mode) or _spool_mode_enabled(find_buildup_mode)
    if search_spool and not _collector_has_buildup_override(args):
        args.append("--preserve-buildup-factor")
        return args, True
    return args, False


def _resolve_sim_config(
    *,
    machine_type: str,
    find_radii_mode: str,
    find_buildup_mode: str,
) -> str:
    env_cfg = os.environ.get("AUTOCAL_RRF_SIM_CONFIG")
    if isinstance(env_cfg, str) and env_cfg.strip():
        return env_cfg.strip()

    search_spool = _spool_mode_enabled(find_radii_mode) or _spool_mode_enabled(find_buildup_mode)
    if str(machine_type) == "slideprinter" and search_spool:
        candidate = REPO_ROOT / RRF_SIM_VSD_PATH / RRF_SIM_LINE_LAYER_CONFIG
        if candidate.exists():
            return RRF_SIM_LINE_LAYER_CONFIG

    return RRF_SIM_DEFAULT_CONFIG


def _effective_hp_sim_reset(
    *,
    sim: bool,
    hp_sim_reset: bool,
    find_radii_mode: str,
    find_buildup_mode: str,
) -> bool:
    """
    Decide whether hp-sim should be reset before collection starts.

    Why this exists:
    - In simulation, accepted M669/M666 updates persist in the running simulator and can
      silently change the "physical truth" between runs.
    - Spool-search workflows are especially sensitive to stale simulator state.

    So for `--sim` + spool search we reset by default unless not in simulation.
    """
    if not bool(sim):
        return False
    if bool(hp_sim_reset):
        return True
    if _spool_mode_enabled(find_radii_mode) or _spool_mode_enabled(find_buildup_mode):
        return True
    return False


def _send_rrf_gcode(server: str, gcode: str, *, timeout_s: float = 5.0) -> str:
    base = server.rstrip("/")
    url = f"{base}/machine/code"
    data = gcode.encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "text/plain"})
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        payload = resp.read()
    return payload.decode("utf-8", errors="replace")


def _wait_for_rrf_server(server: str, *, timeout_s: float = 7.0) -> None:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        try:
            _send_rrf_gcode(server, "M115", timeout_s=1.5)
            return
        except Exception:
            time.sleep(0.25)
    raise RuntimeError(f"rrf_simulator at {server} did not become ready in time")


def _start_rrf_simulator(port: int, *, sim_config: Optional[str] = None) -> subprocess.Popen:
    if not RRF_SIM_BINARY.exists():
        raise FileNotFoundError(f"rrf_simulator not found at {RRF_SIM_BINARY}")
    cfg = str(sim_config or RRF_SIM_DEFAULT_CONFIG)
    args = [
        str(RRF_SIM_BINARY),
        "--vsd",
        RRF_SIM_VSD_PATH,
        "-c",
        cfg,
        "--server",
        "-p",
        str(int(port)),
    ]
    return subprocess.Popen(
        args,
        cwd=str(REPO_ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _stop_process(proc: Optional[subprocess.Popen]) -> None:
    if not proc or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()


def _format_m669_command(anchors: np.ndarray, machine_type: str) -> Optional[str]:
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2 or anchors.shape[0] == 0:
        return None
    if anchors.shape[1] < 3:
        pad = np.zeros((anchors.shape[0], 3), dtype=float)
        pad[:, : anchors.shape[1]] = anchors
        anchors = pad
    if str(machine_type) in ("hangprinter_4", "hangprinter_5"):
        labels = ["A", "B", "C", "D", "I"]
    else:
        labels = ["A", "B", "C", "D", "I", "J", "K", "L", "O"]
    parts = []
    for idx, anchor in enumerate(anchors):
        label = labels[idx] if idx < len(labels) else f"P{idx}"
        parts.append(f"{label}{anchor[0]:.2f}:{anchor[1]:.2f}:{anchor[2]:.2f}")
    return "M669 " + " ".join(parts)


def _m669_from_plan(plan: Dict[str, object]) -> Optional[str]:
    cal = plan.get("calibration")
    if isinstance(cal, dict):
        gcode = cal.get("gcode")
        if isinstance(gcode, str) and gcode.strip().startswith("M669"):
            return gcode.strip()
    anchors = plan.get("anchors")
    if anchors is None:
        return None
    machine_type = str(plan.get("machine_type", ""))
    return _format_m669_command(np.asarray(anchors, dtype=float), machine_type)


def _format_m666_vector(values: np.ndarray, *, fmt: str = ".4g") -> Optional[str]:
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.size == 0 or not np.all(np.isfinite(arr)):
        return None
    return ":".join(f"{float(v):{fmt}}" for v in arr.tolist())


def _format_m666_from_length_model(length_model: object) -> Optional[str]:
    if not isinstance(length_model, dict):
        return None

    radii = np.asarray(
        length_model.get("effective_radii_mm", length_model.get("modeled_radii_mm", [])),
        dtype=float,
    ).reshape(-1)
    radii_spec = _format_m666_vector(radii, fmt=".4g")

    buildup_vals = np.asarray(length_model.get("modeled_buildup_factor", []), dtype=float).reshape(-1)
    if buildup_vals.size == 0 or not np.all(np.isfinite(buildup_vals)):
        try:
            k_scalar = float(length_model.get("buildup_factor_k"))
        except (TypeError, ValueError):
            k_scalar = float("nan")
        if np.isfinite(k_scalar):
            buildup_vals = np.asarray([k_scalar], dtype=float)
        else:
            buildup_vals = np.zeros(0, dtype=float)

    q_spec = None
    if buildup_vals.size > 0 and np.all(np.isfinite(buildup_vals)):
        if buildup_vals.size > 1 and np.max(np.abs(buildup_vals - float(buildup_vals[0]))) > 1e-9:
            q_spec = _format_m666_vector(buildup_vals, fmt=".6g")
        else:
            q_spec = f"{float(buildup_vals[0]):.6g}"

    if radii_spec is None and q_spec is None:
        return None

    parts = ["M666"]
    if radii_spec is not None:
        parts.append(f"R{radii_spec}")
    if q_spec is not None:
        parts.append(f"Q{q_spec}")
    return " ".join(parts)


def _m666_from_plan(plan: Dict[str, object]) -> Optional[str]:
    return _format_m666_from_length_model(plan.get("length_model"))

def _anchor_norms(anchors: np.ndarray) -> List[float]:
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2:
        return []
    return [float(v) for v in np.linalg.norm(anchors, axis=1).tolist()]


def _pairwise_anchor_distances(anchors: np.ndarray) -> Dict[str, float]:
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2:
        return {}
    n = int(anchors.shape[0])
    out: Dict[str, float] = {}
    for i in range(n):
        for j in range(i + 1, n):
            out[f"d{i}{j}"] = float(np.linalg.norm(anchors[i] - anchors[j]))
    return out


def _format_anchor_stats(anchors: np.ndarray) -> str:
    norms = _anchor_norms(anchors)
    pairwise = _pairwise_anchor_distances(anchors)
    norms_str = ", ".join(f"{v:.3f}" for v in norms) if norms else "n/a"
    pair_str = ", ".join(f"{k}={v:.3f}" for k, v in sorted(pairwise.items())) if pairwise else "n/a"
    return f"origin_norms=[{norms_str}] pairwise=[{pair_str}]"


def _estimate_anchor_covariance(info: np.ndarray, *, regularization: float) -> np.ndarray:
    info = np.asarray(info, dtype=float)
    reg = float(regularization)
    if not np.isfinite(reg) or reg < 0.0:
        reg = 0.0
    if reg:
        info = info + reg * np.eye(info.shape[0], dtype=float)
    return np.linalg.pinv(info)


def _covariance_report(cov: np.ndarray, *, top: int = 6, prefix: str = "std") -> str:
    cov = np.asarray(cov, dtype=float)
    if cov.ndim != 2 or cov.shape[0] != cov.shape[1]:
        return "covariance: n/a"
    diag = np.diag(cov)
    if diag.size == 0 or not np.any(np.isfinite(diag)):
        return "covariance: n/a"
    std = np.sqrt(np.maximum(diag, 0.0))
    idx = np.argsort(-std)
    parts = []
    for i in idx[: max(0, int(top))]:
        v = float(std[i])
        if not np.isfinite(v):
            continue
        parts.append(f"p{i}={v:.3g}mm")
    tag = str(prefix) if prefix else "std"
    return f"{tag}(" + ", ".join(parts) + ")"


def _fmt_float(value: object, *, fmt: str = ".4g", suffix: str = "") -> str:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return "n/a"
    if not np.isfinite(f):
        return "n/a"
    return f"{f:{fmt}}{suffix}"


def _covariance_scale_from_noise(noise_metrics: Optional[dict]) -> Tuple[Optional[float], Optional[str]]:
    if not isinstance(noise_metrics, dict):
        return None, None
    for key, label in (("chi2_red", "chi2_red"), ("J", "J")):
        val = noise_metrics.get(key)
        try:
            scale = float(val)
        except (TypeError, ValueError):
            continue
        if np.isfinite(scale) and scale > 0.0:
            return scale, label
    return None, None


def _scale_covariance(
    cov: np.ndarray, noise_metrics: Optional[dict]
) -> Tuple[np.ndarray, Optional[float], Optional[str]]:
    cov = np.asarray(cov, dtype=float)
    if cov.ndim != 2 or cov.shape[0] != cov.shape[1]:
        return cov, None, None
    scale, label = _covariance_scale_from_noise(noise_metrics)
    if scale is None:
        return cov, None, None
    return cov * float(scale), float(scale), label


def _covariance_diag_std(cov: np.ndarray) -> Optional[np.ndarray]:
    cov = np.asarray(cov, dtype=float)
    if cov.ndim != 2 or cov.shape[0] != cov.shape[1]:
        return None
    diag = np.diag(cov)
    if diag.size == 0:
        return None
    std = np.sqrt(np.maximum(diag, 0.0))
    if not np.any(np.isfinite(std)):
        return None
    return std


def _confidence_intervals(
    cov: np.ndarray, *, z_value: float = 1.96
) -> Optional[Dict[str, object]]:
    std = _covariance_diag_std(cov)
    if std is None:
        return None
    z = float(z_value)
    if not np.isfinite(z) or z <= 0.0:
        z = 1.96
    ci_half = z * std
    finite = np.isfinite(std)
    if not np.any(finite):
        return None
    max_std = float(np.max(std[finite]))
    max_ci = float(np.max(ci_half[finite]))
    return {
        "z": z,
        "std_mm": std.tolist(),
        "ci_half_mm": ci_half.tolist(),
        "max_std_mm": max_std,
        "max_ci_half_mm": max_ci,
    }


def _workspace_diag_mm(
    dataset: Optional[dict],
    anchors: Optional[np.ndarray],
    *,
    machine_type: Optional[str],
    num_anchors: Optional[int],
    dimensions: Optional[int],
) -> Optional[float]:
    dims = int(dimensions) if isinstance(dimensions, int) and dimensions > 0 else None
    if isinstance(dataset, dict):
        config = dataset.get("config")
        if isinstance(config, dict):
            raw_max_travel = config.get("max_travel_mm")
            if isinstance(raw_max_travel, (int, float)) and np.isfinite(raw_max_travel):
                travel = float(raw_max_travel)
                if travel > 0.0:
                    scale = float(np.sqrt(float(dims))) if dims else 1.0
                    return float(2.0 * travel * scale)

    if anchors is not None:
        anchors = np.asarray(anchors, dtype=float)
        if anchors.ndim == 2 and anchors.shape[0] >= 2:
            max_dist = 0.0
            for i in range(anchors.shape[0]):
                for j in range(i + 1, anchors.shape[0]):
                    dist = float(np.linalg.norm(anchors[i] - anchors[j]))
                    if np.isfinite(dist) and dist > max_dist:
                        max_dist = dist
            if max_dist > 0.0:
                return max_dist

    if machine_type is not None and num_anchors is not None and dimensions is not None:
        try:
            scale = float(
                np.sqrt(l2_scale_for_machine(str(machine_type), int(num_anchors), int(dimensions)))
            )
        except Exception:
            scale = float("nan")
        if np.isfinite(scale) and scale > 0.0:
            return scale
    return None


def _evaluate_cost_at_anchors(
    dataset: dict,
    anchors: np.ndarray,
    *,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    noise_normalized: bool,
    sigma_source: str,
) -> float:
    stage = 2 if bool(pointwise_filtering) else 0
    cost_fn = EllipseCostFunction(
        dataset,
        residual_threshold=float(residual_threshold),
        pointwise_residual_mode=str(pointwise_residual_mode),
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        pointwise_filter_stage=int(stage),
        use_noise_mean=bool(use_noise_mean),
        noise_normalized=bool(noise_normalized),
        sigma_source=str(sigma_source),
    )
    anchor_vec = np.asarray(anchors, dtype=float).ravel()
    return float(cost_fn.evaluate(anchor_vec))


def _unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    for idx in range(1, 10_000):
        candidate = path.with_name(f"{stem}_{idx}{suffix}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"Could not find available filename for {path}")


def _merge_sweep_datasets(base: dict, new: dict) -> dict:
    if not isinstance(base, dict) or not isinstance(new, dict):
        raise TypeError("datasets must be dicts")

    for key in ("machine_type", "num_anchors", "dimensions"):
        if base.get(key) != new.get(key):
            raise ValueError(
                f"Cannot merge datasets with different {key}: {base.get(key)!r} vs {new.get(key)!r}"
            )

    base_sweeps = base.get("sweeps")
    new_sweeps = new.get("sweeps")
    if not isinstance(base_sweeps, list) or not isinstance(new_sweeps, list):
        raise ValueError("Invalid sweep datasets (missing sweeps list)")

    merged_sweeps: List[dict] = []
    for s in base_sweeps:
        if isinstance(s, dict):
            merged_sweeps.append(s)
    for s in new_sweeps:
        if isinstance(s, dict):
            merged_sweeps.append(s)

    # Renumber sweeps to avoid duplicate ids (collector restarts at sweep_001 each run).
    for idx, sweep in enumerate(merged_sweeps, start=1):
        sweep["id"] = f"sweep_{idx:03d}"

    merged = dict(base)
    merged["timestamp"] = datetime.now().isoformat()
    merged["sweeps"] = merged_sweeps

    base_config = merged.get("config")
    new_config = new.get("config")
    if isinstance(new_config, dict):
        if not isinstance(base_config, dict):
            merged["config"] = dict(new_config)
        elif "force_tuning" in new_config or "torque_tuning" in new_config:
            updated = dict(base_config)
            updated["force_tuning"] = new_config.get("force_tuning") or new_config.get("torque_tuning")
            merged["config"] = updated
    return merged


def _renumber_sweeps(sweeps: Sequence[dict]) -> None:
    for idx, sweep in enumerate(sweeps, start=1):
        if isinstance(sweep, dict):
            sweep["id"] = f"sweep_{idx:03d}"


def _write_sweep_config_file(path: Path, cfg: SweepConfig) -> None:
    fixed = ",".join(str(i) for i in cfg.fixed_anchors)
    content = f"[{fixed}] {int(cfg.drive_anchor)} {int(cfg.sensor_anchor)}\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _fixed_targets_spec(cfg: SweepConfig) -> str:
    if len(cfg.fixed_deltas_mm) == 1:
        return str(float(cfg.fixed_deltas_mm[0]))
    return ",".join(str(float(v)) for v in cfg.fixed_deltas_mm)


def _suggested_collect_command(
    cfg_path: Path,
    cfg: SweepConfig,
    *,
    machine_type: str,
    output_file: Optional[Path] = None,
    extra_args: Sequence[str] = (),
) -> List[str]:
    cmd = [
        "node",
        "autocal/control/cli/collect_sweep_data.mjs",
        "--machineType",
        str(machine_type),
        "--sweep-config-file",
        str(cfg_path),
        "--fixed-targets",
        _fixed_targets_spec(cfg),
    ]
    if output_file is not None:
        cmd.extend(["--output-file", str(output_file)])
    cmd.extend(list(extra_args))
    return cmd


def _parse_csv_floats(spec: Optional[str]) -> Optional[List[float]]:
    if spec is None:
        return None
    parts = [p.strip() for p in str(spec).split(",")]
    out: List[float] = []
    for p in parts:
        if not p:
            continue
        try:
            v = float(p)
        except Exception:
            continue
        if np.isfinite(v):
            out.append(v)
    return out or None


def _parse_min_max_bounds(spec: Optional[str], *, label: str) -> Optional[Tuple[float, float]]:
    if spec is None:
        return None
    text = str(spec).strip()
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 2:
        raise ValueError(f"{label} must be 'min,max'")
    try:
        lo = float(parts[0])
        hi = float(parts[1])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric min,max") from exc
    if not np.isfinite(lo) or not np.isfinite(hi):
        raise ValueError(f"{label} min/max must be finite")
    if hi <= lo:
        raise ValueError(f"{label} requires max > min")
    return float(lo), float(hi)


def _normalize_spool_find_mode(mode: Optional[str]) -> str:
    text = str(mode or "off").strip().lower()
    if text in ("none", "false", "0"):
        text = "off"
    if text in ("true", "1", "on", "yes"):
        text = "per-anchor"
    if text not in _SPOOL_FIND_MODE_CHOICES:
        raise ValueError(
            f"invalid spool find mode '{mode}', expected one of: {', '.join(_SPOOL_FIND_MODE_CHOICES)}"
        )
    return text


def _spool_mode_enabled(mode: str) -> bool:
    return _normalize_spool_find_mode(mode) != "off"


def _normalize_theta0_mode(mode: Optional[str]) -> str:
    text = str(mode or "zero").strip().lower()
    if text not in _THETA0_MODE_CHOICES:
        raise ValueError(
            f"invalid theta0 mode '{mode}', expected one of: {', '.join(_THETA0_MODE_CHOICES)}"
        )
    return text


def _resolve_r0_bounds(
    base_radii_mm: np.ndarray,
    *,
    find_radii_mode: str,
    r0_bounds: Optional[Tuple[float, float]],
) -> Tuple[np.ndarray, np.ndarray]:
    base = np.asarray(base_radii_mm, dtype=float).reshape(-1)
    if base.size == 0:
        raise ValueError("missing base_radii_mm")
    mode = _normalize_spool_find_mode(find_radii_mode)
    if mode == "global":
        if r0_bounds is not None:
            lo, hi = float(r0_bounds[0]), float(r0_bounds[1])
        else:
            # Default spool-fit regime: effective radius at/above base, capped near 1.5x base.
            lo = float(np.max(base))
            hi = float(np.max(base)) * 1.5
        lo = max(lo, 1e-6)
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            raise ValueError("invalid global r0 bounds")
        return np.asarray([lo], dtype=float), np.asarray([hi], dtype=float)

    if r0_bounds is not None:
        lo = np.full(base.shape, max(float(r0_bounds[0]), 1e-6), dtype=float)
        hi = np.full(base.shape, float(r0_bounds[1]), dtype=float)
    else:
        # Default spool-fit regime: effective radius at/above base, capped near 1.5x base.
        lo = np.maximum(base, 1e-6)
        hi = np.maximum(base * 1.5, lo + 1e-6)
    if np.any(~np.isfinite(lo)) or np.any(~np.isfinite(hi)) or np.any(hi <= lo):
        raise ValueError("invalid per-anchor r0 bounds")
    return lo, hi


def _pack_radii_opt_vector(radii_mm: np.ndarray, *, find_radii_mode: str) -> np.ndarray:
    mode = _normalize_spool_find_mode(find_radii_mode)
    radii = np.asarray(radii_mm, dtype=float).reshape(-1)
    if mode == "global":
        return np.asarray([float(np.median(radii))], dtype=float)
    return np.asarray(radii, dtype=float)


def _unpack_radii_opt_vector(opt_vec: np.ndarray, *, num_anchors: int, find_radii_mode: str) -> np.ndarray:
    mode = _normalize_spool_find_mode(find_radii_mode)
    vals = np.asarray(opt_vec, dtype=float).reshape(-1)
    if mode == "global":
        if vals.size != 1:
            raise ValueError("global radius mode expects one optimization variable")
        return np.full(int(num_anchors), float(vals[0]), dtype=float)
    if vals.size != int(num_anchors):
        raise ValueError("per-anchor radius mode expects one variable per anchor")
    return np.asarray(vals, dtype=float)


def _default_modeled_buildup_values(
    *,
    num_anchors: int,
    find_buildup_factor_mode: str,
    buildup_factor: Optional[float],
) -> np.ndarray:
    mode = _normalize_spool_find_mode(find_buildup_factor_mode)
    k = 0.0
    if buildup_factor is not None:
        try:
            k = float(buildup_factor)
        except (TypeError, ValueError):
            k = 0.0
    if not np.isfinite(k):
        k = 0.0
    if mode == "global":
        return np.full(int(num_anchors), float(k), dtype=float)
    if mode == "per-anchor":
        return np.full(int(num_anchors), float(k), dtype=float)
    return np.full(int(num_anchors), float(k), dtype=float)


def _mode_opt_dim(mode: str, *, num_anchors: int) -> int:
    norm = _normalize_spool_find_mode(mode)
    if norm == "off":
        return 0
    if norm == "global":
        return 1
    return int(num_anchors)


def _resolve_b_bounds(
    modeled_buildup_factor: np.ndarray,
    *,
    find_buildup_mode: str,
    b_bounds: Optional[Tuple[float, float]],
) -> Tuple[np.ndarray, np.ndarray]:
    mode = _normalize_spool_find_mode(find_buildup_mode)
    base = np.asarray(modeled_buildup_factor, dtype=float).reshape(-1)
    if mode == "off":
        return np.zeros(0, dtype=float), np.zeros(0, dtype=float)
    if base.size <= 0:
        raise ValueError("missing modeled_buildup_factor")

    if b_bounds is not None:
        lo_raw = float(b_bounds[0])
        hi_raw = float(b_bounds[1])
    else:
        lo_raw = float(_DEFAULT_B_BOUNDS[0])
        hi_raw = float(_DEFAULT_B_BOUNDS[1])
    if not np.isfinite(lo_raw) or not np.isfinite(hi_raw) or hi_raw <= lo_raw:
        raise ValueError("invalid buildup-factor bounds")

    if mode == "global":
        return np.asarray([lo_raw], dtype=float), np.asarray([hi_raw], dtype=float)

    lo = np.full(base.shape, lo_raw, dtype=float)
    hi = np.full(base.shape, hi_raw, dtype=float)
    if np.any(~np.isfinite(lo)) or np.any(~np.isfinite(hi)) or np.any(hi <= lo):
        raise ValueError("invalid per-anchor buildup-factor bounds")
    return lo, hi


def _pack_buildup_opt_vector(buildup_factor: np.ndarray, *, find_buildup_mode: str) -> np.ndarray:
    mode = _normalize_spool_find_mode(find_buildup_mode)
    vals = np.asarray(buildup_factor, dtype=float).reshape(-1)
    if mode == "off":
        return np.zeros(0, dtype=float)
    if mode == "global":
        return np.asarray([float(np.median(vals))], dtype=float)
    return np.asarray(vals, dtype=float)


def _unpack_buildup_opt_vector(opt_vec: np.ndarray, *, num_anchors: int, find_buildup_mode: str) -> np.ndarray:
    mode = _normalize_spool_find_mode(find_buildup_mode)
    vals = np.asarray(opt_vec, dtype=float).reshape(-1)
    if mode == "off":
        return np.zeros(int(num_anchors), dtype=float)
    if mode == "global":
        if vals.size != 1:
            raise ValueError("global buildup-factor mode expects one optimization variable")
        return np.full(int(num_anchors), float(vals[0]), dtype=float)
    if vals.size != int(num_anchors):
        raise ValueError("per-anchor buildup-factor mode expects one variable per anchor")
    return np.asarray(vals, dtype=float)


def _pack_spool_opt_vector(
    *,
    radii_mm: np.ndarray,
    buildup_factor: np.ndarray,
    find_radii_mode: str,
    find_buildup_mode: str,
) -> np.ndarray:
    parts: List[np.ndarray] = []
    if _spool_mode_enabled(find_radii_mode):
        parts.append(_pack_radii_opt_vector(radii_mm, find_radii_mode=find_radii_mode))
    if _spool_mode_enabled(find_buildup_mode):
        parts.append(_pack_buildup_opt_vector(buildup_factor, find_buildup_mode=find_buildup_mode))
    if not parts:
        return np.zeros(0, dtype=float)
    return np.concatenate(parts).astype(float)


def _unpack_spool_opt_vector(
    opt_vec: np.ndarray,
    *,
    num_anchors: int,
    find_radii_mode: str,
    find_buildup_mode: str,
    fixed_radii_mm: np.ndarray,
    fixed_buildup_factor: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    mode_r = _normalize_spool_find_mode(find_radii_mode)
    mode_b = _normalize_spool_find_mode(find_buildup_mode)
    vals = np.asarray(opt_vec, dtype=float).reshape(-1)
    offset = 0

    if _spool_mode_enabled(mode_r):
        dim_r = _mode_opt_dim(mode_r, num_anchors=num_anchors)
        radii = _unpack_radii_opt_vector(vals[offset : offset + dim_r], num_anchors=num_anchors, find_radii_mode=mode_r)
        offset += dim_r
    else:
        radii = np.asarray(fixed_radii_mm, dtype=float).reshape(-1)

    if _spool_mode_enabled(mode_b):
        dim_b = _mode_opt_dim(mode_b, num_anchors=num_anchors)
        buildup = _unpack_buildup_opt_vector(
            vals[offset : offset + dim_b], num_anchors=num_anchors, find_buildup_mode=mode_b
        )
        offset += dim_b
    else:
        buildup = np.asarray(fixed_buildup_factor, dtype=float).reshape(-1)

    if offset != vals.size:
        raise ValueError("spool optimization vector length mismatch")
    return np.asarray(radii, dtype=float), np.asarray(buildup, dtype=float)


def _spool_opt_kinds(
    *,
    num_anchors: int,
    find_radii_mode: str,
    find_buildup_mode: str,
) -> List[str]:
    kinds: List[str] = []
    mode_r = _normalize_spool_find_mode(find_radii_mode)
    mode_b = _normalize_spool_find_mode(find_buildup_mode)
    kinds.extend(["r"] * _mode_opt_dim(mode_r, num_anchors=num_anchors))
    kinds.extend(["k"] * _mode_opt_dim(mode_b, num_anchors=num_anchors))
    return kinds


def _initial_spool_steps(x: np.ndarray, lo: np.ndarray, hi: np.ndarray, kinds: Sequence[str]) -> np.ndarray:
    out = np.zeros_like(x, dtype=float)
    for idx, kind in enumerate(kinds):
        xi = float(x[idx])
        span = float(hi[idx] - lo[idx]) if idx < hi.size else float("nan")
        if kind == "r":
            span_term = 0.02 * span if np.isfinite(span) and span > 0.0 else 0.0
            step = max(0.015 * max(abs(xi), 1.0), span_term, 0.25)
        else:
            span_term = 0.02 * span if np.isfinite(span) and span > 0.0 else 0.0
            step = max(0.08 * max(abs(xi), 1e-3), span_term, 1e-3)
        if np.isfinite(span) and span > 0.0:
            step = min(step, 0.5 * span)
        out[idx] = max(step, 1e-8)
    return out


def _spool_step_tolerances(kinds: Sequence[str]) -> np.ndarray:
    return np.asarray([1e-3 if kind == "r" else 1e-5 for kind in kinds], dtype=float)


def _spool_seed_candidates(x_seed: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> List[np.ndarray]:
    seed = np.asarray(x_seed, dtype=float).reshape(-1)
    if seed.size == 0:
        return [seed]
    if lo.size != seed.size or hi.size != seed.size:
        return [seed]

    span = np.asarray(hi - lo, dtype=float)
    if np.any(~np.isfinite(span)) or np.any(span <= 0.0):
        return [np.clip(seed, lo, hi)]

    candidates: List[np.ndarray] = []

    def _add(vec: np.ndarray) -> None:
        v = np.clip(np.asarray(vec, dtype=float).reshape(-1), lo, hi)
        for existing in candidates:
            if existing.shape == v.shape and np.allclose(existing, v, atol=1e-12, rtol=0.0):
                return
        candidates.append(v)

    _add(seed)
    _add(lo + 0.50 * span)  # midpoint seed
    _add(lo + 0.75 * span)  # high-side seed
    _add(lo + 0.25 * span)  # low-side seed
    return candidates


def _coordinate_descent_spool(
    x0: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    kinds: Sequence[str],
    max_iters: int,
    objective: Any,
) -> Tuple[np.ndarray, Dict[str, object]]:
    x = np.clip(np.asarray(x0, dtype=float).reshape(-1), lo, hi)
    if x.size == 0:
        cost0 = float(objective(x))
        return x, {
            "success": True,
            "message": "no spool parameters selected for fitting",
            "nfev": 1,
            "nit": 0,
            "start_cost": cost0,
            "fitted_cost": cost0,
            "step_final": [],
        }

    steps = _initial_spool_steps(x, lo, hi, kinds)
    tol = _spool_step_tolerances(kinds)
    start_cost = float(objective(x))
    best = float(start_cost)
    nfev = 1
    improved_any = False
    nit = 0
    stall_rounds = 0
    converged_by_stall = False

    for outer in range(max(1, int(max_iters))):
        nit = outer + 1
        best_before_round = float(best)
        improved_this_round = False
        for idx in range(x.size):
            if steps[idx] <= tol[idx]:
                continue
            best_local = float(best)
            improved_this_axis = False
            for direction in (1.0, -1.0):
                x_try = x.copy()
                cand = float(np.clip(x[idx] + direction * steps[idx], lo[idx], hi[idx]))
                if abs(cand - x[idx]) <= 1e-12:
                    continue
                x_try[idx] = cand
                score = float(objective(x_try))
                nfev += 1
                if np.isfinite(score) and score + 1e-12 < best_local:
                    best_local = float(score)
                    best = score
                    x = x_try
                    improved_this_round = True
                    improved_this_axis = True
                    improved_any = True
            if not improved_this_axis:
                steps[idx] *= 0.5
        if improved_this_round:
            gain = float(best_before_round - best)
            tol_gain = max(1e-6, 1e-4 * max(1.0, abs(best_before_round)))
            if gain <= tol_gain:
                stall_rounds += 1
            else:
                stall_rounds = 0
            if stall_rounds >= 3:
                converged_by_stall = True
                break
        else:
            if np.all(steps <= tol):
                break

    return x, {
        "success": bool(improved_any),
        "message": (
            "coordinate descent converged"
            if np.all(steps <= tol)
            else (
                "coordinate descent plateaued"
                if converged_by_stall
                else "coordinate descent reached iteration limit"
            )
        ),
        "nfev": int(nfev),
        "nit": int(nit),
        "start_cost": float(start_cost),
        "fitted_cost": float(best),
        "step_final": [float(v) for v in np.asarray(steps, dtype=float).tolist()],
    }


def _numerical_hessian_symmetric(
    x0: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    kinds: Sequence[str],
    objective: Any,
) -> Optional[np.ndarray]:
    x = np.asarray(x0, dtype=float).reshape(-1)
    if x.size == 0:
        return np.zeros((0, 0), dtype=float)
    if lo.size != x.size or hi.size != x.size:
        return None

    x = np.clip(x, lo, hi)
    n = int(x.size)
    h = np.asarray([1e-3 if kind == "r" else 1e-4 for kind in kinds], dtype=float)
    h = np.where(np.isfinite(h) & (h > 0.0), h, 1e-4)
    hess = np.zeros((n, n), dtype=float)

    f0 = float(objective(x))
    if not np.isfinite(f0):
        return None

    for i in range(n):
        hi_i = float(h[i])
        if hi_i <= 0.0:
            return None

        x_plus = x.copy()
        x_minus = x.copy()
        x_plus[i] = float(np.clip(x[i] + hi_i, lo[i], hi[i]))
        x_minus[i] = float(np.clip(x[i] - hi_i, lo[i], hi[i]))
        f_plus = float(objective(x_plus))
        f_minus = float(objective(x_minus))
        if not np.isfinite(f_plus) or not np.isfinite(f_minus):
            return None
        hess[i, i] = float((f_plus - (2.0 * f0) + f_minus) / (hi_i * hi_i))

        for j in range(i + 1, n):
            hi_j = float(h[j])
            if hi_j <= 0.0:
                return None
            x_pp = x.copy()
            x_pm = x.copy()
            x_mp = x.copy()
            x_mm = x.copy()
            x_pp[i] = float(np.clip(x[i] + hi_i, lo[i], hi[i]))
            x_pp[j] = float(np.clip(x[j] + hi_j, lo[j], hi[j]))
            x_pm[i] = float(np.clip(x[i] + hi_i, lo[i], hi[i]))
            x_pm[j] = float(np.clip(x[j] - hi_j, lo[j], hi[j]))
            x_mp[i] = float(np.clip(x[i] - hi_i, lo[i], hi[i]))
            x_mp[j] = float(np.clip(x[j] + hi_j, lo[j], hi[j]))
            x_mm[i] = float(np.clip(x[i] - hi_i, lo[i], hi[i]))
            x_mm[j] = float(np.clip(x[j] - hi_j, lo[j], hi[j]))
            f_pp = float(objective(x_pp))
            f_pm = float(objective(x_pm))
            f_mp = float(objective(x_mp))
            f_mm = float(objective(x_mm))
            if not (np.isfinite(f_pp) and np.isfinite(f_pm) and np.isfinite(f_mp) and np.isfinite(f_mm)):
                return None
            mixed = float((f_pp - f_pm - f_mp + f_mm) / (4.0 * hi_i * hi_j))
            hess[i, j] = mixed
            hess[j, i] = mixed

    if not np.all(np.isfinite(hess)):
        return None
    return hess


def _default_delta_range(
    *,
    max_travel_mm: Optional[float],
    observed_deltas: Sequence[float],
) -> Optional[Tuple[float, float]]:
    max_travel = None if max_travel_mm is None else float(max_travel_mm)
    if max_travel is None or not np.isfinite(max_travel) or max_travel <= 0.0:
        return None

    has_neg = any(float(v) < -1e-6 for v in observed_deltas)
    has_pos = any(float(v) > 1e-6 for v in observed_deltas)
    min_floor = 10.0

    if has_neg and has_pos:
        return -max_travel, max_travel
    if has_neg and not has_pos:
        hi = -min_floor if max_travel > min_floor else 0.0
        return -max_travel, hi
    lo = min_floor if max_travel > min_floor else 0.0
    return lo, max_travel


def _filter_candidates_by_spacing(
    candidates: Sequence[SweepConfig],
    observed: Sequence[SweepConfig],
    *,
    min_spacing_mm: float,
) -> List[SweepConfig]:
    spacing = float(min_spacing_mm)
    if not np.isfinite(spacing) or spacing <= 0.0:
        return list(candidates)

    history: Dict[int, Dict[int, List[float]]] = {}
    for cfg in observed:
        for anchor, delta in zip(cfg.fixed_anchors, cfg.fixed_deltas_mm):
            if not np.isfinite(delta):
                continue
            sign = 1 if float(delta) >= 0.0 else -1
            anchor_hist = history.setdefault(int(anchor), {1: [], -1: []})
            anchor_hist[sign].append(float(delta))

    if not history:
        return list(candidates)

    filtered: List[SweepConfig] = []
    for cfg in candidates:
        too_close = False
        for anchor, delta in zip(cfg.fixed_anchors, cfg.fixed_deltas_mm):
            if not np.isfinite(delta):
                continue
            anchor_hist = history.get(int(anchor))
            if not anchor_hist:
                continue
            sign = 1 if float(delta) >= 0.0 else -1
            if any(abs(float(delta) - obs) < spacing for obs in anchor_hist.get(sign, [])):
                too_close = True
                break
        if not too_close:
            filtered.append(cfg)

    return filtered


def _annotate_dataset_noise_model(
    dataset: Optional[dict],
    *,
    line_width_mm: float,
    find_radii_mode: str,
    find_buildup_mode: str,
    project_zero_tension: bool,
) -> None:
    if not isinstance(dataset, dict):
        return
    config = dataset.get("config")
    if not isinstance(config, dict):
        config = {}
        dataset["config"] = config

    mode_r = _normalize_spool_find_mode(find_radii_mode)
    mode_b = _normalize_spool_find_mode(find_buildup_mode)
    try:
        line_width = float(line_width_mm)
    except (TypeError, ValueError):
        line_width = float(DEFAULT_LAYER_LINE_WIDTH_MM)
    if not np.isfinite(line_width) or line_width < 0.0:
        line_width = float(DEFAULT_LAYER_LINE_WIDTH_MM)

    noise_model_raw = config.get(_NOISE_MODEL_CONFIG_KEY)
    noise_model = dict(noise_model_raw) if isinstance(noise_model_raw, dict) else {}
    noise_model["line_width_mm"] = float(line_width)
    noise_model["find_radii_mode"] = str(mode_r)
    noise_model["find_buildup_mode"] = str(mode_b)
    noise_model["solver_mode"] = f"{mode_r}/{mode_b}"
    noise_model["layered"] = bool(_spool_mode_enabled(mode_r) or _spool_mode_enabled(mode_b))
    noise_model["project_zero_tension"] = bool(project_zero_tension)
    config[_NOISE_MODEL_CONFIG_KEY] = noise_model


def _dataset_force_tuning(dataset: Optional[dict]) -> Optional[dict]:
    if not isinstance(dataset, dict):
        return None
    config = dataset.get("config")
    if not isinstance(config, dict):
        return None
    tuning = config.get("force_tuning") or config.get("torque_tuning")
    if not isinstance(tuning, dict):
        return None

    def _coerce(key: str, *aliases: str) -> Optional[float]:
        for k in (key, *aliases):
            val = tuning.get(k)
            if isinstance(val, (int, float)) and np.isfinite(val):
                return float(val)
        return None

    low = _coerce("force_low_n", "torque_low_n")
    mid = _coerce("force_mid_n", "torque_mid_n")
    max_force = _coerce("force_max_n", "torque_max_n")
    if low is None or mid is None or max_force is None:
        return None

    normalized = dict(tuning)
    normalized["force_low_n"] = low
    normalized["force_mid_n"] = mid
    normalized["force_max_n"] = max_force
    return normalized


def _collector_has_force_overrides(args: Sequence[str]) -> bool:
    force_flags = {
        "--force-low",
        "--forceLow",
        "--force-mid",
        "--forceMid",
        "--force-max",
        "--forceMax",
        "--auto-tune-force",
        "--autoTuneForce",
        "--no-auto-tune-force",
        "--noAutoTuneForce",
    }
    for arg in args:
        if not isinstance(arg, str):
            continue
        if arg in force_flags:
            return True
        if arg.startswith("--force-low=") or arg.startswith("--forceLow="):
            return True
        if arg.startswith("--force-mid=") or arg.startswith("--forceMid="):
            return True
        if arg.startswith("--force-max=") or arg.startswith("--forceMax="):
            return True
    return False


def _inject_force_args(
    dataset: dict, collector_args: Sequence[str]
) -> Tuple[List[str], Optional[dict], bool]:
    force_tuning = _dataset_force_tuning(dataset)
    args = list(collector_args)
    if force_tuning is None or _collector_has_force_overrides(args):
        return args, force_tuning, False

    args.extend(
        [
            "--force-low",
            str(force_tuning["force_low_n"]),
            "--force-mid",
            str(force_tuning["force_mid_n"]),
            "--force-max",
            str(force_tuning["force_max_n"]),
            "--no-auto-tune-force",
        ]
    )
    return args, force_tuning, True




def _plan_next_ellipse_sweep(
    dataset_path: Path,
    *,
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    robust_debug: bool,
    residuals_csv: Optional[Path],
    generate_report: bool,
    find_radii: str,
    find_buildup_factor: str,
    base_radii: Optional[List[float]],
    buildup_factor: Optional[float],
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    line_width: float,
    candidate_deltas: Optional[List[float]],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    fd_eps_mm: float,
    regularization: float,
    exclude_existing: bool,
    existing_tol_mm: float,
    min_fixed_delta_spacing_mm: float,
    top_k: int,
    write_cfg: Optional[Path],
    collector_output: Optional[Path],
    collector_args: Sequence[str],
) -> Dict[str, object]:
    dataset = _load_json(dataset_path)
    remapped_points = _normalize_dataset_point_roles(dataset)
    machine_type = _require_machine_type(dataset, context=str(dataset_path))
    num_anchors = int(dataset.get("num_anchors", 4))
    dimensions = int(dataset.get("dimensions", 3))
    warnings: List[str] = []
    if remapped_points > 0:
        warnings.append(f"point_role_remap_applied:{int(remapped_points)}")
    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)
    theta0_mode_norm = _normalize_theta0_mode(theta0_mode)
    search_radii = _spool_mode_enabled(find_radii_mode)
    search_buildup = _spool_mode_enabled(find_buildup_mode)
    if r0_bounds is not None and not _spool_mode_enabled(find_radii_mode):
        warnings.append("r0_bounds_ignored_without_find_radii")
    if r0_prior_sigma_mm is not None and not _spool_mode_enabled(find_radii_mode):
        warnings.append("r0_prior_sigma_mm_ignored_without_find_radii")
    if b_bounds is not None and not _spool_mode_enabled(find_buildup_mode):
        warnings.append("b_bounds_ignored_without_find_buildup_factor")
    if b_prior_sigma is not None and not _spool_mode_enabled(find_buildup_mode):
        warnings.append("b_prior_sigma_ignored_without_find_buildup_factor")

    est_buildup = 0.0
    if buildup_factor is not None:
        try:
            est_buildup = float(buildup_factor)
        except (TypeError, ValueError):
            est_buildup = 0.0
    if not np.isfinite(est_buildup):
        est_buildup = 0.0

    length_model: Optional[Dict[str, object]] = None
    spool_params: Optional[SpoolModelParams] = None
    dataset_for_estimation = dataset
    sweep_configs_for_info = dataset_sweep_configs(dataset)
    prefer_zero_tension_angles = _arg_has_flag(
        collector_args,
        "--project-zero-tension",
        "--projectZeroTension",
    )
    _annotate_dataset_noise_model(
        dataset,
        line_width_mm=float(line_width),
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        project_zero_tension=bool(prefer_zero_tension_angles),
    )

    if search_radii or search_buildup:
        validate_dataset_has_raw_angles(
            dataset,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        lm_params = _resolve_length_model_base_params(
            dataset,
            num_anchors=int(num_anchors),
            base_radii_override=base_radii,
        )
        base_radii_mm = np.asarray(lm_params["base_radii_mm"], dtype=float)
        spool_to_motor_gearing_factor = np.asarray(
            lm_params["spool_to_motor_gearing_factor"], dtype=float
        )
        mechanical_advantage = np.asarray(lm_params["mechanical_advantage"], dtype=float)
        lines_per_spool = np.asarray(lm_params["lines_per_spool"], dtype=float)
        modeled_buildup = _default_modeled_buildup_values(
            num_anchors=int(num_anchors),
            find_buildup_factor_mode=find_buildup_mode,
            buildup_factor=est_buildup,
        )

        seed_restarts = max(1, min(2, int(solve_restarts)))
        seed_iterations = max(60, min(200, int(solve_iterations)))
        seed_cal = calibrate_elliptical(
            dataset,
            output_path=None,
            residual_threshold=float(residual_threshold),
            num_restarts=int(seed_restarts),
            max_iterations=int(seed_iterations),
            method=str(solve_optimizer),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            verbose=False,
            use_parallel=False,
            pointwise_residual_mode=str(pointwise_residual_mode),
            robust_debug=False,
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            sigma_source=str(sigma_source),
            generate_report=False,
            residuals_csv=None,
        )
        seed_anchors = np.asarray(seed_cal["anchors"], dtype=float)
        effective_radii_mm, _fit_anchors, spool_params, dataset_for_estimation, radii_fit = (
            _estimate_effective_radii_with_spool_model(
                dataset,
                seed_anchors,
                find_radii_mode=find_radii_mode,
                find_buildup_mode=find_buildup_mode,
                base_radii_mm=base_radii_mm,
                modeled_buildup_factor=modeled_buildup,
                spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                mechanical_advantage=mechanical_advantage,
                lines_per_spool=lines_per_spool,
                r0_bounds=r0_bounds,
                b_bounds=b_bounds,
                r0_prior_sigma_mm=r0_prior_sigma_mm,
                b_prior_sigma=b_prior_sigma,
                spool_outer_iters=int(spool_outer_iters),
                spool_inner_iters=int(spool_inner_iters),
                theta0_mode=theta0_mode_norm,
                solve_restarts=int(solve_restarts),
                solve_iterations=int(solve_iterations),
                solve_optimizer=str(solve_optimizer),
                residual_threshold=float(residual_threshold),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                pointwise_residual_mode=str(pointwise_residual_mode),
                pointwise_filtering=bool(pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                sigma_source=str(sigma_source),
                robust_debug=bool(robust_debug),
                prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
            )
        )
        _ = _fit_anchors
        fitted_buildup = np.asarray(
            radii_fit.get("best_modeled_buildup_factor", modeled_buildup.tolist()),
            dtype=float,
        ).reshape(-1)
        if fitted_buildup.size != int(num_anchors):
            fitted_buildup = np.asarray(modeled_buildup, dtype=float).reshape(-1)
        k_summary = float(np.median(fitted_buildup)) if fitted_buildup.size else float(est_buildup)
        sweep_configs_for_info = sweep_configs_with_modeled_lengths(
            sweep_configs_for_info,
            spool_params,
        )
        cal = calibrate_elliptical(
            dataset_for_estimation,
            output_path=None,
            residual_threshold=float(residual_threshold),
            num_restarts=int(solve_restarts),
            max_iterations=int(solve_iterations),
            method=str(solve_optimizer),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            verbose=False,
            use_parallel=False,
            pointwise_residual_mode=str(pointwise_residual_mode),
            robust_debug=bool(robust_debug),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            sigma_source=str(sigma_source),
            generate_report=bool(generate_report),
            residuals_csv=residuals_csv,
            report_base_path=dataset_path,
        )

        length_model = {
            "coord_planning": "L_base_mm",
            "coord_estimation": "L_model_mm",
            "find_radii_mode": str(find_radii_mode),
            "find_buildup_factor_mode": str(find_buildup_mode),
            "find_radii": _spool_mode_enabled(find_radii_mode),
            "find_buildup_factor": _spool_mode_enabled(find_buildup_mode),
            "buildup_factor_k": float(k_summary),
            "base_radii_mm": [float(v) for v in base_radii_mm.tolist()],
            "effective_radii_mm": [float(v) for v in np.asarray(effective_radii_mm, dtype=float).tolist()],
            "modeled_radii_mm": [float(v) for v in np.asarray(effective_radii_mm, dtype=float).tolist()],
            "modeled_buildup_factor": [float(v) for v in np.asarray(fitted_buildup, dtype=float).tolist()],
            "spool_to_motor_gearing_factor": [
                float(v) for v in np.asarray(spool_to_motor_gearing_factor, dtype=float).tolist()
            ],
            "mechanical_advantage": [float(v) for v in np.asarray(mechanical_advantage, dtype=float).tolist()],
            "lines_per_spool": [float(v) for v in np.asarray(lines_per_spool, dtype=float).tolist()],
            "r0_bounds_mm": (
                [float(r0_bounds[0]), float(r0_bounds[1])]
                if isinstance(r0_bounds, tuple)
                else None
            ),
            "r0_prior_sigma_mm": (
                float(r0_prior_sigma_mm)
                if isinstance(r0_prior_sigma_mm, (int, float)) and np.isfinite(r0_prior_sigma_mm)
                else None
            ),
            "b_bounds": (
                [float(b_bounds[0]), float(b_bounds[1])]
                if isinstance(b_bounds, tuple)
                else None
            ),
            "b_prior_sigma": (
                float(b_prior_sigma)
                if isinstance(b_prior_sigma, (int, float)) and np.isfinite(b_prior_sigma)
                else None
            ),
            "spool_outer_iters": int(spool_outer_iters),
            "spool_inner_iters": int(spool_inner_iters),
            "theta0_mode": theta0_mode_norm,
            "radii_fit": radii_fit,
            "spool_fit": radii_fit,
        }
    else:
        cal = calibrate_elliptical(
            dataset,
            output_path=None,
            residual_threshold=float(residual_threshold),
            num_restarts=int(solve_restarts),
            max_iterations=int(solve_iterations),
            method=str(solve_optimizer),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            verbose=False,
            use_parallel=False,
            pointwise_residual_mode=str(pointwise_residual_mode),
            robust_debug=bool(robust_debug),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            sigma_source=str(sigma_source),
            generate_report=bool(generate_report),
            residuals_csv=residuals_csv,
        )

    anchors = np.asarray(cal["anchors"], dtype=float)
    cost = float(cal.get("cost", float("nan")))
    cost_raw = float(
        _evaluate_cost_at_anchors(
            dataset_for_estimation,
            anchors,
            residual_threshold=float(residual_threshold),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            pointwise_residual_mode=str(pointwise_residual_mode),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            noise_normalized=False,
            sigma_source=str(sigma_source),
        )
    )

    sweeps_obs = dataset_sweep_configs(dataset)
    l2_scale = l2_scale_for_machine(machine_type, num_anchors, dimensions)
    info_obs = total_information_matrix(
        anchors,
        sweep_configs_for_info,
        machine_type=machine_type,
        num_anchors=num_anchors,
        dimensions=dimensions,
        l2_scale=l2_scale,
        fd_eps_mm=float(fd_eps_mm),
    )
    cov = _estimate_anchor_covariance(info_obs, regularization=float(regularization))
    noise_metrics = None
    if isinstance(cal, dict):
        details = cal.get("details")
        if isinstance(details, dict):
            nm = details.get("noise_metrics")
            if isinstance(nm, dict):
                noise_metrics = nm
    cov_scaled, cov_scale, cov_scale_label = _scale_covariance(cov, noise_metrics)
    ci = _confidence_intervals(cov_scaled)
    workspace_diag = _workspace_diag_mm(
        dataset,
        anchors,
        machine_type=machine_type,
        num_anchors=num_anchors,
        dimensions=dimensions,
    )
    rank = None
    rank_deficient = False
    if info_obs.ndim == 2 and info_obs.shape[0] == info_obs.shape[1]:
        rank = int(np.linalg.matrix_rank(info_obs))
        rank_deficient = rank < info_obs.shape[0]
    cov_scaled_std = _covariance_diag_std(cov_scaled)
    if cov_scaled_std is None or not np.all(np.isfinite(cov_scaled_std)):
        warnings.append("covariance_nonfinite")

    observed_deltas: List[float] = []
    for cfg in sweeps_obs:
        observed_deltas.extend(list(cfg.fixed_deltas_mm))

    if candidate_deltas is None:
        config = dataset.get("config") if isinstance(dataset, dict) else None
        max_travel_mm = None
        if isinstance(config, dict):
            raw_max_travel = config.get("max_travel_mm")
            if isinstance(raw_max_travel, (int, float)) and np.isfinite(raw_max_travel):
                max_travel_mm = float(raw_max_travel)
        explicit_delta_range = delta_min is not None or delta_max is not None

        default_range = None
        if not explicit_delta_range:
            default_range = _default_delta_range(
                max_travel_mm=max_travel_mm,
                observed_deltas=observed_deltas,
            )
        if default_range is not None:
            lo, hi = default_range
        elif observed_deltas:
            lo = float(np.min(observed_deltas))
            hi = float(np.max(observed_deltas))
        else:
            lo, hi = -600.0, 600.0
        if delta_min is not None:
            lo = float(delta_min)
        if delta_max is not None:
            hi = float(delta_max)
        if not np.isfinite(lo) or not np.isfinite(hi) or abs(hi - lo) < 1e-9:
            fallback = _default_delta_range(
                max_travel_mm=max_travel_mm,
                observed_deltas=observed_deltas,
            )
            if fallback is not None:
                lo, hi = fallback
            else:
                lo, hi = -600.0, 600.0
        values = np.linspace(lo, hi, max(3, int(candidate_count)))
        candidate_deltas = [float(v) for v in values.tolist()]

    candidates = generate_candidate_sweeps(
        num_anchors=num_anchors,
        dimensions=dimensions,
        fixed_delta_values_mm=candidate_deltas,
        machine_type=machine_type,
    )
    candidates = _filter_candidates_by_spacing(
        candidates,
        sweeps_obs,
        min_spacing_mm=float(min_fixed_delta_spacing_mm),
    )
    if bool(exclude_existing):
        existing_keys = {
            cfg.normalized_key(tol_mm=float(existing_tol_mm))
            for cfg in sweeps_obs
        }
        candidates = [
            cfg
            for cfg in candidates
            if cfg.normalized_key(tol_mm=float(existing_tol_mm)) not in existing_keys
        ]

    if spool_params is not None:
        candidates_model = sweep_configs_with_modeled_lengths(candidates, spool_params)
        id_map = {id(cfg_model): cfg_base for cfg_base, cfg_model in zip(candidates, candidates_model)}
        ranked_model = rank_candidates_d_optimal(
            anchors,
            machine_type=machine_type,
            num_anchors=num_anchors,
            dimensions=dimensions,
            l2_scale=l2_scale,
            observed=sweep_configs_for_info,
            candidates=candidates_model,
            fd_eps_mm=float(fd_eps_mm),
            regularization=float(regularization),
            exclude_existing=False,
            existing_tol_mm=float(existing_tol_mm),
            top_k=int(top_k),
        )
        ranked = [(float(score), id_map.get(id(cfg_model), cfg_model)) for score, cfg_model in ranked_model]
    else:
        ranked = rank_candidates_d_optimal(
            anchors,
            machine_type=machine_type,
            num_anchors=num_anchors,
            dimensions=dimensions,
            l2_scale=l2_scale,
            observed=sweeps_obs,
            candidates=candidates,
            fd_eps_mm=float(fd_eps_mm),
            regularization=float(regularization),
            exclude_existing=False,
            existing_tol_mm=float(existing_tol_mm),
            top_k=int(top_k),
        )

    best_cfg = ranked[0][1] if ranked else None
    cfg_path = write_cfg or dataset_path.with_suffix(".active_sweep_cfg.txt")
    if best_cfg is not None:
        _write_sweep_config_file(cfg_path, best_cfg)

    collector_args_eff, force_tuning, force_args_applied = _inject_force_args(dataset, collector_args)
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    if "--return-to-origin" not in collector_args_eff and "--returnToOrigin" not in collector_args_eff:
        collector_args_eff.append("--return-to-origin")

    cmd = None
    if best_cfg is not None:
        cmd = _suggested_collect_command(
            cfg_path,
            best_cfg,
            machine_type=machine_type,
            output_file=collector_output,
            extra_args=collector_args_eff,
        )

    return {
        "dataset": dataset,
        "machine_type": machine_type,
        "num_anchors": num_anchors,
        "dimensions": dimensions,
        "calibration": cal,
        "anchors": anchors,
        "cost": cost,
        "cost_raw": cost_raw,
        "cost_noise_normalized": cost,
        "info": info_obs,
        "covariance": cov,
        "covariance_scaled": cov_scaled,
        "covariance_scale": cov_scale,
        "covariance_scale_label": cov_scale_label,
        "confidence_intervals": ci,
        "workspace_diag_mm": workspace_diag,
        "info_rank": rank,
        "info_rank_deficient": rank_deficient,
        "warnings": warnings,
        "ranked": ranked,
        "best_cfg": best_cfg,
        "cfg_path": cfg_path,
        "collect_command": cmd,
        "force_tuning": force_tuning,
        "force_args_applied": force_args_applied,
        "length_model": length_model,
        "line_width_mm": float(line_width),
        "dataset_for_estimation": dataset_for_estimation,
    }


def _print_ellipse_plan(
    plan: Dict[str, object],
    *,
    top_n: int = 5,
    print_command: bool = True,
    output_with_explanations: bool = False,
) -> None:
    cal = plan.get("calibration") or {}
    anchors = np.asarray(plan.get("anchors"), dtype=float)
    cost = float(plan.get("cost_noise_normalized", plan.get("cost", float("nan"))))
    cost_raw = plan.get("cost_raw")
    info = np.asarray(plan.get("info"), dtype=float)
    cov = np.asarray(plan.get("covariance"), dtype=float)
    cov_scaled = np.asarray(plan.get("covariance_scaled", cov), dtype=float)
    cov_scale = plan.get("covariance_scale")
    cov_scale_label = plan.get("covariance_scale_label")
    ci = plan.get("confidence_intervals")
    workspace_diag = plan.get("workspace_diag_mm")
    info_rank = plan.get("info_rank")
    rank_deficient = bool(plan.get("info_rank_deficient", False))
    warnings = plan.get("warnings") or []
    ranked = plan.get("ranked") or []
    cmd = plan.get("collect_command")
    length_model = plan.get("length_model")
    noise_summary = _summarize_encoder_noise(plan.get("dataset") if isinstance(plan, dict) else None)

    print("; Active ellipse calibration")
    if isinstance(cal, dict) and "gcode" in cal:
        print(str(cal["gcode"]))
    cost_raw_str = _fmt_float(cost_raw)
    cost_norm_str = _fmt_float(cost)
    print(
        f"; cost={cost_raw_str} cost_noise_normalized={cost_norm_str} {_format_anchor_stats(anchors)}"
    )
    if isinstance(length_model, dict):
        base = np.asarray(length_model.get("base_radii_mm", []), dtype=float)
        eff = np.asarray(length_model.get("effective_radii_mm", []), dtype=float)
        k_val = length_model.get("buildup_factor_k", 0.0)
        k_arr = np.asarray(length_model.get("modeled_buildup_factor", []), dtype=float)
        find_radii_mode = str(length_model.get("find_radii_mode", "off"))
        find_buildup_mode = str(length_model.get("find_buildup_factor_mode", "off"))
        base_str = ",".join(f"{float(v):.4g}" for v in base.tolist()) if base.size else "n/a"
        eff_str = ",".join(f"{float(v):.4g}" for v in eff.tolist()) if eff.size else "n/a"
        if k_arr.size and np.all(np.isfinite(k_arr)):
            if k_arr.size > 1 and np.max(np.abs(k_arr - float(k_arr[0]))) > 1e-9:
                k_str = "[" + ",".join(f"{float(v):.4g}" for v in k_arr.tolist()) + "]"
            else:
                k_str = _fmt_float(float(k_arr[0]))
        else:
            k_str = _fmt_float(k_val)
        print(
            f"; line_model: planning=L_base_mm estimation=L_model_mm "
            f"find_radii={find_radii_mode} "
            f"find_buildup_factor={find_buildup_mode} "
            f"k={k_str} base=[{base_str}] effective=[{eff_str}]"
        )
        m666_line_model = _format_m666_from_length_model(length_model)
        if isinstance(m666_line_model, str) and m666_line_model:
            print(f"; line_model_params (M666): {m666_line_model}")
        radii_fit = length_model.get("radii_fit")
        if isinstance(radii_fit, dict):
            history = radii_fit.get("history")
            if isinstance(history, list) and history:
                last = history[-1] if isinstance(history[-1], dict) else {}
                print(
                    f"; line_model_fit: mode={str(radii_fit.get('mode', 'n/a'))} "
                    f"outer_iters={_fmt_float(radii_fit.get('outer_iters'), fmt='.0f')} "
                    f"inner_iters={_fmt_float(radii_fit.get('inner_iters'), fmt='.0f')} "
                    f"best_cost={_fmt_float(radii_fit.get('best_cost'))} "
                    f"last_start={_fmt_float(last.get('start_cost'))} "
                    f"last_fit={_fmt_float(last.get('fitted_cost'))} "
                    f"last_cal={_fmt_float(last.get('cal_cost'))} "
                    f"nfev={_fmt_float(last.get('nfev'), fmt='.0f')} "
                    f"nit={_fmt_float(last.get('nit'), fmt='.0f')}"
                )
            else:
                print(
                    f"; line_model_fit: success={bool(radii_fit.get('success', False))} "
                    f"start_cost={_fmt_float(radii_fit.get('start_cost'))} "
                    f"fitted_cost={_fmt_float(radii_fit.get('fitted_cost'))} "
                    f"nfev={_fmt_float(radii_fit.get('nfev'), fmt='.0f')} "
                    f"nit={_fmt_float(radii_fit.get('nit'), fmt='.0f')}"
                )
    if isinstance(cal, dict):
        details = cal.get("details")
        if isinstance(details, dict):
            noise_metrics = details.get("noise_metrics")
            if isinstance(noise_metrics, dict):
                j_val = noise_metrics.get("J")
                chi2_red = noise_metrics.get("chi2_red")
                j_trim = noise_metrics.get("J_trimmed")
                chi2_trim = noise_metrics.get("chi2_red_trimmed")
                n_trim = noise_metrics.get("n_obs_trimmed")
                z_med = noise_metrics.get("median_abs_z")
                z_p95 = noise_metrics.get("p95_abs_z")
                outlier_ratio = noise_metrics.get("outlier_ratio")
                n_obs = noise_metrics.get("n_obs")
                norm_mode = noise_metrics.get("norm_mode")
                lengths_mode = noise_metrics.get("lengths_mode")
                j_str = _fmt_float(j_val)
                chi2_str = _fmt_float(chi2_red)
                j_trim_str = _fmt_float(j_trim)
                chi2_trim_str = _fmt_float(chi2_trim)
                med_str = _fmt_float(z_med)
                p95_str = _fmt_float(z_p95)
                outlier_str = _fmt_float(outlier_ratio)
                n_str = f"{int(n_obs)}" if isinstance(n_obs, (int, float)) and np.isfinite(n_obs) else "n/a"
                n_trim_str = (
                    f"{int(n_trim)}" if isinstance(n_trim, (int, float)) and np.isfinite(n_trim) else "n/a"
                )
                mode_str = str(norm_mode) if norm_mode is not None else "n/a"
                lengths_str = str(lengths_mode) if lengths_mode is not None else "n/a"
                print(
                    f"; noise_cost: J={j_str} chi2_red={chi2_str} |z|_med={med_str} |z|_p95={p95_str} "
                    f"outlier_ratio={outlier_str} N={n_str} mode={mode_str} lengths={lengths_str} "
                    f"J_trim={j_trim_str} chi2_red_trim={chi2_trim_str} N_trim={n_trim_str}"
                )
                sigma_min_mm = noise_metrics.get("sigma_min_mm")
                sigma_floor_deg = noise_metrics.get("sigma_floor_deg")
                sigma_source = noise_metrics.get("sigma_source")
                if sigma_min_mm is not None or sigma_floor_deg is not None or sigma_source is not None:
                    min_str = _fmt_float(sigma_min_mm, suffix="mm")
                    floor_str = _fmt_float(sigma_floor_deg, suffix="deg")
                    source_str = str(sigma_source) if sigma_source is not None else "n/a"
                    print(
                        f"; noise_norm_floor: min_sigma={min_str} floor_deg={floor_str} source={source_str}"
                    )
                sigma_encoder = noise_metrics.get("sigma_encoder_mm")
                sigma_friction = noise_metrics.get("sigma_friction_cogging_mm")
                sigma_flex = noise_metrics.get("sigma_flex_mm")
                sigma_floor_term = noise_metrics.get("sigma_floor_term_mm")
                sigma_non_layered = noise_metrics.get("sigma_non_layered_mm")
                sigma_layer_changes = noise_metrics.get("sigma_layer_changes_mm")
                sigma_mode_add = noise_metrics.get("sigma_mode_addition_mm")
                sigma_total = noise_metrics.get("sigma_total_mm")
                sigma_mode = noise_metrics.get("sigma_solver_mode")
                sigma_mode_factor = noise_metrics.get("sigma_solver_mode_factor")
                sigma_line_width = noise_metrics.get("sigma_line_width_mm")
                sigma_base_radius = noise_metrics.get("sigma_base_radius_mm")
                sigma_layered = noise_metrics.get("sigma_layered_enabled")
                if any(
                    val is not None
                    for val in (
                        sigma_encoder,
                        sigma_friction,
                        sigma_flex,
                        sigma_floor_term,
                        sigma_non_layered,
                        sigma_layer_changes,
                        sigma_mode_add,
                        sigma_total,
                        sigma_mode,
                        sigma_mode_factor,
                        sigma_line_width,
                        sigma_base_radius,
                        sigma_layered,
                    )
                ):
                    layered_str = "n/a"
                    if isinstance(sigma_layered, bool):
                        layered_str = "true" if sigma_layered else "false"
                    print(
                        "; noise_sigma_mm: "
                        f"encoder={_fmt_float(sigma_encoder, suffix='mm')} "
                        f"friction_cogging={_fmt_float(sigma_friction, suffix='mm')} "
                        f"flex={_fmt_float(sigma_flex, suffix='mm')} "
                        f"floor={_fmt_float(sigma_floor_term, suffix='mm')} "
                        f"non_layered={_fmt_float(sigma_non_layered, suffix='mm')} "
                        f"layer_changes={_fmt_float(sigma_layer_changes, suffix='mm')} "
                        f"mode_add={_fmt_float(sigma_mode_add, suffix='mm')} "
                        f"total={_fmt_float(sigma_total, suffix='mm')} "
                        f"layered={layered_str} "
                        f"mode={str(sigma_mode) if sigma_mode is not None else 'n/a'} "
                        f"mode_factor={_fmt_float(sigma_mode_factor)} "
                        f"line_width={_fmt_float(sigma_line_width, suffix='mm')} "
                        f"base_radius={_fmt_float(sigma_base_radius, suffix='mm')}"
                    )
    if info.ndim == 2 and info.shape[0] == info.shape[1]:
        rank_val = int(info_rank) if isinstance(info_rank, int) else int(np.linalg.matrix_rank(info))
        print(f"; info rank: {rank_val}/{info.shape[0]} {_covariance_report(cov)}")
        if cov_scale is not None:
            scale_str = _fmt_float(cov_scale)
            label = str(cov_scale_label or "scale")
            print(
                f"; covariance_scaled: {label}={scale_str} "
                f"{_covariance_report(cov_scaled, prefix='std_scaled')}"
            )
        if isinstance(ci, dict):
            max_std = ci.get("max_std_mm")
            max_ci = ci.get("max_ci_half_mm")
            scale_note = "scale=unscaled"
            if cov_scale is not None:
                scale_note = f"scale={str(cov_scale_label or 'scale')}={_fmt_float(cov_scale)}"
            rel_std = None
            rel_ci = None
            if (
                isinstance(workspace_diag, (int, float))
                and np.isfinite(workspace_diag)
                and workspace_diag > 0.0
            ):
                rel_std = float(max_std) / float(workspace_diag) if max_std is not None else None
                rel_ci = float(max_ci) / float(workspace_diag) if max_ci is not None else None
            ws_str = _fmt_float(workspace_diag)
            max_std_str = _fmt_float(max_std, suffix="mm")
            max_ci_str = _fmt_float(max_ci, suffix="mm")
            rel_std_str = _fmt_float(rel_std)
            rel_ci_str = _fmt_float(rel_ci)
            print(
                f"; CI95: max_std={max_std_str} max_ci={max_ci_str} "
                f"workspace={ws_str} rel_std={rel_std_str} rel_ci={rel_ci_str} {scale_note}"
            )
        if output_with_explanations and rank_deficient:
            print(
                "; note: info rank deficiency can arise from symmetric setups or limited sweep/constraint diversity."
            )
            print(
                "; note: covariance values can grow under rank deficiency, even when recovered anchors are good."
            )
        if isinstance(warnings, list) and warnings:
            extra = [w for w in warnings if isinstance(w, str) and not w.startswith("info_rank_deficient")]
            if extra:
                print("; warning_flags: " + ", ".join(extra))
    if isinstance(noise_summary, dict):
        total_points = int(noise_summary.get("total_points", 0))
        points_with_sigma = int(noise_summary.get("points_with_sigma", 0))
        if points_with_sigma == 0 and total_points > 0:
            print("; encoder_noise: missing (legacy dataset)")
        else:
            median_sigma = noise_summary.get("median_sigma")
            sigma_floor = noise_summary.get("sigma_floor")
            min_samples = int(noise_summary.get("min_samples", 0))
            sigma_str = f"{float(median_sigma):.4g}deg" if np.isfinite(median_sigma) else "n/a"
            print(
                f"; encoder_noise: points={points_with_sigma}/{total_points} "
                f"median_sigma={sigma_str} floor={float(sigma_floor):.4g}deg min_samples={min_samples}"
            )
            low_samples = int(noise_summary.get("low_samples", 0))
            sigma_low = int(noise_summary.get("sigma_low", 0))
            sigma_nonfinite = int(noise_summary.get("sigma_nonfinite", 0))
            if low_samples > 0 or sigma_low > 0 or sigma_nonfinite > 0:
                print(
                    f"; encoder_noise_flags: low_samples={low_samples} "
                    f"sigma_below_floor={sigma_low} sigma_nonfinite={sigma_nonfinite}"
                )

    if isinstance(ranked, list) and ranked:
        best_score, best_cfg = ranked[0]
        print(
            f"; next_sweep score={float(best_score):.6g} fixed={list(best_cfg.fixed_anchors)} "
            f"targets={list(best_cfg.fixed_deltas_mm)} pair=[{best_cfg.drive_anchor},{best_cfg.sensor_anchor}]"
        )
        if len(ranked) > 1:
            print("; top_candidates:")
            for score, cfg in ranked[: min(int(top_n), len(ranked))]:
                print(
                    f";   {float(score):.6g} fixed={list(cfg.fixed_anchors)} targets={list(cfg.fixed_deltas_mm)} "
                    f"pair=[{cfg.drive_anchor},{cfg.sensor_anchor}]"
                )
    else:
        print("; No valid candidate sweeps found (check delta range and anchor estimate).")

    if bool(print_command) and isinstance(cmd, list) and cmd:
        print("; collect_command:")
        print(";   " + " ".join(str(x) for x in cmd))
    if plan.get("force_args_applied") and isinstance(plan.get("force_tuning"), dict):
        ft = plan["force_tuning"]
        print(
            "; reusing force tuning:"
            f" low={float(ft.get('force_low_n', float('nan'))):.4g}N"
            f" mid={float(ft.get('force_mid_n', float('nan'))):.4g}N"
            f" max={float(ft.get('force_max_n', float('nan'))):.4g}N"
        )


def ellipse_active(
    dataset_path: Path,
    *,
    machine_type: str,
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    robust_debug: bool,
    residuals_csv: Optional[Path],
    generate_report: bool,
    find_radii: str,
    find_buildup_factor: str,
    base_radii: Optional[List[float]],
    buildup_factor: Optional[float],
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    line_width: float,
    candidate_deltas: Optional[List[float]],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    fd_eps_mm: float,
    regularization: float,
    exclude_existing: bool,
    existing_tol_mm: float,
    min_fixed_delta_spacing_mm: float,
    top_k: int,
    write_cfg: Optional[Path],
    print_command: bool,
    collector_args: Sequence[str],
    collect_once: bool,
    collector_output: Optional[Path],
    merged_output_dataset: Optional[Path],
    sim: bool,
    keep_sim_alive: bool,
    hp_sim_reset: bool,
    output_with_explanations: bool,
) -> int:
    machine_type = _require_machine_type(
        _load_json(dataset_path),
        expected=machine_type,
        context=str(dataset_path),
        mismatch="warn",
    )
    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)
    hp_sim_reset_eff = _effective_hp_sim_reset(
        sim=bool(sim),
        hp_sim_reset=bool(hp_sim_reset),
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    if bool(sim) and hp_sim_reset_eff and not bool(hp_sim_reset):
        print("; auto-enabling --hp-sim-reset for spool search in simulation")
    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    if sim and hp_sim_reset_eff and collect_once and not _arg_has_flag(collector_args_eff, "--hp-sim-reset"):
        collector_args_eff.append("--hp-sim-reset")
    _, server_explicit, port = _resolve_rrf_target(collector_args)
    sim_process: Optional[subprocess.Popen] = None
    sim_config = _resolve_sim_config(
        machine_type=machine_type,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )

    if sim and collect_once and not server_explicit and not user_no_spawn:
        target_port = port or DEFAULT_RRF_PORT
        print(f"; starting rrf_simulator at http://localhost:{target_port} (config: {sim_config})")
        sim_process = _start_rrf_simulator(target_port, sim_config=sim_config)
        _wait_for_rrf_server(f"http://localhost:{target_port}")

    plan = _plan_next_ellipse_sweep(
        dataset_path,
        solve_restarts=solve_restarts,
        solve_iterations=solve_iterations,
        solve_optimizer=solve_optimizer,
        residual_threshold=residual_threshold,
        spring_k_multiplier=spring_k_multiplier,
        use_flex=use_flex,
        pointwise_residual_mode=pointwise_residual_mode,
        pointwise_filtering=pointwise_filtering,
        pointwise_global_mad=pointwise_global_mad,
        sweep_wise_filtering=sweep_wise_filtering,
        sweep_metric=sweep_metric,
        use_noise_mean=use_noise_mean,
        sigma_source=sigma_source,
        robust_debug=robust_debug,
        residuals_csv=residuals_csv,
        generate_report=generate_report,
        find_radii=str(find_radii),
        find_buildup_factor=str(find_buildup_factor),
        base_radii=base_radii,
        buildup_factor=buildup_factor,
        r0_bounds=r0_bounds,
        b_bounds=b_bounds,
        r0_prior_sigma_mm=r0_prior_sigma_mm,
        b_prior_sigma=b_prior_sigma,
        spool_outer_iters=int(spool_outer_iters),
        spool_inner_iters=int(spool_inner_iters),
        theta0_mode=str(theta0_mode),
        line_width=float(line_width),
        candidate_deltas=candidate_deltas,
        candidate_count=candidate_count,
        delta_min=delta_min,
        delta_max=delta_max,
        fd_eps_mm=fd_eps_mm,
        regularization=regularization,
        exclude_existing=exclude_existing,
        existing_tol_mm=existing_tol_mm,
        min_fixed_delta_spacing_mm=min_fixed_delta_spacing_mm,
        top_k=top_k,
        write_cfg=write_cfg,
        collector_output=collector_output,
        collector_args=collector_args_eff,
    )

    _print_ellipse_plan(
        plan,
        top_n=5,
        print_command=print_command,
        output_with_explanations=bool(output_with_explanations),
    )

    ranked = plan.get("ranked") or []
    cmd = plan.get("collect_command")
    if not ranked or not isinstance(cmd, list):
        return 2

    if not collect_once:
        if sim_process and not keep_sim_alive:
            _stop_process(sim_process)
        return 0

    if collector_output is None:
        raise ValueError("--collect-once requires --collector-output")

    print(f"; running: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True)
    finally:
        if sim_process and not keep_sim_alive:
            _stop_process(sim_process)

    base_dataset = _load_json(dataset_path)
    new_dataset = _load_json(collector_output)
    if plan.get("force_args_applied") and isinstance(plan.get("force_tuning"), dict):
        cfg = new_dataset.get("config")
        if isinstance(cfg, dict):
            cfg["force_tuning"] = dict(plan["force_tuning"])
            _write_json(collector_output, new_dataset)

    merged = _merge_sweep_datasets(base_dataset, new_dataset)
    out_path = merged_output_dataset or dataset_path
    _write_json(out_path, merged)
    print(f"; merged dataset written to {out_path}")
    try:
        if collector_output is not None and out_path != collector_output:
            collector_output.unlink()
    except OSError:
        pass
    return 0


def merge_datasets(base_dataset: Path, extra_datasets: Sequence[Path], *, output: Optional[Path] = None) -> int:
    merged: dict = _load_json(base_dataset)
    for path in extra_datasets:
        merged = _merge_sweep_datasets(merged, _load_json(Path(path)))
    out_path = output or base_dataset
    _write_json(out_path, merged)
    sweeps = merged.get("sweeps", [])
    count = len(sweeps) if isinstance(sweeps, list) else "?"
    print(f"; merged dataset written to {out_path} sweeps={count}")
    return 0


def full_auto_loop(
    *,
    work_dataset: Optional[Path],
    machine_type: str,
    max_steps: int,
    stop_cost: Optional[float],
    stop_std_mm: Optional[float],
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    robust_debug: bool,
    residuals_csv: Optional[Path],
    generate_report: bool,
    find_radii: str,
    find_buildup_factor: str,
    base_radii: Optional[List[float]],
    buildup_factor: Optional[float],
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    line_width: float,
    candidate_deltas: Optional[List[float]],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    fd_eps_mm: float,
    regularization: float,
    exclude_existing: bool,
    existing_tol_mm: float,
    min_fixed_delta_spacing_mm: float,
    top_k: int,
    write_cfg: Optional[Path],
    collector_args: Sequence[str],
    sim: bool,
    keep_sim_alive: bool,
    hp_sim_reset: bool,
    sweep_points: Optional[int],
    output_with_explanations: bool,
    full_auto_runs: Optional[Sequence[str]],
    full_auto_log: Optional[Path],
    patience: int,
    full_auto_verbose: bool,
) -> int:
    if work_dataset is not None:
        dataset_path = Path(work_dataset)
    else:
        dataset_path = Path("autocal/data/default_dataset.json")
    work_path = dataset_path
    text_log_path = _unique_path(dataset_path.with_name(f"{dataset_path.stem}.full_auto.log"))
    text_log_path.parent.mkdir(parents=True, exist_ok=True)
    text_log_path.write_text("", encoding="utf-8")
    log_handle = text_log_path.open("a", encoding="utf-8")

    def _log_line(msg: str) -> None:
        log_handle.write(str(msg) + "\n")
        log_handle.flush()

    def _log_console(msg: str) -> None:
        print(msg)
        _log_line(f"Wrote to console: {msg}")

    def _log_context():
        stack = contextlib.ExitStack()
        stack.enter_context(redirect_stdout(log_handle))
        stack.enter_context(redirect_stderr(log_handle))
        return stack

    def _emit_summary_and_send(best_plan: Dict[str, object]) -> int:
        dataset_now = _load_json(work_path)
        sweep_ids: List[str] = []
        sweeps_now = dataset_now.get("sweeps")
        if isinstance(sweeps_now, list):
            for sweep in sweeps_now:
                if isinstance(sweep, dict) and sweep.get("id"):
                    sweep_ids.append(str(sweep["id"]))
        m669 = _m669_from_plan(best_plan)
        m666 = _m666_from_plan(best_plan)
        anchors = best_plan.get("anchors")
        anchor_str = ""
        if isinstance(anchors, np.ndarray):
            anchor_str = np.array2string(anchors, precision=2, separator=", ")
        elif anchors is not None:
            anchor_str = str(np.asarray(anchors))
        quality_label = _solution_quality_label(best_cost)
        _log_console("")
        _log_console("== Calibration summary ==")
        _log_console(f"Found parameters of {quality_label} quality")
        _log_console(f"Best cost: {_fmt_float(best_cost)}")
        if m669:
            _log_console(f"Parameters (M669): {m669}")
        elif anchor_str:
            _log_console(f"Anchors: {anchor_str}")
        if m666:
            _log_console(f"Line model (M666): {m666}")
        if has_variants:
            best_flags = str(best_meta.get("flags", "")).strip()
            best_run = str(best_meta.get("run_id", "")).strip()
            label = best_flags or best_run or "default"
            _log_console(f"Variant/flag setup giving best cost: {label}")
        _log_console(_solution_quality_message(best_cost))

        if m669:
            _log_console(f"Sending {m669} to {rrf_server}")
            try:
                reply = _send_rrf_gcode(rrf_server, m669)
            except Exception as exc:
                _log_console(f"; failed to send M669: {exc}")
                return _finalize(1)
            reply = reply.strip()
            if reply:
                _log_line(f"; M669 reply: {reply}")
        else:
            _log_console("Sending M669 skipped (no command available)")
        return _finalize(0)

    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)

    _log_console(f"Writing additional info to log: {text_log_path}")
    if dataset_path.exists():
        with _log_context():
            machine_type = _require_machine_type(
                _load_json(dataset_path),
                expected=machine_type,
                context=str(dataset_path),
                mismatch="warn",
            )

    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    hp_sim_reset_eff = _effective_hp_sim_reset(
        sim=bool(sim),
        hp_sim_reset=bool(hp_sim_reset),
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    if bool(sim) and hp_sim_reset_eff and not bool(hp_sim_reset):
        _log_line("; auto-enabling --hp-sim-reset for spool search in simulation")
    sweep_points_value = None
    raw_sweep_points = _arg_value(collector_args_eff, "--sweepPoints", "--sweep-points")
    if raw_sweep_points is not None:
        try:
            parsed = int(raw_sweep_points)
        except ValueError:
            parsed = None
        if parsed is not None and parsed > 0:
            sweep_points_value = parsed
    if sweep_points is not None:
        try:
            parsed = int(sweep_points)
        except (TypeError, ValueError):
            parsed = None
        if parsed is not None and parsed > 0:
            sweep_points_value = parsed
            if raw_sweep_points is None:
                collector_args_eff.extend(["--sweepPoints", str(parsed)])
    reset_pending = bool(sim and hp_sim_reset_eff)
    rrf_server, server_explicit, port = _resolve_rrf_target(collector_args)
    sim_process: Optional[subprocess.Popen] = None
    sim_config = _resolve_sim_config(
        machine_type=machine_type,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )

    if sim and not server_explicit and not user_no_spawn:
        target_port = port or DEFAULT_RRF_PORT
        _log_line(f"; starting rrf_simulator at http://localhost:{target_port} (config: {sim_config})")
        sim_process = _start_rrf_simulator(target_port, sim_config=sim_config)
        _wait_for_rrf_server(f"http://localhost:{target_port}")

    def _finalize(code: int) -> int:
        if sim_process and not keep_sim_alive:
            _stop_process(sim_process)
        try:
            log_handle.close()
        except Exception:
            pass
        return code

    if not dataset_path.exists():
        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        bootstrap_cfg = dataset_path.with_suffix(".bootstrap_cfg.txt")
        bootstrap_cfg.parent.mkdir(parents=True, exist_ok=True)
        bootstrap_cfg.write_text(
            "[0] 1 2\n[1] 0 2\n[2] 0 1\n",
            encoding="utf-8",
        )

        def _strip_conflicts(argv: Sequence[str]) -> List[str]:
            skip_with_value = {
                "--output-file",
                "--output",
                "--out",
                "--observability-file",
                "--obs-file",
                "--machineType",
                "--machine-type",
                "--sweepMethod",
                "--sweep-method",
                "--sweep-config-file",
                "--sweep-config",
                "--sweepFile",
                "--fixed-targets",
                "--fixedTargets",
                "--fixed-target",
                "--max-travel-mm",
                "--max-travel",
            }
            out: List[str] = []
            i = 0
            while i < len(argv):
                arg = str(argv[i])
                if arg in skip_with_value:
                    i += 2
                    continue
                out.append(arg)
                i += 1
            return out

        argv_eff = _strip_conflicts(list(collector_args_eff))
        if reset_pending and not _arg_has_flag(argv_eff, "--hp-sim-reset"):
            argv_eff.append("--hp-sim-reset")
        if "--return-to-origin" not in argv_eff and "--returnToOrigin" not in argv_eff:
            argv_eff.append("--return-to-origin")

        cmd = [
            "node",
            "autocal/control/cli/collect_sweep_data.mjs",
            "--machineType",
            str(machine_type),
            "--sweep-config-file",
            str(bootstrap_cfg),
            "--output-file",
            str(dataset_path),
            *argv_eff,
        ]
        _log_line("; bootstrapping dataset (3 sweeps, auto size-tune):")
        _log_line(";   " + " ".join(cmd))
        with _log_context():
            subprocess.run(cmd, check=True, stdout=log_handle, stderr=log_handle)
        reset_pending = False
        _log_line(f"; bootstrap dataset written to {dataset_path}")

    replay_mode = False
    replay_sweeps: List[dict] = []
    replay_index = 0
    replay_work_path: Optional[Path] = None
    if dataset_path.exists():
        dataset_full = _load_json(dataset_path)
        sweeps_full = dataset_full.get("sweeps")
        if isinstance(sweeps_full, list) and len(sweeps_full) > 3:
            replay_mode = True
            replay_work_path = dataset_path.with_suffix(".replay_tmp.json")
            work_path = replay_work_path
            initial = [s for s in sweeps_full[:3] if isinstance(s, dict)]
            replay_sweeps = [s for s in sweeps_full[3:] if isinstance(s, dict)]
            dataset_work = dict(dataset_full)
            dataset_work["timestamp"] = datetime.now().isoformat()
            dataset_work["sweeps"] = initial
            _renumber_sweeps(initial)
            _write_json(work_path, dataset_work)
            _log_line(
                f"; full-auto replay: starting from 3 sweeps, "
                f"replaying {len(replay_sweeps)} additional sweeps"
            )
            _log_line(
                f"; full-auto replay: using {work_path} "
                f"(original {dataset_path} left untouched)"
            )

    runs = _build_full_auto_runs(full_auto_runs)
    log_path = Path(full_auto_log) if full_auto_log is not None else _unique_path(
        dataset_path.with_name(f"{dataset_path.stem}.full_auto_log.jsonl")
    )
    stop_file = _full_auto_stop_path(dataset_path)
    _append_jsonl(
        log_path,
        {
            "timestamp": datetime.now().isoformat(),
            "event": "start",
            "dataset": str(dataset_path),
            "runs": runs,
            "replay_mode": replay_mode,
            "replay_remaining": len(replay_sweeps),
        },
    )
    _log_line(f"; full-auto log: {log_path}")
    _log_console(
        f"; full-auto stop: press Ctrl-C to accept best-so-far, "
        f"or create {stop_file} to request stop (cross-platform)"
    )

    base_solver = {
        "solve_restarts": int(solve_restarts),
        "solve_iterations": int(solve_iterations),
        "solve_optimizer": str(solve_optimizer),
        "residual_threshold": float(residual_threshold),
        "spring_k_multiplier": float(spring_k_multiplier),
        "use_flex": bool(use_flex),
        "pointwise_residual_mode": str(pointwise_residual_mode),
        "pointwise_filtering": bool(pointwise_filtering),
        "pointwise_global_mad": bool(pointwise_global_mad),
        "sweep_wise_filtering": bool(sweep_wise_filtering),
        "sweep_metric": str(sweep_metric),
        "use_noise_mean": bool(use_noise_mean),
        "sigma_source": str(sigma_source),
        "robust_debug": bool(robust_debug),
        "generate_report": bool(generate_report),
        "find_radii": str(find_radii_mode),
        "find_buildup_factor": str(find_buildup_mode),
        "base_radii": (None if base_radii is None else [float(v) for v in base_radii]),
        "buildup_factor": (None if buildup_factor is None else float(buildup_factor)),
        "r0_bounds": (None if r0_bounds is None else [float(r0_bounds[0]), float(r0_bounds[1])]),
        "b_bounds": (None if b_bounds is None else [float(b_bounds[0]), float(b_bounds[1])]),
        "r0_prior_sigma_mm": (
            None
            if r0_prior_sigma_mm is None
            else float(r0_prior_sigma_mm)
        ),
        "b_prior_sigma": (None if b_prior_sigma is None else float(b_prior_sigma)),
        "spool_outer_iters": int(spool_outer_iters),
        "spool_inner_iters": int(spool_inner_iters),
        "theta0_mode": str(_normalize_theta0_mode(theta0_mode)),
        "line_width": float(line_width),
    }

    best_cost = float("inf")
    best_plan: Optional[Dict[str, object]] = None
    best_meta: Dict[str, object] = {}
    no_improve = 0
    min_delta = float(DEFAULT_FULL_AUTO_MIN_DELTA)
    patience_limit = max(1, int(patience))
    has_variants = bool(full_auto_runs)
    selected_costs: List[float] = []

    def _accept_best(reason: str) -> int:
        if best_plan is None:
            _log_console(f"; full-auto: stop requested ({reason}) but no best plan available; stopping.")
            _log_console(_solution_quality_message(None))
            return _finalize(2)
        _log_console(f"; full-auto: stop requested ({reason}); accepting best-so-far.")
        return _emit_summary_and_send(best_plan)

    def _stop_file_requested() -> bool:
        try:
            return stop_file.exists()
        except OSError:
            return False

    try:
        for step in range(1, max(1, int(max_steps)) + 1):
            if _stop_file_requested():
                return _accept_best(f"stop-file {stop_file}")
            _log_line(f"\n; === full-auto iteration {step}/{max_steps} dataset={work_path} ===")
            _log_console("")
            _log_console(f"; === full-auto iteration {step}/{max_steps} dataset={work_path} ===")
            collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")
            run_results: List[Dict[str, object]] = []

            for run in runs:
                if _stop_file_requested():
                    return _accept_best(f"stop-file {stop_file}")
                run_id = str(run.get("id", "run"))
                run_flags = str(run.get("flags", "")).strip()
                overrides = run.get("overrides") or {}
                if not isinstance(overrides, dict):
                    overrides = {}
                settings = dict(base_solver)
                settings.update(overrides)
                if isinstance(settings.get("base_radii"), str):
                    settings["base_radii"] = _parse_csv_floats(str(settings["base_radii"]))
                settings["find_radii"] = _normalize_spool_find_mode(settings.get("find_radii"))
                settings["find_buildup_factor"] = _normalize_spool_find_mode(
                    settings.get("find_buildup_factor")
                )
                settings["theta0_mode"] = _normalize_theta0_mode(settings.get("theta0_mode"))

                raw_r0_bounds = settings.get("r0_bounds")
                if isinstance(raw_r0_bounds, str):
                    settings["r0_bounds"] = _parse_min_max_bounds(raw_r0_bounds, label="--r0-bounds")
                elif isinstance(raw_r0_bounds, (list, tuple)) and len(raw_r0_bounds) >= 2:
                    settings["r0_bounds"] = (float(raw_r0_bounds[0]), float(raw_r0_bounds[1]))
                else:
                    settings["r0_bounds"] = None

                raw_b_bounds = settings.get("b_bounds")
                if isinstance(raw_b_bounds, str):
                    settings["b_bounds"] = _parse_min_max_bounds(raw_b_bounds, label="--b-bounds")
                elif isinstance(raw_b_bounds, (list, tuple)) and len(raw_b_bounds) >= 2:
                    settings["b_bounds"] = (float(raw_b_bounds[0]), float(raw_b_bounds[1]))
                else:
                    settings["b_bounds"] = None

                if settings.get("r0_prior_sigma_mm") is not None:
                    settings["r0_prior_sigma_mm"] = float(settings["r0_prior_sigma_mm"])
                if settings.get("b_prior_sigma") is not None:
                    settings["b_prior_sigma"] = float(settings["b_prior_sigma"])
                settings["spool_outer_iters"] = int(settings.get("spool_outer_iters", 3))
                settings["spool_inner_iters"] = int(settings.get("spool_inner_iters", 30))
                settings["line_width"] = float(settings.get("line_width", DEFAULT_LAYER_LINE_WIDTH_MM))
                if not np.isfinite(settings["line_width"]) or settings["line_width"] < 0.0:
                    settings["line_width"] = float(DEFAULT_LAYER_LINE_WIDTH_MM)
                if run_flags:
                    _log_line(f"; full-auto run {run_id}: flags='{run_flags}'")
                else:
                    _log_line(f"; full-auto run {run_id}: flags=''")

                cfg_path = _full_auto_cfg_path(work_path, run_id)
                residuals_csv_run = None
                if residuals_csv is not None:
                    res_base = Path(residuals_csv)
                    if len(runs) > 1:
                        residuals_csv_run = res_base.with_name(
                            f"{res_base.stem}.{run_id}{res_base.suffix}"
                        )
                    else:
                        residuals_csv_run = res_base

                with _log_context():
                    plan = _plan_next_ellipse_sweep(
                        work_path,
                        solve_restarts=int(settings["solve_restarts"]),
                        solve_iterations=int(settings["solve_iterations"]),
                        solve_optimizer=str(settings["solve_optimizer"]),
                        residual_threshold=float(settings["residual_threshold"]),
                        spring_k_multiplier=float(settings["spring_k_multiplier"]),
                        use_flex=bool(settings["use_flex"]),
                        pointwise_residual_mode=str(settings["pointwise_residual_mode"]),
                        pointwise_filtering=bool(settings["pointwise_filtering"]),
                        pointwise_global_mad=bool(settings["pointwise_global_mad"]),
                        sweep_wise_filtering=bool(settings["sweep_wise_filtering"]),
                        sweep_metric=str(settings["sweep_metric"]),
                        use_noise_mean=bool(settings["use_noise_mean"]),
                        sigma_source=str(settings["sigma_source"]),
                        robust_debug=bool(settings["robust_debug"]),
                        residuals_csv=residuals_csv_run,
                        generate_report=bool(settings["generate_report"]),
                        find_radii=str(settings.get("find_radii", "off")),
                        find_buildup_factor=str(settings.get("find_buildup_factor", "off")),
                        base_radii=(
                            [float(v) for v in settings["base_radii"]]
                            if isinstance(settings.get("base_radii"), list)
                            else None
                        ),
                        buildup_factor=(
                            float(settings["buildup_factor"])
                            if settings.get("buildup_factor") is not None
                            else None
                        ),
                        r0_bounds=settings.get("r0_bounds"),
                        b_bounds=settings.get("b_bounds"),
                        r0_prior_sigma_mm=(
                            float(settings["r0_prior_sigma_mm"])
                            if settings.get("r0_prior_sigma_mm") is not None
                            else None
                        ),
                        b_prior_sigma=(
                            float(settings["b_prior_sigma"])
                            if settings.get("b_prior_sigma") is not None
                            else None
                        ),
                        spool_outer_iters=int(settings.get("spool_outer_iters", 3)),
                        spool_inner_iters=int(settings.get("spool_inner_iters", 30)),
                        theta0_mode=str(settings.get("theta0_mode", "zero")),
                        line_width=float(settings.get("line_width", DEFAULT_LAYER_LINE_WIDTH_MM)),
                        candidate_deltas=candidate_deltas,
                        candidate_count=int(candidate_count),
                        delta_min=delta_min,
                        delta_max=delta_max,
                        fd_eps_mm=float(fd_eps_mm),
                        regularization=float(regularization),
                        exclude_existing=bool(exclude_existing),
                        existing_tol_mm=float(existing_tol_mm),
                        min_fixed_delta_spacing_mm=float(min_fixed_delta_spacing_mm),
                        top_k=int(top_k),
                        write_cfg=cfg_path,
                        collector_output=collector_output,
                        collector_args=collector_args_eff,
                    )

                primary_cost = _plan_primary_cost(plan)
                max_std_mm, rel_std, cov_ok = _plan_covariance_summary(plan)
                warnings = _plan_data_quality_warnings(plan)
                noise_metrics = _plan_noise_metrics(plan)
                underconstrained_penalty = _plan_hits_underconstrained_penalty(plan, primary_cost)
                if underconstrained_penalty:
                    warnings.append("underconstrained_penalty")
                valid = bool(np.isfinite(primary_cost) and cov_ok)
                cost_raw = plan.get("cost_raw")
                cost_norm = plan.get("cost_noise_normalized", plan.get("cost"))
                j_val = noise_metrics.get("J") if isinstance(noise_metrics, dict) else None
                chi2_val = noise_metrics.get("chi2_red") if isinstance(noise_metrics, dict) else None
                _log_line(
                    f"; full-auto run {run_id}: cost_raw={_fmt_float(cost_raw)} "
                    f"cost_noise_normalized={_fmt_float(cost_norm)} J={_fmt_float(j_val)} "
                    f"chi2_red={_fmt_float(chi2_val)}"
                )

                run_results.append(
                    {
                        "id": run_id,
                        "flags": run.get("flags", ""),
                        "overrides": overrides,
                        "settings": settings,
                        "plan": plan,
                        "metrics": {
                            "primary_cost": primary_cost,
                            "cost_noise_normalized": plan.get("cost_noise_normalized"),
                            "chi2_red": noise_metrics.get("chi2_red")
                            if isinstance(noise_metrics, dict)
                            else None,
                            "J": noise_metrics.get("J") if isinstance(noise_metrics, dict) else None,
                            "info_rank": plan.get("info_rank"),
                            "info_rank_deficient": plan.get("info_rank_deficient"),
                            "max_std_mm": max_std_mm,
                            "rel_std": rel_std,
                            "covariance_ok": cov_ok,
                            "underconstrained_penalty": underconstrained_penalty,
                            "warnings": warnings,
                            "valid": valid,
                        },
                    }
                )

            valid_runs = [r for r in run_results if r["metrics"]["valid"]]
            if not valid_runs:
                _append_jsonl(
                    log_path,
                    {
                        "timestamp": datetime.now().isoformat(),
                        "event": "stop",
                        "iteration": step,
                        "dataset": str(dataset_path),
                        "reason": "no_valid_runs",
                        "runs": [
                            {
                                "id": r["id"],
                                "flags": r["flags"],
                                "metrics": r["metrics"],
                            }
                            for r in run_results
                        ],
                    },
                )
                _log_line("; full-auto: no valid calibration runs (non-finite cost or covariance).")
                _log_console("; full-auto: no valid calibration runs (non-finite cost or covariance).")
                _log_console(_solution_quality_message(best_cost if np.isfinite(best_cost) else None))
                return _finalize(2)

            def _sort_key(entry: Dict[str, object]) -> Tuple[float, float, float, str]:
                metrics = entry["metrics"]
                underconstrained = bool(metrics.get("underconstrained_penalty", False))
                cost = float(metrics.get("primary_cost", float("inf")))
                rel = metrics.get("rel_std")
                rel_val = (
                    float(rel)
                    if isinstance(rel, (int, float)) and np.isfinite(rel)
                    else float("inf")
                )
                return (1.0 if underconstrained else 0.0), cost, rel_val, str(entry.get("id", ""))

            selected = sorted(valid_runs, key=_sort_key)[0]
            plan = selected["plan"]
            metrics = selected["metrics"]
            selected_id = str(selected.get("id", "run"))
            selected_flags = str(selected.get("flags", "")).strip()
            selected_cost = float(metrics.get("primary_cost", float("nan")))
            selected_underconstrained = bool(metrics.get("underconstrained_penalty", False))
            selected_max_std = metrics.get("max_std_mm")
            selected_rel_std = metrics.get("rel_std")
            selected_warnings = list(metrics.get("warnings") or [])

            with _log_context():
                _print_ellipse_plan(
                    plan,
                    top_n=5,
                    print_command=True,
                    output_with_explanations=bool(output_with_explanations),
                )

            if write_cfg is not None and plan.get("best_cfg") is not None:
                try:
                    _write_sweep_config_file(Path(write_cfg), plan["best_cfg"])
                except Exception as exc:
                    _log_line(f"; full-auto warning: failed to write sweep config {write_cfg}: {exc}")

            improvement = None
            if np.isfinite(selected_cost):
                improvement = float(best_cost) - float(selected_cost) if np.isfinite(best_cost) else None
            improved = False
            if (not selected_underconstrained) and np.isfinite(selected_cost) and (
                best_plan is None or selected_cost <= best_cost - min_delta
            ):
                best_cost = float(selected_cost)
                best_plan = plan
                best_meta = {
                    "iteration": step,
                    "run_id": selected_id,
                    "flags": selected_flags,
                    "cost": selected_cost,
                    "rel_std": selected_rel_std,
                    "max_std_mm": selected_max_std,
                }
                no_improve = 0
                improved = True
            elif not selected_underconstrained:
                no_improve += 1

            stop_cost_hit = False
            stop_std_hit = False
            if best_plan is not None:
                if stop_cost is not None and np.isfinite(best_cost) and best_cost <= float(stop_cost):
                    stop_cost_hit = True
                if stop_std_mm is not None and isinstance(best_meta.get("max_std_mm"), (int, float)):
                    stop_std_hit = bool(
                        np.isfinite(float(best_meta["max_std_mm"]))
                        and float(best_meta["max_std_mm"]) <= float(stop_std_mm)
                    )

            selected_costs.append(selected_cost)
            summary_flags = f" flags='{selected_flags}'" if selected_flags else ""
            summary_rel = _fmt_float(selected_rel_std)
            summary_std = _fmt_float(selected_max_std, suffix="mm")
            summary_cost = _fmt_float(selected_cost)
            _log_line(
                f"; selected run={selected_id}{summary_flags} cost={summary_cost} "
                f"rel_std={summary_rel} max_std={summary_std}"
            )
            if has_variants:
                _log_console(f"; selected run={selected_id}{summary_flags}")
            _log_console(f"cost: {summary_cost}")
            if selected_underconstrained:
                _log_console("; selected run hit underconstrained sentinel; continuing to collect more sweeps.")

            threshold_accept = False
            if stop_cost is not None and stop_cost_hit:
                threshold_accept = True
            if stop_std_mm is not None and stop_std_hit:
                threshold_accept = True
            if selected_underconstrained:
                decision = "collect"
            elif threshold_accept or no_improve >= patience_limit:
                decision = "accept"
            else:
                decision = "collect"

            _append_jsonl(
                log_path,
                {
                    "timestamp": datetime.now().isoformat(),
                    "iteration": step,
                    "dataset": str(dataset_path),
                    "runs": [
                        {
                            "id": r["id"],
                            "flags": r["flags"],
                            "settings": r["settings"],
                            "metrics": r["metrics"],
                        }
                        for r in run_results
                    ],
                    "selected_run": selected_id,
                    "decision": decision,
                    "cost": selected_cost,
                    "cost_improvement": improvement,
                    "improved_best": improved,
                    "best_cost": best_cost,
                    "best_run": best_meta.get("run_id"),
                    "best_iteration": best_meta.get("iteration"),
                    "no_improve_count": no_improve,
                    "patience": patience_limit,
                    "min_delta": min_delta,
                    "stop_cost_hit": stop_cost_hit,
                    "stop_std_hit": stop_std_hit,
                    "warnings": selected_warnings,
                },
            )

            if decision == "accept":
                if best_plan is None:
                    _log_console("; full-auto: no best plan available; stopping.")
                    _log_console(_solution_quality_message(None))
                    return _finalize(2)
                return _emit_summary_and_send(best_plan)

            if replay_mode:
                if replay_index >= len(replay_sweeps):
                    _log_console(
                        "; full-auto replay: no more sweeps to replay; switching to live collection."
                    )
                    _log_line(
                        "; full-auto replay: no more sweeps to replay; switching to live collection."
                    )
                    replay_mode = False
                    if work_path != dataset_path:
                        work_path = dataset_path

                if replay_mode:
                    next_sweep = replay_sweeps[replay_index]
                    replay_index += 1
                    base_dataset = _load_json(work_path)
                    new_dataset = dict(base_dataset)
                    new_dataset["timestamp"] = datetime.now().isoformat()
                    new_dataset["sweeps"] = [next_sweep]
                    merged = _merge_sweep_datasets(base_dataset, new_dataset)
                    _write_json(work_path, merged)
                    sweeps = merged.get("sweeps", [])
                    count = len(sweeps) if isinstance(sweeps, list) else "?"
                    _log_line(
                        f"; replayed sweep {replay_index}/{len(replay_sweeps)} "
                        f"-> {work_path} sweeps={count}"
                    )
                    _log_console(f"Collected sweep nr {count}")
                    continue

            cmd = plan.get("collect_command")
            if not isinstance(cmd, list) or not cmd:
                _log_line("; No valid candidate to collect; stopping.")
                _log_console("; No valid candidate to collect; stopping.")
                _log_console(_solution_quality_message(best_cost if np.isfinite(best_cost) else None))
                return _finalize(2)

            finite_costs = [c for c in selected_costs if np.isfinite(c)]
            if not np.isfinite(selected_cost) or not finite_costs:
                cost_rank = 1
            else:
                cost_rank = sorted(finite_costs).index(selected_cost) + 1
            rank_label = "best" if cost_rank == 1 else _ordinal(cost_rank) + " best"
            remaining = max(0, patience_limit - no_improve)
            _log_console(f"The {rank_label} try so far.")
            _log_console("Collecting new sweep to try and beat it.")
            _log_console(f"Still has patience for {remaining} more attempts.")
            _log_line(f"; collecting next sweep ({selected_id})")
            _log_line(f"; running: {' '.join(str(x) for x in cmd)}")
            with _log_context():
                subprocess.run(cmd, check=True, stdout=log_handle, stderr=log_handle)
            reset_pending = False

            base_dataset = _load_json(work_path)
            new_dataset = _load_json(collector_output)
            if plan.get("force_args_applied") and isinstance(plan.get("force_tuning"), dict):
                cfg = new_dataset.get("config")
                if isinstance(cfg, dict):
                    cfg["force_tuning"] = dict(plan["force_tuning"])
                    _write_json(collector_output, new_dataset)

            merged = _merge_sweep_datasets(base_dataset, new_dataset)
            _write_json(work_path, merged)
            sweeps = merged.get("sweeps", [])
            count = len(sweeps) if isinstance(sweeps, list) else "?"
            _log_line(f"; merged {collector_output} -> {work_path} sweeps={count}")
            _log_console(f"Collected sweep nr {count}")
            try:
                if collector_output != work_path:
                    collector_output.unlink()
            except OSError:
                pass
    except KeyboardInterrupt:
        return _accept_best("Ctrl-C")

    _log_line(f"; reached max steps; dataset={work_path}")
    _log_console(f"; reached max steps; dataset={work_path}")
    _log_console(_solution_quality_message(best_cost if np.isfinite(best_cost) else None))
    return _finalize(0)


def ellipse_loop(
    *,
    work_dataset: Optional[Path],
    machine_type: str,
    max_steps: int,
    stop_cost: Optional[float],
    stop_std_mm: Optional[float],
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    robust_debug: bool,
    residuals_csv: Optional[Path],
    generate_report: bool,
    find_radii: str,
    find_buildup_factor: str,
    base_radii: Optional[List[float]],
    buildup_factor: Optional[float],
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    line_width: float,
    candidate_deltas: Optional[List[float]],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    fd_eps_mm: float,
    regularization: float,
    exclude_existing: bool,
    existing_tol_mm: float,
    min_fixed_delta_spacing_mm: float,
    top_k: int,
    write_cfg: Optional[Path],
    collector_args: Sequence[str],
    sim: bool,
    keep_sim_alive: bool,
    hp_sim_reset: bool,
    plot_residual_histogram: bool,
    sweep_points: Optional[int],
    output_with_explanations: bool,
) -> int:
    if work_dataset is not None:
        work_path = Path(work_dataset)
    else:
        work_path = Path("autocal/data/default_dataset.json")
    if work_path.exists():
        machine_type = _require_machine_type(
            _load_json(work_path),
            expected=machine_type,
            context=str(work_path),
            mismatch="warn",
        )

    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)
    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    hp_sim_reset_eff = _effective_hp_sim_reset(
        sim=bool(sim),
        hp_sim_reset=bool(hp_sim_reset),
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    if bool(sim) and hp_sim_reset_eff and not bool(hp_sim_reset):
        print("; auto-enabling --hp-sim-reset for spool search in simulation")
    sweep_points_value = None
    raw_sweep_points = _arg_value(collector_args_eff, "--sweepPoints", "--sweep-points")
    if raw_sweep_points is not None:
        try:
            parsed = int(raw_sweep_points)
        except ValueError:
            parsed = None
        if parsed is not None and parsed > 0:
            sweep_points_value = parsed
    if sweep_points is not None:
        try:
            parsed = int(sweep_points)
        except (TypeError, ValueError):
            parsed = None
        if parsed is not None and parsed > 0:
            sweep_points_value = parsed
            if raw_sweep_points is None:
                collector_args_eff.extend(["--sweepPoints", str(parsed)])
    reset_pending = bool(sim and hp_sim_reset_eff)
    rrf_server, server_explicit, port = _resolve_rrf_target(collector_args)
    sim_process: Optional[subprocess.Popen] = None
    sim_config = _resolve_sim_config(
        machine_type=machine_type,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )

    if sim and not server_explicit and not user_no_spawn:
        target_port = port or DEFAULT_RRF_PORT
        print(f"; starting rrf_simulator at http://localhost:{target_port} (config: {sim_config})")
        sim_process = _start_rrf_simulator(target_port, sim_config=sim_config)
        _wait_for_rrf_server(f"http://localhost:{target_port}")

    def _finalize(code: int) -> int:
        if sim_process and not keep_sim_alive:
            _stop_process(sim_process)
        return code

    if not work_path.exists():
        work_path.parent.mkdir(parents=True, exist_ok=True)
        bootstrap_cfg = work_path.with_suffix(".bootstrap_cfg.txt")
        bootstrap_cfg.parent.mkdir(parents=True, exist_ok=True)
        bootstrap_cfg.write_text(
            "[0] 1 2\n[1] 0 2\n[2] 0 1\n",
            encoding="utf-8",
        )

        def _strip_conflicts(argv: Sequence[str]) -> List[str]:
            skip_with_value = {
                "--output-file",
                "--output",
                "--out",
                "--observability-file",
                "--obs-file",
                "--machineType",
                "--machine-type",
                "--sweepMethod",
                "--sweep-method",
                "--sweep-config-file",
                "--sweep-config",
                "--sweepFile",
                "--fixed-targets",
                "--fixedTargets",
                "--fixed-target",
                "--max-travel-mm",
                "--max-travel",
            }
            out: List[str] = []
            i = 0
            while i < len(argv):
                arg = str(argv[i])
                if arg in skip_with_value:
                    i += 2
                    continue
                out.append(arg)
                i += 1
            return out

        argv_eff = _strip_conflicts(list(collector_args_eff))
        if reset_pending and not _arg_has_flag(argv_eff, "--hp-sim-reset"):
            argv_eff.append("--hp-sim-reset")
        if "--return-to-origin" not in argv_eff and "--returnToOrigin" not in argv_eff:
            argv_eff.append("--return-to-origin")

        cmd = [
            "node",
            "autocal/control/cli/collect_sweep_data.mjs",
            "--machineType",
            str(machine_type),
            "--sweep-config-file",
            str(bootstrap_cfg),
            "--output-file",
            str(work_path),
            *argv_eff,
        ]
        print("; bootstrapping dataset (3 sweeps, auto size-tune):")
        print(";   " + " ".join(cmd))
        subprocess.run(cmd, check=True)
        reset_pending = False
        print(f"; bootstrap dataset written to {work_path}")

    base_residuals_csv = residuals_csv
    if plot_residual_histogram:
        base_residuals_csv = work_path.with_suffix(".csv")
    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)

    for step in range(1, max(1, int(max_steps)) + 1):
        print(f"\n; === iteration {step}/{max_steps} dataset={work_path} ===")
        collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")
        step_residuals_csv = None
        if base_residuals_csv is not None:
            if plot_residual_histogram:
                step_residuals_csv = base_residuals_csv
            else:
                stem = base_residuals_csv.stem
                suffix = base_residuals_csv.suffix or ".csv"
                step_residuals_csv = base_residuals_csv.with_name(f"{stem}_{step:03d}{suffix}")

        plan_args = list(collector_args_eff)
        if reset_pending and not _arg_has_flag(plan_args, "--hp-sim-reset"):
            plan_args.append("--hp-sim-reset")

        plan = _plan_next_ellipse_sweep(
            work_path,
            solve_restarts=solve_restarts,
            solve_iterations=solve_iterations,
            solve_optimizer=solve_optimizer,
            residual_threshold=residual_threshold,
            spring_k_multiplier=spring_k_multiplier,
            use_flex=use_flex,
            pointwise_residual_mode=pointwise_residual_mode,
            pointwise_filtering=pointwise_filtering,
            pointwise_global_mad=pointwise_global_mad,
            sweep_wise_filtering=sweep_wise_filtering,
            sweep_metric=sweep_metric,
            use_noise_mean=use_noise_mean,
            sigma_source=sigma_source,
            robust_debug=robust_debug,
            residuals_csv=step_residuals_csv,
            generate_report=generate_report,
            find_radii=str(find_radii_mode),
            find_buildup_factor=str(find_buildup_mode),
            base_radii=base_radii,
            buildup_factor=buildup_factor,
            r0_bounds=r0_bounds,
            b_bounds=b_bounds,
            r0_prior_sigma_mm=r0_prior_sigma_mm,
            b_prior_sigma=b_prior_sigma,
            spool_outer_iters=int(spool_outer_iters),
            spool_inner_iters=int(spool_inner_iters),
            theta0_mode=str(theta0_mode),
            line_width=float(line_width),
            candidate_deltas=candidate_deltas,
            candidate_count=candidate_count,
            delta_min=delta_min,
            delta_max=delta_max,
            fd_eps_mm=fd_eps_mm,
            regularization=regularization,
            exclude_existing=exclude_existing,
            existing_tol_mm=existing_tol_mm,
            min_fixed_delta_spacing_mm=min_fixed_delta_spacing_mm,
            top_k=top_k,
            write_cfg=write_cfg,
            collector_output=collector_output,
            collector_args=plan_args,
        )
        _print_ellipse_plan(
            plan,
            top_n=5,
            print_command=True,
            output_with_explanations=bool(output_with_explanations),
        )

        if plot_residual_histogram and step_residuals_csv is not None:
            plot_output = _unique_path(work_path.with_suffix(".png"))
            plot_script = REPO_ROOT / "autocal" / "plot_residual_hist.py"
            if step_residuals_csv.exists() and plot_script.exists():
                cmd = [
                    sys.executable,
                    str(plot_script),
                    str(step_residuals_csv),
                    "--output",
                    str(plot_output),
                ]
                if sweep_points_value is not None:
                    cmd.extend(["--sweep-points", str(int(sweep_points_value))])
                print("; plotting residual histogram:")
                print(";   " + " ".join(cmd))
                subprocess.run(cmd, check=True)
                print(f"; histogram saved to {plot_output}")
            else:
                print("; residual histogram skipped (missing CSV or plotter)")

        cost = float(plan.get("cost", float("nan")))
        cov = np.asarray(plan.get("covariance_scaled", plan.get("covariance")), dtype=float)
        max_std = float("nan")
        if cov.ndim == 2 and cov.shape[0] == cov.shape[1] and cov.size:
            diag = np.diag(cov)
            if diag.size and np.any(np.isfinite(diag)):
                max_std = float(np.max(np.sqrt(np.maximum(diag, 0.0))))

        if stop_cost is not None and np.isfinite(cost) and cost <= float(stop_cost):
            print(f"; stop condition: cost <= {float(stop_cost):.6g}")
        if stop_std_mm is not None and np.isfinite(max_std) and max_std <= float(stop_std_mm):
            print(f"; stop condition: max_std <= {float(stop_std_mm):.6g}mm")

        cmd = plan.get("collect_command")
        if not isinstance(cmd, list) or not cmd:
            print("; No valid candidate to collect; stopping.")
            return _finalize(2)

        try:
            while True:
                resp = input("Accept anchors [a], collect next sweep [c], quit [q]? ").strip().lower()
                if resp in ("a", "accept", "ok", "y", "yes"):
                    m669 = _m669_from_plan(plan)
                    if m669:
                        print(f"; sending {m669} to {rrf_server}")
                        try:
                            reply = _send_rrf_gcode(rrf_server, m669)
                        except Exception as exc:
                            print(f"; failed to send M669: {exc}")
                            return _finalize(1)
                        reply = reply.strip()
                        if reply:
                            print(f"; M669 reply: {reply}")
                    else:
                        print("; accepted anchors; no M669 command available")
                    print(f"; accepted anchors; dataset={work_path}")
                    return _finalize(0)
                if resp in ("q", "quit", "exit", "n", "no"):
                    print(f"; stopped; dataset={work_path}")
                    return _finalize(0)
                if resp in ("c", "collect", "next", ""):
                    break
                print("Please enter a/c/q.")
        except KeyboardInterrupt:
            print(f"\n; interrupted; dataset={work_path}")
            return _finalize(130)

        base_dataset = _load_json(work_path)
        print(f"; running: {' '.join(str(x) for x in cmd)}")
        subprocess.run(cmd, check=True)
        reset_pending = False

        new_dataset = _load_json(collector_output)
        if plan.get("force_args_applied") and isinstance(plan.get("force_tuning"), dict):
            cfg = new_dataset.get("config")
            if isinstance(cfg, dict):
                cfg["force_tuning"] = dict(plan["force_tuning"])
                _write_json(collector_output, new_dataset)

        merged = _merge_sweep_datasets(base_dataset, new_dataset)
        _write_json(work_path, merged)
        sweeps = merged.get("sweeps", [])
        count = len(sweeps) if isinstance(sweeps, list) else "?"
        print(f"; merged {collector_output} -> {work_path} sweeps={count}")
        try:
            if collector_output != work_path:
                collector_output.unlink()
        except OSError:
            pass

    print(f"; reached max steps; dataset={work_path}")
    return _finalize(0)


def _add_solver_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--solve-restarts", type=int, default=4)
    parser.add_argument("--solve-iterations", type=int, default=400)
    parser.add_argument("--solve-optimizer", default="L-BFGS-B")
    parser.add_argument("--threshold", type=float, default=250.0)
    parser.add_argument("--spring-k-multiplier", type=float, default=1.0)
    parser.add_argument("--flex", action="store_true")
    parser.add_argument(
        "--find-radii",
        choices=_SPOOL_FIND_MODE_CHOICES,
        nargs="?",
        const="per-anchor",
        default="off",
        help="Spool-radius fit mode: off | global | per-anchor (default: off).",
    )
    parser.add_argument(
        "--find-buildup-factor",
        choices=_SPOOL_FIND_MODE_CHOICES,
        nargs="?",
        const="per-anchor",
        default="off",
        help="Buildup-factor fit mode: off | global | per-anchor (default: off).",
    )
    parser.add_argument(
        "--base-radii",
        type=str,
        default=None,
        help="Comma-separated base radii in mm (defaults to config m666 R).",
    )
    parser.add_argument(
        "--buildup-factor",
        type=float,
        default=None,
        help="Use this buildup factor k in the model transform (with --find-radii, default is 0).",
    )
    parser.add_argument(
        "--line-width",
        type=float,
        default=DEFAULT_LAYER_LINE_WIDTH_MM,
        help="Estimated line width in mm used by the layered noise model (default: 1.0).",
    )
    parser.add_argument(
        "--r0-bounds",
        type=str,
        default=None,
        help="Bounds for fitted r0 in mm as 'min,max'.",
    )
    parser.add_argument(
        "--b-bounds",
        type=str,
        default=None,
        help="Bounds for fitted buildup factor as 'min,max' (default: 0,1).",
    )
    parser.add_argument(
        "--r0-prior-sigma-mm",
        type=float,
        default=2.0,
        help="Gaussian prior sigma (mm) around base radii during radius fitting.",
    )
    parser.add_argument(
        "--b-prior-sigma",
        type=float,
        default=None,
        help="Gaussian prior sigma for buildup-factor fitting (auto default: 0.1 when fitting k).",
    )
    parser.add_argument(
        "--spool-outer-iters",
        type=int,
        default=3,
        help="Outer iterations for spool parameter refinement.",
    )
    parser.add_argument(
        "--spool-inner-iters",
        type=int,
        default=30,
        help="Inner optimizer iterations per spool refinement step.",
    )
    parser.add_argument(
        "--theta0-mode",
        choices=_THETA0_MODE_CHOICES,
        default="zero",
        help="Theta offset mode for spool model: infer | zero (default: zero).",
    )
    parser.add_argument(
        "--pointwise-residual",
        choices=["sampson", "euclidean"],
        default="sampson",
        help="Pointwise residual metric (default: sampson)",
    )
    parser.add_argument(
        "--pointwise-filtering",
        dest="pointwise_filtering",
        action="store_true",
        help="Enable GNC-IRLS style pointwise filtering (default).",
    )
    parser.add_argument(
        "--no-pointwise-filtering",
        dest="pointwise_filtering",
        action="store_false",
        help="Disable pointwise filtering.",
    )
    parser.add_argument(
        "--pointwise-global-mad",
        dest="pointwise_global_mad",
        action="store_true",
        help="Use a single global MAD scale for pointwise filtering (default).",
    )
    parser.add_argument(
        "--pointwise-per-sweep-mad",
        dest="pointwise_global_mad",
        action="store_false",
        help="Use a per-sweep MAD scale for pointwise filtering.",
    )
    parser.add_argument(
        "--sweep-wise-filtering",
        dest="sweep_wise_filtering",
        action="store_true",
        help="Enable sweep-wise outlier rejection (default).",
    )
    parser.add_argument(
        "--no-sweep-wise-filtering",
        dest="sweep_wise_filtering",
        action="store_false",
        help="Disable sweep-wise outlier rejection.",
    )
    parser.add_argument(
        "--sweep-metric",
        choices=["mad", "median_abs", "outlier_ratio"],
        default="outlier_ratio",
        help="Per-sweep metric used by sweep-wise filtering (default: outlier_ratio).",
    )
    parser.add_argument(
        "--sigma-source",
        choices=["auto", "point", "origin", "min"],
        default="auto",
        help="Sigma source for normalization: per-point, origin, or fallback min (default: auto).",
    )
    lengths_group = parser.add_mutually_exclusive_group()
    lengths_group.add_argument(
        "--use-noise-mean",
        dest="use_noise_mean",
        action="store_true",
        help="Use encoder noise mean lengths when available (default).",
    )
    lengths_group.add_argument(
        "--use-raw-lengths",
        dest="use_noise_mean",
        action="store_false",
        help="Use raw l_drive/l_sensor lengths (disable noise-mean datapoints).",
    )
    parser.add_argument(
        "--robust-debug",
        dest="robust_debug",
        action="store_true",
        help="Print diagnostics for robustness filtering (default).",
    )
    parser.add_argument(
        "--no-robust-debug",
        dest="robust_debug",
        action="store_false",
        help="Disable robustness filtering diagnostics.",
    )
    parser.add_argument(
        "--residuals-csv",
        type=Path,
        default=None,
        help="Write pointwise residuals (approx mm) to CSV after the final GNC stage.",
    )
    parser.add_argument("--report", action="store_true", help="Write a PNG report (like calibrate.py)")
    parser.set_defaults(
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        robust_debug=True,
        use_noise_mean=True,
    )


def _add_candidate_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--candidate-deltas", type=str, default=None, help="Comma-separated fixed deltas (mm)")
    parser.add_argument("--candidate-count", type=int, default=41, help="Grid size when deltas not provided")
    parser.add_argument("--delta-min", type=float, default=None)
    parser.add_argument("--delta-max", type=float, default=None)
    parser.add_argument("--fd-eps-mm", type=float, default=1.0)
    parser.add_argument("--regularization", type=float, default=1e-6)
    parser.add_argument("--no-exclude-existing", action="store_true")
    parser.add_argument(
        "--existing-tol-mm",
        type=float,
        default=10.0,
        help="Treat sweeps with the same fixed anchors/drive/sensor and fixed delta within this tolerance as duplicates",
    )
    parser.add_argument(
        "--min-fixed-delta-spacing-mm",
        type=float,
        default=20.0,
        help="Minimum spacing (mm) from previously collected fixed deltas with the same sign",
    )
    parser.add_argument("--top-k", type=int, default=10)


def _add_collector_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--collector-args",
        nargs=argparse.REMAINDER,
        default=(),
        help="Extra args passed to autocal/control/cli/collect_sweep_data.mjs (after --collector-args)",
    )
    parser.add_argument(
        "--sim",
        "--simulation",
        action="store_true",
        help="Use rrf_simulator + hp-sim WebSocket bridge (default: talk to real firmware)",
    )
    parser.add_argument(
        "--keep-sim-alive",
        action="store_true",
        help="Leave rrf_simulator running after exit (only when --sim)",
    )
    parser.add_argument(
        "--hp-sim-reset",
        action="store_true",
        help="Reset hp-sim once before the first sweep (only when --sim)",
    )
    parser.add_argument(
        "--project-zero-tension",
        action="store_true",
        help="Project encoder readings to zero tension during each data point.",
    )
    parser.add_argument(
        "--debug-sweep-actions",
        action="store_true",
        help="Enable sweep collector debug traces (force modes, waits, moves, data points).",
    )


def _add_output_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--output-with-explanations",
        action="store_true",
        help="Include explanatory notes for rank deficiency and covariance behavior.",
    )


def build_ellipse_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Active sweep selection for ellipse calibration")
    parser.add_argument("dataset", type=Path, help="Existing sweep dataset JSON")
    parser.add_argument(
        "--machine-type",
        choices=MACHINE_TYPE_CHOICES,
        required=True,
        help=f"Machine type ({MACHINE_TYPE_CHOICES_STR})",
    )
    _add_solver_args(parser)
    _add_candidate_args(parser)
    _add_output_args(parser)
    parser.add_argument(
        "--write-sweep-config",
        type=Path,
        default=None,
        help="Write the suggested sweep config to this path (default: <dataset>.active_sweep_cfg.txt)",
    )
    parser.add_argument(
        "--no-print-command",
        action="store_true",
        help="Suppress printing the suggested collector command.",
    )
    _add_collector_args(parser)
    parser.add_argument(
        "--collect-once",
        action="store_true",
        help="Collect the suggested sweep and merge (requires --collector-output)",
    )
    parser.add_argument(
        "--collector-output",
        type=Path,
        default=None,
        help="Output JSON for collector (required with --collect-once)",
    )
    parser.add_argument(
        "--merged-output-dataset",
        type=Path,
        default=None,
        help="Where to write the merged dataset (default: overwrite input dataset)",
    )
    return parser


def build_merge_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Merge one or more sweep datasets into a base dataset")
    parser.add_argument("base", type=Path, help="Base dataset JSON (keeps config metadata)")
    parser.add_argument("extra", type=Path, nargs="+", help="Additional dataset JSON files to append")
    parser.add_argument("-o", "--output", type=Path, default=None, help="Output dataset path (default: overwrite base)")
    return parser


def build_semi_auto_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Interactive active-learning loop (semi-auto by default)."
    )
    parser.add_argument(
        "--machine-type",
        choices=MACHINE_TYPE_CHOICES,
        required=True,
        help=f"Machine type ({MACHINE_TYPE_CHOICES_STR})",
    )
    parser.add_argument(
        "--semi-auto",
        action="store_true",
        help="Explicitly run the interactive semi-auto loop (default).",
    )
    parser.add_argument(
        "--full-auto",
        action="store_true",
        help="Run the non-interactive full-auto loop.",
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=None,
        help="Dataset file updated each iteration (default: autocal/data/default_dataset.json).",
    )
    parser.add_argument(
        "--sweep-points",
        type=int,
        default=None,
        help="Points per sub-sweep (passed to the collector and histogram plot).",
    )
    parser.add_argument(
        "--plot-residual-histogram",
        action="store_true",
        help="Write residuals CSV next to the dataset and render a histogram PNG (gamma fit included).",
    )
    parser.add_argument("--max-steps", type=int, default=20, help="Maximum active-learning iterations")
    parser.add_argument("--stop-cost", type=float, default=None, help="Optional stop condition on ellipse cost")
    parser.add_argument("--stop-std-mm", type=float, default=None, help="Optional stop condition on max parameter std")
    parser.add_argument(
        "--full-auto-run",
        action="append",
        default=None,
        help="Full-auto run override flags (repeatable). Example: --full-auto-run \"--sweep-metric mad --use-raw-lengths\"",
    )
    parser.add_argument(
        "--shotgun",
        action="store_true",
        help="Append full-auto runs from shotgun.conf (use with --full-auto).",
    )
    parser.add_argument(
        "--full-auto-log",
        type=Path,
        default=None,
        help="Write full-auto JSONL logs to this path (default: <dataset>.full_auto_log.jsonl).",
    )
    parser.add_argument(
        "--full-auto-verbose",
        action="store_true",
        help="Enable verbose solver diagnostics during full-auto runs.",
    )
    parser.add_argument(
        "--patience",
        type=int,
        default=3,
        help="Number of non-improving full-auto iterations to allow (default: 3).",
    )
    _add_solver_args(parser)
    _add_candidate_args(parser)
    _add_output_args(parser)
    parser.add_argument(
        "--write-sweep-config",
        type=Path,
        default=None,
        help="Write the suggested sweep config to this path (default: <dataset>.active_sweep_cfg.txt)",
    )
    _add_collector_args(parser)
    return parser


def _clean_collector_args(raw_args: Sequence[str]) -> List[str]:
    collector_args = list(raw_args)
    if collector_args and collector_args[0] == "--":
        collector_args = collector_args[1:]
    return collector_args


def _resolve_spool_cli_options(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
) -> Dict[str, object]:
    try:
        find_radii_mode = _normalize_spool_find_mode(args.find_radii)
        find_buildup_mode = _normalize_spool_find_mode(args.find_buildup_factor)
        theta0_mode = _normalize_theta0_mode(args.theta0_mode)
        r0_bounds = _parse_min_max_bounds(args.r0_bounds, label="--r0-bounds")
        b_bounds = _parse_min_max_bounds(args.b_bounds, label="--b-bounds")
    except ValueError as exc:
        parser.error(str(exc))

    r0_prior_sigma_mm = args.r0_prior_sigma_mm
    if r0_prior_sigma_mm is not None:
        try:
            r0_prior_sigma_mm = float(r0_prior_sigma_mm)
        except (TypeError, ValueError):
            parser.error("--r0-prior-sigma-mm must be numeric")
        if not np.isfinite(r0_prior_sigma_mm) or r0_prior_sigma_mm <= 0.0:
            parser.error("--r0-prior-sigma-mm must be finite and > 0")

    b_prior_sigma = args.b_prior_sigma
    if b_prior_sigma is not None:
        try:
            b_prior_sigma = float(b_prior_sigma)
        except (TypeError, ValueError):
            parser.error("--b-prior-sigma must be numeric")
        if not np.isfinite(b_prior_sigma) or b_prior_sigma <= 0.0:
            parser.error("--b-prior-sigma must be finite and > 0")

    spool_outer_iters = int(args.spool_outer_iters)
    spool_inner_iters = int(args.spool_inner_iters)
    if spool_outer_iters < 1:
        parser.error("--spool-outer-iters must be >= 1")
    if spool_inner_iters < 1:
        parser.error("--spool-inner-iters must be >= 1")

    line_width = args.line_width
    try:
        line_width = float(line_width)
    except (TypeError, ValueError):
        parser.error("--line-width must be numeric")
    if not np.isfinite(line_width) or line_width < 0.0:
        parser.error("--line-width must be finite and >= 0")

    return {
        "find_radii": str(find_radii_mode),
        "find_buildup_factor": str(find_buildup_mode),
        "theta0_mode": str(theta0_mode),
        "r0_bounds": r0_bounds,
        "b_bounds": b_bounds,
        "r0_prior_sigma_mm": r0_prior_sigma_mm,
        "b_prior_sigma": b_prior_sigma,
        "spool_outer_iters": int(spool_outer_iters),
        "spool_inner_iters": int(spool_inner_iters),
        "line_width": float(line_width),
    }


def _build_full_auto_run_override_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--pointwise-residual",
        choices=["sampson", "euclidean"],
        default=None,
    )
    parser.add_argument(
        "--pointwise-filtering",
        dest="pointwise_filtering",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--no-pointwise-filtering",
        dest="pointwise_filtering",
        action="store_false",
    )
    parser.add_argument(
        "--pointwise-global-mad",
        dest="pointwise_global_mad",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--pointwise-per-sweep-mad",
        dest="pointwise_global_mad",
        action="store_false",
    )
    parser.add_argument(
        "--sweep-wise-filtering",
        dest="sweep_wise_filtering",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--no-sweep-wise-filtering",
        dest="sweep_wise_filtering",
        action="store_false",
    )
    parser.add_argument(
        "--sweep-metric",
        choices=["mad", "median_abs", "outlier_ratio"],
        default=None,
    )
    parser.add_argument(
        "--sigma-source",
        choices=["auto", "point", "origin", "min"],
        default=None,
    )
    parser.add_argument(
        "--use-noise-mean",
        dest="use_noise_mean",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--use-raw-lengths",
        dest="use_noise_mean",
        action="store_false",
    )
    parser.add_argument(
        "--flex",
        dest="use_flex",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--find-radii",
        choices=_SPOOL_FIND_MODE_CHOICES,
        nargs="?",
        const="per-anchor",
        default=None,
    )
    parser.add_argument(
        "--find-buildup-factor",
        choices=_SPOOL_FIND_MODE_CHOICES,
        nargs="?",
        const="per-anchor",
        default=None,
    )
    parser.add_argument("--base-radii", default=None)
    parser.add_argument("--buildup-factor", type=float, default=None)
    parser.add_argument("--r0-bounds", default=None)
    parser.add_argument("--b-bounds", default=None)
    parser.add_argument("--r0-prior-sigma-mm", type=float, default=None)
    parser.add_argument("--b-prior-sigma", type=float, default=None)
    parser.add_argument("--spool-outer-iters", type=int, default=None)
    parser.add_argument("--spool-inner-iters", type=int, default=None)
    parser.add_argument("--theta0-mode", choices=_THETA0_MODE_CHOICES, default=None)
    parser.add_argument("--line-width", type=float, default=None)
    parser.add_argument("--spring-k-multiplier", type=float, default=None)
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--solve-restarts", type=int, default=None)
    parser.add_argument("--solve-iterations", type=int, default=None)
    parser.add_argument("--solve-optimizer", default=None)
    return parser


def _parse_full_auto_run_spec(spec: str) -> Tuple[List[str], Dict[str, object]]:
    tokens = shlex.split(spec)
    if not tokens:
        return [], {}
    parser = _build_full_auto_run_override_parser()
    parsed, unknown = parser.parse_known_args(tokens)
    if unknown:
        raise ValueError(f"Unknown full-auto run flags: {' '.join(unknown)}")
    overrides = {k: v for k, v in vars(parsed).items() if v is not None}
    return tokens, overrides


def _build_full_auto_runs(raw_runs: Optional[Sequence[str]]) -> List[Dict[str, object]]:
    runs: List[Dict[str, object]] = []
    if not raw_runs:
        runs.append({"id": "default", "flags": "", "overrides": {}})
        return runs
    for idx, spec in enumerate(raw_runs, start=1):
        if spec is None:
            continue
        spec_str = str(spec).strip()
        if not spec_str:
            continue
        tokens, overrides = _parse_full_auto_run_spec(spec_str)
        run_id = f"run_{idx:02d}"
        runs.append(
            {
                "id": run_id,
                "flags": " ".join(tokens),
                "overrides": overrides,
            }
        )
    if not runs:
        runs.append({"id": "default", "flags": "", "overrides": {}})
    return runs


def _shotgun_conf_path() -> Path:
    return Path(__file__).with_name("shotgun.conf")


def _load_shotgun_runs() -> List[str]:
    path = _shotgun_conf_path()
    try:
        with path.open("r", encoding="utf-8") as f:
            lines = []
            for raw in f:
                stripped = raw.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                lines.append(stripped)
            return lines
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"shotgun config not found: {path}") from exc


def _plan_noise_metrics(plan: Dict[str, object]) -> Optional[dict]:
    cal = plan.get("calibration")
    if isinstance(cal, dict):
        details = cal.get("details")
        if isinstance(details, dict):
            nm = details.get("noise_metrics")
            if isinstance(nm, dict):
                return nm
    return None


def _plan_primary_cost(plan: Dict[str, object]) -> float:
    raw = plan.get("cost_noise_normalized", plan.get("cost", float("nan")))
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float("nan")


def _plan_covariance_summary(
    plan: Dict[str, object],
) -> Tuple[Optional[float], Optional[float], bool]:
    max_std = None
    ci = plan.get("confidence_intervals")
    if isinstance(ci, dict):
        raw = ci.get("max_std_mm")
        if isinstance(raw, (int, float)) and np.isfinite(raw):
            max_std = float(raw)
    cov_scaled = plan.get("covariance_scaled", plan.get("covariance"))
    cov_std = _covariance_diag_std(np.asarray(cov_scaled, dtype=float))
    cov_ok = cov_std is not None and np.all(np.isfinite(cov_std))
    if max_std is None and cov_std is not None:
        finite = cov_std[np.isfinite(cov_std)]
        if finite.size:
            max_std = float(np.max(finite))
    workspace = plan.get("workspace_diag_mm")
    rel_std = None
    if (
        max_std is not None
        and isinstance(workspace, (int, float))
        and np.isfinite(workspace)
        and workspace > 0.0
    ):
        rel_std = float(max_std) / float(workspace)
    return max_std, rel_std, bool(cov_ok)


def _plan_data_quality_warnings(plan: Dict[str, object]) -> List[str]:
    warnings: List[str] = []
    plan_warnings = plan.get("warnings")
    if isinstance(plan_warnings, list):
        for entry in plan_warnings:
            if isinstance(entry, str) and entry:
                warnings.append(entry)
    noise_summary = _summarize_encoder_noise(plan.get("dataset") if isinstance(plan, dict) else None)
    if isinstance(noise_summary, dict):
        total_points = int(noise_summary.get("total_points", 0))
        points_with_sigma = int(noise_summary.get("points_with_sigma", 0))
        if total_points > 0 and points_with_sigma == 0:
            warnings.append("encoder_noise_missing")
        low_samples = int(noise_summary.get("low_samples", 0))
        sigma_nonfinite = int(noise_summary.get("sigma_nonfinite", 0))
        if low_samples > 0:
            warnings.append("encoder_noise_low_samples")
        if sigma_nonfinite > 0:
            warnings.append("encoder_noise_nonfinite")
    seen = set()
    out: List[str] = []
    for w in warnings:
        if w not in seen:
            out.append(w)
            seen.add(w)
    return out


def _plan_hits_underconstrained_penalty(plan: Dict[str, object], primary_cost: Optional[float] = None) -> bool:
    try:
        cost = float(_plan_primary_cost(plan) if primary_cost is None else primary_cost)
    except (TypeError, ValueError):
        return False
    if not np.isfinite(cost):
        return False
    if abs(cost - 100.0) > 1e-9:
        return False
    noise_metrics = _plan_noise_metrics(plan)
    if isinstance(noise_metrics, dict):
        try:
            chi2_red = float(noise_metrics.get("chi2_red"))
        except (TypeError, ValueError):
            chi2_red = float("nan")
        if np.isfinite(chi2_red) and chi2_red > 1e3:
            return True
    return True


def _full_auto_cfg_path(dataset_path: Path, run_id: str) -> Path:
    return dataset_path.with_name(f"{dataset_path.stem}.active_sweep_cfg.{run_id}.txt")


def _full_auto_stop_path(dataset_path: Path) -> Path:
    return dataset_path.with_name(f"{dataset_path.stem}.full_auto.stop")


def _append_jsonl(path: Path, payload: dict) -> None:
    append_jsonl_line(path, payload, schema="full_auto_log_entry")


def _solution_quality_message(best_cost: Optional[float]) -> str:
    if best_cost is None or not np.isfinite(best_cost):
        return "Interpretation: Cost unavailable."
    cost = float(best_cost)
    if cost < 2.0:
        return (
            "Interpretation: Cost below 2 is considered ideal; residuals are roughly at the noise level."
        )
    if cost < 5.0:
        return (
            "Interpretation: Cost between 2 and 5 is good/acceptable; "
            "the noise model may be slightly optimistic or the model isn't perfect."
        )
    if cost < 10.0:
        return "Interpretation: Cost between 5 and 10 is still usable; this is a reasonable fit."
    return "Interpretation: Cost above 10 is concerning; this usually means a bad fit."


def _solution_quality_label(best_cost: Optional[float]) -> str:
    if best_cost is None or not np.isfinite(best_cost):
        return "unknown"
    cost = float(best_cost)
    if cost < 2.0:
        return "ideal"
    if cost < 5.0:
        return "good"
    if cost < 10.0:
        return "usable"
    return "concerning"


def _ordinal(n: int) -> str:
    n = int(n)
    if 10 <= (n % 100) <= 20:
        suffix = "th"
    else:
        suffix = {1: "st", 2: "nd", 3: "rd"}.get(n % 10, "th")
    return f"{n}{suffix}"


def ellipse_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_ellipse_parser()
    args = parser.parse_args(argv)
    spool_opts = _resolve_spool_cli_options(parser, args)
    collector_args = _clean_collector_args(args.collector_args)
    if bool(args.project_zero_tension) and not _arg_has_flag(collector_args, "--project-zero-tension"):
        collector_args.append("--project-zero-tension")
    if bool(args.debug_sweep_actions) and not _arg_has_flag(collector_args, "--debug-sweep-actions"):
        collector_args.append("--debug-sweep-actions")
    return ellipse_active(
        args.dataset,
        machine_type=str(args.machine_type),
        solve_restarts=int(args.solve_restarts),
        solve_iterations=int(args.solve_iterations),
        solve_optimizer=str(args.solve_optimizer),
        residual_threshold=float(args.threshold),
        spring_k_multiplier=float(args.spring_k_multiplier),
        use_flex=bool(args.flex),
        pointwise_residual_mode=str(args.pointwise_residual),
        pointwise_filtering=bool(args.pointwise_filtering),
        pointwise_global_mad=bool(args.pointwise_global_mad),
        sweep_wise_filtering=bool(args.sweep_wise_filtering),
        sweep_metric=str(args.sweep_metric),
        use_noise_mean=bool(args.use_noise_mean),
        sigma_source=str(args.sigma_source),
        robust_debug=bool(args.robust_debug),
        residuals_csv=args.residuals_csv,
        generate_report=bool(args.report),
        find_radii=str(spool_opts["find_radii"]),
        find_buildup_factor=str(spool_opts["find_buildup_factor"]),
        base_radii=_parse_csv_floats(args.base_radii),
        buildup_factor=args.buildup_factor,
        r0_bounds=spool_opts["r0_bounds"],
        b_bounds=spool_opts["b_bounds"],
        r0_prior_sigma_mm=spool_opts["r0_prior_sigma_mm"],
        b_prior_sigma=spool_opts["b_prior_sigma"],
        spool_outer_iters=int(spool_opts["spool_outer_iters"]),
        spool_inner_iters=int(spool_opts["spool_inner_iters"]),
        theta0_mode=str(spool_opts["theta0_mode"]),
        line_width=float(spool_opts["line_width"]),
        candidate_deltas=_parse_csv_floats(args.candidate_deltas),
        candidate_count=int(args.candidate_count),
        delta_min=args.delta_min,
        delta_max=args.delta_max,
        fd_eps_mm=float(args.fd_eps_mm),
        regularization=float(args.regularization),
        exclude_existing=not bool(args.no_exclude_existing),
        existing_tol_mm=float(args.existing_tol_mm),
        min_fixed_delta_spacing_mm=float(args.min_fixed_delta_spacing_mm),
        top_k=int(args.top_k),
        write_cfg=args.write_sweep_config,
        print_command=not bool(args.no_print_command),
        collector_args=collector_args,
        collect_once=bool(args.collect_once),
        collector_output=args.collector_output,
        merged_output_dataset=args.merged_output_dataset,
        sim=bool(args.sim),
        keep_sim_alive=bool(args.keep_sim_alive),
        hp_sim_reset=bool(args.hp_sim_reset),
        output_with_explanations=bool(args.output_with_explanations),
    )


def merge_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_merge_parser()
    args = parser.parse_args(argv)
    return merge_datasets(args.base, args.extra, output=args.output)


def semi_auto_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_semi_auto_parser()
    args = parser.parse_args(argv)
    spool_opts = _resolve_spool_cli_options(parser, args)
    if bool(args.shotgun) and not bool(args.full_auto):
        parser.error("--shotgun requires --full-auto")
    full_auto_runs = list(args.full_auto_run or [])
    if bool(args.shotgun):
        try:
            full_auto_runs.extend(_load_shotgun_runs())
        except FileNotFoundError as exc:
            parser.error(str(exc))
    collector_args = _clean_collector_args(args.collector_args)
    if bool(args.project_zero_tension) and not _arg_has_flag(collector_args, "--project-zero-tension"):
        collector_args.append("--project-zero-tension")
    if bool(args.debug_sweep_actions) and not _arg_has_flag(collector_args, "--debug-sweep-actions"):
        collector_args.append("--debug-sweep-actions")
    if bool(args.full_auto):
        return full_auto_loop(
            work_dataset=args.dataset,
            machine_type=str(args.machine_type),
            max_steps=int(args.max_steps),
            stop_cost=args.stop_cost,
            stop_std_mm=args.stop_std_mm,
            solve_restarts=int(args.solve_restarts),
            solve_iterations=int(args.solve_iterations),
            solve_optimizer=str(args.solve_optimizer),
            residual_threshold=float(args.threshold),
            spring_k_multiplier=float(args.spring_k_multiplier),
            use_flex=bool(args.flex),
            pointwise_residual_mode=str(args.pointwise_residual),
            pointwise_filtering=bool(args.pointwise_filtering),
            pointwise_global_mad=bool(args.pointwise_global_mad),
            sweep_wise_filtering=bool(args.sweep_wise_filtering),
            sweep_metric=str(args.sweep_metric),
            use_noise_mean=bool(args.use_noise_mean),
            sigma_source=str(args.sigma_source),
            robust_debug=bool(args.robust_debug),
            residuals_csv=args.residuals_csv,
            generate_report=bool(args.report),
            find_radii=str(spool_opts["find_radii"]),
            find_buildup_factor=str(spool_opts["find_buildup_factor"]),
            base_radii=_parse_csv_floats(args.base_radii),
            buildup_factor=args.buildup_factor,
            r0_bounds=spool_opts["r0_bounds"],
            b_bounds=spool_opts["b_bounds"],
            r0_prior_sigma_mm=spool_opts["r0_prior_sigma_mm"],
            b_prior_sigma=spool_opts["b_prior_sigma"],
            spool_outer_iters=int(spool_opts["spool_outer_iters"]),
            spool_inner_iters=int(spool_opts["spool_inner_iters"]),
            theta0_mode=str(spool_opts["theta0_mode"]),
            line_width=float(spool_opts["line_width"]),
            candidate_deltas=_parse_csv_floats(args.candidate_deltas),
            candidate_count=int(args.candidate_count),
            delta_min=args.delta_min,
            delta_max=args.delta_max,
            fd_eps_mm=float(args.fd_eps_mm),
            regularization=float(args.regularization),
            exclude_existing=not bool(args.no_exclude_existing),
            existing_tol_mm=float(args.existing_tol_mm),
            min_fixed_delta_spacing_mm=float(args.min_fixed_delta_spacing_mm),
            top_k=int(args.top_k),
            write_cfg=args.write_sweep_config,
            collector_args=collector_args,
            sim=bool(args.sim),
            keep_sim_alive=bool(args.keep_sim_alive),
            hp_sim_reset=bool(args.hp_sim_reset),
            sweep_points=args.sweep_points,
            output_with_explanations=bool(args.output_with_explanations),
            full_auto_runs=full_auto_runs,
            full_auto_log=args.full_auto_log,
            patience=int(args.patience),
            full_auto_verbose=bool(args.full_auto_verbose),
        )
    return ellipse_loop(
        work_dataset=args.dataset,
        machine_type=str(args.machine_type),
        max_steps=int(args.max_steps),
        stop_cost=args.stop_cost,
        stop_std_mm=args.stop_std_mm,
        solve_restarts=int(args.solve_restarts),
        solve_iterations=int(args.solve_iterations),
        solve_optimizer=str(args.solve_optimizer),
        residual_threshold=float(args.threshold),
        spring_k_multiplier=float(args.spring_k_multiplier),
        use_flex=bool(args.flex),
        pointwise_residual_mode=str(args.pointwise_residual),
        pointwise_filtering=bool(args.pointwise_filtering),
        pointwise_global_mad=bool(args.pointwise_global_mad),
        sweep_wise_filtering=bool(args.sweep_wise_filtering),
        sweep_metric=str(args.sweep_metric),
        use_noise_mean=bool(args.use_noise_mean),
        sigma_source=str(args.sigma_source),
        robust_debug=bool(args.robust_debug),
        residuals_csv=args.residuals_csv,
        generate_report=bool(args.report),
        find_radii=str(spool_opts["find_radii"]),
        find_buildup_factor=str(spool_opts["find_buildup_factor"]),
        base_radii=_parse_csv_floats(args.base_radii),
        buildup_factor=args.buildup_factor,
        r0_bounds=spool_opts["r0_bounds"],
        b_bounds=spool_opts["b_bounds"],
        r0_prior_sigma_mm=spool_opts["r0_prior_sigma_mm"],
        b_prior_sigma=spool_opts["b_prior_sigma"],
        spool_outer_iters=int(spool_opts["spool_outer_iters"]),
        spool_inner_iters=int(spool_opts["spool_inner_iters"]),
        theta0_mode=str(spool_opts["theta0_mode"]),
        line_width=float(spool_opts["line_width"]),
        candidate_deltas=_parse_csv_floats(args.candidate_deltas),
        candidate_count=int(args.candidate_count),
        delta_min=args.delta_min,
        delta_max=args.delta_max,
        fd_eps_mm=float(args.fd_eps_mm),
        regularization=float(args.regularization),
        exclude_existing=not bool(args.no_exclude_existing),
        existing_tol_mm=float(args.existing_tol_mm),
        min_fixed_delta_spacing_mm=float(args.min_fixed_delta_spacing_mm),
        top_k=int(args.top_k),
        write_cfg=args.write_sweep_config,
        collector_args=collector_args,
        sim=bool(args.sim),
        keep_sim_alive=bool(args.keep_sim_alive),
        hp_sim_reset=bool(args.hp_sim_reset),
        plot_residual_histogram=bool(args.plot_residual_histogram),
        sweep_points=args.sweep_points,
        output_with_explanations=bool(args.output_with_explanations),
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv_list = list(sys.argv[1:] if argv is None else argv)
    if argv_list:
        cmd = argv_list[0]
        if cmd == "ellipse-loop":
            return semi_auto_cli(argv_list[1:])
        if cmd == "ellipse":
            return ellipse_cli(argv_list[1:])
        if cmd == "merge":
            return merge_cli(argv_list[1:])
    return semi_auto_cli(argv_list)


if __name__ == "__main__":
    raise SystemExit(main())
