#!/usr/bin/env python3
from __future__ import annotations

"""Unified calibration CLI for point-based and elliptical methods.

Elliptical calibration consumes sweep datasets where lengths are stored as
encoder-driven deltas from the origin. Absolute lengths are reconstructed from
anchor guesses inside the optimizer.

Point-based calibration delegates to the legacy solver in
`autocal/auto-calibration-simulation-for-hangprinter/simulation.py`.
"""

import argparse
import contextlib
import io
import json
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autocal.ellipse_fitting import fit_all_sweeps
from autocal.ellipse_solver import format_anchors_gcode, solve_anchors
from autocal.ellipse_visualization import create_calibration_report
from autocal.sweep_types import MachineConfig, MachineType


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def _rebuild_absolute_sweeps(dataset: dict, anchors: np.ndarray) -> List[Dict[str, Any]]:
    """Deep-copy sweeps and inflate delta lengths using anchor baselines."""
    import copy

    dataset_copy = copy.deepcopy(dataset)
    baselines = np.linalg.norm(anchors, axis=1)
    sweeps_abs = []

    for sweep in dataset_copy.get("sweeps", []):
        fixed_abs = [
            float(baselines[idx] + delta)
            for idx, delta in zip(sweep.get("fixed_anchors", []), sweep.get("fixed_lengths", []))
        ]
        drive_idx = int(sweep["drive_anchor"])
        sensor_idx = int(sweep["sensor_anchor"])
        drive_base = float(baselines[drive_idx])
        sensor_base = float(baselines[sensor_idx])
        points_abs = []
        for p in sweep.get("data_points", []):
            points_abs.append(
                {
                    **p,
                    "l_drive": float(p["l_drive"] + drive_base),
                    "l_sensor": float(p["l_sensor"] + sensor_base),
                }
            )
        sweeps_abs.append({**sweep, "fixed_lengths": fixed_abs, "data_points": points_abs})

    return sweeps_abs


def _validate_sweep_roles(dataset: dict) -> None:
    """Fast validation for carrying anchors and role duplication."""
    machine_type = str(dataset.get("machine_type", "hangprinter_4"))
    try:
        mt = MachineType(machine_type)
    except ValueError as exc:
        raise ValueError(f"Unsupported machine_type {machine_type!r}") from exc

    config = MachineConfig.from_type(mt)
    sweeps = dataset.get("sweeps", [])
    if not isinstance(sweeps, list) or not sweeps:
        raise ValueError("Dataset contains no sweeps")

    errors = []
    for sweep in sweeps:
        fixed = list(sweep.get("fixed_anchors", []))
        drive = sweep.get("drive_anchor")
        sensor = sweep.get("sensor_anchor")
        if sensor in (config.carrying_anchors or []):
            errors.append(f"{sweep.get('id','<unnamed>')}: carrying anchors cannot be Sensor")
        all_indices = fixed + [drive, sensor]
        if len(set(all_indices)) != len(all_indices):
            errors.append(f"{sweep.get('id','<unnamed>')}: duplicate anchor indices in roles")
        if len(fixed) != config.constraints_for_1dof:
            errors.append(
                f"{sweep.get('id','<unnamed>')}: expected {config.constraints_for_1dof} fixed anchors"
            )
    if errors:
        raise ValueError("Invalid sweep dataset:\n" + "\n".join(errors))


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


def _print_solution_summary(method: str, anchors: np.ndarray, *, cost: Optional[float] = None) -> None:
    cost_str = "n/a" if cost is None or not np.isfinite(cost) else f"{float(cost):.6g}"
    print(f"; [{method}] cost={cost_str} {_format_anchor_stats(anchors)}")


def _canonicalize_slideprinter_anchors_for_output(anchors: np.ndarray) -> np.ndarray:
    """
    Fix the unobservable global rotation/reflection gauge for Slideprinter.

    Sweep data contains only lengths, so anchors are identifiable only up to a rigid
    rotation/reflection about the origin. For consistent, human-friendly output we:
      1) rotate so anchor 0 lies on +Y (x≈0, y>0)
      2) reflect (mirror) so anchor 1 has x>=0
    """
    anchors = np.asarray(anchors, dtype=float)
    if anchors.shape != (3, 2):
        return anchors

    a0 = anchors[0]
    ang = float(np.arctan2(a0[1], a0[0]))
    # Slideprinter conventions (Klipper configs, examples) place anchor 0 at (0, -R).
    rot = float(-np.pi / 2 - ang)
    c = float(np.cos(rot))
    s = float(np.sin(rot))
    R = np.array([[c, -s], [s, c]], dtype=float)
    out = anchors @ R.T

    if out[0, 1] > 0:
        out[:, 1] *= -1.0
    if out[1, 0] < 0:
        out[:, 0] *= -1.0
    return out


def _canonicalize_anchors_for_output(machine_type: str, anchors: np.ndarray) -> np.ndarray:
    anchors = np.asarray(anchors, dtype=float)
    if str(machine_type) == "slideprinter" and anchors.shape == (3, 2):
        return _canonicalize_slideprinter_anchors_for_output(anchors)
    return anchors


def _extract_motor_samples_with_metadata(dataset: dict) -> Tuple[np.ndarray, int, List[Tuple[int, List[int]]]]:
    machine_type = str(dataset.get("machine_type", ""))
    dimensions = 2 if machine_type == "slideprinter" else 3

    rows: List[List[float]] = []
    meta: List[Tuple[int, List[int]]] = []
    for sweep_idx, sweep in enumerate(dataset.get("sweeps", [])):
        fixed = [int(x) for x in sweep.get("fixed_anchors", [])]
        for p in sweep.get("data_points", []):
            raw = p.get("raw_angles_deg")
            if raw is None:
                continue
            rows.append(raw)
            meta.append((int(sweep_idx), fixed))

    if not rows:
        raise ValueError("No raw encoder samples found (missing data_points[].raw_angles_deg).")

    motor_pos_samp = np.asarray(rows, dtype=float)
    if motor_pos_samp.ndim != 2:
        raise ValueError(f"raw_angles_deg produced unexpected shape {motor_pos_samp.shape}.")

    num_axes = int(motor_pos_samp.shape[1])
    if num_axes < 3 or num_axes > 8:
        raise ValueError(f"Legacy solver supports 3..8 axes; got {num_axes}.")

    return motor_pos_samp, dimensions, meta


def _enforce_per_sweep_fixed_anchor_constant(
    motor_pos_samp: np.ndarray, meta: List[Tuple[int, List[int]]]
) -> np.ndarray:
    """Replace fixed-anchor motor samples with their per-sweep mean to enforce sweep structure."""
    motor = np.asarray(motor_pos_samp, dtype=float).copy()
    if motor.ndim != 2 or not meta:
        return motor

    num_axes = int(motor.shape[1])
    rows_by_sweep: Dict[int, List[int]] = {}
    fixed_by_sweep: Dict[int, List[int]] = {}
    for row_idx, (sweep_idx, fixed_axes) in enumerate(meta):
        rows_by_sweep.setdefault(int(sweep_idx), []).append(int(row_idx))
        fixed_by_sweep.setdefault(int(sweep_idx), fixed_axes)

    for sweep_idx, rows in rows_by_sweep.items():
        fixed_axes = fixed_by_sweep.get(int(sweep_idx), [])
        for axis in fixed_axes:
            if axis < 0 or axis >= num_axes:
                continue
            vals = motor[np.asarray(rows, dtype=int), axis]
            vals = vals[np.isfinite(vals)]
            if vals.size == 0:
                continue
            motor[np.asarray(rows, dtype=int), axis] = float(np.mean(vals))

    return motor


def _regularize_fixed_anchor_motor_positions_by_fixed_length(
    dataset: dict, motor_pos_samp: np.ndarray, meta: List[Tuple[int, List[int]]]
) -> np.ndarray:
    """
    Optional "super sweep" regularization.

    For each fixed anchor axis, fit a line:
      motor_angle_mean ~= a + b * fixed_length_delta_mm
    across sweeps and replace the per-sweep fixed-anchor motor values with the fitted values.

    This leverages that fixed lengths are typically commanded in regular steps.
    """
    motor = np.asarray(motor_pos_samp, dtype=float).copy()
    sweeps = dataset.get("sweeps", [])
    if motor.ndim != 2 or not isinstance(sweeps, list) or not sweeps or not meta:
        return motor

    num_axes = int(motor.shape[1])
    rows_by_sweep: Dict[int, List[int]] = {}
    fixed_by_sweep: Dict[int, List[int]] = {}
    for row_idx, (sweep_idx, fixed_axes) in enumerate(meta):
        rows_by_sweep.setdefault(int(sweep_idx), []).append(int(row_idx))
        fixed_by_sweep.setdefault(int(sweep_idx), fixed_axes)

    pairs: Dict[Tuple[Tuple[int, ...], int], List[Tuple[float, float, int]]] = {}
    for sweep_idx, rows in rows_by_sweep.items():
        if sweep_idx < 0 or sweep_idx >= len(sweeps):
            continue
        sweep = sweeps[sweep_idx]
        fixed_axes = fixed_by_sweep.get(int(sweep_idx), [])
        fixed_deltas = sweep.get("fixed_lengths", [])
        if not isinstance(fixed_deltas, list):
            continue
        fixed_tuple = tuple(int(x) for x in fixed_axes)
        for axis, delta in zip(fixed_axes, fixed_deltas):
            if axis < 0 or axis >= num_axes:
                continue
            try:
                delta_f = float(delta)
            except Exception:
                continue
            vals = motor[np.asarray(rows, dtype=int), axis]
            vals = vals[np.isfinite(vals)]
            if vals.size == 0:
                continue
            mean_angle = float(np.mean(vals))
            pairs.setdefault((fixed_tuple, int(axis)), []).append((delta_f, mean_angle, int(sweep_idx)))

    for (_fixed_tuple, axis), triplets in pairs.items():
        if len(triplets) < 3:
            continue
        deltas = np.asarray([t[0] for t in triplets], dtype=float)
        means = np.asarray([t[1] for t in triplets], dtype=float)
        if not np.all(np.isfinite(deltas)) or not np.all(np.isfinite(means)):
            continue
        if float(np.std(deltas)) <= 1e-12:
            continue
        A = np.column_stack([np.ones_like(deltas), deltas])
        coef, *_ = np.linalg.lstsq(A, means, rcond=None)
        a, b = float(coef[0]), float(coef[1])

        for delta_f, _mean_angle, sweep_idx in triplets:
            rows = rows_by_sweep.get(int(sweep_idx), [])
            if not rows:
                continue
            motor[np.asarray(rows, dtype=int), axis] = float(a + b * float(delta_f))

    return motor


def _extract_motor_samples_from_sweep_dataset(dataset: dict, max_samples: int) -> Tuple[np.ndarray, int]:
    motor_pos_samp, dimensions, meta = _extract_motor_samples_with_metadata(dataset)
    motor_pos_samp = _enforce_per_sweep_fixed_anchor_constant(motor_pos_samp, meta)

    if max_samples and motor_pos_samp.shape[0] > max_samples:
        idx = np.linspace(0, motor_pos_samp.shape[0] - 1, max_samples).astype(int)
        motor_pos_samp = motor_pos_samp[idx]

    return motor_pos_samp, dimensions


def _extract_motor_and_tension_samples_from_sweep_dataset(
    dataset: dict, max_samples: int
) -> Tuple[np.ndarray, Optional[np.ndarray], int]:
    """
    Extract legacy motor samples plus optional per-axis tension hints.

    For each data point we map:
      data_points[].assumed_tension_drive_n  -> sweep.drive_anchor
      data_points[].assumed_tension_sensor_n -> sweep.sensor_anchor
    All other axes are NaN (unknown) and may be filled in by equilibrium.
    """
    motor_pos_raw, dimensions, meta = _extract_motor_samples_with_metadata(dataset)
    num_axes = int(motor_pos_raw.shape[1])

    tension_rows: List[List[float]] = []
    have_any = False
    row_cursor = 0
    for sweep in dataset.get("sweeps", []):
        drive_idx = sweep.get("drive_anchor")
        sensor_idx = sweep.get("sensor_anchor")
        drive_idx = int(drive_idx) if drive_idx is not None else -1
        sensor_idx = int(sensor_idx) if sensor_idx is not None else -1
        for p in sweep.get("data_points", []):
            raw = p.get("raw_angles_deg")
            if raw is None:
                continue
            row = [float("nan")] * num_axes
            td = p.get("assumed_tension_drive_n")
            ts = p.get("assumed_tension_sensor_n")
            if td is not None and 0 <= drive_idx < num_axes:
                row[drive_idx] = float(td)
                have_any = True
            if ts is not None and 0 <= sensor_idx < num_axes:
                row[sensor_idx] = float(ts)
                have_any = True
            tension_rows.append(row)
            row_cursor += 1

    motor_pos_samp = _enforce_per_sweep_fixed_anchor_constant(motor_pos_raw, meta)

    tension_samp = np.asarray(tension_rows, dtype=float) if tension_rows else None
    if tension_samp is not None and tension_samp.shape[0] != motor_pos_samp.shape[0]:
        # Fallback: if ordering mismatch, disable tensions rather than risk misalignment.
        tension_samp = None
        have_any = False

    if max_samples and motor_pos_samp.shape[0] > max_samples:
        idx = np.linspace(0, motor_pos_samp.shape[0] - 1, max_samples).astype(int)
        motor_pos_samp = motor_pos_samp[idx]
        if tension_samp is not None:
            tension_samp = tension_samp[idx]

    return motor_pos_samp, (tension_samp if have_any else None), dimensions


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
            out.extend([float(out[0] if out else default)] * (num_axes - len(out)))
        return out
    return [float(default)] * num_axes


def _first_float(value: Any, default: float) -> float:
    if isinstance(value, (list, tuple)) and value:
        value = value[0]
    try:
        return float(value)
    except Exception:
        return float(default)


def _extract_legacy_machine_config(dataset: dict, num_axes: int) -> Optional[Dict[str, Any]]:
    cfg = dataset.get("config")
    if not isinstance(cfg, dict):
        return None
    m666 = cfg.get("m666") or cfg.get("m666_after") or cfg.get("m666_before")
    if not isinstance(m666, dict):
        return None

    spool_r = _expand_to_num_axes(m666.get("R"), num_axes, 75.0)
    mech_adv = _expand_to_num_axes(m666.get("U"), num_axes, 1.0)
    lines = _expand_to_num_axes(m666.get("O"), num_axes, 1.0)
    motor_gear = _expand_to_num_axes(m666.get("L"), num_axes, 1.0)
    spool_gear = _expand_to_num_axes(m666.get("H"), num_axes, 1.0)
    guy_wires = _expand_to_num_axes(m666.get("Y"), num_axes, 0.0)
    min_force_limit = _first_float(m666.get("I"), 0.0)
    max_force_limit = _first_float(m666.get("X"), 120.0)
    ignore_gravity = bool(_first_float(m666.get("B"), 0.0))
    ignore_pretension = bool(_first_float(m666.get("P"), 0.0))

    out: Dict[str, Any] = {
        "spool_buildup_factor": _first_float(m666.get("Q"), 0.0),
        "spool_r_in_origin": spool_r,
        "spool_gear_teeth": spool_gear,
        "motor_gear_teeth": motor_gear,
        "mechanical_advantage": mech_adv,
        "lines_per_spool": lines,
        "guy_wire_lengths": guy_wires,
        "min_force_limit": float(min_force_limit),
        "max_force_limit": float(max_force_limit),
        "ignore_gravity": bool(ignore_gravity),
        "ignore_pretension": bool(ignore_pretension),
    }
    if "S" in m666:
        out["spring_k_per_unit_length"] = _first_float(m666.get("S"), 0.0)
    if "W" in m666:
        out["mover_weight"] = _first_float(m666.get("W"), 0.0)
    return out


def _debug_print_legacy_config_comparison(sim: Any, dataset: dict, num_axes: int) -> None:
    cfg = dataset.get("config")
    if not isinstance(cfg, dict):
        return
    dataset_machine_cfg = _extract_legacy_machine_config(dataset, num_axes)
    if dataset_machine_cfg is None:
        return

    try:
        resolved_dataset = sim._resolve_machine_config(dataset_machine_cfg, num_axes)  # type: ignore[attr-defined]
        resolved_defaults = sim._resolve_machine_config(None, num_axes)  # type: ignore[attr-defined]
    except Exception as exc:
        print(f"[config] Unable to resolve legacy machine config: {exc}", file=sys.stderr)
        return

    def _jsonify(obj: Any) -> Any:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.floating, np.integer)):
            return obj.item()
        if isinstance(obj, dict):
            return {k: _jsonify(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [_jsonify(v) for v in obj]
        return obj

    payload = {
        "dataset_config_keys": sorted(cfg.keys()),
        "dataset_m666": cfg.get("m666"),
        "dataset_mm_per_degree": cfg.get("mm_per_degree"),
        "resolved_from_dataset": _jsonify(resolved_dataset),
        "resolved_from_data_py_defaults": _jsonify(resolved_defaults),
    }
    print("[config] Legacy solver config comparison (dataset vs data.py defaults):", file=sys.stderr)
    print(json.dumps(payload, indent=2, sort_keys=True), file=sys.stderr)


def calibrate_elliptical(
    input_path: Path,
    output_path: Optional[Path] = None,
    residual_threshold: float = 0.01,
    num_restarts: int = 8,
    max_iterations: int = 1000,
    method: str = "SLSQP",
    spring_k_multiplier: float = 1.0,
    use_flex: bool = False,
    robust_loss: bool = False,
    huber_delta: float = 1.0,
    robust_debug: bool = False,
    pointwise_filtering: bool = True,
    pointwise_global_mad: bool = True,
    sweep_wise_filtering: bool = True,
    sweep_metric: str = "outlier_ratio",
    use_noise_mean: bool = True,
    noise_normalized: bool = True,
    verbose: bool = False,
    progress_every: int = 10,
    use_parallel: bool = True,
    regularize_supersweep: bool = False,
    generate_report: bool = True,
    include_debug_fits: bool = True,
    pointwise_residual_mode: str = "sampson",
    residuals_csv: Optional[Path] = None,
) -> Dict[str, Any]:
    dataset = _load_json(input_path)
    # `regularize_supersweep` is primarily relevant for the legacy point solver (raw motor samples);
    # ellipse mode consumes fixed_lengths setpoints, which are typically already regular.
    _ = bool(regularize_supersweep)

    if "fitted_ellipses" in dataset:
        raise ValueError(
            "Input appears to contain stored ellipse fits; provide the raw sweep dataset (deltas only)."
        )

    _validate_sweep_roles(dataset)
    if verbose:
        sweeps = dataset.get("sweeps", [])
        print(
            "[ellipse] config:",
            f"machine_type={dataset.get('machine_type','?')}",
            f"anchors={dataset.get('num_anchors','?')}",
            f"dims={dataset.get('dimensions','?')}",
            f"sweeps={len(sweeps) if isinstance(sweeps, list) else '?'}",
            f"optimizer={method}",
            f"restarts={num_restarts}",
            f"maxiter={max_iterations}",
            f"threshold={residual_threshold}",
            f"progress_every={progress_every}",
            f"flex={use_flex}",
        )

    solution = solve_anchors(
        dataset,
        method=method,
        max_iterations=max_iterations,
        num_restarts=num_restarts,
        use_parallel=bool(use_parallel) and not bool(verbose),
        progress_every=int(progress_every),
        residual_threshold=residual_threshold,
        pointwise_residual_mode=str(pointwise_residual_mode),
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        robust_loss=bool(robust_loss),
        huber_delta=float(huber_delta),
        robust_debug=bool(robust_debug),
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        use_noise_mean=bool(use_noise_mean),
        noise_normalized=bool(noise_normalized),
        residuals_csv=residuals_csv,
        verbose=verbose,
    )

    anchors = solution.get("anchors")
    if anchors is None:
        raise RuntimeError("Solver returned no anchors")

    anchors_arr = _canonicalize_anchors_for_output(str(dataset.get("machine_type", "")), np.asarray(anchors, dtype=float))
    if verbose and str(dataset.get("machine_type", "")) == "slideprinter" and anchors_arr.shape == (3, 2):
        print(
            "[ellipse] note: Slideprinter anchors from sweep-length data are only identifiable up to a global "
            "rotation/reflection; output is canonicalized for readability."
        )
    gcode = format_anchors_gcode(anchors_arr, dataset.get("machine_type", ""))

    debug_fits = None
    if include_debug_fits:
        abs_sweeps = _rebuild_absolute_sweeps(dataset, anchors_arr)
        debug_fits = fit_all_sweeps(
            abs_sweeps,
            residual_threshold=residual_threshold,
            square_inputs=True,
        )

    result_payload: Dict[str, Any] = {
        "input_file": str(input_path),
        "timestamp": datetime.now().isoformat(),
        "machine_type": dataset.get("machine_type", "unknown"),
        "anchors": anchors_arr.tolist(),
        "cost": float(solution.get("cost", float("nan"))),
        "success": bool(solution.get("success", False)),
        "anchor_norms": _anchor_norms(anchors_arr),
        "pairwise_distances": _pairwise_anchor_distances(anchors_arr),
        "gcode": gcode,
        "use_flex": bool(use_flex),
    }

    details = solution.get("details")
    if details is not None:
        if hasattr(details, "__dict__"):
            details_dict = asdict(details)  # type: ignore[arg-type]
            if isinstance(details_dict.get("anchor_estimate"), np.ndarray):
                details_dict["anchor_estimate"] = details_dict["anchor_estimate"].tolist()
            result_payload["details"] = details_dict
        elif isinstance(details, dict):
            result_payload["details"] = details

    if debug_fits is not None:
        result_payload["ellipse_fits_debug"] = debug_fits

    if output_path is not None:
        _write_json(output_path, result_payload)

    if generate_report:
        report_path = (
            output_path.with_name(output_path.stem + "_report.png")
            if output_path is not None
            else input_path.with_name(input_path.stem + "_report.png")
        )
        create_calibration_report(dataset, result_payload, ellipse_fits=debug_fits, output_path=str(report_path))

    return result_payload


def _load_legacy_simulation() -> Tuple[Any, Path]:
    """Load legacy simulation module by adding its directory to sys.path."""
    legacy_dir = REPO_ROOT / "autocal" / "auto-calibration-simulation-for-hangprinter"
    if not (legacy_dir / "simulation.py").exists():
        raise FileNotFoundError(f"Legacy solver not found at {legacy_dir}")

    sys.path.insert(0, str(legacy_dir))
    try:
        import importlib

        sim = importlib.import_module("simulation")
        return sim, legacy_dir
    finally:
        if sys.path and sys.path[0] == str(legacy_dir):
            sys.path.pop(0)


def calibrate_point_based(
    use_flex: bool = True,
    use_line_lengths: bool = True,
    flex_mode: str = "per_sample",
    verbose: bool = False,
    input_path: Optional[Path] = None,
    output_path: Optional[Path] = None,
    max_samples: int = 800,
    spring_k_multiplier: float = 1.0,
    optimizer_method: str = "SLSQP",
    tries: int = 4,
    maxiter: int = 500,
    use_parallel: bool = True,
    ftol: float = 1e-9,
    eps: Optional[float] = None,
    regularize_supersweep: bool = False,
    raw_squared_cost: bool = True,
    huber_delta_mm: float = 10.0,
) -> Dict[str, Any]:
    sim, _ = _load_legacy_simulation()

    motor_pos_samp = sim.motor_pos_samp
    xyz_of_samp = sim.xyz_of_samp
    line_lengths_when_at_origin = sim.line_lengths_when_at_origin

    dimensions = 3
    machine_config = None
    tension_samp = None
    if input_path is not None:
        dataset = _load_json(input_path)
        if bool(regularize_supersweep):
            motor_pos_full, tension_full, dimensions = _extract_motor_and_tension_samples_from_sweep_dataset(
                dataset, max_samples=0
            )
            motor_pos_meta, _dims_meta, meta = _extract_motor_samples_with_metadata(dataset)
            motor_pos_meta = _enforce_per_sweep_fixed_anchor_constant(motor_pos_meta, meta)
            motor_pos_full = _regularize_fixed_anchor_motor_positions_by_fixed_length(
                dataset, motor_pos_meta, meta
            )
            motor_pos_samp = motor_pos_full
            tension_samp = tension_full
            if max_samples and motor_pos_samp.shape[0] > max_samples:
                idx = np.linspace(0, motor_pos_samp.shape[0] - 1, max_samples).astype(int)
                motor_pos_samp = motor_pos_samp[idx]
                if tension_samp is not None:
                    tension_samp = tension_samp[idx]
        else:
            motor_pos_samp, tension_samp, dimensions = _extract_motor_and_tension_samples_from_sweep_dataset(
                dataset, max_samples=max_samples
            )
        machine_config = _extract_legacy_machine_config(dataset, int(motor_pos_samp.shape[1]))
        if machine_config is not None and spring_k_multiplier != 1.0:
            base = machine_config.get("spring_k_per_unit_length")
            try:
                base_val = float(base)
            except Exception:
                base_val = None
            if base_val is not None and np.isfinite(base_val) and base_val > 0.0:
                machine_config["spring_k_per_unit_length"] = float(base_val) * float(spring_k_multiplier)
        if verbose and machine_config is not None:
            _debug_print_legacy_config_comparison(sim, dataset, int(motor_pos_samp.shape[1]))

        xyz_of_samp = np.zeros((0, 3), dtype=float)
        use_line_lengths = False
    else:
        dataset = None

    if not bool(verbose):
        # Legacy solver prints directly; keep CLI output comparable to ellipse mode by default.
        with contextlib.redirect_stdout(io.StringIO()):
            solution_vec = sim.solve(
                motor_pos_samp,
                xyz_of_samp,
                line_lengths_when_at_origin,
                bool(use_flex),
                bool(use_line_lengths),
                debug=False,
                dimensions=int(dimensions),
                optimizer_method=str(optimizer_method),
                tries=int(tries),
                maxiter=int(maxiter),
                use_parallel=bool(use_parallel),
                ftol=float(ftol),
                eps=(None if eps is None else float(eps)),
                machine_config=machine_config,
                flex_mode=str(flex_mode),
                tension_samp=tension_samp,
                raw_squared_cost=bool(raw_squared_cost),
                huber_delta_mm=float(huber_delta_mm),
            )
    else:
        solution_vec = sim.solve(
            motor_pos_samp,
            xyz_of_samp,
            line_lengths_when_at_origin,
            bool(use_flex),
            bool(use_line_lengths),
            debug=True,
            dimensions=int(dimensions),
            optimizer_method=str(optimizer_method),
            tries=int(tries),
            maxiter=int(maxiter),
            use_parallel=bool(use_parallel) and not bool(verbose),
            ftol=float(ftol),
            eps=(None if eps is None else float(eps)),
            machine_config=machine_config,
            flex_mode=str(flex_mode),
            tension_samp=tension_samp,
            raw_squared_cost=bool(raw_squared_cost),
            huber_delta_mm=float(huber_delta_mm),
        )

    params_anch = int(np.shape(motor_pos_samp)[1]) * 3
    anchors = sim.anchorsvec2matrix(solution_vec[0:params_anch])
    anchors_arr = np.asarray(anchors, dtype=float)
    if int(dimensions) == 2:
        anchors_arr = anchors_arr[:, :2]
    machine_type = str((dataset or {}).get("machine_type", "unknown")) if dataset is not None else "unknown"
    anchors_arr = _canonicalize_anchors_for_output(machine_type, anchors_arr)

    # Recompute the legacy objective for reporting.
    try:
        num_axes = int(np.shape(motor_pos_samp)[1])
        params_buildup_local = 2 if (num_axes == 5 and int(dimensions) == 3) else num_axes
        resolved_cfg = sim._resolve_machine_config(machine_config, num_axes)  # type: ignore[attr-defined]
        spool_buildup_factor = float(resolved_cfg["spool_buildup_factor"])
        spool_to_motor_gearing_factor = np.asarray(resolved_cfg["spool_to_motor_gearing_factor"], dtype=float)
        mech_adv = np.asarray(resolved_cfg["mechanical_advantage"], dtype=float)
        lines_per_spool_local = np.asarray(resolved_cfg["lines_per_spool"], dtype=float)
        spring_k_per_unit_length = float(resolved_cfg["spring_k_per_unit_length"])
        mover_weight_local = float(resolved_cfg["mover_weight"])
        guy_wire_lengths = np.asarray(resolved_cfg["guy_wire_lengths"], dtype=float)
        ignore_gravity = bool(resolved_cfg.get("ignore_gravity", False))
        ignore_pretension = bool(resolved_cfg.get("ignore_pretension", False))

        flex_mode_effective = str(flex_mode)
        if bool(use_flex) and flex_mode_effective == "per_sample":
            t_arr = None if tension_samp is None else np.asarray(tension_samp, dtype=float)
            if (
                t_arr is None
                or t_arr.ndim != 2
                or t_arr.shape[0] != int(np.shape(motor_pos_samp)[0])
                or t_arr.shape[1] != num_axes
                or not np.any(np.isfinite(t_arr))
            ):
                flex_mode_effective = "inverse_transform_planned"
        flex_param_count = 1 if (bool(use_flex) and flex_mode_effective == "inverse_transform_planned") else 0
        x = np.asarray(solution_vec, dtype=float).reshape(-1)
        params_perturb = int(getattr(sim, "params_perturb", 3))
        posvec = x[params_anch : -(params_buildup_local + params_perturb + flex_param_count)]
        anchvec = x[0:params_anch]
        spool_r = x[-(params_buildup_local + params_perturb + flex_param_count) : -(params_perturb + flex_param_count)]
        perturb = x[-(params_perturb + flex_param_count) : (x.size - flex_param_count)]
        low_axis_max_force = float(x[-1]) if int(flex_param_count) else float(0.0)
        cost = float(
            sim.costx(
                posvec,
                anchvec,
                spool_buildup_factor,
                spool_r,
                line_lengths_when_at_origin,
                perturb,
                bool(use_flex),
                bool(use_line_lengths),
                low_axis_max_force,
                motor_pos_samp,
                xyz_of_samp,
                int(dimensions),
                flex_mode=str(flex_mode_effective),
                tension_samp=tension_samp,
                spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                mech_adv_=mech_adv,
                lines_per_spool_=lines_per_spool_local,
                spring_k_per_unit_length=spring_k_per_unit_length,
                mover_weight=mover_weight_local,
                ignore_gravity=ignore_gravity,
                ignore_pretension=ignore_pretension,
                guy_wire_lengths=guy_wire_lengths,
                raw_squared_cost=bool(raw_squared_cost),
                huber_delta_mm=float(huber_delta_mm),
            )
        )
    except Exception:
        cost = float("nan")

    gcode = format_anchors_gcode(anchors_arr, machine_type)
    result = {
        "method": "point",
        "machine_type": machine_type,
        "anchors": anchors_arr.tolist(),
        "solution_vector": np.asarray(solution_vec, dtype=float).tolist(),
        "cost": float(cost),
        "success": bool(np.isfinite(cost)),
        "use_flex": bool(use_flex),
        "flex_mode": str(flex_mode),
        "use_line_lengths": bool(use_line_lengths),
        "anchor_norms": _anchor_norms(anchors_arr),
        "pairwise_distances": _pairwise_anchor_distances(anchors_arr),
        "gcode": gcode,
    }
    if input_path is not None:
        result["input_file"] = str(input_path)

    if output_path is not None:
        _write_json(output_path, result)

    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Unified calibration for cable-driven robots")
    subparsers = parser.add_subparsers(dest="command", required=True)

    ellipse_parser = subparsers.add_parser("ellipse", help="Elliptical feature calibration")
    ellipse_parser.add_argument("input", type=Path, help="Sweep data JSON file (deltas from origin)")
    ellipse_parser.add_argument("-o", "--output", type=Path, help="Output results JSON")
    ellipse_parser.add_argument(
        "-t",
        "--threshold",
        type=float,
        default=None,
        help="Sampson RMS threshold (default depends on machine_type)",
    )
    ellipse_parser.add_argument("-r", "--restarts", type=int, default=8, help="Number of optimization restarts")
    ellipse_parser.add_argument("-i", "--iterations", type=int, default=1000, help="Max iterations per restart")
    ellipse_parser.add_argument(
        "--pointwise-residual",
        choices=["sampson", "euclidean"],
        default="sampson",
        help="Pointwise residual metric (default: sampson).",
    )
    ellipse_parser.add_argument(
        "--pointwise-filtering",
        dest="pointwise_filtering",
        action="store_true",
        help="Enable GNC-IRLS style pointwise filtering (default).",
    )
    ellipse_parser.add_argument(
        "--no-pointwise-filtering",
        dest="pointwise_filtering",
        action="store_false",
        help="Disable pointwise filtering.",
    )
    ellipse_parser.add_argument(
        "--pointwise-global-mad",
        dest="pointwise_global_mad",
        action="store_true",
        help="Use a single global MAD scale for pointwise filtering (default).",
    )
    ellipse_parser.add_argument(
        "--pointwise-per-sweep-mad",
        dest="pointwise_global_mad",
        action="store_false",
        help="Use a per-sweep MAD scale for pointwise filtering.",
    )
    ellipse_parser.add_argument(
        "--sweep-wise-filtering",
        dest="sweep_wise_filtering",
        action="store_true",
        help="Enable sweep-wise outlier rejection (default).",
    )
    ellipse_parser.add_argument(
        "--no-sweep-wise-filtering",
        dest="sweep_wise_filtering",
        action="store_false",
        help="Disable sweep-wise outlier rejection.",
    )
    ellipse_parser.add_argument(
        "--sweep-metric",
        choices=["mad", "median_abs", "outlier_ratio"],
        default="outlier_ratio",
        help="Per-sweep metric used by sweep-wise filtering (default: mad).",
    )
    ellipse_lengths_group = ellipse_parser.add_mutually_exclusive_group()
    ellipse_lengths_group.add_argument(
        "--use-noise-mean",
        dest="use_noise_mean",
        action="store_true",
        help="Use encoder noise mean lengths when available (default).",
    )
    ellipse_lengths_group.add_argument(
        "--use-raw-lengths",
        dest="use_noise_mean",
        action="store_false",
        help="Use raw l_drive/l_sensor lengths (disable noise-mean datapoints).",
    )
    ellipse_parser.add_argument(
        "--progress-every",
        type=int,
        default=None,
        help="When verbose/debug, print progress every N iterations (default: 1 for --debug, 10 for --verbose).",
    )
    ellipse_parser.set_defaults(
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
    )
    ellipse_parser.add_argument(
        "--optimizer",
        default="L-BFGS-B",
        help="Optimizer method (default: L-BFGS-B)",
    )
    ellipse_cost_group = ellipse_parser.add_mutually_exclusive_group()
    ellipse_cost_group.add_argument(
        "--huber-loss",
        dest="huber_loss",
        action="store_true",
        help="Use a pseudo-Huber loss to reduce the influence of outlier sweeps.",
    )
    ellipse_cost_group.add_argument(
        "--raw-squared-cost",
        dest="huber_loss",
        action="store_false",
        help="Use the legacy raw squared-cost aggregation (default).",
    )
    ellipse_parser.set_defaults(huber_loss=False)
    ellipse_parser.add_argument(
        "--huber-delta",
        type=float,
        default=1.0,
        help="Pseudo-Huber delta for ellipse cost (dimensionless, used with --huber-loss).",
    )
    ellipse_parser.add_argument(
        "--robust-debug",
        action="store_true",
        help="Print diagnostics for robustness filtering.",
    )
    ellipse_parser.add_argument(
        "--residuals-csv",
        type=Path,
        default=None,
        help="Write pointwise residuals (approx mm) to CSV after the final GNC stage.",
    )
    ellipse_parser.add_argument("-v", "--verbose", action="store_true")
    ellipse_parser.add_argument("--debug", action="store_true", help="Alias for --verbose.")
    ellipse_parallel_group = ellipse_parser.add_mutually_exclusive_group()
    ellipse_parallel_group.add_argument(
        "--parallel",
        dest="parallel",
        action="store_true",
        help="Run restarts in parallel processes (default).",
    )
    ellipse_parallel_group.add_argument(
        "--no-parallel",
        dest="parallel",
        action="store_false",
        help="Run restarts sequentially (useful for debugging).",
    )
    ellipse_parser.set_defaults(parallel=True)
    ellipse_parser.add_argument("--no-report", action="store_true", help="Skip report generation")
    ellipse_parser.add_argument(
        "--no-debug-fits",
        action="store_true",
        help="Skip generating fitted-ellipse debug sidecar in output JSON",
    )
    ellipse_parser.add_argument(
        "--spring-k-multiplier",
        type=float,
        default=1.0,
        help="Multiply M666 S by this factor (e.g. 2.0 for two parallel lines per axis).",
    )
    ellipse_parser.add_argument(
        "--regularize-supersweep",
        action="store_true",
        help="Apply optional super-sweep regularization (currently a no-op for ellipse mode).",
    )
    ellipse_flex_group = ellipse_parser.add_mutually_exclusive_group()
    ellipse_flex_group.add_argument(
        "--flex",
        dest="use_flex",
        action="store_true",
        help="Enable flex compensation (default: disabled).",
    )
    ellipse_flex_group.add_argument(
        "--no-flex",
        dest="use_flex",
        action="store_false",
        help="Disable flex compensation (default).",
    )
    ellipse_parser.set_defaults(use_flex=False)

    point_parser = subparsers.add_parser("point", help="Legacy point-based calibration")
    point_parser.add_argument(
        "input_path",
        nargs="?",
        type=Path,
        help="Optional sweep dataset JSON; uses data_points[].raw_angles_deg as motor samples.",
    )
    point_parser.add_argument("--input", dest="input_path", type=Path, help="Same as positional input.")
    point_parser.add_argument("-o", "--output", type=Path, help="Optional output JSON")
    flex_group = point_parser.add_mutually_exclusive_group()
    flex_group.add_argument(
        "--flex",
        dest="use_flex",
        action="store_true",
        help="Enable flex compensation (default: disabled).",
    )
    flex_group.add_argument(
        "--no-flex",
        dest="use_flex",
        action="store_false",
        help="Disable flex compensation (default).",
    )
    point_parser.set_defaults(use_flex=False)
    flex_mode_group = point_parser.add_mutually_exclusive_group()
    flex_mode_group.add_argument(
        "--per-sample-flex",
        action="store_true",
        help="Use per-sample flex from assumed tensions (default).",
    )
    flex_mode_group.add_argument(
        "--inverse-transform-planned-flex",
        action="store_true",
        help="Use legacy inverse-transform planned flex (QP solver + tuned force limit).",
    )
    point_parser.add_argument(
        "--spring-k-multiplier",
        type=float,
        default=1.0,
        help="Multiply M666 S by this factor (e.g. 2.0 for two parallel lines per axis).",
    )
    point_parser.add_argument(
        "--regularize-supersweep",
        action="store_true",
        help="Fit fixed-anchor motor positions to fixed_lengths across sweeps (super-sweep regularization).",
    )
    point_parser.add_argument(
        "--optimizer",
        default="SLSQP",
        help="scipy.optimize.minimize method (default: SLSQP).",
    )
    point_cost_group = point_parser.add_mutually_exclusive_group()
    point_cost_group.add_argument(
        "--huber-loss",
        dest="huber_loss",
        action="store_true",
        help="Use a pseudo-Huber loss (quadratic near zero, linear for outliers).",
    )
    point_cost_group.add_argument(
        "--raw-squared-cost",
        dest="huber_loss",
        action="store_false",
        help="Use legacy raw sum-of-squared residuals cost (default).",
    )
    point_parser.set_defaults(huber_loss=False)
    point_parser.add_argument(
        "--huber-delta-mm",
        type=float,
        default=10.0,
        help="Pseudo-Huber delta in mm for point solver (used with --huber-loss).",
    )
    point_parser.add_argument(
        "--max-samples",
        type=int,
        default=800,
        help="Downsample raw encoder samples to this many rows (0 disables).",
    )
    point_parser.add_argument("--tries", type=int, default=4, help="Number of random restarts (default: 4).")
    point_parser.add_argument("--iterations", type=int, default=500, help="Max iterations per restart (default: 500).")
    point_parser.add_argument("--ftol", type=float, default=1e-9, help="Optimizer ftol (default: 1e-9).")
    point_parser.add_argument(
        "--eps",
        type=float,
        default=None,
        help="SLSQP finite-difference step (optional).",
    )
    point_parser.add_argument(
        "--parallel",
        dest="parallel",
        action="store_true",
        help="Run restarts in parallel processes (default).",
    )
    point_parser.add_argument(
        "--no-parallel",
        dest="parallel",
        action="store_false",
        help="Run restarts sequentially (useful for debugging).",
    )
    point_parser.set_defaults(parallel=True)
    point_parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result (default prints G-code + summary).",
    )
    point_parser.add_argument("--debug", action="store_true", help="Alias for --verbose.")
    point_parser.add_argument(
        "--no-line-lengths",
        action="store_true",
        help="Ignore hand-measured line lengths (default uses them)",
    )
    point_parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args(argv)

    if args.command == "ellipse":
        dataset = _load_json(args.input)
        if args.threshold is None:
            if str(dataset.get("machine_type", "")) == "slideprinter":
                threshold = 250.0
            else:
                threshold = 0.01
        else:
            threshold = float(args.threshold)

        result = calibrate_elliptical(
            args.input,
            args.output,
            residual_threshold=threshold,
            num_restarts=int(args.restarts),
            max_iterations=args.iterations,
            method=args.optimizer,
            spring_k_multiplier=float(args.spring_k_multiplier),
            use_flex=bool(args.use_flex),
            robust_loss=bool(args.huber_loss),
            huber_delta=float(args.huber_delta),
            robust_debug=bool(args.robust_debug),
            pointwise_filtering=bool(args.pointwise_filtering),
            pointwise_global_mad=bool(args.pointwise_global_mad),
            sweep_wise_filtering=bool(args.sweep_wise_filtering),
            sweep_metric=str(args.sweep_metric),
            use_noise_mean=bool(args.use_noise_mean),
            verbose=bool(args.verbose or args.debug),
            use_parallel=bool(args.parallel),
            regularize_supersweep=bool(args.regularize_supersweep),
            progress_every=(
                int(args.progress_every)
                if args.progress_every is not None
                else (1 if bool(args.debug) else 10)
            ),
            pointwise_residual_mode=str(args.pointwise_residual),
            generate_report=not args.no_report,
            include_debug_fits=not args.no_debug_fits,
            residuals_csv=args.residuals_csv,
        )
        print(result["gcode"])
        _print_solution_summary("ellipse", np.asarray(result["anchors"], dtype=float), cost=float(result.get("cost", float("nan"))))
        return 0

    if args.command == "point":
        verbose = bool(args.verbose or args.debug)
        flex_mode = "per_sample"
        if bool(args.inverse_transform_planned_flex):
            flex_mode = "inverse_transform_planned"
        result = calibrate_point_based(
            use_flex=bool(args.use_flex),
            use_line_lengths=not bool(args.no_line_lengths),
            flex_mode=flex_mode,
            verbose=verbose,
            input_path=args.input_path,
            output_path=args.output,
            max_samples=int(args.max_samples),
            spring_k_multiplier=float(args.spring_k_multiplier),
            optimizer_method=str(args.optimizer),
            tries=int(args.tries),
            maxiter=int(args.iterations),
            use_parallel=bool(args.parallel) and not bool(verbose),
            ftol=float(args.ftol),
            eps=args.eps,
            regularize_supersweep=bool(args.regularize_supersweep),
            raw_squared_cost=not bool(args.huber_loss),
            huber_delta_mm=float(args.huber_delta_mm),
        )
        if bool(args.json):
            print(json.dumps(result, indent=2))
        else:
            print(result["gcode"])
            _print_solution_summary("point", np.asarray(result["anchors"], dtype=float), cost=float(result.get("cost", float("nan"))))
        return 0

    raise AssertionError("Unhandled method")


if __name__ == "__main__":
    raise SystemExit(main())
