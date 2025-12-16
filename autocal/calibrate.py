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


def _extract_motor_samples_from_sweep_dataset(dataset: dict, max_samples: int) -> Tuple[np.ndarray, int]:
    machine_type = str(dataset.get("machine_type", ""))
    dimensions = 2 if machine_type == "slideprinter" else 3

    rows = []
    for sweep in dataset.get("sweeps", []):
        for p in sweep.get("data_points", []):
            raw = p.get("raw_angles_deg")
            if raw is None:
                continue
            rows.append(raw)

    if not rows:
        raise ValueError("No raw encoder samples found (missing data_points[].raw_angles_deg).")

    motor_pos_samp = np.asarray(rows, dtype=float)
    if motor_pos_samp.ndim != 2:
        raise ValueError(f"raw_angles_deg produced unexpected shape {motor_pos_samp.shape}.")

    num_axes = int(motor_pos_samp.shape[1])
    if num_axes < 3 or num_axes > 8:
        raise ValueError(f"Legacy solver supports 3..8 axes; got {num_axes}.")

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
    motor_pos_samp, dimensions = _extract_motor_samples_from_sweep_dataset(dataset, max_samples=0)
    num_axes = int(motor_pos_samp.shape[1])

    tensions_rows: List[List[float]] = []
    have_any = False
    for sweep in dataset.get("sweeps", []):
        drive_idx = sweep.get("drive_anchor")
        sensor_idx = sweep.get("sensor_anchor")
        if drive_idx is None or sensor_idx is None:
            continue
        drive_idx = int(drive_idx)
        sensor_idx = int(sensor_idx)
        for p in sweep.get("data_points", []):
            raw = p.get("raw_angles_deg")
            if raw is None:
                continue
            row = [float("nan")] * num_axes
            td = p.get("assumed_tension_drive_n")
            ts = p.get("assumed_tension_sensor_n")
            if td is not None:
                row[drive_idx] = float(td)
                have_any = True
            if ts is not None:
                row[sensor_idx] = float(ts)
                have_any = True
            tensions_rows.append(row)

    tension_samp = np.asarray(tensions_rows, dtype=float) if tensions_rows else None
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
    verbose: bool = False,
    generate_report: bool = True,
    include_debug_fits: bool = True,
) -> Dict[str, Any]:
    dataset = _load_json(input_path)

    if "fitted_ellipses" in dataset:
        raise ValueError(
            "Input appears to contain stored ellipse fits; provide the raw sweep dataset (deltas only)."
        )

    _validate_sweep_roles(dataset)

    solution = solve_anchors(
        dataset,
        method=method,
        max_iterations=max_iterations,
        num_restarts=num_restarts,
        residual_threshold=residual_threshold,
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        verbose=verbose,
    )

    anchors = solution.get("anchors")
    if anchors is None:
        raise RuntimeError("Solver returned no anchors")

    anchors_arr = np.asarray(anchors, dtype=float)
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
    use_parallel: bool = False,
    ftol: float = 1e-9,
    eps: Optional[float] = None,
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

    solution_vec = sim.solve(
        motor_pos_samp,
        xyz_of_samp,
        line_lengths_when_at_origin,
        bool(use_flex),
        bool(use_line_lengths),
        debug=bool(verbose),
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
    )

    params_anch = int(np.shape(motor_pos_samp)[1]) * 3
    anchors = sim.anchorsvec2matrix(solution_vec[0:params_anch])
    result = {
        "method": "point",
        "anchors": np.asarray(anchors, dtype=float).tolist(),
        "solution_vector": np.asarray(solution_vec, dtype=float).tolist(),
        "use_flex": bool(use_flex),
        "flex_mode": str(flex_mode),
        "use_line_lengths": bool(use_line_lengths),
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
        "--optimizer",
        default="SLSQP",
        help="Optimizer method (default: SLSQP)",
    )
    ellipse_parser.add_argument("-v", "--verbose", action="store_true")
    ellipse_parser.add_argument("--debug", action="store_true", help="Alias for --verbose.")
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
        "--optimizer",
        default="SLSQP",
        help="scipy.optimize.minimize method (default: SLSQP).",
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
        action="store_true",
        help="Run restarts in parallel processes (can be slow to start).",
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
            num_restarts=args.restarts,
            max_iterations=args.iterations,
            method=args.optimizer,
            spring_k_multiplier=float(args.spring_k_multiplier),
            use_flex=bool(args.use_flex),
            verbose=bool(args.verbose or args.debug),
            generate_report=not args.no_report,
            include_debug_fits=not args.no_debug_fits,
        )
        print(result["gcode"])
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
            use_parallel=bool(args.parallel),
            ftol=float(args.ftol),
            eps=args.eps,
        )
        print(json.dumps(result, indent=2))
        return 0

    raise AssertionError("Unhandled method")


if __name__ == "__main__":
    raise SystemExit(main())
