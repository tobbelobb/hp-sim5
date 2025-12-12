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


def calibrate_elliptical(
    input_path: Path,
    output_path: Optional[Path] = None,
    residual_threshold: float = 0.01,
    num_restarts: int = 8,
    max_iterations: int = 1000,
    method: str = "SLSQP",
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
    use_flex: bool = False,
    use_line_lengths: bool = True,
    verbose: bool = False,
) -> Dict[str, Any]:
    sim, _ = _load_legacy_simulation()

    solution_vec = sim.solve(
        sim.motor_pos_samp,
        sim.xyz_of_samp,
        sim.line_lengths_when_at_origin,
        bool(use_flex),
        bool(use_line_lengths),
        debug=bool(verbose),
    )

    anchors = sim.anchorsvec2matrix(solution_vec[0 : sim.params_anch])
    return {
        "method": "point",
        "anchors": np.asarray(anchors, dtype=float).tolist(),
        "solution_vector": np.asarray(solution_vec, dtype=float).tolist(),
        "use_flex": bool(use_flex),
        "use_line_lengths": bool(use_line_lengths),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Unified calibration for cable-driven robots")
    subparsers = parser.add_subparsers(dest="command", required=True)

    ellipse_parser = subparsers.add_parser("ellipse", help="Elliptical feature calibration")
    ellipse_parser.add_argument("input", type=Path, help="Sweep data JSON file (deltas from origin)")
    ellipse_parser.add_argument("-o", "--output", type=Path, help="Output results JSON")
    ellipse_parser.add_argument("-t", "--threshold", type=float, default=0.01, help="Sampson RMS threshold")
    ellipse_parser.add_argument("-r", "--restarts", type=int, default=8, help="Number of optimization restarts")
    ellipse_parser.add_argument("-i", "--iterations", type=int, default=1000, help="Max iterations per restart")
    ellipse_parser.add_argument(
        "--optimizer",
        default="SLSQP",
        help="Optimizer method (default: SLSQP)",
    )
    ellipse_parser.add_argument("-v", "--verbose", action="store_true")
    ellipse_parser.add_argument("--no-report", action="store_true", help="Skip report generation")
    ellipse_parser.add_argument(
        "--no-debug-fits",
        action="store_true",
        help="Skip generating fitted-ellipse debug sidecar in output JSON",
    )

    point_parser = subparsers.add_parser("point", help="Legacy point-based calibration")
    point_parser.add_argument("--flex", action="store_true", help="Enable flex compensation (default off)")
    point_parser.add_argument(
        "--no-line-lengths",
        action="store_true",
        help="Ignore hand-measured line lengths (default uses them)",
    )
    point_parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args(argv)

    if args.command == "ellipse":
        result = calibrate_elliptical(
            args.input,
            args.output,
            residual_threshold=args.threshold,
            num_restarts=args.restarts,
            max_iterations=args.iterations,
            method=args.optimizer,
            verbose=args.verbose,
            generate_report=not args.no_report,
            include_debug_fits=not args.no_debug_fits,
        )
        print(result["gcode"])
        return 0

    if args.command == "point":
        result = calibrate_point_based(
            use_flex=bool(args.flex),
            use_line_lengths=not bool(args.no_line_lengths),
            verbose=bool(args.verbose),
        )
        print(json.dumps(result, indent=2))
        return 0

    raise AssertionError("Unhandled method")


if __name__ == "__main__":
    raise SystemExit(main())
