#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from autocal.active_learning import (
    SweepConfig,
    dataset_sweep_configs,
    generate_candidate_sweeps,
    l2_scale_for_machine,
    rank_candidates_d_optimal,
    total_information_matrix,
)
from autocal.calibrate import calibrate_elliptical

GeometryWeights = Tuple[float, float, float]


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


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


def _covariance_report(cov: np.ndarray, *, top: int = 6) -> str:
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
    return "std(" + ", ".join(parts) + ")"


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
    return merged


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
        "scripts/collect_sweep_data.mjs",
        "--machineType",
        str(machine_type),
        "--sweep-method",
        "torque-ramp",
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


def ellipse_active(
    dataset_path: Path,
    *,
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    cost_mode: str,
    generate_report: bool,
    candidate_deltas: Optional[List[float]],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    fd_eps_mm: float,
    regularization: float,
    exclude_existing: bool,
    existing_tol_mm: float,
    top_k: int,
    write_cfg: Optional[Path],
    print_command: bool,
    collector_args: Sequence[str],
    collect_once: bool,
    collector_output: Optional[Path],
    merged_output_dataset: Optional[Path],
) -> int:
    dataset = _load_json(dataset_path)
    machine_type = str(dataset.get("machine_type", "hangprinter_4"))
    num_anchors = int(dataset.get("num_anchors", 4))
    dimensions = int(dataset.get("dimensions", 3))

    cal = calibrate_elliptical(
        dataset_path,
        output_path=None,
        residual_threshold=float(residual_threshold),
        num_restarts=int(solve_restarts),
        max_iterations=int(solve_iterations),
        method=str(solve_optimizer),
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        verbose=False,
        use_parallel=False,
        cost_mode=str(cost_mode),
        generate_report=bool(generate_report),
        include_debug_fits=False,
    )
    anchors = np.asarray(cal["anchors"], dtype=float)
    cost = float(cal.get("cost", float("nan")))

    sweeps_obs = dataset_sweep_configs(dataset)
    l2_scale = l2_scale_for_machine(machine_type, num_anchors, dimensions)
    info_obs = total_information_matrix(
        anchors,
        sweeps_obs,
        machine_type=machine_type,
        num_anchors=num_anchors,
        dimensions=dimensions,
        l2_scale=l2_scale,
        fd_eps_mm=float(fd_eps_mm),
    )
    cov = _estimate_anchor_covariance(info_obs, regularization=float(regularization))

    if candidate_deltas is None:
        observed_deltas = []
        for cfg in sweeps_obs:
            observed_deltas.extend(list(cfg.fixed_deltas_mm))
        if observed_deltas:
            lo = float(np.min(observed_deltas))
            hi = float(np.max(observed_deltas))
        else:
            lo, hi = -600.0, 600.0
        if delta_min is not None:
            lo = float(delta_min)
        if delta_max is not None:
            hi = float(delta_max)
        if not np.isfinite(lo) or not np.isfinite(hi) or abs(hi - lo) < 1e-9:
            lo, hi = -600.0, 600.0
        values = np.linspace(lo, hi, max(3, int(candidate_count)))
        candidate_deltas = [float(v) for v in values.tolist()]

    candidates = generate_candidate_sweeps(
        num_anchors=num_anchors,
        dimensions=dimensions,
        fixed_delta_values_mm=candidate_deltas,
    )

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
        exclude_existing=bool(exclude_existing),
        existing_tol_mm=float(existing_tol_mm),
        top_k=int(top_k),
    )

    print("; Active ellipse calibration")
    print(cal["gcode"])
    print(f"; cost={cost:.6g} {_format_anchor_stats(anchors)}")
    print(
        f"; info rank: {int(np.linalg.matrix_rank(info_obs))}/{info_obs.shape[0]} {_covariance_report(cov)}"
    )

    if not ranked:
        print("; No valid candidate sweeps found (check delta range and anchor estimate).")
        return 2

    best_score, best_cfg = ranked[0]
    print(
        f"; next_sweep score={best_score:.6g} fixed={list(best_cfg.fixed_anchors)} "
        f"targets={list(best_cfg.fixed_deltas_mm)} pair=[{best_cfg.drive_anchor},{best_cfg.sensor_anchor}]"
    )
    if len(ranked) > 1:
        print("; top_candidates:")
        for score, cfg in ranked[: min(5, len(ranked))]:
            print(
                f";   {score:.6g} fixed={list(cfg.fixed_anchors)} targets={list(cfg.fixed_deltas_mm)} "
                f"pair=[{cfg.drive_anchor},{cfg.sensor_anchor}]"
            )

    cfg_path = write_cfg or dataset_path.with_suffix(".active_sweep_cfg.txt")
    _write_sweep_config_file(cfg_path, best_cfg)

    collector_args_eff = list(collector_args)
    if "--return-to-origin" not in collector_args_eff and "--returnToOrigin" not in collector_args_eff:
        collector_args_eff.append("--return-to-origin")

    cmd = _suggested_collect_command(
        cfg_path,
        best_cfg,
        machine_type=machine_type,
        output_file=collector_output,
        extra_args=collector_args_eff,
    )
    if print_command:
        print("; collect_command:")
        print(";   " + " ".join(cmd))

    if not collect_once:
        return 0

    if collector_output is None:
        raise ValueError("--collect-once requires --collector-output")

    print(f"; running: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    new_dataset = _load_json(collector_output)
    merged = _merge_sweep_datasets(dataset, new_dataset)
    out_path = merged_output_dataset or dataset_path
    _write_json(out_path, merged)
    print(f"; merged dataset written to {out_path}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Active-learning calibration helpers")
    subparsers = parser.add_subparsers(dest="command", required=True)

    ellipse = subparsers.add_parser("ellipse", help="Active sweep selection for ellipse calibration")
    ellipse.add_argument("dataset", type=Path, help="Existing sweep dataset JSON")

    ellipse.add_argument("--solve-restarts", type=int, default=4)
    ellipse.add_argument("--solve-iterations", type=int, default=400)
    ellipse.add_argument("--solve-optimizer", default="L-BFGS-B")
    ellipse.add_argument("--threshold", type=float, default=250.0)
    ellipse.add_argument("--spring-k-multiplier", type=float, default=1.0)
    ellipse.add_argument("--flex", action="store_true")
    ellipse.add_argument("--cost-mode", choices=["pointwise", "geometry"], default="pointwise")
    ellipse.add_argument("--report", action="store_true", help="Write a PNG report (like calibrate.py)")

    ellipse.add_argument("--candidate-deltas", type=str, default=None, help="Comma-separated fixed deltas (mm)")
    ellipse.add_argument("--candidate-count", type=int, default=41, help="Grid size when deltas not provided")
    ellipse.add_argument("--delta-min", type=float, default=None)
    ellipse.add_argument("--delta-max", type=float, default=None)
    ellipse.add_argument("--fd-eps-mm", type=float, default=1.0)
    ellipse.add_argument("--regularization", type=float, default=1e-6)
    ellipse.add_argument("--no-exclude-existing", action="store_true")
    ellipse.add_argument("--existing-tol-mm", type=float, default=1e-3)
    ellipse.add_argument("--top-k", type=int, default=10)

    ellipse.add_argument("--write-sweep-config", type=Path, default=None)
    ellipse.add_argument("--no-print-command", action="store_true")
    ellipse.add_argument(
        "--collector-args",
        nargs=argparse.REMAINDER,
        default=(),
        help="Extra args passed to scripts/collect_sweep_data.mjs (after --collector-args)",
    )

    ellipse.add_argument("--collect-once", action="store_true", help="Collect the suggested sweep and merge")
    ellipse.add_argument("--collector-output", type=Path, default=None, help="Output JSON for collector")
    ellipse.add_argument(
        "--merged-output-dataset",
        type=Path,
        default=None,
        help="Where to write the merged dataset (default: overwrite input dataset)",
    )

    args = parser.parse_args(argv)

    if args.command == "ellipse":
        collector_args = list(args.collector_args)
        if collector_args and collector_args[0] == "--":
            collector_args = collector_args[1:]
        return ellipse_active(
            args.dataset,
            solve_restarts=int(args.solve_restarts),
            solve_iterations=int(args.solve_iterations),
            solve_optimizer=str(args.solve_optimizer),
            residual_threshold=float(args.threshold),
            spring_k_multiplier=float(args.spring_k_multiplier),
            use_flex=bool(args.flex),
            cost_mode=str(args.cost_mode),
            generate_report=bool(args.report),
            candidate_deltas=_parse_csv_floats(args.candidate_deltas),
            candidate_count=int(args.candidate_count),
            delta_min=args.delta_min,
            delta_max=args.delta_max,
            fd_eps_mm=float(args.fd_eps_mm),
            regularization=float(args.regularization),
            exclude_existing=not bool(args.no_exclude_existing),
            existing_tol_mm=float(args.existing_tol_mm),
            top_k=int(args.top_k),
            write_cfg=args.write_sweep_config,
            print_command=not bool(args.no_print_command),
            collector_args=collector_args,
            collect_once=bool(args.collect_once),
            collector_output=args.collector_output,
            merged_output_dataset=args.merged_output_dataset,
        )

    raise AssertionError("Unhandled command")


if __name__ == "__main__":
    raise SystemExit(main())
