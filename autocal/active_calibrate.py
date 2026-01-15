#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

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
    cost_mode: str,
    pointwise_residual_mode: str,
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
    collector_output: Optional[Path],
    collector_args: Sequence[str],
) -> Dict[str, object]:
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
        pointwise_residual_mode=str(pointwise_residual_mode),
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
        config = dataset.get("config") if isinstance(dataset, dict) else None
        max_travel_mm = None
        if isinstance(config, dict):
            raw_max_travel = config.get("max_travel_mm")
            if isinstance(raw_max_travel, (int, float)) and np.isfinite(raw_max_travel):
                max_travel_mm = float(raw_max_travel)
        explicit_delta_range = delta_min is not None or delta_max is not None
        observed_deltas: List[float] = []
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
            if (not explicit_delta_range and max_travel_mm is not None
                    and np.isfinite(max_travel_mm) and float(max_travel_mm) > 0.0):
                span = 0.1 * float(max_travel_mm)
                lo = max(0.0, float(max_travel_mm) - span)
                hi = float(max_travel_mm)
            else:
                lo, hi = -600.0, 600.0
        span = float(hi - lo)
        if (
            not explicit_delta_range
            and span < 20.0  # Avoid near-degenerate grids when all observed deltas cluster
            and max_travel_mm is not None
            and np.isfinite(max_travel_mm)
            and float(max_travel_mm) > 0.0
        ):
            hi = max(hi, float(max_travel_mm))
            padded_lo = 10.0 if hi > 10.0 else max(0.0, hi - 20.0)
            # If all observed deltas are positive and tightly packed near hi, expand downwards to ~10mm.
            if lo >= 0.0:
                lo = min(lo, padded_lo)
            else:
                lo = padded_lo
        values = np.linspace(lo, hi, max(3, int(candidate_count)))
        candidate_deltas = [float(v) for v in values.tolist()]

    candidates = generate_candidate_sweeps(
        num_anchors=num_anchors,
        dimensions=dimensions,
        fixed_delta_values_mm=candidate_deltas,
        machine_type=machine_type,
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

    best_cfg = ranked[0][1] if ranked else None
    cfg_path = write_cfg or dataset_path.with_suffix(".active_sweep_cfg.txt")
    if best_cfg is not None:
        _write_sweep_config_file(cfg_path, best_cfg)

    collector_args_eff, force_tuning, force_args_applied = _inject_force_args(dataset, collector_args)
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
        "info": info_obs,
        "covariance": cov,
        "ranked": ranked,
        "best_cfg": best_cfg,
        "cfg_path": cfg_path,
        "collect_command": cmd,
        "force_tuning": force_tuning,
        "force_args_applied": force_args_applied,
    }


def _print_ellipse_plan(plan: Dict[str, object], *, top_n: int = 5, print_command: bool = True) -> None:
    cal = plan.get("calibration") or {}
    anchors = np.asarray(plan.get("anchors"), dtype=float)
    cost = float(plan.get("cost", float("nan")))
    info = np.asarray(plan.get("info"), dtype=float)
    cov = np.asarray(plan.get("covariance"), dtype=float)
    ranked = plan.get("ranked") or []
    cmd = plan.get("collect_command")

    print("; Active ellipse calibration")
    if isinstance(cal, dict) and "gcode" in cal:
        print(str(cal["gcode"]))
    print(f"; cost={cost:.6g} {_format_anchor_stats(anchors)}")
    if info.ndim == 2 and info.shape[0] == info.shape[1]:
        print(f"; info rank: {int(np.linalg.matrix_rank(info))}/{info.shape[0]} {_covariance_report(cov)}")

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
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    cost_mode: str,
    pointwise_residual_mode: str,
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
    plan = _plan_next_ellipse_sweep(
        dataset_path,
        solve_restarts=solve_restarts,
        solve_iterations=solve_iterations,
        solve_optimizer=solve_optimizer,
        residual_threshold=residual_threshold,
        spring_k_multiplier=spring_k_multiplier,
        use_flex=use_flex,
        cost_mode=cost_mode,
        pointwise_residual_mode=pointwise_residual_mode,
        generate_report=generate_report,
        candidate_deltas=candidate_deltas,
        candidate_count=candidate_count,
        delta_min=delta_min,
        delta_max=delta_max,
        fd_eps_mm=fd_eps_mm,
        regularization=regularization,
        exclude_existing=exclude_existing,
        existing_tol_mm=existing_tol_mm,
        top_k=top_k,
        write_cfg=write_cfg,
        collector_output=collector_output,
        collector_args=collector_args,
    )

    _print_ellipse_plan(plan, top_n=5, print_command=print_command)

    ranked = plan.get("ranked") or []
    cmd = plan.get("collect_command")
    if not ranked or not isinstance(cmd, list):
        return 2

    if not collect_once:
        return 0

    if collector_output is None:
        raise ValueError("--collect-once requires --collector-output")

    print(f"; running: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

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


def ellipse_loop(
    seed_dataset: Optional[Path],
    *,
    work_dataset: Optional[Path],
    max_steps: int,
    stop_cost: Optional[float],
    stop_std_mm: Optional[float],
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    cost_mode: str,
    pointwise_residual_mode: str,
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
    collector_args: Sequence[str],
) -> int:
    if work_dataset is not None:
        work_path = Path(work_dataset)
    elif seed_dataset is not None:
        work_path = Path(seed_dataset).with_name(Path(seed_dataset).stem + "_active.json")
    else:
        work_path = Path("autocal/data/active_bootstrap_slideprinter.json")

    if not work_path.exists():
        if seed_dataset is not None:
            _write_json(work_path, _load_json(Path(seed_dataset)))
            print(f"; created working dataset {work_path}")
        else:
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

            argv_eff = _strip_conflicts(list(collector_args))
            if "--return-to-origin" not in argv_eff and "--returnToOrigin" not in argv_eff:
                argv_eff.append("--return-to-origin")

            cmd = [
                "node",
                "scripts/collect_sweep_data.mjs",
                "--machineType",
                "slideprinter",
                "--sweep-config-file",
                str(bootstrap_cfg),
                "--output-file",
                str(work_path),
                *argv_eff,
            ]
            print("; bootstrapping dataset (3 sweeps, auto size-tune):")
            print(";   " + " ".join(cmd))
            subprocess.run(cmd, check=True)
            print(f"; bootstrap dataset written to {work_path}")

    for step in range(1, max(1, int(max_steps)) + 1):
        print(f"\n; === iteration {step}/{max_steps} dataset={work_path} ===")
        collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")

        plan = _plan_next_ellipse_sweep(
            work_path,
            solve_restarts=solve_restarts,
            solve_iterations=solve_iterations,
            solve_optimizer=solve_optimizer,
            residual_threshold=residual_threshold,
            spring_k_multiplier=spring_k_multiplier,
            use_flex=use_flex,
            cost_mode=cost_mode,
            pointwise_residual_mode=pointwise_residual_mode,
            generate_report=generate_report,
            candidate_deltas=candidate_deltas,
            candidate_count=candidate_count,
            delta_min=delta_min,
            delta_max=delta_max,
            fd_eps_mm=fd_eps_mm,
            regularization=regularization,
            exclude_existing=exclude_existing,
            existing_tol_mm=existing_tol_mm,
            top_k=top_k,
            write_cfg=write_cfg,
            collector_output=collector_output,
            collector_args=collector_args,
        )
        _print_ellipse_plan(plan, top_n=5, print_command=True)

        cost = float(plan.get("cost", float("nan")))
        cov = np.asarray(plan.get("covariance"), dtype=float)
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
            return 2

        try:
            while True:
                resp = input("Accept anchors [a], collect next sweep [c], quit [q]? ").strip().lower()
                if resp in ("a", "accept", "ok", "y", "yes"):
                    print(f"; accepted anchors; dataset={work_path}")
                    return 0
                if resp in ("q", "quit", "exit", "n", "no"):
                    print(f"; stopped; dataset={work_path}")
                    return 0
                if resp in ("c", "collect", "next", ""):
                    break
                print("Please enter a/c/q.")
        except KeyboardInterrupt:
            print(f"\n; interrupted; dataset={work_path}")
            return 130

        base_dataset = _load_json(work_path)
        print(f"; running: {' '.join(str(x) for x in cmd)}")
        subprocess.run(cmd, check=True)

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

    print(f"; reached max steps; dataset={work_path}")
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
    ellipse.add_argument(
        "--pointwise-residual",
        choices=["sampson", "euclidean"],
        default="sampson",
        help="Pointwise residual metric (only used when --cost-mode=pointwise)",
    )
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

    merge = subparsers.add_parser("merge", help="Merge one or more sweep datasets into a base dataset")
    merge.add_argument("base", type=Path, help="Base dataset JSON (keeps config metadata)")
    merge.add_argument("extra", type=Path, nargs="+", help="Additional dataset JSON files to append")
    merge.add_argument("-o", "--output", type=Path, default=None, help="Output dataset path (default: overwrite base)")

    loop = subparsers.add_parser("ellipse-loop", help="Interactive active-learning loop (plan → collect → merge)")
    loop.add_argument(
        "dataset",
        type=Path,
        nargs="?",
        default=None,
        help="Optional seed sweep dataset JSON; omitted bootstraps a 3-sweep slideprinter dataset with auto size-tuning",
    )
    loop.add_argument(
        "--work-dataset",
        type=Path,
        default=None,
        help="Working dataset file that is updated each iteration (default: <seed>_active.json)",
    )
    loop.add_argument("--max-steps", type=int, default=20, help="Maximum active-learning iterations")
    loop.add_argument("--stop-cost", type=float, default=None, help="Optional stop condition on ellipse cost")
    loop.add_argument("--stop-std-mm", type=float, default=None, help="Optional stop condition on max parameter std")

    # Reuse the same knobs as the one-shot planner.
    loop.add_argument("--solve-restarts", type=int, default=4)
    loop.add_argument("--solve-iterations", type=int, default=400)
    loop.add_argument("--solve-optimizer", default="L-BFGS-B")
    loop.add_argument("--threshold", type=float, default=250.0)
    loop.add_argument("--spring-k-multiplier", type=float, default=1.0)
    loop.add_argument("--flex", action="store_true")
    loop.add_argument("--cost-mode", choices=["pointwise", "geometry"], default="pointwise")
    loop.add_argument(
        "--pointwise-residual",
        choices=["sampson", "euclidean"],
        default="sampson",
        help="Pointwise residual metric (only used when --cost-mode=pointwise)",
    )
    loop.add_argument("--report", action="store_true", help="Write a PNG report (like calibrate.py)")

    loop.add_argument("--candidate-deltas", type=str, default=None, help="Comma-separated fixed deltas (mm)")
    loop.add_argument("--candidate-count", type=int, default=41, help="Grid size when deltas not provided")
    loop.add_argument("--delta-min", type=float, default=None)
    loop.add_argument("--delta-max", type=float, default=None)
    loop.add_argument("--fd-eps-mm", type=float, default=1.0)
    loop.add_argument("--regularization", type=float, default=1e-6)
    loop.add_argument("--no-exclude-existing", action="store_true")
    loop.add_argument("--existing-tol-mm", type=float, default=1e-3)
    loop.add_argument("--top-k", type=int, default=10)
    loop.add_argument("--write-sweep-config", type=Path, default=None)
    loop.add_argument(
        "--collector-args",
        nargs=argparse.REMAINDER,
        default=(),
        help="Extra args passed to scripts/collect_sweep_data.mjs (after --collector-args)",
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
            pointwise_residual_mode=str(args.pointwise_residual),
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

    if args.command == "merge":
        return merge_datasets(args.base, args.extra, output=args.output)

    if args.command == "ellipse-loop":
        collector_args = list(args.collector_args)
        if collector_args and collector_args[0] == "--":
            collector_args = collector_args[1:]
        return ellipse_loop(
            args.dataset,
            work_dataset=args.work_dataset,
            max_steps=int(args.max_steps),
            stop_cost=args.stop_cost,
            stop_std_mm=args.stop_std_mm,
            solve_restarts=int(args.solve_restarts),
            solve_iterations=int(args.solve_iterations),
            solve_optimizer=str(args.solve_optimizer),
            residual_threshold=float(args.threshold),
            spring_k_multiplier=float(args.spring_k_multiplier),
            use_flex=bool(args.flex),
            cost_mode=str(args.cost_mode),
            pointwise_residual_mode=str(args.pointwise_residual),
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
            collector_args=collector_args,
        )

    raise AssertionError("Unhandled command")


if __name__ == "__main__":
    raise SystemExit(main())
