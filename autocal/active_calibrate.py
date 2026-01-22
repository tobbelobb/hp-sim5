#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

DEFAULT_RRF_PORT = 8081
RRF_SIM_BINARY = REPO_ROOT / "RRF" / "build" / "rrf_simulator"
RRF_SIM_ARGS = ["--vsd", "RRF/run/vsd", "-c", "sys/config_slideprinter.g", "--server", "-p"]

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


def _start_rrf_simulator(port: int) -> subprocess.Popen:
    if not RRF_SIM_BINARY.exists():
        raise FileNotFoundError(f"rrf_simulator not found at {RRF_SIM_BINARY}")
    args = [str(RRF_SIM_BINARY), *RRF_SIM_ARGS, str(int(port))]
    return subprocess.Popen(
        args,
        cwd=str(REPO_ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
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
    ransac: bool,
    ransac_trials: int,
    ransac_sample_size: int,
    ransac_min_inlier_ratio: float,
    ransac_threshold: Optional[float],
    ransac_seed: Optional[int],
    mahalanobis_rejection: bool,
    mahalanobis_threshold: float,
    mahalanobis_min_samples: int,
    mahalanobis_regularization: float,
    robust_debug: bool,
    generate_report: bool,
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
        ransac=bool(ransac),
        ransac_trials=int(ransac_trials),
        ransac_sample_size=int(ransac_sample_size),
        ransac_min_inlier_ratio=float(ransac_min_inlier_ratio),
        ransac_threshold=ransac_threshold if ransac_threshold is None else float(ransac_threshold),
        ransac_seed=ransac_seed if ransac_seed is None else int(ransac_seed),
        mahalanobis_rejection=bool(mahalanobis_rejection),
        mahalanobis_threshold=float(mahalanobis_threshold),
        mahalanobis_min_samples=int(mahalanobis_min_samples),
        mahalanobis_regularization=float(mahalanobis_regularization),
        robust_debug=bool(robust_debug),
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
    ransac: bool,
    ransac_trials: int,
    ransac_sample_size: int,
    ransac_min_inlier_ratio: float,
    ransac_threshold: Optional[float],
    ransac_seed: Optional[int],
    mahalanobis_rejection: bool,
    mahalanobis_threshold: float,
    mahalanobis_min_samples: int,
    mahalanobis_regularization: float,
    robust_debug: bool,
    generate_report: bool,
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
) -> int:
    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
    if sim and hp_sim_reset and collect_once and not _arg_has_flag(collector_args_eff, "--hp-sim-reset"):
        collector_args_eff.append("--hp-sim-reset")
    _, server_explicit, port = _resolve_rrf_target(collector_args)
    sim_process: Optional[subprocess.Popen] = None

    if sim and collect_once and not server_explicit and not user_no_spawn:
        target_port = port or DEFAULT_RRF_PORT
        print(f"; starting rrf_simulator at http://localhost:{target_port}")
        sim_process = _start_rrf_simulator(target_port)
        _wait_for_rrf_server(f"http://localhost:{target_port}")

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
        ransac=ransac,
        ransac_trials=ransac_trials,
        ransac_sample_size=ransac_sample_size,
        ransac_min_inlier_ratio=ransac_min_inlier_ratio,
        ransac_threshold=ransac_threshold,
        ransac_seed=ransac_seed,
        mahalanobis_rejection=mahalanobis_rejection,
        mahalanobis_threshold=mahalanobis_threshold,
        mahalanobis_min_samples=mahalanobis_min_samples,
        mahalanobis_regularization=mahalanobis_regularization,
        robust_debug=robust_debug,
        generate_report=generate_report,
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

    _print_ellipse_plan(plan, top_n=5, print_command=print_command)

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
    ransac: bool,
    ransac_trials: int,
    ransac_sample_size: int,
    ransac_min_inlier_ratio: float,
    ransac_threshold: Optional[float],
    ransac_seed: Optional[int],
    mahalanobis_rejection: bool,
    mahalanobis_threshold: float,
    mahalanobis_min_samples: int,
    mahalanobis_regularization: float,
    robust_debug: bool,
    generate_report: bool,
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
) -> int:
    if work_dataset is not None:
        work_path = Path(work_dataset)
    elif seed_dataset is not None:
        work_path = Path(seed_dataset).with_name(Path(seed_dataset).stem + "_active.json")
    else:
        work_path = Path("autocal/data/active_bootstrap_slideprinter.json")

    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
    reset_pending = bool(sim and hp_sim_reset)
    rrf_server, server_explicit, port = _resolve_rrf_target(collector_args)
    sim_process: Optional[subprocess.Popen] = None

    if sim and not server_explicit and not user_no_spawn:
        target_port = port or DEFAULT_RRF_PORT
        print(f"; starting rrf_simulator at http://localhost:{target_port}")
        sim_process = _start_rrf_simulator(target_port)
        _wait_for_rrf_server(f"http://localhost:{target_port}")

    def _finalize(code: int) -> int:
        if sim_process and not keep_sim_alive:
            _stop_process(sim_process)
        return code

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

            argv_eff = _strip_conflicts(list(collector_args_eff))
            if reset_pending and not _arg_has_flag(argv_eff, "--hp-sim-reset"):
                argv_eff.append("--hp-sim-reset")
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
            reset_pending = False
            print(f"; bootstrap dataset written to {work_path}")

    for step in range(1, max(1, int(max_steps)) + 1):
        print(f"\n; === iteration {step}/{max_steps} dataset={work_path} ===")
        collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")

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
            cost_mode=cost_mode,
            pointwise_residual_mode=pointwise_residual_mode,
            ransac=ransac,
            ransac_trials=ransac_trials,
            ransac_sample_size=ransac_sample_size,
            ransac_min_inlier_ratio=ransac_min_inlier_ratio,
            ransac_threshold=ransac_threshold,
            ransac_seed=ransac_seed,
            mahalanobis_rejection=mahalanobis_rejection,
            mahalanobis_threshold=mahalanobis_threshold,
            mahalanobis_min_samples=mahalanobis_min_samples,
            mahalanobis_regularization=mahalanobis_regularization,
            robust_debug=robust_debug,
            generate_report=generate_report,
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

    print(f"; reached max steps; dataset={work_path}")
    return _finalize(0)


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
    ellipse.add_argument("--ransac", action="store_true", help="Use RANSAC for per-sweep ellipse fits.")
    ellipse.add_argument("--ransac-trials", type=int, default=60)
    ellipse.add_argument("--ransac-sample-size", type=int, default=5)
    ellipse.add_argument("--ransac-min-inlier-ratio", type=float, default=0.5)
    ellipse.add_argument("--ransac-threshold", type=float, default=None)
    ellipse.add_argument("--ransac-seed", type=int, default=0)
    ellipse.add_argument(
        "--mahalanobis-reject",
        action="store_true",
        help="Discard sweeps whose ellipse geometry is a Mahalanobis outlier.",
    )
    ellipse.add_argument("--mahalanobis-threshold", type=float, default=3.0)
    ellipse.add_argument("--mahalanobis-min-samples", type=int, default=8)
    ellipse.add_argument("--mahalanobis-regularization", type=float, default=1e-6)
    ellipse.add_argument(
        "--robust-debug",
        action="store_true",
        help="Print diagnostics for RANSAC and Mahalanobis filtering.",
    )
    ellipse.add_argument("--report", action="store_true", help="Write a PNG report (like calibrate.py)")

    ellipse.add_argument("--candidate-deltas", type=str, default=None, help="Comma-separated fixed deltas (mm)")
    ellipse.add_argument("--candidate-count", type=int, default=41, help="Grid size when deltas not provided")
    ellipse.add_argument("--delta-min", type=float, default=None)
    ellipse.add_argument("--delta-max", type=float, default=None)
    ellipse.add_argument("--fd-eps-mm", type=float, default=1.0)
    ellipse.add_argument("--regularization", type=float, default=1e-6)
    ellipse.add_argument("--no-exclude-existing", action="store_true")
    ellipse.add_argument(
        "--existing-tol-mm",
        type=float,
        default=10.0,
        help="Treat sweeps with the same fixed anchors/drive/sensor and fixed delta within this tolerance as duplicates",
    )
    ellipse.add_argument(
        "--min-fixed-delta-spacing-mm",
        type=float,
        default=20.0,
        help="Minimum spacing (mm) from previously collected fixed deltas with the same sign",
    )
    ellipse.add_argument("--top-k", type=int, default=10)

    ellipse.add_argument("--write-sweep-config", type=Path, default=None)
    ellipse.add_argument("--no-print-command", action="store_true")
    ellipse.add_argument(
        "--collector-args",
        nargs=argparse.REMAINDER,
        default=(),
        help="Extra args passed to scripts/collect_sweep_data.mjs (after --collector-args)",
    )
    ellipse.add_argument(
        "--sim",
        "--simulation",
        action="store_true",
        help="Use rrf_simulator + hp-sim WebSocket bridge (default: talk to real firmware)",
    )
    ellipse.add_argument(
        "--keep-sim-alive",
        action="store_true",
        help="Leave rrf_simulator running after exit (only when --sim)",
    )
    ellipse.add_argument(
        "--hp-sim-reset",
        action="store_true",
        help="Reset hp-sim once before the first sweep (only when --sim)",
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
    loop.add_argument("--ransac", action="store_true", help="Use RANSAC for per-sweep ellipse fits.")
    loop.add_argument("--ransac-trials", type=int, default=60)
    loop.add_argument("--ransac-sample-size", type=int, default=5)
    loop.add_argument("--ransac-min-inlier-ratio", type=float, default=0.5)
    loop.add_argument("--ransac-threshold", type=float, default=None)
    loop.add_argument("--ransac-seed", type=int, default=0)
    loop.add_argument(
        "--mahalanobis-reject",
        action="store_true",
        help="Discard sweeps whose ellipse geometry is a Mahalanobis outlier.",
    )
    loop.add_argument("--mahalanobis-threshold", type=float, default=3.0)
    loop.add_argument("--mahalanobis-min-samples", type=int, default=8)
    loop.add_argument("--mahalanobis-regularization", type=float, default=1e-6)
    loop.add_argument(
        "--robust-debug",
        action="store_true",
        help="Print diagnostics for RANSAC and Mahalanobis filtering.",
    )
    loop.add_argument("--report", action="store_true", help="Write a PNG report (like calibrate.py)")

    loop.add_argument("--candidate-deltas", type=str, default=None, help="Comma-separated fixed deltas (mm)")
    loop.add_argument("--candidate-count", type=int, default=41, help="Grid size when deltas not provided")
    loop.add_argument("--delta-min", type=float, default=None)
    loop.add_argument("--delta-max", type=float, default=None)
    loop.add_argument("--fd-eps-mm", type=float, default=1.0)
    loop.add_argument("--regularization", type=float, default=1e-6)
    loop.add_argument("--no-exclude-existing", action="store_true")
    loop.add_argument(
        "--existing-tol-mm",
        type=float,
        default=10.0,
        help="Treat sweeps with the same fixed anchors/drive/sensor and fixed delta within this tolerance as duplicates",
    )
    loop.add_argument(
        "--min-fixed-delta-spacing-mm",
        type=float,
        default=20.0,
        help="Minimum spacing (mm) from previously collected fixed deltas with the same sign",
    )
    loop.add_argument("--top-k", type=int, default=10)
    loop.add_argument("--write-sweep-config", type=Path, default=None)
    loop.add_argument(
        "--collector-args",
        nargs=argparse.REMAINDER,
        default=(),
        help="Extra args passed to scripts/collect_sweep_data.mjs (after --collector-args)",
    )
    loop.add_argument(
        "--sim",
        "--simulation",
        action="store_true",
        help="Use rrf_simulator + hp-sim WebSocket bridge (default: talk to real firmware)",
    )
    loop.add_argument(
        "--keep-sim-alive",
        action="store_true",
        help="Leave rrf_simulator running after exit (only when --sim)",
    )
    loop.add_argument(
        "--hp-sim-reset",
        action="store_true",
        help="Reset hp-sim once before the first sweep (only when --sim)",
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
            ransac=bool(args.ransac),
            ransac_trials=int(args.ransac_trials),
            ransac_sample_size=int(args.ransac_sample_size),
            ransac_min_inlier_ratio=float(args.ransac_min_inlier_ratio),
            ransac_threshold=args.ransac_threshold,
            ransac_seed=(None if args.ransac_seed is not None and args.ransac_seed < 0 else int(args.ransac_seed)),
            mahalanobis_rejection=bool(args.mahalanobis_reject),
            mahalanobis_threshold=float(args.mahalanobis_threshold),
            mahalanobis_min_samples=int(args.mahalanobis_min_samples),
            mahalanobis_regularization=float(args.mahalanobis_regularization),
            robust_debug=bool(args.robust_debug),
            generate_report=bool(args.report),
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
            ransac=bool(args.ransac),
            ransac_trials=int(args.ransac_trials),
            ransac_sample_size=int(args.ransac_sample_size),
            ransac_min_inlier_ratio=float(args.ransac_min_inlier_ratio),
            ransac_threshold=args.ransac_threshold,
            ransac_seed=(None if args.ransac_seed is not None and args.ransac_seed < 0 else int(args.ransac_seed)),
            mahalanobis_rejection=bool(args.mahalanobis_reject),
            mahalanobis_threshold=float(args.mahalanobis_threshold),
            mahalanobis_min_samples=int(args.mahalanobis_min_samples),
            mahalanobis_regularization=float(args.mahalanobis_regularization),
            robust_debug=bool(args.robust_debug),
            generate_report=bool(args.report),
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
        )

    raise AssertionError("Unhandled command")


if __name__ == "__main__":
    raise SystemExit(main())
