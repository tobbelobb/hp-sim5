#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shlex
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
DEFAULT_NOISE_SIGMA_FLOOR_DEG = 0.01
DEFAULT_NOISE_MIN_SAMPLES = 10

from autocal.active_learning import (
    SweepConfig,
    dataset_sweep_configs,
    generate_candidate_sweeps,
    l2_scale_for_machine,
    rank_candidates_d_optimal,
    total_information_matrix,
)
from autocal.calibrate import calibrate_elliptical
from autocal.ellipse_cost import EllipseCostFunction
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
    for key, label in (("chi2_red", "chi2_red"), ("normalized_cost", "J")):
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
    machine_type = _require_machine_type(dataset, context=str(dataset_path))
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
        pointwise_residual_mode=str(pointwise_residual_mode),
        robust_debug=bool(robust_debug),
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        use_noise_mean=bool(use_noise_mean),
        sigma_source=str(sigma_source),
        generate_report=bool(generate_report),
        include_debug_fits=False,
        residuals_csv=residuals_csv,
    )
    anchors = np.asarray(cal["anchors"], dtype=float)
    cost = float(cal.get("cost", float("nan")))
    cost_raw = float(
        _evaluate_cost_at_anchors(
            dataset,
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
        sweeps_obs,
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
    warnings: List[str] = []
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
    noise_summary = _summarize_encoder_noise(plan.get("dataset") if isinstance(plan, dict) else None)

    print("; Active ellipse calibration")
    if isinstance(cal, dict) and "gcode" in cal:
        print(str(cal["gcode"]))
    cost_raw_str = _fmt_float(cost_raw)
    cost_norm_str = _fmt_float(cost)
    print(
        f"; cost={cost_raw_str} cost_noise_normalized={cost_norm_str} {_format_anchor_stats(anchors)}"
    )
    if isinstance(cal, dict):
        details = cal.get("details")
        if isinstance(details, dict):
            noise_metrics = details.get("noise_metrics")
            if isinstance(noise_metrics, dict):
                j_val = noise_metrics.get("normalized_cost")
                chi2_red = noise_metrics.get("chi2_red")
                z_med = noise_metrics.get("median_abs_z")
                z_p95 = noise_metrics.get("p95_abs_z")
                outlier_ratio = noise_metrics.get("outlier_ratio")
                n_obs = noise_metrics.get("n_obs")
                norm_mode = noise_metrics.get("norm_mode")
                lengths_mode = noise_metrics.get("lengths_mode")
                j_str = _fmt_float(j_val)
                chi2_str = _fmt_float(chi2_red)
                med_str = _fmt_float(z_med)
                p95_str = _fmt_float(z_p95)
                outlier_str = _fmt_float(outlier_ratio)
                n_str = f"{int(n_obs)}" if isinstance(n_obs, (int, float)) and np.isfinite(n_obs) else "n/a"
                mode_str = str(norm_mode) if norm_mode is not None else "n/a"
                lengths_str = str(lengths_mode) if lengths_mode is not None else "n/a"
                print(
                    f"; noise_cost: J={j_str} chi2_red={chi2_str} |z|_med={med_str} |z|_p95={p95_str} "
                    f"outlier_ratio={outlier_str} N={n_str} mode={mode_str} lengths={lengths_str}"
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
    full_auto_cost_epsilon: float,
    full_auto_converge_k: int,
    full_auto_verbose: bool,
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

    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
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

    runs = _build_full_auto_runs(full_auto_runs)
    log_path = Path(full_auto_log) if full_auto_log is not None else work_path.with_name(
        f"{work_path.stem}.full_auto_log.jsonl"
    )
    _append_jsonl(
        log_path,
        {
            "timestamp": datetime.now().isoformat(),
            "event": "start",
            "dataset": str(work_path),
            "runs": runs,
        },
    )
    print(f"; full-auto log: {log_path}")

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
        "robust_debug": bool(robust_debug and full_auto_verbose),
        "generate_report": bool(generate_report),
    }

    cost_history: List[float] = []
    small_improve_streak = 0
    epsilon = float(full_auto_cost_epsilon)
    if not np.isfinite(epsilon) or epsilon < 0.0:
        epsilon = 0.0
    k_required = max(1, int(full_auto_converge_k))

    for step in range(1, max(1, int(max_steps)) + 1):
        print(f"\n; === full-auto iteration {step}/{max_steps} dataset={work_path} ===")
        collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")
        run_results: List[Dict[str, object]] = []

        for run in runs:
            run_id = str(run.get("id", "run"))
            overrides = run.get("overrides") or {}
            if not isinstance(overrides, dict):
                overrides = {}
            settings = dict(base_solver)
            settings.update(overrides)

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
            valid = bool(np.isfinite(primary_cost) and cov_ok)

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
                        "chi2_red": noise_metrics.get("chi2_red") if isinstance(noise_metrics, dict) else None,
                        "normalized_cost": noise_metrics.get("normalized_cost")
                        if isinstance(noise_metrics, dict)
                        else None,
                        "info_rank": plan.get("info_rank"),
                        "info_rank_deficient": plan.get("info_rank_deficient"),
                        "max_std_mm": max_std_mm,
                        "rel_std": rel_std,
                        "covariance_ok": cov_ok,
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
                    "dataset": str(work_path),
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
            print("; full-auto: no valid calibration runs (non-finite cost or covariance).")
            return _finalize(2)

        def _sort_key(entry: Dict[str, object]) -> Tuple[float, float, str]:
            metrics = entry["metrics"]
            cost = float(metrics.get("primary_cost", float("inf")))
            rel = metrics.get("rel_std")
            rel_val = float(rel) if isinstance(rel, (int, float)) and np.isfinite(rel) else float("inf")
            return cost, rel_val, str(entry.get("id", ""))

        selected = sorted(valid_runs, key=_sort_key)[0]
        plan = selected["plan"]
        metrics = selected["metrics"]
        selected_id = str(selected.get("id", "run"))
        selected_flags = str(selected.get("flags", "")).strip()
        selected_cost = float(metrics.get("primary_cost", float("nan")))
        selected_max_std = metrics.get("max_std_mm")
        selected_rel_std = metrics.get("rel_std")
        selected_warnings = list(metrics.get("warnings") or [])

        if full_auto_verbose:
            _print_ellipse_plan(
                plan,
                top_n=5,
                print_command=False,
                output_with_explanations=bool(output_with_explanations),
            )

        if write_cfg is not None and plan.get("best_cfg") is not None:
            try:
                _write_sweep_config_file(Path(write_cfg), plan["best_cfg"])
            except Exception as exc:
                print(f"; full-auto warning: failed to write sweep config {write_cfg}: {exc}")

        prev_cost = cost_history[-1] if cost_history else None
        improvement = None
        if prev_cost is not None and np.isfinite(prev_cost) and np.isfinite(selected_cost):
            improvement = float(prev_cost) - float(selected_cost)
            if 0.0 <= improvement <= epsilon:
                small_improve_streak += 1
            elif improvement > epsilon:
                small_improve_streak = 0
            else:
                small_improve_streak = 0
        else:
            small_improve_streak = 0

        cost_history.append(selected_cost)
        converged = bool(np.isfinite(selected_cost) and small_improve_streak >= k_required)

        stop_cost_hit = (
            stop_cost is not None and np.isfinite(selected_cost) and selected_cost <= float(stop_cost)
        )
        stop_std_hit = False
        if stop_std_mm is not None and isinstance(selected_max_std, (int, float)):
            stop_std_hit = np.isfinite(selected_max_std) and float(selected_max_std) <= float(stop_std_mm)

        summary_flags = f" flags='{selected_flags}'" if selected_flags else ""
        summary_rel = _fmt_float(selected_rel_std)
        summary_std = _fmt_float(selected_max_std, suffix="mm")
        summary_cost = _fmt_float(selected_cost)
        print(
            f"; selected run={selected_id}{summary_flags} cost={summary_cost} "
            f"rel_std={summary_rel} max_std={summary_std}"
        )
        if selected_warnings:
            print("; data_quality_warnings: " + ", ".join(selected_warnings))

        accept = converged and not selected_warnings
        if stop_cost is not None:
            accept = bool(accept and stop_cost_hit)
        if stop_std_mm is not None:
            accept = bool(accept and stop_std_hit)

        if accept:
            decision = "accept"
        elif selected_warnings:
            decision = "stop_warning"
        else:
            decision = "collect"

        _append_jsonl(
            log_path,
            {
                "timestamp": datetime.now().isoformat(),
                "iteration": step,
                "dataset": str(work_path),
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
                "converged": converged,
                "cost": selected_cost,
                "cost_improvement": improvement,
                "small_improve_streak": small_improve_streak,
                "stop_cost_hit": stop_cost_hit,
                "stop_std_hit": stop_std_hit,
                "warnings": selected_warnings,
            },
        )

        if decision == "accept":
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

        if decision == "stop_warning":
            print(f"; stopping full-auto due to data-quality warnings; dataset={work_path}")
            return _finalize(2)

        cmd = plan.get("collect_command")
        if not isinstance(cmd, list) or not cmd:
            print("; No valid candidate to collect; stopping.")
            return _finalize(2)

        print(f"; collecting next sweep ({selected_id})")
        print(f"; running: {' '.join(str(x) for x in cmd)}")
        subprocess.run(cmd, check=True)
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
        print(f"; merged {collector_output} -> {work_path} sweeps={count}")
        try:
            if collector_output != work_path:
                collector_output.unlink()
        except OSError:
            pass

    print(f"; reached max steps; dataset={work_path}")
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

    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
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
        "--full-auto-log",
        type=Path,
        default=None,
        help="Write full-auto JSONL logs to this path (default: <dataset>.full_auto_log.jsonl).",
    )
    parser.add_argument(
        "--full-auto-cost-eps",
        type=float,
        default=0.05,
        help="Convergence epsilon for noise-normalized cost improvements (default: 0.05).",
    )
    parser.add_argument(
        "--full-auto-converge-k",
        type=int,
        default=2,
        help="Number of consecutive <= epsilon improvements required for convergence (default: 2).",
    )
    parser.add_argument(
        "--full-auto-verbose",
        action="store_true",
        help="Enable verbose solver diagnostics during full-auto runs.",
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
    noise_metrics = _plan_noise_metrics(plan)
    if isinstance(noise_metrics, dict):
        val = noise_metrics.get("normalized_cost")
        if isinstance(val, (int, float)) and np.isfinite(val):
            return float(val)
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


def _full_auto_cfg_path(dataset_path: Path, run_id: str) -> Path:
    return dataset_path.with_name(f"{dataset_path.stem}.active_sweep_cfg.{run_id}.txt")


def _append_jsonl(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload) + "\n")


def ellipse_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_ellipse_parser()
    args = parser.parse_args(argv)
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
            full_auto_runs=args.full_auto_run,
            full_auto_log=args.full_auto_log,
            full_auto_cost_epsilon=float(args.full_auto_cost_eps),
            full_auto_converge_k=int(args.full_auto_converge_k),
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
