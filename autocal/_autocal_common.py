"""Shared helper code for the new modular autocal stack.

This file contains shared helpers extracted from the former autocal monolith,
so the current stack can run independently.
"""

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
from scipy.optimize import minimize, minimize_scalar

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

DEFAULT_RRF_PORT = 8081
RRF_SIM_BINARY = REPO_ROOT / "RRF" / "build" / "rrf_simulator"
RRF_SIM_VSD_PATH = "RRF/run/vsd"
RRF_SIM_DEFAULT_CONFIG = "sys/config_slideprinter.g"
RRF_SIM_LINE_LAYER_CONFIG = "sys/config_slideprinter_w_line_layers.g"
RRF_SIM_HP3_CONFIG = "sys/config_hp3.g"
RRF_SIM_HP3_LINE_LAYER_CONFIG = "sys/config_hp3_w_line_layers.g"
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
_SPOOL_PREFIT_GLOBAL_R_GRID_POINTS = 9
_SPOOL_FIND_MODE_CHOICES = ("off", "global", "per-anchor")
_OPTIMIZER_MODE_CHOICES = ("fast", "fast-fd", "legacy")
_THETA0_MODE_CHOICES = ("infer", "zero")
_SCALE_FIX_LEVELS = (1, 2, 3)
_FIT_STRUCTURE_LEVELS = (1, 2, 3)
_OBJECTIVE_SCHEDULE_LEVELS = (0, 1, 2)
_FILTER_SCHEDULE_PASS_CHOICES = ("warmup", "dynamic", "constant")
_DEFAULT_OBJECTIVE_SCHEDULE = (1, 1, 1, 1)
_DEFAULT_FILTER_SCHEDULE = ("warmup", "warmup", "warmup", "dynamic")
_NOISE_MODEL_CONFIG_KEY = "noise_model"
_TAU_MAD_SCALE = 1.4826
_SCORE_UI_LAYERED_COST_RAW_REF = 0.8
_SCORE_UI_LAYERED_COST_RAW_WEIGHT = 0.20
_SCORE_UI_LAYERED_TAU_MAD_REF_MM = 1.0
_SCORE_UI_LAYERED_TAU_MAD_WEIGHT = 0.15
_SCORE_UI_LAYERED_N_TRIM_REF = 40.0
_SCORE_UI_LAYERED_N_TRIM_WEIGHT = 0.10
_SCORE_UI_LAYERED_TAIL_RATIO_REF = 1.8
_SCORE_UI_LAYERED_TAIL_RATIO_WEIGHT = 0.35
_SCORE_UI_LAYERED_SWEEP_BIAS_REF = 0.12
_SCORE_UI_LAYERED_SWEEP_BIAS_WEIGHT = 0.50
_SCORE_UI_LAYERED_TAIL_FACTOR_REF = 3.0
_SCORE_UI_LAYERED_TAIL_FACTOR_WEIGHT = 0.20
_SCORE_UI_LAYERED_MAP_SCALE = 95.0
_SCORE_UI_LAYERED_MAP_EXP = 5.0
_SCORE_UI_LAYERED_MAP_MULT = 1.0
_SCORE_UI_LAYERED_RISK_BLEND_WEIGHT = 0.5
_SCORE_UI_RANK_OBS_REF = 60.0
_SCORE_UI_RANK_OBS_BONUS_WEIGHT = 0.30
_SCORE_UI_RANK_FILTERED_RATIO_WEIGHT = 0.50
_SCORE_UI_HARD_FAIL = 50.0
_SPOOL_ANCHOR_STEP_IMPROVE_REL = 1e-4
_FULL_AUTO_HISTORY_LOG_BONUS_WEIGHT = 0.45
_FULL_AUTO_HISTORY_LINEAR_PENALTY = 0.12
_FULL_AUTO_HISTORY_COVERAGE_WEIGHT = 0.20
_FULL_AUTO_HISTORY_UI_RANK_POINTS = np.asarray([4.00, 4.10, 4.20, 4.35, 4.55, 4.85, 5.50, 50.0], dtype=float)
_FULL_AUTO_HISTORY_UI_SCORE_POINTS = np.asarray([0.6, 1.0, 1.8, 4.5, 9.0, 15.0, 25.0, 50.0], dtype=float)
_FULL_AUTO_HISTORY_UI_SCORE_LOG_POINTS = np.log1p(_FULL_AUTO_HISTORY_UI_SCORE_POINTS)

from autocal.active_learning import (
    SweepConfig,
    dataset_sweep_configs,
    generate_candidate_sweeps,
    l2_scale_for_machine,
    rank_candidates_d_optimal,
    total_information_matrix,
)
from autocal.calibrate import build_anchor_initial_guess, calibrate_elliptical
from autocal.dataset_roles import normalize_dataset_point_roles as _normalize_dataset_point_roles
from autocal.ellipse_cost import EllipseCostFunction
from autocal.ellipse_fitting import fit_ellipse_from_sweep
from autocal.ellipse_solver import format_anchor_matrix_plain
from autocal.json_schema import append_jsonl_line, load_json_file, write_json_file
from autocal.spool_model import (
    SpoolModelParams,
    build_spool_model_params,
    dataset_with_modeled_lengths,
    sweep_configs_with_modeled_lengths,
    validate_dataset_has_raw_angles,
)
from autocal.sweep_config_generator import generate_sweep_configs, select_representative_configs
from autocal.sweep_types import MachineConfig, MachineType

GeometryWeights = Tuple[float, float, float]
_MACHINE_TYPE_ALIASES = {
    "hp3": "hangprinter_4",
    "hp4": "hangprinter_4",
    "hangprinter_3": "hangprinter_4",
}
MACHINE_TYPE_CHOICES = tuple(machine.value for machine in MachineType)
MACHINE_TYPE_INPUT_CHOICES = tuple(dict.fromkeys((*MACHINE_TYPE_CHOICES, *_MACHINE_TYPE_ALIASES.keys())))
MACHINE_TYPE_CHOICES_STR = " | ".join(MACHINE_TYPE_INPUT_CHOICES)


def _normalize_machine_type(machine_type: Optional[str]) -> Optional[str]:
    if machine_type is None:
        return None
    normalized = str(machine_type).strip().lower()
    if not normalized:
        return normalized
    return _MACHINE_TYPE_ALIASES.get(normalized, normalized)


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
    machine_type = _normalize_machine_type(str(raw_machine_type))
    if machine_type not in MACHINE_TYPE_CHOICES:
        raise ValueError(
            f"{context} machine_type '{machine_type}' is not supported. Expected one of: {MACHINE_TYPE_CHOICES_STR}"
        )
    expected_machine_type = _normalize_machine_type(expected)
    if expected_machine_type is not None and expected_machine_type != machine_type:
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
    for key in (
        "m666_adjusted_by_data_collector",
        "m666_before_data_collection",
        "m666",
        "m666_after",
        "m666_before",
    ):
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
    has_base_radii_override = base_radii_override is not None and len(base_radii_override) > 0

    if has_base_radii_override:
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
            "(use --base-radii or provide m666_adjusted_by_data_collector/m666_before_data_collection R in dataset config)."
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


def _extract_first_finite_float(value: object) -> Optional[float]:
    if isinstance(value, (int, float)):
        out = float(value)
        if np.isfinite(out):
            return out
        return None
    if isinstance(value, (list, tuple)):
        for entry in value:
            if isinstance(entry, (int, float)):
                out = float(entry)
                if np.isfinite(out):
                    return out
    return None


def _resolve_buildup_factor_seed(
    dataset: dict,
    *,
    buildup_factor_override: Optional[float],
) -> float:
    if buildup_factor_override is not None:
        try:
            override_val = float(buildup_factor_override)
        except (TypeError, ValueError):
            override_val = float("nan")
        if np.isfinite(override_val):
            return float(override_val)

    config = dataset.get("config")
    if not isinstance(config, dict):
        return 0.0
    m666 = _extract_m666(config)
    if not isinstance(m666, dict):
        return 0.0
    q_val = _extract_first_finite_float(m666.get("Q"))
    if q_val is None:
        return 0.0
    return float(q_val)


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


def _collector_has_base_radii_override(args: Sequence[str]) -> bool:
    override_flags = {
        "--force-base-radii",
        "--forceBaseRadii",
        "--base-radii",
    }
    for arg in args:
        if not isinstance(arg, str):
            continue
        if arg in override_flags:
            return True
        if (
            arg.startswith("--force-base-radii=")
            or arg.startswith("--forceBaseRadii=")
            or arg.startswith("--base-radii=")
        ):
            return True
    return False


def _format_csv_float_list(values: Sequence[float]) -> str:
    return ",".join(f"{float(v):.12g}" for v in values)


def _inject_spool_collection_args(
    collector_args: Sequence[str],
    *,
    find_radii_mode: str,
    find_buildup_mode: str,
    base_radii: Optional[Sequence[float]] = None,
    buildup_factor: Optional[float] = None,
) -> Tuple[List[str], bool]:
    args = list(collector_args)
    _ = find_radii_mode
    _ = find_buildup_mode
    changed = False

    if base_radii is not None and len(base_radii) > 0 and not _collector_has_base_radii_override(args):
        radii_vals: List[float] = []
        for entry in base_radii:
            try:
                val = float(entry)
            except (TypeError, ValueError):
                continue
            if np.isfinite(val):
                radii_vals.append(float(val))
        if radii_vals:
            args.extend(["--force-base-radii", _format_csv_float_list(radii_vals)])
            changed = True

    if buildup_factor is not None and not _collector_has_buildup_override(args):
        try:
            k_val = float(buildup_factor)
        except (TypeError, ValueError):
            k_val = float("nan")
        if np.isfinite(k_val):
            args.extend(["--force-buildup-factor", f"{k_val:.12g}"])
            changed = True

    return args, changed


def _resolve_sim_config(
    *,
    machine_type: str,
    find_radii_mode: str,
    find_buildup_mode: str,
) -> str:
    env_cfg = os.environ.get("AUTOCAL_RRF_SIM_CONFIG")
    if isinstance(env_cfg, str) and env_cfg.strip():
        return env_cfg.strip()

    machine_type = _normalize_machine_type(machine_type) or ""
    search_spool = _spool_mode_enabled(find_radii_mode) or _spool_mode_enabled(find_buildup_mode)
    if machine_type == "hangprinter_4":
        if search_spool:
            candidate = REPO_ROOT / RRF_SIM_VSD_PATH / RRF_SIM_HP3_LINE_LAYER_CONFIG
            if candidate.exists():
                return RRF_SIM_HP3_LINE_LAYER_CONFIG
        return RRF_SIM_HP3_CONFIG

    if machine_type == "slideprinter" and search_spool:
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
    machine_type = _normalize_machine_type(machine_type) or ""
    if machine_type in ("hangprinter_4", "hangprinter_5"):
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


def _float_or_none(value: object) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(out):
        return None
    return float(out)


def _build_ellipse_cost_function(
    dataset: dict,
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
) -> EllipseCostFunction:
    stage = 2 if bool(pointwise_filtering) else 0
    return EllipseCostFunction(
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


def _compute_tau_mad_rescore_from_rows(
    rows: Sequence[dict],
    *,
    cost_noise_normalized_old: Optional[float],
    chi2_red_old: Optional[float],
    sigma_model_mm: Optional[float],
    params_count: Optional[float] = None,
) -> Dict[str, object]:
    inlier_residuals_tau_mm: List[float] = []
    inlier_cutoff_mm_values: List[float] = []
    slope_x_mm: List[float] = []
    slope_signed_y_mm: List[float] = []
    slope_abs_y_mm: List[float] = []
    slope_sq_y_mm2: List[float] = []
    spread_distance_mm: List[float] = []
    spread_residual_mm_signed: List[float] = []
    spread_residual_z_signed: List[float] = []
    spread_sigma_point_mm: List[float] = []
    inlier_z_by_sweep: Dict[str, List[float]] = {}
    inlier_mm_by_sweep: Dict[str, List[float]] = {}
    inlier_abs_z_by_sweep: Dict[str, List[float]] = {}
    clipped_by_sweep: Dict[str, int] = {}
    total_by_sweep: Dict[str, int] = {}
    total_rows = 0
    for row in rows:
        if not isinstance(row, dict):
            continue
        total_rows += 1
        sweep_id = str(row.get("sweep_id", ""))
        total_by_sweep[sweep_id] = int(total_by_sweep.get(sweep_id, 0)) + 1
        residual_mm = _float_or_none(row.get("residual_mm"))
        if residual_mm is None:
            continue
        cutoff_mm = _float_or_none(row.get("cutoff_mm"))
        clipped = cutoff_mm is not None and residual_mm > (cutoff_mm + 1e-12)
        if clipped:
            clipped_by_sweep[sweep_id] = int(clipped_by_sweep.get(sweep_id, 0)) + 1
            continue
        clipped_by_sweep.setdefault(sweep_id, 0)
        if cutoff_mm is not None and np.isfinite(cutoff_mm):
            inlier_cutoff_mm_values.append(float(cutoff_mm))
        residual_z_signed = _float_or_none(row.get("residual_z_signed"))
        if residual_z_signed is None:
            residual_z_signed = _float_or_none(row.get("residual_z"))
            if residual_z_signed is not None:
                residual_z_signed = float(abs(residual_z_signed))
        if residual_z_signed is not None and np.isfinite(residual_z_signed):
            inlier_z_by_sweep.setdefault(sweep_id, []).append(float(residual_z_signed))
            inlier_abs_z_by_sweep.setdefault(sweep_id, []).append(float(abs(residual_z_signed)))
        residual_mm_signed = _float_or_none(row.get("residual_mm_signed"))
        if residual_mm_signed is None:
            residual_mm_signed = float(residual_mm)
        inlier_mm_by_sweep.setdefault(sweep_id, []).append(float(residual_mm_signed))
        if residual_z_signed is None:
            sigma_model_local = _float_or_none(sigma_model_mm)
            if sigma_model_local is not None and sigma_model_local > 0.0:
                inlier_z_by_sweep.setdefault(sweep_id, []).append(float(residual_mm_signed / sigma_model_local))
                inlier_abs_z_by_sweep.setdefault(sweep_id, []).append(
                    float(abs(residual_mm_signed / sigma_model_local))
                )
        if residual_mm_signed is None:
            continue
        inlier_residuals_tau_mm.append(float(residual_mm_signed))
        l_drive_mm = _float_or_none(row.get("l_drive_mm"))
        l_sensor_mm = _float_or_none(row.get("l_sensor_mm"))
        if l_drive_mm is None or l_sensor_mm is None:
            continue
        distance_mm = 0.5 * (float(l_drive_mm) + float(l_sensor_mm))
        if np.isfinite(distance_mm):
            slope_x_mm.append(float(distance_mm))
            slope_signed_y_mm.append(float(residual_mm_signed))
            slope_abs_y_mm.append(float(abs(residual_mm)))
            slope_sq_y_mm2.append(float(residual_mm_signed * residual_mm_signed))
            spread_distance_mm.append(float(distance_mm))
            spread_residual_mm_signed.append(float(residual_mm_signed))
            z_for_tau = residual_z_signed
            if z_for_tau is None:
                sigma_model_local = _float_or_none(sigma_model_mm)
                if sigma_model_local is not None and sigma_model_local > 0.0:
                    z_for_tau = float(residual_mm_signed / sigma_model_local)
            sigma_point = None
            if z_for_tau is not None and np.isfinite(z_for_tau) and abs(float(z_for_tau)) > 1e-12:
                sigma_point_val = float(abs(float(residual_mm_signed) / float(z_for_tau)))
                if np.isfinite(sigma_point_val) and sigma_point_val > 0.0:
                    sigma_point = sigma_point_val
            if sigma_point is None:
                sigma_model_local = _float_or_none(sigma_model_mm)
                if sigma_model_local is not None and sigma_model_local > 0.0:
                    sigma_point = float(sigma_model_local)
            spread_residual_z_signed.append(float(z_for_tau) if z_for_tau is not None and np.isfinite(z_for_tau) else float("nan"))
            spread_sigma_point_mm.append(float(sigma_point) if sigma_point is not None else float("nan"))

    n_inlier_rows_used_for_tau = int(len(inlier_residuals_tau_mm))
    inlier_cutoff_mm = None
    if inlier_cutoff_mm_values:
        cutoff_arr = np.asarray(inlier_cutoff_mm_values, dtype=float)
        finite_cutoff = cutoff_arr[np.isfinite(cutoff_arr)]
        if finite_cutoff.size:
            inlier_cutoff_mm = float(np.median(finite_cutoff))

    def _linear_slope(x_vals: Sequence[float], y_vals: Sequence[float]) -> Optional[float]:
        if len(x_vals) < 2 or len(x_vals) != len(y_vals):
            return None
        x = np.asarray(x_vals, dtype=float)
        y = np.asarray(y_vals, dtype=float)
        finite = np.isfinite(x) & np.isfinite(y)
        if int(np.sum(finite)) < 2:
            return None
        x = x[finite]
        y = y[finite]
        x_centered = x - float(np.mean(x))
        denom = float(np.dot(x_centered, x_centered))
        if not np.isfinite(denom) or denom <= 0.0:
            return None
        y_centered = y - float(np.mean(y))
        slope = float(np.dot(x_centered, y_centered) / denom)
        return float(slope) if np.isfinite(slope) else None

    def _robust_line_fit(x_vals: np.ndarray, y_vals: np.ndarray) -> Tuple[Optional[float], Optional[float]]:
        x = np.asarray(x_vals, dtype=float).ravel()
        y = np.asarray(y_vals, dtype=float).ravel()
        finite = np.isfinite(x) & np.isfinite(y)
        if int(np.sum(finite)) <= 0:
            return None, None
        x = x[finite]
        y = y[finite]
        if x.size == 1:
            return float(y[0]), 0.0
        x_design = np.column_stack([np.ones_like(x), x])
        try:
            beta = np.linalg.lstsq(x_design, y, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None, None
        if beta.size != 2 or not np.all(np.isfinite(beta)):
            return None, None
        for _ in range(6):
            fitted = x_design @ beta
            resid = y - fitted
            center = float(np.median(resid))
            mad = float(np.median(np.abs(resid - center)))
            scale = float(_TAU_MAD_SCALE * mad)
            if not np.isfinite(scale) or scale <= 1e-12:
                break
            cutoff = float(1.345 * scale)
            abs_resid = np.abs(resid)
            weights = np.ones_like(abs_resid)
            heavy = abs_resid > cutoff
            weights[heavy] = cutoff / np.maximum(abs_resid[heavy], 1e-12)
            xw = x_design * weights[:, None]
            yw = y * weights
            try:
                beta_new = np.linalg.lstsq(xw, yw, rcond=None)[0]
            except np.linalg.LinAlgError:
                break
            if beta_new.size != 2 or not np.all(np.isfinite(beta_new)):
                break
            if np.linalg.norm(beta_new - beta) <= 1e-9 * max(1.0, float(np.linalg.norm(beta))):
                beta = beta_new
                break
            beta = beta_new
        return float(beta[0]), float(beta[1])

    tau_mad_mm = None
    if inlier_residuals_tau_mm:
        residual_arr = np.asarray(inlier_residuals_tau_mm, dtype=float)
        center = float(np.median(residual_arr))
        tau_val = float(_TAU_MAD_SCALE * np.median(np.abs(residual_arr - center)))
        if np.isfinite(tau_val) and tau_val >= 0.0:
            tau_mad_mm = float(tau_val)

    residual_vs_distance_slope_inliers = _linear_slope(slope_x_mm, slope_signed_y_mm)
    residual_abs_vs_distance_slope = _linear_slope(slope_x_mm, slope_abs_y_mm)
    residual_sq_vs_distance_slope = _linear_slope(slope_x_mm, slope_sq_y_mm2)

    distance_bin_mad_mm: Dict[str, Optional[float]] = {"near": None, "mid": None, "far": None}
    distance_bin_madn_mm: Dict[str, Optional[float]] = {"near": None, "mid": None, "far": None}
    distance_bin_counts: Dict[str, int] = {"near": 0, "mid": 0, "far": 0}
    distance_bin_mad_debiased_mm: Dict[str, Optional[float]] = {"near": None, "mid": None, "far": None}
    distance_bin_madn_debiased_mm: Dict[str, Optional[float]] = {"near": None, "mid": None, "far": None}
    tau_d_fit_centers: List[float] = []
    tau_d_fit_spreads: List[float] = []
    d_arr = np.zeros(0, dtype=float)
    r_arr = np.zeros(0, dtype=float)
    z_arr = np.zeros(0, dtype=float)
    sigma_arr = np.zeros(0, dtype=float)
    bin_masks: Dict[str, np.ndarray] = {
        "near": np.zeros(0, dtype=bool),
        "mid": np.zeros(0, dtype=bool),
        "far": np.zeros(0, dtype=bool),
    }
    if spread_distance_mm and spread_residual_mm_signed:
        d_full = np.asarray(spread_distance_mm, dtype=float)
        r_full = np.asarray(spread_residual_mm_signed, dtype=float)
        z_full = np.asarray(spread_residual_z_signed, dtype=float)
        sigma_full = np.asarray(spread_sigma_point_mm, dtype=float)
        finite = np.isfinite(d_full) & np.isfinite(r_full)
        if int(np.sum(finite)) >= 1:
            d_arr = d_full[finite]
            r_arr = r_full[finite]
            z_arr = z_full[finite]
            sigma_arr = sigma_full[finite]
            q1, q2 = np.quantile(d_arr, [1.0 / 3.0, 2.0 / 3.0])
            bin_masks = {
                "near": d_arr <= q1,
                "mid": (d_arr > q1) & (d_arr <= q2),
                "far": d_arr > q2,
            }
            for key in ("near", "mid", "far"):
                mask = np.asarray(bin_masks[key], dtype=bool)
                count = int(np.sum(mask))
                distance_bin_counts[key] = count
                if count <= 0:
                    continue
                r_bin = r_arr[mask]
                med_bin = float(np.median(r_bin))
                mad_bin = float(np.median(np.abs(r_bin - med_bin)))
                madn_bin = float(_TAU_MAD_SCALE * mad_bin)
                if np.isfinite(mad_bin):
                    distance_bin_mad_mm[key] = mad_bin
                if np.isfinite(madn_bin):
                    distance_bin_madn_mm[key] = madn_bin
                if count >= 2 and np.isfinite(madn_bin):
                    d_center = float(np.median(d_arr[mask]))
                    if np.isfinite(d_center):
                        tau_d_fit_centers.append(d_center)
                        tau_d_fit_spreads.append(madn_bin)

    bias_vs_distance_intercept_mm = None
    bias_vs_distance_slope_inliers = None
    cost_after_bias_diagnostic = None
    chi2_red_after_bias_diagnostic = None
    r_debiased = np.zeros(0, dtype=float)
    z_debiased = np.zeros(0, dtype=float)
    if d_arr.size and r_arr.size == d_arr.size:
        intercept_fit, slope_fit = _robust_line_fit(d_arr, r_arr)
        if intercept_fit is not None and slope_fit is not None:
            bias_vs_distance_intercept_mm = float(intercept_fit)
            bias_vs_distance_slope_inliers = float(slope_fit)
            r_debiased = r_arr - (
                float(bias_vs_distance_intercept_mm) + (float(bias_vs_distance_slope_inliers) * d_arr)
            )
            sigma_eff = np.asarray(sigma_arr, dtype=float)
            valid_sigma = np.isfinite(sigma_eff) & (sigma_eff > 0.0)
            sigma_model_local = _float_or_none(sigma_model_mm)
            if sigma_model_local is not None and sigma_model_local > 0.0:
                sigma_eff = np.where(valid_sigma, sigma_eff, float(sigma_model_local))
                valid_sigma = np.isfinite(sigma_eff) & (sigma_eff > 0.0)
            z_debiased = np.full_like(r_debiased, np.nan, dtype=float)
            if int(np.sum(valid_sigma)) > 0:
                z_debiased[valid_sigma] = r_debiased[valid_sigma] / sigma_eff[valid_sigma]
                z_fin = z_debiased[np.isfinite(z_debiased)]
                if z_fin.size:
                    cost_after_bias_diagnostic = float(np.mean(z_fin**2))
                    p_count = _float_or_none(params_count)
                    if p_count is None:
                        p_count = 0.0
                    dof = int(z_fin.size - int(p_count))
                    if dof > 0:
                        chi2_red_after_bias_diagnostic = float(np.sum(z_fin**2) / float(dof))

    tau_bin_debiased_mm: Dict[str, Optional[float]] = {"near": None, "mid": None, "far": None}
    cost_rescored_tau_3bin_debiased = None
    chi2_rescored_tau_3bin_debiased = None
    tau_3bin_debiased_rescore_scale = None
    if r_debiased.size and d_arr.size == r_debiased.size:
        for key in ("near", "mid", "far"):
            mask = np.asarray(bin_masks.get(key), dtype=bool)
            if mask.size != r_debiased.size:
                continue
            count = int(np.sum(mask))
            if count <= 0:
                continue
            r_bin = r_debiased[mask]
            med_bin = float(np.median(r_bin))
            mad_bin = float(np.median(np.abs(r_bin - med_bin)))
            madn_bin = float(_TAU_MAD_SCALE * mad_bin)
            if np.isfinite(mad_bin):
                distance_bin_mad_debiased_mm[key] = mad_bin
            if np.isfinite(madn_bin):
                distance_bin_madn_debiased_mm[key] = madn_bin
                tau_bin_debiased_mm[key] = madn_bin

        sigma_model_local = _float_or_none(sigma_model_mm)
        if sigma_model_local is not None and sigma_model_local > 0.0 and z_debiased.size == r_debiased.size:
            tau_global_debiased = None
            if np.any(np.isfinite(r_debiased)):
                center = float(np.median(r_debiased[np.isfinite(r_debiased)]))
                mad = float(np.median(np.abs(r_debiased[np.isfinite(r_debiased)] - center)))
                tau_val = float(_TAU_MAD_SCALE * mad)
                if np.isfinite(tau_val) and tau_val >= 0.0:
                    tau_global_debiased = tau_val
            tau_point = np.full_like(r_debiased, np.nan, dtype=float)
            for key in ("near", "mid", "far"):
                mask = np.asarray(bin_masks.get(key), dtype=bool)
                tau_val = _float_or_none(tau_bin_debiased_mm.get(key))
                if mask.size != tau_point.size:
                    continue
                if tau_val is not None and tau_val >= 0.0:
                    tau_point[mask] = float(tau_val)
            if tau_global_debiased is not None:
                tau_point = np.where(np.isfinite(tau_point), tau_point, float(tau_global_debiased))
            valid = np.isfinite(tau_point) & (tau_point >= 0.0) & np.isfinite(z_debiased)
            if int(np.sum(valid)) > 0:
                sigma_eff_sq = (float(sigma_model_local) * float(sigma_model_local)) + (tau_point[valid] ** 2.0)
                good = np.isfinite(sigma_eff_sq) & (sigma_eff_sq > 0.0)
                if int(np.sum(good)) > 0:
                    scale_arr = (float(sigma_model_local) * float(sigma_model_local)) / sigma_eff_sq[good]
                    z2 = (z_debiased[valid][good]) ** 2.0
                    weighted = z2 * scale_arr
                    if weighted.size:
                        cost_rescored_tau_3bin_debiased = float(np.mean(weighted))
                        if cost_after_bias_diagnostic is not None and cost_after_bias_diagnostic > 0.0:
                            tau_3bin_debiased_rescore_scale = float(
                                cost_rescored_tau_3bin_debiased / float(cost_after_bias_diagnostic)
                            )
                        p_count = _float_or_none(params_count)
                        if p_count is None:
                            p_count = 0.0
                        dof = int(weighted.size - int(p_count))
                        if dof > 0:
                            chi2_rescored_tau_3bin_debiased = float(np.sum(weighted) / float(dof))

    per_sweep_summary: Dict[str, Dict[str, object]] = {}
    per_sweep_bias_medians: Dict[str, float] = {}
    per_sweep_medians_mm: List[float] = []
    per_sweep_mads_mm: List[float] = []
    per_sweep_tail_factors: List[float] = []
    demeaned_mm: List[float] = []
    demeaned_z: List[float] = []
    sweep_ids = sorted(set(total_by_sweep.keys()) | set(inlier_mm_by_sweep.keys()) | set(inlier_z_by_sweep.keys()))
    for sweep_id in sweep_ids:
        resid_mm = np.asarray(inlier_mm_by_sweep.get(sweep_id, []), dtype=float)
        resid_z = np.asarray(inlier_z_by_sweep.get(sweep_id, []), dtype=float)
        resid_abs_z = np.asarray(inlier_abs_z_by_sweep.get(sweep_id, []), dtype=float)
        median_mm = None
        mad_mm = None
        p95_abs_mm = None
        mean_abs_z = None
        tail_factor = None
        if resid_mm.size:
            median_val = float(np.median(resid_mm))
            median_mm = median_val
            per_sweep_bias_medians[sweep_id] = float(median_val)
            if np.isfinite(median_val):
                per_sweep_medians_mm.append(float(median_val))
            mad_val = float(np.median(np.abs(resid_mm - median_val)))
            if np.isfinite(mad_val):
                mad_mm = mad_val
                if mad_val > 0.0:
                    per_sweep_mads_mm.append(float(mad_val))
            p95_val = float(np.percentile(np.abs(resid_mm), 95))
            if np.isfinite(p95_val):
                p95_abs_mm = p95_val
            if (
                p95_abs_mm is not None
                and mad_mm is not None
                and np.isfinite(float(p95_abs_mm))
                and np.isfinite(float(mad_mm))
                and float(mad_mm) > 0.0
            ):
                tail_factor = float(float(p95_abs_mm) / float(mad_mm))
                if np.isfinite(tail_factor):
                    per_sweep_tail_factors.append(float(tail_factor))
            demeaned_mm.extend((resid_mm - median_val).tolist())
        if resid_z.size:
            median_z = float(np.median(resid_z))
            demeaned_z.extend((resid_z - median_z).tolist())
        if resid_abs_z.size:
            mean_abs_z_val = float(np.mean(resid_abs_z))
            if np.isfinite(mean_abs_z_val):
                mean_abs_z = mean_abs_z_val
        per_sweep_summary[sweep_id] = {
            "median_residual_mm": median_mm,
            "mad_residual_mm": mad_mm,
            "p95_abs_residual_mm": p95_abs_mm,
            "mean_abs_z": mean_abs_z,
            "tail_factor": tail_factor,
            "clipped_points": int(clipped_by_sweep.get(sweep_id, 0)),
            "inlier_points": int(resid_mm.size),
            "total_points": int(total_by_sweep.get(sweep_id, 0)),
        }

    per_sweep_demean: Dict[str, object] = {}
    if demeaned_mm:
        demeaned_mm_arr = np.asarray(demeaned_mm, dtype=float)
        med_demeaned = float(np.median(demeaned_mm_arr))
        mad_abs_raw = float(np.median(np.abs(demeaned_mm_arr - med_demeaned)))
        robust_scale_mm = float(_TAU_MAD_SCALE * mad_abs_raw)
        p95_abs_demeaned = float(np.percentile(np.abs(demeaned_mm_arr), 95))
        eps = float(np.finfo(float).eps)
        denom = robust_scale_mm + eps
        if np.isfinite(denom) and denom > 0.0:
            per_sweep_demean["tail_ratio"] = float(p95_abs_demeaned / denom)
        per_sweep_demean["p95_abs_residual_mm"] = p95_abs_demeaned
        per_sweep_demean["mad_abs_residual_mm"] = mad_abs_raw
        per_sweep_demean["madn_residual_mm"] = robust_scale_mm
    eps = float(np.finfo(float).eps)
    if per_sweep_medians_mm and per_sweep_mads_mm:
        medians_arr = np.asarray(per_sweep_medians_mm, dtype=float)
        mads_arr = np.asarray(per_sweep_mads_mm, dtype=float)
        med_mad = float(np.median(mads_arr))
        if np.isfinite(med_mad) and med_mad > 0.0:
            normalized_sweep_bias = float(abs(float(np.mean(medians_arr))) / (med_mad + eps))
            if np.isfinite(normalized_sweep_bias):
                per_sweep_demean["normalized_sweep_bias"] = float(normalized_sweep_bias)
    if per_sweep_tail_factors:
        tail_factor_median = float(np.median(np.asarray(per_sweep_tail_factors, dtype=float)))
        if np.isfinite(tail_factor_median):
            per_sweep_demean["tail_factor_median"] = float(tail_factor_median)
    if len(per_sweep_bias_medians) >= 2:
        medians = np.asarray(list(per_sweep_bias_medians.values()), dtype=float)
        if medians.size:
            per_sweep_demean["sweep_bias_span_mm"] = float(np.max(medians) - np.min(medians))
    if demeaned_z:
        demeaned_z_arr = np.asarray(demeaned_z, dtype=float)
        finite = demeaned_z_arr[np.isfinite(demeaned_z_arr)]
        if finite.size:
            cost_demeaned = float(np.mean(finite**2))
            per_sweep_demean["cost_noise_normalized_demeaned"] = cost_demeaned
            p_count = _float_or_none(params_count)
            if p_count is None:
                p_count = 0.0
            dof = int(finite.size - int(p_count))
            if dof > 0:
                per_sweep_demean["chi2_red_demeaned"] = float(np.sum(finite**2) / float(dof))
            per_sweep_demean["n_obs_demeaned"] = int(finite.size)

    cost_old = _float_or_none(cost_noise_normalized_old)
    chi2_old = _float_or_none(chi2_red_old)
    sigma_model = _float_or_none(sigma_model_mm)
    sigma_eff_mm = None
    rescore_scale = None
    cost_rescored = None
    chi2_rescored = None
    tau_d_tau0_mm = None
    tau_d_tau1_per_mm = None
    tau_d_rescore_scale = None
    cost_rescored_tau_d = None
    chi2_rescored_tau_d = None
    cost_rescored_tau_d_trimmed_direct = None
    chi2_rescored_tau_d_trimmed_direct = None
    if sigma_model is not None and sigma_model > 0.0 and tau_mad_mm is not None:
        sigma_eff_val = float(np.sqrt((sigma_model * sigma_model) + (tau_mad_mm * tau_mad_mm)))
        if np.isfinite(sigma_eff_val) and sigma_eff_val > 0.0:
            sigma_eff_mm = float(sigma_eff_val)
            scale = float((sigma_model * sigma_model) / (sigma_eff_val * sigma_eff_val))
            if np.isfinite(scale) and scale > 0.0:
                rescore_scale = float(scale)
                if cost_old is not None:
                    cost_rescored = float(cost_old * scale)
                if chi2_old is not None:
                    chi2_rescored = float(chi2_old * scale)

    if sigma_model is not None and sigma_model > 0.0:
        tau0_sq = None
        tau1_sq = None
        if len(tau_d_fit_centers) >= 2 and len(tau_d_fit_centers) == len(tau_d_fit_spreads):
            x = np.asarray(tau_d_fit_centers, dtype=float) ** 2.0
            y = np.asarray(tau_d_fit_spreads, dtype=float) ** 2.0
            finite = np.isfinite(x) & np.isfinite(y)
            if int(np.sum(finite)) >= 2:
                x = x[finite]
                y = y[finite]
                design = np.column_stack([np.ones_like(x), x])
                coeffs, *_ = np.linalg.lstsq(design, y, rcond=None)
                tau0_sq = float(max(float(coeffs[0]), 0.0))
                tau1_sq = float(max(float(coeffs[1]), 0.0))
        if tau0_sq is None or tau1_sq is None:
            if tau_mad_mm is not None and np.isfinite(tau_mad_mm):
                tau0_sq = float(max(tau_mad_mm * tau_mad_mm, 0.0))
                tau1_sq = 0.0
        if tau0_sq is not None and tau1_sq is not None:
            tau0 = float(np.sqrt(max(tau0_sq, 0.0)))
            tau1 = float(np.sqrt(max(tau1_sq, 0.0)))
            if np.isfinite(tau0) and np.isfinite(tau1):
                tau_d_tau0_mm = tau0
                tau_d_tau1_per_mm = tau1
                if d_arr.size:
                    tau_d_sq = (tau0 * tau0) + ((tau1 * d_arr) ** 2.0)
                    sigma_eff_d_sq = (sigma_model * sigma_model) + tau_d_sq
                    valid_sigma = np.isfinite(sigma_eff_d_sq) & (sigma_eff_d_sq > 0.0)
                    if int(np.sum(valid_sigma)) > 0:
                        scale_arr = np.zeros_like(sigma_eff_d_sq)
                        scale_arr[valid_sigma] = (sigma_model * sigma_model) / sigma_eff_d_sq[valid_sigma]
                        scale_arr = scale_arr[valid_sigma]
                        if scale_arr.size:
                            ratio = float(np.mean(scale_arr))
                            if z_arr.size == d_arr.size:
                                z2 = (z_arr[valid_sigma]) ** 2.0
                                finite_z = np.isfinite(z2)
                                if int(np.sum(finite_z)) > 0:
                                    z2 = z2[finite_z]
                                    scale_use = scale_arr[finite_z]
                                    weighted = z2 * scale_use
                                    if weighted.size:
                                        cost_rescored_tau_d_trimmed_direct = float(np.mean(weighted))
                                        p_count = _float_or_none(params_count)
                                        if p_count is None:
                                            p_count = 0.0
                                        dof = int(weighted.size - int(p_count))
                                        if dof > 0:
                                            chi2_rescored_tau_d_trimmed_direct = float(np.sum(weighted) / float(dof))
                                    base = float(np.mean(z2))
                                    if np.isfinite(base) and base > 0.0:
                                        ratio = float(np.mean(weighted) / base)
                            if np.isfinite(ratio) and ratio > 0.0:
                                tau_d_rescore_scale = ratio
                                if cost_old is not None:
                                    cost_rescored_tau_d = float(cost_old * ratio)
                                if chi2_old is not None:
                                    chi2_rescored_tau_d = float(chi2_old * ratio)

    trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased = None
    if (
        chi2_rescored_tau_d_trimmed_direct is not None
        and chi2_rescored_tau_3bin_debiased is not None
        and np.isfinite(chi2_rescored_tau_d_trimmed_direct)
        and np.isfinite(chi2_rescored_tau_3bin_debiased)
        and abs(float(chi2_rescored_tau_3bin_debiased)) > 0.0
    ):
        trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased = float(
            float(chi2_rescored_tau_d_trimmed_direct) / float(chi2_rescored_tau_3bin_debiased)
        )

    return {
        "tau_mad_mm": tau_mad_mm,
        "tau_mad_inlier_points": int(len(inlier_residuals_tau_mm)),
        "tau_mad_total_points": int(total_rows),
        "inlier_cutoff_mm": inlier_cutoff_mm,
        "n_inlier_rows_used_for_tau": n_inlier_rows_used_for_tau,
        "cost_noise_normalized_old": cost_old,
        "cost_noise_normalized_rescored": cost_rescored,
        "chi2_red_old": chi2_old,
        "chi2_red_rescored": chi2_rescored,
        "sigma_model_rescore_mm": sigma_model,
        "sigma_eff_mm": sigma_eff_mm,
        "noise_rescore_scale": rescore_scale,
        "residual_vs_distance_slope_inliers": residual_vs_distance_slope_inliers,
        "residual_vs_distance_slope": residual_vs_distance_slope_inliers,
        "residual_abs_vs_distance_slope": residual_abs_vs_distance_slope,
        "residual_sq_vs_distance_slope": residual_sq_vs_distance_slope,
        "distance_bin_mad_mm": distance_bin_mad_mm,
        "distance_bin_madn_mm": distance_bin_madn_mm,
        "distance_bin_mad_debiased_mm": distance_bin_mad_debiased_mm,
        "distance_bin_madn_debiased_mm": distance_bin_madn_debiased_mm,
        "distance_bin_counts": distance_bin_counts,
        "bias_vs_distance_intercept_mm": bias_vs_distance_intercept_mm,
        "bias_vs_distance_slope_inliers": bias_vs_distance_slope_inliers,
        "bias_vs_distance_slope_mm_per_mm": bias_vs_distance_slope_inliers,
        "cost_after_bias_diagnostic": cost_after_bias_diagnostic,
        "chi2_red_after_bias_diagnostic": chi2_red_after_bias_diagnostic,
        "tau_d_tau0_mm": tau_d_tau0_mm,
        "tau_d_tau1_per_mm": tau_d_tau1_per_mm,
        "tau_d_rescore_scale": tau_d_rescore_scale,
        "cost_noise_normalized_tau_d": cost_rescored_tau_d,
        "chi2_red_tau_d": chi2_rescored_tau_d,
        "cost_noise_normalized_rescored_tau_d": cost_rescored_tau_d,
        "chi2_red_rescored_tau_d": chi2_rescored_tau_d,
        "cost_noise_normalized_tau_d_trimmed_direct": cost_rescored_tau_d_trimmed_direct,
        "chi2_red_tau_d_trimmed_direct": chi2_rescored_tau_d_trimmed_direct,
        "tau_3bin_debiased_mm": tau_bin_debiased_mm,
        "tau_3bin_debiased_rescore_scale": tau_3bin_debiased_rescore_scale,
        "chi2_red_tau_3bin_debiased_trimmed_direct": chi2_rescored_tau_3bin_debiased,
        "cost_noise_normalized_rescored_tau_3bin_debiased": cost_rescored_tau_3bin_debiased,
        "chi2_red_rescored_tau_3bin_debiased": chi2_rescored_tau_3bin_debiased,
        "trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased": trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased,
        "tail_ratio": per_sweep_demean.get("tail_ratio"),
        "normalized_sweep_bias": per_sweep_demean.get("normalized_sweep_bias"),
        "tail_factor_median": per_sweep_demean.get("tail_factor_median"),
        "per_sweep_residual_summary": per_sweep_summary,
        "per_sweep_bias_medians_mm": per_sweep_bias_medians,
        "per_sweep_demean": per_sweep_demean,
    }


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
    cost_fn = _build_ellipse_cost_function(
        dataset,
        residual_threshold=float(residual_threshold),
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        pointwise_residual_mode=str(pointwise_residual_mode),
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        use_noise_mean=bool(use_noise_mean),
        noise_normalized=bool(noise_normalized),
        sigma_source=str(sigma_source),
    )
    anchor_vec = np.asarray(anchors, dtype=float).ravel()
    return float(cost_fn.evaluate(anchor_vec))


_DEFAULT_EVALUATE_COST_AT_ANCHORS = _evaluate_cost_at_anchors


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
        else:
            updated = dict(base_config)
            for key in (
                "m666_adjusted_by_data_collector",
                "m666_before_data_collection",
                "m666",
                "m666_after",
                "m666_before",
                "m669",
                "m92",
                "mm_per_degree",
                "notes",
            ):
                if key in new_config:
                    updated[key] = new_config.get(key)
            if "force_tuning" in new_config or "torque_tuning" in new_config:
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


def _write_bootstrap_sweep_config(
    path: Path,
    *,
    machine_type: str,
    max_sweeps: int = 3,
) -> int:
    machine_type = _normalize_machine_type(machine_type) or str(machine_type)
    config = MachineConfig.from_type(MachineType(str(machine_type)))
    all_configs = generate_sweep_configs(config)
    selected = select_representative_configs(
        all_configs,
        config,
        max_sweeps=max(1, int(max_sweeps)),
    )
    if not selected:
        raise ValueError(f"Unable to generate bootstrap sweep config for machine type '{machine_type}'")
    lines = []
    for cfg in selected:
        fixed = ",".join(str(int(v)) for v in cfg["fixed_anchors"])
        lines.append(f"[{fixed}] {int(cfg['drive_anchor'])} {int(cfg['sensor_anchor'])}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(lines)


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


def _normalize_optimizer_mode(mode: Optional[str]) -> str:
    text = str(mode or "fast").strip().lower()
    if text not in _OPTIMIZER_MODE_CHOICES:
        raise ValueError(
            f"invalid optimizer mode '{mode}', expected one of: {', '.join(_OPTIMIZER_MODE_CHOICES)}"
        )
    return text


def _apply_optimizer_mode_env(mode: Optional[str]) -> str:
    """Map CLI optimizer mode to solver/JAX environment settings."""
    mode_norm = _normalize_optimizer_mode(mode)
    os.environ["AUTOCAL_OPTIMIZER_MODE"] = mode_norm
    if mode_norm == "legacy":
        os.environ["AUTOCAL_DISABLE_JAX_OBJECTIVE"] = "1"
        os.environ["AUTOCAL_JAX_LBFGSB_MODE"] = "fun"
    elif mode_norm == "fast-fd":
        os.environ.pop("AUTOCAL_DISABLE_JAX_OBJECTIVE", None)
        os.environ["AUTOCAL_JAX_LBFGSB_MODE"] = "fun"
    else:
        os.environ.pop("AUTOCAL_DISABLE_JAX_OBJECTIVE", None)
        os.environ["AUTOCAL_JAX_LBFGSB_MODE"] = "jac"
    return mode_norm


def _parse_scale_fix_levels(spec: Optional[Any], *, label: str = "--scale-fix") -> Tuple[int, ...]:
    if spec is None:
        return tuple()
    if isinstance(spec, (list, tuple, set)):
        parts = [str(v).strip() for v in spec]
    else:
        text = str(spec).strip()
        if not text:
            return tuple()
        if text.lower() in ("off", "none", "false", "0"):
            return tuple()
        parts = [p.strip() for p in text.split(",")]
    out: List[int] = []
    for part in parts:
        if not part:
            continue
        try:
            val = int(part)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must be a comma-separated list of integers: 1,2,3") from exc
        if val not in _SCALE_FIX_LEVELS:
            allowed = ",".join(str(v) for v in _SCALE_FIX_LEVELS)
            raise ValueError(f"{label} allows only: {allowed}")
        if val not in out:
            out.append(int(val))
    return tuple(out)


def _parse_fit_structure_levels(
    spec: Optional[Any],
    *,
    label: str = "--fit-structure",
) -> Tuple[int, ...]:
    if spec is None:
        return tuple()
    if isinstance(spec, (list, tuple, set)):
        parts = [str(v).strip() for v in spec]
    else:
        text = str(spec).strip()
        if not text:
            return tuple()
        if text.lower() in ("off", "none", "false", "0"):
            return tuple()
        parts = [p.strip() for p in text.split(",")]
    out: List[int] = []
    for part in parts:
        if not part:
            continue
        try:
            val = int(part)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must be a comma-separated list of integers: 1,2,3") from exc
        if val not in _FIT_STRUCTURE_LEVELS:
            allowed = ",".join(str(v) for v in _FIT_STRUCTURE_LEVELS)
            raise ValueError(f"{label} allows only: {allowed}")
        if val not in out:
            out.append(int(val))
    return tuple(out)


def _normalize_objective_schedule_pass(
    pass_name: object,
    *,
    label: str = "--objective-schedule",
) -> int:
    text = str(pass_name or "").strip().lower()
    mapping = {
        "0": 0,
        "prefit": 0,
        "ellipse": 0,
        "ellipse_prefit": 0,
        "1": 1,
        "pointwise": 1,
        "ellipse_cost": 1,
        "pointwise_forward_model": 1,
        "2": 2,
        "position": 2,
        "sim": 2,
        "simulation": 2,
        "position_reconstruction": 2,
    }
    normalized = mapping.get(text)
    if normalized is None:
        allowed = "0,1,2 or prefit,pointwise,position"
        raise ValueError(f"{label} allows only {allowed}; got '{pass_name}'")
    return int(normalized)


def _parse_objective_schedule(
    spec: Optional[Any],
    *,
    label: str = "--objective-schedule",
) -> Tuple[int, ...]:
    if spec is None:
        return tuple(int(v) for v in _DEFAULT_OBJECTIVE_SCHEDULE)

    if isinstance(spec, (list, tuple, set)):
        parts = [str(v).strip() for v in spec]
    else:
        text = str(spec).strip()
        if not text:
            raise ValueError(
                f"{label} must list one or more passes. Example: 1,1,1,1"
            )
        parts = [p.strip() for p in text.split(",")]

    out: List[int] = []
    for raw_part in parts:
        if not raw_part:
            continue
        out.append(int(_normalize_objective_schedule_pass(raw_part, label=label)))

    if not out:
        raise ValueError(
            f"{label} must list one or more passes. Example: 1,1,1,1"
        )
    return tuple(out)


def _normalize_filter_schedule_pass(pass_name: object, *, label: str = "--filter-schedule") -> str:
    text = str(pass_name or "").strip().lower()
    mapping = {
        "0": "warmup",
        "warmup": "warmup",
        "1": "dynamic",
        "dynamic": "dynamic",
        "2": "constant",
        "constant": "constant",
    }
    normalized = mapping.get(text)
    if normalized is None:
        allowed = "warmup,dynamic,constant or 0,1,2"
        raise ValueError(f"{label} allows only {allowed}; got '{pass_name}'")
    return str(normalized)


def _parse_filter_schedule(
    spec: Optional[Any],
    *,
    label: str = "--filter-schedule",
) -> Tuple[str, ...]:
    if spec is None:
        return tuple(str(v) for v in _DEFAULT_FILTER_SCHEDULE)

    if isinstance(spec, (list, tuple, set)):
        parts = [str(v).strip() for v in spec]
    else:
        text = str(spec).strip()
        if not text:
            raise ValueError(
                f"{label} must list one or more passes. Example: warmup,warmup,warmup,dynamic"
            )
        parts = [p.strip() for p in text.split(",")]

    out: List[str] = []
    mask_ready = False
    for idx, raw_part in enumerate(parts, start=1):
        if not raw_part:
            continue
        pass_name = _normalize_filter_schedule_pass(raw_part, label=label)
        if pass_name == "warmup":
            mask_ready = False
        elif pass_name == "dynamic":
            mask_ready = True
        elif not mask_ready:
            raise ValueError(
                f"{label} pass {idx} cannot be constant without a prior dynamic pass "
                "since the most recent warmup. At least one dynamic pass is required "
                "between a warmup pass and a constant pass."
            )
        out.append(str(pass_name))

    if not out:
        raise ValueError(
            f"{label} must list one or more passes. Example: warmup,warmup,warmup,dynamic"
        )
    return tuple(out)


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


def _spool_prefit_seed_candidates(
    x_seed: np.ndarray,
    lo: np.ndarray,
    hi: np.ndarray,
    *,
    kinds: Sequence[str],
) -> List[np.ndarray]:
    candidates = list(_spool_seed_candidates(x_seed, lo, hi))
    seed = np.asarray(x_seed, dtype=float).reshape(-1)
    if seed.size == 0 or lo.size != seed.size or hi.size != seed.size:
        return candidates
    if len(kinds) != seed.size:
        return candidates

    r_indices = [idx for idx, kind in enumerate(kinds) if str(kind) == "r"]
    if len(r_indices) != 1:
        return candidates
    r_idx = int(r_indices[0])
    lo_r = float(lo[r_idx])
    hi_r = float(hi[r_idx])
    span = float(hi_r - lo_r)
    if not np.isfinite(lo_r) or not np.isfinite(hi_r) or span <= 0.0:
        return candidates

    def _add(vec: np.ndarray) -> None:
        v = np.clip(np.asarray(vec, dtype=float).reshape(-1), lo, hi)
        for existing in candidates:
            if existing.shape == v.shape and np.allclose(existing, v, atol=1e-12, rtol=0.0):
                return
        candidates.append(v)

    grid_points = max(5, int(_SPOOL_PREFIT_GLOBAL_R_GRID_POINTS))
    for frac in np.linspace(0.0, 1.0, grid_points):
        vec = seed.copy()
        vec[r_idx] = lo_r + frac * span
        _add(vec)

    return candidates


def _bounded_axis_line_search(
    x: np.ndarray,
    idx: int,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    step: float,
    xatol: float,
    objective: Any,
    current_cost: float,
) -> Tuple[float, float, int]:
    x_arr = np.asarray(x, dtype=float).reshape(-1)
    xi = float(x_arr[idx])
    lo_i = float(lo[idx])
    hi_i = float(hi[idx])
    radius = float(max(step, 0.0))
    left = float(max(lo_i, xi - radius))
    right = float(min(hi_i, xi + radius))
    min_width = float(max(xatol, 1e-12))
    if (not np.isfinite(left)) or (not np.isfinite(right)) or right - left <= min_width:
        return float(xi), float(current_cost), 0

    evals = 0
    cache: Dict[float, float] = {}

    def _eval_axis(value: float) -> float:
        nonlocal evals
        cand = float(np.clip(float(value), left, right))
        key = float(np.round(cand, decimals=12))
        cached = cache.get(key)
        if cached is not None and np.isfinite(cached):
            return float(cached)
        x_try = x_arr.copy()
        x_try[idx] = float(cand)
        score = float(objective(x_try))
        evals += 1
        if not np.isfinite(score):
            score = 1e12
        cache[key] = float(score)
        return float(score)

    best_val = float(xi)
    best_cost = float(current_cost)
    cache[float(np.round(xi, decimals=12))] = float(best_cost)
    if not np.isfinite(best_cost):
        best_cost = float(_eval_axis(xi))

    sample_points: List[float] = [float(xi)]
    midpoint_points: set[float] = set()
    if left < xi - 1e-12:
        midpoint_left = float(np.round(0.5 * (left + xi), decimals=12))
        midpoint_points.add(midpoint_left)
        sample_points.extend([midpoint_left, float(left)])
    if right > xi + 1e-12:
        midpoint_right = float(np.round(0.5 * (xi + right), decimals=12))
        midpoint_points.add(midpoint_right)
        sample_points.extend([midpoint_right, float(right)])
    sample_points = sorted({float(np.round(v, decimals=12)) for v in sample_points})

    sample_scores: List[float] = []
    best_sample_idx = 0
    for sample_idx, cand in enumerate(sample_points):
        score = float(_eval_axis(float(cand)))
        sample_scores.append(float(score))
        if np.isfinite(score) and score + 1e-12 < best_cost:
            best_cost = float(score)
            best_val = float(cand)
            best_sample_idx = int(sample_idx)

    best_sample_is_midpoint = float(np.round(best_val, decimals=12)) in midpoint_points
    if (
        best_sample_is_midpoint
        and np.isfinite(best_cost)
        and best_cost + 1e-12 < float(current_cost)
    ):
        lo_ref = float(sample_points[max(0, best_sample_idx - 1)])
        hi_ref = float(sample_points[min(len(sample_points) - 1, best_sample_idx + 1)])
        if hi_ref - lo_ref > min_width:
            try:
                result = minimize_scalar(
                    lambda value: float(_eval_axis(float(value))),
                    bounds=(float(lo_ref), float(hi_ref)),
                    method="bounded",
                    options={"xatol": float(min_width), "maxiter": 40},
                )
                if bool(getattr(result, "success", False)):
                    cand = float(np.clip(float(getattr(result, "x", best_val)), lo_ref, hi_ref))
                    score = float(_eval_axis(cand))
                    if np.isfinite(score) and score + 1e-12 < best_cost:
                        best_cost = float(score)
                        best_val = float(cand)
            except Exception:
                pass

    return float(best_val), float(best_cost), int(evals)


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
            best_axis_value, best_axis_cost, axis_nfev = _bounded_axis_line_search(
                x,
                idx,
                lo=lo,
                hi=hi,
                step=float(steps[idx]),
                xatol=float(tol[idx]),
                objective=objective,
                current_cost=best_local,
            )
            nfev += int(axis_nfev)
            if (
                np.isfinite(best_axis_cost)
                and best_axis_cost + 1e-12 < best_local
                and abs(float(best_axis_value) - float(x[idx])) > 1e-12
            ):
                x[idx] = float(best_axis_value)
                best = float(best_axis_cost)
                improved_this_round = True
                improved_any = True
            else:
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
    span = float("nan")
    if max_travel is not None and np.isfinite(max_travel) and max_travel > 0.0:
        span = float(max_travel)
    else:
        finite_obs = [float(v) for v in observed_deltas if np.isfinite(float(v))]
        if finite_obs:
            span = float(np.max(np.abs(np.asarray(finite_obs, dtype=float))))

    if not np.isfinite(span) or span <= 0.0:
        return None

    # Keep candidate generation bidirectional by default so the planner can
    # still propose valid negative targets after one sign saturates.
    min_floor = 10.0
    span = max(float(span), float(min_floor))
    return -span, span


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
    sigma_floor_mm: Optional[float],
    sigma_used_mm: Optional[float],
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
    if sigma_floor_mm is not None:
        try:
            sigma_floor = float(sigma_floor_mm)
        except (TypeError, ValueError):
            sigma_floor = float("nan")
        if np.isfinite(sigma_floor) and sigma_floor > 0.0:
            noise_model["sigma_floor_mm"] = float(sigma_floor)
    if sigma_used_mm is not None:
        try:
            sigma_used = float(sigma_used_mm)
        except (TypeError, ValueError):
            sigma_used = float("nan")
        if np.isfinite(sigma_used) and sigma_used > 0.0:
            noise_model["sigma_used_mm"] = float(sigma_used)
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
    gcode = str(cal.get("gcode", "")) if isinstance(cal, dict) else ""
    anchor_line = f"; Anchors: {format_anchor_matrix_plain(anchors)}" if anchors.size else ""
    if anchor_line and gcode.strip() != anchor_line:
        print(anchor_line)
    if gcode:
        print(gcode)
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
            prefit = radii_fit.get("prefit")
            if isinstance(prefit, dict):
                print(
                    f"; line_model_prefit: enabled={bool(prefit.get('enabled', False))} "
                    f"success={bool(prefit.get('success', False))} "
                    f"guarded={bool(prefit.get('guarded', False))} "
                    f"seed={str(prefit.get('seed_choice', ''))} "
                    f"start={_fmt_float(prefit.get('start_cost'))} "
                    f"fit={_fmt_float(prefit.get('fitted_cost'))} "
                    f"start_data={_fmt_float(prefit.get('start_data_cost'))} "
                    f"start_prior={_fmt_float(prefit.get('start_prior_cost'))} "
                    f"start_total={_fmt_float(prefit.get('start_total_cost', prefit.get('start_cost')))} "
                    f"fit_data={_fmt_float(prefit.get('fitted_data_cost'))} "
                    f"fit_prior={_fmt_float(prefit.get('fitted_prior_cost'))} "
                    f"fit_total={_fmt_float(prefit.get('fitted_total_cost', prefit.get('fitted_cost')))} "
                    f"valid_sweeps={_fmt_float(prefit.get('valid_sweeps'), fmt='.0f')} "
                    f"invalid_sweeps={_fmt_float(prefit.get('invalid_sweeps'), fmt='.0f')}"
                )
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
                    f"last_data={_fmt_float(last.get('cal_data_cost', last.get('cal_cost')))} "
                    f"last_prior={_fmt_float(last.get('cal_prior_cost'))} "
                    f"last_total={_fmt_float(last.get('cal_total_cost'))} "
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
            filter_schedule_history = radii_fit.get("filter_schedule_history")
            if isinstance(filter_schedule_history, list) and filter_schedule_history:
                last_filter_pass = (
                    filter_schedule_history[-1]
                    if isinstance(filter_schedule_history[-1], dict)
                    else {}
                )
                filter_schedule_requested = radii_fit.get("filter_schedule_requested")
                objective_schedule_requested = radii_fit.get("objective_schedule_requested")
                schedule_text = ""
                objective_text = ""
                if isinstance(filter_schedule_requested, list) and filter_schedule_requested:
                    schedule_text = ",".join(str(v) for v in filter_schedule_requested)
                if isinstance(objective_schedule_requested, list) and objective_schedule_requested:
                    objective_text = ",".join(str(v) for v in objective_schedule_requested)
                print(
                    f"; line_model_filter_schedule: passes={_fmt_float(len(filter_schedule_history), fmt='.0f')} "
                    f"schedule=[{schedule_text}] "
                    f"objectives=[{objective_text}] "
                    f"last_raw_fit_score_ui={_fmt_float(last_filter_pass.get('score_ui'))} "
                    f"last_rank_score={_fmt_float(last_filter_pass.get('rank_score'))} "
                    f"last_cost={_fmt_float(last_filter_pass.get('cost_noise_normalized'))}"
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
                tau_mad_mm = noise_metrics.get("tau_mad_mm")
                cost_old = noise_metrics.get("cost_noise_normalized_old", cost)
                cost_new = noise_metrics.get("cost_noise_normalized_rescored")
                chi2_old = noise_metrics.get("chi2_red_old", chi2_red)
                chi2_new = noise_metrics.get("chi2_red_rescored")
                residual_slope = noise_metrics.get(
                    "residual_vs_distance_slope_inliers",
                    noise_metrics.get("residual_vs_distance_slope"),
                )
                residual_abs_slope = noise_metrics.get("residual_abs_vs_distance_slope")
                residual_sq_slope = noise_metrics.get("residual_sq_vs_distance_slope")
                inlier_cutoff_mm = noise_metrics.get("inlier_cutoff_mm")
                n_inlier_rows = noise_metrics.get("n_inlier_rows_used_for_tau")
                tau_d_tau0 = noise_metrics.get("tau_d_tau0_mm")
                tau_d_tau1 = noise_metrics.get("tau_d_tau1_per_mm")
                tau_d_cost = noise_metrics.get(
                    "cost_noise_normalized_tau_d",
                    noise_metrics.get("cost_noise_normalized_rescored_tau_d"),
                )
                tau_d_chi2 = noise_metrics.get("chi2_red_tau_d", noise_metrics.get("chi2_red_rescored_tau_d"))
                tau_d_cost_trimmed_direct = noise_metrics.get("cost_noise_normalized_tau_d_trimmed_direct")
                tau_d_chi2_trimmed_direct = noise_metrics.get("chi2_red_tau_d_trimmed_direct")
                bias_intercept = noise_metrics.get("bias_vs_distance_intercept_mm")
                bias_slope = noise_metrics.get(
                    "bias_vs_distance_slope_inliers",
                    noise_metrics.get("bias_vs_distance_slope_mm_per_mm"),
                )
                cost_after_bias = noise_metrics.get("cost_after_bias_diagnostic")
                chi2_after_bias = noise_metrics.get("chi2_red_after_bias_diagnostic")
                tau_3bin = noise_metrics.get("tau_3bin_debiased_mm")
                tau_3bin_cost = noise_metrics.get("cost_noise_normalized_rescored_tau_3bin_debiased")
                tau_3bin_chi2 = noise_metrics.get("chi2_red_rescored_tau_3bin_debiased")
                tau_3bin_chi2_trimmed_direct = noise_metrics.get("chi2_red_tau_3bin_debiased_trimmed_direct")
                trimmed_coherence_ratio = noise_metrics.get("trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased")
                if any(
                    value is not None
                    for value in (
                        tau_mad_mm,
                        cost_new,
                        chi2_new,
                        residual_slope,
                        residual_abs_slope,
                        residual_sq_slope,
                        inlier_cutoff_mm,
                        n_inlier_rows,
                        tau_d_tau0,
                        tau_d_tau1,
                        tau_d_cost,
                        tau_d_chi2,
                        tau_d_cost_trimmed_direct,
                        tau_d_chi2_trimmed_direct,
                        tau_3bin_chi2_trimmed_direct,
                        trimmed_coherence_ratio,
                        bias_intercept,
                        bias_slope,
                        cost_after_bias,
                        chi2_after_bias,
                        tau_3bin_cost,
                        tau_3bin_chi2,
                    )
                ):
                    print(
                        f"; noise_rescore: tau_MAD={_fmt_float(tau_mad_mm, suffix='mm')} "
                        f"cost_noise_normalized_old={_fmt_float(cost_old)} "
                        f"cost_noise_normalized_new={_fmt_float(cost_new)} "
                        f"chi2_red_old={_fmt_float(chi2_old)} "
                        f"chi2_red_new={_fmt_float(chi2_new)} "
                        f"residual_vs_distance_slope_inliers={_fmt_float(residual_slope)} "
                        f"slope(|r|,d)={_fmt_float(residual_abs_slope)} "
                        f"slope(r^2,d)={_fmt_float(residual_sq_slope)} "
                        f"inlier_cutoff_mm={_fmt_float(inlier_cutoff_mm, suffix='mm')} "
                        f"n_inlier_rows_used_for_tau={_fmt_float(n_inlier_rows, fmt='.0f')} "
                        f"tau_d_tau0={_fmt_float(tau_d_tau0, suffix='mm')} "
                        f"tau_d_tau1={_fmt_float(tau_d_tau1)} "
                        f"cost_noise_normalized_tau_d={_fmt_float(tau_d_cost)} "
                        f"chi2_red_tau_d={_fmt_float(tau_d_chi2)} "
                        f"cost_noise_normalized_tau_d_trimmed_direct={_fmt_float(tau_d_cost_trimmed_direct)} "
                        f"chi2_red_tau_d_trimmed_direct={_fmt_float(tau_d_chi2_trimmed_direct)} "
                        f"chi2_red_tau_3bin_debiased_trimmed_direct={_fmt_float(tau_3bin_chi2_trimmed_direct)} "
                        f"trimmed_coherence_ratio_tau_d_over_tau_3bin_debiased={_fmt_float(trimmed_coherence_ratio)}"
                    )
                if any(value is not None for value in (bias_intercept, bias_slope, cost_after_bias, chi2_after_bias)):
                    print(
                        f"; noise_bias_diagnostic: "
                        f"bias_vs_distance_intercept={_fmt_float(bias_intercept, suffix='mm')} "
                        f"bias_vs_distance_slope_inliers={_fmt_float(bias_slope)} "
                        f"cost_after_bias_diagnostic={_fmt_float(cost_after_bias)} "
                        f"chi2_red_after_bias_diagnostic={_fmt_float(chi2_after_bias)}"
                    )
                distance_bin_mad = noise_metrics.get("distance_bin_mad_mm")
                distance_bin_counts = noise_metrics.get("distance_bin_counts")
                if isinstance(distance_bin_mad, dict):
                    near_count = None
                    mid_count = None
                    far_count = None
                    if isinstance(distance_bin_counts, dict):
                        near_count = distance_bin_counts.get("near")
                        mid_count = distance_bin_counts.get("mid")
                        far_count = distance_bin_counts.get("far")
                    near_str = _fmt_float(distance_bin_mad.get("near"), suffix="mm")
                    mid_str = _fmt_float(distance_bin_mad.get("mid"), suffix="mm")
                    far_str = _fmt_float(distance_bin_mad.get("far"), suffix="mm")
                    near_count_str = (
                        f"{int(near_count)}"
                        if isinstance(near_count, (int, float)) and np.isfinite(near_count)
                        else "n/a"
                    )
                    mid_count_str = (
                        f"{int(mid_count)}"
                        if isinstance(mid_count, (int, float)) and np.isfinite(mid_count)
                        else "n/a"
                    )
                    far_count_str = (
                        f"{int(far_count)}"
                        if isinstance(far_count, (int, float)) and np.isfinite(far_count)
                        else "n/a"
                    )
                    print(
                        f"; noise_rescore_bins: near_MAD={near_str} mid_MAD={mid_str} far_MAD={far_str} "
                        f"N=[{near_count_str},{mid_count_str},{far_count_str}]"
                    )
                distance_bin_mad_debiased = noise_metrics.get("distance_bin_mad_debiased_mm")
                if isinstance(distance_bin_mad_debiased, dict):
                    near_str = _fmt_float(distance_bin_mad_debiased.get("near"), suffix="mm")
                    mid_str = _fmt_float(distance_bin_mad_debiased.get("mid"), suffix="mm")
                    far_str = _fmt_float(distance_bin_mad_debiased.get("far"), suffix="mm")
                    print(
                        f"; noise_rescore_bins_debiased: near_MAD={near_str} mid_MAD={mid_str} far_MAD={far_str}"
                    )
                if any(value is not None for value in (tau_3bin, tau_3bin_cost, tau_3bin_chi2)):
                    near_tau = None
                    mid_tau = None
                    far_tau = None
                    if isinstance(tau_3bin, dict):
                        near_tau = tau_3bin.get("near")
                        mid_tau = tau_3bin.get("mid")
                        far_tau = tau_3bin.get("far")
                    print(
                        f"; noise_rescore_tau_3bin_debiased: "
                        f"tau_near={_fmt_float(near_tau, suffix='mm')} "
                        f"tau_mid={_fmt_float(mid_tau, suffix='mm')} "
                        f"tau_far={_fmt_float(far_tau, suffix='mm')} "
                        f"cost_noise_normalized_tau_3bin_debiased={_fmt_float(tau_3bin_cost)} "
                        f"chi2_red_tau_3bin_debiased={_fmt_float(tau_3bin_chi2)} "
                        f"chi2_red_tau_3bin_debiased_trimmed_direct={_fmt_float(tau_3bin_chi2_trimmed_direct)}"
                    )
                per_sweep_summary = noise_metrics.get("per_sweep_residual_summary")
                if isinstance(per_sweep_summary, dict) and per_sweep_summary:
                    ranked_sweeps: List[Tuple[float, str, dict]] = []
                    for sweep_id, sweep_stats in per_sweep_summary.items():
                        if not isinstance(sweep_stats, dict):
                            continue
                        med = _float_or_none(sweep_stats.get("median_residual_mm"))
                        p95 = _float_or_none(sweep_stats.get("p95_abs_residual_mm"))
                        score = abs(med) if med is not None else (p95 if p95 is not None else 0.0)
                        ranked_sweeps.append((float(score), str(sweep_id), sweep_stats))
                    ranked_sweeps.sort(key=lambda item: item[0], reverse=True)
                    sweep_limit = max(1, int(top_n))
                    for _score, sweep_id, sweep_stats in ranked_sweeps[:sweep_limit]:
                        clipped = sweep_stats.get("clipped_points")
                        total = sweep_stats.get("total_points")
                        clipped_str = (
                            f"{int(clipped)}/{int(total)}"
                            if isinstance(clipped, (int, float))
                            and np.isfinite(clipped)
                            and isinstance(total, (int, float))
                            and np.isfinite(total)
                            else "n/a"
                        )
                        print(
                            f"; sweep_residual: id={sweep_id} "
                            f"median={_fmt_float(sweep_stats.get('median_residual_mm'), suffix='mm')} "
                            f"MAD={_fmt_float(sweep_stats.get('mad_residual_mm'), suffix='mm')} "
                            f"p95_abs={_fmt_float(sweep_stats.get('p95_abs_residual_mm'), suffix='mm')} "
                            f"mean|z|={_fmt_float(sweep_stats.get('mean_abs_z'))} "
                            f"clipped={clipped_str}"
                        )
                    extra = int(max(0, len(ranked_sweeps) - sweep_limit))
                    if extra > 0:
                        print(f"; sweep_residual: +{extra} more sweeps")
                per_sweep_demean = noise_metrics.get("per_sweep_demean")
                if isinstance(per_sweep_demean, dict) and per_sweep_demean:
                    n_obs_demeaned = per_sweep_demean.get("n_obs_demeaned")
                    n_obs_str = (
                        f"{int(n_obs_demeaned)}"
                        if isinstance(n_obs_demeaned, (int, float)) and np.isfinite(n_obs_demeaned)
                        else "n/a"
                    )
                    print(
                        f"; noise_demean_by_sweep: "
                        f"cost_noise_normalized_demeaned={_fmt_float(per_sweep_demean.get('cost_noise_normalized_demeaned'))} "
                        f"chi2_red_demeaned={_fmt_float(per_sweep_demean.get('chi2_red_demeaned'))} "
                        f"tail_ratio={_fmt_float(per_sweep_demean.get('tail_ratio'))} "
                        f"normalized_sweep_bias={_fmt_float(per_sweep_demean.get('normalized_sweep_bias'))} "
                        f"tail_factor_median={_fmt_float(per_sweep_demean.get('tail_factor_median'))} "
                        f"sweep_bias_span={_fmt_float(per_sweep_demean.get('sweep_bias_span_mm'), suffix='mm')} "
                        f"n_obs={n_obs_str}"
                    )
                rank_coverage_adjust, rank_coverage_info = _rank_coverage_adjustment_from_noise_metrics(
                    noise_metrics
                )
                if any(
                    rank_coverage_info.get(key) is not None
                    for key in ("effective_obs", "filtered_ratio", "obs_bonus", "filtered_penalty")
                ):
                    print(
                        f"; rank_coverage: "
                        f"effective_obs={_fmt_float(rank_coverage_info.get('effective_obs'), fmt='.0f')} "
                        f"total_obs={_fmt_float(rank_coverage_info.get('total_obs'), fmt='.0f')} "
                        f"filtered_ratio={_fmt_float(rank_coverage_info.get('filtered_ratio'))} "
                        f"obs_bonus={_fmt_float(rank_coverage_info.get('obs_bonus'))} "
                        f"filtered_penalty={_fmt_float(rank_coverage_info.get('filtered_penalty'))} "
                        f"rank_adjust={_fmt_float(rank_coverage_adjust)}"
                    )
                sigma_min_mm = noise_metrics.get("sigma_min_mm")
                sigma_model_mm = noise_metrics.get("sigma_model_mm")
                sigma_used_mm = noise_metrics.get("sigma_used_mm")
                sigma_floor_deg = noise_metrics.get("sigma_floor_deg")
                sigma_source = noise_metrics.get("sigma_source")
                if (
                    sigma_min_mm is not None
                    or sigma_model_mm is not None
                    or sigma_used_mm is not None
                    or sigma_floor_deg is not None
                    or sigma_source is not None
                ):
                    min_str = _fmt_float(sigma_min_mm, suffix="mm")
                    model_str = _fmt_float(sigma_model_mm, suffix="mm")
                    used_str = _fmt_float(sigma_used_mm, suffix="mm")
                    floor_str = _fmt_float(sigma_floor_deg, suffix="deg")
                    source_str = str(sigma_source) if sigma_source is not None else "n/a"
                    print(
                        f"; noise_norm_floor: min_sigma={min_str} model_sigma={model_str} "
                        f"used_sigma={used_str} floor_deg={floor_str} source={source_str}"
                    )
                sigma_encoder = noise_metrics.get("sigma_encoder_mm")
                sigma_friction = noise_metrics.get("sigma_friction_cogging_mm")
                sigma_flex = noise_metrics.get("sigma_flex_mm")
                sigma_floor_term = noise_metrics.get("sigma_floor_term_mm")
                sigma_non_layered = noise_metrics.get("sigma_non_layered_mm")
                sigma_layer_changes = noise_metrics.get("sigma_layer_changes_mm")
                sigma_mode_add = noise_metrics.get("sigma_mode_addition_mm")
                sigma_total = noise_metrics.get("sigma_total_mm")
                sigma_model = noise_metrics.get("sigma_model_mm")
                sigma_used = noise_metrics.get("sigma_used_mm")
                sigma_mode = noise_metrics.get("sigma_solver_mode")
                sigma_mode_factor = noise_metrics.get("sigma_solver_mode_factor")
                sigma_line_width = noise_metrics.get("sigma_line_width_mm")
                sigma_base_radius = noise_metrics.get("sigma_base_radius_mm")
                sigma_layered = noise_metrics.get("sigma_layered_enabled")
                sigma_used_override = noise_metrics.get("sigma_used_override_mm")
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
                        sigma_model,
                        sigma_used,
                        sigma_mode,
                        sigma_mode_factor,
                        sigma_line_width,
                        sigma_base_radius,
                        sigma_layered,
                        sigma_used_override,
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
                        f"model={_fmt_float(sigma_model, suffix='mm')} "
                        f"used={_fmt_float(sigma_used, suffix='mm')} "
                        f"layered={layered_str} "
                        f"mode={str(sigma_mode) if sigma_mode is not None else 'n/a'} "
                        f"mode_factor={_fmt_float(sigma_mode_factor)} "
                        f"line_width={_fmt_float(sigma_line_width, suffix='mm')} "
                        f"base_radius={_fmt_float(sigma_base_radius, suffix='mm')} "
                        f"used_override={_fmt_float(sigma_used_override, suffix='mm')}"
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


def _add_solver_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--solve-restarts", type=int, default=4)
    parser.add_argument("--solve-iterations", type=int, default=400)
    parser.add_argument("--solve-optimizer", default="L-BFGS-B")
    parser.add_argument(
        "--optimizer-mode",
        choices=_OPTIMIZER_MODE_CHOICES,
        default=_normalize_optimizer_mode(os.environ.get("AUTOCAL_OPTIMIZER_MODE", "fast")),
        help=(
            "Optimization mode: 'fast' (JAX exact Jacobian), "
            "'fast-fd' (JAX value + SciPy finite differences), "
            "or 'legacy' (no JAX + SciPy finite differences)."
        ),
    )
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
        help="Comma-separated base radii in mm (defaults to config m666_adjusted_by_data_collector/m666_before_data_collection R).",
    )
    parser.add_argument(
        "--buildup-factor",
        type=float,
        default=None,
        help="Use this buildup factor k in the model transform (if omitted, falls back to config m666_adjusted_by_data_collector/m666_before_data_collection Q, else 0).",
    )
    parser.add_argument(
        "--line-width",
        type=float,
        default=DEFAULT_LAYER_LINE_WIDTH_MM,
        help="Estimated line width in mm used by the layered noise model (default: 1.0).",
    )
    parser.add_argument(
        "--sigma-floor-mm",
        type=float,
        default=None,
        help="Override the base sigma floor (mm) used in the composite noise model.",
    )
    parser.add_argument(
        "--sigma-used-mm",
        type=float,
        default=None,
        help="Override the final sigma used for pointwise normalization (mm).",
    )
    parser.add_argument(
        "--low-anchor-z",
        type=float,
        default=None,
        help="For hangprinter_4, fix the shared low-anchor Z plane to this value in mm.",
    )
    parser.add_argument(
        "--r0-bounds",
        type=str,
        default=None,
        help="Bounds for fitted r0 in mm as 'min,max' (default [base_radius, 1.5*base_radius]).",
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
        default=None,
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
        "--scale-fix",
        type=str,
        default="3",
        help="Enable scale-coupling fixes by id (comma-separated): 1,2,3 (default: 3). Use 'off' to disable.",
    )
    parser.add_argument(
        "--fit-structure",
        type=str,
        default="3",
        help=(
            "Enable fit-structure rank penalties by id (comma-separated): "
            "1=tail_ratio, 2=sweep_bias_directionality, 3=tail_factor (default: 3)."
        ),
    )
    parser.add_argument(
        "--filter-schedule",
        type=str,
        default="warmup,warmup,warmup,dynamic",
        help=(
            "Explicit filter-pass schedule as comma-separated words or numbers. "
            "Words: warmup,dynamic,constant. Numbers: 0=warmup, 1=dynamic, 2=constant. "
            "Default: warmup,warmup,warmup,dynamic."
        ),
    )
    parser.add_argument(
        "--objective-schedule",
        type=str,
        default="1,1,1,1",
        help=(
            "Explicit spool objective schedule as comma-separated words or numbers. "
            "Words: prefit,pointwise,position. Numbers: 0=ellipse_prefit, "
            "1=pointwise_forward_model, 2=position_reconstruction. "
            "Default: 1,1,1,1."
        ),
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


def build_semi_auto_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Non-interactive autocal active-learning loop."
    )
    parser.add_argument(
        "--machine-type",
        choices=MACHINE_TYPE_INPUT_CHOICES,
        required=True,
        help=f"Machine type ({MACHINE_TYPE_CHOICES_STR})",
    )
    parser.add_argument(
        "--no-collect",
        action="store_true",
        help="Do not collect new sweeps; stop when replayed data is exhausted.",
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
        help="Append full-auto runs from shotgun.conf.",
    )
    parser.add_argument(
        "--full-auto-log",
        type=Path,
        default=None,
        help="Write full-auto JSONL logs to this path (default: <dataset>.full_auto_log.jsonl).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose iteration output, including anchors, M666, and rank scoring.",
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
        scale_fix_levels = _parse_scale_fix_levels(args.scale_fix, label="--scale-fix")
        fit_structure_levels = _parse_fit_structure_levels(args.fit_structure, label="--fit-structure")
        objective_schedule = _parse_objective_schedule(
            args.objective_schedule,
            label="--objective-schedule",
        )
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
    try:
        filter_schedule = _parse_filter_schedule(args.filter_schedule, label="--filter-schedule")
    except ValueError as exc:
        parser.error(str(exc))

    line_width = args.line_width
    try:
        line_width = float(line_width)
    except (TypeError, ValueError):
        parser.error("--line-width must be numeric")
    if not np.isfinite(line_width) or line_width < 0.0:
        parser.error("--line-width must be finite and >= 0")

    sigma_floor_mm = args.sigma_floor_mm
    if sigma_floor_mm is not None:
        try:
            sigma_floor_mm = float(sigma_floor_mm)
        except (TypeError, ValueError):
            parser.error("--sigma-floor-mm must be numeric")
        if not np.isfinite(sigma_floor_mm) or sigma_floor_mm <= 0.0:
            parser.error("--sigma-floor-mm must be finite and > 0")

    sigma_used_mm = args.sigma_used_mm
    if sigma_used_mm is not None:
        try:
            sigma_used_mm = float(sigma_used_mm)
        except (TypeError, ValueError):
            parser.error("--sigma-used-mm must be numeric")
        if not np.isfinite(sigma_used_mm) or sigma_used_mm <= 0.0:
            parser.error("--sigma-used-mm must be finite and > 0")

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
        "filter_schedule": [str(v) for v in filter_schedule],
        "objective_schedule": [int(v) for v in objective_schedule],
        "line_width": float(line_width),
        "sigma_floor_mm": (None if sigma_floor_mm is None else float(sigma_floor_mm)),
        "sigma_used_mm": (None if sigma_used_mm is None else float(sigma_used_mm)),
        "scale_fix": [int(v) for v in scale_fix_levels],
        "fit_structure": [int(v) for v in fit_structure_levels],
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
    parser.add_argument("--scale-fix", default=None)
    parser.add_argument("--fit-structure", default=None)
    parser.add_argument("--filter-schedule", default=None)
    parser.add_argument("--objective-schedule", default=None)
    parser.add_argument("--line-width", type=float, default=None)
    parser.add_argument("--sigma-floor-mm", type=float, default=None)
    parser.add_argument("--sigma-used-mm", type=float, default=None)
    parser.add_argument("--low-anchor-z", type=float, default=None)
    parser.add_argument("--spring-k-multiplier", type=float, default=None)
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--solve-restarts", type=int, default=None)
    parser.add_argument("--solve-iterations", type=int, default=None)
    parser.add_argument("--solve-optimizer", default=None)
    parser.add_argument("--optimizer-mode", choices=_OPTIMIZER_MODE_CHOICES, default=None)
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


def _plan_uses_layered_score(plan: Dict[str, object]) -> bool:
    length_model = plan.get("length_model")
    if isinstance(length_model, dict):
        for key in ("find_radii_mode", "find_buildup_factor_mode"):
            mode = str(length_model.get(key, "")).strip().lower()
            if mode and mode != "off":
                return True
        for key in ("find_radii", "find_buildup_factor"):
            if bool(length_model.get(key)):
                return True
    noise_metrics = _plan_noise_metrics(plan)
    if isinstance(noise_metrics, dict):
        if _float_or_none(noise_metrics.get("chi2_red_rescored_tau_3bin_debiased")) is not None:
            return True
    return False


def _noise_metric_float(noise_metrics: dict, key: str) -> Optional[float]:
    direct = _float_or_none(noise_metrics.get(key))
    if direct is not None:
        return float(direct)
    per_sweep_demean = noise_metrics.get("per_sweep_demean")
    if isinstance(per_sweep_demean, dict):
        nested = _float_or_none(per_sweep_demean.get(key))
        if nested is not None:
            return float(nested)
    return None


def _rank_coverage_adjustment_from_noise_metrics(
    noise_metrics: Optional[dict],
    *,
    obs_ref: float = _SCORE_UI_RANK_OBS_REF,
    obs_bonus_weight: float = _SCORE_UI_RANK_OBS_BONUS_WEIGHT,
    filtered_ratio_weight: float = _SCORE_UI_RANK_FILTERED_RATIO_WEIGHT,
) -> Tuple[float, Dict[str, Optional[float]]]:
    info: Dict[str, Optional[float]] = {
        "effective_obs": None,
        "total_obs": None,
        "obs_bonus": None,
        "filtered_ratio": None,
        "filtered_penalty": None,
    }
    if not isinstance(noise_metrics, dict):
        return 0.0, info

    effective_obs = None
    for key in (
        "n_inlier_rows_used_for_tau",
        "tau_mad_inlier_points",
        "n_obs_trimmed",
        "n_obs",
    ):
        val = _float_or_none(noise_metrics.get(key))
        if val is not None and val > 0.0:
            effective_obs = float(val)
            break

    total_obs = None
    for key in (
        "tau_mad_total_points",
        "n_obs",
        "n_obs_trimmed",
    ):
        val = _float_or_none(noise_metrics.get(key))
        if val is not None and val > 0.0:
            total_obs = float(val)
            break

    if effective_obs is not None:
        info["effective_obs"] = float(effective_obs)
    if total_obs is not None:
        if effective_obs is not None and total_obs < effective_obs:
            total_obs = float(effective_obs)
        info["total_obs"] = float(total_obs)

    adjustment = 0.0

    obs_bonus = 0.0
    if (
        effective_obs is not None
        and np.isfinite(effective_obs)
        and effective_obs > max(1.0, float(obs_ref))
    ):
        obs_bonus = float(obs_bonus_weight) * float(
            np.log(max(float(effective_obs) / max(float(obs_ref), 1e-9), 1.0))
        )
        adjustment -= float(obs_bonus)
    info["obs_bonus"] = float(obs_bonus)

    filtered_ratio = None
    filtered_penalty = 0.0
    if (
        effective_obs is not None
        and total_obs is not None
        and np.isfinite(effective_obs)
        and np.isfinite(total_obs)
        and total_obs > 0.0
    ):
        filtered_ratio = max(0.0, 1.0 - (float(effective_obs) / float(total_obs)))
        filtered_penalty = float(filtered_ratio_weight) * float(filtered_ratio)
        adjustment += float(filtered_penalty)
    info["filtered_ratio"] = None if filtered_ratio is None else float(filtered_ratio)
    info["filtered_penalty"] = float(filtered_penalty)

    return float(adjustment), info


def _full_auto_history_iteration_adjustment(iteration_index: int) -> float:
    idx = max(1, int(iteration_index))
    if idx <= 1:
        return 0.0
    return float(
        -_FULL_AUTO_HISTORY_LOG_BONUS_WEIGHT * np.log(float(idx))
        + _FULL_AUTO_HISTORY_LINEAR_PENALTY * float(idx - 1)
    )


def _full_auto_history_selection_score(
    rank_score: Optional[float],
    *,
    iteration_index: int,
    coverage_adjust: Optional[float] = None,
    coverage_weight: float = _FULL_AUTO_HISTORY_COVERAGE_WEIGHT,
    extra_adjust: float = 0.0,
) -> Tuple[float, Dict[str, Optional[float]]]:
    info: Dict[str, Optional[float]] = {
        "iteration_adjust": None,
        "coverage_adjust": None,
        "extra_adjust": None,
        "selection_score": None,
    }
    rank = _float_or_none(rank_score)
    if rank is None or not np.isfinite(rank):
        return float("inf"), info

    idx = max(1, int(iteration_index))
    iteration_adjust = _full_auto_history_iteration_adjustment(idx)

    coverage = _float_or_none(coverage_adjust)
    if coverage is None or not np.isfinite(coverage):
        coverage = 0.0

    extra = _float_or_none(extra_adjust)
    if extra is None or not np.isfinite(extra):
        extra = 0.0

    selection_score = float(rank + iteration_adjust + float(coverage_weight) * coverage + float(extra))
    info["iteration_adjust"] = float(iteration_adjust)
    info["coverage_adjust"] = float(coverage)
    info["extra_adjust"] = float(extra)
    info["selection_score"] = float(selection_score)
    return float(selection_score), info


def _full_auto_hangprinter_sweep_penalty(
    machine_type: Optional[str],
    dimensions: Optional[int],
    sweep_count: Optional[int],
) -> float:
    machine_norm = _normalize_machine_type(machine_type)
    if machine_norm != "hangprinter_4":
        return 0.0
    try:
        dims = int(dimensions) if dimensions is not None else 0
    except (TypeError, ValueError):
        dims = 0
    if dims != 3:
        return 0.0
    try:
        sweeps = int(sweep_count) if sweep_count is not None else 0
    except (TypeError, ValueError):
        sweeps = 0
    deficit = max(0, 5 - sweeps)
    return float(0.65 * float(deficit))


def _fit_score_ui_from_history_rank_score(
    history_rank_score: Optional[float],
    *,
    fallback_score_ui: Optional[float] = None,
) -> float:
    history_rank = _float_or_none(history_rank_score)
    if history_rank is None or not np.isfinite(history_rank):
        fallback = _float_or_none(fallback_score_ui)
        return float(fallback) if fallback is not None and np.isfinite(fallback) else float("nan")

    rank = float(history_rank)
    xp = _FULL_AUTO_HISTORY_UI_RANK_POINTS
    yp = _FULL_AUTO_HISTORY_UI_SCORE_LOG_POINTS
    if rank <= float(xp[0]):
        x0 = float(xp[0])
        x1 = float(xp[1])
        y0 = float(yp[0])
        y1 = float(yp[1])
        slope = (y1 - y0) / max(x1 - x0, 1e-12)
        mapped_log = float(y0 + slope * (rank - x0))
    elif rank >= float(xp[-1]):
        return float(_FULL_AUTO_HISTORY_UI_SCORE_POINTS[-1])
    else:
        mapped_log = float(np.interp(rank, xp, yp))

    mapped = float(np.expm1(mapped_log))
    if mapped < 0.0:
        fallback = _float_or_none(fallback_score_ui)
        if fallback is not None and np.isfinite(fallback) and fallback >= 0.0:
            return float(fallback)
        return 0.0
    return float(min(mapped, float(_FULL_AUTO_HISTORY_UI_SCORE_POINTS[-1])))


def _full_auto_display_fit_score_ui(
    *,
    score_basis: Optional[str],
    raw_fit_score_ui: Optional[float],
    history_rank_score: Optional[float],
) -> float:
    basis = str(score_basis or "")
    if basis != "layered-calibrated":
        raw = _float_or_none(raw_fit_score_ui)
        return float(raw) if raw is not None and np.isfinite(raw) else float("nan")
    return _fit_score_ui_from_history_rank_score(
        history_rank_score,
        fallback_score_ui=raw_fit_score_ui,
    )


def _layered_internal_metric_from_noise_metrics(
    noise_metrics: Optional[dict],
    *,
    cost_raw: Optional[float] = None,
    fit_structure_levels: Optional[Sequence[int]] = None,
    use_fit_structure_penalties: bool = True,
) -> Optional[float]:
    if not isinstance(noise_metrics, dict):
        return None
    fit_structure_set: set[int] = set()
    if use_fit_structure_penalties:
        fit_structure_set = set(
            _parse_fit_structure_levels(fit_structure_levels, label="fit_structure")
        )
    m_layered = None
    for key in (
        "chi2_red_rescored_tau_3bin_debiased",
        "chi2_red_rescored",
        "chi2_red_trimmed",
        "chi2_red",
    ):
        m_try = _float_or_none(noise_metrics.get(key))
        if m_try is not None:
            m_layered = float(m_try)
            break
    if m_layered is None:
        return None

    cost_raw_val = _float_or_none(cost_raw)
    if cost_raw_val is not None:
        m_layered *= (
            1.0
            + _SCORE_UI_LAYERED_COST_RAW_WEIGHT
            * max(0.0, cost_raw_val / _SCORE_UI_LAYERED_COST_RAW_REF - 1.0)
        )

    tau_mad_mm = _float_or_none(noise_metrics.get("tau_mad_mm"))
    if tau_mad_mm is not None:
        m_layered *= (
            1.0
            + _SCORE_UI_LAYERED_TAU_MAD_WEIGHT
            * max(0.0, tau_mad_mm / _SCORE_UI_LAYERED_TAU_MAD_REF_MM - 1.0)
        )

    n_trim = _float_or_none(noise_metrics.get("n_obs_trimmed"))
    if n_trim is not None:
        m_layered *= (
            1.0
            + _SCORE_UI_LAYERED_N_TRIM_WEIGHT
            * max(0.0, (_SCORE_UI_LAYERED_N_TRIM_REF - n_trim) / 10.0)
        )

    if 1 in fit_structure_set:
        tail_ratio = _noise_metric_float(noise_metrics, "tail_ratio")
        if tail_ratio is not None:
            m_layered *= (
                1.0
                + _SCORE_UI_LAYERED_TAIL_RATIO_WEIGHT
                * max(0.0, tail_ratio / _SCORE_UI_LAYERED_TAIL_RATIO_REF - 1.0)
            )
    if 2 in fit_structure_set:
        normalized_sweep_bias = _noise_metric_float(noise_metrics, "normalized_sweep_bias")
        if normalized_sweep_bias is not None:
            m_layered *= (
                1.0
                + _SCORE_UI_LAYERED_SWEEP_BIAS_WEIGHT
                * max(0.0, normalized_sweep_bias / _SCORE_UI_LAYERED_SWEEP_BIAS_REF - 1.0)
            )
    if 3 in fit_structure_set:
        tail_factor_median = _noise_metric_float(noise_metrics, "tail_factor_median")
        if tail_factor_median is not None:
            m_layered *= (
                1.0
                + _SCORE_UI_LAYERED_TAIL_FACTOR_WEIGHT
                * max(0.0, tail_factor_median / _SCORE_UI_LAYERED_TAIL_FACTOR_REF - 1.0)
            )

    return float(m_layered)


def _layered_rank_score_from_internal_metric(
    m_layered: Optional[float],
    *,
    cost_raw: Optional[float] = None,
) -> float:
    if m_layered is None or not np.isfinite(m_layered):
        return float("inf")
    rank_score = float(np.log1p(max(float(m_layered), 0.0)))
    cost_raw_val = _float_or_none(cost_raw)
    if cost_raw_val is not None and cost_raw_val > 5.0:
        rank_score = max(rank_score, float(np.log1p(_SCORE_UI_HARD_FAIL)))
    return float(rank_score)


def _trimmed_risk_metric_from_components(
    noise_metrics: Optional[dict]
) -> Optional[float]:
    if not isinstance(noise_metrics, dict):
        return None

    chi2_trimmed_direct = _float_or_none(noise_metrics.get("chi2_red_tau_d_trimmed_direct"))
    if chi2_trimmed_direct is None or chi2_trimmed_direct < 0.0:
        return None

    risk = float(chi2_trimmed_direct)

    if not np.isfinite(risk):
        return None
    return float(risk)


def _blend_internal_metric_with_risk(
    base_metric: Optional[float],
    risk_metric: Optional[float],
    *,
    weight: float = _SCORE_UI_LAYERED_RISK_BLEND_WEIGHT,
) -> Optional[float]:
    base = _float_or_none(base_metric)
    risk = _float_or_none(risk_metric)
    if base is None:
        return float(risk) if risk is not None else None
    if risk is None:
        return float(base)
    w = max(0.0, float(weight))
    return float(base + w * max(float(risk), 0.0))


def _plan_trimmed_risk_metric(plan: Dict[str, object]) -> Optional[float]:
    noise_metrics = _plan_noise_metrics(plan)
    if not isinstance(noise_metrics, dict):
        return None

    _max_std_mm, rel_std, _cov_ok = _plan_covariance_summary(plan)
    base = _trimmed_risk_metric_from_components(noise_metrics)
    rel_std_val = _float_or_none(rel_std)
    if base is None or rel_std_val is None or not np.isfinite(rel_std_val):
        return None

    direct_metrics = dict(noise_metrics)
    direct_metric = float(base) * float(rel_std_val)
    direct_metrics["chi2_red_rescored_tau_3bin_debiased"] = direct_metric
    direct_metrics["chi2_red_rescored"] = direct_metric
    direct_metrics["chi2_red_trimmed"] = direct_metric
    direct_metrics["chi2_red"] = direct_metric
    return _layered_internal_metric_from_noise_metrics(
        direct_metrics,
        cost_raw=_float_or_none(plan.get("cost_raw")),
        fit_structure_levels=_plan_fit_structure_levels(plan),
    )


def _plan_fit_structure_levels(plan: Dict[str, object]) -> Tuple[int, ...]:
    length_model = plan.get("length_model")
    raw = None
    if isinstance(length_model, dict):
        raw = length_model.get("fit_structure_levels")
    if raw is None:
        raw = plan.get("fit_structure_levels")
    try:
        return _parse_fit_structure_levels(raw, label="fit_structure")
    except ValueError:
        return tuple()


def _compute_score_ui_layered(plan: Dict[str, object]) -> Tuple[float, float]:
    noise_metrics = _plan_noise_metrics(plan)
    nm = noise_metrics if isinstance(noise_metrics, dict) else {}

    cost_raw = _float_or_none(plan.get("cost_raw"))
    tau_mad_mm = _float_or_none(nm.get("tau_mad_mm"))
    n_trim = _float_or_none(nm.get("n_obs_trimmed"))
    m_base = _layered_internal_metric_from_noise_metrics(
        nm,
        cost_raw=cost_raw,
        fit_structure_levels=_plan_fit_structure_levels(plan),
    )
    m_risk = _plan_trimmed_risk_metric(plan)
    m_layered = m_risk if m_risk is not None else m_base
    critical_nonfinite = (
        m_layered is None or cost_raw is None or tau_mad_mm is None or n_trim is None
    )
    m = float(m_layered) if m_layered is not None else float("nan")

    score_ui = float("nan")
    if np.isfinite(m) and m >= 0.0:
        score_ui = _SCORE_UI_LAYERED_MAP_MULT * (m / _SCORE_UI_LAYERED_MAP_SCALE) ** _SCORE_UI_LAYERED_MAP_EXP

    if critical_nonfinite or not np.isfinite(score_ui) or (cost_raw is not None and cost_raw > 5.0):
        score_ui = max(score_ui if np.isfinite(score_ui) else _SCORE_UI_HARD_FAIL, _SCORE_UI_HARD_FAIL)

    return float(score_ui), float(m)


def _plan_score_ui(plan: Dict[str, object]) -> Tuple[float, float, str]:
    # score_ui is a calibrated fit-quality score, not a literal chi-square.
    if _plan_uses_layered_score(plan):
        score_ui, m_layered = _compute_score_ui_layered(plan)
        rank_score = _layered_rank_score_from_internal_metric(
            m_layered,
            cost_raw=_float_or_none(plan.get("cost_raw")),
        )
        if not np.isfinite(rank_score):
            rank_score = score_ui if np.isfinite(score_ui) else float("inf")
        return float(score_ui), float(rank_score), "layered-calibrated"

    score_ui = _plan_primary_cost(plan)
    rank_score = score_ui if np.isfinite(score_ui) else float("inf")
    return float(score_ui), float(rank_score), "standard-noise"


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


def _solution_quality_message(score_ui: Optional[float]) -> str:
    if score_ui is None or not np.isfinite(score_ui):
        return "Interpretation: Quality score is unavailable."
    score = float(score_ui)
    if score < 2.0:
        return "Interpretation: A Quality score below 2 indicates a near-perfect fit."
    if score < 5.0:
        return "Interpretation: A Quality score between 2 and 5 is a useful fit."
    if score < 10.0:
        return "Interpretation: A Quality score between 5 and 10 is borderline but often usable."
    return "Interpretation: A Quality score above 10 is concerning; this fit is probably bad."


def _solution_quality_label(score_ui: Optional[float]) -> str:
    if score_ui is None or not np.isfinite(score_ui):
        return "unknown"
    score = float(score_ui)
    if score < 2.0:
        return "ideal"
    if score < 5.0:
        return "good"
    if score < 10.0:
        return "usable"
    return "concerning"


def _ordinal(n: int) -> str:
    n = int(n)
    if 10 <= (n % 100) <= 20:
        suffix = "th"
    else:
        suffix = {1: "st", 2: "nd", 3: "rd"}.get(n % 10, "th")
    return f"{n}{suffix}"


__all__ = [name for name in globals() if not name.startswith("__")]
