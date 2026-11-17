"""Experiment-only integration of a measurement-only fifth range into HP5 autocal.

This module is imported by the treatment launcher and monkey-patches the
production point/range fitter at run time.  It never edits production files and
is inert for datasets without the explicit extra-sensor policy metadata.

Design
------
* The collector (see build_extra_range_collector.py) writes, for every measured
  point, `extra_sensor_anchor` and `extra_sensor_force_n`, plus a per-sweep
  `extra_sensor_policy`.  The fitter never infers a force mode from a path or a
  file name.
* A predeclared, anchor-free response gate decides per sweep whether the extra
  range may enter the fit.  It compares the extra encoder's travel and smooth
  response to the drive motion with the sensor line's travel.  A frozen line
  with a moving sweep is rejected; a sweep that is genuinely almost insensitive
  to that line is admitted with a low-information flag.
* The production two-stage fit is kept: a joint anchor+home-offset fit, then a
  shifted primary fit.  The static tension solve is generalised to all lines
  with one known-force row per torque-controlled line (sensor and extra).  For
  baseline data (no policy) the original functions are called unchanged.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import autocal.pose_fit as pose_fit
import autocal.fit_stage as fit_stage
import autocal.planning_pass as planning_pass
import autocal.rough_bootstrap as rough_bootstrap
import autocal.predictive_validation as predictive_validation
from autocal.flex import FlexModel

# ---------------------------------------------------------------------------
# Predeclared gate constants (frozen before the full runs).
# ---------------------------------------------------------------------------
MIN_SEGMENT_POINTS = 4
MIN_DRIVE_TRAVEL_MM = 20.0
MIN_SENSOR_TRAVEL_MM = 10.0
FROZEN_ABS_MM = 0.3
LOW_INFO_ABS_MM = 3.0
LOW_INFO_FRACTION_OF_SENSOR = 0.02
MIN_COUPLING = 0.5
SHAPE_RMS_MM = 0.5
SHAPE_MAX_MM = 1.0
GATE_VERSION = "extra-range-gate-v1"

POLICY_MODE = "measurement_only_sensor_force"

_ORIGINAL_LINE_LENGTHS = pose_fit.dataset_line_lengths
_ORIGINAL_STRETCH = pose_fit.dataset_stretch
_ORIGINAL_CORRECTED = pose_fit.stretch_corrected_lengths
_ORIGINAL_SHIFT = pose_fit.shift_dataset_home_offsets
_ORIGINAL_FIT_STAGE_FIT = fit_stage.fit_ellipse_dataset
_ORIGINAL_FIT_STAGE_LOAD = fit_stage._load_json
_ORIGINAL_SUGGESTED_COLLECT = planning_pass._suggested_collect_command

_VALIDITY_CACHE: Dict[str, dict] = {}
_INSTALLED = False


# ---------------------------------------------------------------------------
# Dataset helpers
# ---------------------------------------------------------------------------
def _policy(dataset: dict) -> Optional[dict]:
    policy = dataset.get("extra_sensor_policy")
    if isinstance(policy, dict):
        return policy
    for sweep in dataset.get("sweeps", []):
        if not isinstance(sweep, dict):
            continue
        metadata = sweep.get("metadata") or {}
        if metadata.get("extra_sensor_policy") or metadata.get("extra_sensor_anchors"):
            return {"mode": metadata.get("extra_sensor_policy", POLICY_MODE)}
        for point in sweep.get("data_points", []):
            if isinstance(point, dict) and point.get("extra_sensor_anchor") is not None:
                return {"mode": point.get("extra_sensor_force_mode", POLICY_MODE)}
    return None


def has_extra_policy(dataset: dict) -> bool:
    return _policy(dataset) is not None


def sweep_extra_anchor(dataset: dict, sweep: dict) -> Optional[int]:
    """Physical axis that is neither fixed, drive, sensor nor force-forbidden."""
    metadata = sweep.get("metadata") or {}
    anchors = metadata.get("extra_sensor_anchors")
    if isinstance(anchors, list) and anchors:
        try:
            return int(anchors[0])
        except (TypeError, ValueError):
            pass
    for point in sweep.get("data_points", []):
        value = point.get("extra_sensor_anchor") if isinstance(point, dict) else None
        if value is not None:
            try:
                return int(value)
            except (TypeError, ValueError):
                return None
    used = {int(a) for a in sweep.get("fixed_anchors", [])}
    used.add(int(sweep["drive_anchor"]))
    used.add(int(sweep["sensor_anchor"]))
    spare = [a for a in range(int(dataset["num_anchors"])) if a not in used]
    return spare[0] if spare else None


def _angle(point: dict, axis: int, use_noise_mean: bool) -> float:
    angles = point.get("mu") if (use_noise_mean and point.get("mu") is not None) else point.get("raw_angles_deg")
    return float(angles[axis])


def _segments(points: Sequence[dict]) -> List[List[dict]]:
    segments: List[List[dict]] = []
    for point in points:
        if not segments or (point.get("step_index") == 0 and segments[-1]):
            segments.append([])
        segments[-1].append(point)
    return [segment for segment in segments if segment]


def _cubic_residual(x: np.ndarray, y: np.ndarray) -> Tuple[float, float]:
    if x.size < 4 or np.ptp(x) <= 1e-9:
        return 0.0, 0.0
    basis = np.column_stack([x ** power for power in range(4)])
    coef, *_ = np.linalg.lstsq(basis, y, rcond=None)
    residual = y - basis @ coef
    return float(np.sqrt(np.mean(residual ** 2))), float(np.max(np.abs(residual)))


def _gate_segment(points: Sequence[dict], extra: int, mm_per_degree: Sequence[float],
                  use_noise_mean: bool) -> dict:
    drive = np.asarray([float(p["l_drive"]) for p in points])
    sensor = np.asarray([float(p["l_sensor"]) for p in points])
    extra_length = np.asarray([_angle(p, extra, use_noise_mean) * float(mm_per_degree[extra]) for p in points])
    drive_travel = float(np.ptp(drive))
    sensor_travel = float(np.ptp(sensor))
    extra_travel = float(np.ptp(extra_length))
    correlation = None
    if extra_travel > 1e-9 and drive_travel > 1e-9:
        correlation = float(np.corrcoef(extra_length, drive)[0, 1])
    rms, maximum = _cubic_residual(drive, extra_length)
    record = {
        "points": len(points),
        "drive_travel_mm": drive_travel,
        "sensor_travel_mm": sensor_travel,
        "extra_travel_mm": extra_travel,
        "extra_drive_correlation": correlation,
        "cubic_rms_mm": rms,
        "cubic_max_mm": maximum,
    }
    if len(points) < MIN_SEGMENT_POINTS or drive_travel < MIN_DRIVE_TRAVEL_MM:
        record.update(status="untestable_short", accepted=True)
        return record
    if sensor_travel < MIN_SENSOR_TRAVEL_MM and extra_travel < FROZEN_ABS_MM:
        # Neither reference line moves: the sweep is simply insensitive to this
        # axis.  Do not call that a slack failure.
        record.update(status="untestable_insensitive", accepted=True)
        return record
    if extra_travel < FROZEN_ABS_MM:
        record.update(status="rejected_frozen_or_slack", accepted=False)
        return record
    if correlation is None or abs(correlation) < MIN_COUPLING:
        record.update(status="rejected_no_coupling", accepted=False)
        return record
    if rms > SHAPE_RMS_MM or maximum > SHAPE_MAX_MM:
        record.update(status="rejected_shape_or_slack", accepted=False)
        return record
    low_information = extra_travel < max(LOW_INFO_ABS_MM, LOW_INFO_FRACTION_OF_SENSOR * sensor_travel)
    record.update(status="accepted_low_information" if low_information else "accepted", accepted=True)
    return record


def evaluate_validity(dataset: dict, *, use_noise_mean: bool = True) -> Tuple[dict, dict]:
    """Return a per-sweep validity mask and the gate evidence."""
    mask: Dict[str, dict] = {}
    evidence: Dict[str, dict] = {
        "gate_version": GATE_VERSION,
        "constants": {
            "min_segment_points": MIN_SEGMENT_POINTS,
            "min_drive_travel_mm": MIN_DRIVE_TRAVEL_MM,
            "min_sensor_travel_mm": MIN_SENSOR_TRAVEL_MM,
            "frozen_abs_mm": FROZEN_ABS_MM,
            "low_info_abs_mm": LOW_INFO_ABS_MM,
            "low_info_fraction_of_sensor": LOW_INFO_FRACTION_OF_SENSOR,
            "min_coupling": MIN_COUPLING,
            "shape_rms_mm": SHAPE_RMS_MM,
            "shape_max_mm": SHAPE_MAX_MM,
        },
        "sweeps": {},
    }
    mm_per_degree = (dataset.get("config") or {}).get("mm_per_degree") or []
    policy = _policy(dataset)
    for sweep in dataset.get("sweeps", []):
        sweep_id = str(sweep.get("id", ""))
        metadata = sweep.get("metadata") or {}
        has_policy = bool(metadata.get("extra_sensor_policy") or metadata.get("extra_sensor_anchors")) or (
            policy is not None and any(
                isinstance(p, dict) and p.get("extra_sensor_anchor") is not None
                for p in sweep.get("data_points", []))
        )
        if not has_policy:
            mask[sweep_id] = {"accepted": False, "status": "no_policy"}
            evidence["sweeps"][sweep_id] = mask[sweep_id]
            continue
        extra = sweep_extra_anchor(dataset, sweep)
        if extra is None or extra >= len(mm_per_degree) or not np.isfinite(mm_per_degree[extra]):
            mask[sweep_id] = {"accepted": False, "status": "no_extra_axis_or_scale", "axis": extra}
            evidence["sweeps"][sweep_id] = mask[sweep_id]
            continue
        segments = [
            _gate_segment(segment, extra, mm_per_degree, use_noise_mean)
            for segment in _segments(sweep.get("data_points", []))
        ]
        rejected = [segment for segment in segments if not segment["accepted"]]
        testable = [segment for segment in segments if not segment["status"].startswith("untestable")]
        if rejected:
            status = rejected[0]["status"]
            accepted = False
        elif not testable:
            status = "accepted_untestable"
            accepted = True
        else:
            status = "accepted"
            accepted = True
        record = {"accepted": accepted, "status": status, "axis": int(extra), "segments": segments}
        mask[sweep_id] = record
        evidence["sweeps"][sweep_id] = record
    evidence["accepted"] = sum(1 for record in mask.values() if record.get("accepted"))
    evidence["rejected"] = sum(1 for record in mask.values() if record.get("status", "").startswith("rejected"))
    evidence["no_policy"] = sum(1 for record in mask.values() if record.get("status") == "no_policy")
    return mask, evidence


# ---------------------------------------------------------------------------
# Patched loader / stretch model
# ---------------------------------------------------------------------------
def _accepted(dataset: dict, sweep_id: str) -> bool:
    mask = dataset.get("_extra_range_valid") or {}
    return bool((mask.get(str(sweep_id)) or {}).get("accepted"))


def line_lengths(dataset: dict, *, use_noise_mean: bool = True) -> dict:
    base = _ORIGINAL_LINE_LENGTHS(dataset, use_noise_mean=use_noise_mean)
    if not has_extra_policy(dataset):
        return base
    lengths = base["lengths_mm"].copy()
    mm_per_degree = (dataset.get("config") or {}).get("mm_per_degree") or []
    index = 0
    for sweep in dataset.get("sweeps", []):
        extra = sweep_extra_anchor(dataset, sweep)
        valid = extra is not None and _accepted(dataset, sweep.get("id", ""))
        for point in sweep.get("data_points", []):
            if valid and extra < len(mm_per_degree) and np.isfinite(mm_per_degree[extra]):
                lengths[index, extra] = _angle(point, extra, use_noise_mean) * float(mm_per_degree[extra])
            index += 1
    return {"lengths_mm": lengths, "sweep_index": base["sweep_index"]}


def _general_stretch(dataset: dict) -> Optional[dict]:
    config = dataset.get("config") or {}
    m666 = config.get("m666_before_data_collection") or {}
    num_anchors = int(dataset["num_anchors"])
    model = FlexModel.from_m666(m666, num_axes=num_anchors)
    if model is None:
        return None
    points = [point for sweep in dataset.get("sweeps", []) for point in sweep.get("data_points", [])]
    if not points:
        return None
    policy = _policy(dataset) or {}
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    weight = float(m666.get("W", 0.0)) * 9.81
    force_anchors: List[List[int]] = []
    force_values: List[List[float]] = []
    for sweep in dataset.get("sweeps", []):
        extra = sweep_extra_anchor(dataset, sweep)
        for point in sweep.get("data_points", []):
            sensor = int(point.get("source_sensor_anchor", sweep["sensor_anchor"]))
            sensor_force = point.get("sensor_force_n")
            if not isinstance(sensor_force, (int, float)) or not np.isfinite(sensor_force):
                return None
            rows = [sensor]
            values = [float(sensor_force)]
            if extra is not None:
                value = point.get("extra_sensor_force_n")
                if not isinstance(value, (int, float)) or not np.isfinite(value):
                    value = float(sensor_force) if policy.get("mode") == POLICY_MODE else passive
                rows.append(int(extra))
                values.append(float(value))
            force_anchors.append(rows)
            force_values.append(values)
    return {
        "model": model,
        "force_anchors": np.asarray(force_anchors, dtype=int),
        "force_values": np.asarray(force_values, dtype=float),
        "weight": weight,
        "passive": passive,
        # Production-compatible fields for any downstream code that reads them.
        "sensors": np.asarray([row[0] for row in force_anchors], dtype=int),
        "forces": np.asarray([row[0] for row in force_values], dtype=float),
    }


def stretch(dataset: dict):
    """Production stretch for baseline data; general two-torque model for policy data."""
    policy = _policy(dataset)
    if not policy:
        return _ORIGINAL_STRETCH(dataset)
    if dataset.get("_extra_range_force_stretch"):
        # Test/preflight hook only: exercise the general two-torque model on
        # short constant-force recordings that are not equal-strain profiled.
        return _general_stretch(dataset)
    from autocal.equal_strain import is_equal_strain
    if int(dataset["num_anchors"]) <= int(dataset["dimensions"]) + 1 or not is_equal_strain(dataset):
        # The production fitter keeps historical/rough/profiled-constant data rigid.
        return None
    return _general_stretch(dataset)


def _static_tensions(anchors: np.ndarray, positions: np.ndarray, absolute: np.ndarray,
                     measured: np.ndarray, stretch_model: dict) -> np.ndarray:
    points, num_anchors = absolute.shape
    to_anchor = anchors[None, :, :] - positions[:, None, :]
    distance = np.maximum(np.linalg.norm(to_anchor, axis=2), 1e-9)
    directions = (to_anchor / distance[:, :, None]).transpose(0, 2, 1)  # (p, 3, n)
    force_anchors = np.asarray(stretch_model["force_anchors"], dtype=int)
    force_values = np.asarray(stretch_model["force_values"], dtype=float).copy()
    rows = []
    for j in range(force_anchors.shape[1]):
        selector = np.zeros((points, num_anchors))
        selector[np.arange(points), force_anchors[:, j]] = 1.0
        rows.append(selector)
    # Every unmeasured line that has no explicit torque is compliant at passive force.
    for axis in range(num_anchors):
        unmeasured = ~measured[:, axis]
        force_controlled = np.any(force_anchors == axis, axis=1)
        if np.any(unmeasured & ~force_controlled):
            selector = np.zeros((points, num_anchors))
            selector[:, axis] = 1.0
            rows.append(selector)
            force_values = np.column_stack([force_values, np.full(points, float(stretch_model["passive"]))])
    selector = np.stack(rows, axis=1) + np.zeros((points, len(rows), 1))
    matrix = np.concatenate([directions, selector], axis=1)
    load = np.zeros((points, matrix.shape[1]))
    load[:, 2] = float(stretch_model["weight"])
    load[:, 3:] = force_values
    transpose = matrix.transpose(0, 2, 1)
    normal = transpose @ matrix + 1e-12 * np.eye(num_anchors)
    return np.linalg.solve(normal, transpose @ load[:, :, None])[:, :, 0]


def corrected(anchors, positions, absolute, measured, stretch_model):
    if not isinstance(stretch_model, dict) or "force_anchors" not in stretch_model:
        return _ORIGINAL_CORRECTED(anchors, positions, absolute, measured, stretch_model)
    tensions = _static_tensions(anchors, positions, absolute, measured, stretch_model)
    return np.column_stack([
        stretch_model["model"].corrected_distance_mm(absolute[:, axis], tensions[:, axis], axis=axis)
        for axis in range(anchors.shape[0])
    ])


def shift(dataset: dict, offsets: Sequence[float]) -> dict:
    shifted = _ORIGINAL_SHIFT(dataset, offsets)
    if not has_extra_policy(dataset):
        return shifted
    mm_per_degree = (dataset.get("config") or {}).get("mm_per_degree") or []
    offsets = np.asarray(offsets, dtype=float)
    for sweep_index, sweep in enumerate(dataset.get("sweeps", [])):
        extra = sweep_extra_anchor(dataset, sweep)
        if extra is None or extra >= len(mm_per_degree) or not np.isfinite(mm_per_degree[extra]):
            continue
        delta = float(offsets[extra]) / float(mm_per_degree[extra])
        if delta == 0.0:
            continue
        for point_index, point in enumerate(sweep.get("data_points", [])):
            for key in ("mu", "raw_angles_deg"):
                if point.get(key) is None:
                    continue
                values = list(shifted["sweeps"][sweep_index]["data_points"][point_index][key])
                values[extra] = float(values[extra]) + delta
                shifted["sweeps"][sweep_index]["data_points"][point_index][key] = values
    return shifted


# ---------------------------------------------------------------------------
# Fit wrapper and evidence sidecar
# ---------------------------------------------------------------------------
def _path_key(path) -> str:
    return str(Path(path).resolve())


def _load_json_with_validity(path, *args, **kwargs):
    dataset = _ORIGINAL_FIT_STAGE_LOAD(path, *args, **kwargs)
    mask = _VALIDITY_CACHE.get(_path_key(path))
    if mask is not None and isinstance(dataset, dict):
        dataset = dict(dataset)
        dataset["_extra_range_valid"] = mask
    return dataset


def validity_path(dataset_path: Path) -> Path:
    return Path(dataset_path).with_suffix(".extra_range_validity.json")


def _write_validity(dataset_path: Path, dataset: dict, mask: dict, evidence: dict) -> None:
    path = validity_path(dataset_path)
    record = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "dataset": str(Path(dataset_path).resolve()),
        "sweeps": len(dataset.get("sweeps", [])),
        "points": sum(len(s.get("data_points", [])) for s in dataset.get("sweeps", [])),
        "gate": evidence,
        "mask": mask,
    }
    path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    with path.with_suffix(".jsonl").open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, sort_keys=True) + "\n")


def fit_ellipse_dataset(dataset_path, **options):
    path = Path(dataset_path)
    dataset = _ORIGINAL_FIT_STAGE_LOAD(path)
    if isinstance(dataset, dict) and has_extra_policy(dataset):
        mask, evidence = evaluate_validity(dataset, use_noise_mean=bool(options.get("use_noise_mean", True)))
        _VALIDITY_CACHE[_path_key(path)] = mask
        _write_validity(path, dataset, mask, evidence)
    else:
        _VALIDITY_CACHE.pop(_path_key(path), None)
    return _ORIGINAL_FIT_STAGE_FIT(dataset_path, **options)


def _suggested_collect_command(cfg_path, cfg, **kwargs):
    """Keep production's command builder; only swap the collector CLI for treatment."""
    command = _ORIGINAL_SUGGESTED_COLLECT(cfg_path, cfg, **kwargs)
    collector = os.environ.get("HP5_EXTRA_RANGE_COLLECTOR")
    if collector and len(command) >= 2 and command[0] == "node":
        command = [command[0], collector, *command[2:]]
    return command


def install() -> None:
    """Patch the production fitter in this process only."""
    global _INSTALLED
    if _INSTALLED:
        return
    pose_fit.dataset_line_lengths = line_lengths
    pose_fit.dataset_stretch = stretch
    pose_fit.stretch_corrected_lengths = corrected
    pose_fit.shift_dataset_home_offsets = shift
    fit_stage.dataset_line_lengths = line_lengths
    fit_stage.shift_dataset_home_offsets = shift
    fit_stage._load_json = _load_json_with_validity
    planning_pass.dataset_line_lengths = line_lengths
    planning_pass.fit_ellipse_dataset = fit_ellipse_dataset
    planning_pass._suggested_collect_command = _suggested_collect_command
    rough_bootstrap.dataset_line_lengths = line_lengths
    predictive_validation.shift_dataset_home_offsets = shift
    _INSTALLED = True
