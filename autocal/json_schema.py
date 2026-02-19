from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable, Optional

from autocal.sweep_types import MachineConfig, MachineType

SchemaName = str

SCHEMA_FILES = {
    "sweep_dataset": "sweep_dataset.schema.json",
    "calibration_result": "calibration_result.schema.json",
    "fit_sidecar": "fit_sidecar.schema.json",
    "full_auto_log_entry": "full_auto_log_entry.schema.json",
    "synthetic_dataset_entry": "synthetic_dataset_entry.schema.json",
}


def schema_path(schema: SchemaName) -> Path:
    if schema not in SCHEMA_FILES:
        raise ValueError(f"Unknown schema: {schema}")
    return Path(__file__).resolve().parent / "schemas" / SCHEMA_FILES[schema]


def load_json_file(path: Path, *, schema: Optional[SchemaName] = None) -> Any:
    with Path(path).open("r", encoding="utf-8") as fh:
        payload = json.load(fh)
    validate_payload(payload, schema=schema, source=str(path))
    return payload


def write_json_file(path: Path, payload: Any, *, schema: Optional[SchemaName] = None, indent: int = 2) -> None:
    validate_payload(payload, schema=schema, source=str(path))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=indent)


def append_jsonl_line(path: Path, payload: Any, *, schema: Optional[SchemaName] = None) -> None:
    validate_payload(payload, schema=schema, source=str(path))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, separators=(",", ":")) + "\n")


def parse_jsonl_line(line: str, *, schema: Optional[SchemaName] = None, source: str = "<jsonl>") -> Any:
    payload = json.loads(line)
    validate_payload(payload, schema=schema, source=source)
    return payload


def validate_payload(payload: Any, *, schema: Optional[SchemaName], source: str = "<payload>") -> None:
    if schema is None:
        return
    if schema == "sweep_dataset":
        _validate_sweep_dataset(payload, source=source)
        return
    if schema == "calibration_result":
        _validate_calibration_result(payload, source=source)
        return
    if schema == "fit_sidecar":
        _validate_fit_sidecar(payload, source=source)
        return
    if schema == "full_auto_log_entry":
        _validate_full_auto_log_entry(payload, source=source)
        return
    if schema == "synthetic_dataset_entry":
        _validate_synthetic_dataset_entry(payload, source=source)
        return
    raise ValueError(f"Unknown schema: {schema}")


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _require(cond: bool, source: str, message: str) -> None:
    if not cond:
        raise ValueError(f"{source}: {message}")


def _require_keys(obj: dict[str, Any], keys: Iterable[str], source: str) -> None:
    for key in keys:
        _require(key in obj, source, f"missing required key '{key}'")


def _validate_sweep_dataset(payload: Any, *, source: str) -> None:
    _require(isinstance(payload, dict), source, "sweep_dataset must be an object")
    _require_keys(payload, ("version", "machine_type", "sweeps"), source)
    _require(isinstance(payload["version"], str), source, "version must be a string")
    _require(isinstance(payload["machine_type"], str), source, "machine_type must be a string")
    _require(isinstance(payload["sweeps"], list), source, "sweeps must be a list")

    try:
        machine_type = MachineType(str(payload["machine_type"]))
    except ValueError as exc:
        raise ValueError(f"{source}: unsupported machine_type '{payload['machine_type']}'") from exc
    config = MachineConfig.from_type(machine_type)

    if "num_anchors" in payload:
        _require(
            isinstance(payload["num_anchors"], int) and payload["num_anchors"] == config.num_anchors,
            source,
            f"num_anchors must be {config.num_anchors} for {machine_type.value}",
        )
    if "dimensions" in payload:
        _require(
            isinstance(payload["dimensions"], int) and payload["dimensions"] == config.dimensions,
            source,
            f"dimensions must be {config.dimensions} for {machine_type.value}",
        )

    for idx, sweep in enumerate(payload["sweeps"]):
        label = f"{source}: sweeps[{idx}]"
        _require(isinstance(sweep, dict), label, "must be an object")
        _require_keys(
            sweep,
            ("id", "fixed_anchors", "fixed_lengths", "drive_anchor", "sensor_anchor", "data_points"),
            label,
        )
        _require(isinstance(sweep["id"], str), label, "id must be a string")
        _require(isinstance(sweep["fixed_anchors"], list), label, "fixed_anchors must be a list")
        _require(isinstance(sweep["fixed_lengths"], list), label, "fixed_lengths must be a list")
        _require(len(sweep["fixed_anchors"]) == len(sweep["fixed_lengths"]), label, "fixed lengths count mismatch")
        _require(isinstance(sweep["data_points"], list), label, "data_points must be a list")
        _require(isinstance(sweep["drive_anchor"], int), label, "drive_anchor must be an integer")
        _require(isinstance(sweep["sensor_anchor"], int), label, "sensor_anchor must be an integer")

        all_roles: list[int] = []
        for anchor_idx in sweep["fixed_anchors"]:
            _require(isinstance(anchor_idx, int) and anchor_idx >= 0, label, "fixed_anchors must be non-negative integers")
            _require(anchor_idx < config.num_anchors, label, "fixed_anchors out of range")
            all_roles.append(anchor_idx)

        for length in sweep["fixed_lengths"]:
            _require(_is_number(length), label, "fixed_lengths must be finite numbers")

        drive_anchor = sweep["drive_anchor"]
        sensor_anchor = sweep["sensor_anchor"]
        _require(0 <= drive_anchor < config.num_anchors, label, "drive_anchor out of range")
        _require(0 <= sensor_anchor < config.num_anchors, label, "sensor_anchor out of range")
        all_roles.extend([drive_anchor, sensor_anchor])
        _require(len(set(all_roles)) == len(all_roles), label, "duplicate anchor roles in sweep")
        if config.carrying_anchors:
            _require(sensor_anchor not in config.carrying_anchors, label, "carrying anchors cannot be sensor")

        for jdx, point in enumerate(sweep["data_points"]):
            plabel = f"{label}: data_points[{jdx}]"
            _require(isinstance(point, dict), plabel, "must be an object")
            _require("l_drive" in point and "l_sensor" in point, plabel, "missing l_drive/l_sensor")
            _require(_is_number(point["l_drive"]), plabel, "l_drive must be a finite number")
            _require(_is_number(point["l_sensor"]), plabel, "l_sensor must be a finite number")


def _validate_calibration_result(payload: Any, *, source: str) -> None:
    _require(isinstance(payload, dict), source, "calibration_result must be an object")
    _require_keys(payload, ("machine_type", "anchors", "cost", "success", "gcode"), source)
    _require(isinstance(payload["machine_type"], str), source, "machine_type must be a string")
    _require(isinstance(payload["anchors"], list), source, "anchors must be a list")
    _require(isinstance(payload["success"], bool), source, "success must be a boolean")
    _require(isinstance(payload["gcode"], str), source, "gcode must be a string")
    _require(
        isinstance(payload["cost"], (int, float)) and not isinstance(payload["cost"], bool),
        source,
        "cost must be numeric",
    )

    for idx, anchor in enumerate(payload["anchors"]):
        label = f"{source}: anchors[{idx}]"
        _require(isinstance(anchor, list), label, "anchor must be a list")
        _require(len(anchor) in (2, 3), label, "anchor must have 2 or 3 components")
        for value in anchor:
            _require(_is_number(value), label, "anchor values must be finite numbers")


def _validate_fit_sidecar(payload: Any, *, source: str) -> None:
    _require(isinstance(payload, dict), source, "fit_sidecar must be an object")
    _require_keys(payload, ("source", "residual_threshold", "min_points", "fitted_ellipses"), source)
    _require(isinstance(payload["source"], str), source, "source must be a string")
    _require(_is_number(payload["residual_threshold"]), source, "residual_threshold must be a finite number")
    _require(isinstance(payload["min_points"], int) and payload["min_points"] > 0, source, "min_points must be a positive integer")
    _require(isinstance(payload["fitted_ellipses"], list), source, "fitted_ellipses must be a list")
    for idx, fit in enumerate(payload["fitted_ellipses"]):
        label = f"{source}: fitted_ellipses[{idx}]"
        _require(isinstance(fit, dict), label, "must be an object")
        _require("valid" in fit and isinstance(fit["valid"], bool), label, "valid must be a boolean")
        if "coefficients" in fit:
            coeffs = fit["coefficients"]
            _require(isinstance(coeffs, dict), label, "coefficients must be an object")
            for key in ("A", "B", "C", "D", "E", "F"):
                _require(key in coeffs and _is_number(coeffs[key]), label, f"coefficients.{key} must be a finite number")


def _validate_full_auto_log_entry(payload: Any, *, source: str) -> None:
    _require(isinstance(payload, dict), source, "full_auto_log_entry must be an object")
    _require_keys(payload, ("timestamp", "dataset"), source)
    _require(isinstance(payload["timestamp"], str), source, "timestamp must be a string")
    _require(isinstance(payload["dataset"], str), source, "dataset must be a string")
    if "event" in payload:
        _require(isinstance(payload["event"], str), source, "event must be a string")
    if "iteration" in payload:
        _require(isinstance(payload["iteration"], int) and payload["iteration"] >= 0, source, "iteration must be a non-negative integer")
    if "runs" in payload:
        _require(isinstance(payload["runs"], list), source, "runs must be a list")


def _validate_synthetic_dataset_entry(payload: Any, *, source: str) -> None:
    _require(isinstance(payload, dict), source, "synthetic_dataset_entry must be an object")
    _require_keys(payload, ("dataset", "geometry", "anchors", "real_xyz", "motor_samples"), source)
    _require(isinstance(payload["dataset"], str), source, "dataset must be a string")
    _require(isinstance(payload["geometry"], str), source, "geometry must be a string")
    _require(isinstance(payload["anchors"], list), source, "anchors must be a list")
    _require(isinstance(payload["real_xyz"], list), source, "real_xyz must be a list")
    _require(isinstance(payload["motor_samples"], list), source, "motor_samples must be a list")
    if "anchor_set" in payload:
        _require(isinstance(payload["anchor_set"], int) and payload["anchor_set"] >= 0, source, "anchor_set must be a non-negative integer")
    for key in ("anchors", "real_xyz"):
        for idx, point in enumerate(payload[key]):
            label = f"{source}: {key}[{idx}]"
            _require(isinstance(point, list) and len(point) == 3, label, "point must be [x, y, z]")
            for value in point:
                _require(_is_number(value), label, "point coordinates must be finite numbers")
    for idx, sample in enumerate(payload["motor_samples"]):
        label = f"{source}: motor_samples[{idx}]"
        _require(isinstance(sample, list), label, "sample must be a list")
        for value in sample:
            _require(_is_number(value), label, "sample values must be finite numbers")
