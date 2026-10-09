"""Versioned JSON checkpoints for offline calibration stage experiments.

Only known numeric/container types and the three calibration value classes are
reconstructed. Artifacts contain data, never executable Python or pickle.
"""
from __future__ import annotations

from dataclasses import fields, is_dataclass
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from autocal import extended_reference
from autocal.active_learning import SweepConfig
from autocal.spool_model import SpoolModelParams, WinchSpoolModel

_VALUE_TYPES = {cls.__name__: cls for cls in (SweepConfig, SpoolModelParams, WinchSpoolModel)}


def _encode(value):
    if isinstance(value, np.ndarray):
        return {"$type": "array", "value": _encode(value.tolist())}
    if isinstance(value, np.generic):
        return _encode(value.item())
    if isinstance(value, Path):
        return {"$type": "path", "value": str(value)}
    if is_dataclass(value) and type(value).__name__ in _VALUE_TYPES:
        return {"$type": type(value).__name__, "value": {
            field.name: _encode(getattr(value, field.name)) for field in fields(value)}}
    if isinstance(value, tuple):
        return {"$type": "tuple", "value": [_encode(item) for item in value]}
    if isinstance(value, list):
        return [_encode(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _encode(item) for key, item in value.items()}
    if isinstance(value, float) and not math.isfinite(value):
        return {"$type": "float", "value": str(value)}
    return value


def _decode(value):
    if isinstance(value, list):
        return [_decode(item) for item in value]
    if not isinstance(value, dict):
        return value
    if "$type" not in value:
        return {key: _decode(item) for key, item in value.items()}
    kind, data = value["$type"], value["value"]
    if kind == "array":
        return np.asarray(_decode(data))
    if kind == "path":
        return Path(data)
    if kind == "tuple":
        return tuple(_decode(item) for item in data)
    if kind == "float":
        return float(data)
    if kind in _VALUE_TYPES:
        return _VALUE_TYPES[kind](**_decode(data))
    raise ValueError(f"Unknown calibration artifact type: {kind}")


def write_stage_artifact(path: Path, kind: str, payload: dict) -> None:
    root = Path(__file__).resolve().parents[1]
    sources = sorted((root / "autocal").glob("*.py")) + [root / "autocal/tools/replay_stage.py"]
    envelope = {
        "format_version": 1, "kind": kind,
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "source_sha256": {str(source.relative_to(root)): hashlib.sha256(source.read_bytes()).hexdigest() for source in sources},
        "payload": _encode(payload),
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(envelope, allow_nan=False, separators=(",", ":")), encoding="utf-8")
    temporary.replace(path)
    extended_reference.artifact(path)


def read_stage_artifact(path: Path, *, expected_kind: str | None = None) -> dict:
    envelope = json.loads(Path(path).read_text(encoding="utf-8"))
    if envelope.get("format_version") != 1:
        raise ValueError("Unsupported calibration artifact version")
    if expected_kind is not None and envelope.get("kind") != expected_kind:
        raise ValueError(f"Expected {expected_kind} artifact, got {envelope.get('kind')}")
    return {**envelope, "payload": _decode(envelope["payload"])}
