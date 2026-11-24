"""Discover stored experiments and read provenance from existing run directories."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
from typing import Any, Iterator

from .storage import MANIFEST_NAME, UPLOAD_PENDING_NAME, manifest_summary


def _object(data: Any) -> dict[str, Any]:
    return data if isinstance(data, dict) else {}


def _read_metadata(path: Path) -> dict[str, Any]:
    try:
        return _object(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        return {}


def _run_metadata(data: dict[str, Any]) -> dict[str, Any]:
    selected = (
        "kind", "machine", "status", "backend", "git_revision", "git_dirty", "finished_at",
        "wall_s", "simulated_s", "total_steps", "metrics", "recording_mode", "solver",
        "scene", "steps", "dt_s", "physics_engine", "summary", "tags", "investigation",
    )
    meta = {key: data[key] for key in selected if data.get(key) is not None}
    if data.get("machine_type"):
        meta.setdefault("machine", data["machine_type"])
    if data.get("label"):
        meta.setdefault("summary", data["label"])
    if data.get("commands_sha256"):
        meta.setdefault("kind", "recorded_experiment")
    return meta


def directory_metadata(source: Path) -> dict[str, Any]:
    """Carry forward known provenance when archiving or retrying a run."""
    spec = _read_metadata(source / "spec.json")
    stored = _read_metadata(source / MANIFEST_NAME)
    pending = _read_metadata(source / UPLOAD_PENDING_NAME)
    headless = min(source.glob("*.headless*/manifest.json"), default=None)
    return {
        **(_run_metadata(_read_metadata(headless)) if headless else {}),
        **_run_metadata(_read_metadata(source / "manifest.json")),
        **_object(spec.get("metadata")),
        **_run_metadata(spec),
        **_run_metadata(_read_metadata(source / "result.json")),
        **_object(stored.get("metadata")),
        **_object(pending.get("metadata")),
    }


def local_runs(root: Path, *, skip: set[Path]) -> Iterator[dict[str, Any]]:
    """Read recognized run manifests without opening recordings, datasets, or logs."""
    if root in skip:
        return
    for directory, subdirs, files in os.walk(root):
        path = Path(directory)
        subdirs[:] = [name for name in subdirs if not name.startswith(".") and path / name not in skip]
        if MANIFEST_NAME in files:
            manifest = _read_metadata(path / MANIFEST_NAME)
            if "experiment_id" not in manifest:
                continue
            row = manifest_summary(manifest)
            metadata_path = path / MANIFEST_NAME
            subdirs.clear()
        else:
            result = _read_metadata(path / "result.json") if "result.json" in files else {}
            manifest = _read_metadata(path / "manifest.json") if "manifest.json" in files else {}
            if result.get("experiment_id"):
                row = {"experiment_id": result["experiment_id"], **_run_metadata(directory_metadata(path))}
                metadata_path = path / "result.json"
                subdirs[:] = [name for name in subdirs if ".headless" not in name]
            elif manifest.get("commands_sha256") or manifest.get("kind") == "native_collection" or (
                manifest.get("machine_type") and manifest.get("backend") in ("headless-js", "native-python")
            ):
                run_name = path.parent.name if ".headless" in path.name else path.name
                row = {"experiment_id": manifest.get("run_id") or run_name, **_run_metadata(manifest)}
                metadata_path = path / "manifest.json"
            else:
                continue
        row.pop("metrics", None)
        try:
            modified_at = datetime.fromtimestamp(metadata_path.stat().st_mtime, timezone.utc).isoformat()
        except OSError:
            continue
        yield {**row, "metadata_path": str(metadata_path), "modified_at": modified_at}
