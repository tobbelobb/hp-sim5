from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import List, Union

from autocal.sweep_types import (
    DataPoint,
    MachineConfig,
    MachineType,
    Sweep,
    SweepDataset,
    SweepMetadata,
)


def validate_dataset(dataset: SweepDataset) -> List[str]:
    """Collect validation errors across all sweeps."""
    errors: List[str] = []
    for sweep in dataset.sweeps:
        for err in sweep.validate(dataset.machine_config):
            errors.append(f"{sweep.id}: {err}")
    return errors


def save_sweep_dataset(dataset: SweepDataset, path: Union[str, Path]) -> None:
    """Save dataset to JSON file."""
    validation_errors = validate_dataset(dataset)
    if validation_errors:
        raise ValueError(f"Invalid sweep dataset: {validation_errors}")

    data = {
        "version": dataset.version,
        "machine_type": dataset.machine_config.machine_type.value,
        "num_anchors": dataset.machine_config.num_anchors,
        "dimensions": dataset.machine_config.dimensions,
        "timestamp": dataset.timestamp,
        "sweeps": [
            {
                "id": s.id,
                "fixed_anchors": s.fixed_anchors,
                "fixed_lengths": s.fixed_lengths,
                "drive_anchor": s.drive_anchor,
                "sensor_anchor": s.sensor_anchor,
                "data_points": [
                    {
                        "l_drive": p.l_drive,
                        "l_sensor": p.l_sensor,
                        "timestamp_ms": p.timestamp_ms,
                        "raw_angles_deg": p.raw_angles_deg,
                    }
                    for p in s.data_points
                ],
                "metadata": {
                    "feed_rate": s.metadata.feed_rate,
                    "torque": s.metadata.torque,
                    "settle_ms": s.metadata.settle_ms,
                    "sample_rate_hz": s.metadata.sample_rate_hz,
                },
            }
            for s in dataset.sweeps
        ],
    }

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def load_sweep_dataset(path: Union[str, Path]) -> SweepDataset:
    """Load dataset from JSON file."""
    with Path(path).open("r", encoding="utf-8") as f:
        data = json.load(f)

    machine_type = MachineType(data["machine_type"])
    config = MachineConfig.from_type(machine_type)
    if data.get("num_anchors") not in (None, config.num_anchors):
        raise ValueError("num_anchors in file does not match machine_type")
    if data.get("dimensions") not in (None, config.dimensions):
        raise ValueError("dimensions in file does not match machine_type")

    sweeps = []
    for s in data["sweeps"]:
        data_points = [
            DataPoint(
                l_drive=p["l_drive"],
                l_sensor=p["l_sensor"],
                timestamp_ms=p.get("timestamp_ms"),
                raw_angles_deg=p.get("raw_angles_deg"),
            )
            for p in s["data_points"]
        ]
        metadata = SweepMetadata(**s.get("metadata", {}))
        sweeps.append(
            Sweep(
                id=s["id"],
                fixed_anchors=s["fixed_anchors"],
                fixed_lengths=s["fixed_lengths"],
                drive_anchor=s["drive_anchor"],
                sensor_anchor=s["sensor_anchor"],
                data_points=data_points,
                metadata=metadata,
            )
        )

    timestamp = data.get("timestamp", datetime.now().isoformat())
    dataset = SweepDataset(
        version=data["version"],
        machine_config=config,
        timestamp=timestamp,
        sweeps=sweeps,
    )

    validation_errors = validate_dataset(dataset)
    if validation_errors:
        raise ValueError(f"Invalid sweep dataset: {validation_errors}")

    return dataset

