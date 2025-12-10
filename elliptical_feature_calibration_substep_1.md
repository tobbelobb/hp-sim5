# Substep 1: Sweep Data Structure & JSON Format

## Overview

Define the generalized data structures that support sweep-based calibration for all machine configurations (3-8 anchors, 2D and 3D). This substep establishes the foundation for data collection, storage, and exchange between components. Keep the masterplan warning in mind: on Hangprinter variants the top "carrying" anchor must never be put in torque/sensor mode during sweeps.

Remember the data-collection limitation: we only have encoder deltas, not anchor positions, during logging. Every length value we store (`fixed_lengths`, `l_drive`, `l_sensor`) is therefore relative to the length when the mover was at the origin after encoder zeroing. Any downstream code that needs absolute lengths must reconstruct them using the current anchor guess: `L_abs = ||A_i - origin|| + ΔL_measured`. Keep this contract in mind when consuming these structures in later substeps. Ellipse fitting is now performed inside the optimization loop per anchor guess; no pre-fit ellipses are stored.

## Implementation Details

### 1.1 Core Data Structures

#### Python Classes (`sweep_types.py`)

```python
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from enum import Enum
import numpy as np

class MachineType(Enum):
    SLIDEPRINTER = "slideprinter"      # 3 anchors, 2D
    HANGPRINTER_4 = "hangprinter_4"    # 4 anchors, 3D
    HANGPRINTER_5 = "hangprinter_5"    # 5 anchors, 3D
    CUBECORNERS = "cubecorners"        # 8 anchors, 3D
    SKYCAM = "skycam"                  # 4 anchors, 3D (underactuated)

@dataclass
class MachineConfig:
    """Machine geometry configuration."""
    machine_type: MachineType
    num_anchors: int
    dimensions: int  # 2 for Slideprinter, 3 for others
    carrying_anchors: List[int] = field(default_factory=list)  # Anchors that must never be Sensor/torque mode

    @property
    def constraints_for_1dof(self) -> int:
        """Number of fixed cables needed for 1-DOF motion."""
        return self.dimensions - 1

    @classmethod
    def from_type(cls, machine_type: MachineType) -> 'MachineConfig':
        configs = {
            MachineType.SLIDEPRINTER: (3, 2, []),
            MachineType.HANGPRINTER_4: (4, 3, [3]),  # Anchor 3 carries weight, never Sensor
            MachineType.HANGPRINTER_5: (5, 3, [4]),  # Anchor 4 carries weight, never Sensor
            MachineType.CUBECORNERS: (8, 3, []),
            MachineType.SKYCAM: (4, 3, []),
        }
        num_anchors, dimensions, carrying = configs[machine_type]
        return cls(machine_type, num_anchors, dimensions, carrying)

@dataclass
class DataPoint:
    """Single measurement point in a sweep."""
    l_drive: float          # Drive cable length (mm). Not absolute length. Relative to length when at origin.
    l_sensor: float         # Sensor cable length (mm). Not absolute length. Relative to length when at origin.
    timestamp_ms: Optional[float] = None  # Relative timestamp

@dataclass
class SweepMetadata:
    """Metadata about how the sweep was collected."""
    feed_rate: float = 2000.0       # mm/min
    torque: float = 0.05            # Nm (for sensor motor)
    settle_ms: float = 100.0        # Settle time between points
    sample_rate_hz: Optional[float] = None  # For continuous sweeps

@dataclass
class Sweep:
    """A single sweep measurement sequence."""
    id: str
    fixed_anchors: List[int]        # Indices of anchors held fixed
    fixed_lengths: List[float]      # Lengths of fixed cables (mm). Not absolute length. Relative to length when at origin.
    drive_anchor: int               # Index of actively driven anchor
    sensor_anchor: int              # Index of passively measured anchor
    data_points: List[DataPoint]
    metadata: SweepMetadata = field(default_factory=SweepMetadata)

    def validate(self, config: MachineConfig) -> List[str]:
        """Validate sweep configuration against machine config."""
        errors = []

        # Check anchor indices
        all_anchors = set(self.fixed_anchors + [self.drive_anchor, self.sensor_anchor])
        if max(all_anchors) >= config.num_anchors:
            errors.append(f"Anchor index out of range for {config.machine_type}")

        # Check constraints
        if len(self.fixed_anchors) != config.constraints_for_1dof:
            errors.append(
                f"Expected {config.constraints_for_1dof} fixed anchors, "
                f"got {len(self.fixed_anchors)}"
            )

        # Prevent putting carrying anchors in torque/sensor mode
        if self.sensor_anchor in (config.carrying_anchors or []):
            errors.append("Carrying anchors cannot be assigned the Sensor role")

        # Check no duplicates
        if len(all_anchors) != len(self.fixed_anchors) + 2:
            errors.append("Duplicate anchor indices in sweep configuration")

        # Check data points
        if len(self.data_points) < 5:
            errors.append("Insufficient data points for ellipse fitting (min 5)")

        return errors

    def to_squared_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        """Convert data points to squared length arrays for ellipse fitting."""
        l_drive = np.array([p.l_drive for p in self.data_points])
        l_sensor = np.array([p.l_sensor for p in self.data_points])
        return l_drive**2, l_sensor**2

@dataclass
class SweepConfigSnapshot:
    """
    Minimal sweep configuration needed to reconstruct the ellipse roles.

    Stored alongside fitted ellipses so Phase 2 can match predictions to
    observations even if the sweep list is filtered or re-sampled.
    """
    fixed_anchors: List[int]
    fixed_lengths: List[float]
    drive_anchor: int
    sensor_anchor: int

    @classmethod
    def from_sweep(cls, sweep: Sweep) -> 'SweepConfigSnapshot':
        return cls(
            fixed_anchors=list(sweep.fixed_anchors),
            fixed_lengths=list(sweep.fixed_lengths),
            drive_anchor=sweep.drive_anchor,
            sensor_anchor=sweep.sensor_anchor,
        )

@dataclass
class EllipseCoefficients:
    """Algebraic ellipse coefficients: Ax² + Bxy + Cy² + Dx + Ey + F = 0"""
    A: float
    B: float
    C: float
    D: float
    E: float
    F: float

    def to_array(self) -> np.ndarray:
        """Return normalized coefficient array."""
        arr = np.array([self.A, self.B, self.C, self.D, self.E, self.F])
        # Normalize so ||[A,B,C]|| = 1
        abc_norm = np.linalg.norm(arr[:3])
        if abc_norm > 1e-10:
            arr = arr / abc_norm
        return arr

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'EllipseCoefficients':
        return cls(A=arr[0], B=arr[1], C=arr[2], D=arr[3], E=arr[4], F=arr[5])

@dataclass
class FittedEllipse:
    """Result of ellipse fitting for a sweep."""
    sweep_id: str
    sweep_config: SweepConfigSnapshot  # Snapshot of fixed/drive/sense roles
    coefficients: EllipseCoefficients
    residual_rms: float         # Root mean square algebraic distance
    valid: bool                 # True if residual below threshold
    num_points: int             # Number of points used in fit

@dataclass
class SweepDataset:
    """Complete dataset for calibration."""
    version: str
    machine_config: MachineConfig
    timestamp: str
    sweeps: List[Sweep]
    fitted_ellipses: List[FittedEllipse] = field(default_factory=list)

    def get_valid_ellipses(self) -> List[FittedEllipse]:
        """Return only ellipses that passed QC."""
        return [e for e in self.fitted_ellipses if e.valid]
```

**Phase transition guardrail:** Each `FittedEllipse` now carries a `SweepConfigSnapshot` so forward-model code in Phase 2 can reconstruct exactly which cables were fixed/drive/sense when that ellipse was produced.

### 1.2 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "SweepDataset",
  "type": "object",
  "required": ["version", "machine_type", "num_anchors", "dimensions", "sweeps"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$"
    },
    "machine_type": {
      "type": "string",
      "enum": ["slideprinter", "hangprinter_4", "hangprinter_5", "cubecorners", "skycam"]
    },
    "num_anchors": {
      "type": "integer",
      "minimum": 3,
      "maximum": 8
    },
    "dimensions": {
      "type": "integer",
      "enum": [2, 3]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sweeps": {
      "type": "array",
      "items": {"$ref": "#/definitions/Sweep"},
      "minItems": 1
    },
    "fitted_ellipses": {
      "type": "array",
      "items": {"$ref": "#/definitions/FittedEllipse"}
    }
  },
  "definitions": {
    "Sweep": {
      "type": "object",
      "required": ["id", "fixed_anchors", "fixed_lengths", "drive_anchor", "sensor_anchor", "data_points"],
      "properties": {
        "id": {"type": "string"},
        "fixed_anchors": {
          "type": "array",
          "items": {"type": "integer", "minimum": 0}
        },
        "fixed_lengths": {
          "type": "array",
          "items": {"type": "number"}
        },
        "drive_anchor": {"type": "integer", "minimum": 0},
        "sensor_anchor": {"type": "integer", "minimum": 0},
        "drive_range": {
          "type": "object",
          "properties": {
            "start": {"type": "number"},
            "end": {"type": "number"},
            "unit": {"type": "string", "default": "mm"}
          }
        },
        "data_points": {
          "type": "array",
          "items": {"$ref": "#/definitions/DataPoint"},
          "minItems": 5
        },
        "metadata": {"$ref": "#/definitions/SweepMetadata"}
      }
    },
    "DataPoint": {
      "type": "object",
      "required": ["l_drive", "l_sensor"],
      "properties": {
        "l_drive": {"type": "number"},
        "l_sensor": {"type": "number"},
        "timestamp_ms": {"type": "number"}
      }
    },
    "SweepMetadata": {
      "type": "object",
      "properties": {
        "feed_rate": {"type": "number", "default": 2000},
        "torque": {"type": "number", "default": 0.05},
        "settle_ms": {"type": "number", "default": 100},
        "sample_rate_hz": {"type": "number"}
      }
    },
    "SweepConfigSnapshot": {
      "type": "object",
      "required": ["fixed_anchors", "fixed_lengths", "drive_anchor", "sensor_anchor"],
      "properties": {
        "fixed_anchors": {
          "type": "array",
          "items": {"type": "integer", "minimum": 0}
        },
        "fixed_lengths": {
          "type": "array",
          "items": {"type": "number"}
        },
        "drive_anchor": {"type": "integer", "minimum": 0},
        "sensor_anchor": {"type": "integer", "minimum": 0}
      }
    },
    "FittedEllipse": {
      "type": "object",
      "required": ["sweep_id", "sweep_config", "coefficients", "residual_rms", "valid"],
      "properties": {
        "sweep_id": {"type": "string"},
        "sweep_config": {"$ref": "#/definitions/SweepConfigSnapshot"},
        "coefficients": {
          "type": "object",
          "properties": {
            "A": {"type": "number"},
            "B": {"type": "number"},
            "C": {"type": "number"},
            "D": {"type": "number"},
            "E": {"type": "number"},
            "F": {"type": "number"}
          }
        },
        "residual_rms": {"type": "number"},
        "valid": {"type": "boolean"},
        "num_points": {"type": "integer"}
      }
    }
  }
}
```

### 1.3 Serialization Utilities

```python
# sweep_io.py
import json
from datetime import datetime
from pathlib import Path
from typing import Union

def save_sweep_dataset(dataset: SweepDataset, path: Union[str, Path]) -> None:
    """Save dataset to JSON file."""
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
                    {"l_drive": p.l_drive, "l_sensor": p.l_sensor, "timestamp_ms": p.timestamp_ms}
                    for p in s.data_points
                ],
                "metadata": {
                    "feed_rate": s.metadata.feed_rate,
                    "torque": s.metadata.torque,
                    "settle_ms": s.metadata.settle_ms,
                    "sample_rate_hz": s.metadata.sample_rate_hz,
                }
            }
            for s in dataset.sweeps
        ],
        "fitted_ellipses": [
            {
                "sweep_id": e.sweep_id,
                "sweep_config": {
                    "fixed_anchors": e.sweep_config.fixed_anchors,
                    "fixed_lengths": e.sweep_config.fixed_lengths,
                    "drive_anchor": e.sweep_config.drive_anchor,
                    "sensor_anchor": e.sweep_config.sensor_anchor,
                },
                "coefficients": {
                    "A": e.coefficients.A,
                    "B": e.coefficients.B,
                    "C": e.coefficients.C,
                    "D": e.coefficients.D,
                    "E": e.coefficients.E,
                    "F": e.coefficients.F,
                },
                "residual_rms": e.residual_rms,
                "valid": e.valid,
                "num_points": e.num_points,
            }
            for e in dataset.fitted_ellipses
        ]
    }

    with open(path, 'w') as f:
        json.dump(data, f, indent=2)

def load_sweep_dataset(path: Union[str, Path]) -> SweepDataset:
    """Load dataset from JSON file."""
    with open(path, 'r') as f:
        data = json.load(f)

    machine_type = MachineType(data["machine_type"])
    config = MachineConfig.from_type(machine_type)

    sweeps = []
    for s in data["sweeps"]:
        data_points = [
            DataPoint(p["l_drive"], p["l_sensor"], p.get("timestamp_ms"))
            for p in s["data_points"]
        ]
        metadata = SweepMetadata(**s.get("metadata", {}))
        sweeps.append(Sweep(
            id=s["id"],
            fixed_anchors=s["fixed_anchors"],
            fixed_lengths=s["fixed_lengths"],
            drive_anchor=s["drive_anchor"],
            sensor_anchor=s["sensor_anchor"],
            data_points=data_points,
            metadata=metadata,
        ))

    fitted_ellipses = []
    for e in data.get("fitted_ellipses", []):
        coeffs = EllipseCoefficients(**e["coefficients"])
        sweep_cfg_data = e["sweep_config"]
        fitted_ellipses.append(FittedEllipse(
            sweep_id=e["sweep_id"],
            sweep_config=SweepConfigSnapshot(**sweep_cfg_data),
            coefficients=coeffs,
            residual_rms=e["residual_rms"],
            valid=e["valid"],
            num_points=e.get("num_points", 0),
        ))

    return SweepDataset(
        version=data["version"],
        machine_config=config,
        timestamp=data.get("timestamp", datetime.now().isoformat()),
        sweeps=sweeps,
        fitted_ellipses=fitted_ellipses,
    )
```

### 1.4 Sweep Configuration Generator

```python
# sweep_config_generator.py
from itertools import combinations, permutations
from typing import List, Tuple

def generate_sweep_configs(
    config: MachineConfig,
    forbidden_sensor_anchors: List[int] | None = None
) -> List[dict]:
    """
    Generate all valid sweep configurations for a machine type.

    Returns list of dicts with keys:
      - fixed_anchors: List[int]
      - drive_anchor: int
      - sensor_anchor: int
    """
    n = config.num_anchors
    k = config.constraints_for_1dof  # Number of anchors to fix
    sensor_block = set(forbidden_sensor_anchors or config.carrying_anchors or [])

    all_configs = []

    # For each combination of fixed anchors
    for fixed in combinations(range(n), k):
        fixed_set = set(fixed)
        free_anchors = [i for i in range(n) if i not in fixed_set]

        # For each permutation of (drive, sensor) from free anchors
        for drive, sensor in permutations(free_anchors, 2):
            if sensor in sensor_block:
                continue  # Carrying anchors must never be Sensors
            all_configs.append({
                "fixed_anchors": list(fixed),
                "drive_anchor": drive,
                "sensor_anchor": sensor,
            })

    return all_configs

def select_representative_configs(
    all_configs: List[dict],
    config: MachineConfig,
    max_sweeps: int = 12
) -> List[dict]:
    """
    Select a representative subset of sweep configurations.

    Strategy:
    - Ensure each anchor appears as drive at least once
    - Ensure each non-carrying anchor appears as sensor at least once
    - Distribute fixed anchor combinations evenly
    """
    if len(all_configs) <= max_sweeps:
        return all_configs

    selected = []
    used_as_drive = set()
    used_as_sensor = set()
    sensor_block = set(config.carrying_anchors or [])

    # First pass: ensure coverage
    for cfg in all_configs:
        drive = cfg["drive_anchor"]
        sensor = cfg["sensor_anchor"]

        if sensor in sensor_block:
            continue

        if drive not in used_as_drive or sensor not in used_as_sensor:
            selected.append(cfg)
            used_as_drive.add(drive)
            used_as_sensor.add(sensor)

            if len(selected) >= max_sweeps:
                break

    # Second pass: fill remaining slots with diverse configurations
    if len(selected) < max_sweeps:
        remaining = [c for c in all_configs if c not in selected]
        # Sort by uniqueness of fixed anchor combination
        selected.extend(remaining[:max_sweeps - len(selected)])

    return selected
```

## Testing

### Unit Tests

```python
# test_sweep_types.py
import pytest
from sweep_types import *

def test_machine_config_slideprinter():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    assert config.num_anchors == 3
    assert config.dimensions == 2
    assert config.constraints_for_1dof == 1

def test_machine_config_hangprinter_4():
    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    assert config.num_anchors == 4
    assert config.dimensions == 3
    assert config.constraints_for_1dof == 2

def test_sweep_validation_valid():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    sweep = Sweep(
        id="test",
        fixed_anchors=[0],
        fixed_lengths=[10.0],
        drive_anchor=1,
        sensor_anchor=2,
        data_points=[DataPoint(i, i+100) for i in range(10)],
    )
    errors = sweep.validate(config)
    assert len(errors) == 0

def test_sweep_validation_wrong_constraints():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    sweep = Sweep(
        id="test",
        fixed_anchors=[0, 1],  # Wrong: Slideprinter needs only 1 fixed
        fixed_lengths=[10.0, 11.0],
        drive_anchor=2,
        sensor_anchor=0,  # Duplicate!
        data_points=[DataPoint(i, i+100) for i in range(10)],
    )
    errors = sweep.validate(config)
    assert len(errors) > 0

def test_sweep_validation_blocks_carrying_sensor():
    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    sweep = Sweep(
        id="bad_sensor",
        fixed_anchors=[0, 1],
        fixed_lengths=[10.0, 11.0],
        drive_anchor=2,
        sensor_anchor=3,  # Carrying anchor must never be Sensor
        data_points=[DataPoint(i, i+100) for i in range(10)],
    )
    errors = sweep.validate(config)
    assert any("Carrying" in err for err in errors)

def test_ellipse_coefficients_normalization():
    coeffs = EllipseCoefficients(A=3.0, B=0.0, C=4.0, D=10, E=20, F=100)
    arr = coeffs.to_array()
    assert abs(np.linalg.norm(arr[:3]) - 1.0) < 1e-10

def test_sweep_config_generator_slideprinter():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    configs = generate_sweep_configs(config)
    # 3 anchors, fix 1, permute remaining 2 -> 3 * 2 * 1 = 6
    assert len(configs) == 6

def test_sweep_config_generator_hangprinter_4():
    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    configs = generate_sweep_configs(config)
    # C(4,2) = 6 ways to fix 2, but anchor 3 is carrying and never Sensor -> 9 configs
    assert len(configs) == 9
```

### Integration Test

```python
def test_roundtrip_serialization(tmp_path):
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    dataset = SweepDataset(
        version="1.0",
        machine_config=config,
        timestamp="2024-01-15T10:00:00Z",
        sweeps=[
            Sweep(
                id="sweep_001",
                fixed_anchors=[0],
                fixed_lengths=[10.0],
                drive_anchor=1,
                sensor_anchor=2,
                data_points=[DataPoint(i*10, i*12+50) for i in range(20)],
            )
        ],
        fitted_ellipses=[
            FittedEllipse(
                sweep_id="sweep_001",
                sweep_config=SweepConfigSnapshot(
                    fixed_anchors=[0],
                    fixed_lengths=[10.0],
                    drive_anchor=1,
                    sensor_anchor=2,
                ),
                coefficients=EllipseCoefficients(1.0, 0.1, 0.9, -50, -60, 500),
                residual_rms=0.001,
                valid=True,
                num_points=20,
            )
        ]
    )

    path = tmp_path / "test_dataset.json"
    save_sweep_dataset(dataset, path)
    loaded = load_sweep_dataset(path)

    assert loaded.version == dataset.version
    assert loaded.machine_config.machine_type == config.machine_type
    assert len(loaded.sweeps) == 1
    assert len(loaded.fitted_ellipses) == 1
    assert loaded.sweeps[0].id == "sweep_001"
```

## Validation Criteria

1. **Schema Compliance**: All generated JSON files validate against the schema
2. **Roundtrip Fidelity**: `load(save(dataset)) == dataset` for all fields
3. **Configuration Coverage**: `generate_sweep_configs()` produces correct count for each machine type while respecting carrying/sensor exclusions
4. **Constraint Validation**: `Sweep.validate()` catches all invalid configurations
5. **Normalization Consistency**: Ellipse coefficient normalization is idempotent

## Dependencies

- Python 3.8+
- numpy
- dataclasses (stdlib)
- json (stdlib)
- pytest (for testing)

## Estimated Complexity

**Effort**: Low-Medium (2-3 hours)

This substep is mostly data structure definition with straightforward serialization logic. The complexity lies in ensuring the generalization works correctly across all machine types.

## Files to Create/Modify

| File | Action |
|------|--------|
| `autocal/sweep_types.py` | Create |
| `autocal/sweep_io.py` | Create |
| `autocal/sweep_config_generator.py` | Create |
| `autocal/tests/test_sweep_types.py` | Create |
