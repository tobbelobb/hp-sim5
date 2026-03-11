from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

__all__ = [
    "MachineType",
    "MachineConfig",
    "DataPoint",
    "SweepMetadata",
    "Sweep",
    "SweepConfigSnapshot",
    "EllipseCoefficients",
    "FittedEllipse",
    "SweepDataset",
]


class MachineType(Enum):
    SLIDEPRINTER = "slideprinter"  # 3 anchors, 2D
    HANGPRINTER_4 = "hangprinter_4"  # 4 anchors, 3D
    HANGPRINTER_5 = "hangprinter_5"  # 5 anchors, 3D
    CUBECORNERS = "cubecorners"  # 8 anchors, 3D
    SKYCAM = "skycam"  # 4 anchors, 3D (underactuated)


@dataclass
class MachineConfig:
    """Machine geometry configuration."""

    machine_type: MachineType
    num_anchors: int
    dimensions: int  # 2 for Slideprinter, 3 for others
    carrying_anchors: List[int] = field(default_factory=list)
    must_be_fixed_anchors: List[int] = field(default_factory=list)
    fixed_anchor_delta_bounds: Dict[int, Tuple[Optional[float], Optional[float]]] = field(default_factory=dict)

    @property
    def constraints_for_1dof(self) -> int:
        """Number of fixed cables needed for 1-DOF motion."""
        return self.dimensions - 1

    @classmethod
    def from_type(cls, machine_type: MachineType) -> "MachineConfig":
        configs = {
            MachineType.SLIDEPRINTER: (3, 2, [], [], {}),
            # Anchor 3 is the load-bearing line on hangprinter_4:
            # keep it in the fixed set and never command a positive fixed delta.
            MachineType.HANGPRINTER_4: (4, 3, [3], [3], {3: (None, 0.0)}),
            MachineType.HANGPRINTER_5: (5, 3, [4], [], {}),
            MachineType.CUBECORNERS: (8, 3, [], [], {}),
            MachineType.SKYCAM: (4, 3, [], [], {}),
        }
        if machine_type not in configs:
            raise ValueError(f"Unsupported machine type: {machine_type}")
        num_anchors, dimensions, carrying, must_be_fixed, fixed_bounds = configs[machine_type]
        return cls(
            machine_type,
            num_anchors,
            dimensions,
            carrying,
            list(must_be_fixed),
            dict(fixed_bounds),
        )


@dataclass
class DataPoint:
    """Single measurement point in a sweep."""

    l_drive: float  # Drive cable length delta (mm)
    l_sensor: float  # Sensor cable length delta (mm)
    l_drive_mu: Optional[float] = None  # Mean drive length delta (mm)
    l_sensor_mu: Optional[float] = None  # Mean sensor length delta (mm)
    timestamp_ms: Optional[float] = None  # Relative timestamp
    raw_angles_deg: Optional[List[float]] = None  # Raw encoder angles
    mu: Optional[List[float]] = None  # Mean encoder angles (deg)
    sigma: Optional[List[float]] = None  # Robust encoder noise (deg, MAD-based)
    sample_count: Optional[int] = None  # Encoder samples used for mu/sigma
    sampling_hz: Optional[float] = None  # Effective sampling rate
    sample_duration_ms: Optional[float] = None  # Sampling duration
    noise_warnings: Optional[List[str]] = None  # Data quality flags


@dataclass
class SweepMetadata:
    """Metadata about how the sweep was collected."""

    feed_rate: float = 2000.0  # mm/min
    torque: float = 0.05  # Nm (for sensor motor)
    settle_ms: float = 100.0  # Settle time between points
    sample_rate_hz: Optional[float] = None  # For continuous sweeps


@dataclass
class Sweep:
    """A single sweep measurement sequence."""

    id: str
    fixed_anchors: List[int]
    fixed_lengths: List[float]
    drive_anchor: int
    sensor_anchor: int
    data_points: List[DataPoint]
    metadata: SweepMetadata = field(default_factory=SweepMetadata)

    def validate(self, config: MachineConfig) -> List[str]:
        """Validate sweep configuration against machine config."""
        errors: List[str] = []

        all_indices = self.fixed_anchors + [self.drive_anchor, self.sensor_anchor]
        if any(idx < 0 for idx in all_indices):
            errors.append("Anchor indices must be non-negative")
        if all_indices and max(all_indices) >= config.num_anchors:
            errors.append(f"Anchor index out of range for {config.machine_type.value}")

        if len(self.fixed_anchors) != config.constraints_for_1dof:
            errors.append(
                f"Expected {config.constraints_for_1dof} fixed anchors, got {len(self.fixed_anchors)}"
            )
        if len(set(self.fixed_anchors)) != len(self.fixed_anchors):
            errors.append("Duplicate anchor indices in fixed_anchors")
        if len(self.fixed_lengths) != len(self.fixed_anchors):
            errors.append("fixed_lengths must match length of fixed_anchors")

        if self.sensor_anchor in (config.carrying_anchors or []):
            errors.append("Carrying anchors cannot be assigned the Sensor role")

        if len(set(all_indices)) != len(all_indices):
            errors.append("Duplicate anchor indices in sweep configuration")

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
    """Minimal sweep configuration needed to reconstruct the ellipse roles."""

    fixed_anchors: List[int]
    fixed_lengths: List[float]
    drive_anchor: int
    sensor_anchor: int

    @classmethod
    def from_sweep(cls, sweep: Sweep) -> "SweepConfigSnapshot":
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
        arr = np.array([self.A, self.B, self.C, self.D, self.E, self.F], dtype=float)
        abc_norm = np.linalg.norm(arr[:3])
        if abc_norm > 1e-10:
            arr = arr / abc_norm
        return arr

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "EllipseCoefficients":
        return cls(A=float(arr[0]), B=float(arr[1]), C=float(arr[2]), D=float(arr[3]), E=float(arr[4]), F=float(arr[5]))


@dataclass
class FittedEllipse:
    """Result of ellipse fitting for a sweep (in-memory/debug only, not persisted)."""

    sweep_id: str
    sweep_config: SweepConfigSnapshot
    coefficients: EllipseCoefficients
    center: Tuple[float, float]
    semi_axes: Tuple[float, float]  # (a, b) with a >= b
    orientation_rad: float  # θ in radians
    residual_rms: float
    valid: bool
    num_points: int
    residual_series: Optional[List[float]] = None


@dataclass
class SweepDataset:
    """Complete dataset for calibration."""

    version: str
    machine_config: MachineConfig
    timestamp: str
    sweeps: List[Sweep]
