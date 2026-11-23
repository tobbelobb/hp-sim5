"""Core protocols and serializable contracts for hp-sim5 experiments.

Separates five layers:
1. Experiment control (ExperimentSpec, ExperimentController)
2. Firmware (FirmwareService)
3. Physics (PhysicsBackend, and BatchedPhysicsBackend for future GPU batching)
4. Execution scheduling (ExecutionScheduler)
5. Artifact storage (ArtifactStore)
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable
import uuid


@dataclass(frozen=True)
class FirmwareReply:
    """Reply returned by a firmware service for a single G-code line."""

    gcode: str
    reply: str
    motion_lines: tuple[str, ...] = ()


@runtime_checkable
class FirmwareService(Protocol):
    """Stateful firmware planner (e.g. RepRapFirmware host simulator)."""

    @property
    def url(self) -> str: ...

    @property
    def config_sha256(self) -> str: ...

    @property
    def binary_sha256(self) -> str: ...

    def start(self) -> "FirmwareService": ...

    def send_gcode(self, line: str, *, timeout_s: float = 30.0) -> FirmwareReply: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class PhysicsStatus:
    """Snapshot of a single physics world's clock and engine metadata."""

    backend: str
    physics_engine: str
    step: int
    total_steps: int
    epoch: int
    dt_s: float
    simulated_s: float
    queue_length: int
    wall_s: float = 0.0
    error: str | None = None


@runtime_checkable
class PhysicsBackend(Protocol):
    """Single-world deterministic fixed-step physics engine."""

    def advance(self, seconds: float, *, drain: bool = False) -> PhysicsStatus: ...

    def read_encoders(self, axes: Sequence[str]) -> list[float]: ...

    def reset(self) -> PhysicsStatus: ...

    def status(self) -> PhysicsStatus: ...

    def close(self) -> None: ...


# ---------------------------------------------------------------------------
# Future GPU-batching contracts (CPU adapters today; GPU kernels later).
#
# Adaptive Hangprinter experiments (like active-learning autocal) diverge in
# step count, settling duration, slip recovery, and sweep plans per world.
# Keeping stepping decoupled from per-world adaptive controllers lets a future
# GPU BatchedPhysicsBackend pack all currently advancing worlds into one kernel
# launch while worlds awaiting firmware planning or optimizer fits yield cleanly.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class WorldStepRequest:
    """One world's next action inside a batched adaptive execution loop."""

    world_id: int
    steps: int = 0
    commands: tuple[Mapping[str, Any], ...] = ()
    drain_queue: bool = False
    active: bool = True


@dataclass(frozen=True)
class ObservationBatch:
    """Contiguous or indexed observations emitted by a BatchedPhysicsBackend.

    Array fields are shaped `[B, ...]` when backed by numpy/device arrays,
    keyed by `world_ids` so adaptive controllers can step subsets independently.
    """

    world_ids: tuple[int, ...]
    steps: tuple[int, ...]
    sim_times_s: tuple[float, ...]
    queue_lengths: tuple[int, ...]
    active_mask: tuple[bool, ...]
    encoder_angles_rad: Any = None  # shape [B, num_axes]
    effector_positions_m: Any = None  # shape [B, num_effectors, 3]
    metadata: Mapping[str, Any] = field(default_factory=dict)


@runtime_checkable
class BatchedPhysicsBackend(Protocol):
    """Pool of independent physics worlds stepped together (CPU pool or GPU batch)."""

    @property
    def world_count(self) -> int: ...

    def step_batch(self, requests: Sequence[WorldStepRequest]) -> ObservationBatch: ...

    def reset_worlds(self, world_ids: Sequence[int]) -> ObservationBatch: ...

    def close(self) -> None: ...


@runtime_checkable
class ExperimentController(Protocol):
    """Adaptive per-world controller decoupled from physics transport."""

    def initial_request(self, world_id: int) -> WorldStepRequest: ...

    def on_observation(self, world_id: int, observation: ObservationBatch) -> WorldStepRequest | None: ...


# ---------------------------------------------------------------------------
# Portable experiment specification, result, storage and scheduling contracts.
# ---------------------------------------------------------------------------


@dataclass
class ExperimentSpec:
    """Portable, JSON-serializable specification of one headless experiment."""

    kind: str  # 'autocal', 'sweep_collection', 'command'
    machine: str = "hangprinter_3"
    experiment_id: str = field(default_factory=lambda: uuid.uuid4().hex[:16])
    args: list[str] = field(default_factory=list)
    collector_args: list[str] = field(default_factory=list)
    command: list[str] = field(default_factory=list)
    recording_mode: str = "none"
    solver: str = "native"  # 'native' or 'wasm'
    timeout_s: float = 3600.0
    max_physics_steps: int | None = None
    threads: int = 1
    env: dict[str, str] = field(default_factory=dict)
    tags: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ExperimentSpec":
        allowed = set(cls.__dataclass_fields__)
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"Unknown ExperimentSpec fields: {sorted(unknown)}")
        return cls(**dict(data))


@dataclass
class ExperimentResult:
    """Portable result returned by local or remote ExecutionSchedulers."""

    experiment_id: str
    kind: str
    status: str  # 'ok', 'failed', 'timeout', 'cancelled'
    exit_code: int
    backend: str
    wall_s: float
    simulated_s: float | None = None
    total_steps: int | None = None
    metrics: dict[str, Any] = field(default_factory=dict)
    scratch_dir: str | None = None
    artifact_uri: str | None = None
    manifest: dict[str, Any] = field(default_factory=dict)
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ExperimentResult":
        allowed = set(cls.__dataclass_fields__)
        filtered = {k: v for k, v in data.items() if k in allowed}
        return cls(**filtered)


@dataclass(frozen=True)
class ArtifactRecord:
    """Verified metadata for one stored artifact file."""

    experiment_id: str
    rel_path: str
    size_bytes: int
    sha256: str
    uri: str
    remote_id: str | None = None
    content_type: str = "application/octet-stream"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@runtime_checkable
class ArtifactStore(Protocol):
    """Persistent storage for large experiment artifacts (local or Google Drive)."""

    @property
    def uri(self) -> str: ...

    def put_file(
        self,
        experiment_id: str,
        rel_path: str,
        local_path: Path,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> ArtifactRecord: ...

    def put_directory(
        self,
        experiment_id: str,
        local_dir: Path,
        *,
        metadata: Mapping[str, Any] | None = None,
        exclude_globs: Sequence[str] = (),
    ) -> dict[str, Any]: ...

    def get_file(self, experiment_id: str, rel_path: str, dest_path: Path) -> Path: ...

    def get_experiment(
        self,
        experiment_id: str,
        dest_dir: Path,
        *,
        rel_paths: Sequence[str] | None = None,
    ) -> Path: ...

    def get_manifest(self, experiment_id: str) -> dict[str, Any]: ...

    def list_experiments(
        self,
        *,
        tags: Sequence[str] = (),
        kind: str | None = None,
        limit: int = 100,
    ) -> list[dict[str, Any]]: ...

    def exists(self, experiment_id: str, rel_path: str | None = None) -> bool: ...


@runtime_checkable
class ExecutionScheduler(Protocol):
    """Schedules single or batched experiments locally or on remote compute."""

    @property
    def backend_name(self) -> str: ...

    def run(
        self,
        spec: ExperimentSpec,
        *,
        store: ArtifactStore | None = None,
        keep_scratch: bool = False,
    ) -> ExperimentResult: ...

    def run_batch(
        self,
        specs: Sequence[ExperimentSpec],
        *,
        max_workers: int | None = None,
        store: ArtifactStore | None = None,
        keep_scratch: bool = False,
    ) -> list[ExperimentResult]: ...
