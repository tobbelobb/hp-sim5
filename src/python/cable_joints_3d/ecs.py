"""3D ECS components sharing the Python :class:`World` implementation."""
from dataclasses import dataclass, field
import numpy as np
from cable_joints.ecs import (World, RadiusComponent, MassComponent,
    RestitutionComponent, GravityAffectedComponent, CoefficientOfFrictionComponent,
    DistanceConstraintComponent, MachineTagComponent)
from .inertia_tensor import MomentOfInertiaComponent
from .quaternion import Quaternion

@dataclass(init=False)
class PositionComponent:
    pos: np.ndarray
    def __init__(self, x=0., y=0., z=0.): self.pos = np.array([x, y, z], dtype=float)
@dataclass(init=False)
class PrevFinalPosComponent(PositionComponent): pass
@dataclass(init=False)
class VelocityComponent:
    vel: np.ndarray
    def __init__(self, x=0., y=0., z=0.): self.vel = np.array([x, y, z], dtype=float)
@dataclass(init=False)
class OrientationComponent:
    quaternion: Quaternion
    def __init__(self, x=0., y=0., z=0., w=1.): self.quaternion = Quaternion(x, y, z, w)
@dataclass(init=False)
class PrevFinalOrientationComponent(OrientationComponent): pass
@dataclass(init=False)
class AngularVelocityComponent:
    omega: np.ndarray
    def __init__(self, x=0., y=0., z=0.): self.omega = np.array([x, y, z], dtype=float)
@dataclass
class EncoderComponent:
    angle: float = 0.
    axis: np.ndarray = field(default_factory=lambda: np.array([0., 0., 1.]))
@dataclass
class RenderableComponent:
    shape: str = 'circle'
    color: str = '#888888'
    height: float | None = None

    def __post_init__(self):
        if self.height is not None and (not np.isfinite(self.height) or self.height <= 0):
            self.height = None
@dataclass
class BallTagComponent: pass
@dataclass
class ObstacleTagComponent: pass
@dataclass
class ObstaclePushComponent:
    push_vel: float = 2.
@dataclass
class SceneEntityInfoComponent:
    name: str
    tags: list[str] = field(default_factory=list)
@dataclass
class HybridKnotAngleComponent:
    angle: float = 0.
    path_angles: dict = field(default_factory=dict)
@dataclass
class RigidBodyComponent:
    members: list[int] = field(default_factory=list)
    render_segments: list | None = None
    synced_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    synced_orientation: Quaternion = field(default_factory=Quaternion)

    def __post_init__(self):
        self.members = list(self.members)
@dataclass(init=False)
class RigidBodyMemberComponent:
    body_entity: int | None = None
    local_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    local_orientation: Quaternion = field(default_factory=Quaternion)
    physical_mass: float | None = None

    def __init__(self, body_entity=None, local_position=None,
                 local_orientation=None, physical_mass=None):
        self.body_entity = body_entity
        self.local_position = (
            np.asarray(local_position, dtype=float).copy()
            if local_position is not None else np.zeros(3)
        )
        self.local_orientation = (
            local_orientation.copy().normalize()
            if local_orientation is not None else Quaternion()
        )
        self.physical_mass = (
            float(physical_mass) if physical_mass is not None
            and np.isfinite(physical_mass) else None
        )

def layering_enabled(world):
    value = world.get_resource("enableLayering")
    return value if isinstance(value, bool) else True
