import numpy as np
from dataclasses import dataclass, field
import math

class World:
    def __init__(self):
        self.entities = {}  # entity_id -> set of component classes
        self.components = {}  # component_class -> {entity_id: component_instance}
        self.next_entity_id = 0
        self.systems = []
        self.resources = {}

    def create_entity(self):
        entity_id = self.next_entity_id
        self.next_entity_id += 1
        self.entities[entity_id] = set()
        return entity_id

    def add_component(self, entity_id, component):
        component_class = type(component)
        if component_class not in self.components:
            self.components[component_class] = {}
        self.components[component_class][entity_id] = component
        if entity_id in self.entities:
            self.entities[entity_id].add(component_class)
        else:
            # This case might happen if an entity is created outside the world system
            # but we still want to track its components. For tests, this is fine.
            self.entities[entity_id] = {component_class}

    def get_component(self, entity_id, component_class):
        return self.components.get(component_class, {}).get(entity_id)

    def has_component(self, entity_id, component_class):
        return entity_id in self.components.get(component_class, {})

    def remove_component(self, entity_id, component_class):
        if component_class in self.components:
            if entity_id in self.components[component_class]:
                del self.components[component_class][entity_id]
        if entity_id in self.entities:
            self.entities[entity_id].discard(component_class)

    def destroy_entity(self, entity_id):
        if entity_id in self.entities:
            for component_class in list(self.entities[entity_id]):
                self.remove_component(entity_id, component_class)
            if entity_id in self.entities:
                del self.entities[entity_id]

    def query(self, component_classes):
        if not component_classes:
            return []

        if not isinstance(component_classes, (list, tuple)):
            component_classes = [component_classes]

        first_class = component_classes[0]
        if first_class not in self.components:
            return []

        candidate_ids = set(self.components[first_class].keys())
        if not candidate_ids:
            return []

        for component_class in component_classes[1:]:
            if component_class not in self.components:
                return []
            candidate_ids.intersection_update(self.components[component_class].keys())

        return list(candidate_ids)

    def register_system(self, system):
        self.systems.append(system)

    def get_system(self, system_class):
        for system in self.systems:
            if isinstance(system, system_class):
                return system
        return None

    def set_resource(self, name, value):
        self.resources[name] = value

    def get_resource(self, name):
        return self.resources.get(name)

    def clear(self):
        self.entities.clear()
        self.components.clear()
        self.next_entity_id = 0
        # Note: systems and resources are kept, same as JS version

    def update(self, dt):
        pause_state = self.get_resource('pauseState')
        error_state = self.get_resource('errorState')

        is_paused = pause_state.paused if pause_state and hasattr(pause_state, 'paused') else False
        has_error = error_state.has_error if error_state and hasattr(error_state, 'hasError') else False

        for system in self.systems:
            if not hasattr(system, 'update'):
                continue

            run_in_pause = getattr(system, 'runInPause', False)
            if (not run_in_pause and is_paused) or has_error:
                continue

            system.update(self, dt)

# Using dataclasses for components is a common and clean practice in Python ECS.
# The `field` with `default_factory` is used to ensure that each component
# instance gets its own mutable numpy array, rather than sharing one.

@dataclass
class PositionComponent:
    """Stores the 3D position of an entity (with Z often unused for 2D physics)."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

@dataclass
class VelocityComponent:
    """Stores the 3D velocity of an entity (with Z often unused for 2D physics)."""
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

@dataclass
class MassComponent:
    """Stores the mass of an entity."""
    mass: float = 1.0

@dataclass
class RadiusComponent:
    """Stores the radius of an entity."""
    radius: float = 1.0

@dataclass
class OrientationComponent:
    """Stores the orientation (angle in radians) of an entity."""
    angle: float = 0.0

@dataclass
class AngularVelocityComponent:
    """Stores the angular velocity (radians per second) of an entity."""
    angular_velocity: float = 0.0

@dataclass
class MomentOfInertiaComponent:
    """
    Stores the moment of inertia for an entity.
    The inverse inertia is pre-calculated for use in physics calculations.
    """
    inertia: float = 1.0
    inv_inertia: float = field(init=False)

    def __post_init__(self):
        if self.inertia > 0 and np.isfinite(self.inertia):
            self.inv_inertia = 1.0 / self.inertia
        else:
            self.inv_inertia = 0.0

@dataclass
class CoefficientOfFrictionComponent:
    """Stores the coefficient of friction for an entity."""
    mu: float = 0.0

@dataclass
class CableLinkComponent:
    """
    Indicates an entity can be part of a cable.
    Corresponds to CableLinkComponent in JavaScript.
    """
    # These fields are from the JS version, for tracking previous state.
    prev_cable_attachment_time_pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    prev_cable_attachment_time_angle: float = 0.0

@dataclass
class GravityAffectedComponent:
    """A tag component for entities affected by gravity."""
    pass

@dataclass
class BallTagComponent:
    """A tag component for entities considered to be balls."""
    pass

@dataclass
class ObstacleTagComponent:
    """A tag component for entities considered to be obstacles."""
    pass

@dataclass
class ObstaclePushComponent:
    """Stores the push velocity for an obstacle entity."""
    pushVel: float = 2.0
    pass

@dataclass
class FlipperTagComponent:
    """A tag component for entities considered to be flippers."""
    pass

@dataclass
class FlipperStateComponent:
    """
    Represents the state of a flipper mechanism with rotational behavior.
    """
    length: float
    rest_angle: float
    max_rotation: float
    angular_velocity: float

    sign: float = field(init=False)  # Direction of rotation (+1 or -1)
    rotation: float = 0.0            # Current rotation from rest_angle
    current_angular_velocity: float = 0.0  # Angular velocity in last frame
    pressed: bool = False            # Was the flipper activated?

    def __post_init__(self):
        self.sign = math.copysign(1.0, self.max_rotation)
        self.max_rotation = abs(self.max_rotation)

@dataclass
class BorderComponent:
    """
    Stores a list of 2D or 3D points that define a border polygon or polyline.
    Each point is cloned on initialization to prevent shared references.
    """
    points: list = field(default_factory=list)

    def __init__(self, points=None):
        if points is None:
            self.points = []
        else:
            # Clone each point to avoid shared references
            self.points = [np.copy(p) for p in points]

@dataclass
class RenderableComponent:
    """
    Stores rendering info for an entity.
    'shape' can be 'circle', 'line', 'flipper', or 'border'.
    'color' is a hex string like '#888888'.
    """
    shape: str = 'circle'
    color: str = '#888888'

@dataclass
class ScoredTagComponent:
    """A tag component for entities considered to have scored."""
    pass

@dataclass
class RestitutionComponent:
    """Stores the restitution for an entity."""
    restitution: float = 0.5

@dataclass
class PrevFinalPosComponent:
    """Stores the 3D position of an entity (with Z often unused for 2D physics)."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

@dataclass
class PrevFinalOrientationComponent:
    """Stores the orientation (angle in radians) of an entity."""
    angle: float = 0.0

@dataclass
class PauseStateComponent:
    """Stores the state that says whether the simulation is paused or not."""
    paused: bool = True

@dataclass
class DistanceConstraintComponent:
    """
    Represents a distance constraint between two entities,
    following the XPBD formulation.
    """
    entityA: int
    entityB: int
    rest_length: float
    compliance: float = 0.0
    lambda_: float = 0.0  # Accumulated Lagrange multiplier
