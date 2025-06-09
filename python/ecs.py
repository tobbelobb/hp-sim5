import numpy as np
from dataclasses import dataclass, field

# Using dataclasses for components is a common and clean practice in Python ECS.
# The `field` with `default_factory` is used to ensure that each component
# instance gets its own mutable numpy array, rather than sharing one.

@dataclass
class PositionComponent:
    """Stores the 2D position of an entity."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))

@dataclass
class VelocityComponent:
    """Stores the 2D velocity of an entity."""
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))

@dataclass
class MassComponent:
    """Stores the mass of an entity."""
    mass: float = 1.0

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
        if self.inertia > 0:
            self.inv_inertia = 1.0 / self.inertia
        else:
            self.inv_inertia = 0.0
