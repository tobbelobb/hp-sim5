from dataclasses import dataclass, field
import math

import sys
from pathlib import Path

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from cable_joints.ecs import (
    OrientationComponent,
    AngularVelocityComponent
)


@dataclass
class SpoolTagComponent:
    """A tag component for entities that are spools."""
    pass

@dataclass
class SpoolStateComponent:
    """Represents the state of a spool mechanism with rotational behavior."""
    target_angle: float = 0.0
    current_angle: float = 0.0
    speed: float = 0.0  # rad/min

class SlideprinterMotionSystem:
    def update(self, world, dt):
        for entity_id in world.query([SpoolStateComponent, SpoolTagComponent, OrientationComponent, AngularVelocityComponent]):

            state = world.get_component(entity_id, SpoolStateComponent)

            delta_angle = state.target_angle - state.current_angle
            if abs(delta_angle) < 1e-6:
                continue

            speed_rad_per_sec = state.speed / 60.0
            max_rotation_this_frame = speed_rad_per_sec * dt

            if abs(delta_angle) < max_rotation_this_frame:
                state.current_angle = state.target_angle
            else:
                state.current_angle += math.copysign(max_rotation_this_frame, delta_angle)

            orientation = world.get_component(entity_id, OrientationComponent)
            angular_vel = world.get_component(entity_id, AngularVelocityComponent)
            angular_vel.angular_velocity = (state.current_angle - orientation.angle)/dt
            orientation.angle = state.current_angle
            print("Set angular_velocity to", angular_vel.angular_velocity)
            print("Set angle to", orientation.angle)

