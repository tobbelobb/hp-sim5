from dataclasses import dataclass, field
import math

import sys
from pathlib import Path

import numpy as np

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from cable_joints.ecs import (
    OrientationComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent
)


@dataclass
class SpoolTagComponent:
    """A tag component for entities that are spools."""
    pass

@dataclass
class SpoolStateComponent:
    """Represents the state of a spool mechanism with rotational behavior."""
    axis: str = None


# Adapted from the Stepper Motor Model for Dynamic Simulation paper by Alexandru Morar
@dataclass
class StepperMotorComponent:
    """Holds the state and physical properties of a stepper motor."""
    commanded_angle: float = 0.0
    delta_angle: float = 0.0
    holding_torque: float = 0.5  # Nm. Within the typical range for Nema 17 motors
    num_pole_pairs: int = 50     # For a 1.8 deg/step motor
    damping_coeff: float = 0.01   # Gotten by trial and error. For stepper inertia 5e-05

# Adapted from the Stepper Motor Model for Dynamic Simulation paper by Alexandru Morar
class StepperMotorSystem:
    def update(self, world, dt):
        query = [
            StepperMotorComponent,
            OrientationComponent,
            AngularVelocityComponent,
            MomentOfInertiaComponent,
        ]
        for e in world.query(query):
            stepper = world.get_component(e, StepperMotorComponent)
            orient = world.get_component(e, OrientationComponent)
            ang_vel = world.get_component(e, AngularVelocityComponent)
            inertia = world.get_component(e, MomentOfInertiaComponent)

            # Calculate restoring torque based on angular error
            error = orient.angle - (stepper.commanded_angle - stepper.delta_angle)
            restoring_torque = -stepper.holding_torque * np.sin(stepper.num_pole_pairs * error)

            # Add damping to help the motor settle
            damping_torque = -stepper.damping_coeff * ang_vel.angular_velocity

            total_torque = restoring_torque + damping_torque

            # Apply torque to angular velocity (F=ma -> a=F/m -> v=v+a*dt)
            angular_acceleration = total_torque / inertia.inertia
            ang_vel.angular_velocity += angular_acceleration * dt
