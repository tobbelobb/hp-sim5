"""Torque-mode spool ODE and implicit cable loading in the JS timestep order."""
import math

import numpy as np

from .ecs import AngularVelocityComponent, MomentOfInertiaComponent, OrientationComponent
from .inertia_tensor import effective_inertia_about_world_axis
from .spools import SpoolStateComponent, get_rigid_body_member_spool_frame, get_spool_rotation_angle, get_spool_world_axis
from .stepper_motor import (
    MOTOR_COMPONENTS, StepperMotorComponent, apply_rigid_body_reaction_angular_velocity,
    finite_number, set_member_spool_angle,
)


def compute_torque_mode_torque(stepper, angle, omega):
    max_speed = max(1e-6, stepper.max_speed_rad if stepper.max_speed_rad is not None else 600)
    droop = max(0., min(1., 1 - abs(omega) / max_speed))
    electrical = stepper.target_torque * droop
    damping = -(stepper.holding_torque / max_speed) * omega
    windage_coeff = stepper.windage_coeff if stepper.windage_coeff is not None else stepper.damping_coeff * 1e-3
    windage = -windage_coeff * omega * abs(omega)
    smooth_sign = omega / (abs(omega) + 1e-3)
    coulomb = stepper.coulomb_friction if stepper.coulomb_friction is not None else .002 * stepper.holding_torque
    stiction = stepper.stiction_torque if stepper.stiction_torque is not None else .003 * stepper.holding_torque
    stiction_speed = stepper.stiction_speed if stepper.stiction_speed is not None else 1.
    friction = -(coulomb + stiction * math.exp(-abs(omega) / stiction_speed)) * smooth_sign
    cogging = stepper.cogging_torque if stepper.cogging_torque is not None else .01 * stepper.holding_torque
    cogging_freq = stepper.cogging_freq if stepper.cogging_freq is not None else stepper.num_pole_pairs
    return electrical + damping + windage + friction - cogging * math.sin(cogging_freq * angle)


def read_torque_mode_cable_load(world, entity, key='torqueModeCableLoadTorques'):
    values = world.get_resource(key)
    value = values.get(entity) if isinstance(values, dict) else None
    if not finite_number(value):
        return 0.
    return value if key == 'torqueModeCableLoadTorques' else max(0., value)


class TorqueModeSystem:
    def update(self, world, dt):
        for entity in world.query(MOTOR_COMPONENTS):
            stepper = world.get_component(entity, StepperMotorComponent)
            if not stepper.torque_mode:
                continue
            state = world.get_component(entity, SpoolStateComponent)
            orientation = world.get_component(entity, OrientationComponent)
            velocity = world.get_component(entity, AngularVelocityComponent)
            moment = world.get_component(entity, MomentOfInertiaComponent)
            frame = get_rigid_body_member_spool_frame(world, entity, state)
            current = frame.world_orientation if frame else orientation.quaternion
            angle = get_spool_rotation_angle(frame.local_spool_state, frame.member.local_orientation) if frame else get_spool_rotation_angle(state, current)
            axis = get_spool_world_axis(state, current)
            omega = np.dot(velocity.omega, axis)
            inertia = effective_inertia_about_world_axis(moment, current, axis)
            if not inertia > 0:
                continue
            drive = compute_torque_mode_torque(stepper, angle, omega)
            load = read_torque_mode_cable_load(world, entity)
            stiffness = read_torque_mode_cable_load(world, entity, 'torqueModeCableLoadStiffnesses')
            damping = read_torque_mode_cable_load(world, entity, 'torqueModeCableLoadDampings')
            if stiffness > 0 and finite_number(dt) and dt > 0:
                spring_torque = load + damping * omega
                denominator = 1 + dt * damping / inertia + dt * dt * stiffness / inertia
                next_omega = (omega + dt / inertia * (drive + spring_torque)) / denominator
                delta_omega = next_omega - omega
            else:
                delta_omega = (drive + load) / inertia * dt
            velocity.omega += axis * delta_omega
            # Only electrical/mechanical drive reacts on the motor housing;
            # external cable loads already react through cable constraints.
            apply_rigid_body_reaction_angular_velocity(world, frame, axis, drive, dt)
            if frame:
                set_member_spool_angle(frame, orientation, angle + np.dot(velocity.omega, axis) * dt)
