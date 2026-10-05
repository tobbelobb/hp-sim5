"""Motor state, position drive and the specialized rigid-member reaction path."""
from dataclasses import dataclass
import math
import numbers

import numpy as np

from .ecs import (
    AngularVelocityComponent, EncoderComponent, MassComponent, MomentOfInertiaComponent,
    OrientationComponent, RigidBodyComponent, RigidBodyMemberComponent,
)
from .inertia_tensor import (
    apply_world_inverse_inertia, effective_inertia_about_world_axis,
    parallel_axis_tensor, transform_inertia_tensor_to_world,
)
from .quaternion import Quaternion
from .spools import (
    SpoolStateComponent, compose_spool_orientation, get_rigid_body_member_spool_frame,
    get_spool_rotation_angle, get_spool_world_axis, normalize_angle,
)


@dataclass
class StepperMotorComponent:
    commanded_angle: float = 0.
    delta_angle: float = 0.
    holding_torque: float = .5
    num_pole_pairs: float = 50
    damping_coeff: float = .01
    max_speed_rad: float = 600
    torque_mode: bool = False
    target_torque: float = 0.
    closed_loop: bool = False
    missed_steps: float = 0
    current_missed_steps: float = 0
    missed_step_encoder_offset: float | None = None
    windage_coeff: float | None = None
    coulomb_friction: float | None = None
    stiction_torque: float | None = None
    stiction_speed: float | None = None
    cogging_torque: float | None = None
    cogging_freq: float | None = None


MOTOR_COMPONENTS = [StepperMotorComponent, SpoolStateComponent, OrientationComponent,
                    AngularVelocityComponent, MomentOfInertiaComponent]


def finite_number(value):
    return isinstance(value, numbers.Real) and not isinstance(value, bool) and math.isfinite(value)


def finite_resource(world, key, fallback):
    value = world.get_resource(key)
    return value if finite_number(value) else fallback


def is_stepper_closed_loop_enabled(world, stepper):
    return (stepper is not None and stepper.closed_loop is True) or world.get_resource('closedLoopMotorsEnabled') is True


def position_stepper_constraint_stiffness(world, stepper):
    if stepper is None or stepper.torque_mode:
        return 0.
    if is_stepper_closed_loop_enabled(world, stepper):
        return max(0., finite_resource(world, 'closedLoopStepperConstraintStiffness', 1e8))
    torque = max(0., stepper.holding_torque) if finite_number(stepper.holding_torque) else 0.
    poles = max(1., abs(stepper.num_pole_pairs)) if finite_number(stepper.num_pole_pairs) else 1.
    return torque * poles * max(0., finite_resource(world, 'openLoopStepperConstraintStiffnessScale', 1.))


def effective_motorized_spin_inv_inertia(world, entity, inv_inertia, inertia, dt):
    if not inv_inertia > 1e-9:
        return 0.
    stepper = world.get_component(entity, StepperMotorComponent)
    if stepper is not None and stepper.torque_mode:
        return 0.
    stiffness = position_stepper_constraint_stiffness(world, stepper)
    if not stiffness > 1e-9 or not finite_number(dt) or dt <= 1e-9:
        return inv_inertia
    effective = (inertia if finite_number(inertia) and inertia > 1e-9 else 1 / inv_inertia) + stiffness * dt * dt
    return 1 / effective if effective > 1e-9 else inv_inertia


def open_loop_stepper_holding_torque_limit(world, stepper):
    if stepper is None or stepper.torque_mode or is_stepper_closed_loop_enabled(world, stepper):
        return math.inf
    torque = max(0., stepper.holding_torque) if finite_number(stepper.holding_torque) else 0.
    return torque * max(0., finite_resource(world, 'openLoopStepperHoldingTorqueScale', 1.))


def rigid_body_motor_moment(world, body_entity):
    """Motor reactions prefer the live member aggregate over the authored tensor."""
    body = world.get_component(body_entity, RigidBodyComponent)
    tensor, has_contribution = np.zeros((3, 3)), False
    for entity in body.members if body is not None else []:
        member = world.get_component(entity, RigidBodyMemberComponent)
        if member is None:
            continue
        moment = world.get_component(entity, MomentOfInertiaComponent)
        mass = world.get_component(entity, MassComponent)
        physical_mass = member.physical_mass if member.physical_mass is not None else (mass.mass if mass else 0.)
        if moment is not None:
            tensor += transform_inertia_tensor_to_world(moment.inertia_tensor, member.local_orientation)
            has_contribution = True
        if physical_mass > 0:
            tensor += parallel_axis_tensor(physical_mass, member.local_position)
            has_contribution = True
    return MomentOfInertiaComponent(tensor) if has_contribution else world.get_component(body_entity, MomentOfInertiaComponent)


def apply_rigid_body_reaction_rotation(world, frame, axis, motor_angle_delta, rotor_inertia):
    if frame is None or abs(motor_angle_delta) <= 1e-12:
        return
    moment = rigid_body_motor_moment(world, frame.member.body_entity)
    inertia = effective_inertia_about_world_axis(moment, frame.body_orientation, axis)
    rotor_inertia = max(0., rotor_inertia) if finite_number(rotor_inertia) else 0.
    if inertia > 1e-12 and rotor_inertia > 1e-12:
        delta = Quaternion().set_from_axis_angle(axis, -motor_angle_delta * (rotor_inertia / inertia))
        frame.body_orientation.premultiply(delta).normalize()


def apply_rigid_body_reaction_angular_velocity(world, frame, axis, torque, dt):
    if frame is None or abs(torque) <= 1e-12:
        return
    moment = rigid_body_motor_moment(world, frame.member.body_entity)
    velocity = world.get_component(frame.member.body_entity, AngularVelocityComponent)
    if moment is not None and velocity is not None:
        velocity.omega += apply_world_inverse_inertia(moment, frame.body_orientation, axis * (-torque * dt))


def add_encoder_angle(world, entity, delta):
    encoder = world.get_component(entity, EncoderComponent)
    if finite_number(delta) and abs(delta) > 1e-12 and encoder is not None and finite_number(encoder.angle):
        encoder.angle += delta


def set_member_spool_angle(frame, orientation, angle):
    frame.member.local_orientation.set(compose_spool_orientation(frame.local_spool_state, None, angle))
    orientation.quaternion.set(frame.body_orientation.copy().multiply(frame.member.local_orientation).normalize())


class StepperMotorSystem:
    def update(self, world, dt):
        for entity in world.query(MOTOR_COMPONENTS):
            stepper = world.get_component(entity, StepperMotorComponent)
            if stepper.torque_mode:
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
            target = stepper.commanded_angle - stepper.delta_angle
            if is_stepper_closed_loop_enabled(world, stepper):
                delta = -normalize_angle(angle - target)
                # The captured frame retains the live body's quaternion object.
                # Recomposition must see its reaction rotation immediately.
                apply_rigid_body_reaction_rotation(world, frame, axis, delta, inertia)
                if frame:
                    set_member_spool_angle(frame, orientation, target)
                else:
                    orientation.quaternion.set(compose_spool_orientation(state, None, target))
                add_encoder_angle(world, entity, delta)
                velocity.omega[:] = 0.
                continue
            error = normalize_angle(angle - target)
            torque = -stepper.holding_torque * math.sin(stepper.num_pole_pairs * error) - stepper.damping_coeff * omega
            if not inertia > 1e-12:
                continue
            velocity.omega += axis * (torque / inertia * dt)
            apply_rigid_body_reaction_angular_velocity(world, frame, axis, torque, dt)
            if frame:
                # Member rotors integrate here; AngularMovementSystem excludes
                # them. Standalone rotors integrate later in that system.
                set_member_spool_angle(frame, orientation, angle + np.dot(velocity.omega, axis) * dt)
