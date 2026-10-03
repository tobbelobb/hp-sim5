"""Hangprinter motor state and cable-solver holding semantics."""
from dataclasses import dataclass
import math
import numbers


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
