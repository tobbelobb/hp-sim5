"""Persistent missed-step state and machine reports with JS rounding semantics."""
import math
import re
import unicodedata

from .ecs import EncoderComponent, MachineTagComponent, OrientationComponent
from .spools import SpoolStateComponent, get_rigid_body_member_spool_frame, get_spool_rotation_angle
from .stepper_motor import StepperMotorComponent, finite_number

TWO_PI = 2 * math.pi


def _round(value):
    # Python round uses ties-to-even; Math.round uses ties toward +infinity.
    lower = math.floor(value)
    return lower + int(value - lower >= .5)


def _unwrap_near(reference, angle):
    while angle - reference > math.pi:
        angle -= TWO_PI
    while angle - reference < -math.pi:
        angle += TWO_PI
    return angle


def _step_angle(pole_pairs):
    try:
        pairs = float(pole_pairs) if pole_pairs is not None else 0.
    except (TypeError, ValueError):
        pairs = 0.
    if math.isnan(pairs):
        pairs = 0.
    if math.isinf(pairs):
        return 0.
    return TWO_PI / max(1, abs(_round(pairs)))


def update_stepper_missed_step_state(world, entity):
    stepper = world.get_component(entity, StepperMotorComponent)
    state = world.get_component(entity, SpoolStateComponent)
    if stepper is None or state is None:
        return 0
    previous_peak = max(0, _round(stepper.missed_steps)) if finite_number(stepper.missed_steps) else 0
    if stepper.torque_mode:
        stepper.current_missed_steps = 0
        stepper.missed_steps = previous_peak
        return previous_peak
    command = stepper.commanded_angle if finite_number(stepper.commanded_angle) else 0.
    offset = stepper.delta_angle if finite_number(stepper.delta_angle) else 0.
    target = command - offset
    encoder = world.get_component(entity, EncoderComponent)
    if encoder is not None and finite_number(encoder.angle):
        if not finite_number(stepper.missed_step_encoder_offset):
            difference = encoder.angle - target
            stepper.missed_step_encoder_offset = _round(difference / TWO_PI) * TWO_PI if finite_number(difference) else 0.
        measured = encoder.angle - stepper.missed_step_encoder_offset
    else:
        frame = get_rigid_body_member_spool_frame(world, entity, state)
        orientation = world.get_component(entity, OrientationComponent)
        angle = get_spool_rotation_angle(frame.local_spool_state, frame.member.local_orientation) if frame else get_spool_rotation_angle(state, orientation.quaternion if orientation else None)
        measured = _unwrap_near(target, angle)
    angle_per_step = _step_angle(stepper.num_pole_pairs)
    current = abs(_round((measured - target) / angle_per_step)) if finite_number(measured) and finite_number(target) and angle_per_step > 0 else 0
    stepper.current_missed_steps = current
    stepper.missed_steps = max(previous_peak, current)
    return stepper.missed_steps


def _machine_motors(world, machine_id):
    target = machine_id if isinstance(machine_id, str) and machine_id else None
    for entity in world.query([StepperMotorComponent, SpoolStateComponent]):
        tag = world.get_component(entity, MachineTagComponent)
        owner = tag.id if tag is not None and tag.id else 'default'
        if target is None or owner == target:
            yield entity


def reset_machine_motor_diagnostics(world, machine_id=None):
    for entity in _machine_motors(world, machine_id):
        stepper = world.get_component(entity, StepperMotorComponent)
        stepper.missed_steps = stepper.current_missed_steps = 0
        stepper.missed_step_encoder_offset = None


def _axis_sort_key(motor):
    text = ''.join(c for c in unicodedata.normalize('NFD', motor['axis'].casefold()) if not unicodedata.combining(c))
    return tuple((1, int(part)) if part.isdigit() else (0, part) for part in re.split(r'(\d+)', text))


def get_machine_motor_diagnostics(world, machine_id=None):
    motors = []
    for entity in _machine_motors(world, machine_id):
        stepper = world.get_component(entity, StepperMotorComponent)
        state = world.get_component(entity, SpoolStateComponent)
        axis = state.axis.upper() if isinstance(state.axis, str) and state.axis else f'Motor {entity}'
        missed = update_stepper_missed_step_state(world, entity) if not stepper.torque_mode else 0
        motors.append({'axis': axis, 'missedSteps': missed})
    motors.sort(key=_axis_sort_key)
    return {'totalMissedSteps': sum(motor['missedSteps'] for motor in motors), 'motors': motors}


class MissedStepTrackingSystem:
    def update(self, world, dt_unused):
        for entity in world.query([StepperMotorComponent, SpoolStateComponent]):
            update_stepper_missed_step_state(world, entity)
