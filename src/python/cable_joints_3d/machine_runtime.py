"""Machine command state and authored mapping order."""
from .stepper_motor import StepperMotorComponent


def set_stepper_torque_mode(world, entity, torque_nm):
    motor = world.get_component(entity, StepperMotorComponent)
    if motor is not None:
        motor.torque_mode = True
        motor.target_torque = torque_nm


def set_stepper_position_mode(world, entity):
    motor = world.get_component(entity, StepperMotorComponent)
    if motor is not None:
        motor.torque_mode = False
        motor.target_torque = 0.


def object_keys(mapping):
    # Object.keys visits canonical array-index strings before other keys.
    # This affects default-machine selection and command-axis traversal.
    indices, others = [], []
    for key in mapping:
        if isinstance(key, str) and key.isascii() and key.isdigit() and str(int(key)) == key and int(key) < 2**32 - 1:
            indices.append(key)
        else:
            others.append(key)
    return sorted(indices, key=int) + others
