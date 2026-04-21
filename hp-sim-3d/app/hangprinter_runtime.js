import { StepperMotorComponent } from './hangprinter_stepper_motor.js';

export const STEPPER_CLOSED_LOOP_RESOURCE = 'closedLoopMotorsEnabled';

export function isStepperClosedLoopEnabled(world, stepper) {
  if (stepper?.closedLoop === true) {
    return true;
  }
  return world?.getResource?.(STEPPER_CLOSED_LOOP_RESOURCE) === true;
}

export function setStepperTorqueMode(world, entity, torqueNm) {
  const stepper = world.getComponent(entity, StepperMotorComponent);
  if (stepper) {
    stepper.torqueMode = true;
    stepper.targetTorque = torqueNm;
  }
}

export function setStepperPositionMode(world, entity) {
  const stepper = world.getComponent(entity, StepperMotorComponent);
  if (stepper) {
    stepper.torqueMode = false;
    stepper.targetTorque = 0;
  }
}
