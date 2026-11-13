import { StepperMotorComponent } from './hangprinter_stepper_motor.js';

// Scale every stepper's holding torque relative to its machine's own value.
export function setMotorStrength(world, factor) {
  if (!Number.isFinite(factor) || factor <= 0) {
    throw new RangeError('motorStrength must be a positive number');
  }
  world.setResource('motorStrength', factor);
  syncMotorStrength(world);
}

export function syncMotorStrength(world) {
  const factor = world.getResource('motorStrength');
  for (const id of world.query([StepperMotorComponent])) {
    const stepper = world.getComponent(id, StepperMotorComponent);
    stepper.machineHoldingTorque ??= stepper.holdingTorque;
    stepper.holdingTorque = factor * stepper.machineHoldingTorque;
  }
}
