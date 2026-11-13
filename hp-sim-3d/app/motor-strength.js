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

// Multiple machines may have different ratings; do not present their multiplier
// as a torque or imply that every motor has the same rating.
export function motorHoldingTorqueLabel(world) {
  const torques = world.query([StepperMotorComponent])
    .map(id => world.getComponent(id, StepperMotorComponent).holdingTorque);
  if (!torques.length) return '— Nm';
  const min = Math.min(...torques);
  const max = Math.max(...torques);
  const format = value => Number(value.toPrecision(3)).toString();
  return `${min === max ? format(min) : `${format(min)}–${format(max)}`} Nm`;
}
