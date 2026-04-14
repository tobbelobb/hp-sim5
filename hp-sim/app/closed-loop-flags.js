import { STEPPER_CLOSED_LOOP_RESOURCE } from '../../example_apps/js/slideprinter/slideprinter_common.js';

export const CLOSED_LOOP_MOTOR_TOGGLE_KEYS = Object.freeze([
  STEPPER_CLOSED_LOOP_RESOURCE,
]);

export function setClosedLoopMotorFeatureFlags(world, enabled) {
  if (world === null || world === undefined || typeof world.setResource !== 'function') {
    return;
  }
  const next = enabled === true;
  for (const key of CLOSED_LOOP_MOTOR_TOGGLE_KEYS) {
    world.setResource(key, next);
  }
}
