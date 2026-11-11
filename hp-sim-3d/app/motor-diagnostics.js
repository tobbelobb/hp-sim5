import { MachineTagComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { SpoolStateComponent } from './hangprinter_spools.js';
import { StepperMotorComponent } from './hangprinter_stepper_motor.js';
import { STEPPER_CLOSED_LOOP_RESOURCE } from './hangprinter_runtime.js';

// The physics engine tracks slips every physics step: `missedSteps` counts
// whole electrical cycles the rotor fell into another detent (either
// direction) and `currentMissedSteps` is the signed net offset. A two-phase
// hybrid stepper has four full steps per electrical cycle.
export const FULL_STEPS_PER_SLIP = 4;

function sortMotorDiagnostics(a, b) {
  return a.axis.localeCompare(b.axis, undefined, { numeric: true, sensitivity: 'base' });
}

function machineSteppers(world, machineId) {
  const targetMachineId = typeof machineId === 'string' && machineId.length > 0 ? machineId : null;
  const result = [];
  for (const entityId of world.query([StepperMotorComponent, SpoolStateComponent])) {
    const entityMachineId = world.getComponent(entityId, MachineTagComponent)?.id || 'default';
    if (targetMachineId && entityMachineId !== targetMachineId) {
      continue;
    }
    result.push(entityId);
  }
  return result;
}

// Restarts the slip count only. The engine's encoder baseline (shared with the
// closed-loop servo's calibration) and the net offset stay, so a rotor that
// already sits in another detent is not counted again.
export function resetMachineMotorDiagnostics(world, machineId = null) {
  if (!world || typeof world.query !== 'function' || typeof world.getComponent !== 'function') {
    return;
  }
  for (const entityId of machineSteppers(world, machineId)) {
    world.getComponent(entityId, StepperMotorComponent).missedSteps = 0;
  }
}

export function getMachineMotorDiagnostics(world, machineId = null) {
  const empty = { totalMissedSteps: 0, motors: [] };
  if (!world || typeof world.query !== 'function' || typeof world.getComponent !== 'function') {
    return empty;
  }
  const closedLoopWorld = world.getResource?.(STEPPER_CLOSED_LOOP_RESOURCE) === true;
  const motors = [];
  for (const entityId of machineSteppers(world, machineId)) {
    const stepper = world.getComponent(entityId, StepperMotorComponent);
    const spoolState = world.getComponent(entityId, SpoolStateComponent);
    const axis = typeof spoolState.axis === 'string' && spoolState.axis.length > 0
      ? spoolState.axis.toUpperCase()
      : `Motor ${entityId}`;
    const slips = Number.isFinite(stepper.missedSteps) ? Math.max(0, Math.round(stepper.missedSteps)) : 0;
    const netSlips = Number.isFinite(stepper.currentMissedSteps) ? Math.round(stepper.currentMissedSteps) : 0;
    motors.push({
      axis,
      closedLoop: closedLoopWorld || Boolean(stepper.closedLoop),
      slips,
      netSlips,
      missedSteps: slips * FULL_STEPS_PER_SLIP,
    });
  }
  motors.sort(sortMotorDiagnostics);
  const totalMissedSteps = motors.reduce((sum, motor) => sum + motor.missedSteps, 0);
  return { totalMissedSteps, motors };
}
