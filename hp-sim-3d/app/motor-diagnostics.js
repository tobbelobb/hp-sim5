import {
  EncoderComponent,
  MachineTagComponent,
  OrientationComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import {
  SpoolStateComponent,
  getSpoolRotationAngle,
} from './hangprinter_spools.js';
import { StepperMotorComponent } from './hangprinter_stepper_motor.js';

const TWO_PI = Math.PI * 2;

function stepAngleFromPolePairs(numPolePairs) {
  const pairs = Math.max(1, Math.abs(Math.round(Number(numPolePairs) || 0)));
  return TWO_PI / pairs;
}

function countMissedSteps(actualAngle, targetAngle, numPolePairs) {
  if (!Number.isFinite(actualAngle) || !Number.isFinite(targetAngle)) {
    return 0;
  }
  const stepAngle = stepAngleFromPolePairs(numPolePairs);
  if (!(stepAngle > 0)) {
    return 0;
  }
  return Math.max(0, Math.round(Math.abs(actualAngle - targetAngle) / stepAngle));
}

function unwrapAngleNear(reference, wrappedValue) {
  let value = wrappedValue;
  while (value - reference > Math.PI) value -= Math.PI * 2.0;
  while (value - reference < -Math.PI) value += Math.PI * 2.0;
  return value;
}

function sortMotorDiagnostics(a, b) {
  return a.axis.localeCompare(b.axis, undefined, { numeric: true, sensitivity: 'base' });
}

export function getMachineMotorDiagnostics(world, machineId = null) {
  const empty = { totalMissedSteps: 0, motors: [] };
  if (!world || typeof world.query !== 'function' || typeof world.getComponent !== 'function') {
    return empty;
  }

  const targetMachineId = typeof machineId === 'string' && machineId.length > 0 ? machineId : null;
  const stepperEntities = world.query([StepperMotorComponent, SpoolStateComponent]);
  const motors = [];

  for (const entityId of stepperEntities) {
    const machineTag = world.getComponent(entityId, MachineTagComponent);
    const entityMachineId = machineTag?.id || 'default';
    if (targetMachineId && entityMachineId !== targetMachineId) {
      continue;
    }

    const stepper = world.getComponent(entityId, StepperMotorComponent);
    const spoolState = world.getComponent(entityId, SpoolStateComponent);
    if (!stepper || !spoolState) {
      continue;
    }

    const axis = typeof spoolState.axis === 'string' && spoolState.axis.length > 0
      ? spoolState.axis.toUpperCase()
      : `Motor ${entityId}`;
    let missedSteps = 0;
    if (!stepper.torqueMode) {
      const encoder = world.getComponent(entityId, EncoderComponent);
      const orientation = world.getComponent(entityId, OrientationComponent);
      const targetAngle = (stepper.commandedAngle || 0.0) - (stepper.deltaAngle || 0.0);
      const measuredAngle = Number.isFinite(encoder?.angle)
        ? encoder.angle
        : getSpoolRotationAngle(spoolState, orientation?.quaternion);
      const currentAngle = unwrapAngleNear(targetAngle, measuredAngle);
      missedSteps = countMissedSteps(currentAngle, targetAngle, stepper.numPolePairs);
    }

    motors.push({ axis, missedSteps });
  }

  motors.sort(sortMotorDiagnostics);
  const totalMissedSteps = motors.reduce((sum, motor) => sum + motor.missedSteps, 0);
  return { totalMissedSteps, motors };
}
