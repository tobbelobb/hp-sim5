import {
  EncoderComponent,
  MachineTagComponent,
  OrientationComponent,
  RigidBodyMemberComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
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

function missedStepIndex(actualAngle, targetAngle, numPolePairs) {
  if (!Number.isFinite(actualAngle) || !Number.isFinite(targetAngle)) {
    return 0;
  }
  const stepAngle = stepAngleFromPolePairs(numPolePairs);
  if (!(stepAngle > 0)) {
    return 0;
  }
  return Math.round((actualAngle - targetAngle) / stepAngle);
}

function unwrapAngleNear(reference, wrappedValue) {
  let value = wrappedValue;
  while (value - reference > Math.PI) value -= Math.PI * 2.0;
  while (value - reference < -Math.PI) value += Math.PI * 2.0;
  return value;
}

function nearestFullTurnOffset(value) {
  if (!Number.isFinite(value)) {
    return 0.0;
  }
  return Math.round(value / TWO_PI) * TWO_PI;
}

function sortMotorDiagnostics(a, b) {
  return a.axis.localeCompare(b.axis, undefined, { numeric: true, sensitivity: 'base' });
}

function targetAngleForStepper(stepper) {
  const commandedAngle = Number.isFinite(stepper?.commandedAngle) ? stepper.commandedAngle : 0.0;
  const deltaAngle = Number.isFinite(stepper?.deltaAngle) ? stepper.deltaAngle : 0.0;
  return commandedAngle - deltaAngle;
}

function rigidBodyMemberSpoolAngle(world, entityId, spoolState) {
  const member = world.getComponent(entityId, RigidBodyMemberComponent);
  if (!member?.localOrientation) {
    return null;
  }
  const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
  if (!bodyOrientation) {
    return null;
  }
  const bodyInverse = bodyOrientation.clone().conjugate().normalize();
  const localReferenceOrientation = new Quaternion()
    .multiplyQuaternions(bodyInverse, spoolState.referenceOrientation)
    .normalize();
  return getSpoolRotationAngle(
    {
      axisLocal: spoolState.axisLocal,
      referenceOrientation: localReferenceOrientation,
    },
    member.localOrientation,
  );
}

function measuredAngleForStepper(world, entityId, stepper, spoolState, targetAngle) {
  const encoder = world.getComponent(entityId, EncoderComponent);
  if (Number.isFinite(encoder?.angle)) {
    if (!Number.isFinite(stepper.missedStepEncoderOffset)) {
      stepper.missedStepEncoderOffset = nearestFullTurnOffset(encoder.angle - targetAngle);
    }
    return encoder.angle - stepper.missedStepEncoderOffset;
  }

  const memberAngle = rigidBodyMemberSpoolAngle(world, entityId, spoolState);
  if (Number.isFinite(memberAngle)) {
    return unwrapAngleNear(targetAngle, memberAngle);
  }

  const orientation = world.getComponent(entityId, OrientationComponent);
  return unwrapAngleNear(targetAngle, getSpoolRotationAngle(spoolState, orientation?.quaternion));
}

export function updateStepperMissedStepState(world, entityId) {
  const stepper = world?.getComponent?.(entityId, StepperMotorComponent);
  const spoolState = world?.getComponent?.(entityId, SpoolStateComponent);
  if (!stepper || !spoolState) {
    return 0;
  }

  if (stepper.torqueMode) {
    stepper.currentMissedSteps = 0;
    stepper.missedSteps = Number.isFinite(stepper.missedSteps)
      ? Math.max(0, Math.round(stepper.missedSteps))
      : 0;
    return stepper.missedSteps;
  }

  const targetAngle = targetAngleForStepper(stepper);
  const measuredAngle = measuredAngleForStepper(world, entityId, stepper, spoolState, targetAngle);
  const currentMissedSteps = Math.abs(missedStepIndex(measuredAngle, targetAngle, stepper.numPolePairs));
  stepper.currentMissedSteps = currentMissedSteps;
  stepper.missedSteps = Math.max(
    Number.isFinite(stepper.missedSteps) ? Math.max(0, Math.round(stepper.missedSteps)) : 0,
    currentMissedSteps,
  );
  return stepper.missedSteps;
}

export function resetMachineMotorDiagnostics(world, machineId = null) {
  if (!world || typeof world.query !== 'function' || typeof world.getComponent !== 'function') {
    return;
  }
  const targetMachineId = typeof machineId === 'string' && machineId.length > 0 ? machineId : null;
  for (const entityId of world.query([StepperMotorComponent, SpoolStateComponent])) {
    const machineTag = world.getComponent(entityId, MachineTagComponent);
    const entityMachineId = machineTag?.id || 'default';
    if (targetMachineId && entityMachineId !== targetMachineId) {
      continue;
    }
    const stepper = world.getComponent(entityId, StepperMotorComponent);
    if (!stepper) {
      continue;
    }
    stepper.missedSteps = 0;
    stepper.currentMissedSteps = 0;
    stepper.missedStepEncoderOffset = null;
  }
}

export class MissedStepTrackingSystem {
  runInPause = false;

  update(world, _dt) {
    const entities = world.query([StepperMotorComponent, SpoolStateComponent]);
    for (const entityId of entities) {
      updateStepperMissedStepState(world, entityId);
    }
  }
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
      missedSteps = updateStepperMissedStepState(world, entityId);
    }

    motors.push({ axis, missedSteps });
  }

  motors.sort(sortMotorDiagnostics);
  const totalMissedSteps = motors.reduce((sum, motor) => sum + motor.missedSteps, 0);
  return { totalMissedSteps, motors };
}
