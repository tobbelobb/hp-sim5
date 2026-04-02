import {
  MachineTagComponent,
  OrientationComponent,
  RigidGroupComponent,
} from '../../src/js/cable_joints/ecs.js';
import {
  SpoolStateComponent,
  StepperMotorComponent,
} from '../../examples/js/slideprinter/slideprinter_common.js';

const TWO_PI = Math.PI * 2;

function buildGroupAngleByMember(world) {
  const groupAngleByMember = new Map();
  if (!world || typeof world.query !== 'function' || typeof world.getComponent !== 'function') {
    return groupAngleByMember;
  }
  try {
    const groupEntities = world.query([RigidGroupComponent]);
    for (const entityId of groupEntities) {
      const group = world.getComponent(entityId, RigidGroupComponent);
      const angle = Number.isFinite(group?.prevAngle) ? group.prevAngle : 0.0;
      const members = Array.isArray(group?.members) ? group.members : [];
      for (const memberId of members) {
        groupAngleByMember.set(memberId, angle);
      }
    }
  } catch (_err) {
    // Ignore worlds without rigid groups.
  }
  return groupAngleByMember;
}

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

function sortMotorDiagnostics(a, b) {
  return a.axis.localeCompare(b.axis, undefined, { numeric: true, sensitivity: 'base' });
}

export function getMachineMotorDiagnostics(world, machineId = null) {
  const empty = { totalMissedSteps: 0, motors: [] };
  if (!world || typeof world.query !== 'function' || typeof world.getComponent !== 'function') {
    return empty;
  }

  const targetMachineId = typeof machineId === 'string' && machineId.length > 0 ? machineId : null;
  const groupAngleByMember = buildGroupAngleByMember(world);
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
    const orientation = world.getComponent(entityId, OrientationComponent);
    if (!stepper || !spoolState) {
      continue;
    }

    const axis = typeof spoolState.axis === 'string' && spoolState.axis.length > 0
      ? spoolState.axis.toUpperCase()
      : `Motor ${entityId}`;
    let missedSteps = 0;
    if (!stepper.torqueMode) {
      const currentAngle = Number.isFinite(orientation?.angle) ? orientation.angle : 0.0;
      const groupAngle = groupAngleByMember.get(entityId) || 0.0;
      const targetAngle = groupAngle + ((stepper.commandedAngle || 0.0) - (stepper.deltaAngle || 0.0));
      missedSteps = countMissedSteps(currentAngle, targetAngle, stepper.numPolePairs);
    }

    motors.push({ axis, missedSteps });
  }

  motors.sort(sortMotorDiagnostics);
  const totalMissedSteps = motors.reduce((sum, motor) => sum + motor.missedSteps, 0);
  return { totalMissedSteps, motors };
}
