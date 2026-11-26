import {
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import {
  cloneMatrix3,
  normalizeInertiaTensor,
} from '../../src/js/cable_joints_3d/inertia_tensor.js';
import {
  SpoolTagComponent,
  SpoolStateComponent,
  composeSpoolOrientation,
} from './hangprinter_spools.js';
import {
  StepperMotorComponent,
  getRigidBodyMemberSpoolFrame,
} from './hangprinter_stepper_motor.js';

export const STEPPER_CLOSED_LOOP_RESOURCE = 'closedLoopMotorsEnabled';
export const STEPPER_KINEMATIC_RESOURCE = 'kinematicMotors';

const savedSpoolInertia = new WeakMap();

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

export function applyKinematicSpoolAngle(world, entityId, targetAngle) {
  if (!world || !Number.isFinite(targetAngle)) {
    return;
  }
  const spoolState = world.getComponent(entityId, SpoolStateComponent);
  const orientation = world.getComponent(entityId, OrientationComponent);
  if (!spoolState || !orientation?.quaternion) {
    return;
  }
  const memberFrame = getRigidBodyMemberSpoolFrame(world, entityId, spoolState);
  if (memberFrame) {
    const localOrientation = composeSpoolOrientation(memberFrame.localSpoolState, null, targetAngle);
    memberFrame.member.localOrientation.set(localOrientation);
    orientation.quaternion.multiplyQuaternions(memberFrame.bodyOrientation, localOrientation).normalize();
  } else {
    orientation.quaternion.set(composeSpoolOrientation(spoolState, null, targetAngle));
  }
  const angularVelocity = world.getComponent(entityId, AngularVelocityComponent);
  if (angularVelocity?.omega) {
    angularVelocity.omega.set({ x: 0, y: 0, z: 0 });
  }
}

export function setKinematicMotorMode(world, enabled) {
  if (!world) {
    return;
  }
  const active = Boolean(enabled);
  const wasActive = world.getResource(STEPPER_KINEMATIC_RESOURCE) === true;
  world.setResource(STEPPER_KINEMATIC_RESOURCE, active);

  let changed = active !== wasActive;
  const backend = world.getResource('cableSolverBackend');
  if (changed && backend?.invalidate) {
    backend.invalidate();
  }

  const zeroTensor = normalizeInertiaTensor(0);
  for (const entityId of world.query([SpoolTagComponent, StepperMotorComponent])) {
    const moi = world.getComponent(entityId, MomentOfInertiaComponent);
    if (active) {
      if (moi) {
        if (!savedSpoolInertia.has(moi)) {
          savedSpoolInertia.set(moi, {
            inertiaTensor: cloneMatrix3(moi.inertiaTensor),
            invInertiaTensor: cloneMatrix3(moi.invInertiaTensor),
            inertia: moi.inertia,
            invInertia: moi.invInertia,
          });
        }
        moi.inertiaTensor = cloneMatrix3(zeroTensor);
        moi.tensor = moi.inertiaTensor;
        moi.invInertiaTensor = cloneMatrix3(zeroTensor);
        moi.inverseInertiaTensor = moi.invInertiaTensor;
        moi.inertia = 0;
        moi.invInertia = 0;
        changed = true;
      }
      const angularVelocity = world.getComponent(entityId, AngularVelocityComponent);
      if (angularVelocity?.omega) {
        angularVelocity.omega.set({ x: 0, y: 0, z: 0 });
      }
    } else if (moi && savedSpoolInertia.has(moi)) {
      const saved = savedSpoolInertia.get(moi);
      moi.inertiaTensor = cloneMatrix3(saved.inertiaTensor);
      moi.tensor = moi.inertiaTensor;
      moi.invInertiaTensor = cloneMatrix3(saved.invInertiaTensor);
      moi.inverseInertiaTensor = moi.invInertiaTensor;
      moi.inertia = saved.inertia;
      moi.invInertia = saved.invInertia;
      savedSpoolInertia.delete(moi);
      changed = true;
    }
  }

  if (changed && backend?.invalidate) {
    backend.invalidate();
  }
}
