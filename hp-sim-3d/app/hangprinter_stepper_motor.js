import {
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  MassComponent,
  RigidBodyComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import {
  getRigidBodyMemberComponent,
} from '../../src/js/cable_joints_3d/rigid_bodies.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import {
  addMatrix3,
  applyWorldInverseInertia,
  normalizeInertiaTensor,
  parallelAxisTensor,
  transformInertiaTensorToWorld,
} from '../../src/js/cable_joints_3d/inertia_tensor.js';

const EPSILON = 1e-12;

export function getRigidBodyMemberSpoolFrame(world, entityId, spoolState) {
  const member = getRigidBodyMemberComponent(world, entityId);
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
  const worldOrientation = new Quaternion()
    .multiplyQuaternions(bodyOrientation, member.localOrientation)
    .normalize();
  return {
    member,
    bodyOrientation,
    worldOrientation,
    localSpoolState: {
      axisLocal: spoolState.axisLocal,
      referenceOrientation: localReferenceOrientation,
    },
  };
}

function computeRigidBodyAggregateMomentOfInertia(world, bodyEntity) {
  const body = world.getComponent(bodyEntity, RigidBodyComponent);
  if (!Array.isArray(body?.members) || body.members.length < 1) {
    return null;
  }

  let tensor = normalizeInertiaTensor(0.0);
  let hasContribution = false;
  for (const entityId of body.members) {
    const member = getRigidBodyMemberComponent(world, entityId);
    if (!member?.localPosition) {
      continue;
    }

    const memberMoi = world.getComponent(entityId, MomentOfInertiaComponent);
    const memberMass = member.physicalMass ?? (world.getComponent(entityId, MassComponent)?.mass ?? 0.0);
    if (memberMoi) {
      const memberOrientation = member.localOrientation || new Quaternion();
      tensor = addMatrix3(
        tensor,
        transformInertiaTensorToWorld(memberMoi.inertiaTensor, memberOrientation),
      );
      hasContribution = true;
    }
    if (memberMass > 0.0) {
      tensor = addMatrix3(tensor, parallelAxisTensor(memberMass, member.localPosition));
      hasContribution = true;
    }
  }

  return hasContribution ? new MomentOfInertiaComponent(tensor) : null;
}

function getRigidBodyMomentOfInertia(world, bodyEntity) {
  return (
    computeRigidBodyAggregateMomentOfInertia(world, bodyEntity)
    || world.getComponent(bodyEntity, MomentOfInertiaComponent)
    || null
  );
}

export function applyRigidBodyReactionAngularVelocity(world, rigidBodyMemberFrame, worldAxis, totalTorque, dt) {
  const bodyEntity = rigidBodyMemberFrame?.member?.bodyEntity;
  if (bodyEntity === null || bodyEntity === undefined || Math.abs(totalTorque) <= EPSILON) {
    return;
  }

  const bodyMoi = getRigidBodyMomentOfInertia(world, bodyEntity);
  const bodyAngularVelocity = world.getComponent(bodyEntity, AngularVelocityComponent);
  const bodyOrientation = world.getComponent(bodyEntity, OrientationComponent)?.quaternion;
  if (!bodyAngularVelocity?.omega || !bodyMoi) {
    return;
  }

  const angularImpulse = worldAxis.clone().scale(-totalTorque * dt);
  bodyAngularVelocity.omega.add(applyWorldInverseInertia(bodyMoi, bodyOrientation, angularImpulse));
}

export class StepperMotorComponent {
  constructor(
    commandedAngle = 0.0,
    deltaAngle = 0.0,
    holdingTorque = 0.5,
    numPolePairs = 50,
    dampingCoeff = 0.01,
    maxSpeedRad = 600
  ) {
    this.commandedAngle = commandedAngle;
    this.deltaAngle = deltaAngle;
    this.holdingTorque = holdingTorque;
    this.numPolePairs = numPolePairs;
    this.dampingCoeff = dampingCoeff;
    this.maxSpeedRad = maxSpeedRad;
    this.torqueMode = false;
    this.targetTorque = 0.0;
    this.closedLoop = false;
    this.missedSteps = 0;
    this.currentMissedSteps = 0;
    this.missedStepEncoderOffset = null;
    // Closed-loop servo state, kept across world rebuilds such as grabs.
    this.servoEngaged = false;
    this.servoIntegralTorque = 0.0;
    this.servoError = 0.0;
  }
}
