import {
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  MassComponent,
  RigidBodyComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import { isStepperClosedLoopEnabled } from './hangprinter_runtime.js';
import {
  SpoolStateComponent,
  composeSpoolOrientation,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
} from './hangprinter_spools.js';
import {
  getRigidBodyMemberComponent,
} from '../../src/js/cable_joints_3d/rigid_bodies.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';

const EPSILON = 1e-12;

function normalizeAngle(angle) {
  let normalized = angle;
  while (normalized > Math.PI) normalized -= 2.0 * Math.PI;
  while (normalized < -Math.PI) normalized += 2.0 * Math.PI;
  return normalized;
}

function getRigidBodyMemberSpoolFrame(world, entityId, spoolState) {
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

function getRigidBodyEffectiveInertiaAboutAxis(world, bodyEntity, worldAxis) {
  const axis = worldAxis?.clone?.();
  if (!axis || axis.lengthSq() <= EPSILON) {
    return world.getComponent(bodyEntity, MomentOfInertiaComponent)?.inertia ?? 0.0;
  }
  axis.normalize();

  const body = world.getComponent(bodyEntity, RigidBodyComponent);
  const bodyOrientation = world.getComponent(bodyEntity, OrientationComponent)?.quaternion;
  const members = Array.isArray(body?.members) ? body.members : [];
  if (!bodyOrientation || members.length < 1) {
    return world.getComponent(bodyEntity, MomentOfInertiaComponent)?.inertia ?? 0.0;
  }

  let totalInertia = 0.0;
  for (const entityId of members) {
    const member = getRigidBodyMemberComponent(world, entityId);
    if (!member?.localPosition) {
      continue;
    }

    const memberInertia = world.getComponent(entityId, MomentOfInertiaComponent)?.inertia ?? 0.0;
    const memberMass = member.physicalMass ?? (world.getComponent(entityId, MassComponent)?.mass ?? 0.0);
    const offsetWorld = bodyOrientation.transformVector(member.localPosition);
    const perpendicularMomentArmSq = offsetWorld.cross(axis).lengthSq();
    totalInertia += memberInertia + (memberMass * perpendicularMomentArmSq);
  }

  if (totalInertia > EPSILON) {
    return totalInertia;
  }
  return world.getComponent(bodyEntity, MomentOfInertiaComponent)?.inertia ?? 0.0;
}

function applyRigidBodyReactionRotation(world, rigidBodyMemberFrame, worldAxis, motorAngleDelta, spoolInertia) {
  const bodyEntity = rigidBodyMemberFrame?.member?.bodyEntity;
  if (
    bodyEntity === null
    || bodyEntity === undefined
    || Math.abs(motorAngleDelta) <= EPSILON
  ) {
    return;
  }

  const bodyInertia = getRigidBodyEffectiveInertiaAboutAxis(world, bodyEntity, worldAxis);
  const bodyOrientation = world.getComponent(bodyEntity, OrientationComponent)?.quaternion;
  const motorRotorInertia = Number.isFinite(spoolInertia) ? Math.max(0.0, spoolInertia) : 0.0;
  if (!bodyOrientation || !(bodyInertia > EPSILON) || !(motorRotorInertia > EPSILON)) {
    return;
  }

  const reactionFraction = motorRotorInertia / bodyInertia;
  const dq = new Quaternion().setFromAxisAngle(worldAxis, -motorAngleDelta * reactionFraction);
  bodyOrientation.multiplyQuaternions(dq, bodyOrientation).normalize();
}

function applyRigidBodyReactionAngularVelocity(world, rigidBodyMemberFrame, worldAxis, totalTorque, dt) {
  const bodyEntity = rigidBodyMemberFrame?.member?.bodyEntity;
  if (bodyEntity === null || bodyEntity === undefined || Math.abs(totalTorque) <= EPSILON) {
    return;
  }

  const bodyInertia = getRigidBodyEffectiveInertiaAboutAxis(world, bodyEntity, worldAxis);
  const bodyAngularVelocity = world.getComponent(bodyEntity, AngularVelocityComponent);
  if (!bodyAngularVelocity?.omega || !(bodyInertia > EPSILON)) {
    return;
  }

  const bodyAngularAcceleration = -totalTorque / bodyInertia;
  bodyAngularVelocity.omega.add(worldAxis, bodyAngularAcceleration * dt);
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
  }
}

export class StepperMotorSystem {
  update(world, dt) {
    const query = [
      StepperMotorComponent,
      SpoolStateComponent,
      OrientationComponent,
      AngularVelocityComponent,
      MomentOfInertiaComponent,
    ];

    for (const entityId of world.query(query)) {
      const stepper = world.getComponent(entityId, StepperMotorComponent);
      const spoolState = world.getComponent(entityId, SpoolStateComponent);
      const orient = world.getComponent(entityId, OrientationComponent);
      const angVel = world.getComponent(entityId, AngularVelocityComponent);
      const inertia = world.getComponent(entityId, MomentOfInertiaComponent);
      if (!stepper || !spoolState || !orient || !angVel || !inertia) {
        continue;
      }

      const rigidBodyMemberFrame = getRigidBodyMemberSpoolFrame(world, entityId, spoolState);
      const currentOrientation = rigidBodyMemberFrame?.worldOrientation || orient.quaternion;
      const currentAngle = rigidBodyMemberFrame
        ? getSpoolRotationAngle(
          rigidBodyMemberFrame.localSpoolState,
          rigidBodyMemberFrame.member.localOrientation,
        )
        : getSpoolRotationAngle(spoolState, currentOrientation);
      const worldAxis = getSpoolWorldAxis(spoolState, currentOrientation);
      const omegaAlongAxis = angVel.omega?.dot?.(worldAxis) ?? 0.0;
      let totalTorque;

      if (stepper.torqueMode) {
        const maxSpeedRad = Math.max(1e-6, stepper.maxSpeedRad ?? 600);
        const droop = Math.max(0, Math.min(1, 1 - Math.abs(omegaAlongAxis) / maxSpeedRad));
        const electricalTorque = stepper.targetTorque * droop;
        const dampingTorque = -(stepper.holdingTorque / maxSpeedRad) * omegaAlongAxis;
        const windageCoeff = stepper.windageCoeff ?? (stepper.dampingCoeff * 1e-3);
        const windageTorque = -windageCoeff * omegaAlongAxis * Math.abs(omegaAlongAxis);

        const epsW = 1e-3;
        const smoothSign = omegaAlongAxis / (Math.abs(omegaAlongAxis) + epsW);
        const coulomb = stepper.coulombFriction ?? (0.002 * stepper.holdingTorque);
        const stiction = stepper.stictionTorque ?? (0.003 * stepper.holdingTorque);
        const stictionSpeed = stepper.stictionSpeed ?? 1.0;
        const stictionFactor = Math.exp(-Math.abs(omegaAlongAxis) / stictionSpeed);
        const frictionTorque = -(coulomb + stiction * stictionFactor) * smoothSign;

        const cogAmp = stepper.coggingTorque ?? (0.01 * stepper.holdingTorque);
        const cogFreq = stepper.coggingFreq ?? stepper.numPolePairs;
        const coggingTorque = -cogAmp * Math.sin(cogFreq * currentAngle);

        totalTorque =
          electricalTorque +
          dampingTorque +
          windageTorque +
          frictionTorque +
          coggingTorque;
      } else {
        const targetAngle = stepper.commandedAngle - stepper.deltaAngle;
        const error = normalizeAngle(currentAngle - targetAngle);
        const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);
        const dampingTorque = -stepper.dampingCoeff * omegaAlongAxis;
        totalTorque = restoringTorque + dampingTorque;
      }

      const angularAcceleration = totalTorque / inertia.inertia;
      angVel.omega.add(worldAxis, angularAcceleration * dt);
      // This is a custom motor/reaction path for rigid-body-member spools. A
      // full XPBD hinge motor would instead solve body <-> rotor constraints
      // and distribute off-axis corrections through inverse inertia.
      applyRigidBodyReactionAngularVelocity(world, rigidBodyMemberFrame, worldAxis, totalTorque, dt);
      if (rigidBodyMemberFrame) {
        // Integrate the rotor's free twist directly in member-local space.
        // RigidBodySyncSystem later recomposes body orientation + local twist
        // and clamps any swing away.
        const integratedAngle = currentAngle + ((angVel.omega?.dot?.(worldAxis) ?? 0.0) * dt);
        rigidBodyMemberFrame.member.localOrientation.set(
          composeSpoolOrientation(rigidBodyMemberFrame.localSpoolState, null, integratedAngle),
        );
        orient.quaternion.set(
          new Quaternion()
            .multiplyQuaternions(
              rigidBodyMemberFrame.bodyOrientation,
              rigidBodyMemberFrame.member.localOrientation,
            )
            .normalize(),
        );
      }
    }
  }
}
