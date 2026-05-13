import {
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  MassComponent,
  RigidBodyComponent,
  EncoderComponent,
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
import {
  addMatrix3,
  applyWorldInverseInertia,
  effectiveInertiaAboutWorldAxis,
  normalizeInertiaTensor,
  parallelAxisTensor,
  transformInertiaTensorToWorld,
} from '../../src/js/cable_joints_3d/inertia_tensor.js';

const EPSILON = 1e-12;

function normalizeAngle(angle) {
  let normalized = angle;
  while (normalized > Math.PI) normalized -= 2.0 * Math.PI;
  while (normalized < -Math.PI) normalized += 2.0 * Math.PI;
  return normalized;
}

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

function getRigidBodyEffectiveInertiaAboutAxis(world, bodyEntity, worldAxis) {
  const axis = worldAxis?.clone?.();
  const bodyMoi = getRigidBodyMomentOfInertia(world, bodyEntity);
  const bodyOrientation = world.getComponent(bodyEntity, OrientationComponent)?.quaternion;
  if (!axis || axis.lengthSq() <= EPSILON) {
    return bodyMoi?.inertia ?? 0.0;
  }
  axis.normalize();
  return effectiveInertiaAboutWorldAxis(bodyMoi, bodyOrientation, axis);
}

function applyRigidBodyReactionRotation(world, rigidBodyMemberFrame, worldAxis, motorAngleDelta, rotorInertia) {
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
  const motorRotorInertia = Number.isFinite(rotorInertia) ? Math.max(0.0, rotorInertia) : 0.0;
  if (!bodyOrientation || !(bodyInertia > EPSILON) || !(motorRotorInertia > EPSILON)) {
    return;
  }

  const reactionFraction = motorRotorInertia / bodyInertia;
  const dq = new Quaternion().setFromAxisAngle(worldAxis, -motorAngleDelta * reactionFraction);
  bodyOrientation.multiplyQuaternions(dq, bodyOrientation).normalize();
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

function addEncoderAngle(world, entityId, deltaAngle) {
  if (!(Number.isFinite(deltaAngle) && Math.abs(deltaAngle) > EPSILON)) {
    return;
  }
  const encoder = world.getComponent(entityId, EncoderComponent);
  if (encoder && Number.isFinite(encoder.angle)) {
    encoder.angle += deltaAngle;
  }
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
      if (stepper.torqueMode) {
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
      const axisInertia = effectiveInertiaAboutWorldAxis(inertia, currentOrientation, worldAxis);

      if (isStepperClosedLoopEnabled(world, stepper)) {
        const targetAngle = stepper.commandedAngle - stepper.deltaAngle;
        const motorAngleDelta = -normalizeAngle(currentAngle - targetAngle);

        applyRigidBodyReactionRotation(
          world,
          rigidBodyMemberFrame,
          worldAxis,
          motorAngleDelta,
          axisInertia,
        );

        if (rigidBodyMemberFrame) {
          rigidBodyMemberFrame.member.localOrientation.set(
            composeSpoolOrientation(rigidBodyMemberFrame.localSpoolState, null, targetAngle),
          );
          orient.quaternion.set(
            new Quaternion()
              .multiplyQuaternions(
                rigidBodyMemberFrame.bodyOrientation,
                rigidBodyMemberFrame.member.localOrientation,
              )
              .normalize(),
          );
        } else {
          orient.quaternion.set(composeSpoolOrientation(spoolState, null, targetAngle));
        }
        addEncoderAngle(world, entityId, motorAngleDelta);
        angVel.omega.x = 0.0;
        angVel.omega.y = 0.0;
        angVel.omega.z = 0.0;
        continue;
      }

      const targetAngle = stepper.commandedAngle - stepper.deltaAngle;
      const error = normalizeAngle(currentAngle - targetAngle);
      const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);
      const dampingTorque = -stepper.dampingCoeff * omegaAlongAxis;
      const totalTorque = restoringTorque + dampingTorque;

      if (!(axisInertia > EPSILON)) {
        continue;
      }
      const angularAcceleration = totalTorque / axisInertia;
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
