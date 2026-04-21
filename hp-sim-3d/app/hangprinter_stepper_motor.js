import {
  EncoderComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
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
  getRigidBodyEntityForMember,
} from '../../src/js/cable_joints_3d/rigid_bodies.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';

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
      const encoder = world.getComponent(entityId, EncoderComponent);
      if (encoder) {
        encoder.angle = currentAngle;
        encoder.axis = worldAxis.clone();
      }
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
        if (isStepperClosedLoopEnabled(world, stepper)) {
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
          angVel.omega.x = 0.0;
          angVel.omega.y = 0.0;
          angVel.omega.z = 0.0;
          if (encoder) {
            encoder.angle = targetAngle;
            encoder.axis = getSpoolWorldAxis(spoolState, orient.quaternion);
          }
          continue;
        }
        const error = normalizeAngle(currentAngle - targetAngle);
        const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);
        const dampingTorque = -stepper.dampingCoeff * omegaAlongAxis;
        totalTorque = restoringTorque + dampingTorque;
      }

      const angularAcceleration = totalTorque / inertia.inertia;
      angVel.omega.add(worldAxis, angularAcceleration * dt);
      if (rigidBodyMemberFrame) {
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

      // This block has to do with reaction torque, which should really be handled by its own system,
      // maybe call it RigidBodyReactionTorqueSystem
      const bodyEntity = getRigidBodyEntityForMember(world, entityId);
      if (bodyEntity !== null && bodyEntity !== undefined) {
        const bodyAngularVelocity = world.getComponent(bodyEntity, AngularVelocityComponent);
        const bodyInertia = world.getComponent(bodyEntity, MomentOfInertiaComponent);
        if (bodyAngularVelocity?.omega && bodyInertia?.inertia > 0.0) {
          const bodyDeltaOmega = -(inertia.inertia / bodyInertia.inertia) * angularAcceleration * dt;
          bodyAngularVelocity.omega.add(worldAxis, bodyDeltaOmega);
        }
      }
    }
  }
}
