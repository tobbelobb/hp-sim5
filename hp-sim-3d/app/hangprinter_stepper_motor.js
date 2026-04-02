import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import {
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidGroupComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import { isStepperClosedLoopEnabled } from './hangprinter_runtime.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0.0, 0.0, 1.0);

function normalizeAngle(angle) {
  let normalized = angle;
  while (normalized > Math.PI) normalized -= 2.0 * Math.PI;
  while (normalized < -Math.PI) normalized += 2.0 * Math.PI;
  return normalized;
}

function getPlanarAngle(quaternion) {
  if (!quaternion || typeof quaternion.transformVector !== 'function') {
    return 0.0;
  }
  const axis = quaternion.transformVector(new Vector3(1.0, 0.0, 0.0));
  return Math.atan2(axis.y, axis.x);
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
      OrientationComponent,
      AngularVelocityComponent,
      MomentOfInertiaComponent,
    ];

    const groupAngleByMember = new Map();
    try {
      const groups = world.query([RigidGroupComponent]);
      for (const groupId of groups) {
        const group = world.getComponent(groupId, RigidGroupComponent);
        const angle = group?.prevAngle || 0.0;
        const members = group?.members || [];
        for (const memberId of members) {
          groupAngleByMember.set(memberId, angle);
        }
      }
    } catch (_err) {
      // Ignore if the scene does not use rigid groups.
    }

    for (const entityId of world.query(query)) {
      const stepper = world.getComponent(entityId, StepperMotorComponent);
      const orient = world.getComponent(entityId, OrientationComponent);
      const angVel = world.getComponent(entityId, AngularVelocityComponent);
      const inertia = world.getComponent(entityId, MomentOfInertiaComponent);
      if (!stepper || !orient || !angVel || !inertia) {
        continue;
      }

      const currentAngle = getPlanarAngle(orient.quaternion);
      const omegaZ = angVel.omega?.z ?? 0.0;
      let totalTorque;

      if (stepper.torqueMode) {
        const maxSpeedRad = Math.max(1e-6, stepper.maxSpeedRad ?? 600);
        const droop = Math.max(0, Math.min(1, 1 - Math.abs(omegaZ) / maxSpeedRad));
        const electricalTorque = stepper.targetTorque * droop;
        const dampingTorque = -(stepper.holdingTorque / maxSpeedRad) * omegaZ;
        const windageCoeff = stepper.windageCoeff ?? (stepper.dampingCoeff * 1e-3);
        const windageTorque = -windageCoeff * omegaZ * Math.abs(omegaZ);

        const epsW = 1e-3;
        const smoothSign = omegaZ / (Math.abs(omegaZ) + epsW);
        const coulomb = stepper.coulombFriction ?? (0.002 * stepper.holdingTorque);
        const stiction = stepper.stictionTorque ?? (0.003 * stepper.holdingTorque);
        const stictionSpeed = stepper.stictionSpeed ?? 1.0;
        const stictionFactor = Math.exp(-Math.abs(omegaZ) / stictionSpeed);
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
        const groupAngle = groupAngleByMember.get(entityId) || 0.0;
        const targetWorldAngle = groupAngle + (stepper.commandedAngle - stepper.deltaAngle);
        if (isStepperClosedLoopEnabled(world, stepper)) {
          orient.quaternion.setFromAxisAngle(DEFAULT_PLANE_NORMAL, targetWorldAngle).normalize();
          angVel.omega.z = 0.0;
          continue;
        }
        const error = normalizeAngle(currentAngle - targetWorldAngle);
        const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);
        const dampingTorque = -stepper.dampingCoeff * omegaZ;
        totalTorque = restoringTorque + dampingTorque;
      }

      const angularAcceleration = totalTorque / inertia.inertia;
      angVel.omega.z += angularAcceleration * dt;
    }
  }
}
