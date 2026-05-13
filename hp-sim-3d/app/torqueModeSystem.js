import {
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import {
  StepperMotorComponent,
  applyRigidBodyReactionAngularVelocity,
  getRigidBodyMemberSpoolFrame,
} from './hangprinter_stepper_motor.js';
import {
  SpoolStateComponent,
  composeSpoolOrientation,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
} from './hangprinter_spools.js';
import { effectiveInertiaAboutWorldAxis } from '../../src/js/cable_joints_3d/inertia_tensor.js';

export function computeTorqueModeTorque(stepper, currentAngle, omegaAlongAxis) {
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

  return (
    electricalTorque
    + dampingTorque
    + windageTorque
    + frictionTorque
    + coggingTorque
  );
}

export function readTorqueModeCableLoadTorque(world, entityId) {
  const loads = world?.getResource?.('torqueModeCableLoadTorques');
  const load = loads instanceof Map
    ? loads.get(entityId)
    : loads?.[entityId];
  return Number.isFinite(load) ? load : 0.0;
}

function readTorqueModeCableLoadScalar(world, resourceName, entityId) {
  const values = world?.getResource?.(resourceName);
  const value = values instanceof Map
    ? values.get(entityId)
    : values?.[entityId];
  return Number.isFinite(value) ? Math.max(0.0, value) : 0.0;
}

export class TorqueModeSystem {
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
      if (!stepper?.torqueMode) {
        continue;
      }

      const spoolState = world.getComponent(entityId, SpoolStateComponent);
      const orient = world.getComponent(entityId, OrientationComponent);
      const angVel = world.getComponent(entityId, AngularVelocityComponent);
      const inertia = world.getComponent(entityId, MomentOfInertiaComponent);
      if (!spoolState || !orient || !angVel || !inertia) {
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
      if (!(axisInertia > 0.0)) {
        continue;
      }
      const driveTorque = computeTorqueModeTorque(stepper, currentAngle, omegaAlongAxis);
      const cableLoadTorque = readTorqueModeCableLoadTorque(world, entityId);
      const cableLoadStiffness = readTorqueModeCableLoadScalar(
        world,
        'torqueModeCableLoadStiffnesses',
        entityId,
      );
      const cableLoadDamping = readTorqueModeCableLoadScalar(
        world,
        'torqueModeCableLoadDampings',
        entityId,
      );

      let deltaOmegaAlongAxis = 0.0;
      if (
        cableLoadStiffness > 0.0
        && Number.isFinite(dt)
        && dt > 0.0
      ) {
        const springLoadTorque = cableLoadTorque + (cableLoadDamping * omegaAlongAxis);
        const implicitDenom = 1.0
          + ((dt * cableLoadDamping) / axisInertia)
          + ((dt * dt * cableLoadStiffness) / axisInertia);
        const nextOmegaAlongAxis = (
          omegaAlongAxis
          + ((dt / axisInertia) * (driveTorque + springLoadTorque))
        ) / implicitDenom;
        deltaOmegaAlongAxis = nextOmegaAlongAxis - omegaAlongAxis;
      } else {
        const totalTorque = driveTorque + cableLoadTorque;
        deltaOmegaAlongAxis = (totalTorque / axisInertia) * dt;
      }

      angVel.omega.add(worldAxis, deltaOmegaAlongAxis);
      // This is a custom motor/reaction path for rigid-body-member spools. A
      // full XPBD hinge motor would instead solve body <-> rotor constraints
      // and distribute off-axis corrections through inverse inertia.
      applyRigidBodyReactionAngularVelocity(world, rigidBodyMemberFrame, worldAxis, driveTorque, dt);
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
