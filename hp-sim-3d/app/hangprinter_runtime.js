import Vector3 from '../../src/js/cable_joints_3d/vector3.js';

import {
  PositionComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidGroupComponent,
  MachineTagComponent,
} from '../../src/js/cable_joints_3d/ecs.js';

export {
  ExtruderComponent,
  SpoolTagComponent,
  SpoolStateComponent,
  StepperMotorComponent,
  STEPPER_CLOSED_LOOP_RESOURCE,
  RemoteSpoolSystem,
  setStepperTorqueMode,
  setStepperPositionMode,
  isStepperInTorqueMode,
  isStepperClosedLoopEnabled,
  getStepperTorque,
} from '../../examples/js/slideprinter/slideprinter_common.js';

import {
  ExtruderComponent,
  SpoolTagComponent,
  StepperMotorComponent,
  isStepperClosedLoopEnabled,
} from '../../examples/js/slideprinter/slideprinter_common.js';

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

export class ExtruderSystem {
  update(world, dt) {
    let extruderComp = null;
    for (const entityId of world.query([ExtruderComponent])) {
      extruderComp = world.getComponent(entityId, ExtruderComponent);
      break;
    }

    if (!extruderComp) {
      return;
    }

    const spoolEntities = world.query([SpoolTagComponent, PositionComponent]);
    const sumByMachine = {};
    const countByMachine = {};

    for (const entityId of spoolEntities) {
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (!pos) {
        continue;
      }
      const machineTag = world.getComponent(entityId, MachineTagComponent);
      const machineId = machineTag?.id || 'default';
      let sum = sumByMachine[machineId];
      if (!sum) {
        sum = new Vector3();
        sumByMachine[machineId] = sum;
        countByMachine[machineId] = 0;
      }
      sum.add(pos);
      countByMachine[machineId] += 1;
    }

    const centerSources = extruderComp.centerSources && typeof extruderComp.centerSources === 'object'
      ? extruderComp.centerSources
      : {};
    const sourceMachineIds = Object.keys(centerSources);

    const resolveAverage = (entityIds) => {
      if (!Array.isArray(entityIds) || entityIds.length === 0) {
        return null;
      }
      const sum = new Vector3();
      let count = 0;
      for (const entityId of entityIds) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        if (pos) {
          sum.add(pos);
          count += 1;
        }
      }
      if (count === 0) {
        return null;
      }
      return sum.clone().scale(1.0 / count);
    };

    const centerOffsets = extruderComp.centerOffsets && typeof extruderComp.centerOffsets === 'object'
      ? extruderComp.centerOffsets
      : extruderComp.extruderOffsets && typeof extruderComp.extruderOffsets === 'object'
        ? extruderComp.extruderOffsets
      : {};
    const machineCenters = {};
    for (const machineId of sourceMachineIds) {
      const entityIds = centerSources[machineId];
      let center = resolveAverage(entityIds);
      if (!center && sumByMachine[machineId] && (countByMachine[machineId] ?? 0) > 0) {
        center = sumByMachine[machineId].clone().scale(1.0 / countByMachine[machineId]);
      }
      if (center) {
        const offset = centerOffsets[machineId];
        if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
          center = center.clone().add(offset);
        }
        machineCenters[machineId] = center;
      }
    }

    if (sourceMachineIds.length === 0) {
      for (const machineId of Object.keys(sumByMachine)) {
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          const center = sumByMachine[machineId].clone().scale(1.0 / count);
          const offset = centerOffsets[machineId];
          if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
            center.add(offset);
          }
          machineCenters[machineId] = center;
        }
      }
    } else {
      for (const machineId of Object.keys(sumByMachine)) {
        if (machineCenters[machineId]) {
          continue;
        }
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          const center = sumByMachine[machineId].clone().scale(1.0 / count);
          const offset = centerOffsets[machineId];
          if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
            center.add(offset);
          }
          machineCenters[machineId] = center;
        }
      }
    }

    const machineIds = Object.keys(machineCenters);
    if (machineIds.length > 0) {
      extruderComp.machineCenters = machineCenters;
      const preferredOrder = sourceMachineIds.length > 0 ? sourceMachineIds : machineIds;
      let chosenId = preferredOrder.find((id) => machineCenters[id]) || null;
      if (!chosenId) {
        chosenId = machineIds[0];
      }
      extruderComp.centerPos = machineCenters[chosenId].clone();
    }
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
          orient.quaternion.setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), targetWorldAngle).normalize();
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
