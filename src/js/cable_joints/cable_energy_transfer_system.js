import {
  CablePathComponent,
  CableJointComponent
} from './cable_joints_core.js';
import {
  MassComponent,
  VelocityComponent,
  MomentOfInertiaComponent,
  AngularVelocityComponent
} from './ecs.js';

const PREV_POTENTIAL_RESOURCE = 'cableJointPotentialPrev';
const DIAG_RESOURCE = 'cableEnergyTransferDiag';
const EPSILON = 1e-12;

function _jointPotential(path, joint) {
  if (!path || !joint) {
    return 0.0;
  }
  const k = path.spring_constant;
  if (!Number.isFinite(k) || k <= 0.0) {
    return 0.0;
  }
  const length = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
  const stretch = Math.max(0.0, length - joint.restLength);
  return 0.5 * k * stretch * stretch;
}

function _removeRotationalEnergy(world, entityIds, energyToRemove) {
  if (!(energyToRemove > EPSILON)) {
    return 0.0;
  }

  const entries = [];
  let total = 0.0;
  for (const entityId of entityIds) {
    const moiComp = world.getComponent(entityId, MomentOfInertiaComponent);
    const angComp = world.getComponent(entityId, AngularVelocityComponent);
    if (!moiComp || !angComp || !(moiComp.inertia > EPSILON)) {
      continue;
    }
    const omega = angComp.angularVelocity;
    const energy = 0.5 * moiComp.inertia * omega * omega;
    if (!(energy > EPSILON)) {
      continue;
    }
    entries.push({
      entityId,
      inertia: moiComp.inertia,
      omega,
      energy
    });
    total += energy;
  }

  if (!(total > EPSILON)) {
    return 0.0;
  }

  const removed = Math.min(energyToRemove, total);
  for (const entry of entries) {
    const share = removed * (entry.energy / total);
    const nextEnergy = Math.max(0.0, entry.energy - share);
    const nextOmegaMag = Math.sqrt((2.0 * nextEnergy) / entry.inertia);
    const angComp = world.getComponent(entry.entityId, AngularVelocityComponent);
    if (angComp) {
      const sign = entry.omega >= 0.0 ? 1.0 : -1.0;
      angComp.angularVelocity = sign * nextOmegaMag;
    }
  }
  return removed;
}

function _removeLinearEnergy(world, entityIds, energyToRemove) {
  if (!(energyToRemove > EPSILON)) {
    return 0.0;
  }

  const entries = [];
  let total = 0.0;
  for (const entityId of entityIds) {
    const massComp = world.getComponent(entityId, MassComponent);
    const velComp = world.getComponent(entityId, VelocityComponent);
    if (!massComp || !velComp || !(massComp.mass > EPSILON)) {
      continue;
    }
    const speedSq = velComp.vel.lengthSq();
    if (!(speedSq > EPSILON)) {
      continue;
    }
    const energy = 0.5 * massComp.mass * speedSq;
    if (!(energy > EPSILON)) {
      continue;
    }
    entries.push({
      entityId,
      mass: massComp.mass,
      energy
    });
    total += energy;
  }

  if (!(total > EPSILON)) {
    return 0.0;
  }

  const removed = Math.min(energyToRemove, total);
  for (const entry of entries) {
    const share = removed * (entry.energy / total);
    const nextEnergy = Math.max(0.0, entry.energy - share);
    const velComp = world.getComponent(entry.entityId, VelocityComponent);
    if (!velComp) {
      continue;
    }
    const speedSq = velComp.vel.lengthSq();
    if (!(speedSq > EPSILON)) {
      continue;
    }
    const nextSpeed = Math.sqrt((2.0 * nextEnergy) / entry.mass);
    const speed = Math.sqrt(speedSq);
    const scale = speed > EPSILON ? (nextSpeed / speed) : 0.0;
    velComp.vel.scale(scale);
  }
  return removed;
}

export class CableEnergyTransferSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const tuning = world.getResource('cableEnergyTransferTuning') || {};
    if (tuning.enabled === false) {
      world.setResource(DIAG_RESOURCE, {
        positivePotentialJoints: 0,
        energyRemovedRotational: 0.0,
        energyRemovedLinear: 0.0,
        unmetPotentialRise: 0.0,
        maxJointPotentialRise: 0.0
      });
      return;
    }

    const transferScale = Number.isFinite(tuning.transferScale)
      ? Math.max(0.0, tuning.transferScale)
      : 1.0;

    const prevResource = world.getResource(PREV_POTENTIAL_RESOURCE);
    const prevByJoint = (prevResource instanceof Map) ? prevResource : new Map();
    const nextByJoint = new Map();

    const diag = {
      positivePotentialJoints: 0,
      energyRemovedRotational: 0.0,
      energyRemovedLinear: 0.0,
      unmetPotentialRise: 0.0,
      maxJointPotentialRise: 0.0
    };

    const pathEntities = world.query([CablePathComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || !Array.isArray(path.jointEntities)) {
        continue;
      }
      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        if (!joint) {
          continue;
        }
        const potential = _jointPotential(path, joint);
        nextByJoint.set(jointId, potential);
        const prevPotential = Number.isFinite(prevByJoint.get(jointId)) ? prevByJoint.get(jointId) : 0.0;
        const rise = potential - prevPotential;
        if (!(rise > EPSILON)) {
          continue;
        }

        diag.positivePotentialJoints += 1;
        diag.maxJointPotentialRise = Math.max(diag.maxJointPotentialRise, rise);

        let remaining = rise * transferScale;
        const jointEntities = [joint.entityA, joint.entityB];

        const rotRemoved = _removeRotationalEnergy(world, jointEntities, remaining);
        remaining -= rotRemoved;
        diag.energyRemovedRotational += rotRemoved;

        if (remaining > EPSILON) {
          const linRemoved = _removeLinearEnergy(world, jointEntities, remaining);
          remaining -= linRemoved;
          diag.energyRemovedLinear += linRemoved;
        }

        if (remaining > EPSILON) {
          diag.unmetPotentialRise += remaining;
        }
      }
    }

    world.setResource(PREV_POTENTIAL_RESOURCE, nextByJoint);
    world.setResource(DIAG_RESOURCE, diag);
  }
}

