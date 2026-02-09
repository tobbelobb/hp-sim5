import Vector2 from './vector2.js';
import {
  CableJointComponent,
  CablePathComponent,
  calculateAttachmentPoints
} from './cable_joints_core.js';
import {
  PositionComponent,
  MassComponent,
  MomentOfInertiaComponent,
  OrientationComponent
} from './ecs.js';

function ensureArrayMapEntry(map, key, value) {
  if (!map.has(key)) {
    map.set(key, []);
  }
  if (value !== undefined) {
    map.get(key).push(value);
  }
}

export class PBDResolveCableOverCorrections {
  runInPause = false;

  update(world, _dt_unused) {
    const pathEntities = world.query([CablePathComponent]);
    if (!pathEntities) return;
    const diag = {
      overCorrectedJointCount: 0,
      entitiesPosAdjusted: 0,
      entitiesAngAdjusted: 0,
      maxPosAdjustment: 0.0,
      maxAngAdjustment: 0.0
    };

    const jointToPathAndIndex = new Map();
    const allJointIds = new Set();

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path.jointEntities || path.jointEntities.length === 0) continue;

      for (let i = 0; i < path.jointEntities.length; i++) {
        const jointId = path.jointEntities[i];
        allJointIds.add(jointId);
        jointToPathAndIndex.set(jointId, { path, i });
      }
    }

    const overCorrected = [];
    for (const jointId of allJointIds) {
      const joint = world.getComponent(jointId, CableJointComponent);
      const preLen = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
      if (preLen >= joint.restLength) {
        const data = jointToPathAndIndex.get(jointId);
        if (!data) continue;
        const { path, i } = data;
        const { attachmentA_current: pA, attachmentB_current: pB } = calculateAttachmentPoints(world, joint, path, i);
        if (!pA || !pB) continue;
        const postLen = pA.distanceTo(pB);
        if (postLen < joint.restLength) {
          overCorrected.push(jointId);
        }
      }
    }

    diag.overCorrectedJointCount = overCorrected.length;
    if (overCorrected.length < 2) {
      world.setResource('cableOverCorrectionDiag', diag);
      return;
    }

    const posCorrections = new Map();
    const angCorrections = new Map();

    for (const jointId of overCorrected) {
      this.calculateJointCorrection(world, jointId, jointToPathAndIndex, posCorrections, angCorrections);
    }

    for (const [entityId, deltas] of posCorrections.entries()) {
      if (deltas.length >= 2) {
        // Multiple joints connected to this one body are slack now
        // They should not have been considered to have such a great effect on
        // the body during this time step. Correct that back by averaging.
        const avg = deltas.reduce((acc, v) => acc.add(v), new Vector2()).scale(1 / deltas.length);
        const posComp = world.getComponent(entityId, PositionComponent);
        if (posComp) {
          posComp.pos.add(avg);
          diag.entitiesPosAdjusted += 1;
          diag.maxPosAdjustment = Math.max(diag.maxPosAdjustment, avg.length());
        }
      }
    }

    for (const [entityId, deltas] of angCorrections.entries()) {
      if (deltas.length >= 2) {
        // Multiple joints connected to this one body are slack now
        // They should not have been considered to have such a great effect on
        // the body during this time step. Correct that back by averaging.
        const sum = deltas.reduce((a, b) => a + b, 0);
        const avg = sum / deltas.length;
        const orientComp = world.getComponent(entityId, OrientationComponent);
        if (orientComp) {
          orientComp.angle += avg;
          diag.entitiesAngAdjusted += 1;
          diag.maxAngAdjustment = Math.max(diag.maxAngAdjustment, Math.abs(avg));
        }
      }
    }
    world.setResource('cableOverCorrectionDiag', diag);
  }

  calculateJointCorrection(world, jointId, jointToPathAndIndex, posCorrections, angCorrections) {
    const joint = world.getComponent(jointId, CableJointComponent);
    const data = jointToPathAndIndex.get(jointId);
    if (!data) return;
    const { path, i } = data;

    const { attachmentA_current: pA, attachmentB_current: pB } = calculateAttachmentPoints(world, joint, path, i);
    if (!pA || !pB) return;

    const currentLen = pA.distanceTo(pB);
    const constraintError = currentLen - joint.restLength;
    const epsilon = 1e-9;
    if (constraintError >= -epsilon) return;

    const entityA = joint.entityA;
    const entityB = joint.entityB;

    const massAComp = world.getComponent(entityA, MassComponent);
    const invMassA = massAComp && massAComp.mass > 0 && Number.isFinite(massAComp.mass) ? 1.0 / massAComp.mass : 0.0;
    const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
    const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;

    const massBComp = world.getComponent(entityB, MassComponent);
    const invMassB = massBComp && massBComp.mass > 0 && Number.isFinite(massBComp.mass) ? 1.0 / massBComp.mass : 0.0;
    const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
    const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

    if (invMassA + invMassB + invInertiaA + invInertiaB <= epsilon) return;

    const diff = new Vector2().subtractVectors(pB, pA);
    const len = diff.length();
    if (len <= epsilon) return;
    const dir = diff.clone().scale(1 / len);

    const gradPosA = dir.clone().scale(-1);
    const gradPosB = dir.clone();

    const posAComp = world.getComponent(entityA, PositionComponent);
    const rA = new Vector2().subtractVectors(pA, posAComp.pos);
    const gradAngA = rA.x * dir.y - rA.y * dir.x;

    const posBComp = world.getComponent(entityB, PositionComponent);
    const rB = new Vector2().subtractVectors(pB, posBComp.pos);
    const gradAngB = rB.x * (-dir.y) - rB.y * (-dir.x);

    let denom = 0;
    denom += invMassA * gradPosA.lengthSq();
    denom += invInertiaA * gradAngA * gradAngA;
    denom += invMassB * gradPosB.lengthSq();
    denom += invInertiaB * gradAngB * gradAngB;

    const dt = world.getResource('dt');
    if (dt !== undefined && dt > 0) {
      denom += path.compliance / (dt * dt);
    }

    if (denom <= epsilon) return;

    const lambda = -constraintError / denom;

    if (invMassA > 0) {
      const delta = gradPosA.clone().scale(invMassA * lambda);
      ensureArrayMapEntry(posCorrections, entityA, delta);
    }
    if (invInertiaA > 0) {
      const delta = -invInertiaA * lambda * gradAngA;
      ensureArrayMapEntry(angCorrections, entityA, delta);
    }
    if (invMassB > 0) {
      const delta = gradPosB.clone().scale(invMassB * lambda);
      ensureArrayMapEntry(posCorrections, entityB, delta);
    }
    if (invInertiaB > 0) {
      const delta = -invInertiaB * lambda * gradAngB;
      ensureArrayMapEntry(angCorrections, entityB, delta);
    }
  }
}
