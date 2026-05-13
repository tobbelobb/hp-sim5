import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';
import {
  CableLinkComponent,
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
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  computeWorldAttachment,
  getEntityWorldPosition,
  getRigidBodyEntityForMember,
  resolveRigidBodySolverEndpoint,
  updateRigidBodyMemberLocalOrientation,
} from './rigid_bodies.js';
import {
  applyWorldInverseInertia,
  hasAnyInverseInertia,
  inverseInertiaQuadraticForm,
} from './inertia_tensor.js';

const EPSILON = 1e-9;

function ensureArrayMapEntry(map, key, value) {
  if (!map.has(key)) {
    map.set(key, []);
  }
  if (value !== undefined) {
    map.get(key).push(value);
  }
}

function applyAngularCorrection(orientationComp, deltaVec) {
  const angle = deltaVec.length();
  if (angle <= EPSILON) return;
  const axis = deltaVec.clone().scale(1.0 / angle);
  const dq = new Quaternion().setFromAxisAngle(axis, angle);
  orientationComp.quaternion.multiplyQuaternions(dq, orientationComp.quaternion).normalize();
}

function buildExternalMemberAngularSolveInfo(world, entityId, pointWorld, gradPos) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (!linkComp?.cablePlaneNormalLocal || !pointWorld || !gradPos) {
    return null;
  }
  const orientationComp = world.getComponent(entityId, OrientationComponent);
  const moiComp = world.getComponent(entityId, MomentOfInertiaComponent);
  const centerWorld = getEntityWorldPosition(world, entityId);
  if (!orientationComp?.quaternion || !centerWorld || !hasAnyInverseInertia(moiComp)) {
    return null;
  }
  return {
    entityId,
    centerWorld,
    momentOfInertia: moiComp,
    orientation: orientationComp.quaternion,
    pointWorld: pointWorld.clone(),
    gradPos: gradPos.clone(),
  };
}

function isHybridLinkType(value) {
  return value === 'hybrid' || value === 'hybrid-attachment';
}

function hasAxisOnlyCableSpinDof(world, entityId) {
  const spoolState = world.getComponent(entityId, SpoolStateComponent);
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  return Boolean(spoolState && linkComp?.cablePlaneNormalLocal);
}

function getExternalMemberAngularSolveInfo(
  world,
  path,
  jointIndex,
  side,
  originalEntityId,
  mappedEndpoint,
  pointWorld,
  gradPos
) {
  if (!mappedEndpoint || mappedEndpoint.internalToBody || mappedEndpoint.entityId === originalEntityId) {
    return null;
  }

  const directInfo = buildExternalMemberAngularSolveInfo(world, originalEntityId, pointWorld, gradPos);
  if (directInfo) {
    return directInfo;
  }

  if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.jointEntities) || path.jointEntities.length < 2) {
    return null;
  }

  const pinholeLinkIndex = side === 'A' ? jointIndex : jointIndex + 1;
  if (path.linkTypes[pinholeLinkIndex] !== 'pinhole') {
    return null;
  }

  let internalJointIndex = null;
  let hybridOnJointSide = null;
  if (side === 'A' && pinholeLinkIndex > 0 && isHybridLinkType(path.linkTypes[pinholeLinkIndex - 1])) {
    internalJointIndex = jointIndex - 1;
    hybridOnJointSide = 'A';
  } else if (
    side === 'B' &&
    pinholeLinkIndex < path.linkTypes.length - 1 &&
    isHybridLinkType(path.linkTypes[pinholeLinkIndex + 1])
  ) {
    internalJointIndex = jointIndex + 1;
    hybridOnJointSide = 'B';
  }
  if (!(internalJointIndex >= 0 && internalJointIndex < path.jointEntities.length)) {
    return null;
  }

  const internalJointId = path.jointEntities[internalJointIndex];
  const internalJoint = world.getComponent(internalJointId, CableJointComponent);
  if (!internalJoint) {
    return null;
  }
  const hybridEntityId = hybridOnJointSide === 'A' ? internalJoint.entityA : internalJoint.entityB;
  const hybridPointWorld = hybridOnJointSide === 'A'
    ? internalJoint.attachmentPointA_world
    : internalJoint.attachmentPointB_world;
  if (!hybridPointWorld || !pointWorld) {
    return null;
  }

  const coupledGradPos = pointWorld.clone().subtract(hybridPointWorld);
  if (coupledGradPos.lengthSq() <= EPSILON) {
    return null;
  }
  coupledGradPos.normalize();

  return buildExternalMemberAngularSolveInfo(world, hybridEntityId, hybridPointWorld, coupledGradPos);
}

export class PBDResolveCableOverCorrections {
  runInPause = false;

  update(world, _dt_unused) {
    const pathEntities = world.query([CablePathComponent]);
    if (!pathEntities) return;

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

    if (overCorrected.length < 2) return;

    const posCorrections = new Map();
    const angCorrections = new Map();

    for (const jointId of overCorrected) {
      this.calculateJointCorrection(world, jointId, jointToPathAndIndex, posCorrections, angCorrections);
    }

    for (const [entityId, deltas] of posCorrections.entries()) {
      if (deltas.length >= 2) {
        const avg = deltas.reduce((acc, v) => acc.add(v), new Vector3()).scale(1 / deltas.length);
        const posComp = world.getComponent(entityId, PositionComponent);
        if (posComp) {
          posComp.pos.add(avg);
        }
      }
    }

    for (const [entityId, deltas] of angCorrections.entries()) {
      if (deltas.length >= 2) {
        const avg = deltas.reduce((acc, v) => acc.add(v), new Vector3()).scale(1 / deltas.length);
        const orientComp = world.getComponent(entityId, OrientationComponent);
        if (orientComp) {
          applyAngularCorrection(orientComp, avg);
          updateRigidBodyMemberLocalOrientation(world, entityId);
        }
      }
    }
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
    if (constraintError >= -EPSILON) return;

    const entityA = joint.entityA;
    const entityB = joint.entityB;
    const mappedA = resolveRigidBodySolverEndpoint(world, entityA, entityB, pA);
    const mappedB = resolveRigidBodySolverEndpoint(world, entityB, entityA, pB);
    const solverEntityA = mappedA.entityId;
    const solverEntityB = mappedB.entityId;
    const solverPointA = computeWorldAttachment(world, solverEntityA, mappedA.localPoint);
    const solverPointB = computeWorldAttachment(world, solverEntityB, mappedB.localPoint);
    if (!solverPointA || !solverPointB) return;

    const massAComp = world.getComponent(solverEntityA, MassComponent);
    const invMassA = massAComp && massAComp.mass > 0 && Number.isFinite(massAComp.mass) ? 1.0 / massAComp.mass : 0.0;
    const moiAComp = world.getComponent(solverEntityA, MomentOfInertiaComponent);
    const orientationAComp = world.getComponent(solverEntityA, OrientationComponent);
    const useInertiaA = Boolean(moiAComp && orientationAComp?.quaternion && !hasAxisOnlyCableSpinDof(world, solverEntityA));
    // Apply reaction torque to the rigid body that owns the endpoint member,
    // regardless of whether the solver endpoint was promoted to the body.
    const reactionBodyEntityA = getRigidBodyEntityForMember(world, entityA);
    const reactionBodyInertiaA = reactionBodyEntityA !== null && reactionBodyEntityA !== undefined
      ? world.getComponent(reactionBodyEntityA, MomentOfInertiaComponent)
      : null;
    const reactionBodyOrientationA = reactionBodyEntityA !== null && reactionBodyEntityA !== undefined
      ? world.getComponent(reactionBodyEntityA, OrientationComponent)
      : null;
    const useReactionInertiaA = Boolean(reactionBodyInertiaA && reactionBodyOrientationA?.quaternion);

    const massBComp = world.getComponent(solverEntityB, MassComponent);
    const invMassB = massBComp && massBComp.mass > 0 && Number.isFinite(massBComp.mass) ? 1.0 / massBComp.mass : 0.0;
    const moiBComp = world.getComponent(solverEntityB, MomentOfInertiaComponent);
    const orientationBComp = world.getComponent(solverEntityB, OrientationComponent);
    const useInertiaB = Boolean(moiBComp && orientationBComp?.quaternion && !hasAxisOnlyCableSpinDof(world, solverEntityB));
    // Apply reaction torque to the rigid body that owns the endpoint member,
    // regardless of whether the solver endpoint was promoted to the body.
    const reactionBodyEntityB = getRigidBodyEntityForMember(world, entityB);
    const reactionBodyInertiaB = reactionBodyEntityB !== null && reactionBodyEntityB !== undefined
      ? world.getComponent(reactionBodyEntityB, MomentOfInertiaComponent)
      : null;
    const reactionBodyOrientationB = reactionBodyEntityB !== null && reactionBodyEntityB !== undefined
      ? world.getComponent(reactionBodyEntityB, OrientationComponent)
      : null;
    const useReactionInertiaB = Boolean(reactionBodyInertiaB && reactionBodyOrientationB?.quaternion);

    const diff = new Vector3().subtractVectors(pB, pA);
    const len = diff.length();
    if (len <= EPSILON) return;
    const dir = diff.clone().scale(1 / len);

    const gradPosA = dir.clone().scale(-1);
    const gradPosB = dir.clone();
    const memberAngularA = getExternalMemberAngularSolveInfo(
      world,
      path,
      i,
      'A',
      entityA,
      mappedA,
      pA,
      gradPosA,
    );
    const memberAngularB = getExternalMemberAngularSolveInfo(
      world,
      path,
      i,
      'B',
      entityB,
      mappedB,
      pB,
      gradPosB,
    );
    const posAComp = world.getComponent(solverEntityA, PositionComponent);
    const posBComp = world.getComponent(solverEntityB, PositionComponent);
    if (!posAComp?.pos || !posBComp?.pos) return;

    const rA = new Vector3().subtractVectors(solverPointA, posAComp.pos);
    const gradAngA = rA.cross(dir);
    const gradAngMemberA = memberAngularA
      ? new Vector3().subtractVectors(memberAngularA.pointWorld, memberAngularA.centerWorld).cross(memberAngularA.gradPos)
      : null;

    const rB = new Vector3().subtractVectors(solverPointB, posBComp.pos);
    const gradAngB = rB.cross(dir.clone().scale(-1));
    const gradAngMemberB = memberAngularB
      ? new Vector3().subtractVectors(memberAngularB.pointWorld, memberAngularB.centerWorld).cross(memberAngularB.gradPos)
      : null;

    const angularDenomA = useInertiaA
      ? inverseInertiaQuadraticForm(moiAComp, orientationAComp.quaternion, gradAngA)
      : 0.0;
    const angularDenomB = useInertiaB
      ? inverseInertiaQuadraticForm(moiBComp, orientationBComp.quaternion, gradAngB)
      : 0.0;
    const reactionAngularDenomA = useReactionInertiaA
      ? inverseInertiaQuadraticForm(reactionBodyInertiaA, reactionBodyOrientationA.quaternion, gradAngA)
      : 0.0;
    const reactionAngularDenomB = useReactionInertiaB
      ? inverseInertiaQuadraticForm(reactionBodyInertiaB, reactionBodyOrientationB.quaternion, gradAngB)
      : 0.0;
    const memberAngularDenomA = gradAngMemberA
      ? inverseInertiaQuadraticForm(memberAngularA?.momentOfInertia, memberAngularA?.orientation, gradAngMemberA)
      : 0.0;
    const memberAngularDenomB = gradAngMemberB
      ? inverseInertiaQuadraticForm(memberAngularB?.momentOfInertia, memberAngularB?.orientation, gradAngMemberB)
      : 0.0;

    let denom = 0;
    denom += invMassA * gradPosA.lengthSq();
    denom += angularDenomA;
    denom += reactionAngularDenomA;
    denom += memberAngularDenomA;
    denom += invMassB * gradPosB.lengthSq();
    denom += angularDenomB;
    denom += reactionAngularDenomB;
    denom += memberAngularDenomB;

    const dt = world.getResource('dt');
    if (dt !== undefined && dt > 0) {
      denom += path.compliance / (dt * dt);
    }

    if (denom <= EPSILON) return;

    const lambda = -constraintError / denom;

    if (invMassA > 0) {
      const delta = gradPosA.clone().scale(invMassA * lambda);
      ensureArrayMapEntry(posCorrections, solverEntityA, delta);
    }
    if (angularDenomA > 0.0) {
      const delta = applyWorldInverseInertia(moiAComp, orientationAComp.quaternion, gradAngA).scale(-lambda);
      ensureArrayMapEntry(angCorrections, solverEntityA, delta);
    }
    if (memberAngularDenomA > 0.0 && gradAngMemberA) {
      const delta = applyWorldInverseInertia(
        memberAngularA.momentOfInertia,
        memberAngularA.orientation,
        gradAngMemberA,
      ).scale(-lambda);
      ensureArrayMapEntry(angCorrections, memberAngularA.entityId, delta);
    }
    if (reactionAngularDenomA > 0.0 && reactionBodyEntityA !== null && reactionBodyEntityA !== undefined) {
      const delta = applyWorldInverseInertia(
        reactionBodyInertiaA,
        reactionBodyOrientationA.quaternion,
        gradAngA,
      ).scale(lambda);
      ensureArrayMapEntry(angCorrections, reactionBodyEntityA, delta);
    }
    if (invMassB > 0) {
      const delta = gradPosB.clone().scale(invMassB * lambda);
      ensureArrayMapEntry(posCorrections, solverEntityB, delta);
    }
    if (angularDenomB > 0.0) {
      const delta = applyWorldInverseInertia(moiBComp, orientationBComp.quaternion, gradAngB).scale(-lambda);
      ensureArrayMapEntry(angCorrections, solverEntityB, delta);
    }
    if (memberAngularDenomB > 0.0 && gradAngMemberB) {
      const delta = applyWorldInverseInertia(
        memberAngularB.momentOfInertia,
        memberAngularB.orientation,
        gradAngMemberB,
      ).scale(-lambda);
      ensureArrayMapEntry(angCorrections, memberAngularB.entityId, delta);
    }
    if (reactionAngularDenomB > 0.0 && reactionBodyEntityB !== null && reactionBodyEntityB !== undefined) {
      const delta = applyWorldInverseInertia(
        reactionBodyInertiaB,
        reactionBodyOrientationB.quaternion,
        gradAngB,
      ).scale(lambda);
      ensureArrayMapEntry(angCorrections, reactionBodyEntityB, delta);
    }
  }
}
