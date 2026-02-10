import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

import {
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  tangentFromSphereToSphere,
  signedArcLengthOnWheel,
  lineSegmentSphereIntersection,
  rightOfPlane
} from './geometry3.js';

import {
  PositionComponent,
  RadiusComponent,
  MassComponent,
  OrientationComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
  MachineTagComponent
} from './ecs.js';

export class CableLinkComponent {
  constructor(
    x = 0,
    y = 0,
    z = 0,
    orientation = null,
    planeNormal = null
  ) {
    this.prevCableAttachmentTimePos = new Vector3(x, y, z);
    this.prevCableAttachmentTimeOrientation = orientation ? orientation.clone() : new Quaternion();
    this.cablePlaneNormal = planeNormal ? planeNormal.clone() : new Vector3(0, 0, 1);
  }
}

// Represents a single segment constraint between two entities
export class CableJointComponent {
  constructor(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.restLength = restLength; // dn - the dynamic maximum length
    this.attachmentPointA_world = attachmentPointA_world.clone();
    this.attachmentPointB_world = attachmentPointB_world.clone();
  }

  static fromWorld(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    return new CableJointComponent(
      entityA,
      entityB,
      restLength,
      attachmentPointA_world,
      attachmentPointB_world
    );
  }

  static fromLocal(world, entityA, entityB, restLength, attachmentPointA_local, attachmentPointB_local) {
    return new CableJointComponent(
      entityA,
      entityB,
      restLength,
      _computeWorldAttachment(world, entityA, attachmentPointA_local),
      _computeWorldAttachment(world, entityB, attachmentPointB_local)
    );
  }
}

// Connects individual cable joints into a cable path
export class CablePathComponent {
  constructor(world, jointEntities = [], linkTypes = [], cw = [], spring_constant = 1e6, stored = null) {
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw; // Ordered. cw.length === linkTypes.length
    this.spring_constant = spring_constant;
    this.compliance = 1.0 / spring_constant;
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length

    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      this.totalRestLength += joint.restLength;
    }
    for (let i = 0; i < jointEntities.length - 1; i++) {
      const jointId_i = jointEntities[i];
      const jointId_i_plus_1 = jointEntities[i + 1];
      const joint_i = world.getComponent(jointId_i, CableJointComponent);
      const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
      const linkId = joint_i.entityB;
      const linkId2 = joint_i_plus_1.entityA;
      if (linkId !== linkId2) {
        console.warn("CablePathComponent constructor: Links don't match up. There's something wrong with this cable path.");
        return;
      }
      const isRolling = linkTypes[i + 1] === 'rolling';
      if (isRolling) {
        const center = world.getComponent(linkId, PositionComponent).pos;
        const radius = world.getComponent(linkId, RadiusComponent).radius;
        const isCw = cw[i + 1];

        const planeNormal = world.getComponent(linkId, CableLinkComponent)?.cablePlaneNormal;

        const initialStoredLength = signedArcLengthOnWheel(
          joint_i.attachmentPointB_world,
          joint_i_plus_1.attachmentPointA_world,
          center,
          radius,
          isCw,
          planeNormal,
          true
        );
        this.stored[i + 1] = initialStoredLength;
        this.totalRestLength += initialStoredLength;
      }
    }

    if (stored !== null) {
      for (let i = 0; i < this.stored.length; i++) {
        if (stored[i] !== null) {
          this.totalRestLength -= this.stored[i];
          this.totalRestLength += stored[i];
          this.stored[i] = stored[i];
        }
      }
    }
  }
}

export const linecolor1 = '#FFFF00';

const EPSILON = 1e-9;

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

function getMachineId(world, entityId) {
  if (entityId == null) {
    return '';
  }
  const tag = world.getComponent(entityId, MachineTagComponent);
  return tag ? tag.id : '';
}

function ensureMachineTag(world, entityId, machineId) {
  if (!entityId) {
    return;
  }
  const existing = world.getComponent(entityId, MachineTagComponent);
  if (existing) {
    return;
  }
  world.addComponent(entityId, new MachineTagComponent(machineId));
}

function _getPlaneNormal(world, entityId) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp && linkComp.cablePlaneNormal) {
    return linkComp.cablePlaneNormal;
  }
  return DEFAULT_PLANE_NORMAL;
}

function _computeWorldAttachment(world, entityId, localPoint) {
  if (!localPoint) {
    return null;
  }
  const localVec = localPoint.clone();
  if (!world) {
    return localVec;
  }

  const posComp = world.getComponent(entityId, PositionComponent);
  if (!posComp || !posComp.pos) {
    return localVec;
  }

  const orientationComp = world.getComponent(entityId, OrientationComponent);
  if (orientationComp && orientationComp.quaternion) {
    const rotated = orientationComp.quaternion.transformVector(localVec);
    return posComp.pos.clone().add(rotated);
  }

  return posComp.pos.clone().add(localVec);
}

function _computeLocalAttachment(world, entityId, worldPoint) {
  const posComp = world.getComponent(entityId, PositionComponent);
  if (!posComp) {
    return worldPoint.clone();
  }
  const rel = worldPoint.clone().subtract(posComp.pos);
  const orientationComp = world.getComponent(entityId, OrientationComponent);
  if (orientationComp && orientationComp.quaternion) {
    const inv = orientationComp.quaternion.clone().conjugate().normalize();
    return inv.transformVector(rel);
  }
  return rel;
}

function _deltaAngleAroundAxis(prevQuat, currQuat, axis) {
  if (!prevQuat || !currQuat) {
    return 0.0;
  }
  const invPrev = prevQuat.clone().conjugate().normalize();
  const qRel = new Quaternion().multiplyQuaternions(currQuat, invPrev).normalize();
  const w = Math.max(-1.0, Math.min(1.0, qRel.w));
  let angle = 2.0 * Math.acos(w);
  if (angle < EPSILON) {
    return 0.0;
  }
  const sinHalf = Math.sqrt(Math.max(0.0, 1.0 - w * w));
  let axisRel;
  if (sinHalf < EPSILON) {
    axisRel = axis.clone().normalize();
  } else {
    axisRel = new Vector3(qRel.x / sinHalf, qRel.y / sinHalf, qRel.z / sinHalf);
  }
  const sign = Math.sign(axisRel.dot(axis));
  if (sign === 0.0) {
    return angle;
  }
  if (angle > Math.PI) {
    angle -= 2.0 * Math.PI;
  }
  return angle * sign;
}

function _rotateAroundAxis(point, center, axis, angle, cw) {
  const axisNorm = axis.clone().normalize();
  const signedAngle = cw ? angle : -angle;
  return point.clone().subtract(center).rotateAroundAxis(axisNorm, signedAngle).add(center);
}

function _effectiveCW(path, linkIndex, travellingFromCircle) {
  if (linkIndex === 0 && travellingFromCircle) {
    return !path.cw[linkIndex];
  }
  return path.cw[linkIndex];
}

function _clearDebugPoints(world) {
  const debugPoints = world.getResource('debugRenderPoints');
  if (debugPoints) {
      for (const key in debugPoints) {
          delete debugPoints[key];
      }
  }
}

function _isAttachment(value) {
  return value === 'attachment' || value === 'hybrid-attachment' || value === 'pinhole';
}

function _isRolling(value) {
  return value === 'rolling' || value === 'hybrid';
}

function _isHybrid(value) {
  return value === 'hybrid' || value === 'hybrid-attachment';
}

export function calculateAttachmentPoints(world, joint, path, i) {
  const A = i;
  const B = i + 1;

  const entityA = joint.entityA;
  const entityB = joint.entityB;

  const posAComp = world.getComponent(entityA, PositionComponent);
  const radiusAComp = world.getComponent(entityA, RadiusComponent);
  const linkAComp = world.getComponent(entityA, CableLinkComponent);
  const orientationAComp = world.getComponent(entityA, OrientationComponent);

  const posA = posAComp?.pos;
  const attachmentA_previous = joint.attachmentPointA_world;
  const prevPosA = linkAComp?.prevCableAttachmentTimePos;
  const radiusA = radiusAComp?.radius;
  const planeNormalA = _getPlaneNormal(world, entityA);
  const angleA = orientationAComp?.quaternion;
  const prevAngleA = linkAComp?.prevCableAttachmentTimeOrientation;
  const deltaAngleA = _deltaAngleAroundAxis(prevAngleA, angleA, planeNormalA);

  const cwA = _effectiveCW(path, A, true);
  const attachmentLinkA = _isAttachment(path.linkTypes[A]);
  const rollingLinkA = _isRolling(path.linkTypes[A]);
  const isHybridA = _isHybrid(path.linkTypes[A]);

  const pADiffFromTranslation = (posA && prevPosA) ? posA.clone().subtract(prevPosA) : null;
  let pADiffFromRotation = new Vector3(0, 0, 0);
  if (prevPosA) {
    const rotated = attachmentA_previous.clone().subtract(prevPosA).rotateAroundAxis(planeNormalA, deltaAngleA).add(prevPosA);
    pADiffFromRotation = rotated.clone().subtract(attachmentA_previous);
  }

  const posBComp = world.getComponent(entityB, PositionComponent);
  const radiusBComp = world.getComponent(entityB, RadiusComponent);
  const linkBComp = world.getComponent(entityB, CableLinkComponent);
  const orientationBComp = world.getComponent(entityB, OrientationComponent);

  const posB = posBComp?.pos;
  const attachmentB_previous = joint.attachmentPointB_world;
  const prevPosB = linkBComp?.prevCableAttachmentTimePos;
  const radiusB = radiusBComp?.radius;
  const planeNormalB = _getPlaneNormal(world, entityB);
  const angleB = orientationBComp?.quaternion;
  const prevAngleB = linkBComp?.prevCableAttachmentTimeOrientation;
  const deltaAngleB = _deltaAngleAroundAxis(prevAngleB, angleB, planeNormalB);

  const cwB = _effectiveCW(path, B, false);
  const attachmentLinkB = _isAttachment(path.linkTypes[B]);
  const rollingLinkB = _isRolling(path.linkTypes[B]);
  const isHybridB = _isHybrid(path.linkTypes[B]);

  const pBDiffFromTranslation = (posB && prevPosB) ? posB.clone().subtract(prevPosB) : null;
  let pBDiffFromRotation = new Vector3(0, 0, 0);
  if (prevPosB) {
    const rotated = attachmentB_previous.clone().subtract(prevPosB).rotateAroundAxis(planeNormalB, deltaAngleB).add(prevPosB);
    pBDiffFromRotation = rotated.clone().subtract(attachmentB_previous);
  }

  let attachmentA_current = posA ? posA.clone() : null;
  let attachmentB_current = posB ? posB.clone() : null;

  if (attachmentLinkA && rollingLinkB) {
    if (isHybridA && pADiffFromTranslation) {
      attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
    }
    if (attachmentA_current && posB && radiusB !== undefined) {
      attachmentB_current = tangentFromPointToSphere(attachmentA_current, posB, radiusB, planeNormalB, cwB).a_sphere;
    }
  } else if (rollingLinkA && attachmentLinkB) {
    if (isHybridB && pBDiffFromTranslation) {
      attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
    }
    if (attachmentB_current && posA && radiusA !== undefined) {
      attachmentA_current = tangentFromSphereToPoint(attachmentB_current, posA, radiusA, planeNormalA, cwA).a_sphere;
    }
  } else if (rollingLinkA && rollingLinkB) {
    if (posA && posB && radiusA !== undefined && radiusB !== undefined) {
      const tangents = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormalA);
      attachmentA_current = tangents.a_sphere;
      attachmentB_current = tangents.b_sphere;
    }
  } else {
    if (isHybridA && pADiffFromTranslation) {
      attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
    }
    if (isHybridB && pBDiffFromTranslation) {
      attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
    }
  }

  return { attachmentA_current, attachmentB_current };
}

export function _updateAttachmentPoints(world) {
  const pathEntities = world.query([CablePathComponent]);

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);

    for (let i = 0; i < path.jointEntities.length; i++) {
      const jointId = path.jointEntities[i];
      const joint = world.getComponent(jointId, CableJointComponent);

      const attachmentA_previous = joint.attachmentPointA_world.clone();
      const attachmentB_previous = joint.attachmentPointB_world.clone();

      const { attachmentA_current, attachmentB_current } = calculateAttachmentPoints(world, joint, path, i);

      const A = i;
      const B = i + 1;
      const entityA = joint.entityA;
      const entityB = joint.entityB;

      const posAComp = world.getComponent(entityA, PositionComponent);
      const radiusAComp = world.getComponent(entityA, RadiusComponent);
      const linkAComp = world.getComponent(entityA, CableLinkComponent);
      const orientationAComp = world.getComponent(entityA, OrientationComponent);
      const posA = posAComp?.pos;
      const prevPosA = linkAComp?.prevCableAttachmentTimePos;
      const radiusA = radiusAComp?.radius;
      const planeNormalA = _getPlaneNormal(world, entityA);
      const angleA = orientationAComp?.quaternion;
      const prevAngleA = linkAComp?.prevCableAttachmentTimeOrientation;
      const deltaAngleA = _deltaAngleAroundAxis(prevAngleA, angleA, planeNormalA);
      const cwA = _effectiveCW(path, A, true);
      const rollingLinkA = _isRolling(path.linkTypes[A]);
      const isHybridA = _isHybrid(path.linkTypes[A]);
      const hasFrictionA = world.getComponent(entityA, CoefficientOfFrictionComponent);

      const posBComp = world.getComponent(entityB, PositionComponent);
      const radiusBComp = world.getComponent(entityB, RadiusComponent);
      const linkBComp = world.getComponent(entityB, CableLinkComponent);
      const orientationBComp = world.getComponent(entityB, OrientationComponent);
      const posB = posBComp?.pos;
      const prevPosB = linkBComp?.prevCableAttachmentTimePos;
      const radiusB = radiusBComp?.radius;
      const planeNormalB = _getPlaneNormal(world, entityB);
      const angleB = orientationBComp?.quaternion;
      const prevAngleB = linkBComp?.prevCableAttachmentTimeOrientation;
      const deltaAngleB = _deltaAngleAroundAxis(prevAngleB, angleB, planeNormalB);
      const cwB = _effectiveCW(path, B, false);
      const rollingLinkB = _isRolling(path.linkTypes[B]);
      const isHybridB = _isHybrid(path.linkTypes[B]);
      const hasFrictionB = world.getComponent(entityB, CoefficientOfFrictionComponent);

      let sA = 0;
      let sB = 0;

      if (rollingLinkA && attachmentA_previous && attachmentA_current && prevPosA && posA && radiusA !== undefined) {
          sA = signedArcLengthOnWheel(
            attachmentA_previous.clone().subtract(prevPosA),
            attachmentA_current.clone().subtract(posA),
            new Vector3(0.0, 0.0, 0.0),
            radiusA,
            cwA,
            planeNormalA
          );
          if (isHybridA || hasFrictionA) {
            sA += (cwA ? deltaAngleA * radiusA : -deltaAngleA * radiusA);
          }
      }

      if (rollingLinkB && attachmentB_previous && attachmentB_current && prevPosB && posB && radiusB !== undefined) {
          sB = signedArcLengthOnWheel(
            attachmentB_previous.clone().subtract(prevPosB),
            attachmentB_current.clone().subtract(posB),
            new Vector3(0.0, 0.0, 0.0),
            radiusB,
            cwB,
            planeNormalB
          );
          if (isHybridB || hasFrictionB) {
            sB += (cwB ? deltaAngleB * radiusB : -deltaAngleB * radiusB);
          }
      }

      path.stored[A] += sA;
      joint.restLength -= sA;
      path.stored[B] -= sB;
      joint.restLength += sB;

      if (attachmentA_current) {
        joint.attachmentPointA_world.set(attachmentA_current);
      }
      if (attachmentB_current) {
        joint.attachmentPointB_world.set(attachmentB_current);
      }
    }
  }
}

export function _mergeJoints(world) {
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (path.jointEntities.length < 2) continue;
    let reRunMerge = true;
    while (reRunMerge) {
      reRunMerge = false;
      for (let i = 0; i < path.jointEntities.length - 1; i++) {
        if (path.linkTypes[i + 1] !== 'rolling') {
          continue;
        }
        const jointId_i = path.jointEntities[i];
        const jointId_i_plus_1 = path.jointEntities[i + 1];
        const joint_i = world.getComponent(jointId_i, CableJointComponent);
        const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
        const linkId = joint_i.entityB;
        const linkId2 = joint_i_plus_1.entityA;
        if (linkId !== linkId2) {
          console.warn("Merge loop saw disconnected cable path");
          continue;
        }
        if (joint_i.entityA === joint_i_plus_1.entityB) {
          continue;
        }
        if (path.stored[i + 1] < 0.0) {
          const pA1 = joint_i.attachmentPointA_world;
          const pB2 = joint_i_plus_1.attachmentPointB_world;
          const posA = world.getComponent(joint_i.entityA, PositionComponent).pos;
          const radiusA = world.getComponent(joint_i.entityA, RadiusComponent)?.radius;
          const planeNormalA = _getPlaneNormal(world, joint_i.entityA);
          const cwA = _effectiveCW(path, i, true);
          const posB = world.getComponent(joint_i_plus_1.entityB, PositionComponent).pos;
          const radiusB = world.getComponent(joint_i_plus_1.entityB, RadiusComponent)?.radius;
          const planeNormalB = _getPlaneNormal(world, joint_i_plus_1.entityB);
          const cwB = path.cw[i + 2];

          joint_i.restLength += joint_i_plus_1.restLength + path.stored[i + 1];
          joint_i.entityB = joint_i_plus_1.entityB;
          const isAttachmentA = _isAttachment(path.linkTypes[i]);
          const isRollingA = _isRolling(path.linkTypes[i]);
          const isAttachmentB = _isAttachment(path.linkTypes[i + 2]);
          const isRollingB = _isRolling(path.linkTypes[i + 2]);

          let attachmentA_current = pA1;
          let attachmentB_current = pB2;
          if (isRollingA && isRollingB) {
            const tangents = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormalA);
            attachmentA_current = tangents.a_sphere;
            attachmentB_current = tangents.b_sphere;
          } else if (isRollingA && isAttachmentB) {
            attachmentA_current = tangentFromSphereToPoint(pB2, posA, radiusA, planeNormalA, cwA).a_sphere;
          } else if (isAttachmentA && isRollingB) {
            attachmentB_current = tangentFromPointToSphere(pA1, posB, radiusB, planeNormalB, cwB).a_sphere;
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA && isRollingB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA, planeNormalA);
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB, planeNormalB);
          } else if (isRollingA && isAttachmentB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA, planeNormalA);
          } else if (isAttachmentA && isRollingB) {
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB, planeNormalB);
          }

          path.stored[i] += sA;
          joint_i.restLength -= sA;
          path.stored[i + 2] -= sB;
          joint_i.restLength += sB;
          reRunMerge = path.stored[i] < 0.0 || path.stored[i + 2] < 0.0;

          joint_i.attachmentPointA_world.set(attachmentA_current);
          joint_i.attachmentPointB_world.set(attachmentB_current);

          path.jointEntities.splice(i + 1, 1);
          path.stored.splice(i + 1, 1);
          path.cw.splice(i + 1, 1);
          path.linkTypes.splice(i + 1, 1);
          world.destroyEntity(jointId_i_plus_1);
        }
      }
    }
  }
}

export function _splitJoints(world) {
  const potentialSplitters = world.query([PositionComponent, RadiusComponent, CableLinkComponent]);
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    const pathMachine = getMachineId(world, pathId);
    if (path.jointEntities.length < 1) continue;
    for (let i = 0; i < path.jointEntities.length; i++) {
      const jointId = path.jointEntities[i];
      const joint = world.getComponent(jointId, CableJointComponent);

      const pA = joint.attachmentPointA_world;
      const pB = joint.attachmentPointB_world;
      for (const splitterId of potentialSplitters) {
        if (splitterId === joint.entityA || splitterId === joint.entityB) {
          continue;
        }
        if (getMachineId(world, splitterId) !== pathMachine) {
          continue;
        }
        const posSplitter = world.getComponent(splitterId, PositionComponent).pos;
        const radiusSplitter = world.getComponent(splitterId, RadiusComponent)?.radius;
        if (lineSegmentSphereIntersection(pA, pB, posSplitter, radiusSplitter)) {
          const entityA = joint.entityA;
          const entityB = joint.entityB;
          const newJointId = world.createEntity();
          ensureMachineTag(world, newJointId, pathMachine);

          const posA = world.getComponent(entityA, PositionComponent).pos;
          const linkTypeA = path.linkTypes[i];
          const isAttachmentA = _isAttachment(linkTypeA);
          const isRollingA = _isRolling(linkTypeA);
          const radiusA = world.getComponent(entityA, RadiusComponent)?.radius;
          const planeNormalA = _getPlaneNormal(world, entityA);
          const cwA = _effectiveCW(path, i, true);

          const posB = world.getComponent(entityB, PositionComponent).pos;
          const linkTypeB = path.linkTypes[i + 1];
          const isAttachmentB = _isAttachment(linkTypeB);
          const isRollingB = _isRolling(linkTypeB);
          const radiusB = world.getComponent(entityB, RadiusComponent)?.radius;
          const planeNormalB = _getPlaneNormal(world, entityB);
          const cwB = path.cw[i + 1];

          const prevPosSplitter = world.getComponent(splitterId, CableLinkComponent).prevCableAttachmentTimePos;
          const planeNormalSplitter = _getPlaneNormal(world, splitterId);
          const cw = rightOfPlane(prevPosSplitter, pA, pB, planeNormalSplitter);

          let newAttachmentPointAForJoint = null;
          let newAttachmentPointBForJoint = null;
          if (isRollingA) {
            const tangentAS = tangentFromSphereToSphere(posA, radiusA, cwA, posSplitter, radiusSplitter, cw, planeNormalA);
            newAttachmentPointAForJoint = tangentAS.a_sphere;
            newAttachmentPointBForJoint = tangentAS.b_sphere;
          } else if (isAttachmentA) {
            const tangentAS = tangentFromPointToSphere(pA, posSplitter, radiusSplitter, planeNormalSplitter, cw);
            newAttachmentPointAForJoint = tangentAS.a_attach;
            newAttachmentPointBForJoint = tangentAS.a_sphere;
          } else {
            console.warn(`Splitting cable joint coming from link type ${linkTypeA} is not supported.`);
            continue;
          }

          let attachmentPointAForNewJoint = null;
          let attachmentPointBForNewJoint = null;
          if (isRollingB) {
            const tangentSB = tangentFromSphereToSphere(posSplitter, radiusSplitter, cw, posB, radiusB, cwB, planeNormalB);
            attachmentPointAForNewJoint = tangentSB.a_sphere;
            attachmentPointBForNewJoint = tangentSB.b_sphere;
          } else if (isAttachmentB) {
            const tangentSB = tangentFromSphereToPoint(pB, posSplitter, radiusSplitter, planeNormalSplitter, cw);
            attachmentPointAForNewJoint = tangentSB.a_sphere;
            attachmentPointBForNewJoint = tangentSB.a_attach;
          } else {
            console.warn(`Splitting cable joint attached to ${linkTypeB} is not supported.`);
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA) {
            sA = signedArcLengthOnWheel(pA, newAttachmentPointAForJoint, posA, radiusA, cwA, planeNormalA);
          }
          if (isRollingB) {
            sB = signedArcLengthOnWheel(pB, attachmentPointBForNewJoint, posB, radiusB, cwB, planeNormalB);
          }

          const s = signedArcLengthOnWheel(
              newAttachmentPointBForJoint,
              attachmentPointAForNewJoint,
              posSplitter,
              radiusSplitter,
              cw,
              planeNormalSplitter
          );
          if (s <= 0.0) {
              console.warn(`Nothing wraps around splitter. Aborting split.`);
              continue;
          }
          if ((s + 1e-9) >= 2.0*Math.PI * radiusSplitter) {
              console.warn(`Split resulted a full wrap around splitter. Aborting split.`);
              continue;
          }

          const dAS = newAttachmentPointAForJoint.clone().subtract(newAttachmentPointBForJoint).length();
          const dSB = attachmentPointAForNewJoint.clone().subtract(attachmentPointBForNewJoint).length();
          const originalRestLength = joint.restLength + sB - sA;
          const totalDist = dAS + dSB;

          let newRestLengthAS = 0;
          let newRestLengthSB = 0;
          if (totalDist > EPSILON) {
              const availableRestLength = originalRestLength - s;
              if (availableRestLength < EPSILON) {
                  console.warn(`Split resulted in < ${EPSILON.toFixed(4)} available rest length (${availableRestLength.toFixed(4)}). Aborting split.`);
                  continue;
              }
              newRestLengthAS = availableRestLength * dAS / totalDist;
              newRestLengthSB = availableRestLength * dSB / totalDist;
          } else {
              console.warn("Split occurred with near-zero distance between new segments:", totalDist);
          }
          path.stored[i + 1] -= sB;
          joint.restLength += sB;
          path.jointEntities.splice(i + 1, 0, newJointId);
          path.cw.splice(i + 1, 0, cw);
          path.linkTypes.splice(i + 1, 0, 'rolling');
          joint.attachmentPointA_world.set(newAttachmentPointAForJoint);
          path.stored[i] += sA;
          joint.restLength -= sA;
          joint.entityB = splitterId;
          path.stored.splice(i + 1, 0, s);

          joint.restLength = newRestLengthAS;
          joint.attachmentPointB_world.set(newAttachmentPointBForJoint);

          world.addComponent(newJointId, new CableJointComponent(
              splitterId, entityB, newRestLengthSB, attachmentPointAForNewJoint, attachmentPointBForNewJoint));
        }
      }
    }
  }
}

export function _updateHybridLinkStates(world) {
  const pathEntities = world.query([CablePathComponent]);

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    for (const i of [0, path.linkTypes.length - 1]) {
      if (path.linkTypes[i] === 'hybrid') {
        if (path.stored[i] < 0.0) {
          path.linkTypes[i] = 'hybrid-attachment';
          const joint = (i === 0 ? world.getComponent(path.jointEntities[i], CableJointComponent) : world.getComponent(path.jointEntities[i - 1], CableJointComponent));
          const linkEntity = (i === 0 ? joint.entityA : joint.entityB);
          const radius = world.getComponent(linkEntity, RadiusComponent).radius;
          const pos = world.getComponent(linkEntity, PositionComponent).pos;
          const planeNormal = _getPlaneNormal(world, linkEntity);
          joint.restLength += path.stored[i];
          const rotAng = -path.stored[i] / radius;
          if (i === 0) {
            const rotated = _rotateAroundAxis(joint.attachmentPointA_world, pos, planeNormal, rotAng, path.cw[i]);
            joint.attachmentPointA_world.set(rotated);
          } else if (i === path.linkTypes.length - 1) {
            const rotated = _rotateAroundAxis(joint.attachmentPointB_world, pos, planeNormal, rotAng, path.cw[i]);
            joint.attachmentPointB_world.set(rotated);
          }
          path.stored[i] = 0;
        }
      }
      else if (path.linkTypes[i] === 'hybrid-attachment') {
        let jointId, joint, entityId, attachmentPoint, neighborId, neighborAttachmentPoint;
        if (i === 0) {
          jointId = path.jointEntities[0];
          joint   = world.getComponent(jointId, CableJointComponent);
          entityId        = joint.entityA;
          attachmentPoint = joint.attachmentPointA_world;
          neighborId      = joint.entityB;
          neighborAttachmentPoint = joint.attachmentPointB_world;
        }
        else if (i === path.linkTypes.length - 1) {
          jointId = path.jointEntities[path.jointEntities.length - 1];
          joint   = world.getComponent(jointId, CableJointComponent);
          entityId        = joint.entityB;
          attachmentPoint = joint.attachmentPointB_world;
          neighborId      = joint.entityA;
          neighborAttachmentPoint = joint.attachmentPointA_world;
        }

        const C = world.getComponent(entityId, PositionComponent).pos;
        const P = world.getComponent(neighborId, PositionComponent).pos;
        const R = world.getComponent(entityId, RadiusComponent).radius;
        const planeNormal = _getPlaneNormal(world, entityId);

        const tanCW  = tangentFromSphereToPoint(neighborAttachmentPoint, C, R, planeNormal, true).a_sphere;
        const tanCCW = tangentFromSphereToPoint(neighborAttachmentPoint, C, R, planeNormal, false).a_sphere;

        const crossedCW  = signedArcLengthOnWheel(attachmentPoint, tanCW,  C, R, true, planeNormal);
        const crossedCCW  = signedArcLengthOnWheel(attachmentPoint, tanCCW,  C, R, false, planeNormal);
        const distSqCW = attachmentPoint.distanceToSq(tanCW);
        const distSqCCW = attachmentPoint.distanceToSq(tanCCW);

        let newCW = null, crossingTangent = null;
        if (crossedCCW > 0.0 && distSqCCW < distSqCW) {
            newCW = true;
            crossingTangent = tanCCW;
            path.stored[i] = crossedCCW;
            joint.restLength -= crossedCCW;
        } else if (crossedCW > 0.0 && distSqCW < distSqCCW) {
            newCW = false;
            crossingTangent = tanCW;
            path.stored[i] = crossedCW;
            joint.restLength -= crossedCW;
        }

        if (newCW !== null) {
          path.linkTypes[i] = 'hybrid';
          path.cw[i]        = newCW;
          attachmentPoint.set(crossingTangent);
        }
      }
    }
  }
}

export class CableAttachmentUpdateSystem {
  runInPause = false;

  update(world, dt) {
    _clearDebugPoints(world);
    _updateAttachmentPoints(world);
    _mergeJoints(world);
    _splitJoints(world);
    _updateHybridLinkStates(world);
  }
}

export class PBDCableConstraintSolver {
  runInPause = false;

  update(world, _dt_unused) {
    const pathEntities = world.query([CablePathComponent]);
    const dt = world.getResource('dt');

    const ITERATIONS = 2;

    const jointLocals = new Map();

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        jointLocals.set(jointId, {
          localA: _computeLocalAttachment(world, joint.entityA, joint.attachmentPointA_world),
          localB: _computeLocalAttachment(world, joint.entityB, joint.attachmentPointB_world)
        });
      }
    }

    for (let iter = 0; iter < ITERATIONS; iter++) {
      const isForward = (iter % 2 === 0);

      const startPath = isForward ? 0 : pathEntities.length - 1;
      const endPath = isForward ? pathEntities.length : -1;
      const stepPath = isForward ? 1 : -1;

      for (let p = startPath; p !== endPath; p += stepPath) {
        const pathId = pathEntities[p];
        const path = world.getComponent(pathId, CablePathComponent);
        if (path.jointEntities.length < 1) continue;

        const startJoint = isForward ? 0 : path.jointEntities.length - 1;
        const endJoint = isForward ? path.jointEntities.length : -1;
        const stepJoint = isForward ? 1 : -1;

        for (let j = startJoint; j !== endJoint; j += stepJoint) {
          const jointId = path.jointEntities[j];
          const joint = world.getComponent(jointId, CableJointComponent);
          const locals = jointLocals.get(jointId);

          const entityA = joint.entityA;
          const entityB = joint.entityB;

          const pA = _computeWorldAttachment(world, entityA, locals.localA);
          const pB = _computeWorldAttachment(world, entityB, locals.localB);

          const currentSegmentLength = pA.distanceTo(pB);
          const constraintError = currentSegmentLength - joint.restLength;
          if (constraintError > EPSILON) {
            const massAComp = world.getComponent(entityA, MassComponent);
            const invMassA = (massAComp && massAComp.mass > 0) ? 1.0 / massAComp.mass : 0.0;
            const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
            // Scalar inertia here is treated as effective inertia about the constraint axis.
            const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;

            const massBComp = world.getComponent(entityB, MassComponent);
            const invMassB = (massBComp && massBComp.mass > 0) ? 1.0 / massBComp.mass : 0.0;
            const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
            // Scalar inertia here is treated as effective inertia about the constraint axis.
            const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

            if (invMassA + invMassB + invInertiaA + invInertiaB <= EPSILON) {
              continue;
            }

            const diff = new Vector3().subtractVectors(pB, pA);
            const len = diff.length();
            if (len <= EPSILON) continue;
            const dir = diff.clone().scale(1.0 / len);

            const gradPosA = dir.clone();
            const gradPosB = dir.clone().scale(-1.0);

            const posAComp = world.getComponent(entityA, PositionComponent);
            const rA = new Vector3().subtractVectors(pA, posAComp.pos);
            const gradAngA = rA.cross(dir);

            const posBComp = world.getComponent(entityB, PositionComponent);
            const rB = new Vector3().subtractVectors(pB, posBComp.pos);
            const gradAngB = rB.cross(dir.clone().scale(-1.0));

            let denom = 0.0;
            denom += invMassA * gradPosA.lengthSq();
            denom += invInertiaA * gradAngA.lengthSq();
            denom += invMassB * gradPosB.lengthSq();
            denom += invInertiaB * gradAngB.lengthSq();
            if (dt !== undefined && dt > EPSILON) {
              denom += path.compliance / (dt * dt);
            }

            if (denom <= EPSILON) {
              continue;
            }

            const lambda = -constraintError / denom;

            if (invMassA > 0.0) {
              const deltaPosA = gradPosA.clone().scale(-invMassA * lambda);
              posAComp.pos.add(deltaPosA);
            }
            if (invInertiaA > 0.0) {
              const deltaAngA = gradAngA.clone().scale(-invInertiaA * lambda);
              const orientationAComp = world.getComponent(entityA, OrientationComponent);
              if (orientationAComp) {
                const angle = deltaAngA.length();
                if (angle > EPSILON) {
                  const axis = deltaAngA.clone().scale(1.0 / angle);
                  const dq = new Quaternion().setFromAxisAngle(axis, angle);
                  orientationAComp.quaternion.multiplyQuaternions(dq, orientationAComp.quaternion).normalize();
                }
              }
            }

            if (invMassB > 0.0) {
              const deltaPosB = gradPosB.clone().scale(-invMassB * lambda);
              posBComp.pos.add(deltaPosB);
            }
            if (invInertiaB > 0.0) {
              const deltaAngB = gradAngB.clone().scale(-invInertiaB * lambda);
              const orientationBComp = world.getComponent(entityB, OrientationComponent);
              if (orientationBComp) {
                const angle = deltaAngB.length();
                if (angle > EPSILON) {
                  const axis = deltaAngB.clone().scale(1.0 / angle);
                  const dq = new Quaternion().setFromAxisAngle(axis, angle);
                  orientationBComp.quaternion.multiplyQuaternions(dq, orientationBComp.quaternion).normalize();
                }
              }
            }
          }
        }
      }
    }
  }
}
