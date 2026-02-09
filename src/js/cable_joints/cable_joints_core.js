import Vector2 from './vector2.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel,
  lineSegmentCircleIntersection,
  rightOfLine,
} from './geometry.js';

import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
  RenderableComponent,
  MachineTagComponent
} from './ecs.js';

import {
  GravitySystem,
  MovementSystem,
} from './commonSystems.js';

export const linecolor1 = '#FFFF00';
const EPSILON = 1e-9;
const PINCH_NON_TRANSITIONAL_STORED_BUFFER = 1e-5;
const PINCH_CANDIDATES_RESOURCE = 'cablePinchCandidates';
const PINCH_CONFIGS_RESOURCE = 'cablePinchJointConfigs';
const PINCH_CONTACTS_RESOURCE = 'cablePinchContacts';
const CABLE_DEBUG_PREFIX = '[CableJointsDebug]';

function _debugCable(world, message) {
  if (world?.getResource('cableDebugLogs') === true) {
    console.debug(`${CABLE_DEBUG_PREFIX} ${message}`);
  }
}

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
  const angle = orientationComp?.angle ?? 0.0;
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);

  const rotatedX = localVec.x * cos - localVec.y * sin;
  const rotatedY = localVec.x * sin + localVec.y * cos;

  return new Vector2(posComp.pos.x + rotatedX, posComp.pos.y + rotatedY);
}

export class CableLinkComponent {
  constructor(x = 0, y = 0, angle = 0.0) {
    this.prevCableAttachmentTimePos = new Vector2(x, y);
    this.prevCableAttachmentTimeAngle = angle;
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
    return new CableJointComponent(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world);
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
  constructor(
    world,
    jointEntities = [],
    linkTypes = [],
    cw = [],
    spring_constant = 1e6,
    stored = null,
    cableHalfWidth = 0.0
  ) {
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw // Ordered. cw.length === linkTypes.length
    this.spring_constant = spring_constant;
    this.compliance = 1.0/spring_constant;
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length
    this.cableHalfWidth = Number.isFinite(cableHalfWidth) ? Math.max(0.0, cableHalfWidth) : 0.0;

    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      this.totalRestLength += joint.restLength;
    }
    for (let i = 0; i < jointEntities.length - 1; i++) { // Iterate over adjacent pairs
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
      // Assuming standard path structure A->B, A->B, check B_i == A_i+1
      if (isRolling) {
        const center = world.getComponent(linkId, PositionComponent).pos;
        const radius = world.getComponent(linkId, RadiusComponent).radius + this.cableHalfWidth;
        const isCw = cw[i + 1];

        const initialStoredLength = signedArcLengthOnWheel(
            joint_i.attachmentPointB_world,
            joint_i_plus_1.attachmentPointA_world,
            center,
            radius,
            isCw,
            true
        );
        this.stored[i + 1] = initialStoredLength;
        this.totalRestLength += initialStoredLength;
      }
    }

    // If amount of stored line is supplied directly, use those values instead of the calculated ones
    if (stored !== null) {
      for (let i = 0; i < this.stored.length; i++) {
        // Allow the user to skip some values by sending null
        // eg [1.0, null, null, 5.0] only sets stored at endpoints.
        if (stored[i] !== null) {
          this.totalRestLength -= this.stored[i];
          this.totalRestLength += stored[i];
          this.stored[i] = stored[i];
        }
      }
    }
  }
}

export class PinchContact {
  constructor(entityA, entityB, pathId, minDistance, normal, compliance = 0.0) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.pathId = pathId;
    this.minDistance = minDistance;
    this.normal = normal.clone();
    this.compliance = compliance;
  }
}


function _effectiveCW(path, linkIndex, travellingFromCircle) {
  if (linkIndex === 0 && travellingFromCircle)
    return !path.cw[linkIndex];
  return path.cw[linkIndex];
}

function _effectiveRadius(path, radius) {
  if (radius === undefined || radius === null) {
    return radius;
  }
  return radius + (path?.cableHalfWidth ?? 0.0);
}

function _effectiveRollingRadius(path, linkIndex, baseRadius) {
  if (!Number.isFinite(baseRadius) || baseRadius <= EPSILON) {
    return baseRadius;
  }
  if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.stored)) {
    return baseRadius;
  }
  if (!_isHybrid(path.linkTypes[linkIndex])) {
    return baseRadius;
  }
  if (!(linkIndex === 0 || linkIndex === path.linkTypes.length - 1)) {
    return baseRadius;
  }

  const halfWidth = path.cableHalfWidth ?? 0.0;
  if (!(halfWidth > EPSILON)) {
    return baseRadius;
  }

  const stored = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
  if (!(stored > EPSILON)) {
    return baseRadius;
  }

  const decomposition = _decomposeStoredWrapLayers(stored, baseRadius, halfWidth);
  if (!decomposition) {
    return baseRadius;
  }
  if (decomposition.hasPartial) {
    return decomposition.partialRadius;
  }
  return baseRadius + 2.0 * halfWidth * Math.max(0, decomposition.fullLayers - 1);
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

  // Get components for Entity A
  const posAComp = world.getComponent(entityA, PositionComponent);
  const radiusAComp = world.getComponent(entityA, RadiusComponent);
  const linkAComp = world.getComponent(entityA, CableLinkComponent);
  const orientationAComp = world.getComponent(entityA, OrientationComponent);

  const posA = posAComp?.pos;
  const attachmentA_previous = joint.attachmentPointA_world;
  const prevPosA = linkAComp?.prevCableAttachmentTimePos;
  const baseRadiusA = _effectiveRadius(path, radiusAComp?.radius);
  const radiusA = _effectiveRollingRadius(path, A, baseRadiusA);
  const angleA = orientationAComp?.angle ?? 0.0;
  const prevAngleA = linkAComp?.prevCableAttachmentTimeAngle ?? 0.0;
  const deltaAngleA = angleA - prevAngleA;

  const cwA = _effectiveCW(path, A, true);
  const attachmentLinkA = _isAttachment(path.linkTypes[A]);
  const rollingLinkA = _isRolling(path.linkTypes[A]);
  const isHybridA = _isHybrid(path.linkTypes[A]);

  const pADiffFromTranslation = (posA && prevPosA) ? posA.clone().subtract(prevPosA) : null;
  const tempRotatedA = attachmentA_previous.clone();
  if (prevPosA) {
    tempRotatedA.rotate(deltaAngleA, prevPosA, true);
  }
  const pADiffFromRotation = tempRotatedA.clone().subtract(attachmentA_previous);

  // Get components for Entity B
  const posBComp = world.getComponent(entityB, PositionComponent);
  const radiusBComp = world.getComponent(entityB, RadiusComponent);
  const linkBComp = world.getComponent(entityB, CableLinkComponent);
  const orientationBComp = world.getComponent(entityB, OrientationComponent);

  const posB = posBComp?.pos;
  const attachmentB_previous = joint.attachmentPointB_world;
  const prevPosB = linkBComp?.prevCableAttachmentTimePos;
  const baseRadiusB = _effectiveRadius(path, radiusBComp?.radius);
  const radiusB = _effectiveRollingRadius(path, B, baseRadiusB);
  const angleB = orientationBComp?.angle ?? 0.0;
  const prevAngleB = linkBComp?.prevCableAttachmentTimeAngle ?? 0.0;
  const deltaAngleB = angleB - prevAngleB;

  const cwB = _effectiveCW(path, B, false);
  const attachmentLinkB = _isAttachment(path.linkTypes[B]);
  const rollingLinkB = _isRolling(path.linkTypes[B]);
  const isHybridB = _isHybrid(path.linkTypes[B]);

  const pBDiffFromTranslation = (posB && prevPosB) ? posB.clone().subtract(prevPosB) : null;
  const tempRotatedB = attachmentB_previous.clone();
  if (prevPosB) {
    tempRotatedB.rotate(deltaAngleB, prevPosB, true);
  }
  const pBDiffFromRotation = tempRotatedB.clone().subtract(attachmentB_previous);

  let attachmentA_current = posA ? posA.clone() : null;
  let attachmentB_current = posB ? posB.clone() : null;

  if (attachmentLinkA && rollingLinkB) {
    if (isHybridA && pADiffFromTranslation) {
      attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
    }
    if (attachmentA_current && posB && radiusB !== undefined) {
      attachmentB_current = tangentFromPointToCircle(attachmentA_current, posB, radiusB, cwB).a_circle;
    }
  } else if (rollingLinkA && attachmentLinkB) {
    if (isHybridB && pBDiffFromTranslation) {
      attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
    }
    if (attachmentB_current && posA && radiusA !== undefined) {
      attachmentA_current = tangentFromCircleToPoint(attachmentB_current, posA, radiusA, cwA).a_circle;
    }
  } else if (rollingLinkA && rollingLinkB) {
    if (posA && posB && radiusA !== undefined && radiusB !== undefined) {
      const tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
      attachmentA_current = tangents.a_circle;
      attachmentB_current = tangents.b_circle;
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
  const pinchConfigsResource = world.getResource(PINCH_CONFIGS_RESOURCE);
  const pinchConfigs = pinchConfigsResource instanceof Map ? pinchConfigsResource : new Map();

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);

    for (let i = 0; i < path.jointEntities.length; i++) {
      const jointId = path.jointEntities[i];
      const joint = world.getComponent(jointId, CableJointComponent);

      const attachmentA_previous = joint.attachmentPointA_world.clone();
      const attachmentB_previous = joint.attachmentPointB_world.clone();

      let { attachmentA_current, attachmentB_current } = calculateAttachmentPoints(world, joint, path, i);

      const A = i;
      const B = i + 1;
      const entityA = joint.entityA;
      const entityB = joint.entityB;

      const pinchConfig = pinchConfigs.get(jointId);
      if (pinchConfig) {
        const surfacePair = _computeCircleSurfacePair(world, entityA, entityB, pinchConfig.normal);
        const pinchPair = _computePinchAttachmentPair(world, path, entityA, entityB, pinchConfig.normal);
        if (
          surfacePair &&
          pinchPair &&
          surfacePair.surfaceDistance <= pinchConfig.minDistance + EPSILON
        ) {
          attachmentA_current = pinchPair.pointA_world;
          attachmentB_current = pinchPair.pointB_world;
        }
      }

      // Get components for Entity A
      const posAComp = world.getComponent(entityA, PositionComponent);
      const radiusAComp = world.getComponent(entityA, RadiusComponent);
      const linkAComp = world.getComponent(entityA, CableLinkComponent);
      const orientationAComp = world.getComponent(entityA, OrientationComponent);
      const posA = posAComp?.pos;
      const prevPosA = linkAComp?.prevCableAttachmentTimePos;
      const baseRadiusA = _effectiveRadius(path, radiusAComp?.radius);
      const radiusA = _effectiveRollingRadius(path, A, baseRadiusA);
      const angleA = orientationAComp?.angle ?? 0.0;
      const prevAngleA = linkAComp?.prevCableAttachmentTimeAngle ?? 0.0;
      const deltaAngleA = angleA - prevAngleA;
      const cwA = _effectiveCW(path, A, true);
      const rollingLinkA = _isRolling(path.linkTypes[A]);
      const isHybridA = _isHybrid(path.linkTypes[A]);
      const hasFrictionA = world.getComponent(entityA, CoefficientOfFrictionComponent);

      // Get components for Entity B
      const posBComp = world.getComponent(entityB, PositionComponent);
      const radiusBComp = world.getComponent(entityB, RadiusComponent);
      const linkBComp = world.getComponent(entityB, CableLinkComponent);
      const orientationBComp = world.getComponent(entityB, OrientationComponent);
      const posB = posBComp?.pos;
      const prevPosB = linkBComp?.prevCableAttachmentTimePos;
      const baseRadiusB = _effectiveRadius(path, radiusBComp?.radius);
      const radiusB = _effectiveRollingRadius(path, B, baseRadiusB);
      const angleB = orientationBComp?.angle ?? 0.0;
      const prevAngleB = linkBComp?.prevCableAttachmentTimeAngle ?? 0.0;
      const deltaAngleB = angleB - prevAngleB;
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
            new Vector2(0.0, 0.0),
            radiusA,
            cwA
          );
          if (isHybridA || hasFrictionA) {
            sA += (cwA ? deltaAngleA * radiusA : -deltaAngleA * radiusA);
          }
      }

      if (rollingLinkB && attachmentB_previous && attachmentB_current && prevPosB && posB && radiusB !== undefined) {
          sB = signedArcLengthOnWheel(
            attachmentB_previous.clone().subtract(prevPosB),
            attachmentB_current.clone().subtract(posB),
            new Vector2(0.0, 0.0),
            radiusB,
            cwB
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
    const jointsInPath = path.jointEntities;
    // A merge operation in itself might move attachment points in such a way
    // that an additional merge operation is required, or a single joint might
    // require a merge of both of its ends at the same time step. We re run merge
    // until all such cases are resolved for a given time step.
    let reRunMerge = true;
    while (reRunMerge) {
      reRunMerge = false;
      for (let i = 0; i < path.jointEntities.length - 1; i++) { // Iterate over adjacent pairs
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
        if (path.stored[i + 1] < 0.0) {
          if (joint_i.entityA === joint_i_plus_1.entityB) {
            // Non-transitional pinch removal case:
            // ... -> A->B, B->A -> ...
            // Remove the two transitional joints and merge their length budget back
            // into the remaining wrapped link on body A.
            if (i <= 0 || (i + 2) >= path.stored.length) {
              console.warn("Pinch merge loop saw non-transitional pair without both neighbors.");
              continue;
            }

            const removedBudget =
              joint_i.restLength +
              joint_i_plus_1.restLength +
              path.stored[i + 1] +
              path.stored[i + 2];
            _debugCable(
              world,
              `merge pinch-pair path=${pathId} idx=${i} joints=${jointId_i},${jointId_i_plus_1} ` +
              `storedMid=${path.stored[i + 1].toFixed(6)} storedNext=${path.stored[i + 2].toFixed(6)} ` +
              `removedBudget=${removedBudget.toFixed(6)}`
            );
            path.stored[i] += removedBudget;

            path.jointEntities.splice(i, 2);
            path.stored.splice(i + 1, 2);
            path.cw.splice(i + 1, 2);
            path.linkTypes.splice(i + 1, 2);
            world.destroyEntity(jointId_i);
            world.destroyEntity(jointId_i_plus_1);

            reRunMerge = true;
            continue;
          }
          // console.log(`Merging joints ${jointId_i} and ${jointId_i_plus_1} (stored: ${path.stored[i + 1].toFixed(4)})`);

          // Calculate angle between the two segments, just for debug
          const pA1 = joint_i.attachmentPointA_world;
          const pB2 = joint_i_plus_1.attachmentPointB_world;
          const posA = world.getComponent(joint_i.entityA, PositionComponent).pos;
          const radiusA = _effectiveRadius(path, world.getComponent(joint_i.entityA, RadiusComponent)?.radius);
          const cwA = _effectiveCW(path, i, true);
          const posB = world.getComponent(joint_i_plus_1.entityB, PositionComponent).pos;
          const radiusB = _effectiveRadius(path, world.getComponent(joint_i_plus_1.entityB, RadiusComponent)?.radius);
          const cwB = path.cw[i+2];

          joint_i.restLength += joint_i_plus_1.restLength + path.stored[i + 1];
          joint_i.entityB = joint_i_plus_1.entityB;
          const isAttachmentA = _isAttachment(path.linkTypes[i]);
          const isRollingA = _isRolling(path.linkTypes[i]);
          const isAttachmentB = _isAttachment(path.linkTypes[i+2]);
          const isRollingB = _isRolling(path.linkTypes[i+2]);

          let attachmentA_current = pA1;
          let attachmentB_current = pB2;
          if (isRollingA && isRollingB) {
            const tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
            attachmentA_current = tangents.a_circle;
            attachmentB_current = tangents.b_circle;
          } else if (isRollingA && isAttachmentB) {
            attachmentA_current = tangentFromCircleToPoint(pB2, posA, radiusA, cwA).a_circle;
          } else if (isAttachmentA && isRollingB) {
            attachmentB_current = tangentFromPointToCircle(pA1, posB, radiusB, cwB).a_circle;
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA && isRollingB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA);
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB);
          } else if (isRollingA && isAttachmentB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA);
          } else if (isAttachmentA && isRollingB) {
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB);
          }

          path.stored[i] += sA;
          joint_i.restLength -= sA;
          path.stored[i+2] -= sB;
          joint_i.restLength += sB;
          reRunMerge = path.stored[i] < 0.0 || path.stored[i+2] < 0.0;

          joint_i.attachmentPointA_world.set(attachmentA_current);
          joint_i.attachmentPointB_world.set(attachmentB_current);

          path.jointEntities.splice(i+1, 1);
          path.stored.splice(i+1, 1);
          path.cw.splice(i+1, 1);
          path.linkTypes.splice(i+1, 1);
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
        const radiusSplitter = _effectiveRadius(path, world.getComponent(splitterId, RadiusComponent)?.radius);
        if (lineSegmentCircleIntersection(pA, pB, posSplitter, radiusSplitter)) {
          // console.log(`Splitting joint ${jointId} due to intersection with ${splitterId}`);
          const entityA = joint.entityA;
          const entityB = joint.entityB;
          const newJointId = world.createEntity();
          ensureMachineTag(world, newJointId, pathMachine);

          // Get components for Entity A
          const posA = world.getComponent(entityA, PositionComponent).pos;
          const linkTypeA = path.linkTypes[i];
          const isAttachmentA = _isAttachment(linkTypeA);
          const isRollingA = _isRolling(linkTypeA);
          const radiusA = _effectiveRadius(path, world.getComponent(entityA, RadiusComponent)?.radius);
          const cwA = _effectiveCW(path, i, true);

          // Get components for Entity B
          const posB = world.getComponent(entityB, PositionComponent).pos;
          const linkTypeB = path.linkTypes[i + 1];
          const isAttachmentB = _isAttachment(linkTypeB);
          const isRollingB = _isRolling(linkTypeB);
          const radiusB = _effectiveRadius(path, world.getComponent(entityB, RadiusComponent)?.radius);
          const cwB = path.cw[i + 1];

          // Calculate components for new joint
          const prevPosSplitter = world.getComponent(splitterId, CableLinkComponent).prevCableAttachmentTimePos;
          const cw = rightOfLine(prevPosSplitter, pA, pB);

          let newAttachmentPointAForJoint = null;
          let newAttachmentPointBForJoint = null;
          if (isRollingA) {
            // Rolling -> Splitter
            const tangentAS = tangentFromCircleToCircle(posA, radiusA, cwA, posSplitter, radiusSplitter, cw);
            newAttachmentPointAForJoint = tangentAS.a_circle;
            newAttachmentPointBForJoint = tangentAS.b_circle;
          } else if (isAttachmentA) {
            // Attachment -> Splitter
            const tangentAS = tangentFromPointToCircle(pA, posSplitter, radiusSplitter, cw);
            newAttachmentPointAForJoint = tangentAS.a_attach;
            newAttachmentPointBForJoint = tangentAS.a_circle;
          } else {
            console.warn(`Splitting cable joint coming from link type ${linkTypeA} is not supported.`);
            continue;
          }

          let attachmentPointAForNewJoint = null;
          let attachmentPointBForNewJoint = null;
          if (isRollingB) {
            // Splitter -> Rolling
            const tangentSB = tangentFromCircleToCircle(posSplitter, radiusSplitter, cw, posB, radiusB, cwB);
            attachmentPointAForNewJoint = tangentSB.a_circle;
            attachmentPointBForNewJoint = tangentSB.b_circle;
          } else if (isAttachmentB) {
            // Splitter -> Attachment
            const tangentSB = tangentFromCircleToPoint(pB, posSplitter, radiusSplitter, cw);
            attachmentPointAForNewJoint = tangentSB.a_circle;
            attachmentPointBForNewJoint = tangentSB.a_attach;
          } else {
            console.warn(`Splitting cable joint attached to ${linkTypeB} is not supported.`);
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA) {
            sA = signedArcLengthOnWheel(pA, newAttachmentPointAForJoint, posA, radiusA, cwA);
          }
          if (isRollingB) {
            sB = signedArcLengthOnWheel(pB, attachmentPointBForNewJoint, posB, radiusB, cwB);
          }

          // Calculate stored length 's' on the new splitter link
          const s = signedArcLengthOnWheel(
              newAttachmentPointBForJoint, // Point on splitter from A side
              attachmentPointAForNewJoint, // Point on splitter from B side
              posSplitter,
              radiusSplitter,
              cw
          );
          if (s <= 0.0) {
              console.warn(`Nothing wraps around splitter. Aborting split.`);
              continue;
          }
          if ((s + 1e-9) >= 2.0*Math.PI * radiusSplitter) {
              console.warn(`Split resulted a full wrap around splitter. Aborting split.`);
              continue;
          }

          // The line is slightly stretched due to the intersection with the new link (the splitterId Entity).
          // Distributes the discrepancy (length_error) to both sides of the new link such that tension remains constant on both sides of the new link.
          // The paper states:
          // "The rest length of the original joint is distributed among the new joints such that the tension on both sides are equal."
          // "The rest length is split such that the tension remains constant."
          // "tension (the current length divided by the rest length)"
          // Distribute original rest length (minus new stored length s)
          // between the two new segments to maintain equal tension.
          const dAS = newAttachmentPointAForJoint.clone().subtract(newAttachmentPointBForJoint).length();
          const dSB = attachmentPointAForNewJoint.clone().subtract(attachmentPointBForNewJoint).length();
          const originalRestLength = joint.restLength + sB - sA; // Store before modifying
          const totalDist = dAS + dSB;

          let newRestLengthAS = 0; // For original joint (entityA -> splitter)
          let newRestLengthSB = 0; // For new joint (splitter -> entityB)
          if (totalDist > 1e-9) {
              const availableRestLength = originalRestLength - s;
              if (availableRestLength < 1e-9) {
                  console.warn(`Split resulted in < 1e-9 available rest length (${availableRestLength.toFixed(4)}). Aborting split.`);
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
          joint.entityB = splitterId; // Original joint now connects A -> Splitter
          path.stored.splice(i + 1, 0, s);

          // Update original joint (now entityA -> splitter)
          joint.restLength = newRestLengthAS;
          joint.attachmentPointB_world.set(newAttachmentPointBForJoint); // Update endpoint

          // Create the new joint (splitter -> entityB)
          world.addComponent(newJointId, new CableJointComponent(
              splitterId, entityB, newRestLengthSB, attachmentPointAForNewJoint, attachmentPointBForNewJoint));
          world.addComponent(newJointId, new RenderableComponent('line', linecolor1));

          // Some final debug logging
          const discrepancy = originalRestLength - s - newRestLengthAS - newRestLengthSB;
          const tensionAS = dAS/newRestLengthAS;
          const tensionSB = dSB/newRestLengthSB;
          // console.log(`Split: L_orig=${originalRestLength.toFixed(4)}, s=${s.toFixed(4)} -> L_AS=${newRestLengthAS.toFixed(4)} (d_AS=${dAS.toFixed(4)}), L_SB=${newRestLengthSB.toFixed(4)} (d_SB=${initialDistSB.toFixed(4)})`);
          // console.log(`Split stats: discrepancy=${discrepancy.toFixed(4)}, tensionAS=${tensionAS.toFixed(4)}, tensionSB=${tensionSB.toFixed(4)}`);
          if (discrepancy > 1.0) {
            console.warn("discrepancy > 1.0");
          }
          if (tensionAS > 1.5) {
            console.warn("tensionAS > 1.5");
          }
          if (tensionSB > 1.5) {
            console.warn("tensionSB > 1.5");
          }
        }
      }
    }
  }
}

export function _updateHybridLinkStates(world) {
  const pathEntities = world.query([CablePathComponent]);
  const pinchConfigsResource = world.getResource(PINCH_CONFIGS_RESOURCE);
  const pinchConfigs = pinchConfigsResource instanceof Map ? pinchConfigsResource : new Map();

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    for (const i of [0, path.linkTypes.length - 1]) {
      const endpointJointId = (i === 0 ? path.jointEntities[0] : path.jointEntities[path.jointEntities.length - 1]);
      if (endpointJointId !== undefined && pinchConfigs.has(endpointJointId)) {
        // Transitional pinches create near-degenerate endpoint geometry where
        // hybrid state switching is numerically ambiguous. Keep the endpoint
        // mode/cw stable while pinched.
        continue;
      }
      if (path.linkTypes[i] === 'hybrid') {
        if (path.stored[i] < 0.0) {
          // console.log(`Switching joint ${path.jointEntities[i == 0 ? 0 : path.jointEntities.length - 1]} to hybrid-attachment`);
          path.linkTypes[i] = 'hybrid-attachment';
          const joint = (i === 0 ? world.getComponent(path.jointEntities[i], CableJointComponent) : world.getComponent(path.jointEntities[i - 1], CableJointComponent));
          const linkEntity = (i === 0 ? joint.entityA : joint.entityB);
          const radius = _effectiveRadius(path, world.getComponent(linkEntity, RadiusComponent).radius);
          const pos = world.getComponent(linkEntity, PositionComponent).pos;
          const rawCW = path.cw[i];
          const effectiveCW = _effectiveCW(path, i, true);
          const oldStored = path.stored[i];
          // We have "fed out negative line", undo that
          joint.restLength += oldStored;
          const rotAng = -oldStored/radius;
          if (i === 0) {
            joint.attachmentPointA_world.rotate(rotAng, pos, rawCW);
          } else if (i === path.linkTypes.length - 1) {
            joint.attachmentPointB_world.rotate(rotAng, pos, rawCW);
          }
          _debugCable(
            world,
            `hybrid->hybrid-attachment path=${pathId} link=${i} ` +
            `joint=${endpointJointId} rawCW=${rawCW} effectiveCW=${effectiveCW} ` +
            `storedBefore=${oldStored.toFixed(6)} rotAng=${rotAng.toFixed(6)}`
          );
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
        const R = _effectiveRadius(path, world.getComponent(entityId, RadiusComponent).radius);
        const halfWidth = path.cableHalfWidth ?? 0.0;
        const nearPinchThreshold = 2.0 * halfWidth + 1e-6;
        let nearPinchSurfaceDistance = null;
        let nearPinch = false;
        if (halfWidth > EPSILON) {
          const surfacePair = _computeCircleSurfacePair(world, entityId, neighborId, null);
          if (surfacePair) {
            nearPinchSurfaceDistance = surfacePair.surfaceDistance;
            nearPinch = surfacePair.surfaceDistance <= nearPinchThreshold;
          }
        }

        // When the endpoint and neighbor attachment collapse to (near) the same
        // point, both CW/CCW tangent solutions become ill-conditioned.
        const degenerateThreshold = Math.max(1e-6, 2.0 * (path.cableHalfWidth ?? 0.0) + 1e-6);
        if (attachmentPoint.distanceTo(neighborAttachmentPoint) <= degenerateThreshold) {
          continue;
        }

        const tanCW  = tangentFromCircleToPoint(neighborAttachmentPoint, C, R, true).a_circle;
        const tanCCW = tangentFromCircleToPoint(neighborAttachmentPoint, C, R, false).a_circle;

        const crossedCW  = signedArcLengthOnWheel(attachmentPoint, tanCW,  C, R, true);
        const crossedCCW  = signedArcLengthOnWheel(attachmentPoint, tanCCW,  C, R, false);
        const distSqCW = attachmentPoint.distanceToSq(tanCW);
        const distSqCCW = attachmentPoint.distanceToSq(tanCCW);

        let newCW = null;
        let crossingTangent = null;
        let candidateStored = null;
        if (crossedCCW > 0.0 && distSqCCW < distSqCW) {
            newCW = true;
            crossingTangent = tanCCW;
            candidateStored = crossedCCW;
        } else if (crossedCW > 0.0 && distSqCW < distSqCCW) {
            newCW = false;
            crossingTangent = tanCW;
            candidateStored = crossedCW;
        }

        if (newCW !== null) {
          if (nearPinch) {
            _debugCable(
              world,
              `defer hybrid-attachment->hybrid near-pinch path=${pathId} link=${i} ` +
              `joint=${jointId} candidateCW=${newCW} ` +
              `surfaceDistance=${nearPinchSurfaceDistance?.toFixed(6)} threshold=${nearPinchThreshold.toFixed(6)} ` +
              `crossedCW=${crossedCW.toFixed(6)} crossedCCW=${crossedCCW.toFixed(6)} ` +
              `stored=${(path.stored[i] ?? 0.0).toFixed(6)}`
            );
            continue;
          }
          const oldRawCW = path.cw[i];
          const oldEffectiveCW = (i === 0 ? !oldRawCW : oldRawCW);
          const newEffectiveCW = (i === 0 ? !newCW : newCW);
          const oldStored = path.stored[i] ?? 0.0;
          const newStored = candidateStored ?? oldStored;
          // console.log(`Switching joint ${jointId} to hybrid`);
          path.linkTypes[i] = 'hybrid';
          path.cw[i]        = newCW;
          path.stored[i] = newStored;
          joint.restLength -= (newStored - oldStored);
          attachmentPoint.set(crossingTangent);
          _debugCable(
            world,
            `hybrid-attachment->hybrid path=${pathId} link=${i} joint=${jointId} ` +
            `rawCW=${oldRawCW}->${newCW} effectiveCW=${oldEffectiveCW}->${newEffectiveCW} ` +
            `crossedCW=${crossedCW.toFixed(6)} crossedCCW=${crossedCCW.toFixed(6)} ` +
            `stored=${oldStored.toFixed(6)}->${newStored.toFixed(6)} ` +
            `distSqCW=${distSqCW.toExponential(3)} distSqCCW=${distSqCCW.toExponential(3)}`
          );
        }
      }
    }

    for (let linkIndex = 1; linkIndex < path.linkTypes.length - 1; linkIndex++) {
      if (!_isRolling(path.linkTypes[linkIndex])) {
        continue;
      }
      const leftJointId = path.jointEntities[linkIndex - 1];
      const rightJointId = path.jointEntities[linkIndex];
      const leftJoint = world.getComponent(leftJointId, CableJointComponent);
      const rightJoint = world.getComponent(rightJointId, CableJointComponent);
      if (!leftJoint || !rightJoint || leftJoint.entityB !== rightJoint.entityA) {
        continue;
      }
      const bodyId = leftJoint.entityB;
      const center = world.getComponent(bodyId, PositionComponent)?.pos;
      const radius = _effectiveRadius(path, world.getComponent(bodyId, RadiusComponent)?.radius);
      if (!center || !Number.isFinite(radius) || radius <= EPSILON) {
        continue;
      }

      const cw = path.cw[linkIndex];
      const arc = signedArcLengthOnWheel(
        leftJoint.attachmentPointB_world,
        rightJoint.attachmentPointA_world,
        center,
        radius,
        cw,
        true
      );
      const altArc = signedArcLengthOnWheel(
        leftJoint.attachmentPointB_world,
        rightJoint.attachmentPointA_world,
        center,
        radius,
        !cw,
        true
      );
      const stored = path.stored[linkIndex] ?? 0.0;
      if (arc <= 1e-6 && altArc > arc + 1e-4 && stored > 1e-6) {
        _debugCable(
          world,
          `rolling-arc-mismatch path=${pathId} link=${linkIndex} body=${bodyId} ` +
          `cw=${cw} arc=${arc.toFixed(6)} altArc=${altArc.toFixed(6)} stored=${stored.toFixed(6)} ` +
          `leftJoint=${leftJointId} rightJoint=${rightJointId}`
        );
      }
    }
  }
}

function _computeCircleSurfacePair(
  world,
  entityA,
  entityB,
  fallbackNormal = null,
  radiusOffsetA = 0.0,
  radiusOffsetB = 0.0
) {
  const posAComp = world.getComponent(entityA, PositionComponent);
  const posBComp = world.getComponent(entityB, PositionComponent);
  const radiusAComp = world.getComponent(entityA, RadiusComponent);
  const radiusBComp = world.getComponent(entityB, RadiusComponent);
  if (!posAComp || !posBComp || !radiusAComp || !radiusBComp) {
    return null;
  }

  const centerDelta = new Vector2().subtractVectors(posBComp.pos, posAComp.pos);
  const centerDistance = centerDelta.length();
  let normal;
  if (centerDistance > EPSILON) {
    normal = centerDelta.clone().scale(1.0 / centerDistance);
  } else if (fallbackNormal && fallbackNormal.lengthSq() > EPSILON) {
    normal = fallbackNormal.clone().normalize();
  } else {
    normal = new Vector2(1.0, 0.0);
  }

  const radiusA = Math.max(0.0, radiusAComp.radius + radiusOffsetA);
  const radiusB = Math.max(0.0, radiusBComp.radius + radiusOffsetB);
  const pointA_world = posAComp.pos.clone().add(normal, radiusA);
  const pointB_world = posBComp.pos.clone().subtract(normal, radiusB);
  const surfaceDistance = centerDistance - (radiusA + radiusB);

  return {
    normal,
    pointA_world,
    pointB_world,
    centerDistance,
    surfaceDistance,
    radiusA,
    radiusB
  };
}

function _computePinchAttachmentPair(world, path, entityA, entityB, fallbackNormal = null) {
  const halfWidth = path?.cableHalfWidth ?? 0.0;
  return _computeCircleSurfacePair(
    world,
    entityA,
    entityB,
    fallbackNormal,
    halfWidth,
    halfWidth
  );
}

function _pinchSegmentDirection(normal, referenceDir = null) {
  const segmentDir = new Vector2(-normal.y, normal.x);
  if (segmentDir.lengthSq() <= EPSILON) {
    return null;
  }
  segmentDir.normalize();
  if (referenceDir && referenceDir.lengthSq() > EPSILON && segmentDir.dot(referenceDir) < 0.0) {
    segmentDir.scale(-1.0);
  }
  return segmentDir;
}

function _orient2D(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function _transitionalPinchCrossData(world, joint) {
  if (!joint) {
    return null;
  }
  const posA = world.getComponent(joint.entityA, PositionComponent)?.pos;
  const posB = world.getComponent(joint.entityB, PositionComponent)?.pos;
  const attachmentA = joint.attachmentPointA_world;
  const attachmentB = joint.attachmentPointB_world;
  if (!posA || !posB || !attachmentA || !attachmentB) {
    return null;
  }

  const orientCentersAtAttachA = _orient2D(posA, posB, attachmentA);
  const orientCentersAtAttachB = _orient2D(posA, posB, attachmentB);
  const orientAttachmentsAtCenterA = _orient2D(attachmentA, attachmentB, posA);
  const orientAttachmentsAtCenterB = _orient2D(attachmentA, attachmentB, posB);
  const oppositeOnCenterLine =
    (orientCentersAtAttachA > EPSILON && orientCentersAtAttachB < -EPSILON) ||
    (orientCentersAtAttachA < -EPSILON && orientCentersAtAttachB > EPSILON);
  const oppositeOnAttachmentLine =
    (orientAttachmentsAtCenterA > EPSILON && orientAttachmentsAtCenterB < -EPSILON) ||
    (orientAttachmentsAtCenterA < -EPSILON && orientAttachmentsAtCenterB > EPSILON);
  return {
    crosses: oppositeOnCenterLine && oppositeOnAttachmentLine,
    orientCentersAtAttachA,
    orientCentersAtAttachB,
    orientAttachmentsAtCenterA,
    orientAttachmentsAtCenterB
  };
}

function _pathCurrentLengthBudget(world, path) {
  let total = 0.0;
  for (const jointId of path.jointEntities) {
    const joint = world.getComponent(jointId, CableJointComponent);
    if (joint) {
      total += joint.restLength;
    }
  }
  for (const value of path.stored) {
    total += value;
  }
  return total;
}

function _isPointInsideRollingArc(path, leftJoint, rightJoint, linkIndex, pointOnBody, center, radius) {
  if (!leftJoint || !rightJoint || !pointOnBody || !center) {
    return false;
  }
  if (!Number.isFinite(radius) || radius <= EPSILON) {
    return false;
  }

  const tailPoint = leftJoint.attachmentPointB_world;
  const headPoint = rightJoint.attachmentPointA_world;
  const cw = path.cw[linkIndex];

  let totalArc = signedArcLengthOnWheel(tailPoint, headPoint, center, radius, cw, true);
  if (!(totalArc > EPSILON)) {
    totalArc = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
    if (!(totalArc > EPSILON)) {
      return false;
    }
  }

  const arcToPoint = signedArcLengthOnWheel(tailPoint, pointOnBody, center, radius, cw, true);
  const arcFromPoint = signedArcLengthOnWheel(pointOnBody, headPoint, center, radius, cw, true);

  const arcTolerance = Math.max(1e-6, totalArc * 1e-4);
  if (arcToPoint <= arcTolerance || arcFromPoint <= arcTolerance) {
    return false;
  }
  if (arcToPoint > totalArc + arcTolerance || arcFromPoint > totalArc + arcTolerance) {
    return false;
  }
  return (arcToPoint + arcFromPoint) <= totalArc + (2.0 * arcTolerance);
}

function _decomposeStoredWrapLayers(storedLength, baseRadius, halfWidth) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  if (!(stored > EPSILON) || !(baseRadius > EPSILON) || !(halfWidth > EPSILON)) {
    return null;
  }

  const fullWidth = 2.0 * halfWidth;
  let remaining = stored;
  let fullLayers = 0;
  const MAX_LAYERS = 2048;
  while (fullLayers < MAX_LAYERS) {
    const layerRadius = baseRadius + fullWidth * fullLayers;
    if (!(layerRadius > EPSILON)) {
      break;
    }
    const layerCircumference = 2.0 * Math.PI * layerRadius;
    if (remaining + EPSILON >= layerCircumference) {
      remaining -= layerCircumference;
      if (remaining < 0.0) {
        remaining = 0.0;
      }
      fullLayers++;
      continue;
    }

    const partialLength = remaining;
    return {
      fullLayers,
      partialLength,
      partialRadius: layerRadius,
      hasPartial: partialLength > EPSILON,
      extraDistanceForFullCoverage: 2.0 * halfWidth * Math.max(0, fullLayers - 1),
      extraDistanceForPartialCoverage: 2.0 * halfWidth * fullLayers
    };
  }

  const fallbackRadius = baseRadius + fullWidth * fullLayers;
  return {
    fullLayers,
    partialLength: 0.0,
    partialRadius: fallbackRadius,
    hasPartial: false,
    extraDistanceForFullCoverage: 2.0 * halfWidth * Math.max(0, fullLayers - 1),
    extraDistanceForPartialCoverage: 2.0 * halfWidth * fullLayers
  };
}

function _getEndpointRollingArcContext(world, path, linkIndex) {
  if (path.jointEntities.length < 1) {
    return null;
  }

  let joint;
  let bodyA;
  let neighborBody;
  let attachmentPoint;
  if (linkIndex === 0) {
    joint = world.getComponent(path.jointEntities[0], CableJointComponent);
    if (!joint) {
      return null;
    }
    bodyA = joint.entityA;
    neighborBody = joint.entityB;
    attachmentPoint = joint.attachmentPointA_world;
  } else if (linkIndex === (path.linkTypes.length - 1)) {
    joint = world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent);
    if (!joint) {
      return null;
    }
    bodyA = joint.entityB;
    neighborBody = joint.entityA;
    attachmentPoint = joint.attachmentPointB_world;
  } else {
    return null;
  }

  const center = world.getComponent(bodyA, PositionComponent)?.pos;
  const radius = _effectiveRadius(path, world.getComponent(bodyA, RadiusComponent)?.radius);
  if (!center || !Number.isFinite(radius) || radius <= EPSILON || !attachmentPoint) {
    return null;
  }

  return {
    bodyA,
    neighborBody,
    center,
    baseRadius: radius,
    attachmentPoint,
    cwDirection: _effectiveCW(path, linkIndex, true)
  };
}

function _selectEndpointWrapExtraDistance(
  context,
  decomposition,
  pointOnBody,
  baseMinDistance,
  surfaceDistance
) {
  if (!context || !decomposition || !pointOnBody) {
    return null;
  }

  const arcOnBase = signedArcLengthOnWheel(
    context.attachmentPoint,
    pointOnBody,
    context.center,
    context.baseRadius,
    context.cwDirection,
    true
  );
  const deltaAngle = arcOnBase / context.baseRadius;
  const partialArcLength = deltaAngle * decomposition.partialRadius;
  const partialTolerance = Math.max(1e-6, decomposition.partialLength * 1e-4);
  const inPartialCoverage =
    decomposition.hasPartial &&
    partialArcLength > partialTolerance &&
    partialArcLength < (decomposition.partialLength - partialTolerance);

  const minDistancePartial = baseMinDistance + decomposition.extraDistanceForPartialCoverage;
  if (inPartialCoverage && surfaceDistance <= minDistancePartial + EPSILON) {
    return decomposition.extraDistanceForPartialCoverage;
  }

  const minDistanceFull = baseMinDistance + decomposition.extraDistanceForFullCoverage;
  const hasFullCoverageEverywhere = decomposition.fullLayers > 0;
  if (
    (hasFullCoverageEverywhere || inPartialCoverage) &&
    surfaceDistance <= minDistanceFull + EPSILON
  ) {
    return decomposition.extraDistanceForFullCoverage;
  }

  return null;
}

function _detectPinchCandidates(world) {
  const candidates = [];
  const nonTransitionalByKey = new Map();
  const pathEntities = world.query([CablePathComponent]);
  const potentialBodies = world.query([PositionComponent, RadiusComponent, CableLinkComponent]);

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (!path || path.jointEntities.length < 1) {
      continue;
    }
    const halfWidth = path.cableHalfWidth ?? 0.0;
    if (!(halfWidth > EPSILON)) {
      continue;
    }

    const minDistance = 2.0 * halfWidth;
    const pathMachine = getMachineId(world, pathId);

    for (const jointId of path.jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      if (!joint) {
        continue;
      }

      const fallbackNormal = joint.attachmentPointB_world.clone().subtract(joint.attachmentPointA_world);
      const surfacePair = _computeCircleSurfacePair(world, joint.entityA, joint.entityB, fallbackNormal);
      if (!surfacePair) {
        continue;
      }

      if (surfacePair.surfaceDistance > minDistance + EPSILON) {
        continue;
      }

      const pinchPair = _computePinchAttachmentPair(world, path, joint.entityA, joint.entityB, surfacePair.normal);
      if (!pinchPair) {
        continue;
      }
      const crossData = _transitionalPinchCrossData(world, joint);
      if (!crossData || !crossData.crosses) {
        if (crossData) {
          _debugCable(
            world,
            `skip transitional path=${pathId} joint=${jointId} ` +
            `surfaceDistance=${surfacePair.surfaceDistance.toFixed(6)} ` +
            `oCA=${crossData.orientCentersAtAttachA.toExponential(3)} ` +
            `oCB=${crossData.orientCentersAtAttachB.toExponential(3)} ` +
            `oAC=${crossData.orientAttachmentsAtCenterA.toExponential(3)} ` +
            `oBC=${crossData.orientAttachmentsAtCenterB.toExponential(3)}`
          );
        }
        continue;
      }

      candidates.push({
        kind: 'transitional',
        pathId,
        jointId,
        entityA: joint.entityA,
        entityB: joint.entityB,
        minDistance,
        normal: surfacePair.normal.clone(),
        pointA_world: pinchPair.pointA_world.clone(),
        pointB_world: pinchPair.pointB_world.clone(),
        surfaceDistance: surfacePair.surfaceDistance
      });
    }

    for (let linkIndex = 1; linkIndex < path.linkTypes.length - 1; linkIndex++) {
      if (!_isRolling(path.linkTypes[linkIndex])) {
        continue;
      }
      if (!(path.stored[linkIndex] > EPSILON)) {
        continue;
      }

      const leftJointId = path.jointEntities[linkIndex - 1];
      const rightJointId = path.jointEntities[linkIndex];
      const leftJoint = world.getComponent(leftJointId, CableJointComponent);
      const rightJoint = world.getComponent(rightJointId, CableJointComponent);
      if (!leftJoint || !rightJoint) {
        continue;
      }
      const bodyA = leftJoint.entityB;
      if (bodyA !== rightJoint.entityA) {
        continue;
      }

      const centerA = world.getComponent(bodyA, PositionComponent)?.pos;
      const radiusA = _effectiveRadius(path, world.getComponent(bodyA, RadiusComponent)?.radius);
      if (!centerA || !Number.isFinite(radiusA) || radiusA <= EPSILON) {
        continue;
      }

      const neighborTail = leftJoint.entityA;
      const neighborHead = rightJoint.entityB;
      for (const bodyB of potentialBodies) {
        if (bodyB === bodyA || bodyB === neighborTail || bodyB === neighborHead) {
          continue;
        }
        if (pathMachine !== getMachineId(world, bodyB)) {
          continue;
        }

        const surfacePair = _computeCircleSurfacePair(world, bodyA, bodyB, null);
        if (!surfacePair) {
          continue;
        }
        if (surfacePair.surfaceDistance > minDistance + EPSILON) {
          continue;
        }

        const pinchPair = _computePinchAttachmentPair(world, path, bodyA, bodyB, surfacePair.normal);
        if (!pinchPair) {
          continue;
        }
        if (!_isPointInsideRollingArc(path, leftJoint, rightJoint, linkIndex, pinchPair.pointA_world, centerA, radiusA)) {
          continue;
        }

        const key = `${pathId}:${linkIndex}:${bodyB}`;
        const existing = nonTransitionalByKey.get(key);
        if (!existing || surfacePair.surfaceDistance < existing.surfaceDistance) {
          nonTransitionalByKey.set(key, {
            kind: 'non-transitional',
            pathId,
            linkIndex,
            entityA: bodyA,
            entityB: bodyB,
            minDistance,
            normal: surfacePair.normal.clone(),
            surfaceDistance: surfacePair.surfaceDistance
          });
        }
      }
    }

    for (const linkIndex of [0, path.linkTypes.length - 1]) {
      if (!_isHybrid(path.linkTypes[linkIndex])) {
        continue;
      }
      if (!(path.stored[linkIndex] > EPSILON)) {
        continue;
      }

      const endpointContext = _getEndpointRollingArcContext(world, path, linkIndex);
      if (!endpointContext) {
        continue;
      }
      const decomposition = _decomposeStoredWrapLayers(
        path.stored[linkIndex],
        endpointContext.baseRadius,
        halfWidth
      );
      if (!decomposition) {
        continue;
      }

      const maxMinDistance = minDistance + (
        decomposition.hasPartial
          ? decomposition.extraDistanceForPartialCoverage
          : decomposition.extraDistanceForFullCoverage
      );
      for (const bodyB of potentialBodies) {
        if (
          bodyB === endpointContext.bodyA ||
          bodyB === endpointContext.neighborBody
        ) {
          continue;
        }
        if (pathMachine !== getMachineId(world, bodyB)) {
          continue;
        }

        const surfacePair = _computeCircleSurfacePair(world, endpointContext.bodyA, bodyB, null);
        if (!surfacePair) {
          continue;
        }
        if (surfacePair.surfaceDistance > maxMinDistance + EPSILON) {
          continue;
        }

        const pinchPair = _computePinchAttachmentPair(
          world,
          path,
          endpointContext.bodyA,
          bodyB,
          surfacePair.normal
        );
        if (!pinchPair) {
          continue;
        }

        const extraDistance = _selectEndpointWrapExtraDistance(
          endpointContext,
          decomposition,
          pinchPair.pointA_world,
          minDistance,
          surfacePair.surfaceDistance
        );
        if (extraDistance === null) {
          continue;
        }
        const minDistanceWithLayers = minDistance + extraDistance;

        const key = `${pathId}:${linkIndex}:${bodyB}`;
        const existing = nonTransitionalByKey.get(key);
        if (
          !existing ||
          minDistanceWithLayers > existing.minDistance + EPSILON ||
          surfacePair.surfaceDistance < existing.surfaceDistance - EPSILON
        ) {
          nonTransitionalByKey.set(key, {
            kind: 'non-transitional-endpoint',
            pathId,
            linkIndex,
            entityA: endpointContext.bodyA,
            entityB: bodyB,
            minDistance: minDistanceWithLayers,
            normal: surfacePair.normal.clone(),
            surfaceDistance: surfacePair.surfaceDistance
          });
        }
      }
    }
  }

  for (const candidate of nonTransitionalByKey.values()) {
    candidates.push(candidate);
  }

  world.setResource(PINCH_CANDIDATES_RESOURCE, candidates);
}

function _configureTransitionalPinch(world, candidate, pinchConfigs) {
  const path = world.getComponent(candidate.pathId, CablePathComponent);
  const joint = world.getComponent(candidate.jointId, CableJointComponent);
  if (!path || !joint) {
    return false;
  }

  const referenceDir = joint.attachmentPointB_world.clone().subtract(joint.attachmentPointA_world);
  const pinchPair = _computePinchAttachmentPair(world, path, joint.entityA, joint.entityB, candidate.normal);
  if (!pinchPair) {
    return false;
  }

  const segmentDir = _pinchSegmentDirection(pinchPair.normal, referenceDir);
  if (!segmentDir) {
    return false;
  }

  joint.attachmentPointA_world.set(pinchPair.pointA_world);
  joint.attachmentPointB_world.set(pinchPair.pointB_world);

  pinchConfigs.set(candidate.jointId, {
    pathId: candidate.pathId,
    jointId: candidate.jointId,
    entityA: candidate.entityA,
    entityB: candidate.entityB,
    minDistance: candidate.minDistance,
    normal: pinchPair.normal.clone(),
    segmentDir,
    pinchPointA_world: pinchPair.pointA_world.clone(),
    pinchPointB_world: pinchPair.pointB_world.clone()
  });
  return true;
}

function _insertNonTransitionalPinch(world, candidate, pinchConfigs) {
  const path = world.getComponent(candidate.pathId, CablePathComponent);
  if (!path) {
    return false;
  }
  const linkIndex = candidate.linkIndex;
  if (linkIndex <= 0 || linkIndex >= path.jointEntities.length) {
    return false;
  }
  if (!_isRolling(path.linkTypes[linkIndex])) {
    return false;
  }

  const leftJointId = path.jointEntities[linkIndex - 1];
  const rightJointId = path.jointEntities[linkIndex];
  const leftJoint = world.getComponent(leftJointId, CableJointComponent);
  const rightJoint = world.getComponent(rightJointId, CableJointComponent);
  if (!leftJoint || !rightJoint) {
    return false;
  }

  const bodyA = leftJoint.entityB;
  if (bodyA !== rightJoint.entityA || bodyA !== candidate.entityA) {
    return false;
  }
  const bodyB = candidate.entityB;
  if (bodyB === bodyA || bodyB === leftJoint.entityA || bodyB === rightJoint.entityB) {
    return false;
  }

  const centerA = world.getComponent(bodyA, PositionComponent)?.pos;
  const radiusA = _effectiveRadius(path, world.getComponent(bodyA, RadiusComponent)?.radius);
  if (!centerA || !Number.isFinite(radiusA) || radiusA <= EPSILON) {
    return false;
  }

  const pinchPair = _computePinchAttachmentPair(world, path, bodyA, bodyB, candidate.normal);
  if (!pinchPair) {
    return false;
  }
  if (!_isPointInsideRollingArc(path, leftJoint, rightJoint, linkIndex, pinchPair.pointA_world, centerA, radiusA)) {
    return false;
  }

  const totalBefore = _pathCurrentLengthBudget(world, path);

  const oldTail = leftJoint.attachmentPointB_world.clone();
  const oldHead = rightJoint.attachmentPointA_world.clone();
  const splitPointA = pinchPair.pointA_world.clone();
  const splitPointB = pinchPair.pointB_world.clone();
  const referenceDir = oldHead.clone().subtract(oldTail);
  const segmentDir = _pinchSegmentDirection(pinchPair.normal, referenceDir);
  if (!segmentDir) {
    return false;
  }

  leftJoint.attachmentPointB_world.set(splitPointA);
  rightJoint.attachmentPointA_world.set(splitPointA);

  const machineId = getMachineId(world, candidate.pathId);
  const jointABId = world.createEntity();
  ensureMachineTag(world, jointABId, machineId);
  world.addComponent(
    jointABId,
    new CableJointComponent(bodyA, bodyB, 0.0, splitPointA.clone(), splitPointB.clone())
  );
  world.addComponent(jointABId, new RenderableComponent('line', linecolor1));

  const jointBAId = world.createEntity();
  ensureMachineTag(world, jointBAId, machineId);
  world.addComponent(
    jointBAId,
    new CableJointComponent(bodyB, bodyA, 0.0, splitPointB.clone(), splitPointA.clone())
  );
  world.addComponent(jointBAId, new RenderableComponent('line', linecolor1));

  path.jointEntities.splice(linkIndex, 0, jointABId, jointBAId);

  const cwA = path.cw[linkIndex];
  const leftArc = signedArcLengthOnWheel(oldTail, splitPointA, centerA, radiusA, cwA, true);
  const rightArc = signedArcLengthOnWheel(splitPointA, oldHead, centerA, radiusA, cwA, true);
  const bodyBPrevPos = world.getComponent(bodyB, CableLinkComponent)?.prevCableAttachmentTimePos;
  const bodyBPos = world.getComponent(bodyB, PositionComponent)?.pos;
  const cwB = rightOfLine(
    bodyBPrevPos ?? bodyBPos ?? splitPointB,
    oldTail,
    oldHead
  );

  path.linkTypes.splice(linkIndex, 1, 'rolling', 'rolling', 'rolling');
  path.cw.splice(linkIndex, 1, cwA, cwB, cwA);
  path.stored.splice(
    linkIndex,
    1,
    leftArc,
    PINCH_NON_TRANSITIONAL_STORED_BUFFER,
    rightArc
  );

  const totalAfter = _pathCurrentLengthBudget(world, path);
  const deltaBudget = totalBefore - totalAfter;
  const jointBA = world.getComponent(jointBAId, CableJointComponent);
  if (jointBA) {
    jointBA.restLength += deltaBudget;
  }

  pinchConfigs.set(jointABId, {
    pathId: candidate.pathId,
    jointId: jointABId,
    entityA: bodyA,
    entityB: bodyB,
    minDistance: candidate.minDistance,
    normal: pinchPair.normal.clone(),
    segmentDir: segmentDir.clone(),
    pinchPointA_world: splitPointA.clone(),
    pinchPointB_world: splitPointB.clone()
  });
  pinchConfigs.set(jointBAId, {
    pathId: candidate.pathId,
    jointId: jointBAId,
    entityA: bodyB,
    entityB: bodyA,
    minDistance: candidate.minDistance,
    normal: pinchPair.normal.clone().scale(-1.0),
    segmentDir: segmentDir.clone(),
    pinchPointA_world: splitPointB.clone(),
    pinchPointB_world: splitPointA.clone()
  });
  _debugCable(
    world,
    `insert non-transitional path=${candidate.pathId} link=${linkIndex} bodies=${bodyA}<->${bodyB} ` +
    `joints=${jointABId},${jointBAId} arcs=${leftArc.toFixed(6)},${rightArc.toFixed(6)} ` +
    `buffer=${PINCH_NON_TRANSITIONAL_STORED_BUFFER.toFixed(6)} deltaBudget=${deltaBudget.toFixed(6)}`
  );
  return true;
}

function _configureEndpointNonTransitionalPinch(world, candidate, pinchConfigs) {
  const path = world.getComponent(candidate.pathId, CablePathComponent);
  if (!path) {
    return false;
  }

  const pinchPair = _computePinchAttachmentPair(
    world,
    path,
    candidate.entityA,
    candidate.entityB,
    candidate.normal
  );
  if (!pinchPair) {
    return false;
  }

  const key = `endpoint:${candidate.pathId}:${candidate.linkIndex}:${candidate.entityA}:${candidate.entityB}`;
  pinchConfigs.set(key, {
    pathId: candidate.pathId,
    jointId: null,
    entityA: candidate.entityA,
    entityB: candidate.entityB,
    minDistance: candidate.minDistance,
    normal: pinchPair.normal.clone(),
    pinchPointA_world: pinchPair.pointA_world.clone(),
    pinchPointB_world: pinchPair.pointB_world.clone()
  });
  _debugCable(
    world,
    `configure endpoint-non-transitional path=${candidate.pathId} link=${candidate.linkIndex} ` +
    `bodies=${candidate.entityA}<->${candidate.entityB} minDistance=${candidate.minDistance.toFixed(6)}`
  );
  return true;
}

function _configurePinches(world) {
  const candidates = world.getResource(PINCH_CANDIDATES_RESOURCE);
  const pinchConfigs = new Map();
  if (!Array.isArray(candidates)) {
    world.setResource(PINCH_CONFIGS_RESOURCE, pinchConfigs);
    return;
  }

  const nonTransitional = candidates.filter((candidate) => candidate.kind === 'non-transitional');
  nonTransitional.sort((a, b) => {
    if (a.pathId !== b.pathId) {
      return a.pathId - b.pathId;
    }
    return b.linkIndex - a.linkIndex;
  });
  for (const candidate of nonTransitional) {
    _insertNonTransitionalPinch(world, candidate, pinchConfigs);
  }

  const endpointNonTransitional = candidates.filter((candidate) => candidate.kind === 'non-transitional-endpoint');
  for (const candidate of endpointNonTransitional) {
    _configureEndpointNonTransitionalPinch(world, candidate, pinchConfigs);
  }

  for (const candidate of candidates) {
    if (candidate.kind !== 'transitional') {
      continue;
    }
    _configureTransitionalPinch(world, candidate, pinchConfigs);
  }

  world.setResource(PINCH_CONFIGS_RESOURCE, pinchConfigs);
}

function _buildPinchContacts(world) {
  const pinchConfigs = world.getResource(PINCH_CONFIGS_RESOURCE);
  const dedupedContacts = new Map();
  if (!(pinchConfigs instanceof Map)) {
    world.setResource(PINCH_CONTACTS_RESOURCE, []);
    return;
  }

  for (const config of pinchConfigs.values()) {
    let entityA = config.entityA;
    let entityB = config.entityB;
    let normal = config.normal.clone();
    if (entityA > entityB) {
      entityA = config.entityB;
      entityB = config.entityA;
      normal.scale(-1.0);
    }
    const key = `${entityA}:${entityB}`;
    const existing = dedupedContacts.get(key);
    if (!existing || config.minDistance > existing.minDistance) {
      dedupedContacts.set(
        key,
        new PinchContact(
          entityA,
          entityB,
          config.pathId,
          config.minDistance,
          normal,
          0.0
        )
      );
    }
  }

  world.setResource(PINCH_CONTACTS_RESOURCE, Array.from(dedupedContacts.values()));
}

export class CableAttachmentUpdateSystem {
  constructor(options = {}) {
    this.includeTopology = options.includeTopology ?? true;
  }

  runInPause = false;

  update(world, dt) {
    _clearDebugPoints(world);
    _updateAttachmentPoints(world);
    if (!this.includeTopology) {
      return;
    }
    _mergeJoints(world);
    _splitJoints(world);
    _updateHybridLinkStates(world);
  }
}

export class CableTopologySystem {
  runInPause = false;

  update(world, _dt_unused) {
    _mergeJoints(world);
    _splitJoints(world);
    _updateHybridLinkStates(world);
  }
}

export class PinchDetectionSystem {
  runInPause = false;

  update(world, _dt_unused) {
    _detectPinchCandidates(world);
  }
}

export class PinchConfigureSystem {
  runInPause = false;

  update(world, _dt_unused) {
    _configurePinches(world);
  }
}

export class PinchConstraintBuildSystem {
  runInPause = false;

  update(world, _dt_unused) {
    _buildPinchContacts(world);
  }
}

export class PBDCableConstraintSolver {
  runInPause = false;

  update(world, _dt_unused) {
    const pathEntities = world.query([CablePathComponent]);
    const dt = world.getResource('dt');
    const pinchConfigsResource = world.getResource(PINCH_CONFIGS_RESOURCE);
    const pinchContactsResource = world.getResource(PINCH_CONTACTS_RESOURCE);
    const pinchConfigs = pinchConfigsResource instanceof Map ? pinchConfigsResource : new Map();
    const pinchContacts = Array.isArray(pinchContactsResource) ? pinchContactsResource : [];

    const ITERATIONS = 2; // 0: Forward, 1: Backward

    // Pre-calculate local offsets for all joints to ensure attachment points move with bodies
    // during the multi-iteration solve. This prevents "stale" world points from distorting geometry.
    const jointLocals = new Map();
    const computeLocal = (entityId, worldPoint) => {
      const posComp = world.getComponent(entityId, PositionComponent);
      const orientComp = world.getComponent(entityId, OrientationComponent);
      if (!posComp) return new Vector2(0, 0); // Should not happen for valid bodies

      const rel = worldPoint.clone().subtract(posComp.pos);
      if (orientComp) {
        // Rotate backwards by -angle to get local frame
        const c = Math.cos(-orientComp.angle);
        const s = Math.sin(-orientComp.angle);
        return new Vector2(rel.x * c - rel.y * s, rel.x * s + rel.y * c);
      }
      return rel;
    };

    const applyConstraint = (
      entityA,
      entityB,
      pointA_world,
      pointB_world,
      gradPosA,
      gradPosB,
      constraintError,
      compliance
    ) => {
      if (constraintError <= EPSILON) {
        return;
      }

      const massAComp = world.getComponent(entityA, MassComponent);
      const invMassA = (massAComp && massAComp.mass > 0) ? 1.0 / massAComp.mass : 0.0;
      const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
      const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;

      const massBComp = world.getComponent(entityB, MassComponent);
      const invMassB = (massBComp && massBComp.mass > 0) ? 1.0 / massBComp.mass : 0.0;
      const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
      const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

      // If both entities are effectively immovable for this constraint, skip.
      if (invMassA + invMassB + invInertiaA + invInertiaB <= EPSILON) {
        return;
      }

      const posAComp = world.getComponent(entityA, PositionComponent);
      const posBComp = world.getComponent(entityB, PositionComponent);
      if (!posAComp || !posBComp) {
        return;
      }

      const rA = new Vector2().subtractVectors(pointA_world, posAComp.pos);
      const rB = new Vector2().subtractVectors(pointB_world, posBComp.pos);
      const gradAngA = rA.x * gradPosA.y - rA.y * gradPosA.x;
      const gradAngB = rB.x * gradPosB.y - rB.y * gradPosB.x;

      let denom = 0.0;
      denom += invMassA * gradPosA.lengthSq();
      denom += invInertiaA * gradAngA * gradAngA;
      denom += invMassB * gradPosB.lengthSq();
      denom += invInertiaB * gradAngB * gradAngB;
      if (Number.isFinite(dt) && dt > EPSILON) {
        denom += (compliance ?? 0.0) / (dt * dt);
      }

      if (denom <= EPSILON) {
        return;
      }

      const lambda = -constraintError / denom;

      if (invMassA > 0.0) {
        const deltaPosA = gradPosA.clone().scale(-invMassA * lambda);
        posAComp.pos.add(deltaPosA);
      }
      if (invInertiaA > 0.0) {
        const deltaAngA = -invInertiaA * lambda * gradAngA;
        const orientationAComp = world.getComponent(entityA, OrientationComponent);
        if (orientationAComp) {
          orientationAComp.angle += deltaAngA;
        }
      }

      if (invMassB > 0.0) {
        const deltaPosB = gradPosB.clone().scale(-invMassB * lambda);
        posBComp.pos.add(deltaPosB);
      }
      if (invInertiaB > 0.0) {
        const deltaAngB = -invInertiaB * lambda * gradAngB;
        const orientationBComp = world.getComponent(entityB, OrientationComponent);
        if (orientationBComp) {
          orientationBComp.angle += deltaAngB;
        }
      }
    };

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        jointLocals.set(jointId, {
          localA: computeLocal(joint.entityA, joint.attachmentPointA_world),
          localB: computeLocal(joint.entityB, joint.attachmentPointB_world)
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

          let currentSegmentLength = pA.distanceTo(pB);
          const pinchConfig = pinchConfigs.get(jointId);
          let dir = null;
          if (pinchConfig) {
            dir = pinchConfig.segmentDir.clone().normalize();
            currentSegmentLength = 0.0;
          } else if (currentSegmentLength > EPSILON) {
            dir = new Vector2().subtractVectors(pB, pA).scale(1.0 / currentSegmentLength);
          }
          if (!dir || dir.lengthSq() <= EPSILON) {
            continue;
          }

          const constraintError = currentSegmentLength - joint.restLength;
          if (constraintError <= EPSILON) {
            continue;
          }

          const gradPosA = dir.clone();
          const gradPosB = dir.clone().scale(-1.0);
          applyConstraint(
            entityA,
            entityB,
            pA,
            pB,
            gradPosA,
            gradPosB,
            constraintError,
            path.compliance
          );
        } // End loop through joints in path

      } // End loop through paths

      for (const contact of pinchContacts) {
        const surfacePair = _computeCircleSurfacePair(world, contact.entityA, contact.entityB, contact.normal);
        if (!surfacePair) {
          continue;
        }

        // Contact inequality C = 2w - d <= 0, where d is the closest surface distance.
        const constraintError = contact.minDistance - surfacePair.surfaceDistance;
        if (constraintError <= EPSILON) {
          continue;
        }

        const gradPosA = surfacePair.normal.clone().scale(-1.0);
        const gradPosB = surfacePair.normal.clone();
        applyConstraint(
          contact.entityA,
          contact.entityB,
          surfacePair.pointA_world,
          surfacePair.pointB_world,
          gradPosA,
          gradPosB,
          constraintError,
          contact.compliance
        );
      }
    } // End loop through iterations
  } // end update
} // end PBDCableConstraintSolver
