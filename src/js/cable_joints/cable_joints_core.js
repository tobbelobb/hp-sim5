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
  constructor(world, jointEntities = [], linkTypes = [], cw = [], spring_constant = 1e6, stored = null) {
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw // Ordered. cw.length === linkTypes.length
    this.spring_constant = spring_constant;
    this.compliance = 1.0/spring_constant;
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length

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
        const rolling_link = world.getComponent(linkId, CableLinkComponent);
        const center = world.getComponent(linkId, PositionComponent).pos;
        const radius = world.getComponent(linkId, RadiusComponent).radius;
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


function _effectiveCW(path, linkIndex, travellingFromCircle) {
  if (linkIndex === 0 && travellingFromCircle)
    return !path.cw[linkIndex];
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

  // Get components for Entity A
  const posAComp = world.getComponent(entityA, PositionComponent);
  const radiusAComp = world.getComponent(entityA, RadiusComponent);
  const linkAComp = world.getComponent(entityA, CableLinkComponent);
  const orientationAComp = world.getComponent(entityA, OrientationComponent);

  const posA = posAComp?.pos;
  const attachmentA_previous = joint.attachmentPointA_world;
  const prevPosA = linkAComp?.prevCableAttachmentTimePos;
  const radiusA = radiusAComp?.radius;
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
  const radiusB = radiusBComp?.radius;
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

      // Get components for Entity A
      const posAComp = world.getComponent(entityA, PositionComponent);
      const radiusAComp = world.getComponent(entityA, RadiusComponent);
      const linkAComp = world.getComponent(entityA, CableLinkComponent);
      const orientationAComp = world.getComponent(entityA, OrientationComponent);
      const posA = posAComp?.pos;
      const prevPosA = linkAComp?.prevCableAttachmentTimePos;
      const radiusA = radiusAComp?.radius;
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
      const radiusB = radiusBComp?.radius;
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
        if (joint_i.entityA === joint_i_plus_1.entityB) {
          // A cable has wrapped around a link, and back. This will not be a merge candidate.
          continue;
        }
        if (path.stored[i + 1] < 0.0) {
          // console.log(`Merging joints ${jointId_i} and ${jointId_i_plus_1} (stored: ${path.stored[i + 1].toFixed(4)})`);

          // Calculate angle between the two segments, just for debug
          const pA1 = joint_i.attachmentPointA_world;
          const pB2 = joint_i_plus_1.attachmentPointB_world;
          const posA = world.getComponent(joint_i.entityA, PositionComponent).pos;
          const radiusA = world.getComponent(joint_i.entityA, RadiusComponent)?.radius;
          const cwA = _effectiveCW(path, i, true);
          const posB = world.getComponent(joint_i_plus_1.entityB, PositionComponent).pos;
          const radiusB = world.getComponent(joint_i_plus_1.entityB, RadiusComponent)?.radius;
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
        const radiusSplitter = world.getComponent(splitterId, RadiusComponent)?.radius;
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
          const radiusA = world.getComponent(entityA, RadiusComponent)?.radius;
          const cwA = _effectiveCW(path, i, true);

          // Get components for Entity B
          const posB = world.getComponent(entityB, PositionComponent).pos;
          const linkTypeB = path.linkTypes[i + 1];
          const isAttachmentB = _isAttachment(linkTypeB);
          const isRollingB = _isRolling(linkTypeB);
          const radiusB = world.getComponent(entityB, RadiusComponent)?.radius;
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
  const debugPoints = world.getResource('debugRenderPoints');
  const pathEntities = world.query([CablePathComponent]);

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    for (const i of [0, path.linkTypes.length - 1]) {
      const epsilon = 1e-9;
      if (path.linkTypes[i] === 'hybrid') {
        if (path.stored[i] < 0.0) {
          // console.log(`Switching joint ${path.jointEntities[i == 0 ? 0 : path.jointEntities.length - 1]} to hybrid-attachment`);
          path.linkTypes[i] = 'hybrid-attachment';
          const joint = (i === 0 ? world.getComponent(path.jointEntities[i], CableJointComponent) : world.getComponent(path.jointEntities[i - 1], CableJointComponent));
          const linkEntity = (i === 0 ? joint.entityA : joint.entityB);
          const radius = world.getComponent(linkEntity, RadiusComponent).radius;
          const pos = world.getComponent(linkEntity, PositionComponent).pos;
          // We have "fed out negative line", undo that
          joint.restLength += path.stored[i];
          const rotAng = -path.stored[i]/radius;
          if (i === 0) {
            joint.attachmentPointA_world.rotate(rotAng, pos, path.cw[i]);
          } else if (i === path.linkTypes.length - 1) {
            joint.attachmentPointB_world.rotate(rotAng, pos, path.cw[i]);
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

        const tanCW  = tangentFromCircleToPoint(neighborAttachmentPoint, C, R, true).a_circle;
        const tanCCW = tangentFromCircleToPoint(neighborAttachmentPoint, C, R, false).a_circle;

        const crossedCW  = signedArcLengthOnWheel(attachmentPoint, tanCW,  C, R, true);
        const crossedCCW  = signedArcLengthOnWheel(attachmentPoint, tanCCW,  C, R, false);
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
          // console.log(`Switching joint ${jointId} to hybrid`);
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
    const epsilon = 1e-9; // Small value to avoid division by zero
    const dt = world.getResource('dt');

    const ITERATIONS = 2; // 0: Forward, 1: Backward

    // Pre-calculate local offsets for all joints to ensure attachment points move with bodies
    // during the multi-iteration solve. This prevents "stale" world points from distorting geometry.
    const jointLocals = new Map();

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        
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

          const currentSegmentLength = pA.distanceTo(pB);
          const constraintError = currentSegmentLength - joint.restLength;
        // Apply correction only if the segment is longer than its rest length
        if (constraintError > epsilon) {
          const massAComp = world.getComponent(entityA, MassComponent);
          const invMassA = (massAComp && massAComp.mass > 0) ? 1.0 / massAComp.mass : 0.0;
          const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
          const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;

          const massBComp = world.getComponent(entityB, MassComponent);
          const invMassB = (massBComp && massBComp.mass > 0) ? 1.0 / massBComp.mass : 0.0;
          const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
          const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

          // If both entities are effectively immovable for this constraint, skip
          if (invMassA + invMassB + invInertiaA + invInertiaB <= epsilon) {
            continue;
          }

          const diff = new Vector2().subtractVectors(pB, pA);
          const len = diff.length();
          if (len <= epsilon) continue;
          const dir = diff.clone().scale(1.0 / len); // Normalized direction from A to B

          // Gradients of the constraint C = |pB - pA| - restLength
          // ∇_pA C = (pA - pB) / |pA - pB| = -dir
          // ∇_pB C = (pB - pA) / |pB - pA| = dir
          // However, the existing PBD solver code structure implies gradients for |pB-pA|
          // gradPosA = dir (for entity A's contribution to |pB-pA|)
          // gradPosB = -dir (for entity B's contribution to |pB-pA|)
          const gradPosA = dir.clone();
          const gradPosB = dir.clone().scale(-1.0);

          const posAComp = world.getComponent(entityA, PositionComponent);
          const rA = new Vector2().subtractVectors(pA, posAComp.pos); // Vector from CoM of A to attachment point A
          // gradAngA = rA × gradPosA_contrib_to_pA = rA × dir
          const gradAngA = rA.x * dir.y - rA.y * dir.x;

          const posBComp = world.getComponent(entityB, PositionComponent);
          const rB = new Vector2().subtractVectors(pB, posBComp.pos); // Vector from CoM of B to attachment point B
          // gradAngB = rB × gradPosB_contrib_to_pB = rB × (-dir)
          const gradAngB = rB.x * (-dir.y) - rB.y * (-dir.x); // which is -(rB.x * dir.y - rB.y * dir.x)

          let denom = 0.0;
          denom += invMassA * gradPosA.lengthSq();
          denom += invInertiaA * gradAngA * gradAngA;
          denom += invMassB * gradPosB.lengthSq();
          denom += invInertiaB * gradAngB * gradAngB;
          denom += path.compliance / (dt * dt);

          if (denom <= epsilon) {
            // console.warn("PBDCableConstraintSolver: zero denominator for joint " + jointId);
            continue;
          }

          const lambda = -constraintError / denom;

          // Apply corrections to Entity A
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

          // Apply corrections to Entity B
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
        }
      } // End loop through joints in path
    } // End loop through paths
    } // End loop through iterations
  } // end update
} // end PBDCableConstraintSolver
