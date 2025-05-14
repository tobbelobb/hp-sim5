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
  RenderableComponent
} from './ecs.js';

import {
  GravitySystem,
  MovementSystem,
} from './commonSystems.js';

export const linecolor1 = '#FFFF00'

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
}

// Connects individual cable joints into a cable path
export class CablePathComponent {
  constructor(world, jointEntities = [], linkTypes = [], cw = [], spring_constant = 1e6) {
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
  }
}


// --- System: Cable Attachment Update ---
// Calculates tangent points and updates rest lengths (dn) BEFORE the main solver
//
// # AttachmentPoints updates means updating the "ABRS" variables
// - _updateAttachmentPoints's main job is to update `joint.attachmentPointA_world` and `joint.attachmentPointB_world`
//   (call them AB for short), so that they remain on top of or tangent to whatever points they're connected to.
// - A change of AB must balance out with an update of `joint.restLength` and `path[i].stored`
//   (call them RS for short) so that no amount of cable vanishes or is created.
// - Let's call those four variables the ABRS-variables or just "ABRS".
// - Updates to ABRS are based on new PositionComponent pos and OrientationComponent angles that were calculated
//   during the previous time step (possibly modified by earlier systems in the current time step).
//
//  # Features
//  - CableAttachmentUpdateSystem also handles how joints might be merged or split do to intersections or non-intersections with objects
//    that interact with cables (objects with CableLinkComponent). That's two features: split and merge.
//  - And CableAttachmentUpdateSystem also handles something called "hybrid links" which is a pair of features that let cables attach to the perimeter of
//    rolling links, effectively impementing the behaviour of a spool in a cable system. A spool has to modes of operation: "line left" which is modelled with a rolling
//    (hybrid) link, and "no line left" which is modelled with a moving attachment point at the spool's perimeter (a "hybrid-attachment").
//  - Switching from hybrid to hybrid-attachment is one feature, switching back is another feature.
//  - All these four features run their logic _after_  an initial round of ABRS updates.
//  - Each feature is responsible for leaving ABRS in a physically consistent state. No new line is allowed to be removed or created.
//
//  ## Merge Feature
//  - The merge feature updates ABRS. It also destroys one element from `path.jointEntities`, `path.stored`, `path.cw`, `path.linkTypes`. Then it destroys the whole joint object.
//
//  ## Hybrid Features
//  - The hybrid/hybrid-attachment features are currently both handled by one function which is called `_updateHybridLinkStates`.
//  - The hybrid->hybrid-attachment logic updates ABRS, and `path.linkTypes[i]`.
//  - The hybrid-attachment->hybrid logic updates ABRS, `path.linkTypes[i]`, and `path.cw[i]`.
//
//  ## Split Feature
//  - The split feature updates ABRS. It also updates joint.entityB and it creates a whole new joint with `entityA`, `entityB`, `restLength`, `attachmentPointA_world`, and `attachmentPointB_world` all set.
//    It also creates an element in `path.jointEntities`, `path.stored`, `path.cw`, `path.linkTypes`.
//  - The split feature distributes the available restLength so that tension becomes equal on both sides of the new link.
//  - There's a separate feature at the end of CableAttachmentUpdateSystem update that tries to even out tension across links.
//    It _only_ changes pairs of `joint.restLength`. It does not change `joint.attachmentPointA_world`, `joint.attachmentPointB_world`, or `path.stored[i]`.
//
//  ## Tension Distribution Feature
//  - This logic is called "// Even out tension" in the code and currently has no separate function.
//
//  ## Memory Feature
//  - At the very end of CableAttachmentUpdateSystem there's a feature that updates CableLinkComponent.prevCableAttachmentTimePos for all links with a PositionComponent
//    so that correct prevPos is available for the next time step/iteration of CableAttachmentUpdateSystem update.
//    It does the same with CableLinkComponent.prevCableAttachmentTimeAngle
//    There's a subtle point to why we store this timestep's intermediate position and angle but not the final or initial one at any given timestep.
//    This is because the full position and angle deltas affect our ABRS, and both position and angle variables are changed before and after
//    CableAttachmentUpdateSystem is invoked at each time step. So we need to capture the previous values (which current ABRS values derived from)
//    at the exact part of the time step/game loop where CableAttachmentUpdateSystem itself is placed.
//  - Each feature is responsible for updating ABRS and any other variable (`path.cw[i]`, `path.linkTypes[i], `joint.entityA`, `joint.entityB`);
export class CableAttachmentUpdateSystem {
  runInPause = false;

  _effectiveCW(path, linkIndex, travellingFromCircle) {
    if (linkIndex === 0 && travellingFromCircle)
      return !path.cw[linkIndex];
    return path.cw[linkIndex];
  }

  _clearDebugPoints(world) {
      const debugPoints = world.getResource('debugRenderPoints');
      if (debugPoints) {
          for (const key in debugPoints) {
              delete debugPoints[key];
          }
      }
  }

  _updateAttachmentPoints(world) {
    const pathEntities = world.query([CablePathComponent]);

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);

      for (let i = 0; i < path.jointEntities.length; i++) {
        const jointId = path.jointEntities[i];
        const joint = world.getComponent(jointId, CableJointComponent);

        const A = i; // Index for link/cw/stored related to entity A side
        const B = i + 1; // Index for link/cw/stored related to entity B side

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
        const cwA = this._effectiveCW(path, A,  true);
        const attachmentLinkA = path.linkTypes[A] === 'attachment' || path.linkTypes[A] === 'hybrid-attachment';
        const rollingLinkA = path.linkTypes[A] === 'rolling' || path.linkTypes[A] === 'hybrid';
        const isHybridA = path.linkTypes[A] === 'hybrid' || path.linkTypes[A] === 'hybrid-attachment';
        const pADiffFromTranslation = posA.clone().subtract(prevPosA)
        const pADiffFromRotation = attachmentA_previous.clone().rotate(deltaAngleA, prevPosA, true).subtract(attachmentA_previous);

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
        const cwB = this._effectiveCW(path, B,  false);
        const attachmentLinkB = path.linkTypes[B] === 'attachment' || path.linkTypes[B] === 'hybrid-attachment';
        const rollingLinkB = path.linkTypes[B] === 'rolling' || path.linkTypes[B] === 'hybrid';
        const isHybridB = path.linkTypes[B] === 'hybrid' || path.linkTypes[B] === 'hybrid-attachment';
        const pBDiffFromTranslation = posB.clone().subtract(prevPosB)
        const pBDiffFromRotation = attachmentB_previous.clone().rotate(deltaAngleB, prevPosB, true).subtract(attachmentB_previous);

        // --- Calculate  Attachment Points based on this frame's positions and rotations ---
        let attachmentA_current = posA
        let attachmentB_current = posB;
        if (attachmentLinkA && rollingLinkB) {
          if (isHybridA) {
            attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
          }
          attachmentB_current = tangentFromPointToCircle(attachmentA_current, posB, radiusB, cwB).a_circle;
        } else if (rollingLinkA && attachmentLinkB) {
          if (isHybridB) {
            attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
          }
          attachmentA_current = tangentFromCircleToPoint(attachmentB_current, posA, radiusA, cwA).a_circle;
        } else if (rollingLinkA && rollingLinkB) {
          const tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
          attachmentA_current = tangents.a_circle;
          attachmentB_current = tangents.b_circle;
        } else { // attachmentLinkA && attachmentLinkB
          if (isHybridA) {
            attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
          }
          if (isHybridB) {
            attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
          }
        }

        // --- Calculate Wrapping/Unwrapping Arc Lengths (sA, sB) ---
        let sA = 0; // Change in stored length on side A due to wrapping/unwrapping this frame
        let sB = 0; // Change in stored length on side B due to wrapping/unwrapping this frame
        if (rollingLinkA) {
            sA = signedArcLengthOnWheel(attachmentA_previous.clone().subtract(prevPosA), attachmentA_current.clone().subtract(posA), new Vector2(0.0, 0.0), radiusA, cwA);
            if (isHybridA) {
              sA += (cwA ? deltaAngleA*radiusA : -deltaAngleA*radiusA);
            }
        }
        if (rollingLinkB) {
            sB = signedArcLengthOnWheel(attachmentB_previous.clone().subtract(prevPosB), attachmentB_current.clone().subtract(posB), new Vector2(0.0, 0.0), radiusB, cwB);
            if (isHybridB) {
              sB += (cwB ? deltaAngleB*radiusB : -deltaAngleB*radiusB);
            }
        }
        path.stored[A] += sA;
        joint.restLength -= sA;
        path.stored[B] -= sB;
        joint.restLength += sB;

        joint.attachmentPointA_world.set(attachmentA_current);
        joint.attachmentPointB_world.set(attachmentB_current);
      }
    }
  }

  _mergeJoints(world) {
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
            const cwA = this._effectiveCW(path, i, true);
            const posB = world.getComponent(joint_i_plus_1.entityB, PositionComponent).pos;
            const radiusB = world.getComponent(joint_i_plus_1.entityB, RadiusComponent)?.radius;
            const cwB = path.cw[i+2];

            joint_i.restLength += joint_i_plus_1.restLength + path.stored[i + 1];
            joint_i.entityB = joint_i_plus_1.entityB;
            const isRollingA = path.linkTypes[i] === 'rolling' || path.linkTypes[i] === 'hybrid';
            const isAttachmentA = path.linkTypes[i] === 'attachment' || path.linkTypes[i] === 'hybrid-attachment';
            const isRollingB = path.linkTypes[i+2] === 'rolling' || path.linkTypes[i+2] === 'hybrid';
            const isAttachmentB = path.linkTypes[i+2] === 'attachment' || path.linkTypes[i+2] === 'hybrid-attachment';

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

  _evenOutTension(world) {
    const pathEntities = world.query([CablePathComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 2) continue;
      for (let i = 0; i < path.jointEntities.length - 1; i++) {
        const jointId_i = path.jointEntities[i];
        const jointId_i_plus_1 = path.jointEntities[i + 1];
        const joint_i = world.getComponent(jointId_i, CableJointComponent);
        const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
        const pA = joint_i.attachmentPointA_world;
        const pB = joint_i.attachmentPointB_world;
        const pC = joint_i_plus_1.attachmentPointA_world;
        const pD = joint_i_plus_1.attachmentPointB_world;
        const l_i = joint_i.restLength;
        const l_i_plus_1 = joint_i_plus_1.restLength;
        const availableRestLength = l_i + l_i_plus_1;
        const d_i = new Vector2().subtractVectors(pA, pB).length();
        const d_i_plus_1 = new Vector2().subtractVectors(pC, pD).length();
        const totalDist = d_i + d_i_plus_1;
        const tension_i = d_i/joint_i.restLength;
        const tension_i_plus_1 = d_i_plus_1/joint_i_plus_1.restLength;
        joint_i.restLength = availableRestLength * d_i/totalDist;
        joint_i_plus_1.restLength = availableRestLength * d_i_plus_1/totalDist;
        //console.log(`tension_i=${tension_i}, tension_i_plus_1=${tension_i_plus_1}`);
      }
    }
  }

  _evenOutTensionPartial(world, alpha = 0.5) {
    const pathEntities = world.query([CablePathComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 2) continue;

      for (let i = 0; i < path.jointEntities.length - 1; i++) {
        const j0 = world.getComponent(path.jointEntities[i],     CableJointComponent);
        const j1 = world.getComponent(path.jointEntities[i + 1], CableJointComponent);

        // current segment lengths
        const d0 = j0.attachmentPointA_world
                     .clone().subtract(j0.attachmentPointB_world)
                     .length();
        const d1 = j1.attachmentPointA_world
                     .clone().subtract(j1.attachmentPointB_world)
                     .length();

        // total rest length available to split
        const sumL = j0.restLength + j1.restLength;
        const totalD = d0 + d1;
        if (totalD <= 1e-9) continue;  // avoid divide by zero

        // target rest lengths for perfect even-tension
        const target0 = sumL * (d0 / totalD);
        const target1 = sumL * (d1 / totalD);

        // move each restLength a fraction α toward its target
        j0.restLength += alpha * (target0 - j0.restLength);
        j1.restLength += alpha * (target1 - j1.restLength);
      }
    }
  }

  _splitJoints(world) {
    const potentialSplitters = world.query([PositionComponent, RadiusComponent, CableLinkComponent]);
    const pathEntities = world.query([CablePathComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
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
          const posSplitter = world.getComponent(splitterId, PositionComponent).pos;
          const radiusSplitter = world.getComponent(splitterId, RadiusComponent)?.radius;
          if (lineSegmentCircleIntersection(pA, pB, posSplitter, radiusSplitter)) {
            // console.log(`Splitting joint ${jointId} due to intersection with ${splitterId}`);
            const entityA = joint.entityA;
            const entityB = joint.entityB;
            const newJointId = world.createEntity();

            // Get components for Entity A
            const posA = world.getComponent(entityA, PositionComponent).pos;
            const linkTypeA = path.linkTypes[i];
            const isRollingA = linkTypeA === 'rolling' || linkTypeA === 'hybrid';
            const isAttachmentA = linkTypeA === 'attachment' || linkTypeA === 'hybrid-attachment';
            const radiusA = world.getComponent(entityA, RadiusComponent)?.radius;
            const cwA = this._effectiveCW(path, i, true);

            // Get components for Entity B
            const posB = world.getComponent(entityB, PositionComponent).pos;
            const linkTypeB = path.linkTypes[i + 1];
            const isRollingB = linkTypeB === 'rolling' || linkTypeB === 'hybrid';
            const isAttachmentB = linkTypeB === 'attachment' || linkTypeB === 'hybrid-attachment';
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

  _updateHybridLinkStates(world) {
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

  _storeCableLinkPoses(world) {
    const linkEntities = world.query([CableLinkComponent, PositionComponent]);
    for (const linkId of linkEntities) {
      const posComp = world.getComponent(linkId, PositionComponent);
      const orientationComp = world.getComponent(linkId, OrientationComponent);
      const linkComp = world.getComponent(linkId, CableLinkComponent);
      linkComp.prevCableAttachmentTimePos.set(posComp.pos);
      if (orientationComp) {
        linkComp.prevCableAttachmentTimeAngle = orientationComp.angle;
      }
    }
  }

  _sanityCheck(world) {
    const pathEntities = world.query([CablePathComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      let totalCurrentDist = path.stored[0];
      let totalCurrentRestLength = path.stored[0];
      for (let i = 0; i < path.jointEntities.length; i++) {
        const jointId = path.jointEntities[i];
        const joint = world.getComponent(jointId, CableJointComponent);

        // Calculate current lengths using the attachment points updated in Pass A
        const currentDist = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
        totalCurrentDist += currentDist + path.stored[i + 1];
        totalCurrentRestLength += joint.restLength + path.stored[i + 1];
      }

      const error = path.totalRestLength - totalCurrentRestLength;
      if (error > 1e-9) {
        console.warn(`Nonzero error path ${pathId}: ${error}`);
      }
      if (path.stored.some(x => x < 0)) {
        console.warn(`Negative stored: ${path.stored}`);
      }
    }
  }

  update(world, dt) {
    this._clearDebugPoints(world);
    this._updateAttachmentPoints(world);
    const even_out_how_much = 1.0;
    this._evenOutTensionPartial(world, even_out_how_much);
    this._mergeJoints(world);
    this._evenOutTensionPartial(world, even_out_how_much);
    this._splitJoints(world);
    this._evenOutTensionPartial(world, even_out_how_much);
    this._updateHybridLinkStates(world);
    this._evenOutTensionPartial(world, even_out_how_much);
    this._storeCableLinkPoses(world);
    this._sanityCheck(world);
  }
}


export class PBDCableConstraintSolver {
  runInPause = false;

  update(world, dt) {
    const pathEntities = world.query([CablePathComponent]);
    const epsilon = 1e-9; // Small value to avoid division by zero

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;

      // --- 1. Calculate Total Current Length and Overall Tension ---
      let totalCurrentLength = 0.0;
      for (let i = 0; i < path.jointEntities.length; i++) {
        const jointId = path.jointEntities[i];
        const joint = world.getComponent(jointId, CableJointComponent);

        const currentSegmentLength = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
        totalCurrentLength += currentSegmentLength;

        // Add stored length for the link *after* this joint (if it exists)
        if (path.linkTypes[i+1] === 'rolling') {
           totalCurrentLength += path.stored[i+1];
        }
      }
      if (path.linkTypes[0] === 'hybrid') {
        totalCurrentLength += path.stored[0];
      }
      if (path.linkTypes[path.linkTypes.length - 1] === 'hybrid') {
        totalCurrentLength += path.stored[path.linkTypes.length - 1];
      }

      let overallTension = 0.0;
      if (path.totalRestLength > epsilon) {
        overallTension = totalCurrentLength / path.totalRestLength;
      }

      // --- Calculate Total Inverse Mass for the Entire Path ---
      let pathTotalInvMass = 0.0;
      const uniqueEntities = new Set();
      for (const jointId of path.jointEntities) {
          const joint = world.getComponent(jointId, CableJointComponent);

          const entityA = joint.entityA;
          const entityB = joint.entityB;

          if (!uniqueEntities.has(entityA)) {
              uniqueEntities.add(entityA);
              const massAComp = world.getComponent(entityA, MassComponent);
              if (massAComp) {
                  const massA = massAComp.mass;
                  pathTotalInvMass += (massA > 0 ? 1.0 / massA : 0.0);
              }
          }
          if (!uniqueEntities.has(entityB)) {
              uniqueEntities.add(entityB);
              const massBComp = world.getComponent(entityB, MassComponent);
              if (massBComp) {
                  const massB = massBComp.mass;
                  pathTotalInvMass += (massB > 0 ? 1.0 / massB : 0.0);
              }
          }
      }

      // If the entire path is immovable, skip corrections for this path
      if (pathTotalInvMass <= epsilon) {
          continue;
      }

      // --- Calculate Total Path Error ---
      const totalPathError = totalCurrentLength - path.totalRestLength;


      // --- 2. Apply PBD distance constraint using proper gradient sums ---
      // Apply correction only if the entire path is longer than its rest length
      if (totalPathError > epsilon) {
        // Build per-entity gradient data including translation and rotation
        const gradData = new Map(); // entityId -> {gradPos, gradAng, invMass, invInertia}
        // Initialize gradient data for each unique entity in the path
        for (const jointId of path.jointEntities) {
          const joint = world.getComponent(jointId, CableJointComponent);
          for (const e of [joint.entityA, joint.entityB]) {
            if (!gradData.has(e)) {
              const massComp = world.getComponent(e, MassComponent);
              const invMass = massComp && massComp.mass > 0 ? 1.0 / massComp.mass : 0.0;
              const moiComp = world.getComponent(e, MomentOfInertiaComponent);
              const invInertia = moiComp ? moiComp.invInertia : 0.0;
              gradData.set(e, {
                gradPos: new Vector2(0, 0),
                gradAng: 0.0,
                invMass: invMass,
                invInertia: invInertia
              });
            }
          }
        }
        // Accumulate gradients for each segment
        for (const jointId of path.jointEntities) {
          const joint = world.getComponent(jointId, CableJointComponent);
          const pA = joint.attachmentPointA_world;
          const pB = joint.attachmentPointB_world;
          const diff = new Vector2().subtractVectors(pB, pA);
          const len = diff.length();
          if (len <= epsilon) continue;
          const dir = diff.clone().scale(1.0 / len);
          // Entity A translation and rotation gradients
          const dataA = gradData.get(joint.entityA);
          dataA.gradPos.add(dir);
          const posAC = world.getComponent(joint.entityA, PositionComponent).pos;
          const rA = new Vector2().subtractVectors(pA, posAC);
          dataA.gradAng += (rA.x * dir.y - rA.y * dir.x);
          // Entity B translation and rotation gradients
          const dataB = gradData.get(joint.entityB);
          dataB.gradPos.add(dir.clone().scale(-1.0));
          const posBC = world.getComponent(joint.entityB, PositionComponent).pos;
          const rB = new Vector2().subtractVectors(pB, posBC);
          dataB.gradAng += (-(rB.x * dir.y - rB.y * dir.x));
        }
        // Compute denominator: Σ invMass * |gradPos|² + invInertia * gradAng²
        let denom = 0.0;
        for (const data of gradData.values()) {
          denom += data.invMass * data.gradPos.lengthSq();
          denom += data.invInertia * data.gradAng * data.gradAng;
          denom += path.compliance / (dt*dt);
        }
        if (denom <= epsilon) {
          console.warn("PBDCableConstraintSolver: zero denominator in constraint correction");
        } else {
          // Correction magnitude λ = -C/D
          const lambda = -totalPathError / denom;
          // Apply corrections to translation and rotation
          for (const [entityId, data] of gradData.entries()) {
            // Translation correction
            if (data.invMass > 0.0) {
              const deltaPos = data.gradPos.clone().scale(-data.invMass * lambda);
              const posComp = world.getComponent(entityId, PositionComponent);
              if (posComp) {
                posComp.pos.add(deltaPos);
              }
              const velComp = world.getComponent(entityId, VelocityComponent);
              if (velComp && dt > epsilon) {
                velComp.vel.add(deltaPos, 1.0 / dt);
                const v = velComp.vel.length();
                const maxSpeed = 0.03/(2.0*dt);
                if (v > maxSpeed) { // Enforce max speed
                  velComp.vel.scale(maxSpeed/v);
                  console.log(`Scaling speed from ${v} down to ${velComp.vel.length()}`);
                }
              }
            }
            // Rotation correction
            if (data.invInertia > 0.0) {
              const deltaAng = -data.invInertia * lambda * data.gradAng;
              const orientationComp = world.getComponent(entityId, OrientationComponent);
              if (orientationComp) {
                orientationComp.angle += deltaAng;
              }
              const angVelComp = world.getComponent(entityId, AngularVelocityComponent);
              if (angVelComp && dt > epsilon) {
                angVelComp.angularVelocity += deltaAng / dt;
              }
            }
          }
        }
      }
    } // End loop through paths
  } // end update
} // end PBDCableConstraintSolver
