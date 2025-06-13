import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

import {
  tangentFromPointToSphere,
  tangentFromSphereToSphere,
  arcLengthOnSphere,
  lineSegmentSphereIntersection,
} from './geometry3.js'; // NOTE: Switched to geometry3

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
  RenderableComponent
} from './ecs.js';

import {
  GravitySystem,
  MovementSystem,
} from './commonSystems.js';

export const linecolor1 = '#FFFF00';

export class CableLinkComponent {
  constructor(x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 1) {
    this.prevCableAttachmentTimePos = new Vector3(x, y, z);
    this.prevCableAttachmentTimeOrientation = new Quaternion(qx, qy, qz, qw);
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
  constructor(world, jointEntities = [], linkTypes = [], cw = [], spring_constant = 1e6, stored = null) {
    // NOTE: This constructor's logic is highly 2D-specific and needs to be ported to 3D.
    // The concept of 'cw' (clockwise) must be replaced with a 3D equivalent,
    // like a wrapping axis vector. The geometry functions for arc length are also 2D.
    // The logic has been commented out and will need a 3D implementation.
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw // This is 2D specific. In 3D, this would be a wrapping axis.
    this.spring_constant = spring_constant;
    this.compliance = 1.0/spring_constant;
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length

    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      this.totalRestLength += joint.restLength;
    }
    /*
    // 2D-specific logic below, needs to be ported to 3D using 3D geometry functions.
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
    */

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

// NOTE: The systems below are highly dependent on 2D geometry and have not been ported to 3D.
// Porting them requires a significant redesign of the geometric calculations for wrapping,
// merging, and splitting cables around 3D objects (spheres or cylinders).

function _isAttachment(value) {
  return value === 'attachment' || value === 'hybrid-attachment' || value === 'pinhole';
}

function _isRolling(value) {
  return value === 'rolling' || value === 'hybrid';
}

export class CableAttachmentUpdateSystem {
  runInPause = false;

  update(world, dt) {
    // This system's logic is entirely 2D and needs a full rewrite for 3D.
    // console.warn("CableAttachmentUpdateSystem is not implemented for 3D yet.");
  }
}

export class PBDCableConstraintSolver {
  runInPause = false;

  update(world, _dt_unused) {
    // This system's logic needs to be updated for 3D, particularly the gradient calculations
    // which must account for 3D positions and quaternion orientations.
    // console.warn("PBDCableConstraintSolver is not fully implemented for 3D yet.");
  }
}
