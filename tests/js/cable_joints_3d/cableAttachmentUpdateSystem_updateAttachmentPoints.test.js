import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';

import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent
} from '../../../src/js/cable_joints_3d/ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  _updateAttachmentPoints
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

import {
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  tangentFromSphereToSphere,
  signedArcLengthOnWheel
} from '../../../src/js/cable_joints_3d/geometry3.js';

const PLANE_NORMAL = new Vector3(0, 0, 1);

function setAxisAngle(orientationComp, axis, angle) {
  const q = new Quaternion().setFromAxisAngle(axis, angle);
  orientationComp.quaternion.set(q);
}

describe('_updateAttachmentPoints (3D)', () => {
  test('Attachment to Rolling - Translation', () => {
    const world = new World();

    // Entities
    const attachPoint = world.createEntity();
    const rollingLink = world.createEntity();

    // Components
    const attachPos = new Vector3(0, 2, 0);
    const rollingPos = new Vector3(0, 0, 0);
    const rollingRadius = 1.0;
    const cw = true;

    world.addComponent(attachPoint, new PositionComponent(attachPos.x, attachPos.y, attachPos.z));
    world.addComponent(attachPoint, new CableLinkComponent(attachPos.x, attachPos.y, attachPos.z, null, PLANE_NORMAL)); // prevPos = current pos initially

    world.addComponent(rollingLink, new PositionComponent(rollingPos.x, rollingPos.y, rollingPos.z));
    world.addComponent(rollingLink, new RadiusComponent(rollingRadius));
    world.addComponent(rollingLink, new CableLinkComponent(rollingPos.x, rollingPos.y, rollingPos.z, null, PLANE_NORMAL)); // prevPos = current pos initially

    // Initial Tangent
    const initialTangent = tangentFromPointToSphere(attachPos, rollingPos, rollingRadius, PLANE_NORMAL, cw);
    const initialAttachA = initialTangent.a_attach;
    const initialAttachB = initialTangent.a_sphere;
    const initialRestLength = initialAttachA.distanceTo(initialAttachB);

    // Cable Joint
    const jointId = world.createEntity();
    const jointComp = new CableJointComponent(attachPoint, rollingLink, initialRestLength, initialAttachA.clone(), initialAttachB.clone());
    world.addComponent(jointId, jointComp);

    // Cable Path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointId], ['attachment', 'hybrid'], [cw, cw]);
    world.addComponent(pathId, pathComp);
    const initialStoredB = pathComp.stored[1];
    expect(initialStoredB).toBeCloseTo(0.0, 6);

    // --- Simulate Movement ---
    const moveVector = new Vector3(0.5, 0, 0);
    const newAttachPos = attachPos.clone().add(moveVector);
    world.getComponent(attachPoint, PositionComponent).pos.set(newAttachPos); // Update current position

    _updateAttachmentPoints(world);

    // --- Assertions ---
    // Calculate expected new tangent points
    const expectedTangent = tangentFromPointToSphere(newAttachPos, rollingPos, rollingRadius, PLANE_NORMAL, cw);
    const expectedAttachA = expectedTangent.a_attach;
    const expectedAttachB = expectedTangent.a_sphere;

    // Check attachment points
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(expectedAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(expectedAttachA.y);
    expect(jointComp.attachmentPointA_world.z).toBeCloseTo(expectedAttachA.z);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(expectedAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(expectedAttachB.y);
    expect(jointComp.attachmentPointB_world.z).toBeCloseTo(expectedAttachB.z);

    // Check stored length change (sB)
    const expectedSB = signedArcLengthOnWheel(
        initialAttachB.clone().subtract(rollingPos), // prevAttachB relative to prevCenterB
        expectedAttachB.clone().subtract(rollingPos), // currentAttachB relative to currentCenterB
        new Vector3(0,0,0), // Center offset for calculation is zero
        rollingRadius,
        cw,
        PLANE_NORMAL
    );
    expect(pathComp.stored[1]).toBeCloseTo(initialStoredB - expectedSB); // stored[B] -= sB

    // Check rest length change
    // restLength += sB
    expect(jointComp.restLength).toBeCloseTo(initialRestLength + expectedSB);

    // Total rest length of path should remain constant
    const finalTotalRestLength = jointComp.restLength + pathComp.stored[0] + pathComp.stored[1];
    expect(finalTotalRestLength).toBeCloseTo(pathComp.totalRestLength);

  });

  test('Rolling to Rolling (hybrid) - Translation', () => {
    const world = new World();

    // Entities
    const attach0 = world.createEntity();
    const rollingA = world.createEntity();
    const rollingB = world.createEntity();

    // Components
    const pos0 = new Vector3(-2.5, -2, 0);
    const posA = new Vector3(-2.0, 0, 0);
    const radiusA = 0.5;
    const cwA = true;
    const posB = new Vector3(2.0, 0, 0);
    const radiusB = 0.5;
    const cwB = true;

    world.addComponent(attach0, new PositionComponent(pos0.x, pos0.y, pos0.z));
    world.addComponent(attach0, new CableLinkComponent(pos0.x, pos0.y, pos0.z, null, PLANE_NORMAL)); // prevPos

    world.addComponent(rollingA, new PositionComponent(posA.x, posA.y, posA.z));
    world.addComponent(rollingA, new RadiusComponent(radiusA));
    world.addComponent(rollingA, new CableLinkComponent(posA.x, posA.y, posA.z, null, PLANE_NORMAL)); // prevPos

    world.addComponent(rollingB, new PositionComponent(posB.x, posB.y, posB.z));
    world.addComponent(rollingB, new RadiusComponent(radiusB));
    world.addComponent(rollingB, new CableLinkComponent(posB.x, posB.y, posB.z, null, PLANE_NORMAL)); // prevPos

    // Initial Tangents and joint attach->rolling
    const initialTangents_0 = tangentFromPointToSphere(pos0, posA, radiusA, PLANE_NORMAL, cwA);
    const initialAttachA_0 = initialTangents_0.a_attach;
    const initialAttachB_0 = initialTangents_0.a_sphere;
    const initialRestLength_0 = initialAttachA_0.distanceTo(initialAttachB_0);
    const jointId_0 = world.createEntity();
    const jointComp_0 = new CableJointComponent(attach0, rollingA, initialRestLength_0, initialAttachA_0, initialAttachB_0);
    world.addComponent(jointId_0, jointComp_0);

    // Initial Tangents Rolling links
    const initialTangents = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, PLANE_NORMAL);
    const initialAttachA = initialTangents.a_sphere;
    const initialAttachB = initialTangents.b_sphere;
    const initialRestLength = initialAttachA.distanceTo(initialAttachB);

    // Cable Joint
    const jointId = world.createEntity();
    const jointComp = new CableJointComponent(rollingA, rollingB, initialRestLength, initialAttachA.clone(), initialAttachB.clone());
    world.addComponent(jointId, jointComp);
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(initialAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(initialAttachA.y);
    expect(jointComp.attachmentPointA_world.z).toBeCloseTo(initialAttachA.z);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(initialAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(initialAttachB.y);
    expect(jointComp.attachmentPointB_world.z).toBeCloseTo(initialAttachB.z);

    // Cable Path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointId_0, jointId], ['attachment', 'rolling', 'hybrid'], [true, cwA, cwB]);
    world.addComponent(pathId, pathComp);
    const initialStored0 = pathComp.stored[0];
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];

    // --- Simulate Movement ---
    const moveVectorA = new Vector3(0.0, 0.1, 0.0); // Move A slightly up
    const newPosA = posA.clone().add(moveVectorA);
    world.getComponent(rollingA, PositionComponent).pos.set(newPosA);

    _updateAttachmentPoints(world);

    // --- Assertions ---
    // Calculate expected new tangent points
    const expectedTangents = tangentFromSphereToSphere(newPosA, radiusA, cwA, posB, radiusB, cwB, PLANE_NORMAL);
    const expectedAttachA = expectedTangents.a_sphere;
    const expectedAttachB = expectedTangents.b_sphere;

    // Check attachment points
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(expectedAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(expectedAttachA.y);
    expect(jointComp.attachmentPointA_world.z).toBeCloseTo(expectedAttachA.z);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(expectedAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(expectedAttachB.y);
    expect(jointComp.attachmentPointB_world.z).toBeCloseTo(expectedAttachB.z);

    // Check stored length changes (sA, sB)
    const expectedSA = signedArcLengthOnWheel(
        initialAttachA.clone().subtract(posA),    // prevAttachA relative to prevCenterA
        expectedAttachA.clone().subtract(newPosA), // currentAttachA relative to currentCenterA
        new Vector3(0,0,0), radiusA, cwA, PLANE_NORMAL
    );

    const expectedSB = signedArcLengthOnWheel(
        initialAttachB.clone().subtract(posB), // prevAttachB relative to prevCenterB
        expectedAttachB.clone().subtract(posB), // currentAttachB relative to currentCenterB
        new Vector3(0,0,0), radiusB, cwB, PLANE_NORMAL
    );

    expect(pathComp.stored[1]).toBeCloseTo(initialStoredA + expectedSA); // stored[A] += sA
    expect(pathComp.stored[2]).toBeCloseTo(initialStoredB - expectedSB); // stored[B] -= sB

    // Check rest length change
    // restLength -= sA; restLength += sB;
    expect(jointComp.restLength).toBeCloseTo(initialRestLength - expectedSA + expectedSB);

    // Total rest length of path should remain constant
    const finalTotalRestLength = jointComp_0.restLength + jointComp.restLength + pathComp.stored[0] + pathComp.stored[1] + pathComp.stored[2];
     expect(finalTotalRestLength).toBeCloseTo(pathComp.totalRestLength);
  });

  test('hybrid-attachment -> rolling -> rolling -> hybrid with small rotations', () => {
    const world = new World();

    // Entities: hybrid-attachment start, two rolling links, hybrid end
    const r = 0.5;
    const startAttach = world.createEntity();
    const rollA = world.createEntity();
    const rollB = world.createEntity();
    const endHybrid = world.createEntity();
    const cwStart = true;
    const cwEnd = true;

    // Initial positions and parameters
    const posStart = new Vector3(-2.5, -2.0, 0.0);
    const posA     = new Vector3(-2.0,  0.0, 0.0);
    const posB     = new Vector3( 2.0,  0.0, 0.0);
    const posEnd   = new Vector3( 2.5,  2.0, 0.0);
    const cwA = true;
    const cwB = false;
    const smallRot = 0.1;

    // Setup positions, radii, orientations on all four entities
    [ [startAttach, posStart, r],
      [rollA, posA, r],
      [rollB, posB, r],
      [endHybrid, posEnd, r] ].forEach(([e, pos, r_], i) => {
      world.addComponent(e, new PositionComponent(pos.x, pos.y, pos.z));
      world.addComponent(e, new OrientationComponent(0, 0, 0, 1));
      world.addComponent(e, new RadiusComponent(r_));
      world.addComponent(e, new CableLinkComponent(pos.x, pos.y, pos.z, null, PLANE_NORMAL));
    });

    // Create joints: start->rollA, rollA->rollB, rollB->end
    const jointIds = [];
    const t0_tangent = tangentFromPointToSphere(posStart, posA, r, PLANE_NORMAL, cwA);
    const t0_dir = t0_tangent.a_sphere.clone().subtract(t0_tangent.a_attach).normalize();
    const t0_hybrid_attach = posStart.clone().add(t0_dir.scale(r));
    const t1 = tangentFromSphereToSphere(posA, r, cwA, posB, r, cwB, PLANE_NORMAL);
    const t2 = tangentFromSphereToSphere(posB, r, cwB, posEnd, r, cwEnd, PLANE_NORMAL);
    const tArgs = [
      [startAttach, rollA, t0_hybrid_attach, t0_tangent.a_sphere],
      [rollA, rollB, t1.a_sphere, t1.b_sphere],
      [rollB, endHybrid, t2.a_sphere, t2.b_sphere],
    ];
    tArgs.forEach(([a, b, Apt, Bpt]) => {
      const id = world.createEntity();
      world.addComponent(id, new CableJointComponent(
        a, b, Apt.distanceTo(Bpt), Apt.clone(), Bpt.clone()
      ));
      jointIds.push(id);
    });
    const firstJoint = world.getComponent(jointIds[0], CableJointComponent);
    expect(firstJoint.attachmentPointA_world).toEqual(t0_hybrid_attach);
    expect(firstJoint.attachmentPointB_world).toEqual(t0_tangent.a_sphere);

    // Build the cable path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      jointIds,
      ['hybrid-attachment', 'rolling', 'rolling', 'hybrid'],
      [cwStart, cwA, cwB, cwEnd]
    );
    world.addComponent(pathId, pathComp);
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];
    const initialStoredEnd = pathComp.stored[3];
    expect(initialStoredEnd).toBeCloseTo(0.0, 8);

    // Rotate both hybrid ends slightly before update
    setAxisAngle(world.getComponent(startAttach, OrientationComponent), PLANE_NORMAL, smallRot);
    setAxisAngle(world.getComponent(endHybrid, OrientationComponent), PLANE_NORMAL, -smallRot);

    // Run attachment update
    _updateAttachmentPoints(world);

    // Verity stored
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    expect(pathComp.stored[1]).toBeLessThan(initialStoredA);
    expect(pathComp.stored[2]).toBeCloseTo(initialStoredB, 8);
    expect(pathComp.stored[3]).toBeCloseTo(r*smallRot, 8);

    // Verify that the hybrid ends have moved from their original tangents
    expect(firstJoint.attachmentPointA_world).not.toEqual(t0_hybrid_attach);
    expect(firstJoint.attachmentPointA_world.x).toBeLessThan(t0_hybrid_attach.x);
    expect(firstJoint.attachmentPointA_world.y).toBeLessThan(t0_hybrid_attach.y);
    expect(firstJoint.attachmentPointA_world.distanceTo(t0_hybrid_attach)).toBeLessThan(r*smallRot);
    expect(firstJoint.attachmentPointA_world.distanceTo(posStart)).toBeCloseTo(r, 6);

    expect(firstJoint.attachmentPointB_world).not.toEqual(t0_tangent.a_sphere);
    expect(firstJoint.attachmentPointB_world.x).toBeGreaterThan(t0_tangent.a_sphere.x);
    expect(firstJoint.attachmentPointB_world.y).toBeGreaterThan(t0_tangent.a_sphere.y);
    expect(firstJoint.attachmentPointB_world.distanceTo(t0_tangent.a_sphere)).toBeLessThan(r*smallRot);
    expect(firstJoint.attachmentPointB_world.distanceTo(posA)).toBeCloseTo(r, 6);

    const lastJoint = world.getComponent(jointIds[jointIds.length - 1], CableJointComponent);
    expect(lastJoint.attachmentPointB_world).toEqual(t2.b_sphere);
  });

  test('hybrid -> rolling -> rolling -> hybrid-attachment with small rotations', () => {
    const world = new World();

    // Entities: hybrid-attachment start, two rolling links, hybrid end
    const r = 0.5;
    const startHybrid = world.createEntity();
    const rollA = world.createEntity();
    const rollB = world.createEntity();
    const endHybridAttach = world.createEntity();
    const cwStart = false;
    const cwEnd = true;

    // Initial positions and parameters
    const posStart = new Vector3(-3.0, -2.0, 0.0);
    const posA     = new Vector3(-2.0,  0.0, 0.0);
    const posB     = new Vector3( 2.0,  0.0, 0.0);
    const posEnd   = new Vector3( 2.5,  2.0, 0.0);
    const cwA = true;
    const cwB = false;
    const smallRot = 0.1;

    // Setup positions, radii, orientations on all four entities
    [ [startHybrid, posStart, r],
      [rollA, posA, r],
      [rollB, posB, r],
      [endHybridAttach, posEnd, r] ].forEach(([e, pos, r_], i) => {
      world.addComponent(e, new PositionComponent(pos.x, pos.y, pos.z));
      world.addComponent(e, new OrientationComponent(0, 0, 0, 1));
      world.addComponent(e, new RadiusComponent(r_));
      world.addComponent(e, new CableLinkComponent(pos.x, pos.y, pos.z, null, PLANE_NORMAL));
    });

    // Create joints: start->rollA, rollA->rollB, rollB->end
    const jointIds = [];
    // NOTE: This !cwStart is the biggest gotcha in the whole code base.
    //       cw/ccw are treated differently for hybrid links at position A.
    const t0 = tangentFromSphereToSphere(posStart, r, !cwStart, posA, r, cwA, PLANE_NORMAL); // !cwStart because of _effectiveCW
    const t1 = tangentFromSphereToSphere(posA, r, cwA, posB, r, cwB, PLANE_NORMAL);
    const t2_tangent = tangentFromSphereToPoint(posEnd, posB, r, PLANE_NORMAL, cwB);
    const t2_dir = t2_tangent.a_sphere.clone().subtract(t2_tangent.a_attach).normalize();
    const t2_hybrid_attach = posEnd.clone().add(t2_dir.scale(r));
    const tArgs = [
      [startHybrid, rollA, t0.a_sphere, t0.b_sphere],
      [rollA, rollB, t1.a_sphere, t1.b_sphere],
      [rollB, endHybridAttach, t2_tangent.a_sphere, t2_hybrid_attach],
    ];
    tArgs.forEach(([a, b, Apt, Bpt]) => {
      const id = world.createEntity();
      world.addComponent(id, new CableJointComponent(
        a, b, Apt.distanceTo(Bpt), Apt.clone(), Bpt.clone()
      ));
      jointIds.push(id);
    });
    const lastJoint = world.getComponent(jointIds[jointIds.length - 1], CableJointComponent);
    expect(lastJoint.attachmentPointA_world).toEqual(t2_tangent.a_sphere);
    expect(lastJoint.attachmentPointB_world).toEqual(t2_hybrid_attach);

    // Build the cable path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      jointIds,
      ['hybrid', 'rolling', 'rolling', 'hybrid-attachment'],
      [cwStart, cwA, cwB, cwEnd]
    );
    expect(lastJoint.attachmentPointA_world).toEqual(t2_tangent.a_sphere);
    expect(lastJoint.attachmentPointB_world).toEqual(t2_hybrid_attach);
    world.addComponent(pathId, pathComp);
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];
    const initialStoredEnd = pathComp.stored[3];
    expect(initialStoredEnd).toBeCloseTo(0.0, 8);

    // Rotate both hybrid ends slightly before update
    setAxisAngle(world.getComponent(startHybrid, OrientationComponent), PLANE_NORMAL, smallRot);
    setAxisAngle(world.getComponent(endHybridAttach, OrientationComponent), PLANE_NORMAL, -smallRot);

    // Run attachment update
    _updateAttachmentPoints(world);

    // Verity stored
    expect(pathComp.stored[0]).toBeGreaterThan(0.0, 8);
    expect(pathComp.stored[0]).toBeCloseTo(r*smallRot, 8);
    expect(pathComp.stored[1]).toBeCloseTo(initialStoredA, 8);
    expect(pathComp.stored[2]).toBeGreaterThan(initialStoredB, 8);

    // Verify that the hybrid ends have moved from their original tangents
    expect(lastJoint.attachmentPointB_world).not.toEqual(t2_hybrid_attach);
    expect(lastJoint.attachmentPointB_world.x).toBeLessThan(t2_hybrid_attach.x);
    expect(lastJoint.attachmentPointB_world.y).toBeGreaterThan(t2_hybrid_attach.y);
    expect(lastJoint.attachmentPointB_world.distanceTo(t2_hybrid_attach)).toBeLessThan(r*smallRot);
    expect(lastJoint.attachmentPointB_world.distanceTo(posEnd)).toBeCloseTo(r, 6);

    expect(lastJoint.attachmentPointA_world).not.toEqual(t2_tangent.a_sphere);
    expect(lastJoint.attachmentPointA_world.x).toBeLessThan(t2_tangent.a_sphere.x);
    expect(lastJoint.attachmentPointA_world.y).toBeGreaterThan(t2_tangent.a_sphere.y);
    expect(lastJoint.attachmentPointA_world.distanceTo(t2_tangent.a_sphere)).toBeLessThan(r*smallRot);
    expect(lastJoint.attachmentPointA_world.distanceTo(posB)).toBeCloseTo(r, 6);

    const firstJoint = world.getComponent(jointIds[0], CableJointComponent);
    expect(firstJoint.attachmentPointA_world).toEqual(t0.a_sphere);
  });
});
