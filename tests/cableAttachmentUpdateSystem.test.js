import Vector2 from '../cable_joints/vector2.js';

import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent
} from '../cable_joints/ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem
} from '../cable_joints/cable_joints_core.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel
} from '../cable_joints/geometry.js';

describe('CableAttachmentUpdateSystem', () => {
  test('merges joints when positions opposite vertically, conserving rest length', () => {
    const world = new World();
    const center = new Vector2(0, 0);
    const radius = 1;
    const cw = true;

    // Create entities
    const point1 = world.createEntity();
    const wheel = world.createEntity();
    const point2 = world.createEntity();

    // Components: links and positions
    world.addComponent(point1, new CableLinkComponent());
    world.addComponent(wheel, new CableLinkComponent());
    world.addComponent(point2, new CableLinkComponent());
    world.addComponent(wheel, new PositionComponent(0, 0));
    world.addComponent(point1, new PositionComponent(0.9999, 2));
    world.addComponent(point2, new PositionComponent(1.0, -2));
    world.addComponent(wheel, new RadiusComponent(radius));

    // Initial tangents on the rolling link (wheel)
    const tp1 = tangentFromPointToCircle(
      world.getComponent(point1, PositionComponent).pos,
      center,
      radius,
      cw
    );
    const tp2 = tangentFromCircleToPoint(
      world.getComponent(point2, PositionComponent).pos,
      center,
      radius,
      cw
    );

    // Create cable joints with initial rest lengths
    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        point1,
        wheel,
        tp1.a_attach.clone().subtract(tp1.a_circle).length(),
        tp1.a_attach,
        tp1.a_circle
      )
    );
    const joint2 = world.createEntity();
    world.addComponent(
      joint2,
      new CableJointComponent(
        wheel,
        point2,
        tp2.a_attach.clone().subtract(tp2.a_circle).length(),
        tp2.a_circle,
        tp2.a_attach
      )
    );

    // Build cable path
    const cablePath = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint1, joint2],
      ['attachment', 'rolling', 'attachment'],
      [cw, cw, cw]
    );
    world.addComponent(cablePath, pathComp);

    // Record initial total rest length and joint count
    const initialTotalRest = pathComp.totalRestLength;
    expect(pathComp.jointEntities).toHaveLength(2);
    expect(pathComp.linkTypes).toHaveLength(3);
    expect(pathComp.stored[1]).toBeLessThan(radius);
    expect(pathComp.stored[1]).toBeGreaterThan(0.0);

    // Let point1 travel to the right
    world.getComponent(point1, PositionComponent).prevPos = new Vector2(1.001, 2);
    world.getComponent(point1, PositionComponent).pos = new Vector2(1.001, 2);

    // Run the attachment update to trigger merge
    const system = new CableAttachmentUpdateSystem();
    world.setResource('debugRenderPoints', {});
    system.update(world);

    // After merge, one joint should be removed
    expect(pathComp.jointEntities).toHaveLength(1);
    expect(pathComp.linkTypes).toHaveLength(2);
    // Total rest length should remain unchanged
    expect(pathComp.totalRestLength).toBeCloseTo(initialTotalRest);
  });

  test('_updateAttachmentPoints: Attachment to Rolling - Translation', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();

    // Entities
    const attachPoint = world.createEntity();
    const rollingLink = world.createEntity();

    // Components
    const attachPos = new Vector2(0, 2);
    const rollingPos = new Vector2(0, 0);
    const rollingRadius = 1.0;
    const cw = true;

    world.addComponent(attachPoint, new PositionComponent(attachPos.x, attachPos.y));
    world.addComponent(attachPoint, new CableLinkComponent(attachPos.x, attachPos.y)); // prevPos = current pos initially

    world.addComponent(rollingLink, new PositionComponent(rollingPos.x, rollingPos.y));
    world.addComponent(rollingLink, new RadiusComponent(rollingRadius));
    world.addComponent(rollingLink, new CableLinkComponent(rollingPos.x, rollingPos.y)); // prevPos = current pos initially

    // Initial Tangent
    const initialTangent = tangentFromPointToCircle(attachPos, rollingPos, rollingRadius, cw);
    const initialAttachA = initialTangent.a_attach;
    const initialAttachB = initialTangent.a_circle;
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
    const moveVector = new Vector2(0.5, 0);
    const newAttachPos = attachPos.clone().add(moveVector);
    world.getComponent(attachPoint, PositionComponent).pos.set(newAttachPos); // Update current position

    // --- Run the System ---
    system._updateAttachmentPoints(world);

    // --- Assertions ---
    // Calculate expected new tangent points
    const expectedTangent = tangentFromPointToCircle(newAttachPos, rollingPos, rollingRadius, cw);
    const expectedAttachA = expectedTangent.a_attach;
    const expectedAttachB = expectedTangent.a_circle;

    // Check attachment points
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(expectedAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(expectedAttachA.y);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(expectedAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(expectedAttachB.y);

    // Check stored length change (sB)
    const expectedSB = signedArcLengthOnWheel(
        initialAttachB.clone().subtract(rollingPos), // prevAttachB relative to prevCenterB
        expectedAttachB.clone().subtract(rollingPos), // currentAttachB relative to currentCenterB
        new Vector2(0,0), // Center offset for calculation is zero
        rollingRadius,
        cw
    );
    expect(pathComp.stored[1]).toBeCloseTo(initialStoredB - expectedSB); // stored[B] -= sB

    // Check rest length change
    // restLength += sB
    expect(jointComp.restLength).toBeCloseTo(initialRestLength + expectedSB);

    // Total rest length of path should remain constant
    const finalTotalRestLength = jointComp.restLength + pathComp.stored[0] + pathComp.stored[1];
    expect(finalTotalRestLength).toBeCloseTo(pathComp.totalRestLength);

  });

  test('_updateAttachmentPoints: Rolling to Rolling (hybrid) - Translation', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();

    // Entities
    const attach0 = world.createEntity();
    const rollingA = world.createEntity();
    const rollingB = world.createEntity();

    // Components
    const pos0 = new Vector2(-2.5, -2);
    const posA = new Vector2(-2.0, 0);
    const radiusA = 0.5;
    const cwA = true;
    const posB = new Vector2(2.0, 0);
    const radiusB = 0.5;
    const cwB = true;

    world.addComponent(attach0, new PositionComponent(pos0.x, pos0.y));
    world.addComponent(attach0, new CableLinkComponent(pos0.x, pos0.y)); // prevPos

    world.addComponent(rollingA, new PositionComponent(posA.x, posA.y));
    world.addComponent(rollingA, new RadiusComponent(radiusA));
    world.addComponent(rollingA, new CableLinkComponent(posA.x, posA.y)); // prevPos

    world.addComponent(rollingB, new PositionComponent(posB.x, posB.y));
    world.addComponent(rollingB, new RadiusComponent(radiusB));
    world.addComponent(rollingB, new CableLinkComponent(posB.x, posB.y)); // prevPos

    // Initial Tangents and joint attach->rolling
    const initialTangents_0 = tangentFromPointToCircle(pos0, posA, radiusA, cwA);
    const initialAttachA_0 = initialTangents_0.a_attach;
    const initialAttachB_0 = initialTangents_0.a_circle;
    const initialRestLength_0 = initialAttachA_0.distanceTo(initialAttachB_0);
    const jointId_0 = world.createEntity();
    const jointComp_0 = new CableJointComponent(attach0, rollingA, initialRestLength_0, initialAttachA_0, initialAttachB_0);
    world.addComponent(jointId_0, jointComp_0);

    // Initial Tangents Rolling links
    const initialTangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    const initialAttachA = initialTangents.a_circle;
    const initialAttachB = initialTangents.b_circle;
    const initialRestLength = initialAttachA.distanceTo(initialAttachB);

    // Cable Joint
    const jointId = world.createEntity();
    const jointComp = new CableJointComponent(rollingA, rollingB, initialRestLength, initialAttachA.clone(), initialAttachB.clone());
    world.addComponent(jointId, jointComp);
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(initialAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(initialAttachA.y);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(initialAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(initialAttachB.y);

    // Cable Path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointId_0, jointId], ['attachment', 'rolling', 'hybrid'], [true, cwA, cwB]);
    world.addComponent(pathId, pathComp);
    const initialStored0 = pathComp.stored[0];
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];

    // --- Simulate Movement ---
    const moveVectorA = new Vector2(0.0, 0.1); // Move A slightly up
    const newPosA = posA.clone().add(moveVectorA);
    world.getComponent(rollingA, PositionComponent).pos.set(newPosA);

    // --- Run the System ---
    system._updateAttachmentPoints(world);

    // --- Assertions ---
    // Calculate expected new tangent points
    const expectedTangents = tangentFromCircleToCircle(newPosA, radiusA, cwA, posB, radiusB, cwB);
    const expectedAttachA = expectedTangents.a_circle;
    const expectedAttachB = expectedTangents.b_circle;

    // Check attachment points
    expect(jointComp.attachmentPointA_world.x).toBeCloseTo(expectedAttachA.x);
    expect(jointComp.attachmentPointA_world.y).toBeCloseTo(expectedAttachA.y);
    expect(jointComp.attachmentPointB_world.x).toBeCloseTo(expectedAttachB.x);
    expect(jointComp.attachmentPointB_world.y).toBeCloseTo(expectedAttachB.y);

    // Check stored length changes (sA, sB)
    const expectedSA = signedArcLengthOnWheel(
        initialAttachA.clone().subtract(posA),    // prevAttachA relative to prevCenterA
        expectedAttachA.clone().subtract(newPosA), // currentAttachA relative to currentCenterA
        new Vector2(0,0), radiusA, cwA
    );

    const expectedSB = signedArcLengthOnWheel(
        initialAttachB.clone().subtract(posB), // prevAttachB relative to prevCenterB
        expectedAttachB.clone().subtract(posB), // currentAttachB relative to currentCenterB
        new Vector2(0,0), radiusB, cwB
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

  test('_updateAttachmentPoints: hybrid-attachment -> rolling -> rolling -> hybrid with small rotations', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();

    // Entities: hybrid-attachment start, two rolling links, hybrid end
    const r = 0.5;
    const startAttach = world.createEntity();
    const rollA = world.createEntity();
    const rollB = world.createEntity();
    const endHybrid = world.createEntity();
    const cwStart = true;
    const cwEnd = true;

    // Initial positions and parameters
    const posStart = new Vector2(-2.5, -2.0);
    const posA     = new Vector2(-2.0,  0.0);
    const posB     = new Vector2( 2.0,  0.0);
    const posEnd   = new Vector2( 2.5,  2.0);
    const cwA = true;
    const cwB = false;
    const smallRot = 0.1;

    // Setup positions, radii, orientations on all four entities
    [ [startAttach, posStart, r],
      [rollA, posA, r],
      [rollB, posB, r],
      [endHybrid, posEnd, r] ].forEach(([e, pos, r_], i) => {
      world.addComponent(e, new PositionComponent(pos.x, pos.y));
      world.addComponent(e, new OrientationComponent(0.0));
      world.addComponent(e, new RadiusComponent(r_));
      world.addComponent(e, new CableLinkComponent(pos.x, pos.y));
    });

    // Create joints: start->rollA, rollA->rollB, rollB->end
    const jointIds = [];
    const t0_tangent = tangentFromPointToCircle(posStart, posA, r, cwA);
    const t0_dir = t0_tangent.a_circle.clone().subtract(t0_tangent.a_attach).normalize();
    const t0_hybrid_attach = posStart.clone().add(t0_dir.scale(r));
    const t1 = tangentFromCircleToCircle(posA, r, cwA, posB, r, cwB);
    const t2 = tangentFromCircleToCircle(posB, r, cwB, posEnd, r, cwEnd);
    const tArgs = [
      [startAttach, rollA, t0_hybrid_attach, t0_tangent.a_circle],
      [rollA, rollB, t1.a_circle, t1.b_circle],
      [rollB, endHybrid, t2.a_circle, t2.b_circle],
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
    expect(firstJoint.attachmentPointB_world).toEqual(t0_tangent.a_circle);

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
    world.getComponent(startAttach, OrientationComponent).angle += smallRot;
    world.getComponent(endHybrid, OrientationComponent).angle += -smallRot;

    // Run attachment update
    system._updateAttachmentPoints(world);

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

    expect(firstJoint.attachmentPointB_world).not.toEqual(t0_tangent.a_circle);
    expect(firstJoint.attachmentPointB_world.x).toBeGreaterThan(t0_tangent.a_circle.x);
    expect(firstJoint.attachmentPointB_world.y).toBeGreaterThan(t0_tangent.a_circle.y);
    expect(firstJoint.attachmentPointB_world.distanceTo(t0_tangent.a_circle)).toBeLessThan(r*smallRot);
    expect(firstJoint.attachmentPointB_world.distanceTo(posA)).toBeCloseTo(r, 6);

    const lastJoint = world.getComponent(jointIds[jointIds.length - 1], CableJointComponent);
    expect(lastJoint.attachmentPointB_world).toEqual(t2.b_circle);
  });

  test('_updateAttachmentPoints: hybrid -> rolling -> rolling -> hybrid-attachment with small rotations', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();

    // Entities: hybrid-attachment start, two rolling links, hybrid end
    const r = 0.5;
    const startHybrid = world.createEntity();
    const rollA = world.createEntity();
    const rollB = world.createEntity();
    const endHybridAttach = world.createEntity();
    const cwStart = false;
    const cwEnd = true;

    // Initial positions and parameters
    const posStart = new Vector2(-3.0, -2.0);
    const posA     = new Vector2(-2.0,  0.0);
    const posB     = new Vector2( 2.0,  0.0);
    const posEnd   = new Vector2( 2.5,  2.0);
    const cwA = true;
    const cwB = false;
    const smallRot = 0.1;

    // Setup positions, radii, orientations on all four entities
    [ [startHybrid, posStart, r],
      [rollA, posA, r],
      [rollB, posB, r],
      [endHybridAttach, posEnd, r] ].forEach(([e, pos, r_], i) => {
      world.addComponent(e, new PositionComponent(pos.x, pos.y));
      world.addComponent(e, new OrientationComponent(0.0));
      world.addComponent(e, new RadiusComponent(r_));
      world.addComponent(e, new CableLinkComponent(pos.x, pos.y));
    });

    // Create joints: start->rollA, rollA->rollB, rollB->end
    const jointIds = [];
    // NOTE: This !cwStart is the biggest gotcha in the whole code base.
    //       cw/ccw are treated differently for hybrid links at position A.
    const t0 = tangentFromCircleToCircle(posStart, r, !cwStart, posA, r, cwA); // !cwStart because of _effectiveCW
    const t1 = tangentFromCircleToCircle(posA, r, cwA, posB, r, cwB);
    const t2_tangent = tangentFromCircleToPoint(posEnd, posB, r, cwB);
    const t2_dir = t2_tangent.a_circle.clone().subtract(t2_tangent.a_attach).normalize();
    const t2_hybrid_attach = posEnd.clone().add(t2_dir.scale(r));
    const tArgs = [
      [startHybrid, rollA, t0.a_circle, t0.b_circle],
      [rollA, rollB, t1.a_circle, t1.b_circle],
      [rollB, endHybridAttach, t2_tangent.a_circle, t2_hybrid_attach],
    ];
    tArgs.forEach(([a, b, Apt, Bpt]) => {
      const id = world.createEntity();
      world.addComponent(id, new CableJointComponent(
        a, b, Apt.distanceTo(Bpt), Apt.clone(), Bpt.clone()
      ));
      jointIds.push(id);
    });
    const lastJoint = world.getComponent(jointIds[jointIds.length - 1], CableJointComponent);
    expect(lastJoint.attachmentPointA_world).toEqual(t2_tangent.a_circle);
    expect(lastJoint.attachmentPointB_world).toEqual(t2_hybrid_attach);

    // Build the cable path
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      jointIds,
      ['hybrid', 'rolling', 'rolling', 'hybrid-attachment'],
      [cwStart, cwA, cwB, cwEnd]
    );
    expect(lastJoint.attachmentPointA_world).toEqual(t2_tangent.a_circle);
    expect(lastJoint.attachmentPointB_world).toEqual(t2_hybrid_attach);
    world.addComponent(pathId, pathComp);
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    const initialStoredA = pathComp.stored[1];
    const initialStoredB = pathComp.stored[2];
    const initialStoredEnd = pathComp.stored[3];
    expect(initialStoredEnd).toBeCloseTo(0.0, 8);

    // Rotate both hybrid ends slightly before update
    world.getComponent(startHybrid, OrientationComponent).angle += smallRot;
    world.getComponent(endHybridAttach, OrientationComponent).angle += -smallRot;

    // Run attachment update
    system._updateAttachmentPoints(world);

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

    expect(lastJoint.attachmentPointA_world).not.toEqual(t2_tangent.a_circle);
    expect(lastJoint.attachmentPointA_world.x).toBeLessThan(t2_tangent.a_circle.x);
    expect(lastJoint.attachmentPointA_world.y).toBeGreaterThan(t2_tangent.a_circle.y);
    expect(lastJoint.attachmentPointA_world.distanceTo(t2_tangent.a_circle)).toBeLessThan(r*smallRot);
    expect(lastJoint.attachmentPointA_world.distanceTo(posB)).toBeCloseTo(r, 6);

    expect(firstJoint.attachmentPointA_world).toEqual(t0.a_circle);
  });
});
