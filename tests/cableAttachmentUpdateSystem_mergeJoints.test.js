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


describe('_mergeJoints', () => {
  test('_mergeJoints does nothing for a single-joint path', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();
    // Setup one cable joint between two anchors (no wheel involved)
    const anchorA = world.createEntity();
    const anchorB = world.createEntity();
    world.addComponent(anchorA, new PositionComponent(0, 0));
    world.addComponent(anchorB, new PositionComponent(5, 0));
    // Create a CableJointComponent connecting anchorA to anchorB
    const jointId = world.createEntity();
    const restLen = 5.0;
    const attachComp = new CableJointComponent(anchorA, anchorB, restLen, new Vector2(0,0), new Vector2(5,0));
    world.addComponent(jointId, attachComp);
    // CablePath with a single joint (both link types are 'attachment')
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [jointId], ['attachment','attachment'], [false, false]);
    world.addComponent(pathId, pathComp);

    system._mergeJoints(world);
    expect(pathComp.jointEntities.length).toBe(1);            // still one joint
    expect(pathComp.jointEntities[0]).toBe(jointId);          // same joint remains
    expect(world.getComponent(jointId, CableJointComponent)).toBeDefined(); // joint still exists
  });


  test('_mergeJoints does not merge joints if wheel contact is still needed', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();
    // Setup: Anchors on opposite sides of a wheel
    const anchorL = world.createEntity();
    const anchorR = world.createEntity();
    const wheel   = world.createEntity();
    world.addComponent(anchorL, new PositionComponent(-4, 2));    // left anchor
    world.addComponent(anchorR, new PositionComponent(4, 2));     // right anchor
    world.addComponent(wheel,   new PositionComponent(0, 0));
    world.addComponent(wheel,   new RadiusComponent(1.5));
    // Calculate initial tangent contacts for each anchor (left and right side of wheel)
    const tangL = tangentFromPointToCircle(new Vector2(-4,2), new Vector2(0,0), 1.5, /*cw=*/false);
    const tangR = tangentFromPointToCircle(new Vector2(4,2),  new Vector2(0,0), 1.5, /*cw=*/true);
    const contactL = tangL.a_circle;  // point on wheel for left anchor
    const contactR = tangR.a_circle;  // point on wheel for right anchor
    const distL = contactL.distanceTo(tangL.a_attach);  // length of left segment
    const distR = contactR.distanceTo(tangR.a_attach);  // length of right segment
    // Joint 1: AnchorL to Wheel (left side contact)
    const joint1 = world.createEntity();
    world.addComponent(joint1, new CableJointComponent(anchorL, wheel, distL, tangL.a_attach.clone(), contactL.clone()));
    // Joint 2: Wheel to AnchorR (right side contact)
    const joint2 = world.createEntity();
    world.addComponent(joint2, new CableJointComponent(wheel, anchorR, distR, contactR.clone(), tangR.a_attach.clone()));
    // Path linking the two joints (rope goes around wheel)
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [joint1, joint2], ['attachment','rolling','attachment'], [false, /*cwArc=*/true, true]);
    world.addComponent(pathId, pathComp);

    // No movement; wheel contact remains valid
    system._mergeJoints(world);
    expect(pathComp.jointEntities.length).toBe(2);         // still two joints
    expect(pathComp.jointEntities).toContain(joint1);
    expect(pathComp.jointEntities).toContain(joint2);
    // Both joint components still exist and reference the wheel
    const j1Comp = world.getComponent(joint1, CableJointComponent);
    const j2Comp = world.getComponent(joint2, CableJointComponent);
    expect(j1Comp).toBeDefined();
    expect(j2Comp).toBeDefined();
    expect([j1Comp.entityA, j1Comp.entityB]).toContain(wheel);
    expect([j2Comp.entityA, j2Comp.entityB]).toContain(wheel);
  });


  test('_mergeJoints merges two joints into one when the cable detaches from a wheel', () => {
    const world = new World();
    const system = new CableAttachmentUpdateSystem();
    // Initial setup: rope over a wheel (same as above)
    // Anchors on opposite sides of a wheel
    const anchorL = world.createEntity();
    const anchorR = world.createEntity();
    const wheel   = world.createEntity();
    const lPos = new Vector2(-4, 2);
    const rPos = new Vector2(4, 2);
    const r = 1.5;
    world.addComponent(anchorL, new PositionComponent(lPos.x, lPos.y));
    world.addComponent(anchorL, new CableLinkComponent(lPos.x, lPos.y));
    world.addComponent(anchorR, new PositionComponent(rPos.x, rPos.y));
    world.addComponent(anchorR, new CableLinkComponent(rPos.x, rPos.y));
    world.addComponent(wheel,   new PositionComponent(0, 0));
    world.addComponent(wheel,   new CableLinkComponent(0, 0));
    world.addComponent(wheel,   new RadiusComponent(r));
    // Compute initial contacts and create two joints (anchorL–wheel and wheel–anchorR)
    const tangL = tangentFromPointToCircle(lPos, new Vector2(0,0), r, true);
    const tangR = tangentFromCircleToPoint(rPos,  new Vector2(0,0), r, true);
    const contactL = tangL.a_circle;
    const contactR = tangR.a_circle;
    const distL = contactL.distanceTo(tangL.a_attach);
    const distR = contactR.distanceTo(tangR.a_attach);
    const joint1 = world.createEntity();
    world.addComponent(joint1, new CableJointComponent(anchorL, wheel, distL, tangL.a_attach.clone(), contactL.clone()));
    const joint2 = world.createEntity();
    world.addComponent(joint2, new CableJointComponent(wheel, anchorR, distR, contactR.clone(), tangR.a_attach.clone()));
    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(world, [joint1, joint2], ['attachment','rolling','attachment'], [false, true, true]);
    world.addComponent(pathId, pathComp);
    // Expect first stored calculation to wrap from joint1B to jonit2A since CablePathComponent's constructor
    // enforces positive stored-values in the 0 to 1 wraps range.
    expect(pathComp.stored[1]).toBeGreaterThan(r*Math.PI);
    expect(pathComp.stored[1]).toBeLessThan(2.0*r*Math.PI);
    pathComp.stored[1] -= r*2.0*Math.PI;
    pathComp.totalRestLength -= r*2.0*Math.PI;
    expect(pathComp.stored[1]).toBeLessThan(0.0);
    expect(pathComp.totalRestLength).toBeCloseTo(distL + distL + pathComp.stored[1]);
    const initialTotalRestLength = pathComp.totalRestLength;

    // Now perform merging of joints if wheel contact dropped
    system._mergeJoints(world);

    // totalRestLength should not be altered by _mergeJoints or any other function.
    expect(pathComp.totalRestLength).toBe(initialTotalRestLength);

    // After merge, expect only one joint remains in the path
    expect(pathComp.jointEntities.length).toBe(1);
    const remainingJointId = pathComp.jointEntities[0];
    const remainingJoint = world.getComponent(remainingJointId, CableJointComponent);
    // The remaining joint should connect anchorL and anchorR directly
    expect([remainingJoint.entityA, remainingJoint.entityB]).toContain(anchorL);
    expect([remainingJoint.entityA, remainingJoint.entityB]).toContain(anchorR);
    expect([remainingJoint.entityA, remainingJoint.entityB]).not.toContain(wheel);
    // The removed joint (either joint1 or joint2) should no longer be in the world or path
    const removedJointId = (remainingJointId === joint1) ? joint2 : joint1;
    expect(pathComp.jointEntities).not.toContain(removedJointId);
    expect(world.getComponent(removedJointId, CableJointComponent)).toBeUndefined();
    // Total rope length is preserved
    expect(pathComp.stored[0]).toBeCloseTo(0.0, 8);
    expect(pathComp.stored[1]).toBeCloseTo(0.0, 8);
    expect(remainingJoint.restLength).toBeCloseTo(initialTotalRestLength, 8);

    // Link types should now indicate no rolling segment
    expect(pathComp.linkTypes).toEqual(['attachment', 'attachment']);
  });
});
