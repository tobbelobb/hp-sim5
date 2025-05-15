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
  _splitJoints
} from '../cable_joints/cable_joints_core.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel
} from '../cable_joints/geometry.js';


describe('_splitJoints', () => {
  test('_splitJoints does nothing for a single-joint path that misses every wheel', () => {
    const world  = new World();

    // Anchors five units apart; the wheel is off to the side - the segment
    //  between anchors never intersects it.
    const A = world.createEntity();
    const B = world.createEntity();
    const wheel = world.createEntity();

    world.addComponent(A, new PositionComponent(0, 0));
    world.addComponent(B, new PositionComponent(5, 0));
    world.addComponent(wheel, new PositionComponent(0, 5));  // far above
    world.addComponent(wheel, new RadiusComponent(1.5));

    const joint = world.createEntity();
    world.addComponent(
      joint,
      new CableJointComponent(A, B, 5, new Vector2(0, 0), new Vector2(5, 0)),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],                      // only one joint
      ['attachment', 'attachment'], // no rolling link
      [false, false],
    );
    world.addComponent(path, pathComp);

    _splitJoints(world);

    // Nothing should change
    expect(pathComp.jointEntities.length).toBe(1);
    expect(pathComp.jointEntities[0]).toBe(joint);
    expect(world.getComponent(joint, CableJointComponent)).toBeDefined();
  });

  test('_splitJoints does not duplicate joints when rope already bends over a wheel', () => {
    const world  = new World();

    // Rope is ALREADY split: AnchorL - Wheel - AnchorR
    const L = world.createEntity();
    const R = world.createEntity();
    const wheel = world.createEntity();

    world.addComponent(L, new PositionComponent(-4, 2));
    world.addComponent(R, new PositionComponent(4, 2));
    world.addComponent(wheel, new PositionComponent(0, 0));
    world.addComponent(wheel, new RadiusComponent(1.5));

    // Pre-compute tangents so the rope genuinely touches the wheel
    const tangL = tangentFromPointToCircle(new Vector2(-4, 2), new Vector2(0, 0), 1.5, false);
    const tangR = tangentFromCircleToPoint(new Vector2(4, 2), new Vector2(0, 0), 1.5, true);

    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(L, wheel,
        tangL.a_circle.distanceTo(tangL.a_attach),
        tangL.a_attach, tangL.a_circle),
    );

    const joint2 = world.createEntity();
    world.addComponent(
      joint2,
      new CableJointComponent(wheel, R,
        tangR.a_circle.distanceTo(tangR.a_attach),
        tangR.a_circle, tangR.a_attach),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint1, joint2],
      ['attachment', 'rolling', 'attachment'],
      [false, true, true],
    );
    world.addComponent(path, pathComp);

    _splitJoints(world);

    // The path must still have exactly two joints - no dupes
    expect(pathComp.jointEntities.length).toBe(2);
    expect(pathComp.jointEntities).toEqual(expect.arrayContaining([joint1, joint2]));
  });

  test('_splitJoints inserts a wheel joint when a straight segment intersects the wheel', () => {
    const world  = new World();

    // Anchors on opposite sides of a wheel, but the rope is *still* a single
    //  straight joint.  The line AB pierces the wheel’s circle => must split.
    const A = world.createEntity();
    const B = world.createEntity();
    const wheel = world.createEntity();
    world.addComponent(A, new PositionComponent(-4, 0));
    world.addComponent(B, new PositionComponent(4, 0));
    world.addComponent(wheel, new PositionComponent(0, 0));
    world.addComponent(wheel, new CableLinkComponent(0, 0));
    world.addComponent(wheel, new RadiusComponent(1.5));

    // One straight joint for the entire rope
    const joint = world.createEntity();
    world.addComponent(
      joint,
      new CableJointComponent(A, B, 8, new Vector2(-4, 0), new Vector2(4, 0)),
    );
    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['attachment', 'attachment'],
      [false, false],
    );
    world.addComponent(path, pathComp);

    _splitJoints(world);

    // Expect the path to have grown to two joints (Anchor-Wheel, Wheel-Anchor)
    expect(pathComp.jointEntities.length).toBe(2);

    // Find new joint entity that wasn’t in the original single-joint array
    const newJointId = pathComp.jointEntities.find(id => id !== joint);
    const newJoint = world.getComponent(newJointId, CableJointComponent);
    const oldJoint = world.getComponent(joint, CableJointComponent);

    // One of them must terminate / originate at the wheel
    expect([oldJoint.entityA, oldJoint.entityB, newJoint.entityA, newJoint.entityB])
      .toContain(wheel);

    // Link-type list should now contain 'rolling' in the middle
    expect(pathComp.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);

    // Stored arc length around the wheel must be non-negative
    expect(pathComp.stored[1]).toBeGreaterThanOrEqual(0);
  });

  test('_splitJoints creates three joints when a straight segment intersects two wheels', () => {
    // Requires reRunSplit-style logic inside _splitJoints,
    // analogous to the reRunMerge branch you exercised earlier.

    const world  = new World();

    const A = world.createEntity();
    const B = world.createEntity();
    const wheel1 = world.createEntity();
    const wheel2 = world.createEntity();

    // Geometry: wheels sit below the rope so AB crosses both circles
    const aPos = new Vector2(-4,  0);
    const bPos = new Vector2( 4,  0);
    const w1Pos = new Vector2(-1.5, -0.75);
    const w2Pos = new Vector2( 1.5,  0.75);
    const r = 0.9;

    world.addComponent(A, new PositionComponent(aPos.x, aPos.y));
    world.addComponent(B, new PositionComponent(bPos.x, bPos.y));

    world.addComponent(wheel1, new PositionComponent(w1Pos.x, w1Pos.y));
    world.addComponent(wheel1, new CableLinkComponent(w1Pos.x, w1Pos.y));
    world.addComponent(wheel1, new RadiusComponent(r));

    world.addComponent(wheel2, new PositionComponent(w2Pos.x, w2Pos.y));
    world.addComponent(wheel2, new CableLinkComponent(w2Pos.x, w2Pos.y));
    world.addComponent(wheel2, new RadiusComponent(r));

    // One joint: A - B
    const joint = world.createEntity();
    world.addComponent(
      joint,
      new CableJointComponent(A, B,
        aPos.distanceTo(bPos), aPos.clone(), bPos.clone()),
    );

    const path = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [joint],
      ['attachment', 'attachment'],
      [false, false],
    );
    world.addComponent(path, pathComp);

    _splitJoints(world);

    // Path should now have three joints (A-W1, W1-W2, W2-B)
    expect(pathComp.jointEntities.length).toBe(3);

    // Collect the new joints and verify each wheel appears exactly once
    const comps = pathComp.jointEntities.map(id =>
      world.getComponent(id, CableJointComponent));

    const wheelOccurrences = comps.flatMap(c => [c.entityA, c.entityB])
      .filter(id => id === wheel1 || id === wheel2);

    expect(wheelOccurrences.filter(id => id === wheel1).length).toBe(2); // in/out
    expect(wheelOccurrences.filter(id => id === wheel2).length).toBe(2); // in/out

    // Link types now: attachment - rolling - rolling - attachment
    expect(pathComp.linkTypes).toEqual(['attachment', 'rolling', 'rolling', 'attachment']);

    // Stored arc lengths: two positive values (one per wheel), no negatives
    expect(pathComp.stored.length).toBe(4);
    expect(pathComp.stored[1]).toBeGreaterThanOrEqual(0);
    expect(pathComp.stored[2]).toBeGreaterThanOrEqual(0);
  });
});
