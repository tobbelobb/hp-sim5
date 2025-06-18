import Vector2 from '../cable_joints/vector2.js';

import {
  World,
  PositionComponent,
  RadiusComponent
} from '../cable_joints/ecs.js';

import {
  signedArcLengthOnWheel
} from '../cable_joints/geometry.js';

import {
  CableJointComponent,
  CablePathComponent,
} from '../cable_joints/cable_joints_core.js';

describe('CablePathComponent constructor', () => {
  test('initial stored lengths and totalRestLength are computed correctly', () => {
    const world = new World();
    // Entities: start, rolling center, end
    const start = world.createEntity();
    const center = world.createEntity();
    const end = world.createEntity();
    world.addComponent(center, new PositionComponent(0, 0));
    const radius = 1.0;
    world.addComponent(center, new RadiusComponent(radius));

    // Create two joints: start->center and center->end
    const restLen1 = 1.5;
    const restLen2 = 2.0;
    const pointB1 = new Vector2(1, 0); // on circle at angle 0
    const pointA2 = new Vector2(0, 1); // on circle at π/2
    // Joint from start to center: both attach points at pointB1
    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        start,
        center,
        restLen1,
        pointB1.clone(),
        pointB1.clone()
      )
    );
    // Joint from center to end: both attach points at pointA2
    const joint2 = world.createEntity();
    world.addComponent(
      joint2,
      new CableJointComponent(
        center,
        end,
        restLen2,
        pointA2.clone(),
        pointA2.clone()
      )
    );

    const linkTypes = ['attachment', 'rolling', 'attachment'];
    const cw = [false, false, false];
    const path = new CablePathComponent(world, [joint1, joint2], linkTypes, cw);

    // Expected stored: [0, arc length from (1,0) to (0,1) on circle radius 1, 0]
    const expectedArc = signedArcLengthOnWheel(
      pointB1,
      pointA2,
      new Vector2(0, 0),
      radius,
      false,
      true
    );
    expect(path.stored).toHaveLength(3);
    expect(path.stored[0]).toBeCloseTo(0.0);
    expect(path.stored[1]).toBeCloseTo(expectedArc);
    expect(path.stored[2]).toBeCloseTo(0.0);

    // totalRestLength = restLen1 + restLen2 + expectedArc
    expect(path.totalRestLength).toBeCloseTo(restLen1 + restLen2 + expectedArc);
  });
});
