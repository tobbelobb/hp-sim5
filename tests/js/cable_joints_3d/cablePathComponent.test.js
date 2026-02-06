import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

import {
  World,
  PositionComponent,
  RadiusComponent
} from '../../../src/js/cable_joints_3d/ecs.js';

import {
  signedArcLengthOnWheel
} from '../../../src/js/cable_joints_3d/geometry3.js';

import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

describe('CablePathComponent constructor (3D planar)', () => {
  test('initial stored lengths and totalRestLength are computed correctly', () => {
    const world = new World();
    const start = world.createEntity();
    const center = world.createEntity();
    const end = world.createEntity();
    world.addComponent(center, new PositionComponent(0, 0, 0));
    const radius = 1.0;
    world.addComponent(center, new RadiusComponent(radius));

    const restLen1 = 1.5;
    const restLen2 = 2.0;
    const pointB1 = new Vector3(1, 0, 0);
    const pointA2 = new Vector3(0, 1, 0);

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

    const expectedArc = signedArcLengthOnWheel(
      pointB1,
      pointA2,
      new Vector3(0, 0, 0),
      radius,
      false,
      new Vector3(0, 0, 1),
      true
    );
    expect(path.stored).toHaveLength(3);
    expect(path.stored[0]).toBeCloseTo(0.0);
    expect(path.stored[1]).toBeCloseTo(expectedArc);
    expect(path.stored[2]).toBeCloseTo(0.0);

    expect(path.totalRestLength).toBeCloseTo(restLen1 + restLen2 + expectedArc);
  });
});
