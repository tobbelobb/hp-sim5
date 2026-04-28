import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { CableSlackSystem } from '../../../src/js/cable_joints_3d/cable_slack_system.js';

describe('CableSlackSystem (3D)', () => {
  test('equalizes tension across pinholes even when neither side has spare slack', () => {
    const world = new World();

    const attachment = world.createEntity();
    world.addComponent(attachment, new PositionComponent(0.0, 0.0, 0.0));

    const pinhole = world.createEntity();
    world.addComponent(pinhole, new PositionComponent(10.0, 0.0, 0.0));

    const rolling = world.createEntity();
    world.addComponent(rolling, new PositionComponent(22.0, 0.0, 0.0));

    const joint0 = world.createEntity();
    world.addComponent(
      joint0,
      new CableJointComponent(
        attachment,
        pinhole,
        10.0,
        new Vector3(0.0, 0.0, 0.0),
        new Vector3(10.0, 0.0, 0.0),
      ),
    );

    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        pinhole,
        rolling,
        10.0,
        new Vector3(10.0, 0.0, 0.0),
        new Vector3(22.0, 0.0, 0.0),
      ),
    );

    const path = world.createEntity();
    world.addComponent(
      path,
      new CablePathComponent(
        world,
        [joint0, joint1],
        ['attachment', 'pinhole', 'rolling'],
        [true, true, true],
      ),
    );

    new CableSlackSystem().update(world, 1.0 / 60.0);

    const left = world.getComponent(joint0, CableJointComponent);
    const right = world.getComponent(joint1, CableJointComponent);

    expect(left.restLength + right.restLength).toBeCloseTo(20.0, 12);
    expect(left.restLength).toBeCloseTo(20.0 * 10.0 / 22.0, 12);
    expect(right.restLength).toBeCloseTo(20.0 * 12.0 / 22.0, 12);
    expect(10.0 / left.restLength).toBeCloseTo(12.0 / right.restLength, 12);
  });
});
