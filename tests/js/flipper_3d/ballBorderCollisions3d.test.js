import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  BallTagComponent,
  BorderComponent,
  PBDBallBorderCollisions
} from '../../../examples/js/flipper_3d/flipper_common_3d.js';

describe('PBDBallBorderCollisions (3D)', () => {
  test('keeps ball on simulation plane and emits planar contact normals', () => {
    const world = new World();

    const border = world.createEntity();
    world.addComponent(
      border,
      new BorderComponent([
        new Vector3(0, 0, 0),
        new Vector3(1, 0, 0),
        new Vector3(1, 1, 0),
        new Vector3(0, 1, 0)
      ])
    );

    const ball = world.createEntity();
    world.addComponent(ball, new BallTagComponent());
    world.addComponent(ball, new PositionComponent(-0.01, 0.5, 0.42));
    world.addComponent(ball, new RadiusComponent(0.05));
    world.addComponent(ball, new MassComponent(1.0));

    const system = new PBDBallBorderCollisions();
    system.update(world, 1 / 120);

    const pos = world.getComponent(ball, PositionComponent).pos;
    const contacts = world.getResource('ball_border_contacts');

    expect(pos.z).toBeCloseTo(0.0, 12);
    expect(contacts.length).toBe(1);
    expect(contacts[0].normal.z).toBeCloseTo(0.0, 12);
  });
});
