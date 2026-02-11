import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  BallTagComponent,
  BorderComponent,
  FlipperTagComponent,
  FlipperStateComponent,
  PBDUnifiedContactManifoldSystem,
} from '../../../examples/js/flipper/flipper_common.js';

describe('PBDUnifiedContactManifoldSystem', () => {
  test('resolves border and ball-ball contacts in one unified pass', () => {
    const world = new World();
    world.setResource('enableLayering', true);
    world.setResource('layeringCollisionSectorSolvers', true);

    const borderId = world.createEntity();
    world.addComponent(
      borderId,
      new BorderComponent([
        new Vector2(-3.0, 0.0),
        new Vector2(3.0, 0.0),
      ])
    );

    const ballA = world.createEntity();
    world.addComponent(ballA, new BallTagComponent());
    world.addComponent(ballA, new PositionComponent(0.0, 0.95));
    world.addComponent(ballA, new RadiusComponent(1.0));
    world.addComponent(ballA, new MassComponent(1.0));

    const ballB = world.createEntity();
    world.addComponent(ballB, new BallTagComponent());
    world.addComponent(ballB, new PositionComponent(1.8, 0.95));
    world.addComponent(ballB, new RadiusComponent(1.0));
    world.addComponent(ballB, new MassComponent(1.0));

    const system = new PBDUnifiedContactManifoldSystem();
    system.update(world, 0.016);

    const borderContacts = world.getResource('ball_border_contacts');
    const ballBallContacts = world.getResource('ball_ball_contacts');
    expect(Array.isArray(borderContacts)).toBe(true);
    expect(Array.isArray(ballBallContacts)).toBe(true);
    expect(borderContacts.length).toBeGreaterThanOrEqual(2);
    expect(ballBallContacts.length).toBeGreaterThanOrEqual(1);

    const posA = world.getComponent(ballA, PositionComponent).pos;
    const posB = world.getComponent(ballB, PositionComponent).pos;
    expect(posA.y).toBeGreaterThan(0.95);
    expect(posB.y).toBeGreaterThan(0.95);
    expect(posA.x).toBeLessThan(0.0);
    expect(posB.x).toBeGreaterThan(1.8);
  });

  test('resolves simultaneous flipper and ball-ball contacts', () => {
    const world = new World();
    world.setResource('enableLayering', true);
    world.setResource('layeringCollisionSectorSolvers', true);

    const flipperId = world.createEntity();
    world.addComponent(flipperId, new FlipperTagComponent());
    world.addComponent(flipperId, new PositionComponent(0.0, 0.0));
    world.addComponent(flipperId, new RadiusComponent(0.05));
    world.addComponent(flipperId, new FlipperStateComponent(1.0, 0.0, 1.0, 0.0));

    const ballA = world.createEntity();
    world.addComponent(ballA, new BallTagComponent());
    world.addComponent(ballA, new PositionComponent(0.50, 0.22));
    world.addComponent(ballA, new RadiusComponent(0.2));
    world.addComponent(ballA, new MassComponent(1.0));

    const ballB = world.createEntity();
    world.addComponent(ballB, new BallTagComponent());
    world.addComponent(ballB, new PositionComponent(0.82, 0.22));
    world.addComponent(ballB, new RadiusComponent(0.2));
    world.addComponent(ballB, new MassComponent(1.0));

    const initialDistance = world
      .getComponent(ballA, PositionComponent)
      .pos
      .distanceTo(world.getComponent(ballB, PositionComponent).pos);

    const system = new PBDUnifiedContactManifoldSystem();
    system.update(world, 0.016);

    const flipperContacts = world.getResource('ball_flipper_contacts');
    const ballBallContacts = world.getResource('ball_ball_contacts');
    expect(Array.isArray(flipperContacts)).toBe(true);
    expect(Array.isArray(ballBallContacts)).toBe(true);
    expect(flipperContacts.length).toBeGreaterThanOrEqual(1);
    expect(ballBallContacts.length).toBeGreaterThanOrEqual(1);

    const posA = world.getComponent(ballA, PositionComponent).pos;
    const posB = world.getComponent(ballB, PositionComponent).pos;
    const finalDistance = posA.distanceTo(posB);
    expect(posA.y).toBeGreaterThan(0.22);
    expect(finalDistance).toBeGreaterThan(initialDistance);
  });
});
