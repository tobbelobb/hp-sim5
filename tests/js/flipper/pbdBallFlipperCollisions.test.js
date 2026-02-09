import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';
import {
  BallTagComponent,
  FlipperTagComponent,
  FlipperStateComponent,
  PBDBallFlipperCollisions,
} from '../../../examples/js/flipper/flipper_common.js';

function createWorldWithFlipperContactCandidate(storedOnHybrid) {
  const world = new World();

  const ballId = world.createEntity();
  world.addComponent(ballId, new BallTagComponent());
  world.addComponent(ballId, new PositionComponent(0.5, 1.25));
  world.addComponent(ballId, new RadiusComponent(1.0));
  world.addComponent(ballId, new MassComponent(1.0));
  world.addComponent(ballId, new CableLinkComponent(0.5, 1.25));

  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(3.0, 1.25));
  world.addComponent(anchorId, new RadiusComponent(0.1));
  world.addComponent(anchorId, new CableLinkComponent(3.0, 1.25));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      ballId,
      anchorId,
      0.0,
      new Vector2(1.5, 1.25),
      new Vector2(2.9, 1.25)
    )
  );

  const pathId = world.createEntity();
  world.addComponent(
    pathId,
    new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [true, true],
      1e4,
      [storedOnHybrid, 0.0],
      0.1
    )
  );

  const flipperId = world.createEntity();
  world.addComponent(flipperId, new FlipperTagComponent());
  world.addComponent(flipperId, new PositionComponent(0.0, 0.0));
  world.addComponent(flipperId, new RadiusComponent(0.1));
  world.addComponent(flipperId, new FlipperStateComponent(1.0, 0.0, 1.0, 0.0));

  return { world, ballId };
}

describe('PBDBallFlipperCollisions wrapped hybrid radius', () => {
  test('uses endpoint wrap expansion when colliding against flipper segment', () => {
    const baseLayerCircumference = 2.0 * Math.PI * 1.1;
    const withWrap = createWorldWithFlipperContactCandidate(baseLayerCircumference);
    const withoutWrap = createWorldWithFlipperContactCandidate(0.0);

    const system = new PBDBallFlipperCollisions();
    system.update(withoutWrap.world, 0.016);
    system.update(withWrap.world, 0.016);

    const withoutWrapContacts = withoutWrap.world.getResource('ball_flipper_contacts');
    const withWrapContacts = withWrap.world.getResource('ball_flipper_contacts');

    expect(withoutWrapContacts).toHaveLength(0);
    expect(withWrapContacts).toHaveLength(1);

    const yWithoutWrap = withoutWrap.world.getComponent(withoutWrap.ballId, PositionComponent).pos.y;
    const yWithWrap = withWrap.world.getComponent(withWrap.ballId, PositionComponent).pos.y;
    expect(yWithoutWrap).toBeCloseTo(1.25, 9);
    expect(yWithWrap).toBeGreaterThan(1.25);
    expect(yWithWrap).toBeCloseTo(1.30, 6);
  });
});
