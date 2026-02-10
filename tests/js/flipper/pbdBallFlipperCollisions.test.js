import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  BallTagComponent,
  FlipperTagComponent,
  FlipperStateComponent,
  PBDBallFlipperCollisions,
  FlipperCircleSectorCollisions,
  OverlayRadiusComponent,
  CircleSectorComponent,
} from '../../../examples/js/flipper/flipper_common.js';

function makeWorld() {
  const world = new World();

  const ballId = world.createEntity();
  world.addComponent(ballId, new BallTagComponent());
  world.addComponent(ballId, new PositionComponent(0.5, 1.25));
  world.addComponent(ballId, new RadiusComponent(1.0));
  world.addComponent(ballId, new MassComponent(1.0));

  const flipperId = world.createEntity();
  world.addComponent(flipperId, new FlipperTagComponent());
  world.addComponent(flipperId, new PositionComponent(0.0, 0.0));
  world.addComponent(flipperId, new RadiusComponent(0.1));
  world.addComponent(flipperId, new FlipperStateComponent(1.0, 0.0, 1.0, 0.0));

  return { world, ballId, flipperId };
}

describe('PBDBallFlipperCollisions with overlay geometry', () => {
  test('overlay radius can create a flipper collision when raw radius does not', () => {
    const withoutOverlay = makeWorld();
    const withOverlay = makeWorld();
    withOverlay.world.addComponent(withOverlay.ballId, new OverlayRadiusComponent(1.2));

    const system = new PBDBallFlipperCollisions();
    system.update(withoutOverlay.world, 0.016);
    system.update(withOverlay.world, 0.016);

    const contactsWithout = withoutOverlay.world.getResource('ball_flipper_contacts') || [];
    const contactsWith = withOverlay.world.getResource('ball_flipper_contacts') || [];

    expect(contactsWithout).toHaveLength(0);
    expect(contactsWith).toHaveLength(1);
    expect(contactsWith[0].ball_contact_radius).toBeCloseTo(1.2, 9);
  });

  test('circle sector applies only in matching direction', () => {
    const worldA = makeWorld();
    worldA.world.getComponent(worldA.ballId, PositionComponent).pos.set({ x: 0.0, y: 0.0 });
    worldA.world.getComponent(worldA.flipperId, PositionComponent).pos.set({ x: 2.35, y: -1.0 });
    worldA.world.getComponent(worldA.flipperId, FlipperStateComponent).length = 2.0;
    worldA.world.getComponent(worldA.flipperId, FlipperStateComponent).restAngle = Math.PI / 2.0;
    worldA.world.addComponent(
      worldA.ballId,
      new CircleSectorComponent([{
        radius: 2.3,
        startAngle: -0.2,
        endAngle: 0.2,
        cw: false
      }])
    );

    const worldB = makeWorld();
    worldB.world.getComponent(worldB.ballId, PositionComponent).pos.set({ x: 0.0, y: 0.0 });
    worldB.world.getComponent(worldB.flipperId, PositionComponent).pos.set({ x: 2.35, y: -1.0 });
    worldB.world.getComponent(worldB.flipperId, FlipperStateComponent).length = 2.0;
    worldB.world.getComponent(worldB.flipperId, FlipperStateComponent).restAngle = Math.PI / 2.0;
    worldB.world.addComponent(
      worldB.ballId,
      new CircleSectorComponent([{
        radius: 2.3,
        startAngle: Math.PI - 0.2,
        endAngle: Math.PI + 0.2,
        cw: false
      }])
    );

    const system = new PBDBallFlipperCollisions();
    const sectorSystem = new FlipperCircleSectorCollisions();
    system.update(worldA.world, 0.016);
    sectorSystem.update(worldA.world, 0.016);
    system.update(worldB.world, 0.016);
    sectorSystem.update(worldB.world, 0.016);

    const contactsA = worldA.world.getResource('ball_flipper_contacts') || [];
    const contactsB = worldB.world.getResource('ball_flipper_contacts') || [];
    expect(contactsA).toHaveLength(1);
    expect(contactsB).toHaveLength(0);
  });
});
