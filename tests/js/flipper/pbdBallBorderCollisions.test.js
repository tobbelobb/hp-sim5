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
  PBDUnifiedContactManifoldSystem,
  OverlayRadiusComponent,
  CircleSectorComponent,
} from '../../../example_apps/js/flipper/flipper_common.js';

function _makeBorderWorld() {
  const world = new World();
  world.setResource('enableLayering', true);
  world.setResource('layeringCollisionSectorSolvers', true);
  world.setResource('layeringCollisionCircleSectors', true);
  world.setResource('layeringCollisionOverlayRadius', true);
  return world;
}

describe('PBDUnifiedContactManifoldSystem border contact behavior', () => {
  test('detects border contact when overlay radius exceeds raw radius', () => {
    const makeWorld = (withWrap) => {
      const world = _makeBorderWorld();

      const ballId = world.createEntity();
      world.addComponent(ballId, new BallTagComponent());
      world.addComponent(ballId, new PositionComponent(0.0, 1.15));
      world.addComponent(ballId, new RadiusComponent(1.0));
      world.addComponent(ballId, new MassComponent(1.0));

      const borderId = world.createEntity();
      world.addComponent(
        borderId,
        new BorderComponent([
          new Vector2(-2.0, 0.0),
          new Vector2(2.0, 0.0),
        ])
      );

      if (withWrap) {
        world.addComponent(ballId, new OverlayRadiusComponent(1.2));
      }
      return { world, ballId };
    };

    const noWrap = makeWorld(false);
    const withWrap = makeWorld(true);
    const system = new PBDUnifiedContactManifoldSystem();

    system.update(noWrap.world, 0.016);
    system.update(withWrap.world, 0.016);

    const noWrapContacts = noWrap.world.getResource('ball_border_contacts');
    const withWrapContacts = withWrap.world.getResource('ball_border_contacts');
    expect(noWrapContacts).toHaveLength(0);
    expect(withWrapContacts).toHaveLength(1);

    const yNoWrap = noWrap.world.getComponent(noWrap.ballId, PositionComponent).pos.y;
    const yWithWrap = withWrap.world.getComponent(withWrap.ballId, PositionComponent).pos.y;
    expect(yNoWrap).toBeCloseTo(1.15, 9);
    expect(yWithWrap).toBeGreaterThan(1.15);
    expect(yWithWrap).toBeCloseTo(1.2, 6);
  });

  test('keeps simultaneous raw and sector border contacts on different segments', () => {
    const world = _makeBorderWorld();
    world.setResource('layeringCollisionOverlayRadius', false);

    const ballId = world.createEntity();
    world.addComponent(ballId, new BallTagComponent());
    world.addComponent(ballId, new PositionComponent(-1.05, -0.95));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(
      ballId,
      new CircleSectorComponent(
        1.2,
        -0.85 * Math.PI,
        -0.15 * Math.PI,
        false
      )
    );

    const borderId = world.createEntity();
    world.addComponent(
      borderId,
      new BorderComponent([
        new Vector2(-2.0, -2.0),
        new Vector2(2.0, -2.0),
        new Vector2(2.0, 2.0),
        new Vector2(-2.0, 2.0),
      ])
    );

    const system = new PBDUnifiedContactManifoldSystem();
    system.update(world, 0.016);

    const contacts = world.getResource('ball_border_contacts');
    expect(Array.isArray(contacts)).toBe(true);
    expect(contacts.length).toBeGreaterThanOrEqual(2);

    const ballContacts = contacts.filter((c) => c.ball_id === ballId);
    expect(ballContacts.length).toBeGreaterThanOrEqual(2);
    expect(ballContacts.some((c) => c.ball_contact_radius >= 1.1)).toBe(true);
    expect(ballContacts.some((c) => c.normal.x > 0.5)).toBe(true);
    expect(ballContacts.some((c) => c.normal.y > 0.5)).toBe(true);
  });
});
