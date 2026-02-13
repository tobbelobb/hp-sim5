import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  RadiusComponent,
  MassComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';
import { LayerContactStaticFrictionSystem } from '../../../examples/js/flipper/layer_contact_static_friction_system.js';
import {
  FlipperStateComponent,
  FlipperTipComponent,
} from '../../../examples/js/flipper/flipper_common.js';

function makeWorld() {
  const world = new World();

  const ballId = world.createEntity();
  world.addComponent(ballId, new PositionComponent(0.0, 0.0));
  world.addComponent(ballId, new PrevFinalPosComponent(-0.2, 0.0));
  world.addComponent(ballId, new OrientationComponent(0.0));
  world.addComponent(ballId, new PrevFinalOrientationComponent(0.0));
  world.addComponent(ballId, new RadiusComponent(1.0));
  world.addComponent(ballId, new MassComponent(1.0));
  world.addComponent(ballId, new MomentOfInertiaComponent(1.0));
  world.addComponent(ballId, new CoefficientOfFrictionComponent(0.0));

  const obsId = world.createEntity();
  world.addComponent(obsId, new PositionComponent(0.0, -2.0));
  world.addComponent(obsId, new PrevFinalPosComponent(0.0, -2.0));
  world.addComponent(obsId, new OrientationComponent(0.0));
  world.addComponent(obsId, new PrevFinalOrientationComponent(0.0));
  world.addComponent(obsId, new RadiusComponent(1.0));
  world.addComponent(obsId, new MassComponent(-1.0)); // fixed center
  world.addComponent(obsId, new MomentOfInertiaComponent(1.0)); // free rotation
  world.addComponent(obsId, new CoefficientOfFrictionComponent(0.0));

  return { world, ballId, obsId };
}

describe('LayerContactStaticFrictionSystem', () => {
  test('reduces tangential displacement on layered obstacle contacts', () => {
    const { world, ballId, obsId } = makeWorld();
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ballId,
      obs_id: obsId,
      direction: new Vector2(0.0, 1.0),
      raw_hit: false,
      delta_lambda: 0.4,
      ball_contact_offset: new Vector2(0.0, -1.0),
      obstacle_contact_radius: 1.0,
      ball_friction: 1.0,
      obstacle_friction: 1.0
    }]);

    const beforeX = world.getComponent(ballId, PositionComponent).pos.x;
    const system = new LayerContactStaticFrictionSystem();
    system.update(world, 0.016);
    const afterX = world.getComponent(ballId, PositionComponent).pos.x;

    expect(afterX).toBeLessThan(beforeX);
  });

  test('is limited by static friction bound mu * lambda_n', () => {
    const runWithMu = (mu) => {
      const { world, ballId, obsId } = makeWorld();
      world.setResource('ball_obstacle_contacts', [{
        ball_id: ballId,
        obs_id: obsId,
        direction: new Vector2(0.0, 1.0),
        raw_hit: false,
        delta_lambda: 0.4,
        ball_contact_offset: new Vector2(0.0, -1.0),
        obstacle_contact_radius: 1.0,
        ball_friction: mu,
        obstacle_friction: mu
      }]);
      const system = new LayerContactStaticFrictionSystem();
      system.update(world, 0.016);
      return Math.abs(world.getComponent(ballId, PositionComponent).pos.x);
    };

    const small = runWithMu(0.1);
    const large = runWithMu(1.0);
    expect(large).toBeGreaterThan(small + 1e-6);
  });

  test('ignores raw obstacle contacts', () => {
    const { world, ballId, obsId } = makeWorld();
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ballId,
      obs_id: obsId,
      direction: new Vector2(0.0, 1.0),
      raw_hit: true,
      delta_lambda: 0.4,
      ball_contact_offset: new Vector2(0.0, -1.0),
      obstacle_contact_radius: 1.0,
      ball_friction: 1.0,
      obstacle_friction: 1.0
    }]);

    const system = new LayerContactStaticFrictionSystem();
    system.update(world, 0.016);

    const pos = world.getComponent(ballId, PositionComponent).pos;
    expect(pos.x).toBeCloseTo(0.0, 9);
    expect(pos.y).toBeCloseTo(0.0, 9);
  });

  test('reduces tangential displacement for border contacts with layered ball friction', () => {
    const { world, ballId } = makeWorld();
    world.setResource('ball_border_contacts', [{
      ball_id: ballId,
      normal: new Vector2(0.0, 1.0),
      delta_lambda: 0.4,
      ball_contact_offset: new Vector2(0.0, -1.0),
      ball_friction: 1.0,
      friction: 1.0
    }]);

    const beforeX = world.getComponent(ballId, PositionComponent).pos.x;
    const system = new LayerContactStaticFrictionSystem();
    system.update(world, 0.016);
    const afterX = world.getComponent(ballId, PositionComponent).pos.x;
    expect(afterX).toBeLessThan(beforeX);
  });

  test('uses moving flipper surface displacement for static friction correction', () => {
    const { world, ballId } = makeWorld();

    const flipperId = world.createEntity();
    world.addComponent(flipperId, new PositionComponent(0.0, -1.0));
    world.addComponent(flipperId, new PrevFinalPosComponent(0.0, -1.0));
    world.addComponent(flipperId, new FlipperStateComponent(1.0, 0.0, 1.0, 0.0));
    const state = world.getComponent(flipperId, FlipperStateComponent);
    state.currentAngularVelocity = 20.0;
    state.rotation = 0.0;

    const tipId = world.createEntity();
    world.addComponent(tipId, new FlipperTipComponent(flipperId));
    world.addComponent(tipId, new CoefficientOfFrictionComponent(1.0));

    world.setResource('dt', 0.016);
    world.setResource('ball_flipper_contacts', [{
      ball_id: ballId,
      flip_id: flipperId,
      normal: new Vector2(0.0, 1.0),
      delta_lambda: 0.4,
      contact_point_on_flipper: new Vector2(0.0, 0.0),
      ball_contact_offset: new Vector2(0.0, -1.0),
      ball_friction: 1.0
    }]);

    const beforeX = world.getComponent(ballId, PositionComponent).pos.x;
    const system = new LayerContactStaticFrictionSystem();
    system.update(world, 0.016);
    const afterX = world.getComponent(ballId, PositionComponent).pos.x;
    expect(Math.abs(afterX - beforeX)).toBeGreaterThan(1e-6);
  });
});
