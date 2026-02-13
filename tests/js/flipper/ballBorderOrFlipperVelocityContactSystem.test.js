import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  FlipperStateComponent,
  FlipperTipComponent,
} from '../../../examples/js/flipper/flipper_common.js';
import { BallBorderOrFlipperVelocityContactSystem } from '../../../examples/js/flipper/ball_border_or_flipper_velocity_contact_system.js';

function makeWorldWithFlipperContact() {
  const world = new World();

  const ballId = world.createEntity();
  world.addComponent(ballId, new PositionComponent(0.0, 0.0));
  world.addComponent(ballId, new VelocityComponent(0.0, 0.0));
  world.addComponent(ballId, new RadiusComponent(1.0));
  world.addComponent(ballId, new MassComponent(1.0));
  world.addComponent(ballId, new MomentOfInertiaComponent(1.0));
  world.addComponent(ballId, new AngularVelocityComponent(0.0));
  world.addComponent(ballId, new RestitutionComponent(1.0));
  world.addComponent(ballId, new CoefficientOfFrictionComponent(0.8));

  const flipperId = world.createEntity();
  world.addComponent(flipperId, new PositionComponent(0.0, 0.0));
  world.addComponent(flipperId, new FlipperStateComponent(1.0, 0.0, 1.0, 0.0));
  world.getComponent(flipperId, FlipperStateComponent).currentAngularVelocity = -10.0;
  world.addComponent(flipperId, new RestitutionComponent(1.0));

  const tipId = world.createEntity();
  world.addComponent(tipId, new FlipperTipComponent(flipperId));
  world.addComponent(tipId, new CoefficientOfFrictionComponent(0.8));

  world.setResource('ball_flipper_contacts', [{
    ball_id: ballId,
    flip_id: flipperId,
    normal: new Vector2(1.0, 0.0),
    contact_point_on_flipper: new Vector2(0.0, 1.0),
    delta_lambda: 0.5,
    ball_contact_offset: new Vector2(-1.0, 0.0)
  }]);

  return { world, ballId, flipperId };
}

describe('BallBorderOrFlipperVelocityContactSystem', () => {
  test('applies velocity impulse for flipper contacts', () => {
    const { world, ballId } = makeWorldWithFlipperContact();
    const system = new BallBorderOrFlipperVelocityContactSystem();
    system.update(world, 0.016);

    const vel = world.getComponent(ballId, VelocityComponent).vel;
    expect(vel.x).toBeGreaterThan(0.0);
  });

  test('does not apply restitution while separating from contact normal', () => {
    const { world, ballId, flipperId } = makeWorldWithFlipperContact();
    world.getComponent(ballId, VelocityComponent).vel.set(new Vector2(1.0, 0.0));
    world.getComponent(flipperId, FlipperStateComponent).currentAngularVelocity = 0.0;
    const contact = world.getResource('ball_flipper_contacts')[0];
    contact.delta_lambda = 0.0;

    const system = new BallBorderOrFlipperVelocityContactSystem();
    system.update(world, 0.016);

    const vel = world.getComponent(ballId, VelocityComponent).vel;
    expect(vel.x).toBeCloseTo(1.0, 9);
    expect(vel.y).toBeCloseTo(0.0, 9);
  });

  test('uses ball_contact_offset for friction torque leverage', () => {
    const runWithContactOffset = (offsetX) => {
      const { world, ballId, flipperId } = makeWorldWithFlipperContact();
      world.getComponent(flipperId, FlipperStateComponent).currentAngularVelocity = -10.0;
      const contact = world.getResource('ball_flipper_contacts')[0];
      contact.normal = new Vector2(1.0, 0.0);
      contact.contact_point_on_flipper = new Vector2(1.0, 0.0);
      contact.delta_lambda = 1.0;
      contact.ball_contact_offset = new Vector2(offsetX, 0.0);
      const system = new BallBorderOrFlipperVelocityContactSystem();
      system.update(world, 0.016);
      return world.getComponent(ballId, AngularVelocityComponent).angularVelocity;
    };

    const angSmall = Math.abs(runWithContactOffset(-1.0));
    const angLarge = Math.abs(runWithContactOffset(-2.0));
    expect(Math.abs(angLarge - angSmall)).toBeGreaterThan(1e-6);
  });

  test('keeps border-contact velocity finite when restitution/friction are missing', () => {
    const world = new World();

    const ballId = world.createEntity();
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new VelocityComponent(1.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(ballId, new MomentOfInertiaComponent(1.0));
    world.addComponent(ballId, new AngularVelocityComponent(0.0));
    world.addComponent(ballId, new RestitutionComponent(0.6));
    world.addComponent(ballId, new CoefficientOfFrictionComponent(0.2));

    world.setResource('ball_border_contacts', [{
      ball_id: ballId,
      normal: new Vector2(0.0, 1.0),
      delta_lambda: 0.25,
      ball_contact_offset: new Vector2(0.0, -1.0),
      restitution: undefined,
      friction: undefined
    }]);

    const system = new BallBorderOrFlipperVelocityContactSystem();
    system.update(world, 0.016);

    const vel = world.getComponent(ballId, VelocityComponent).vel;
    const angVel = world.getComponent(ballId, AngularVelocityComponent).angularVelocity;
    expect(Number.isFinite(vel.x)).toBe(true);
    expect(Number.isFinite(vel.y)).toBe(true);
    expect(Number.isFinite(angVel)).toBe(true);
  });

  test('uses layered ball_friction from contact when present', () => {
    const world = new World();

    const ballId = world.createEntity();
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new VelocityComponent(0.0, -1.0));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(ballId, new MomentOfInertiaComponent(1.0));
    world.addComponent(ballId, new AngularVelocityComponent(0.0));
    world.addComponent(ballId, new RestitutionComponent(0.0));
    world.addComponent(ballId, new CoefficientOfFrictionComponent(0.0));

    const runWithLayeredMu = (layeredMu) => {
      world.getComponent(ballId, VelocityComponent).vel.set(new Vector2(1.0, 0.0));
      world.getComponent(ballId, AngularVelocityComponent).angularVelocity = 0.0;
      world.setResource('ball_border_contacts', [{
        ball_id: ballId,
        normal: new Vector2(0.0, 1.0),
        delta_lambda: 0.4,
        ball_contact_offset: new Vector2(0.0, -1.0),
        restitution: 0.0,
        friction: 0.0,
        ball_friction: layeredMu
      }]);
      const system = new BallBorderOrFlipperVelocityContactSystem();
      system.update(world, 0.016);
      return Math.abs(world.getComponent(ballId, AngularVelocityComponent).angularVelocity);
    };

    const noLayerFriction = runWithLayeredMu(0.0);
    const withLayerFriction = runWithLayeredMu(1.0);
    expect(withLayerFriction).toBeGreaterThan(noLayerFriction + 1e-6);
  });
});
