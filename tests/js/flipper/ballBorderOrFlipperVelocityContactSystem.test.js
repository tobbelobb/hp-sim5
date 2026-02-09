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
} from '../../../src/js/cable_joints/ecs.js';
import {
  FlipperStateComponent,
} from '../../../examples/js/flipper/flipper_common.js';
import { BallBorderOrFlipperVelocityContactSystem } from '../../../examples/js/flipper/ball_border_or_flipper_velocity_contact_system.js';

describe('BallBorderOrFlipperVelocityContactSystem flipper wrap gating', () => {
  function makeWorldWithFlipperContact(rawContact) {
    const world = new World();

    const ballId = world.createEntity();
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new VelocityComponent(0.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(ballId, new MomentOfInertiaComponent(1.0));
    world.addComponent(ballId, new AngularVelocityComponent(0.0));
    world.addComponent(ballId, new RestitutionComponent(1.0));

    const flipperId = world.createEntity();
    world.addComponent(flipperId, new PositionComponent(0.0, 0.0));
    world.addComponent(flipperId, new FlipperStateComponent(1.0, 0.0, 1.0, 0.0));
    world.getComponent(flipperId, FlipperStateComponent).currentAngularVelocity = -10.0;
    world.addComponent(flipperId, new RestitutionComponent(1.0));

    world.setResource('ball_flipper_contacts', [{
      ball_id: ballId,
      flip_id: flipperId,
      normal: new Vector2(1.0, 0.0),
      contact_point_on_flipper: new Vector2(0.0, 1.0),
      delta_lambda: 0.5,
      ball_contact_radius: 1.0,
      raw_contact: rawContact
    }]);

    return { world, ballId };
  }

  test('ignores wrap-only flipper contacts in velocity phase', () => {
    const { world, ballId } = makeWorldWithFlipperContact(false);
    const system = new BallBorderOrFlipperVelocityContactSystem();
    system.update(world, 0.016);

    const vel = world.getComponent(ballId, VelocityComponent).vel;
    expect(vel.x).toBeCloseTo(0.0, 9);
    expect(vel.y).toBeCloseTo(0.0, 9);
  });

  test('applies velocity impulse for raw flipper contacts', () => {
    const { world, ballId } = makeWorldWithFlipperContact(true);
    const system = new BallBorderOrFlipperVelocityContactSystem();
    system.update(world, 0.016);

    const vel = world.getComponent(ballId, VelocityComponent).vel;
    expect(vel.x).toBeGreaterThan(0.0);
  });

  test('uses ball_contact_offset for non-radial raw flipper impulse', () => {
    const { world, ballId } = makeWorldWithFlipperContact(true);
    const contact = world.getResource('ball_flipper_contacts')[0];
    contact.normal = new Vector2(0.0, -1.0);
    contact.contact_point_on_flipper = new Vector2(1.0, 0.0);
    contact.ball_contact_offset = new Vector2(1.0, 0.0);

    const system = new BallBorderOrFlipperVelocityContactSystem();
    system.update(world, 0.016);

    const angVel = world.getComponent(ballId, AngularVelocityComponent).angularVelocity;
    expect(Math.abs(angVel)).toBeGreaterThan(1e-6);
  });
});
