import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
} from '../../../src/js/cable_joints/ecs.js';
import { ObstaclePushComponent } from '../../../examples/js/flipper/flipper_common.js';
import { BallObstacleBumpSystem } from '../../../examples/js/flipper/ball_obstacle_bump_system.js';

describe('BallObstacleBumpSystem raw-contact gating', () => {
  function createWorld() {
    const world = new World();

    const ballId = world.createEntity();
    world.addComponent(ballId, new VelocityComponent(0.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(ballId, new AngularVelocityComponent(0.0));
    world.addComponent(ballId, new MomentOfInertiaComponent(1.0));

    const obsId = world.createEntity();
    world.addComponent(obsId, new RadiusComponent(1.0));
    world.addComponent(obsId, new ObstaclePushComponent(2.0));

    return { world, ballId, obsId };
  }

  test('skips push impulse for wrap-only contacts (raw_contact=false)', () => {
    const { world, ballId, obsId } = createWorld();
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ballId,
      obs_id: obsId,
      direction: new Vector2(1.0, 0.0),
      raw_contact: false,
      ball_contact_radius: 1.2,
      obs_contact_radius: 1.2,
    }]);

    const system = new BallObstacleBumpSystem();
    system.update(world, 0.016);

    const v = world.getComponent(ballId, VelocityComponent).vel;
    expect(v.x).toBeCloseTo(0.0, 9);
    expect(v.y).toBeCloseTo(0.0, 9);
  });

  test('applies push impulse for raw contacts (raw_contact=true)', () => {
    const { world, ballId, obsId } = createWorld();
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ballId,
      obs_id: obsId,
      direction: new Vector2(1.0, 0.0),
      raw_contact: true,
      ball_contact_radius: 1.0,
      obs_contact_radius: 1.0,
    }]);

    const system = new BallObstacleBumpSystem();
    system.update(world, 0.016);

    const v = world.getComponent(ballId, VelocityComponent).vel;
    expect(v.x).toBeGreaterThan(0.0);
  });
});
