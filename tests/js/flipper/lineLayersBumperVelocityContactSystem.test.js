import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';
import { LineLayersBumperVelocityContactSystem } from '../../../examples/js/flipper/line_layers_bumper_velocity_contact_system.js';

function createWorld() {
  const world = new World();

  const ballId = world.createEntity();
  world.addComponent(ballId, new VelocityComponent(0.0, 0.0));
  world.addComponent(ballId, new RadiusComponent(1.0));
  world.addComponent(ballId, new MassComponent(1.0));
  world.addComponent(ballId, new RestitutionComponent(0.0));
  world.addComponent(ballId, new AngularVelocityComponent(0.0));
  world.addComponent(ballId, new MomentOfInertiaComponent(1.0));
  world.addComponent(ballId, new CoefficientOfFrictionComponent(0.0));

  const obsId = world.createEntity();
  world.addComponent(obsId, new VelocityComponent(0.0, 0.0));
  world.addComponent(obsId, new RadiusComponent(1.0));
  world.addComponent(obsId, new MassComponent(-1.0));
  world.addComponent(obsId, new RestitutionComponent(0.0));
  world.addComponent(obsId, new AngularVelocityComponent(0.0));
  world.addComponent(obsId, new MomentOfInertiaComponent(1.0));
  world.addComponent(obsId, new CoefficientOfFrictionComponent(0.0));

  return { world, ballId, obsId };
}

describe('LineLayersBumperVelocityContactSystem', () => {
  test('applies frictional angular impulse for non-raw layered contacts', () => {
    const { world, ballId, obsId } = createWorld();
    world.getComponent(ballId, VelocityComponent).vel.set(new Vector2(1.0, 0.0));

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

    const system = new LineLayersBumperVelocityContactSystem();
    system.update(world, 0.016);

    const ballOmega = Math.abs(world.getComponent(ballId, AngularVelocityComponent).angularVelocity);
    expect(ballOmega).toBeGreaterThan(1e-6);
  });

  test('transfers obstacle surface velocity to layered contact response', () => {
    const { world, ballId, obsId } = createWorld();
    world.getComponent(obsId, AngularVelocityComponent).angularVelocity = 20.0;

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

    const system = new LineLayersBumperVelocityContactSystem();
    system.update(world, 0.016);

    const ballVel = world.getComponent(ballId, VelocityComponent).vel;
    expect(Math.abs(ballVel.x)).toBeGreaterThan(1e-6);
  });

  test('skips raw contacts handled by bumper push system', () => {
    const { world, ballId, obsId } = createWorld();
    world.getComponent(ballId, VelocityComponent).vel.set(new Vector2(1.0, 0.0));
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

    const system = new LineLayersBumperVelocityContactSystem();
    system.update(world, 0.016);

    const ballVel = world.getComponent(ballId, VelocityComponent).vel;
    const ballOmega = world.getComponent(ballId, AngularVelocityComponent).angularVelocity;
    expect(ballVel.x).toBeCloseTo(1.0, 9);
    expect(ballVel.y).toBeCloseTo(0.0, 9);
    expect(ballOmega).toBeCloseTo(0.0, 9);
  });
});
