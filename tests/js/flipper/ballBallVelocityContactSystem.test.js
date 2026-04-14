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
import { BallBallVelocityContactSystem } from '../../../example_apps/js/flipper/ball_ball_velocity_contact_system.js';

function createWorld() {
  const world = new World();

  const a = world.createEntity();
  world.addComponent(a, new VelocityComponent(0.0, 0.0));
  world.addComponent(a, new RadiusComponent(1.0));
  world.addComponent(a, new MassComponent(1.0));
  world.addComponent(a, new RestitutionComponent(0.0));
  world.addComponent(a, new AngularVelocityComponent(0.0));
  world.addComponent(a, new MomentOfInertiaComponent(1.0));
  world.addComponent(a, new CoefficientOfFrictionComponent(0.0));

  const b = world.createEntity();
  world.addComponent(b, new VelocityComponent(0.0, 0.0));
  world.addComponent(b, new RadiusComponent(1.0));
  world.addComponent(b, new MassComponent(1.0));
  world.addComponent(b, new RestitutionComponent(0.0));
  world.addComponent(b, new AngularVelocityComponent(0.0));
  world.addComponent(b, new MomentOfInertiaComponent(1.0));
  world.addComponent(b, new CoefficientOfFrictionComponent(0.0));

  return { world, a, b };
}

describe('BallBallVelocityContactSystem', () => {
  test('applies tangential friction torque for ball-ball layered contact', () => {
    const { world, a, b } = createWorld();
    world.getComponent(a, VelocityComponent).vel.set(new Vector2(1.0, 0.0));

    world.setResource('ball_ball_contacts', [{
      ball_a: a,
      ball_b: b,
      normal: new Vector2(0.0, 1.0),
      delta_lambda: 0.4,
      radius_a: 1.0,
      radius_b: 1.0,
      contact_offset_a: new Vector2(0.0, -1.0),
      contact_offset_b: new Vector2(0.0, 1.0),
      friction_a: 1.0,
      friction_b: 1.0
    }]);

    const system = new BallBallVelocityContactSystem();
    system.update(world, 0.016);

    const omegaA = Math.abs(world.getComponent(a, AngularVelocityComponent).angularVelocity);
    const omegaB = Math.abs(world.getComponent(b, AngularVelocityComponent).angularVelocity);
    expect(omegaA).toBeGreaterThan(1e-6);
    expect(omegaB).toBeGreaterThan(1e-6);
  });

  test('includes both bodies surface velocities in relative tangential speed', () => {
    const { world, a, b } = createWorld();
    world.getComponent(b, AngularVelocityComponent).angularVelocity = 20.0;

    world.setResource('ball_ball_contacts', [{
      ball_a: a,
      ball_b: b,
      normal: new Vector2(0.0, 1.0),
      delta_lambda: 0.4,
      radius_a: 1.0,
      radius_b: 1.0,
      contact_offset_a: new Vector2(0.0, -1.0),
      contact_offset_b: new Vector2(0.0, 1.0),
      friction_a: 1.0,
      friction_b: 1.0
    }]);

    const system = new BallBallVelocityContactSystem();
    system.update(world, 0.016);

    const velA = world.getComponent(a, VelocityComponent).vel;
    expect(Math.abs(velA.x)).toBeGreaterThan(1e-6);
  });

  test('uses layered friction payloads when present', () => {
    const runWithLayerFriction = (mu) => {
      const { world, a, b } = createWorld();
      world.getComponent(a, VelocityComponent).vel.set(new Vector2(1.0, 0.0));
      world.setResource('ball_ball_contacts', [{
        ball_a: a,
        ball_b: b,
        normal: new Vector2(0.0, 1.0),
        delta_lambda: 0.4,
        radius_a: 1.0,
        radius_b: 1.0,
        contact_offset_a: new Vector2(0.0, -1.0),
        contact_offset_b: new Vector2(0.0, 1.0),
        friction_a: mu,
        friction_b: mu
      }]);
      const system = new BallBallVelocityContactSystem();
      system.update(world, 0.016);
      return Math.abs(world.getComponent(a, AngularVelocityComponent).angularVelocity);
    };

    const low = runWithLayerFriction(0.0);
    const high = runWithLayerFriction(1.0);
    expect(high).toBeGreaterThan(low + 1e-6);
  });
});
