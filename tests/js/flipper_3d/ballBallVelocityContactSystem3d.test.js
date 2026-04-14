import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { BallBallVelocityContactSystem3D } from '../../../example_apps/js/flipper_3d/ball_ball_velocity_contact_system_3d.js';

describe('BallBallVelocityContactSystem3D', () => {
  test('uses layered contact offsets and friction values from the manifold', () => {
    const world = new World();

    const ballA = world.createEntity();
    world.addComponent(ballA, new VelocityComponent(1.0, 0.0, 0.0));
    world.addComponent(ballA, new RadiusComponent(1.0));
    world.addComponent(ballA, new MassComponent(1.0));
    world.addComponent(ballA, new RestitutionComponent(0.0));
    world.addComponent(ballA, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(ballA, new MomentOfInertiaComponent(1.0));
    world.addComponent(ballA, new CoefficientOfFrictionComponent(0.0));

    const ballB = world.createEntity();
    world.addComponent(ballB, new VelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(ballB, new RadiusComponent(1.0));
    world.addComponent(ballB, new MassComponent(1.0));
    world.addComponent(ballB, new RestitutionComponent(0.0));
    world.addComponent(ballB, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(ballB, new MomentOfInertiaComponent(1.0));
    world.addComponent(ballB, new CoefficientOfFrictionComponent(0.0));

    const runContact = (offsetScale, friction) => {
      world.getComponent(ballA, VelocityComponent).vel.set(new Vector3(1.0, 0.0, 0.0));
      world.getComponent(ballB, VelocityComponent).vel.set(new Vector3(0.0, 0.0, 0.0));
      world.getComponent(ballA, AngularVelocityComponent).omega.set(new Vector3(0.0, 0.0, 0.0));
      world.getComponent(ballB, AngularVelocityComponent).omega.set(new Vector3(0.0, 0.0, 0.0));
      world.setResource('ball_ball_contacts', [{
        ball_a: ballA,
        ball_b: ballB,
        normal: new Vector3(0.0, 1.0, 0.0),
        delta_lambda: 0.5,
        contact_offset_a: new Vector3(0.0, offsetScale, 0.0),
        contact_offset_b: new Vector3(0.0, -offsetScale, 0.0),
        friction_a: friction,
        friction_b: friction,
        radius_a: 1.0,
        radius_b: 1.0
      }]);
      const system = new BallBallVelocityContactSystem3D();
      system.update(world, 0.016);
      return {
        omegaA: Math.abs(world.getComponent(ballA, AngularVelocityComponent).omega.z),
        omegaB: Math.abs(world.getComponent(ballB, AngularVelocityComponent).omega.z)
      };
    };

    const noLeverArm = runContact(0.0, 1.0);
    const withLayeredOffset = runContact(1.0, 1.0);
    const withoutLayeredFriction = runContact(1.0, 0.0);

    expect(noLeverArm.omegaA).toBeCloseTo(0.0, 9);
    expect(noLeverArm.omegaB).toBeCloseTo(0.0, 9);
    expect(withLayeredOffset.omegaA).toBeGreaterThan(1e-6);
    expect(withLayeredOffset.omegaB).toBeGreaterThan(1e-6);
    expect(withoutLayeredFriction.omegaA).toBeCloseTo(0.0, 9);
    expect(withoutLayeredFriction.omegaB).toBeCloseTo(0.0, 9);
  });
});
