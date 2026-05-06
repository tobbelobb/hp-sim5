import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

import {
  World,
  PositionComponent,
  RadiusComponent,
  CoefficientOfFrictionComponent
} from '../../../src/js/cable_joints_3d/ecs.js';

import {
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

import { CableFrictionSystem } from '../../../src/js/cable_joints_3d/cable_friction_system.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('CableFrictionSystem (3D)', () => {
  test('uses cable half-width as effective rolling radius when the base radius is zero', () => {
    const world = new World();

    const left = world.createEntity();
    world.addComponent(left, new PositionComponent(-10, 0, 0));

    const wheel = world.createEntity();
    world.addComponent(wheel, new PositionComponent(0, 0, 0));
    world.addComponent(wheel, new RadiusComponent(0.0));
    world.addComponent(wheel, new CoefficientOfFrictionComponent(0.5));

    const right = world.createEntity();
    world.addComponent(right, new PositionComponent(1, 0, 0));

    const joint0 = world.createEntity();
    world.addComponent(
      joint0,
      new CableJointComponent(
        left,
        wheel,
        1.0,
        new Vector3(0, 0, 0),
        new Vector3(10, 0, 0)
      )
    );

    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        wheel,
        right,
        1.0,
        new Vector3(0, 0, 0),
        new Vector3(1, 0, 0)
      )
    );

    const pathId = world.createEntity();
    world.addComponent(
      pathId,
      new CablePathComponent(
        world,
        [joint0, joint1],
        ['attachment', 'rolling', 'attachment'],
        [true, true, true],
        1e6,
        [0.0, 1.0, 0.0],
        0.5
      )
    );

    const system = new CableFrictionSystem();
    system.update(world, 1 / 2000);

    const threshold = Math.exp(0.5 * (1.0 / 0.5));
    const totalRest = 2.0;
    const expectedHigh = (10.0 * totalRest) / (10.0 + 1.0 * threshold);
    const expectedLow = totalRest - expectedHigh;

    expect(world.getComponent(joint0, CableJointComponent).restLength).toBeCloseTo(expectedHigh, 8);
    expect(world.getComponent(joint1, CableJointComponent).restLength).toBeCloseTo(expectedLow, 8);
  });

  test('free-spinning rolling links equalize tension despite material friction', () => {
    const world = new World();

    const left = world.createEntity();
    world.addComponent(left, new PositionComponent(-10, 0, 0));

    const wheel = world.createEntity();
    world.addComponent(wheel, new PositionComponent(0, 0, 0));
    world.addComponent(wheel, new RadiusComponent(0.5));
    world.addComponent(wheel, new CoefficientOfFrictionComponent(0.5));
    world.addComponent(wheel, new SpoolStateComponent(null, new Vector3(0, 0, 1)));

    const right = world.createEntity();
    world.addComponent(right, new PositionComponent(1, 0, 0));

    const joint0 = world.createEntity();
    world.addComponent(
      joint0,
      new CableJointComponent(
        left,
        wheel,
        1.0,
        new Vector3(0, 0, 0),
        new Vector3(10, 0, 0)
      )
    );

    const joint1 = world.createEntity();
    world.addComponent(
      joint1,
      new CableJointComponent(
        wheel,
        right,
        1.0,
        new Vector3(0, 0, 0),
        new Vector3(1, 0, 0)
      )
    );

    const pathId = world.createEntity();
    world.addComponent(
      pathId,
      new CablePathComponent(
        world,
        [joint0, joint1],
        ['attachment', 'rolling', 'attachment'],
        [true, true, true],
        1e6,
        [0.0, 1.0, 0.0],
        0.0
      )
    );

    const system = new CableFrictionSystem();
    system.update(world, 1 / 2000);

    expect(world.getComponent(joint0, CableJointComponent).restLength).toBeCloseTo(20 / 11, 8);
    expect(world.getComponent(joint1, CableJointComponent).restLength).toBeCloseTo(2 / 11, 8);
  });
});
