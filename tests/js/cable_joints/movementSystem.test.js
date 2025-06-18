import {
  World,
  PositionComponent,
  VelocityComponent
} from '../cable_joints/ecs.js';

import { MovementSystem } from '../cable_joints/commonSystems.js';

describe('MovementSystem', () => {
  test('updates position based on velocity and dt', () => {
    const world = new World();
    const e = world.createEntity();
    world.addComponent(e, new PositionComponent(1, 2));
    world.addComponent(e, new VelocityComponent(3, 4));
    const dt = 0.5;
    const system = new MovementSystem();
    system.update(world, dt);

    const pos = world.getComponent(e, PositionComponent).pos;
    expect(pos.x).toBeCloseTo(1 + 3 * dt);
    expect(pos.y).toBeCloseTo(2 + 4 * dt);
  });

  test('does not update entities missing components', () => {
    const world = new World();
    const e1 = world.createEntity();
    world.addComponent(e1, new PositionComponent(5, 5));
    // missing VelocityComponent
    const e2 = world.createEntity();
    world.addComponent(e2, new VelocityComponent(1, 1));
    const dt = 1.0;
    const system = new MovementSystem();
    system.update(world, dt);

    const pos1 = world.getComponent(e1, PositionComponent).pos;
    expect(pos1.x).toBeCloseTo(5);
    expect(pos1.y).toBeCloseTo(5);

    // e2 has no PositionComponent
    expect(world.getComponent(e2, PositionComponent)).toBeUndefined();
  });

  test('handles empty world without error', () => {
    const world = new World();
    const dt = 0.1;
    const system = new MovementSystem();
    expect(() => system.update(world, dt)).not.toThrow();
  });
});
