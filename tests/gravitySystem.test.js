const { World, VelocityComponent, GravityAffectedComponent, Vector2, GravitySystem } = require('../cable_joints/cable_joints_core');

// Mocked world for system tests
describe('GravitySystem', () => {
  test('applies gravity to velocity when GravityAffectedComponent present', () => {
    const world = new World();
    const e = world.createEntity();
    world.addComponent(e, new VelocityComponent(0, 0));
    world.addComponent(e, new GravityAffectedComponent());
    const gravity = new Vector2(0, -9.8);
    world.setResource('gravity', gravity);

    const dt = 0.5;
    const system = new GravitySystem();
    system.update(world, dt);

    const velComp = world.getComponent(e, VelocityComponent);
    expect(velComp.vel.x).toBeCloseTo(gravity.x * dt);
    expect(velComp.vel.y).toBeCloseTo(gravity.y * dt);
  });

  test('does not change velocity when no GravityAffectedComponent', () => {
    const world = new World();
    const e = world.createEntity();
    world.addComponent(e, new VelocityComponent(1, 1));
    const gravity = new Vector2(0, -9.8);
    world.setResource('gravity', gravity);
    const dt = 1.0;
    const system = new GravitySystem();
    system.update(world, dt);

    const velComp = world.getComponent(e, VelocityComponent);
    expect(velComp.vel.x).toBeCloseTo(1);
    expect(velComp.vel.y).toBeCloseTo(1);
  });

  test('does not crash if no VelocityComponent', () => {
    const world = new World();
    const e = world.createEntity();
    world.addComponent(e, new GravityAffectedComponent());
    const gravity = new Vector2(0, -9.8);
    world.setResource('gravity', gravity);
    const dt = 1.0;
    const system = new GravitySystem();
    system.update(world, dt);
  });

  test('does nothing if gravity resource undefined', () => {
    const world = new World();
    const e = world.createEntity();
    world.addComponent(e, new VelocityComponent(2, 3));
    world.addComponent(e, new GravityAffectedComponent());
    const dt = 1.0;
    const system = new GravitySystem();
    system.update(world, dt);

    const velComp = world.getComponent(e, VelocityComponent);
    expect(velComp.vel.x).toBeCloseTo(2);
    expect(velComp.vel.y).toBeCloseTo(3);
  });
});
