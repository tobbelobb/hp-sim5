const {
  World,
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  BallTagComponent,
  PBDBallBallCollisions
} = require('../cable_joints/cable_joints_core');

describe('PBDBallBallCollisions', () => {
  test('swaps velocities on perfectly elastic head-on collision', () => {
    const world = new World();
    const dt = 0.016;
    const ball1 = world.createEntity();
    const ball2 = world.createEntity();

    world.addComponent(ball1, new BallTagComponent());
    world.addComponent(ball1, new PositionComponent(0, 0));
    world.addComponent(ball1, new VelocityComponent(1, 0));
    world.addComponent(ball1, new RadiusComponent(1));
    world.addComponent(ball1, new MassComponent(1));
    world.addComponent(ball1, new RestitutionComponent(1));

    world.addComponent(ball2, new BallTagComponent());
    world.addComponent(ball2, new PositionComponent(2, 0)); // touching: radius sum = 2
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(1));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1 = world.getComponent(ball1, VelocityComponent).vel;
    const v2 = world.getComponent(ball2, VelocityComponent).vel;
    // Velocities should swap
    expect(v1.x).toBeCloseTo(-1);
    expect(v2.x).toBeCloseTo(1);
  });

  test('no change when balls do not intersect', () => {
    const world = new World();
    const dt = 0.016;
    const ball1 = world.createEntity();
    const ball2 = world.createEntity();

    world.addComponent(ball1, new BallTagComponent());
    world.addComponent(ball1, new PositionComponent(0, 0));
    world.addComponent(ball1, new VelocityComponent(1, 0));
    world.addComponent(ball1, new RadiusComponent(1));
    world.addComponent(ball1, new MassComponent(1));
    world.addComponent(ball1, new RestitutionComponent(1));

    world.addComponent(ball2, new BallTagComponent());
    world.addComponent(ball2, new PositionComponent(5, 0)); // far apart
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(1));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1 = world.getComponent(ball1, VelocityComponent).vel;
    const v2 = world.getComponent(ball2, VelocityComponent).vel;
    // Velocities should remain unchanged
    expect(v1.x).toBeCloseTo(1);
    expect(v2.x).toBeCloseTo(-1);
  });
});
