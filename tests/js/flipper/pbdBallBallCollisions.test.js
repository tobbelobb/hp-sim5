import {
  World,
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
} from '../../../src/js/cable_joints/ecs.js';

import {
  BallTagComponent,
  PBDBallBallCollisions,
  PBDBallCircleSectorCollisions,
  OverlayRadiusComponent,
  CircleSectorComponent
} from '../../../examples/js/flipper/flipper_common.js';

describe('PBDBallBallCollisions', () => {
  test('velocities remain unchanged on perfectly elastic head-on collision', () => {
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
    // Position-only resolution should not change velocities
    expect(v1.x).toBeCloseTo(1);
    expect(v2.x).toBeCloseTo(-1);
  });

  test('velocities unchanged with restitution 0.5 on head-on collision', () => {
    const world = new World();
    const dt = 0.016;
    const ball1 = world.createEntity();
    const ball2 = world.createEntity();

    world.addComponent(ball1, new BallTagComponent());
    world.addComponent(ball1, new PositionComponent(0, 0));
    world.addComponent(ball1, new VelocityComponent(1, 0));
    world.addComponent(ball1, new RadiusComponent(1));
    world.addComponent(ball1, new MassComponent(1));
    world.addComponent(ball1, new RestitutionComponent(0.5));

    world.addComponent(ball2, new BallTagComponent());
    world.addComponent(ball2, new PositionComponent(2, 0)); // touching: radius sum = 2
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(0.5));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1 = world.getComponent(ball1, VelocityComponent).vel;
    const v2 = world.getComponent(ball2, VelocityComponent).vel;
    // Position correction only
    expect(v1.x).toBeCloseTo(1);
    expect(v2.x).toBeCloseTo(-1);
  });

  test('uses the smallest restitution but leaves velocities unchanged', () => {
    // This is a simple heuristics we use. Might want to change to more realistic restitution/collision handling later
    const world = new World();
    const dt = 0.016;
    const ball1 = world.createEntity();
    const ball2 = world.createEntity();

    world.addComponent(ball1, new BallTagComponent());
    world.addComponent(ball1, new PositionComponent(0, 0));
    world.addComponent(ball1, new VelocityComponent(1, 0));
    world.addComponent(ball1, new RadiusComponent(1));
    world.addComponent(ball1, new MassComponent(1));
    world.addComponent(ball1, new RestitutionComponent(1.0));

    world.addComponent(ball2, new BallTagComponent());
    world.addComponent(ball2, new PositionComponent(2, 0)); // touching: radius sum = 2
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(0.5));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1 = world.getComponent(ball1, VelocityComponent).vel;
    const v2 = world.getComponent(ball2, VelocityComponent).vel;
    // Even with different restitution, velocities are unaffected
    expect(v1.x).toBeCloseTo(1);
    expect(v2.x).toBeCloseTo(-1);
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

  test('handles overlaps gracefully without altering velocity', () => {
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
    world.addComponent(ball2, new PositionComponent(1.9, 0)); // overlapping: radius sum = 2
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(1));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1 = world.getComponent(ball1, VelocityComponent).vel;
    const v2 = world.getComponent(ball2, VelocityComponent).vel;
    // Only penetration is resolved
    expect(v1.x).toBeCloseTo(1);
    expect(v2.x).toBeCloseTo(-1);
  });

  test('distributes impact based on inverse mass without changing velocity', () => {
    const world = new World();
    const dt = 0.016;
    const ball1 = world.createEntity();
    const ball2 = world.createEntity();

    world.addComponent(ball1, new BallTagComponent());
    world.addComponent(ball1, new PositionComponent(0, 0));
    world.addComponent(ball1, new VelocityComponent(1, 0));
    world.addComponent(ball1, new RadiusComponent(1));
    world.addComponent(ball1, new MassComponent(2)); // twice the mass
    world.addComponent(ball1, new RestitutionComponent(1));

    world.addComponent(ball2, new BallTagComponent());
    world.addComponent(ball2, new PositionComponent(2, 0)); // overlapping: radius sum = 2
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(1));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1 = world.getComponent(ball1, VelocityComponent).vel;
    const v2 = world.getComponent(ball2, VelocityComponent).vel;
    // Velocities remain the same since the position solver only resolves overlap
    expect(v1.x).toBeCloseTo(1);
    expect(v2.x).toBeCloseTo(-1);
  });

  test('velocities unchanged with restitution 0.9 and different masses', () => {
    // invMass sum = 1/2 + 1 = 1.5
    // restitution = 0.9
    // Computed velocities: entity1 ≈ -0.26667, entity2 ≈ 1.53333
    const world = new World();
    const dt = 0.016;
    const ball1 = world.createEntity();
    const ball2 = world.createEntity();

    world.addComponent(ball1, new BallTagComponent());
    world.addComponent(ball1, new PositionComponent(0, 0));
    world.addComponent(ball1, new VelocityComponent(1, 0));
    world.addComponent(ball1, new RadiusComponent(1));
    world.addComponent(ball1, new MassComponent(2));
    world.addComponent(ball1, new RestitutionComponent(0.9));

    world.addComponent(ball2, new BallTagComponent());
    world.addComponent(ball2, new PositionComponent(2, 0)); // touching: radius sum = 2
    world.addComponent(ball2, new VelocityComponent(-1, 0));
    world.addComponent(ball2, new RadiusComponent(1));
    world.addComponent(ball2, new MassComponent(1));
    world.addComponent(ball2, new RestitutionComponent(0.9));

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const v1_r = world.getComponent(ball1, VelocityComponent).vel;
    const v2_r = world.getComponent(ball2, VelocityComponent).vel;
    expect(v1_r.x).toBeCloseTo(1);
    expect(v2_r.x).toBeCloseTo(-1);
    expect(v1_r.y).toBeCloseTo(0.0);
    expect(v2_r.y).toBeCloseTo(0.0);
  });

  test('uses overlay radius for collision detection', () => {
    const withoutWrapWorld = new World();
    const withWrapWorld = new World();

    const createPair = (world) => {
      const ball1 = world.createEntity();
      world.addComponent(ball1, new BallTagComponent());
      world.addComponent(ball1, new PositionComponent(0, 0));
      world.addComponent(ball1, new VelocityComponent(0, 0));
      world.addComponent(ball1, new RadiusComponent(1));
      world.addComponent(ball1, new MassComponent(1));
      world.addComponent(ball1, new RestitutionComponent(1));

      const ball2 = world.createEntity();
      world.addComponent(ball2, new BallTagComponent());
      world.addComponent(ball2, new PositionComponent(2.15, 0)); // no base collision (2.0), but collides with +0.2 wrap
      world.addComponent(ball2, new VelocityComponent(0, 0));
      world.addComponent(ball2, new RadiusComponent(1));
      world.addComponent(ball2, new MassComponent(1));
      world.addComponent(ball2, new RestitutionComponent(1));
      return { ball1, ball2 };
    };

    const noWrap = createPair(withoutWrapWorld);
    const withWrap = createPair(withWrapWorld);
    withWrapWorld.addComponent(withWrap.ball1, new OverlayRadiusComponent(1.2));

    const system = new PBDBallBallCollisions();
    system.update(withoutWrapWorld, 0.016);
    system.update(withWrapWorld, 0.016);

    const noWrapBall1X = withoutWrapWorld.getComponent(noWrap.ball1, PositionComponent).pos.x;
    const noWrapBall2X = withoutWrapWorld.getComponent(noWrap.ball2, PositionComponent).pos.x;
    expect(noWrapBall1X).toBeCloseTo(0.0, 9);
    expect(noWrapBall2X).toBeCloseTo(2.15, 9);

    const withWrapBall1X = withWrapWorld.getComponent(withWrap.ball1, PositionComponent).pos.x;
    const withWrapBall2X = withWrapWorld.getComponent(withWrap.ball2, PositionComponent).pos.x;
    expect(withWrapBall1X).toBeLessThan(0.0);
    expect(withWrapBall2X).toBeGreaterThan(2.15);
  });

  test('collision radius applies directional circle sectors', () => {
    const withoutSectors = new World();
    const world = new World();
    const dt = 0.016;

    const createPair = (targetWorld) => {
      const left = targetWorld.createEntity();
      targetWorld.addComponent(left, new BallTagComponent());
      targetWorld.addComponent(left, new PositionComponent(0.0, 0.0));
      targetWorld.addComponent(left, new VelocityComponent(0, 0));
      targetWorld.addComponent(left, new RadiusComponent(1));
      targetWorld.addComponent(left, new MassComponent(1));
      targetWorld.addComponent(left, new RestitutionComponent(1));

      const right = targetWorld.createEntity();
      targetWorld.addComponent(right, new BallTagComponent());
      targetWorld.addComponent(right, new PositionComponent(2.15, 0.0));
      targetWorld.addComponent(right, new VelocityComponent(0, 0));
      targetWorld.addComponent(right, new RadiusComponent(1));
      targetWorld.addComponent(right, new MassComponent(1));
      targetWorld.addComponent(right, new RestitutionComponent(1));
      return { left, right };
    };

    const noSectorsPair = createPair(withoutSectors);
    const withSectorsPair = createPair(world);
    world.addComponent(
      withSectorsPair.left,
      new CircleSectorComponent(
        1.2,
        -0.3,
        0.3,
        false
      )
    );
    world.addComponent(
      withSectorsPair.right,
      new CircleSectorComponent(
        1.2,
        Math.PI - 0.3,
        Math.PI + 0.3,
        false
      )
    );

    const baseNoSectors = new PBDBallBallCollisions();
    const sectorNoSectors = new PBDBallCircleSectorCollisions();
    baseNoSectors.update(withoutSectors, dt);
    sectorNoSectors.update(withoutSectors, dt);
    const base = new PBDBallBallCollisions();
    const sector = new PBDBallCircleSectorCollisions();
    base.update(world, dt);
    sector.update(world, dt);

    const noLeftX = withoutSectors.getComponent(noSectorsPair.left, PositionComponent).pos.x;
    const noRightX = withoutSectors.getComponent(noSectorsPair.right, PositionComponent).pos.x;
    expect(noLeftX).toBeCloseTo(0.0, 9);
    expect(noRightX).toBeCloseTo(2.15, 9);

    const leftX = world.getComponent(withSectorsPair.left, PositionComponent).pos.x;
    const rightX = world.getComponent(withSectorsPair.right, PositionComponent).pos.x;
    expect(leftX).toBeLessThan(0.0);
    expect(rightX).toBeGreaterThan(2.15);
  });
});
