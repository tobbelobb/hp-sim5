import {
  World,
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
} from '../../../src/js/cable_joints/ecs.js';
import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';

import {
  BallTagComponent,
  PBDBallBallCollisions,
  getEffectiveCollisionRadius
} from '../../../examples/js/flipper/flipper_common.js';

function addEndpointHybridWrap(world, endpointId, storedLength, halfWidth = 0.1) {
  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(3.0, 0.0));
  world.addComponent(anchorId, new RadiusComponent(0.1));
  world.addComponent(anchorId, new CableLinkComponent(3.0, 0.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      endpointId,
      anchorId,
      0.0,
      new Vector2(1.0, 0.0),
      new Vector2(2.9, 0.0)
    )
  );

  const pathId = world.createEntity();
  world.addComponent(
    pathId,
    new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [true, true],
      1e4,
      [storedLength, 0.0],
      halfWidth
    )
  );
}

function addEndpointHybridWrapCustomAttachment(world, endpointId, storedLength, attachmentDirX, halfWidth = 0.1) {
  const endpointPos = world.getComponent(endpointId, PositionComponent).pos;
  const dirSign = attachmentDirX >= 0 ? 1.0 : -1.0;
  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(endpointPos.x + 3.0 * dirSign, endpointPos.y));
  world.addComponent(anchorId, new RadiusComponent(0.1));
  world.addComponent(anchorId, new CableLinkComponent(endpointPos.x + 3.0 * dirSign, endpointPos.y));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      endpointId,
      anchorId,
      0.0,
      new Vector2(endpointPos.x + 1.0 * dirSign, endpointPos.y),
      new Vector2(endpointPos.x + 2.9 * dirSign, endpointPos.y)
    )
  );

  const pathId = world.createEntity();
  world.addComponent(
    pathId,
    new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'attachment'],
      [true, true],
      1e4,
      [storedLength, 0.0],
      halfWidth
    )
  );
}

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

  test('uses wrapped effective radius for collision detection', () => {
    const baseLayerCircumference = 2.0 * Math.PI * 1.1;

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
      world.addComponent(ball1, new CableLinkComponent(0, 0));

      const ball2 = world.createEntity();
      world.addComponent(ball2, new BallTagComponent());
      world.addComponent(ball2, new PositionComponent(2.15, 0)); // no base collision (2.0), but collides with +0.2 wrap
      world.addComponent(ball2, new VelocityComponent(0, 0));
      world.addComponent(ball2, new RadiusComponent(1));
      world.addComponent(ball2, new MassComponent(1));
      world.addComponent(ball2, new RestitutionComponent(1));
      world.addComponent(ball2, new CableLinkComponent(2.15, 0));
      return { ball1, ball2 };
    };

    const noWrap = createPair(withoutWrapWorld);
    const withWrap = createPair(withWrapWorld);
    addEndpointHybridWrap(withWrapWorld, withWrap.ball1, baseLayerCircumference);

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

  test('collision radius keeps wrapped endpoints outside rolling circle near partial-arc boundary', () => {
    const world = new World();
    const dt = 0.016;
    const partialStored = 0.06;

    const left = world.createEntity();
    world.addComponent(left, new BallTagComponent());
    world.addComponent(left, new PositionComponent(0.0, 0.0));
    world.addComponent(left, new VelocityComponent(0, 0));
    world.addComponent(left, new RadiusComponent(1));
    world.addComponent(left, new MassComponent(1));
    world.addComponent(left, new RestitutionComponent(1));
    world.addComponent(left, new CableLinkComponent(0.0, 0.0));

    const right = world.createEntity();
    world.addComponent(right, new BallTagComponent());
    world.addComponent(right, new PositionComponent(2.15, 0.0));
    world.addComponent(right, new VelocityComponent(0, 0));
    world.addComponent(right, new RadiusComponent(1));
    world.addComponent(right, new MassComponent(1));
    world.addComponent(right, new RestitutionComponent(1));
    world.addComponent(right, new CableLinkComponent(2.15, 0.0));

    addEndpointHybridWrapCustomAttachment(world, left, partialStored, +1.0);
    addEndpointHybridWrapCustomAttachment(world, right, partialStored, -1.0);

    const leftRadius = getEffectiveCollisionRadius(world, left, 1.0, new Vector2(1.0, 0.0));
    const rightRadius = getEffectiveCollisionRadius(world, right, 1.0, new Vector2(-1.0, 0.0));
    expect(leftRadius).toBeGreaterThanOrEqual(1.1 - 1e-9);
    expect(rightRadius).toBeGreaterThanOrEqual(1.1 - 1e-9);

    const system = new PBDBallBallCollisions();
    system.update(world, dt);

    const leftX = world.getComponent(left, PositionComponent).pos.x;
    const rightX = world.getComponent(right, PositionComponent).pos.x;
    expect(leftX).toBeLessThan(0.0);
    expect(rightX).toBeGreaterThan(2.15);
  });
});
