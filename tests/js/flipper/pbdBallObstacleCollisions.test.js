import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';
import {
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  ScoreComponent,
  ScoreSystem,
  ScoredTagComponent,
  PBDUnifiedContactManifoldSystem,
  OverlayRadiusComponent,
} from '../../../example_apps/js/flipper/flipper_common.js';

function _makeWorld() {
  const world = new World();
  world.setResource('enableLayering', true);
  world.setResource('layeringCollisionSectorSolvers', true);
  return world;
}

describe('PBDUnifiedContactManifoldSystem obstacle scoring behavior', () => {
  test('scores only on obstacle contact entry, not every frame of persistent overlap', () => {
    const world = _makeWorld();

    const scoreId = world.createEntity();
    world.addComponent(scoreId, new ScoreComponent(0));

    const ballId = world.createEntity();
    world.addComponent(ballId, new BallTagComponent());
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));

    const obstacleId = world.createEntity();
    world.addComponent(obstacleId, new ObstacleTagComponent());
    world.addComponent(obstacleId, new PositionComponent(1.8, 0.0));
    world.addComponent(obstacleId, new RadiusComponent(1.0));
    world.addComponent(obstacleId, new ObstaclePushComponent(2.0));

    const collisionSystem = new PBDUnifiedContactManifoldSystem();
    const scoreSystem = new ScoreSystem();

    collisionSystem.update(world, 0.016);
    expect(world.hasComponent(ballId, ScoredTagComponent)).toBe(true);
    scoreSystem.update(world, 0.016);
    expect(world.getComponent(scoreId, ScoreComponent).value).toBe(1);

    collisionSystem.update(world, 0.016);
    expect(world.hasComponent(ballId, ScoredTagComponent)).toBe(false);
    scoreSystem.update(world, 0.016);
    expect(world.getComponent(scoreId, ScoreComponent).value).toBe(1);

    world.getComponent(ballId, PositionComponent).pos.set({ x: 0.0, y: 3.5 });
    collisionSystem.update(world, 0.016);
    scoreSystem.update(world, 0.016);
    expect(world.getComponent(scoreId, ScoreComponent).value).toBe(1);

    world.getComponent(ballId, PositionComponent).pos.set({ x: 0.0, y: 0.0 });
    collisionSystem.update(world, 0.016);
    expect(world.hasComponent(ballId, ScoredTagComponent)).toBe(true);
    scoreSystem.update(world, 0.016);
    expect(world.getComponent(scoreId, ScoreComponent).value).toBe(2);
  });

  test('uses overlay radius for obstacle side in circle-circle collision', () => {
    const makeCollisionWorld = (withWrap) => {
      const world = _makeWorld();

      const ballId = world.createEntity();
      world.addComponent(ballId, new BallTagComponent());
      world.addComponent(ballId, new PositionComponent(0.0, 0.0));
      world.addComponent(ballId, new RadiusComponent(1.0));
      world.addComponent(ballId, new MassComponent(1.0));

      const obsId = world.createEntity();
      world.addComponent(obsId, new ObstacleTagComponent());
      world.addComponent(obsId, new PositionComponent(2.15, 0.0));
      world.addComponent(obsId, new RadiusComponent(1.0));
      world.addComponent(obsId, new ObstaclePushComponent(2.0));

      if (withWrap) {
        world.addComponent(obsId, new OverlayRadiusComponent(1.2));
      }
      return { world, ballId };
    };

    const noWrap = makeCollisionWorld(false);
    const withWrap = makeCollisionWorld(true);
    const collisionSystem = new PBDUnifiedContactManifoldSystem();

    collisionSystem.update(noWrap.world, 0.016);
    collisionSystem.update(withWrap.world, 0.016);

    const noWrapContacts = noWrap.world.getResource('ball_obstacle_contacts');
    const withWrapContacts = withWrap.world.getResource('ball_obstacle_contacts');
    expect(noWrapContacts).toHaveLength(0);
    expect(withWrapContacts).toHaveLength(1);
    expect(withWrapContacts[0].raw_hit).toBe(false);
    expect(withWrap.world.hasComponent(withWrap.ballId, ScoredTagComponent)).toBe(false);

    const noWrapX = noWrap.world.getComponent(noWrap.ballId, PositionComponent).pos.x;
    const withWrapX = withWrap.world.getComponent(withWrap.ballId, PositionComponent).pos.x;
    expect(noWrapX).toBeCloseTo(0.0, 9);
    expect(withWrapX).toBeLessThan(0.0);
  });

  test('crossing CableJoint between ball and obstacle centers enforces one cable-width gap', () => {
    const rawRadius = 0.02;
    const cableHalfWidth = 0.0025;
    const distanceBelowCrossingThreshold = (2.0 * rawRadius) + (2.0 * cableHalfWidth) - 0.0005; // (2r + 2w) - eps

    const makeCrossingWorld = (pinchShareEnabled) => {
      const world = _makeWorld();
      world.setResource('layeringCollisionPinchShare', pinchShareEnabled);
      world.setResource('layeringCollisionSectorSolvers', false);
      world.setResource('layeringCollisionCircleSectors', false);
      world.setResource('layeringCollisionOverlayRadius', false);

      const ballId = world.createEntity();
      world.addComponent(ballId, new BallTagComponent());
      world.addComponent(ballId, new PositionComponent(0.0, 0.0));
      world.addComponent(ballId, new RadiusComponent(rawRadius));
      world.addComponent(ballId, new MassComponent(1.0));
      world.addComponent(ballId, new CableLinkComponent(0.0, 0.0, 0.0));

      const obsId = world.createEntity();
      world.addComponent(obsId, new ObstacleTagComponent());
      world.addComponent(obsId, new PositionComponent(distanceBelowCrossingThreshold, 0.0));
      world.addComponent(obsId, new RadiusComponent(rawRadius));
      world.addComponent(obsId, new ObstaclePushComponent(2.0));
      world.addComponent(obsId, new CableLinkComponent(distanceBelowCrossingThreshold, 0.0, 0.0));

      const midX = distanceBelowCrossingThreshold * 0.5;
      const top = world.createEntity();
      world.addComponent(top, new PositionComponent(midX, 0.05));
      world.addComponent(top, new CableLinkComponent(midX, 0.05, 0.0));
      const bottom = world.createEntity();
      world.addComponent(bottom, new PositionComponent(midX, -0.05));
      world.addComponent(bottom, new CableLinkComponent(midX, -0.05, 0.0));

      const crossingJoint = world.createEntity();
      world.addComponent(
        crossingJoint,
        CableJointComponent.fromWorld(
          top,
          bottom,
          0.1,
          new Vector2(midX, -0.05),
          new Vector2(midX, 0.05)
        )
      );

      const pathId = world.createEntity();
      world.addComponent(
        pathId,
        new CablePathComponent(
          world,
          [crossingJoint],
          ['hybrid', 'hybrid'],
          [true, false],
          1e6,
          [0.0, 0.0],
          cableHalfWidth
        )
      );
      return { world, ballId };
    };

    const withoutShare = makeCrossingWorld(false);
    const withShare = makeCrossingWorld(true);
    const collisionSystem = new PBDUnifiedContactManifoldSystem();

    collisionSystem.update(withoutShare.world, 0.016);
    collisionSystem.update(withShare.world, 0.016);

    expect(withoutShare.world.getComponent(withoutShare.ballId, PositionComponent).pos.x).toBeCloseTo(0.0, 9);
    expect((withoutShare.world.getResource('ball_obstacle_contacts') || []).length).toBe(0);

    expect(withShare.world.getComponent(withShare.ballId, PositionComponent).pos.x).toBeLessThan(0.0);
    const withShareContacts = withShare.world.getResource('ball_obstacle_contacts') || [];
    expect(withShareContacts.length).toBeGreaterThanOrEqual(1);
    expect(withShareContacts[0].pinch_shared).toBe(true);
  });

  test('non-direct CableJoint touching at center-segment endpoint does not trigger pinch-share for ball-obstacle', () => {
    const rawRadius = 0.02;
    const cableHalfWidth = 0.0025;
    const distanceBelowPinchThreshold = (2.0 * rawRadius) + (2.0 * cableHalfWidth) - 0.0005;

    const world = _makeWorld();
    world.setResource('layeringCollisionPinchShare', true);
    world.setResource('layeringCollisionSectorSolvers', false);
    world.setResource('layeringCollisionCircleSectors', false);
    world.setResource('layeringCollisionOverlayRadius', false);

    const ballId = world.createEntity();
    world.addComponent(ballId, new BallTagComponent());
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(rawRadius));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(ballId, new CableLinkComponent(0.0, 0.0, 0.0));

    const obsId = world.createEntity();
    world.addComponent(obsId, new ObstacleTagComponent());
    world.addComponent(obsId, new PositionComponent(distanceBelowPinchThreshold, 0.0));
    world.addComponent(obsId, new RadiusComponent(rawRadius));
    world.addComponent(obsId, new ObstaclePushComponent(2.0));
    world.addComponent(obsId, new CableLinkComponent(distanceBelowPinchThreshold, 0.0, 0.0));

    const top = world.createEntity();
    world.addComponent(top, new PositionComponent(0.0, 0.05));
    world.addComponent(top, new CableLinkComponent(0.0, 0.05, 0.0));
    const bottom = world.createEntity();
    world.addComponent(bottom, new PositionComponent(0.0, 0.0));
    world.addComponent(bottom, new CableLinkComponent(0.0, 0.0, 0.0));

    const touchingJoint = world.createEntity();
    world.addComponent(
      touchingJoint,
      CableJointComponent.fromWorld(
        top,
        bottom,
        0.05,
        new Vector2(0.0, 0.05),
        new Vector2(0.0, 0.0)
      )
    );

    const pathId = world.createEntity();
    world.addComponent(
      pathId,
      new CablePathComponent(
        world,
        [touchingJoint],
        ['hybrid', 'hybrid'],
        [true, false],
        1e6,
        [0.0, 0.0],
        cableHalfWidth
      )
    );

    const collisionSystem = new PBDUnifiedContactManifoldSystem();
    collisionSystem.update(world, 0.016);

    expect(world.getComponent(ballId, PositionComponent).pos.x).toBeCloseTo(0.0, 9);
    expect((world.getResource('ball_obstacle_contacts') || []).length).toBe(0);
  });

  test('ball-obstacle pinch-share uses attachment-time cache transform for moved CableJoint segments', () => {
    const rawRadius = 0.02;
    const cableHalfWidth = 0.0025;
    const distanceBelowPinchThreshold = (2.0 * rawRadius) + (2.0 * cableHalfWidth) - 0.0005;

    const world = _makeWorld();
    world.setResource('layeringCollisionPinchShare', true);
    world.setResource('layeringCollisionSectorSolvers', false);
    world.setResource('layeringCollisionCircleSectors', false);
    world.setResource('layeringCollisionOverlayRadius', false);

    const ballId = world.createEntity();
    world.addComponent(ballId, new BallTagComponent());
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(rawRadius));
    world.addComponent(ballId, new MassComponent(1.0));
    world.addComponent(ballId, new CableLinkComponent(0.0, 0.0, 0.0));

    const obsId = world.createEntity();
    world.addComponent(obsId, new ObstacleTagComponent());
    world.addComponent(obsId, new PositionComponent(distanceBelowPinchThreshold, 0.0));
    world.addComponent(obsId, new RadiusComponent(rawRadius));
    world.addComponent(obsId, new ObstaclePushComponent(2.0));
    world.addComponent(obsId, new CableLinkComponent(distanceBelowPinchThreshold, 0.0, 0.0));

    const top = world.createEntity();
    world.addComponent(top, new PositionComponent(distanceBelowPinchThreshold * 0.5, 0.05));
    world.addComponent(top, new CableLinkComponent(0.10, 0.05, 0.0));
    const bottom = world.createEntity();
    world.addComponent(bottom, new PositionComponent(distanceBelowPinchThreshold * 0.5, -0.05));
    world.addComponent(bottom, new CableLinkComponent(0.10, -0.05, 0.0));

    const topLink = world.getComponent(top, CableLinkComponent);
    const bottomLink = world.getComponent(bottom, CableLinkComponent);
    topLink.prevCableAttachmentTimePos.set({ x: 0.10, y: 0.05 });
    bottomLink.prevCableAttachmentTimePos.set({ x: 0.10, y: -0.05 });
    topLink.prevCableAttachmentTimeAngle = 0.0;
    bottomLink.prevCableAttachmentTimeAngle = 0.0;

    const crossingJoint = world.createEntity();
    world.addComponent(
      crossingJoint,
      CableJointComponent.fromWorld(
        top,
        bottom,
        0.1,
        new Vector2(0.10, 0.05),
        new Vector2(0.10, -0.05)
      )
    );

    const pathId = world.createEntity();
    world.addComponent(
      pathId,
      new CablePathComponent(
        world,
        [crossingJoint],
        ['hybrid', 'hybrid'],
        [true, false],
        1e6,
        [0.0, 0.0],
        cableHalfWidth
      )
    );

    const collisionSystem = new PBDUnifiedContactManifoldSystem();
    collisionSystem.update(world, 0.016);

    expect(world.getComponent(ballId, PositionComponent).pos.x).toBeLessThan(0.0);
    const contacts = world.getResource('ball_obstacle_contacts') || [];
    expect(contacts.length).toBeGreaterThanOrEqual(1);
    expect(contacts[0].pinch_shared).toBe(true);
  });
});
