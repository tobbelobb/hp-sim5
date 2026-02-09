import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';
import {
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  FlipperTagComponent,
  ScoreComponent,
  ScoreSystem,
  ScoredTagComponent,
  PBDBallObstacleCollisions,
} from '../../../examples/js/flipper/flipper_common.js';

function addEndpointHybridWrap(world, endpointId, storedLength, halfWidth = 0.1) {
  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(endpointId + 10.0, 0.0));
  world.addComponent(anchorId, new RadiusComponent(0.1));
  world.addComponent(anchorId, new CableLinkComponent(endpointId + 10.0, 0.0));

  const endpointPos = world.getComponent(endpointId, PositionComponent).pos;
  const endpointRadius = world.getComponent(endpointId, RadiusComponent).radius;
  const baseRadius = endpointRadius + halfWidth;
  const attachmentA = endpointPos.clone().add(new Vector2(-baseRadius, 0.0));
  const attachmentB = attachmentA.clone().add(new Vector2(-0.8, 0.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      endpointId,
      anchorId,
      0.0,
      attachmentA,
      attachmentB
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

describe('PBDBallObstacleCollisions scoring behavior', () => {
  test('scores only on obstacle contact enter, not every frame of persistent overlap', () => {
    const world = new World();

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

    const collisionSystem = new PBDBallObstacleCollisions();
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

  test('ignores obstacle contacts on entities that are also tagged as flippers', () => {
    const world = new World();

    const ballId = world.createEntity();
    world.addComponent(ballId, new BallTagComponent());
    world.addComponent(ballId, new PositionComponent(0.0, 0.0));
    world.addComponent(ballId, new RadiusComponent(1.0));
    world.addComponent(ballId, new MassComponent(1.0));

    const hybridObstacleFlipperId = world.createEntity();
    world.addComponent(hybridObstacleFlipperId, new ObstacleTagComponent());
    world.addComponent(hybridObstacleFlipperId, new FlipperTagComponent());
    world.addComponent(hybridObstacleFlipperId, new PositionComponent(1.8, 0.0));
    world.addComponent(hybridObstacleFlipperId, new RadiusComponent(1.0));
    world.addComponent(hybridObstacleFlipperId, new ObstaclePushComponent(2.0));

    const collisionSystem = new PBDBallObstacleCollisions();
    collisionSystem.update(world, 0.016);

    const contacts = world.getResource('ball_obstacle_contacts');
    expect(Array.isArray(contacts)).toBe(true);
    expect(contacts).toHaveLength(0);
    expect(world.hasComponent(ballId, ScoredTagComponent)).toBe(false);
  });

  test('uses effective wrapped radius for obstacle side in circle-circle collision', () => {
    const baseLayerCircumference = 2.0 * Math.PI * 1.1;
    const collisionSystem = new PBDBallObstacleCollisions();

    const makeWorld = (withWrap) => {
      const world = new World();

      const ballId = world.createEntity();
      world.addComponent(ballId, new BallTagComponent());
      world.addComponent(ballId, new PositionComponent(0.0, 0.0));
      world.addComponent(ballId, new RadiusComponent(1.0));
      world.addComponent(ballId, new MassComponent(1.0));
      world.addComponent(ballId, new CableLinkComponent(0.0, 0.0));

      const obsId = world.createEntity();
      world.addComponent(obsId, new ObstacleTagComponent());
      world.addComponent(obsId, new PositionComponent(2.15, 0.0));
      world.addComponent(obsId, new RadiusComponent(1.0));
      world.addComponent(obsId, new ObstaclePushComponent(2.0));
      world.addComponent(obsId, new CableLinkComponent(2.15, 0.0));

      if (withWrap) {
        addEndpointHybridWrap(world, obsId, baseLayerCircumference);
      }
      return { world, ballId };
    };

    const noWrap = makeWorld(false);
    const withWrap = makeWorld(true);

    collisionSystem.update(noWrap.world, 0.016);
    collisionSystem.update(withWrap.world, 0.016);

    const noWrapContacts = noWrap.world.getResource('ball_obstacle_contacts');
    const withWrapContacts = withWrap.world.getResource('ball_obstacle_contacts');
    expect(noWrapContacts).toHaveLength(0);
    expect(withWrapContacts).toHaveLength(1);
    expect(withWrapContacts[0].raw_contact).toBe(false);
    expect(withWrap.world.hasComponent(withWrap.ballId, ScoredTagComponent)).toBe(false);

    const noWrapX = noWrap.world.getComponent(noWrap.ballId, PositionComponent).pos.x;
    const withWrapX = withWrap.world.getComponent(withWrap.ballId, PositionComponent).pos.x;
    expect(noWrapX).toBeCloseTo(0.0, 9);
    expect(withWrapX).toBeLessThan(0.0);
  });
});
