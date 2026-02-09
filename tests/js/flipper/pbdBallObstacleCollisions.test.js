import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
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
});
