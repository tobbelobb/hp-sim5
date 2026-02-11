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
  ScoreComponent,
  ScoreSystem,
  ScoredTagComponent,
  PBDUnifiedContactManifoldSystem,
  OverlayRadiusComponent,
} from '../../../examples/js/flipper/flipper_common.js';

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
});
