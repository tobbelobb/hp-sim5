import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import {
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  OverlayRadiusComponent,
  PBDUnifiedContactManifoldSystem
} from '../../../examples/js/flipper_3d/flipper_common_3d.js';

function makeWorld() {
  const world = new World();
  world.setResource('enableLayering', true);
  world.setResource('layeringCollisionSectorSolvers', false);
  world.setResource('layeringCollisionCircleSectors', false);
  world.setResource('layeringCollisionOverlayRadius', true);
  world.setResource('layeringCollisionPinchShare', true);
  return world;
}

function addBall(world, x, y, radius = 0.02) {
  const id = world.createEntity();
  world.addComponent(id, new BallTagComponent());
  world.addComponent(id, new PositionComponent(x, y, 0.0));
  world.addComponent(id, new RadiusComponent(radius));
  world.addComponent(id, new MassComponent(1.0));
  world.addComponent(id, new CableLinkComponent(x, y, 0.0));
  return id;
}

function addCrossingPinchPath(world, centerDistance, cableHalfWidth) {
  const midX = centerDistance * 0.5;
  const topId = world.createEntity();
  world.addComponent(topId, new PositionComponent(midX, 0.05, 0.0));
  world.addComponent(topId, new CableLinkComponent(midX, 0.05, 0.0));

  const bottomId = world.createEntity();
  world.addComponent(bottomId, new PositionComponent(midX, -0.05, 0.0));
  world.addComponent(bottomId, new CableLinkComponent(midX, -0.05, 0.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    CableJointComponent.fromWorld(
      topId,
      bottomId,
      0.1,
      new Vector3(midX, 0.05, 0.0),
      new Vector3(midX, -0.05, 0.0)
    )
  );

  const pathId = world.createEntity();
  world.addComponent(
    pathId,
    new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'hybrid'],
      [true, false],
      1e6,
      [0.0, 0.0],
      cableHalfWidth
    )
  );
}

function runManifold(world) {
  new PBDUnifiedContactManifoldSystem().update(world, 0.016);
}

describe('PBDUnifiedContactManifoldSystem (3D) pinch-share layering', () => {
  test('ball-ball pinch-share keeps using layered overlay radius', () => {
    const rawRadius = 0.02;
    const cableHalfWidth = 0.0025;
    const overlayRadius = 0.03;
    const centerDistance = 0.05;

    const world = makeWorld();
    const leftId = addBall(world, 0.0, 0.0, rawRadius);
    const rightId = addBall(world, centerDistance, 0.0, rawRadius);
    world.addComponent(leftId, new OverlayRadiusComponent(overlayRadius));
    world.addComponent(rightId, new OverlayRadiusComponent(overlayRadius));
    addCrossingPinchPath(world, centerDistance, cableHalfWidth);

    runManifold(world);

    const contacts = world.getResource('ball_ball_contacts') || [];
    expect(contacts).toHaveLength(1);
    expect(contacts[0].pinch_shared).toBe(true);
    expect(contacts[0].radius_a).toBeCloseTo(overlayRadius + cableHalfWidth, 9);
    expect(contacts[0].radius_b).toBeCloseTo(overlayRadius + cableHalfWidth, 9);
    expect(world.getComponent(leftId, PositionComponent).pos.x).toBeLessThan(0.0);
    expect(world.getComponent(rightId, PositionComponent).pos.x).toBeGreaterThan(centerDistance);
  });

  test('ball-obstacle pinch-share keeps using layered overlay radius', () => {
    const rawRadius = 0.02;
    const cableHalfWidth = 0.0025;
    const overlayRadius = 0.03;
    const centerDistance = 0.055;

    const world = makeWorld();
    const ballId = addBall(world, 0.0, 0.0, rawRadius);
    world.addComponent(ballId, new OverlayRadiusComponent(overlayRadius));

    const obstacleId = world.createEntity();
    world.addComponent(obstacleId, new ObstacleTagComponent());
    world.addComponent(obstacleId, new PositionComponent(centerDistance, 0.0, 0.0));
    world.addComponent(obstacleId, new RadiusComponent(rawRadius));
    world.addComponent(obstacleId, new ObstaclePushComponent(2.0));
    world.addComponent(obstacleId, new OverlayRadiusComponent(overlayRadius));
    world.addComponent(obstacleId, new CableLinkComponent(centerDistance, 0.0, 0.0));

    addCrossingPinchPath(world, centerDistance, cableHalfWidth);

    runManifold(world);

    const contacts = world.getResource('ball_obstacle_contacts') || [];
    expect(contacts).toHaveLength(1);
    expect(contacts[0].pinch_shared).toBe(true);
    expect(contacts[0].ball_contact_radius).toBeCloseTo(overlayRadius + cableHalfWidth, 9);
    expect(contacts[0].obstacle_contact_radius).toBeCloseTo(overlayRadius + cableHalfWidth, 9);
    expect(contacts[0].raw_hit).toBe(false);
    expect(world.getComponent(ballId, PositionComponent).pos.x).toBeLessThan(0.0);
  });
});
