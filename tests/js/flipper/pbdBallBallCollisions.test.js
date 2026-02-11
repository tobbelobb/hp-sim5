import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  BallTagComponent,
  CircleSectorComponent,
  OverlayRadiusAndCircleSectorSystem,
  PBDUnifiedContactManifoldSystem,
} from '../../../examples/js/flipper/flipper_common.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';

function _addBall(world, x, y, radius = 1.0, mass = 1.0) {
  const id = world.createEntity();
  world.addComponent(id, new BallTagComponent());
  world.addComponent(id, new PositionComponent(x, y));
  world.addComponent(id, new RadiusComponent(radius));
  world.addComponent(id, new MassComponent(mass));
  return id;
}

function _baseWorld() {
  const world = new World();
  world.setResource('enableLayering', true);
  world.setResource('layeringCollisionSectorSolvers', true);
  world.setResource('layeringCollisionCircleSectors', true);
  world.setResource('layeringCollisionOverlayRadius', true);
  world.setResource('layeringCollisionPinchShare', true);
  return world;
}

function _runManifold(world) {
  const system = new PBDUnifiedContactManifoldSystem();
  system.update(world, 0.016);
}

function _makePinchPairWorld({
  centerDistance,
  rawRadius = 0.02,
  cableHalfWidth = 0.0025,
  storedLength = 0.06,
  pinchShareEnabled = true,
} = {}) {
  const world = _baseWorld();
  world.setResource('layeringCollisionPinchShare', pinchShareEnabled);

  const left = _addBall(world, 0.0, 0.0, rawRadius, 1.0);
  const right = _addBall(world, centerDistance, 0.0, rawRadius, 1.0);
  world.addComponent(left, new CableLinkComponent(0.0, 0.0, 0.0));
  world.addComponent(right, new CableLinkComponent(centerDistance, 0.0, 0.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    CableJointComponent.fromWorld(
      left,
      right,
      0.03,
      new Vector2(rawRadius, 0.0),
      new Vector2(centerDistance - rawRadius, 0.0)
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
      [storedLength, storedLength],
      cableHalfWidth
    )
  );

  new OverlayRadiusAndCircleSectorSystem().update(world, 0.016);

  return { world, left, right, rawRadius, cableHalfWidth };
}

function _makeCrossingSegmentPinchWorld({
  centerDistance,
  rawRadius = 0.02,
  cableHalfWidth = 0.0025,
  pinchShareEnabled = true,
} = {}) {
  const world = _baseWorld();
  world.setResource('layeringCollisionPinchShare', pinchShareEnabled);
  world.setResource('layeringCollisionSectorSolvers', false);
  world.setResource('layeringCollisionCircleSectors', false);
  world.setResource('layeringCollisionOverlayRadius', false);

  const left = _addBall(world, 0.0, 0.0, rawRadius, 1.0);
  const right = _addBall(world, centerDistance, 0.0, rawRadius, 1.0);
  world.addComponent(left, new CableLinkComponent(0.0, 0.0, 0.0));
  world.addComponent(right, new CableLinkComponent(centerDistance, 0.0, 0.0));

  const midX = centerDistance * 0.5;
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

  return { world, left, right };
}

describe('PBDUnifiedContactManifoldSystem ball-ball behavior', () => {
  test('no correction when there is no overlap', () => {
    const world = _baseWorld();
    const left = _addBall(world, 0.0, 0.0, 1.0, 1.0);
    const right = _addBall(world, 2.2, 0.0, 1.0, 1.0);

    _runManifold(world);

    expect(world.getComponent(left, PositionComponent).pos.x).toBeCloseTo(0.0, 9);
    expect(world.getComponent(right, PositionComponent).pos.x).toBeCloseTo(2.2, 9);
    expect((world.getResource('ball_ball_contacts') || []).length).toBe(0);
  });

  test('directional circle sectors can trigger contact only when sector faces the pair direction', () => {
    const worldA = _baseWorld();
    const leftA = _addBall(worldA, 0.0, 0.0, 1.0, 1.0);
    const rightA = _addBall(worldA, 2.15, 0.0, 1.0, 1.0);
    worldA.addComponent(leftA, new CircleSectorComponent(1.2, -0.3, 0.3, false));
    worldA.addComponent(rightA, new CircleSectorComponent(1.2, Math.PI - 0.3, Math.PI + 0.3, false));

    const worldB = _baseWorld();
    const leftB = _addBall(worldB, 0.0, 0.0, 1.0, 1.0);
    const rightB = _addBall(worldB, 2.15, 0.0, 1.0, 1.0);
    worldB.addComponent(leftB, new CircleSectorComponent(1.2, Math.PI - 0.3, Math.PI + 0.3, false));
    worldB.addComponent(rightB, new CircleSectorComponent(1.2, -0.3, 0.3, false));

    _runManifold(worldA);
    _runManifold(worldB);

    expect(worldA.getComponent(leftA, PositionComponent).pos.x).toBeLessThan(0.0);
    expect(worldA.getComponent(rightA, PositionComponent).pos.x).toBeGreaterThan(2.15);
    expect((worldA.getResource('ball_ball_contacts') || []).length).toBeGreaterThanOrEqual(1);

    expect(worldB.getComponent(leftB, PositionComponent).pos.x).toBeCloseTo(0.0, 9);
    expect(worldB.getComponent(rightB, PositionComponent).pos.x).toBeCloseTo(2.15, 9);
    expect((worldB.getResource('ball_ball_contacts') || []).length).toBe(0);
  });

  test('direct pinched pair at exactly 2r+2w has no penetration for either mode', () => {
    const rawRadius = 0.02;
    const halfWidth = 0.0025;
    const distanceAtPhysicalCableThickness = (2.0 * rawRadius) + (2.0 * halfWidth); // 2r + 2w

    const withoutShare = _makePinchPairWorld({
      centerDistance: distanceAtPhysicalCableThickness,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: false
    });
    const withShare = _makePinchPairWorld({
      centerDistance: distanceAtPhysicalCableThickness,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: true
    });

    _runManifold(withoutShare.world);
    _runManifold(withShare.world);

    const leftNoShareX = withoutShare.world.getComponent(withoutShare.left, PositionComponent).pos.x;
    const rightNoShareX = withoutShare.world.getComponent(withoutShare.right, PositionComponent).pos.x;
    const leftShareX = withShare.world.getComponent(withShare.left, PositionComponent).pos.x;
    const rightShareX = withShare.world.getComponent(withShare.right, PositionComponent).pos.x;

    expect(leftNoShareX).toBeCloseTo(0.0, 9);
    expect(rightNoShareX).toBeCloseTo(distanceAtPhysicalCableThickness, 9);
    expect(leftShareX).toBeCloseTo(0.0, 9);
    expect(rightShareX).toBeCloseTo(distanceAtPhysicalCableThickness, 9);

    const contactsNoShare = withoutShare.world.getResource('ball_ball_contacts') || [];
    const contactsShare = withShare.world.getResource('ball_ball_contacts') || [];
    expect(contactsNoShare.length).toBe(0);
    expect(contactsShare.length).toBe(0);
  });

  test('below 2r+2w, pinch-share ON resolves while OFF does not for direct pinched pair', () => {
    const rawRadius = 0.02;
    const halfWidth = 0.0025;
    const belowThresholdDistance = (2.0 * rawRadius) + (2.0 * halfWidth) - 0.001;

    const withoutShare = _makePinchPairWorld({
      centerDistance: belowThresholdDistance,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: false
    });
    const withShare = _makePinchPairWorld({
      centerDistance: belowThresholdDistance,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: true
    });

    _runManifold(withoutShare.world);
    _runManifold(withShare.world);

    const withoutShareContacts = withoutShare.world.getResource('ball_ball_contacts') || [];
    const withShareContacts = withShare.world.getResource('ball_ball_contacts') || [];

    expect(withoutShare.world.getComponent(withoutShare.left, PositionComponent).pos.x).toBeCloseTo(0.0, 9);
    expect(withoutShare.world.getComponent(withoutShare.right, PositionComponent).pos.x).toBeCloseTo(belowThresholdDistance, 9);
    expect(withoutShareContacts.length).toBe(0);

    expect(withShare.world.getComponent(withShare.left, PositionComponent).pos.x).toBeLessThan(0.0);
    expect(withShare.world.getComponent(withShare.right, PositionComponent).pos.x).toBeGreaterThan(belowThresholdDistance);
    expect(withShareContacts.length).toBeGreaterThanOrEqual(1);
    expect(withShareContacts[0].pinch_shared).toBe(true);
  });

  test('pinch-share ON still enforces contact below 2r+2w threshold', () => {
    const rawRadius = 0.02;
    const halfWidth = 0.0025;
    const belowThresholdDistance = (2.0 * rawRadius) + (2.0 * halfWidth) - 0.001;
    const worldState = _makePinchPairWorld({
      centerDistance: belowThresholdDistance,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: true
    });

    _runManifold(worldState.world);

    const leftX = worldState.world.getComponent(worldState.left, PositionComponent).pos.x;
    const rightX = worldState.world.getComponent(worldState.right, PositionComponent).pos.x;
    expect(leftX).toBeLessThan(0.0);
    expect(rightX).toBeGreaterThan(belowThresholdDistance);

    const contacts = worldState.world.getResource('ball_ball_contacts') || [];
    expect(contacts.length).toBeGreaterThanOrEqual(1);
    expect(contacts[0].pinch_shared).toBe(true);
  });

  test('crossing CableJoint between centers enforces one cable-width gap even without sector inflation', () => {
    const rawRadius = 0.02;
    const halfWidth = 0.0025;
    const distanceBelowCrossingThreshold = (2.0 * rawRadius) + (2.0 * halfWidth) - 0.0005; // (2r + 2w) - eps

    const withoutShare = _makeCrossingSegmentPinchWorld({
      centerDistance: distanceBelowCrossingThreshold,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: false
    });
    const withShare = _makeCrossingSegmentPinchWorld({
      centerDistance: distanceBelowCrossingThreshold,
      rawRadius,
      cableHalfWidth: halfWidth,
      pinchShareEnabled: true
    });

    _runManifold(withoutShare.world);
    _runManifold(withShare.world);

    expect(withoutShare.world.getComponent(withoutShare.left, PositionComponent).pos.x).toBeCloseTo(0.0, 9);
    expect(withoutShare.world.getComponent(withoutShare.right, PositionComponent).pos.x).toBeCloseTo(distanceBelowCrossingThreshold, 9);
    expect((withoutShare.world.getResource('ball_ball_contacts') || []).length).toBe(0);

    expect(withShare.world.getComponent(withShare.left, PositionComponent).pos.x).toBeLessThan(0.0);
    expect(withShare.world.getComponent(withShare.right, PositionComponent).pos.x).toBeGreaterThan(distanceBelowCrossingThreshold);
    const contacts = withShare.world.getResource('ball_ball_contacts') || [];
    expect(contacts.length).toBeGreaterThanOrEqual(1);
    expect(contacts[0].pinch_shared).toBe(true);
  });
});
