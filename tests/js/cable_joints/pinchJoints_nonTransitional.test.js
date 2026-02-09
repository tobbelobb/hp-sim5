import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint
} from '../../../src/js/cable_joints/geometry.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  PinchDetectionSystem,
  PinchConfigureSystem,
  PinchConstraintBuildSystem,
  PBDCableConstraintSolver,
  _mergeJoints
} from '../../../src/js/cable_joints/cable_joints_core.js';

function setupNonTransitionalPinchWorld() {
  const world = new World();

  const anchorL = world.createEntity();
  const anchorR = world.createEntity();
  const wrappedA = world.createEntity();
  const pincherB = world.createEntity();

  const posL = new Vector2(-3.0, 1.5);
  const posR = new Vector2(3.0, 1.5);
  const posA = new Vector2(0.0, 0.0);
  const posB = new Vector2(0.0, -1.95);
  const radiusA = 1.0;
  const radiusB = 0.75;
  const cableHalfWidth = 0.1;
  const effectiveRadiusA = radiusA + cableHalfWidth;
  const cwAroundA = false;

  world.addComponent(anchorL, new PositionComponent(posL.x, posL.y));
  world.addComponent(anchorR, new PositionComponent(posR.x, posR.y));

  world.addComponent(wrappedA, new PositionComponent(posA.x, posA.y));
  world.addComponent(wrappedA, new RadiusComponent(radiusA));
  world.addComponent(wrappedA, new CableLinkComponent(posA.x, posA.y));
  world.addComponent(wrappedA, new MassComponent(1.0));

  world.addComponent(pincherB, new PositionComponent(posB.x, posB.y));
  world.addComponent(pincherB, new RadiusComponent(radiusB));
  world.addComponent(pincherB, new CableLinkComponent(posB.x, posB.y));
  world.addComponent(pincherB, new MassComponent(1.0));

  const tangL = tangentFromPointToCircle(posL, posA, effectiveRadiusA, cwAroundA);
  const tangR = tangentFromCircleToPoint(posR, posA, effectiveRadiusA, cwAroundA);

  const leftJointId = world.createEntity();
  world.addComponent(
    leftJointId,
    new CableJointComponent(
      anchorL,
      wrappedA,
      tangL.a_attach.distanceTo(tangL.a_circle),
      tangL.a_attach.clone(),
      tangL.a_circle.clone()
    )
  );

  const rightJointId = world.createEntity();
  world.addComponent(
    rightJointId,
    new CableJointComponent(
      wrappedA,
      anchorR,
      tangR.a_attach.distanceTo(tangR.a_circle),
      tangR.a_circle.clone(),
      tangR.a_attach.clone()
    )
  );

  const pathId = world.createEntity();
  const path = new CablePathComponent(
    world,
    [leftJointId, rightJointId],
    ['attachment', 'rolling', 'attachment'],
    [false, cwAroundA, false],
    Infinity,
    null,
    cableHalfWidth
  );
  world.addComponent(pathId, path);
  world.setResource('dt', 0.016);

  return {
    world,
    pathId,
    wrappedA,
    pincherB
  };
}

describe('Non-transitional pinch joints', () => {
  test('inserts A->B and B->A transitional joints when body pinches stored cable on A', () => {
    const { world, pathId, wrappedA, pincherB } = setupNonTransitionalPinchWorld();
    const path = world.getComponent(pathId, CablePathComponent);
    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    const build = new PinchConstraintBuildSystem();

    expect(path.jointEntities).toHaveLength(2);
    expect(path.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
    expect(path.stored[1]).toBeGreaterThan(0.0);

    detect.update(world, 0.016);
    const candidates = world.getResource('cablePinchCandidates');
    const nonTransitional = candidates.filter((c) => c.kind === 'non-transitional');
    expect(nonTransitional).toHaveLength(1);
    expect(nonTransitional[0].entityA).toBe(wrappedA);
    expect(nonTransitional[0].entityB).toBe(pincherB);
    expect(nonTransitional[0].linkIndex).toBe(1);

    configure.update(world, 0.016);
    expect(path.jointEntities).toHaveLength(4);
    expect(path.linkTypes).toEqual(['attachment', 'rolling', 'rolling', 'rolling', 'attachment']);
    expect(path.stored).toHaveLength(5);
    expect(path.stored[2]).toBeCloseTo(1e-5, 10);

    const joints = path.jointEntities.map((jointId) => world.getComponent(jointId, CableJointComponent));
    expect(joints[1].entityA).toBe(wrappedA);
    expect(joints[1].entityB).toBe(pincherB);
    expect(joints[2].entityA).toBe(pincherB);
    expect(joints[2].entityB).toBe(wrappedA);

    const pinchConfigs = world.getResource('cablePinchJointConfigs');
    expect(pinchConfigs instanceof Map).toBe(true);
    expect(pinchConfigs.size).toBe(2);
    expect(pinchConfigs.has(path.jointEntities[1])).toBe(true);
    expect(pinchConfigs.has(path.jointEntities[2])).toBe(true);

    build.update(world, 0.016);
    const contacts = world.getResource('cablePinchContacts');
    expect(contacts).toHaveLength(1);
    expect([contacts[0].entityA, contacts[0].entityB]).toEqual([Math.min(wrappedA, pincherB), Math.max(wrappedA, pincherB)]);
  });

  test('pinch-specific merge removes A->B and B->A pair when middle stored cable goes negative', () => {
    const { world, pathId, wrappedA, pincherB } = setupNonTransitionalPinchWorld();
    const path = world.getComponent(pathId, CablePathComponent);
    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();

    const initialTotalRestLength = path.totalRestLength;
    detect.update(world, 0.016);
    configure.update(world, 0.016);
    expect(path.jointEntities).toHaveLength(4);

    path.stored[2] = -1e-6;
    _mergeJoints(world);

    expect(path.jointEntities).toHaveLength(2);
    expect(path.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
    expect(path.totalRestLength).toBeCloseTo(initialTotalRestLength, 9);

    const joints = path.jointEntities.map((jointId) => world.getComponent(jointId, CableJointComponent));
    expect(joints[0].entityB).toBe(wrappedA);
    expect(joints[1].entityA).toBe(wrappedA);
    expect(joints[0].entityA).not.toBe(pincherB);
    expect(joints[1].entityB).not.toBe(pincherB);
  });

  test('contact inequality solver enforces d >= 2w for non-transitional pinch', () => {
    const { world, wrappedA, pincherB } = setupNonTransitionalPinchWorld();
    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    const build = new PinchConstraintBuildSystem();
    const solver = new PBDCableConstraintSolver();

    detect.update(world, 0.016);
    configure.update(world, 0.016);
    build.update(world, 0.016);
    solver.update(world, 0.016);

    const posA = world.getComponent(wrappedA, PositionComponent).pos;
    const posB = world.getComponent(pincherB, PositionComponent).pos;
    const rA = world.getComponent(wrappedA, RadiusComponent).radius;
    const rB = world.getComponent(pincherB, RadiusComponent).radius;
    const surfaceDistance = posA.distanceTo(posB) - (rA + rB);

    expect(surfaceDistance).toBeGreaterThanOrEqual(0.2 - 1e-6);
  });
});
