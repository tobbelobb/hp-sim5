import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  PinchDetectionSystem,
  PinchConfigureSystem,
  PinchConstraintBuildSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints/cable_joints_core.js';

function setupEndpointPinchWorld({
  storedLength = 0.5,
  pincherAngle = -0.2,
  pincherSurfaceDistance = 0.05
} = {}) {
  const world = new World();

  const spool = world.createEntity();
  const neighbor = world.createEntity();
  const pincher = world.createEntity();

  const spoolPos = new Vector2(0.0, 0.0);
  const neighborPos = new Vector2(0.0, 3.0);
  const spoolRadius = 1.0;
  const pincherRadius = 0.5;
  const cableHalfWidth = 0.05;
  const baseRadius = spoolRadius + cableHalfWidth;

  world.addComponent(spool, new PositionComponent(spoolPos.x, spoolPos.y));
  world.addComponent(spool, new RadiusComponent(spoolRadius));
  world.addComponent(spool, new CableLinkComponent(spoolPos.x, spoolPos.y));
  world.addComponent(spool, new MassComponent(1.0));

  const centerDistance = spoolRadius + pincherRadius + pincherSurfaceDistance;
  const pincherPos = new Vector2(
    spoolPos.x + centerDistance * Math.cos(pincherAngle),
    spoolPos.y + centerDistance * Math.sin(pincherAngle)
  );
  world.addComponent(pincher, new PositionComponent(pincherPos.x, pincherPos.y));
  world.addComponent(pincher, new RadiusComponent(pincherRadius));
  world.addComponent(pincher, new CableLinkComponent(pincherPos.x, pincherPos.y));
  world.addComponent(pincher, new MassComponent(1.0));

  world.addComponent(neighbor, new PositionComponent(neighborPos.x, neighborPos.y));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      spool,
      neighbor,
      3.2,
      new Vector2(baseRadius, 0.0),
      neighborPos.clone()
    )
  );

  const pathId = world.createEntity();
  const path = new CablePathComponent(
    world,
    [jointId],
    ['hybrid', 'attachment'],
    [false, false],
    Infinity,
    null,
    cableHalfWidth
  );
  path.stored[0] = storedLength;
  path.totalRestLength += storedLength;
  world.addComponent(pathId, path);
  world.setResource('dt', 0.016);

  return {
    world,
    pathId,
    spool,
    pincher
  };
}

describe('Endpoint non-transitional pinch joints', () => {
  test('detects and enforces pinch contact for wrapped first-link hybrid endpoint', () => {
    const { world, spool, pincher } = setupEndpointPinchWorld();

    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    const build = new PinchConstraintBuildSystem();
    const solver = new PBDCableConstraintSolver();

    detect.update(world, 0.016);
    const candidates = world.getResource('cablePinchCandidates');
    const endpointCandidates = candidates.filter((c) => c.kind === 'non-transitional-endpoint');
    expect(endpointCandidates).toHaveLength(1);
    expect(endpointCandidates[0].entityA).toBe(spool);
    expect(endpointCandidates[0].entityB).toBe(pincher);
    expect(endpointCandidates[0].minDistance).toBeCloseTo(0.1, 9);

    configure.update(world, 0.016);
    const pinchConfigs = world.getResource('cablePinchJointConfigs');
    const endpointConfigs = Array.from(pinchConfigs.values()).filter((config) =>
      config.entityA === spool && config.entityB === pincher
    );
    expect(endpointConfigs).toHaveLength(1);
    expect(endpointConfigs[0].minDistance).toBeCloseTo(0.1, 9);

    build.update(world, 0.016);
    solver.update(world, 0.016);

    const posA = world.getComponent(spool, PositionComponent).pos;
    const posB = world.getComponent(pincher, PositionComponent).pos;
    const rA = world.getComponent(spool, RadiusComponent).radius;
    const rB = world.getComponent(pincher, RadiusComponent).radius;
    const surfaceDistance = posA.distanceTo(posB) - (rA + rB);
    expect(surfaceDistance).toBeGreaterThanOrEqual(0.1 - 1e-6);
  });

  test('does not detect endpoint pinch when body is outside wrapped hybrid arc', () => {
    const { world } = setupEndpointPinchWorld({
      storedLength: 0.5,
      pincherAngle: 0.2,
      pincherSurfaceDistance: 0.05
    });

    const detect = new PinchDetectionSystem();
    detect.update(world, 0.016);
    const candidates = world.getResource('cablePinchCandidates');
    const endpointCandidates = candidates.filter((c) => c.kind === 'non-transitional-endpoint');
    expect(endpointCandidates).toHaveLength(0);
  });

  test('increases endpoint pinch minDistance when stored cable spans multiple full layers', () => {
    const { world, spool, pincher } = setupEndpointPinchWorld({
      storedLength: 14.0,
      pincherAngle: -1.0,
      pincherSurfaceDistance: 0.18
    });

    const detect = new PinchDetectionSystem();
    detect.update(world, 0.016);
    const candidates = world.getResource('cablePinchCandidates');
    const endpointCandidates = candidates.filter((c) => c.kind === 'non-transitional-endpoint');
    expect(endpointCandidates).toHaveLength(1);
    expect(endpointCandidates[0].entityA).toBe(spool);
    expect(endpointCandidates[0].entityB).toBe(pincher);
    expect(endpointCandidates[0].minDistance).toBeGreaterThan(0.1);
    expect(endpointCandidates[0].minDistance).toBeCloseTo(0.2, 6);
  });
});
