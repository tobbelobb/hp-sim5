import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  MassComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
  PinchDetectionSystem,
  PinchConfigureSystem,
  PinchConstraintBuildSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints/cable_joints_core.js';

function setupTransitionalPinchWorld() {
  const world = new World();
  const entityA = world.createEntity();
  const entityB = world.createEntity();
  world.addComponent(entityA, new PositionComponent(0.0, 0.0));
  world.addComponent(entityB, new PositionComponent(0.34, 0.0));
  world.addComponent(entityA, new RadiusComponent(0.15));
  world.addComponent(entityB, new RadiusComponent(0.15));
  world.addComponent(entityA, new MassComponent(1.0));
  world.addComponent(entityB, new MassComponent(1.0));

  const jointId = world.createEntity();
  world.addComponent(
    jointId,
    new CableJointComponent(
      entityA,
      entityB,
      0.0,
      new Vector2(0.15, 0.0),
      new Vector2(0.19, 0.0)
    )
  );

  const pathId = world.createEntity();
  world.addComponent(
    pathId,
    new CablePathComponent(
      world,
      [jointId],
      ['rolling', 'rolling'],
      [true, true],
      Infinity,
      null,
      0.05
    )
  );
  world.setResource('dt', 0.016);

  return { world, entityA, entityB, jointId };
}

describe('Transitional pinch joints', () => {
  test('detects and configures a transitional pinch with margined attachment points', () => {
    const { world, jointId } = setupTransitionalPinchWorld();
    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    const build = new PinchConstraintBuildSystem();

    detect.update(world, 0.016);
    const candidates = world.getResource('cablePinchCandidates');
    expect(candidates).toHaveLength(1);

    configure.update(world, 0.016);
    const pinchConfigs = world.getResource('cablePinchJointConfigs');
    expect(pinchConfigs instanceof Map).toBe(true);
    expect(pinchConfigs.size).toBe(1);

    const joint = world.getComponent(jointId, CableJointComponent);
    const posA = world.getComponent(joint.entityA, PositionComponent).pos;
    const posB = world.getComponent(joint.entityB, PositionComponent).pos;
    const rEffA = world.getComponent(joint.entityA, RadiusComponent).radius + 0.05;
    const rEffB = world.getComponent(joint.entityB, RadiusComponent).radius + 0.05;
    expect(joint.attachmentPointA_world.distanceTo(posA)).toBeCloseTo(rEffA, 7);
    expect(joint.attachmentPointB_world.distanceTo(posB)).toBeCloseTo(rEffB, 7);

    const pinchConfig = pinchConfigs.get(jointId);
    expect(pinchConfig).toBeDefined();
    expect(Math.abs(pinchConfig.segmentDir.dot(pinchConfig.normal))).toBeLessThan(1e-9);

    build.update(world, 0.016);
    const contacts = world.getResource('cablePinchContacts');
    expect(contacts).toHaveLength(1);
    expect(contacts[0].minDistance).toBeCloseTo(0.1, 9);
  });

  test('does not detect transitional pinch if bodies are farther apart than 2w', () => {
    const { world, entityB } = setupTransitionalPinchWorld();
    world.getComponent(entityB, PositionComponent).pos.set(new Vector2(0.41, 0.0));

    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    detect.update(world, 0.016);

    const candidates = world.getResource('cablePinchCandidates');
    expect(Array.isArray(candidates)).toBe(true);
    expect(candidates).toHaveLength(0);

    configure.update(world, 0.016);
    const pinchConfigs = world.getResource('cablePinchJointConfigs');
    expect(pinchConfigs instanceof Map).toBe(true);
    expect(pinchConfigs.size).toBe(0);
  });

  test('deduplicates contacts for multiple transitional joints between same body pair', () => {
    const { world, entityA, entityB, jointId } = setupTransitionalPinchWorld();
    const secondJointId = world.createEntity();
    world.addComponent(
      secondJointId,
      new CableJointComponent(
        entityA,
        entityB,
        0.0,
        new Vector2(0.15, 0.02),
        new Vector2(0.19, 0.02)
      )
    );
    const secondPathId = world.createEntity();
    world.addComponent(
      secondPathId,
      new CablePathComponent(
        world,
        [secondJointId],
        ['rolling', 'rolling'],
        [true, true],
        Infinity,
        null,
        0.05
      )
    );

    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    const build = new PinchConstraintBuildSystem();

    detect.update(world, 0.016);
    const candidates = world.getResource('cablePinchCandidates');
    expect(candidates).toHaveLength(2);

    configure.update(world, 0.016);
    const pinchConfigs = world.getResource('cablePinchJointConfigs');
    expect(pinchConfigs.size).toBe(2);
    expect(pinchConfigs.has(jointId)).toBe(true);
    expect(pinchConfigs.has(secondJointId)).toBe(true);

    build.update(world, 0.016);
    const contacts = world.getResource('cablePinchContacts');
    expect(contacts).toHaveLength(1);
  });

  test('contact inequality solver enforces d >= 2w', () => {
    const { world, entityA, entityB } = setupTransitionalPinchWorld();
    const detect = new PinchDetectionSystem();
    const configure = new PinchConfigureSystem();
    const build = new PinchConstraintBuildSystem();
    const solver = new PBDCableConstraintSolver();

    detect.update(world, 0.016);
    configure.update(world, 0.016);
    build.update(world, 0.016);
    solver.update(world, 0.016);

    const posA = world.getComponent(entityA, PositionComponent).pos;
    const posB = world.getComponent(entityB, PositionComponent).pos;
    const rA = world.getComponent(entityA, RadiusComponent).radius;
    const rB = world.getComponent(entityB, RadiusComponent).radius;
    const surfaceDistance = posA.distanceTo(posB) - (rA + rB);

    expect(surfaceDistance).toBeGreaterThanOrEqual(0.1 - 1e-6);
  });
});
