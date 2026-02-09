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
  test('detects and configures a transitional pinch with zero-length segment', () => {
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
    expect(joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world)).toBeCloseTo(0.0, 9);

    build.update(world, 0.016);
    const contacts = world.getResource('cablePinchContacts');
    expect(contacts).toHaveLength(1);
    expect(contacts[0].minDistance).toBeCloseTo(0.1, 9);
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

