import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
  RadiusComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { signedArcLengthOnWheel } from '../../../src/js/cable_joints_3d/geometry3.js';
import {
  createCablePaths
} from '../../../src/js/cable_joints_3d/createCablePaths.js';

describe('createCablePaths (3D planar)', () => {
  let world;
  const springConstant = 1000;

  beforeEach(() => {
    world = new World();
    jest.spyOn(console, 'warn').mockImplementation(() => {});
  });

  afterEach(() => {
    console.warn.mockRestore();
  });

  const createMockJointEntity = (entityA, entityB, restLength = 1.0, attachA = new Vector3(0, 0, 0), attachB = new Vector3(0, 0, 0)) => {
    const jointId = world.createEntity();
    world.addComponent(jointId, new CableJointComponent(entityA, entityB, restLength, attachA, attachB));
    return jointId;
  };

  const createMockLinkEntity = (id, isRolling = false, pos = new Vector3(0, 0, 0), radius = 1.0) => {
    world.addComponent(id, new PositionComponent(pos.x, pos.y, pos.z));
    if (isRolling) {
      world.addComponent(id, new RadiusComponent(radius));
    }
    return id;
  };

  test('should create a single path if no intermediate attachment links', () => {
    const e0 = createMockLinkEntity(world.createEntity(), true, new Vector3(0, 0, 0), 1);
    const e1 = createMockLinkEntity(world.createEntity(), true, new Vector3(2, 0, 0), 1);
    const e2 = createMockLinkEntity(world.createEntity(), true, new Vector3(4, 0, 0), 1);

    const j1 = createMockJointEntity(e0, e1, 1, new Vector3(0, 1, 0), new Vector3(2, 1, 0));
    const j2 = createMockJointEntity(e1, e2, 1, new Vector3(2, -1, 0), new Vector3(4, -1, 0));

    const jointEntities = [j1, j2];
    const linkTypes = ['hybrid-attachment', 'rolling', 'hybrid-attachment'];
    const cw = [true, true, true];

    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant);

    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.jointEntities).toEqual(jointEntities);
    expect(pathComp.linkTypes).toEqual(['hybrid-attachment', 'rolling', 'hybrid-attachment']);
    expect(pathComp.cw).toEqual(cw);
    expect(pathComp.spring_constant).toBe(springConstant);

    const expectedStored1 = signedArcLengthOnWheel(
      world.getComponent(j1, CableJointComponent).attachmentPointB_world,
      world.getComponent(j2, CableJointComponent).attachmentPointA_world,
      world.getComponent(e1, PositionComponent).pos,
      world.getComponent(e1, RadiusComponent).radius,
      cw[1],
      new Vector3(0, 0, 1),
      true
    );
    expect(pathComp.stored[0]).toBeCloseTo(0.0);
    expect(pathComp.stored[1]).toBeCloseTo(expectedStored1);
    expect(pathComp.stored[2]).toBeCloseTo(0.0);
  });

  test('should split into two paths for one intermediate attachment link', () => {
    const e0 = createMockLinkEntity(world.createEntity());
    const e1 = createMockLinkEntity(world.createEntity(), true, new Vector3(0, 0, 0), 1);
    const e2 = createMockLinkEntity(world.createEntity());
    const e3 = createMockLinkEntity(world.createEntity(), true, new Vector3(2, 0, 0), 1);
    const e4 = createMockLinkEntity(world.createEntity());

    const j1 = createMockJointEntity(e0, e1, 1.0, new Vector3(0, 0, 0), new Vector3(0, 1, 0));
    const j2 = createMockJointEntity(e1, e2, 1.0, new Vector3(0, -1, 0), new Vector3(1, 0, 0));
    const j3 = createMockJointEntity(e2, e3, 1.0, new Vector3(1, 0, 0), new Vector3(2, 1, 0));
    const j4 = createMockJointEntity(e3, e4, 1.0, new Vector3(2, -1, 0), new Vector3(3, 0, 0));

    const jointEntities = [j1, j2, j3, j4];
    const linkTypes = ['hybrid-attachment', 'rolling', 'attachment', 'rolling', 'hybrid-attachment'];
    const cw = [true, true, false, true, false];
    const userStored = [0.1, null, 0.3, null, 0.5];

    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant, userStored);

    expect(pathEntityIds).toHaveLength(2);

    const pathComp1 = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp1.jointEntities).toEqual([j1, j2]);
    expect(pathComp1.linkTypes).toEqual(['hybrid-attachment', 'rolling', 'attachment']);
    expect(pathComp1.cw).toEqual([true, true, false]);
    expect(pathComp1.stored[0]).toBeCloseTo(userStored[0]);
    const expectedStored1_1 = signedArcLengthOnWheel(
      world.getComponent(j1, CableJointComponent).attachmentPointB_world,
      world.getComponent(j2, CableJointComponent).attachmentPointA_world,
      world.getComponent(e1, PositionComponent).pos,
      world.getComponent(e1, RadiusComponent).radius,
      cw[1],
      new Vector3(0, 0, 1),
      true
    );
    expect(pathComp1.stored[1]).toBeCloseTo(expectedStored1_1);
    expect(pathComp1.stored[2]).toBeCloseTo(userStored[2]);

    const pathComp2 = world.getComponent(pathEntityIds[1], CablePathComponent);
    expect(pathComp2.jointEntities).toEqual([j3, j4]);
    expect(pathComp2.linkTypes).toEqual(['attachment', 'rolling', 'hybrid-attachment']);
    expect(pathComp2.cw).toEqual([false, true, false]);
    expect(pathComp2.stored[0]).toBeCloseTo(userStored[2]);
    const expectedStored2_1 = signedArcLengthOnWheel(
      world.getComponent(j3, CableJointComponent).attachmentPointB_world,
      world.getComponent(j4, CableJointComponent).attachmentPointA_world,
      world.getComponent(e3, PositionComponent).pos,
      world.getComponent(e3, RadiusComponent).radius,
      cw[3],
      new Vector3(0, 0, 1),
      true
    );
    expect(pathComp2.stored[1]).toBeCloseTo(expectedStored2_1);
    expect(pathComp2.stored[2]).toBeCloseTo(userStored[4]);
  });

  test('should split into three paths for two intermediate attachment links', () => {
    const e = Array(7).fill(null).map((_, i) => createMockLinkEntity(world.createEntity(), [0, 2, 4, 6].includes(i), new Vector3(i, 0, 0)));
    const j = Array(6).fill(null).map((_, i) => createMockJointEntity(e[i], e[i + 1]));

    const jointEntities = [j[0], j[1], j[2], j[3], j[4], j[5]];
    const linkTypes = ['hybrid-attachment', 'attachment', 'rolling', 'attachment', 'rolling', 'attachment', 'hybrid-attachment'];
    const cw = [true, false, true, false, true, false, true];

    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant);
    expect(pathEntityIds).toHaveLength(4);

    const pathComp1 = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp1.jointEntities).toEqual([j[0]]);
    expect(pathComp1.linkTypes).toEqual(['hybrid-attachment', 'attachment']);
    expect(pathComp1.cw).toEqual([true, false]);

    const pathComp2 = world.getComponent(pathEntityIds[1], CablePathComponent);
    expect(pathComp2.jointEntities).toEqual([j[1], j[2]]);
    expect(pathComp2.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
    expect(pathComp2.cw).toEqual([false, true, false]);

    const pathComp3 = world.getComponent(pathEntityIds[2], CablePathComponent);
    expect(pathComp3.jointEntities).toEqual([j[3], j[4]]);
    expect(pathComp3.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
    expect(pathComp3.cw).toEqual([false, true, false]);

    const pathComp4 = world.getComponent(pathEntityIds[3], CablePathComponent);
    expect(pathComp4.jointEntities).toEqual([j[5]]);
    expect(pathComp4.linkTypes).toEqual(['attachment', 'hybrid-attachment']);
    expect(pathComp4.cw).toEqual([false, true]);
  });

  test('should not split if attachment link is at the start', () => {
    const e0 = createMockLinkEntity(world.createEntity());
    const e1 = createMockLinkEntity(world.createEntity());
    const j1 = createMockJointEntity(e0, e1);
    const pathEntityIds = createCablePaths(world, [j1], ['attachment', 'rolling'], [true, true], springConstant);
    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.linkTypes).toEqual(['attachment', 'rolling']);
  });

  test('should not split if attachment link is at the end', () => {
    const e0 = createMockLinkEntity(world.createEntity(), true);
    const e1 = createMockLinkEntity(world.createEntity());
    const j1 = createMockJointEntity(e0, e1);
    const pathEntityIds = createCablePaths(world, [j1], ['hybrid-attachment', 'attachment'], [true, true], springConstant);
    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.linkTypes).toEqual(['hybrid-attachment', 'attachment']);
  });

  test('should not split if attachment links are only at start and end', () => {
    const e0 = createMockLinkEntity(world.createEntity());
    const e1 = createMockLinkEntity(world.createEntity(), true);
    const e2 = createMockLinkEntity(world.createEntity());
    const j1 = createMockJointEntity(e0, e1);
    const j2 = createMockJointEntity(e1, e2);
    const pathEntityIds = createCablePaths(world, [j1, j2], ['attachment', 'rolling', 'attachment'], [true, true, true], springConstant);
    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
  });

  test('should split correctly with attachments at start, middle, and end', () => {
    const e = Array(4).fill(null).map((_, i) => createMockLinkEntity(world.createEntity(), (i === 1 || i === 3), new Vector3(i, 0, 0)));
    const j = Array(3).fill(null).map((_, i) => createMockJointEntity(e[i], e[i + 1]));

    const jointEntities = [j[0], j[1], j[2]];
    const linkTypes = ['attachment', 'rolling', 'attachment', 'hybrid-attachment'];
    const cw = [true, true, false, true];
    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant);

    expect(pathEntityIds).toHaveLength(2);

    const pathComp1 = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp1.jointEntities).toEqual([j[0], j[1]]);
    expect(pathComp1.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
    expect(pathComp1.cw).toEqual([true, true, false]);

    const pathComp2 = world.getComponent(pathEntityIds[1], CablePathComponent);
    expect(pathComp2.jointEntities).toEqual([j[2]]);
    expect(pathComp2.linkTypes).toEqual(['attachment', 'hybrid-attachment']);
    expect(pathComp2.cw).toEqual([false, true]);
  });

  test('should create one path for a single link (no joints)', () => {
    const linkTypes = ['hybrid-attachment'];
    const cw = [true];
    const userStored = [0.5];
    const pathEntityIds = createCablePaths(world, [], linkTypes, cw, springConstant, userStored);

    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.jointEntities).toEqual([]);
    expect(pathComp.linkTypes).toEqual(['hybrid-attachment']);
    expect(pathComp.cw).toEqual(cw);
    expect(pathComp.stored).toEqual(userStored);
    expect(pathComp.totalRestLength).toBeCloseTo(0.5);
  });

  test('preserves cable half width on created path components', () => {
    const e0 = createMockLinkEntity(world.createEntity(), true, new Vector3(0, 0, 0), 1);
    const e1 = createMockLinkEntity(world.createEntity(), true, new Vector3(2, 0, 0), 1);
    const j0 = createMockJointEntity(e0, e1, 1.0, new Vector3(0, 1, 0), new Vector3(2, 1, 0));

    const pathEntityIds = createCablePaths(
      world,
      [j0],
      ['hybrid-attachment', 'hybrid-attachment'],
      [true, true],
      springConstant,
      [0.0, 0.0],
      0.125
    );

    expect(pathEntityIds).toHaveLength(1);
    expect(world.getComponent(pathEntityIds[0], CablePathComponent).cableHalfWidth).toBeCloseTo(0.125);
  });

  test('should create one path for a single attachment link (no joints)', () => {
    const linkTypes = ['attachment'];
    const cw = [true];
    const pathEntityIds = createCablePaths(world, [], linkTypes, cw, springConstant);
    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.jointEntities).toEqual([]);
    expect(pathComp.linkTypes).toEqual(linkTypes);
    expect(pathComp.cw).toEqual(cw);
    expect(pathComp.stored).toEqual([0.0]);
    expect(pathComp.totalRestLength).toBeCloseTo(0.0);
  });

  test('should return empty array and warn for mismatched linkTypes and jointEntities length', () => {
    const pathEntityIds = createCablePaths(world, [createMockJointEntity(0, 1)], ['type1'], [true], springConstant);
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: linkTypes.length must be jointEntities.length + 1. Aborting.");
  });

  test('should return empty array and warn for mismatched cw and linkTypes length', () => {
    const pathEntityIds = createCablePaths(world, [], ['type1'], [], springConstant);
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: cw.length must be linkTypes.length. Aborting.");
  });

  test('should return empty array and warn for mismatched userStored and linkTypes length', () => {
    const pathEntityIds = createCablePaths(world, [], ['type1'], [true], springConstant, []);
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: userStored.length must be linkTypes.length if provided. Aborting.");
  });

  test('should return empty array for completely empty inputs (violates linkTypes vs joints rule)', () => {
    const pathEntityIds = createCablePaths(world, [], [], [], springConstant);
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: linkTypes.length must be jointEntities.length + 1. Aborting.");
  });
});
