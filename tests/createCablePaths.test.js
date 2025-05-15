import Vector2 from '../cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  RadiusComponent
} from '../cable_joints/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
  createCablePaths
} from '../cable_joints/cable_joints_core.js';
import { signedArcLengthOnWheel } from '../cable_joints/geometry.js';

describe('createCablePaths', () => {
  let world;
  const springConstant = 1000;

  beforeEach(() => {
    world = new World();
    // Mock console.warn to prevent test output pollution and allow checking calls
    jest.spyOn(console, 'warn').mockImplementation(() => {});
  });

  afterEach(() => {
    // Restore console.warn
    console.warn.mockRestore();
  });

  const createMockJointEntity = (entityA, entityB, restLength = 1.0, attachA = new Vector2(0,0), attachB = new Vector2(0,0)) => {
    const jointId = world.createEntity();
    world.addComponent(jointId, new CableJointComponent(entityA, entityB, restLength, attachA, attachB));
    return jointId;
  };

  // Helper to create a generic entity, optionally with Position and Radius for rolling links
  const createMockLinkEntity = (id, isRolling = false, pos = new Vector2(0,0), radius = 1.0) => {
    // In a real scenario, entities would be created by world.createEntity()
    // For testing, we just need IDs and to add components if they are 'rolling'
    // and CablePathComponent constructor might need them.
    world.addComponent(id, new PositionComponent(pos.x, pos.y));
    if (isRolling) {
      world.addComponent(id, new RadiusComponent(radius));
    }
    return id;
  };


  test('should create a single path if no intermediate attachment links', () => {
    const e0 = createMockLinkEntity(world.createEntity(), true, new Vector2(0,0), 1);
    const e1 = createMockLinkEntity(world.createEntity(), true, new Vector2(2,0), 1);
    const e2 = createMockLinkEntity(world.createEntity(), true, new Vector2(4,0), 1);

    const j1 = createMockJointEntity(e0, e1, 1, new Vector2(0,1), new Vector2(2,1)); // A on e0, B on e1
    const j2 = createMockJointEntity(e1, e2, 1, new Vector2(2,-1), new Vector2(4,-1)); // A on e1, B on e2

    const jointEntities = [j1, j2];
    const linkTypes = ['rolling', 'rolling', 'rolling'];
    const cw = [true, true, true];

    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant);

    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.jointEntities).toEqual(jointEntities);
    expect(pathComp.linkTypes).toEqual(linkTypes);
    expect(pathComp.cw).toEqual(cw);
    expect(pathComp.spring_constant).toBe(springConstant);

    // Check stored lengths (simplified check, more detailed in CablePathComponent tests)
    // stored[0] and stored[2] are terminal 'rolling', CablePathComponent init to 0 if not userStored
    // stored[1] is intermediate 'rolling'
    const expectedStored1 = signedArcLengthOnWheel(
        world.getComponent(j1, CableJointComponent).attachmentPointB_world, // on e1
        world.getComponent(j2, CableJointComponent).attachmentPointA_world, // on e1
        world.getComponent(e1, PositionComponent).pos,
        world.getComponent(e1, RadiusComponent).radius,
        cw[1], true
    );
    expect(pathComp.stored[0]).toBeCloseTo(0.0); // Terminal rolling, not calculated by default loop
    expect(pathComp.stored[1]).toBeCloseTo(expectedStored1);
    expect(pathComp.stored[2]).toBeCloseTo(0.0); // Terminal rolling
  });

  test('should split into two paths for one intermediate attachment link', () => {
    const e0 = createMockLinkEntity(world.createEntity());
    const e1 = createMockLinkEntity(world.createEntity(), true, new Vector2(0,0),1); // Rolling
    const e2 = createMockLinkEntity(world.createEntity()); // Attachment
    const e3 = createMockLinkEntity(world.createEntity(), true, new Vector2(2,0),1); // Rolling
    const e4 = createMockLinkEntity(world.createEntity());


    const j1 = createMockJointEntity(e0, e1, 1.0, new Vector2(0,0), new Vector2(0,1));
    const j2 = createMockJointEntity(e1, e2, 1.0, new Vector2(0,-1), new Vector2(1,0));
    const j3 = createMockJointEntity(e2, e3, 1.0, new Vector2(1,0), new Vector2(2,1));
    const j4 = createMockJointEntity(e3, e4, 1.0, new Vector2(2,-1), new Vector2(3,0));

    const jointEntities = [j1, j2, j3, j4];
    const linkTypes = ['hybrid-attachment', 'rolling', 'attachment', 'rolling', 'hybrid-attachment'];
    const cw = [true, true, false, true, false];
    const userStored = [0.1, null, 0.3, null, 0.5]; // null means calculate

    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant, userStored);

    expect(pathEntityIds).toHaveLength(2);

    // Path 1
    const pathComp1 = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp1.jointEntities).toEqual([j1, j2]);
    expect(pathComp1.linkTypes).toEqual(['hybrid-attachment', 'rolling', 'attachment']);
    expect(pathComp1.cw).toEqual([true, true, false]);
    expect(pathComp1.stored[0]).toBeCloseTo(userStored[0]); // from userStored
    const expectedStored1_1 = signedArcLengthOnWheel(
        world.getComponent(j1, CableJointComponent).attachmentPointB_world,
        world.getComponent(j2, CableJointComponent).attachmentPointA_world,
        world.getComponent(e1, PositionComponent).pos,
        world.getComponent(e1, RadiusComponent).radius,
        cw[1], true
    );
    expect(pathComp1.stored[1]).toBeCloseTo(expectedStored1_1); // calculated as userStored[1] is null
    expect(pathComp1.stored[2]).toBeCloseTo(userStored[2]); // from userStored

    // Path 2
    const pathComp2 = world.getComponent(pathEntityIds[1], CablePathComponent);
    expect(pathComp2.jointEntities).toEqual([j3, j4]);
    expect(pathComp2.linkTypes).toEqual(['attachment', 'rolling', 'hybrid-attachment']);
    expect(pathComp2.cw).toEqual([false, true, false]); // cw[2], cw[3], cw[4] from original
    expect(pathComp2.stored[0]).toBeCloseTo(userStored[2]); // from userStored (this is linkTypes[2] from original)
    const expectedStored2_1 = signedArcLengthOnWheel(
        world.getComponent(j3, CableJointComponent).attachmentPointB_world,
        world.getComponent(j4, CableJointComponent).attachmentPointA_world,
        world.getComponent(e3, PositionComponent).pos,
        world.getComponent(e3, RadiusComponent).radius,
        cw[3], true
    );
    expect(pathComp2.stored[1]).toBeCloseTo(expectedStored2_1); // calculated as userStored[3] is null
    expect(pathComp2.stored[2]).toBeCloseTo(userStored[4]); // from userStored
  });

  test('should split into three paths for two intermediate attachment links', () => {
    const e = Array(7).fill(null).map((_,i) => createMockLinkEntity(world.createEntity(), false, new Vector2(i,0)));
    const j = Array(6).fill(null).map((_,i) => createMockJointEntity(e[i], e[i+1]));

    const jointEntities = [j[0], j[1], j[2], j[3], j[4], j[5]];
    const linkTypes = ['rolling', 'attachment', 'rolling', 'attachment', 'rolling', 'attachment', 'rolling'];
    const cw = [true, false, true, false, true, false, true];

    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant);
    expect(pathEntityIds).toHaveLength(3);

    const pathComp1 = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp1.jointEntities).toEqual([j[0], j[1]]);
    expect(pathComp1.linkTypes).toEqual(['rolling', 'attachment', 'rolling']);
    expect(pathComp1.cw).toEqual([true, false, true]);


    const pathComp2 = world.getComponent(pathEntityIds[1], CablePathComponent);
    expect(pathComp2.jointEntities).toEqual([j[2], j[3]]);
    expect(pathComp2.linkTypes).toEqual(['rolling', 'attachment', 'rolling']);
    expect(pathComp2.cw).toEqual([true, false, true]); // Original cw indices 2,3,4

    const pathComp3 = world.getComponent(pathEntityIds[2], CablePathComponent);
    expect(pathComp3.jointEntities).toEqual([j[4], j[5]]);
    expect(pathComp3.linkTypes).toEqual(['rolling', 'attachment', 'rolling']);
    expect(pathComp3.cw).toEqual([true, false, true]); // Original cw indices 4,5,6
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
    const e0 = createMockLinkEntity(world.createEntity());
    const e1 = createMockLinkEntity(world.createEntity());
    const j1 = createMockJointEntity(e0, e1);
    const pathEntityIds = createCablePaths(world, [j1], ['rolling', 'attachment'], [true, true], springConstant);
    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.linkTypes).toEqual(['rolling', 'attachment']);
  });

  test('should not split if attachment links are only at start and end', () => {
    const e0 = createMockLinkEntity(world.createEntity());
    const e1 = createMockLinkEntity(world.createEntity());
    const e2 = createMockLinkEntity(world.createEntity());
    const j1 = createMockJointEntity(e0, e1);
    const j2 = createMockJointEntity(e1, e2);
    const pathEntityIds = createCablePaths(world, [j1, j2], ['attachment', 'rolling', 'attachment'], [true, true, true], springConstant);
    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
  });

   test('should split correctly with attachments at start, middle, and end', () => {
    const e = Array(4).fill(null).map((_,i) => createMockLinkEntity(world.createEntity(), false, new Vector2(i,0)));
    const j = Array(3).fill(null).map((_,i) => createMockJointEntity(e[i], e[i+1]));

    const jointEntities = [j[0], j[1], j[2]];
    // Path: e0 --j0-- e1 --j1-- e2 --j2-- e3
    // Links: L(e0) L(e1) L(e2) L(e3)
    // LT:    attach, roll, attach, roll
    const linkTypes = ['attachment', 'rolling', 'attachment', 'rolling'];
    const cw = [true, true, false, true];
    const pathEntityIds = createCablePaths(world, jointEntities, linkTypes, cw, springConstant);

    expect(pathEntityIds).toHaveLength(2);

    const pathComp1 = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp1.jointEntities).toEqual([j[0], j[1]]);
    expect(pathComp1.linkTypes).toEqual(['attachment', 'rolling', 'attachment']);
    expect(pathComp1.cw).toEqual([true, true, false]);

    const pathComp2 = world.getComponent(pathEntityIds[1], CablePathComponent);
    expect(pathComp2.jointEntities).toEqual([j[2]]);
    expect(pathComp2.linkTypes).toEqual(['attachment', 'rolling']);
    expect(pathComp2.cw).toEqual([false, true]);
  });

  test('should create one path for a single link (no joints)', () => {
    const linkTypes = ['rolling'];
    const cw = [true];
    const userStored = [0.5];
    const pathEntityIds = createCablePaths(world, [], linkTypes, cw, springConstant, userStored);

    expect(pathEntityIds).toHaveLength(1);
    const pathComp = world.getComponent(pathEntityIds[0], CablePathComponent);
    expect(pathComp.jointEntities).toEqual([]);
    expect(pathComp.linkTypes).toEqual(linkTypes);
    expect(pathComp.cw).toEqual(cw);
    expect(pathComp.stored).toEqual(userStored);
    expect(pathComp.totalRestLength).toBeCloseTo(0.5); // From userStored
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
    expect(pathComp.stored).toEqual([0.0]); // Default for attachment
    expect(pathComp.totalRestLength).toBeCloseTo(0.0);
  });


  test('should return empty array and warn for mismatched linkTypes and jointEntities length', () => {
    const pathEntityIds = createCablePaths(world, [createMockJointEntity(0,1)], ['type1'], [true], springConstant); // linkTypes too short
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: linkTypes.length must be jointEntities.length + 1. Aborting.");
  });

  test('should return empty array and warn for mismatched cw and linkTypes length', () => {
    const pathEntityIds = createCablePaths(world, [], ['type1'], [], springConstant); // cw too short
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: cw.length must be linkTypes.length. Aborting.");
  });

  test('should return empty array and warn for mismatched userStored and linkTypes length', () => {
    const pathEntityIds = createCablePaths(world, [], ['type1'], [true], springConstant, []); // userStored too short
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: userStored.length must be linkTypes.length if provided. Aborting.");
  });

  test('should return empty array for completely empty inputs (violates linkTypes vs joints rule)', () => {
    // linkTypes.length (0) !== jointEntities.length (0) + 1
    const pathEntityIds = createCablePaths(world, [], [], [], springConstant);
    expect(pathEntityIds).toEqual([]);
    expect(console.warn).toHaveBeenCalledWith("createCablePaths: linkTypes.length must be jointEntities.length + 1. Aborting.");
  });
});
