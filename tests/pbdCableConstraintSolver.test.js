import Vector2 from '../cable_joints/vector2.js';
import { World, PositionComponent, VelocityComponent, MassComponent, GravityAffectedComponent } from '../cable_joints/ecs.js';
import {
  CableJointComponent,
  CableLinkComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../cable_joints/cable_joints_core.js';
import { GravitySystem } from '../cable_joints/commonSystems.js';

describe('PBDCableConstraintSolver', () => {
  test('does nothing when compliance is zero', () => {
    const world = new World();
    // Create entities and positions
    const e0 = world.createEntity();
    const e1 = world.createEntity();
    const e2 = world.createEntity();
    world.addComponent(e0, new PositionComponent(0, 0));
    world.addComponent(e1, new PositionComponent(1, 0));
    world.addComponent(e2, new PositionComponent(2, 0));

    // Create two joints with rest length 1
    const j1 = world.createEntity();
    const aA1 = new Vector2(0, 0);
    const aB1 = new Vector2(1, 0);
    world.addComponent(
      j1,
      new CableJointComponent(e0, e1, 1.0, aA1.clone(), aB1.clone())
    );
    const j2 = world.createEntity();
    const aA2 = new Vector2(1, 0);
    const aB2 = new Vector2(2, 0);
    world.addComponent(
      j2,
      new CableJointComponent(e1, e2, 1.0, aA2.clone(), aB2.clone())
    );

    // Build a path with infinite spring_constant => zero compliance
    const pathEnt = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [j1, j2],
      ['attachment', 'attachment', 'attachment'],
      [true, true, true],
      Infinity
    );
    world.addComponent(pathEnt, pathComp);

    // Save initial attachment points
    const initA1 = aA1.clone();
    const initB1 = aB1.clone();
    const initA2 = aA2.clone();
    const initB2 = aB2.clone();

    // Run solver
    const solver = new PBDCableConstraintSolver();
    expect(() => solver.update(world, 0.016)).not.toThrow();

    // Verify points unchanged
    const comp1 = world.getComponent(j1, CableJointComponent);
    expect(comp1.attachmentPointA_world).toEqual(initA1);
    expect(comp1.attachmentPointB_world).toEqual(initB1);
    const comp2 = world.getComponent(j2, CableJointComponent);
    expect(comp2.attachmentPointA_world).toEqual(initA2);
    expect(comp2.attachmentPointB_world).toEqual(initB2);
  });

  test('handles empty world without error', () => {
    const world = new World();
    const solver = new PBDCableConstraintSolver();
    expect(() => solver.update(world, 0.1)).not.toThrow();
  });

  test('clamps each segment to rest length when stretched', () => {
    const world = new World();
    const e0 = world.createEntity();
    const e1 = world.createEntity();
    world.addComponent(e0, new PositionComponent(0, 0));
    world.addComponent(e1, new PositionComponent(5, 0));
    world.addComponent(e0, new VelocityComponent(0, 0));
    world.addComponent(e1, new VelocityComponent(0, 0));
    world.addComponent(e0, new MassComponent(1.0));
    world.addComponent(e1, new MassComponent(1.0));
    world.addComponent(e0, new CableLinkComponent);
    world.addComponent(e1, new CableLinkComponent);

    const j1 = world.createEntity();
    world.addComponent(
      j1,
      new CableJointComponent(
        e0,
        e1,
        3.0,
        new Vector2(0, 0),
        new Vector2(5, 0)
      )
    );

    const pathEnt = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [j1],
      ['attachment', 'attachment'],
      [true, true]
    );
    world.addComponent(pathEnt, pathComp);

    const dt = 0.016;
    const solver = new PBDCableConstraintSolver();
    const cableAttachmentSystem = new CableAttachmentUpdateSystem();
    cableAttachmentSystem.update(world, dt);
    solver.update(world, dt);
    cableAttachmentSystem.update(world, dt);
    solver.update(world, dt);

    const comp1 = world.getComponent(j1, CableJointComponent);
    const d1 = comp1.attachmentPointA_world.distanceTo(comp1.attachmentPointB_world);
    expect(d1).toBeCloseTo(3.0, 1);
  });

  test('pendulum constraint keeps mass within restLength under gravity', () => {
    const startLength = 2.0;
    const restLength = 1.0;
    const world = new World();
    // Fixed point at origin
    const origin = world.createEntity();
    world.addComponent(origin, new PositionComponent(0, 0));
    world.addComponent(origin, new CableLinkComponent);
    // Mass entity starting stretched beyond rest length
    const mass = world.createEntity();
    world.addComponent(mass, new PositionComponent(0, -startLength));
    world.addComponent(mass, new VelocityComponent(0, 0));
    world.addComponent(mass, new MassComponent(1.0));
    world.addComponent(mass, new GravityAffectedComponent);
    world.addComponent(mass, new CableLinkComponent);
    // Cable joint with rest length 1 between origin and mass
    const j = world.createEntity();
    world.addComponent(
      j,
      new CableJointComponent(
        origin,
        mass,
        restLength,
        new Vector2(0, 0),
        new Vector2(0, -startLength)
      )
    );
    // Build path
    const pathEnt = world.createEntity();
    world.addComponent(
      pathEnt,
      new CablePathComponent(
        world,
        [j],
        ['attachment', 'attachment'],
        [true, true]
      )
    );
    const gravitySystem = new GravitySystem();
    const cableAttachmentSystem = new CableAttachmentUpdateSystem();
    const solver = new PBDCableConstraintSolver();
    const dt =  0.016;
    // Run solver multiple times to enforce constraint
    for (let i = 0; i < 5; i++) {
      gravitySystem.update(world, dt);
      cableAttachmentSystem.update(world, dt);
      solver.update(world, dt);
    }
    // Check that the mass is no further than rest length from origin
    const posOrigin = world.getComponent(origin, PositionComponent).pos;
    const posMass = world.getComponent(mass, PositionComponent).pos;
    const distance = posOrigin.distanceTo(posMass);
    expect(distance).toBeCloseTo(restLength, 5);
  });
});
