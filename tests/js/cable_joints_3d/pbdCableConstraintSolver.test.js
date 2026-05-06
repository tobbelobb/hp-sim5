import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  MassComponent,
  GravityAffectedComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CableLinkComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { GravitySystem } from '../../../src/js/cable_joints_3d/commonSystems.js';

describe('PBDCableConstraintSolver (3D)', () => {
  test('does nothing when compliance is zero', () => {
    const world = new World();
    // Create entities and positions
    const e0 = world.createEntity();
    const e1 = world.createEntity();
    const e2 = world.createEntity();
    world.addComponent(e0, new PositionComponent(0, 0, 0));
    world.addComponent(e1, new PositionComponent(1, 0, 0));
    world.addComponent(e2, new PositionComponent(2, 0, 0));

    // Create two joints with rest length 1
    const j1 = world.createEntity();
    const aA1 = new Vector3(0, 0, 0);
    const aB1 = new Vector3(1, 0, 0);
    world.addComponent(
      j1,
      new CableJointComponent(e0, e1, 1.0, aA1.clone(), aB1.clone())
    );
    const j2 = world.createEntity();
    const aA2 = new Vector3(1, 0, 0);
    const aB2 = new Vector3(2, 0, 0);
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
    world.addComponent(e0, new PositionComponent(0, 0, 0));
    world.addComponent(e1, new PositionComponent(5, 0, 0));
    world.addComponent(e0, new VelocityComponent(0, 0, 0));
    world.addComponent(e1, new VelocityComponent(0, 0, 0));
    world.addComponent(e0, new MassComponent(1.0));
    world.addComponent(e1, new MassComponent(1.0));
    world.addComponent(e0, new CableLinkComponent());
    world.addComponent(e1, new CableLinkComponent());

    const j1 = world.createEntity();
    world.addComponent(
      j1,
      new CableJointComponent(
        e0,
        e1,
        3.0,
        new Vector3(0, 0, 0),
        new Vector3(5, 0, 0)
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
    world.setResource('dt', dt);
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

  test('records XPBD constraint force as lambda times direction over dt squared', () => {
    const world = new World();
    const e0 = world.createEntity();
    const e1 = world.createEntity();
    world.addComponent(e0, new PositionComponent(0, 0, 0));
    world.addComponent(e1, new PositionComponent(5, 0, 0));
    world.addComponent(e0, new MassComponent(1.0));
    world.addComponent(e1, new MassComponent(1.0));

    const jointId = world.createEntity();
    world.addComponent(
      jointId,
      new CableJointComponent(
        e0,
        e1,
        3.0,
        new Vector3(0, 0, 0),
        new Vector3(5, 0, 0)
      )
    );

    const pathEnt = world.createEntity();
    world.addComponent(
      pathEnt,
      new CablePathComponent(
        world,
        [jointId],
        ['attachment', 'attachment'],
        [true, true],
        Infinity
      )
    );

    const dt = 0.5;
    world.setResource('dt', dt);
    new PBDCableConstraintSolver().update(world, dt);

    const joint = world.getComponent(jointId, CableJointComponent);
    expect(joint.constraintLambda).toBeCloseTo(-1.0, 8);
    expect(joint.constraintForce.x).toBeCloseTo(-4.0, 8);
    expect(joint.constraintForce.y).toBeCloseTo(0.0, 8);
    expect(joint.constraintForce.z).toBeCloseTo(0.0, 8);
    expect(joint.constraintForceMagnitude).toBeCloseTo(4.0, 8);
  });

  test('damping increases force only when a taut cable is stretching', () => {
    const solveForce = (prevZ) => {
      const world = new World();
      const mass = world.createEntity();
      const anchor = world.createEntity();

      world.addComponent(mass, new PositionComponent(0, 0, 0));
      world.addComponent(mass, new PrevFinalPosComponent(0, 0, prevZ));
      world.addComponent(mass, new MassComponent(1));
      world.addComponent(anchor, new PositionComponent(0, 0, 1));
      world.addComponent(anchor, new PrevFinalPosComponent(0, 0, 1));
      world.addComponent(anchor, new MassComponent(-1));

      const jointId = world.createEntity();
      world.addComponent(
        jointId,
        new CableJointComponent(
          mass,
          anchor,
          0.9,
          new Vector3(0, 0, 0),
          new Vector3(0, 0, 1),
        ),
      );

      const pathEnt = world.createEntity();
      world.addComponent(
        pathEnt,
        new CablePathComponent(
          world,
          [jointId],
          ['attachment', 'attachment'],
          [true, true],
          20000,
          null,
          0.0,
          5.0,
        ),
      );

      const dt = 1 / 500;
      world.setResource('dt', dt);
      new PBDCableConstraintSolver().update(world, dt);
      return world.getComponent(jointId, CableJointComponent).constraintForceMagnitude;
    };

    const staticForce = solveForce(0.0);
    const lengtheningForce = solveForce(0.01);
    const shorteningForce = solveForce(-0.01);

    expect(lengtheningForce).toBeGreaterThan(staticForce);
    expect(shorteningForce).toBeLessThan(staticForce);
  });

  test('pendulum constraint keeps mass within restLength under gravity', () => {
    const startLength = 2.0;
    const restLength = 1.0;
    const world = new World();
    // Fixed point at origin
    const origin = world.createEntity();
    world.addComponent(origin, new PositionComponent(0, 0, 0));
    world.addComponent(origin, new CableLinkComponent());
    // Mass entity starting stretched beyond rest length
    const mass = world.createEntity();
    world.addComponent(mass, new PositionComponent(0, -startLength, 0));
    world.addComponent(mass, new VelocityComponent(0, 0, 0));
    world.addComponent(mass, new MassComponent(1.0));
    world.addComponent(mass, new GravityAffectedComponent());
    world.addComponent(mass, new CableLinkComponent());
    // Cable joint with rest length 1 between origin and mass
    const j = world.createEntity();
    world.addComponent(
      j,
      new CableJointComponent(
        origin,
        mass,
        restLength,
        new Vector3(0, 0, 0),
        new Vector3(0, -startLength, 0)
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
    world.setResource('dt', dt);
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
