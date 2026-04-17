import {
  World,
  PositionComponent,
  OrientationComponent,
  MassComponent,
  MomentOfInertiaComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
  PBDCableConstraintSolver,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

describe('PBDCableConstraintSolver rigid-body endpoint mapping', () => {
  test('maps external member corrections onto the host rigid body', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(2.0));
    world.addComponent(body, new MomentOfInertiaComponent(1.0));

    const pinhole = world.createEntity();
    world.addComponent(pinhole, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(pinhole, new MassComponent(0.0));
    world.addComponent(
      pinhole,
      new RigidBodyMemberComponent(
        body,
        new Vector3(0.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([pinhole]));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(2.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        pinhole,
        anchor,
        1.0,
        new Vector3(0.0, 0.0, 0.0),
        new Vector3(2.0, 0.0, 0.0),
      ),
    );

    const pathEntity = world.createEntity();
    world.addComponent(
      pathEntity,
      new CablePathComponent(
        world,
        [jointEntity],
        ['attachment', 'attachment'],
        [true, true],
        Infinity,
        null,
        0.0,
      ),
    );

    const solver = new PBDCableConstraintSolver();
    solver.update(world, 0.0);

    expect(world.getComponent(body, PositionComponent).pos.x).toBeGreaterThan(0.5);
    expect(world.getComponent(pinhole, PositionComponent).pos.x).toBeCloseTo(0.0, 12);
  });

  test('keeps internal pinhole-to-spool corrections local to the spool', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(5.0));
    world.addComponent(body, new MomentOfInertiaComponent(5.0));

    const pinhole = world.createEntity();
    world.addComponent(pinhole, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(pinhole, new MassComponent(0.0));
    world.addComponent(
      pinhole,
      new RigidBodyMemberComponent(
        body,
        new Vector3(0.0, 0.0, 0.0),
      ),
    );

    const spool = world.createEntity();
    world.addComponent(spool, new PositionComponent(1.0, 0.0, 0.0));
    world.addComponent(spool, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(spool, new MassComponent(0.0));
    world.addComponent(spool, new MomentOfInertiaComponent(1.0));
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(1.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([pinhole, spool]));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromLocal(
        world,
        pinhole,
        spool,
        1.0,
        new Vector3(0.0, 0.0, 0.0),
        new Vector3(0.0, 1.0, 0.0),
      ),
    );

    const pathEntity = world.createEntity();
    world.addComponent(
      pathEntity,
      new CablePathComponent(
        world,
        [jointEntity],
        ['attachment', 'attachment'],
        [true, true],
        Infinity,
        null,
        0.0,
      ),
    );

    const solver = new PBDCableConstraintSolver();
    solver.update(world, 0.0);

	    const bodyPosition = world.getComponent(body, PositionComponent).pos;
	    const bodyOrientation = world.getComponent(body, OrientationComponent).quaternion;
	    const spoolOrientation = world.getComponent(spool, OrientationComponent).quaternion;

	    expect(bodyPosition.x).toBeCloseTo(0.0, 12);
	    expect(bodyPosition.y).toBeCloseTo(0.0, 12);
	    expect(bodyOrientation.z).toBeLessThan(-1e-4);
	    expect(Math.abs(spoolOrientation.z) + Math.abs(spoolOrientation.w - 1.0)).toBeGreaterThan(1e-4);
	  });
	});
