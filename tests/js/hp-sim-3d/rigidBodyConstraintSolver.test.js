import {
  World,
  PositionComponent,
  OrientationComponent,
  MassComponent,
  RadiusComponent,
  MomentOfInertiaComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  PBDCableConstraintSolver,
  _updateAttachmentPoints,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { tangentFromPointToSphere } from '../../../src/js/cable_joints_3d/geometry3.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

describe('PBDCableConstraintSolver rigid-body endpoint mapping', () => {
  function rotateBodyZ(world, bodyEntity, angle) {
    const q = world.getComponent(bodyEntity, OrientationComponent).quaternion;
    q.x = 0.0;
    q.y = 0.0;
    q.z = Math.sin(angle / 2.0);
    q.w = Math.cos(angle / 2.0);
  }

  test('external rigid-body-mounted spool winding includes host body twist', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(1.0));
    world.addComponent(body, new MomentOfInertiaComponent(1.0));

    const spool = world.createEntity();
    world.addComponent(spool, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(spool, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(spool, new RadiusComponent(1.0));
    world.addComponent(spool, new MassComponent(0.0));
    world.addComponent(
      spool,
      new CableLinkComponent(
        0.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(0.0, 0.0, 0.0),
      ),
    );
    world.addComponent(body, new RigidBodyComponent([spool]));

    const anchor = world.createEntity();
    const anchorPos = new Vector3(2.0, 0.0, 0.0);
    const initialSpoolAttachment = tangentFromPointToSphere(
      anchorPos,
      new Vector3(0.0, 0.0, 0.0),
      1.0,
      new Vector3(0.0, 0.0, 1.0),
      true,
    ).a_sphere;
    world.addComponent(anchor, new PositionComponent(anchorPos.x, anchorPos.y, anchorPos.z));
    world.addComponent(anchor, new MassComponent(-1.0));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        anchor,
        spool,
        2.0,
        anchorPos,
        initialSpoolAttachment,
      ),
    );

    const pathEntity = world.createEntity();
    world.addComponent(
      pathEntity,
      new CablePathComponent(
        world,
        [jointEntity],
        ['attachment', 'hybrid'],
        [true, true],
        Infinity,
        [0.0, 1.0],
        0.0,
      ),
    );
    const restBefore = world.getComponent(jointEntity, CableJointComponent).restLength;

    rotateBodyZ(world, body, 0.25);
    _updateAttachmentPoints(world);

    const restAfter = world.getComponent(jointEntity, CableJointComponent).restLength;
    expect(Math.abs(restAfter - restBefore)).toBeGreaterThan(1e-4);
  });

  test('same-body pinhole-to-spool winding ignores host body twist', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(1.0));
    world.addComponent(body, new MomentOfInertiaComponent(1.0));

    const pinhole = world.createEntity();
    world.addComponent(pinhole, new PositionComponent(2.0, 0.0, 0.0));
    world.addComponent(pinhole, new MassComponent(0.0));
    world.addComponent(
      pinhole,
      new RigidBodyMemberComponent(
        body,
        new Vector3(2.0, 0.0, 0.0),
      ),
    );

    const spool = world.createEntity();
    world.addComponent(spool, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(spool, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(spool, new RadiusComponent(1.0));
    world.addComponent(spool, new MassComponent(0.0));
    world.addComponent(
      spool,
      new CableLinkComponent(
        0.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(0.0, 0.0, 0.0),
      ),
    );
    world.addComponent(body, new RigidBodyComponent([pinhole, spool]));

    const pinholePos = new Vector3(2.0, 0.0, 0.0);
    const initialSpoolAttachment = tangentFromPointToSphere(
      pinholePos,
      new Vector3(0.0, 0.0, 0.0),
      1.0,
      new Vector3(0.0, 0.0, 1.0),
      true,
    ).a_sphere;

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        pinhole,
        spool,
        2.0,
        pinholePos,
        initialSpoolAttachment,
      ),
    );

    const pathEntity = world.createEntity();
    world.addComponent(
      pathEntity,
      new CablePathComponent(
        world,
        [jointEntity],
        ['pinhole', 'hybrid'],
        [true, true],
        Infinity,
        [0.0, 1.0],
        0.0,
      ),
    );
    const restBefore = world.getComponent(jointEntity, CableJointComponent).restLength;

    rotateBodyZ(world, body, 0.25);
    _updateAttachmentPoints(world);

    const restAfter = world.getComponent(jointEntity, CableJointComponent).restLength;
    expect(restAfter).toBeCloseTo(restBefore, 12);
  });

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
	    expect(Math.abs(bodyOrientation.x)).toBeCloseTo(0.0, 12);
	    expect(Math.abs(bodyOrientation.y)).toBeCloseTo(0.0, 12);
	    expect(Math.abs(bodyOrientation.z)).toBeCloseTo(0.0, 12);
	    expect(Math.abs(spoolOrientation.z) + Math.abs(spoolOrientation.w - 1.0)).toBeGreaterThan(1e-4);
	  });

  test('external cable corrections can backdrive a rigid-body-mounted spool member', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(2.0));
    world.addComponent(body, new MomentOfInertiaComponent(5.0));

    const spool = world.createEntity();
    world.addComponent(spool, new PositionComponent(1.0, 0.0, 0.0));
    world.addComponent(spool, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(spool, new MassComponent(0.0));
    world.addComponent(spool, new MomentOfInertiaComponent(0.1));
    world.addComponent(
      spool,
      new CableLinkComponent(
        1.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(1.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([spool]));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(2.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        spool,
        anchor,
        1.0,
        new Vector3(1.0, 1.0, 0.0),
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

    const bodyPosition = world.getComponent(body, PositionComponent).pos;
    const bodyOrientation = world.getComponent(body, OrientationComponent).quaternion;
    const spoolOrientation = world.getComponent(spool, OrientationComponent).quaternion;
    const spoolMember = world.getComponent(spool, RigidBodyMemberComponent);

    expect(bodyPosition.x).toBeGreaterThan(0.0);
    expect(Math.abs(bodyOrientation.z) + Math.abs(bodyOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(Math.abs(spoolOrientation.x)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.y)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.z) + Math.abs(spoolOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(Math.abs(spoolMember.localOrientation.z) + Math.abs(spoolMember.localOrientation.w - 1.0)).toBeGreaterThan(1e-4);
  });

  test('closed-loop rigid-body-mounted steppers do not expose a cable-driven spin DOF', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(2.0));
    world.addComponent(body, new MomentOfInertiaComponent(5.0));

    const spool = world.createEntity();
    world.addComponent(spool, new PositionComponent(1.0, 0.0, 0.0));
    world.addComponent(spool, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(spool, new MassComponent(0.0));
    world.addComponent(spool, new MomentOfInertiaComponent(0.1));
    world.addComponent(
      spool,
      new CableLinkComponent(
        1.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );
    const stepper = new StepperMotorComponent();
    stepper.closedLoop = true;
    world.addComponent(spool, stepper);
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(1.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([spool]));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(2.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        spool,
        anchor,
        1.0,
        new Vector3(1.0, 1.0, 0.0),
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

    const bodyPosition = world.getComponent(body, PositionComponent).pos;
    const bodyOrientation = world.getComponent(body, OrientationComponent).quaternion;
    const spoolMember = world.getComponent(spool, RigidBodyMemberComponent);

    expect(bodyPosition.x).toBeGreaterThan(0.0);
    expect(Math.abs(bodyOrientation.z) + Math.abs(bodyOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(Math.abs(spoolMember.localOrientation.x)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolMember.localOrientation.y)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolMember.localOrientation.z)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolMember.localOrientation.w - 1.0)).toBeCloseTo(0.0, 12);
  });

  test('standalone closed-loop steppers are excluded from cable solver DOFs', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const spool = world.createEntity();
    world.addComponent(spool, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(spool, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(spool, new MassComponent(2.0));
    world.addComponent(spool, new MomentOfInertiaComponent(0.1));
    world.addComponent(
      spool,
      new CableLinkComponent(
        0.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );
    const stepper = new StepperMotorComponent();
    stepper.closedLoop = true;
    world.addComponent(spool, stepper);

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(2.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        spool,
        anchor,
        1.0,
        new Vector3(0.0, 1.0, 0.0),
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

    const spoolPosition = world.getComponent(spool, PositionComponent).pos;
    const spoolOrientation = world.getComponent(spool, OrientationComponent).quaternion;

    expect(spoolPosition.x).toBeCloseTo(0.0, 12);
    expect(spoolPosition.y).toBeCloseTo(0.0, 12);
    expect(spoolPosition.z).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.x)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.y)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.z)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.w - 1.0)).toBeCloseTo(0.0, 12);
  });

  test('external pinhole cable corrections can backdrive a downstream rigid-body-mounted spool member', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new MassComponent(2.0));
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
    world.addComponent(spool, new RadiusComponent(1.0));
    world.addComponent(spool, new MomentOfInertiaComponent(0.1));
    world.addComponent(
      spool,
      new CableLinkComponent(
        1.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );
    world.addComponent(
      spool,
      new RigidBodyMemberComponent(
        body,
        new Vector3(1.0, 0.0, 0.0),
      ),
    );

    world.addComponent(body, new RigidBodyComponent([pinhole, spool]));

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(2.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const outerJoint = world.createEntity();
    world.addComponent(
      outerJoint,
      CableJointComponent.fromWorld(
        anchor,
        pinhole,
        1.0,
        new Vector3(2.0, 0.0, 0.0),
        new Vector3(0.0, 0.0, 0.0),
      ),
    );

    const innerJoint = world.createEntity();
    world.addComponent(
      innerJoint,
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
        [outerJoint, innerJoint],
        ['attachment', 'pinhole', 'hybrid'],
        [true, true, true],
        Infinity,
        [0.0, 0.0, 0.2],
        0.0,
      ),
    );

    const solver = new PBDCableConstraintSolver();
    solver.update(world, 0.0);

    const bodyPosition = world.getComponent(body, PositionComponent).pos;
    const spoolOrientation = world.getComponent(spool, OrientationComponent).quaternion;
    const spoolMember = world.getComponent(spool, RigidBodyMemberComponent);

    expect(bodyPosition.x).toBeGreaterThan(0.0);
    expect(Math.abs(spoolOrientation.x)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.y)).toBeCloseTo(0.0, 12);
    expect(Math.abs(spoolOrientation.z) + Math.abs(spoolOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(Math.abs(spoolMember.localOrientation.z) + Math.abs(spoolMember.localOrientation.w - 1.0)).toBeGreaterThan(1e-4);
  });
});
