import {
  World,
  PositionComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  PrevFinalPosComponent,
  AngularVelocityComponent,
  MassComponent,
  RadiusComponent,
  MomentOfInertiaComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver,
  _updateAttachmentPoints,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  tangentFromSphereToSphere,
} from '../../../src/js/cable_joints_3d/geometry3.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  AngularMovementSystem,
  PrevFinalOrientationSystem,
  PrevFinalPosSystem,
  PBDAngularVelocityUpdateSystem,
  RigidBodySyncSystem,
} from '../../../src/js/cable_joints_3d/commonSystems.js';
import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableFrictionSystem } from '../../../src/js/cable_joints_3d/cable_friction_system.js';

describe('PBDCableConstraintSolver rigid-body endpoint mapping', () => {
  function rotateBodyZ(world, bodyEntity, angle) {
    const q = world.getComponent(bodyEntity, OrientationComponent).quaternion;
    q.x = 0.0;
    q.y = 0.0;
    q.z = Math.sin(angle / 2.0);
    q.w = Math.cos(angle / 2.0);
  }

  test('frictionless rolling wheels expose their axis-only cable spin DOF', () => {
    const world = new World();
    world.setResource('dt', 1.0);

    const wheel = world.createEntity();
    world.addComponent(wheel, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(wheel, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(wheel, new MassComponent(-1.0));
    world.addComponent(wheel, new RadiusComponent(1.0));
    world.addComponent(wheel, new MomentOfInertiaComponent(0.1));
    world.addComponent(wheel, new SpoolStateComponent(null, new Vector3(0.0, 0.0, 1.0)));
    world.addComponent(
      wheel,
      new CableLinkComponent(
        0.0,
        0.0,
        0.0,
        null,
        null,
        new Vector3(0.0, 0.0, 1.0),
      ),
    );

    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(3.0, 0.0, 0.0));
    world.addComponent(anchor, new MassComponent(-1.0));

    const jointEntity = world.createEntity();
    world.addComponent(
      jointEntity,
      CableJointComponent.fromWorld(
        wheel,
        anchor,
        1.0,
        new Vector3(0.0, 1.0, 0.0),
        new Vector3(3.0, 0.0, 0.0),
      ),
    );

    const pathEntity = world.createEntity();
    world.addComponent(
      pathEntity,
      new CablePathComponent(
        world,
        [jointEntity],
        ['rolling', 'attachment'],
        [true, true],
        Infinity,
        null,
        0.0,
      ),
    );

    const solver = new PBDCableConstraintSolver();
    solver.update(world, 0.0);

    const wheelOrientation = world.getComponent(wheel, OrientationComponent).quaternion;
    expect(Math.abs(wheelOrientation.x)).toBeCloseTo(0.0, 12);
    expect(Math.abs(wheelOrientation.y)).toBeCloseTo(0.0, 12);
    expect(Math.abs(wheelOrientation.z) + Math.abs(wheelOrientation.w - 1.0)).toBeGreaterThan(1e-4);
  });

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
    world.addComponent(spool, new SpoolStateComponent(null, new Vector3(0.0, 0.0, 1.0)));
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

  test('onboard rolling links behind pinholes backdrive and roll rest length', () => {
    const world = new World();
    const dt = 1.0 / 60.0;
    const axis = new Vector3(0.0, 0.0, 1.0);
    const radius = 0.2;
    world.setResource('dt', dt);
    world.setResource('enableLayering', false);

    const addFixedPoint = (pos, cableLink = true) => {
      const entity = world.createEntity();
      world.addComponent(entity, new PositionComponent(pos.x, pos.y, pos.z));
      world.addComponent(entity, new MassComponent(-1.0));
      world.addComponent(entity, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
      if (cableLink) {
        world.addComponent(entity, new CableLinkComponent(pos.x, pos.y, pos.z, null, axis));
      }
      return entity;
    };

    const addWheelMember = (body, pos) => {
      const entity = world.createEntity();
      world.addComponent(entity, new PositionComponent(pos.x, pos.y, pos.z));
      world.addComponent(entity, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
      world.addComponent(entity, new PrevFinalOrientationComponent(0.0, 0.0, 0.0, 1.0));
      world.addComponent(entity, new AngularVelocityComponent(0.0, 0.0, 0.0));
      world.addComponent(entity, new MassComponent(0.0));
      world.addComponent(entity, new RadiusComponent(radius));
      world.addComponent(entity, new MomentOfInertiaComponent(1e-4));
      world.addComponent(entity, new SpoolStateComponent(null, axis));
      // The test includes CableFrictionSystem for pinhole transfer. Keep the
      // rolling spans stuck so wheel rotation is the only rolling transfer path.
      world.addComponent(entity, new CoefficientOfFrictionComponent(1000.0));
      world.addComponent(entity, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
      world.addComponent(
        entity,
        new CableLinkComponent(pos.x, pos.y, pos.z, null, null, axis),
      );
      world.addComponent(entity, new RigidBodyMemberComponent(body, pos));
      return entity;
    };

    const addPinholeMember = (body, pos) => {
      const entity = world.createEntity();
      world.addComponent(entity, new PositionComponent(pos.x, pos.y, pos.z));
      world.addComponent(entity, new MassComponent(0.0));
      world.addComponent(entity, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
      world.addComponent(entity, new CableLinkComponent(pos.x, pos.y, pos.z, null, axis));
      world.addComponent(entity, new RigidBodyMemberComponent(body, pos));
      return entity;
    };

    const addJoint = (entityA, entityB, attachmentA, attachmentB, restLength = null) => {
      const jointEntity = world.createEntity();
      world.addComponent(
        jointEntity,
        CableJointComponent.fromWorld(
          entityA,
          entityB,
          restLength ?? attachmentA.distanceTo(attachmentB),
          attachmentA,
          attachmentB,
        ),
      );
      return jointEntity;
    };

    const body = world.createEntity();
    world.addComponent(body, new PositionComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new PrevFinalOrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(body, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(body, new MassComponent(-1.0));
    world.addComponent(body, new MomentOfInertiaComponent(0.0));
    world.addComponent(body, new PrevFinalPosComponent(0.0, 0.0, 0.0));

    const pos0 = new Vector3(-3.0, 0.7, 0.0);
    const pos1 = new Vector3(-2.0, 0.2, 0.0);
    const pos2 = new Vector3(-1.0, 0.2, 0.0);
    const pos3 = new Vector3(0.0, 0.0, 0.0);
    const pos4 = new Vector3(1.0, 0.0, 0.0);
    const pos5 = new Vector3(2.0, 0.2, 0.0);
    const pos6 = new Vector3(4.0, 0.8, 0.0);

    const hybrid = addFixedPoint(pos0);
    world.addComponent(hybrid, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(hybrid, new PrevFinalOrientationComponent(0.0, 0.0, 0.0, 1.0));
    world.addComponent(hybrid, new RadiusComponent(radius));

    const pinholeExternal = addFixedPoint(pos1);
    const pinholeLeft = addPinholeMember(body, pos2);
    const wheelLeft = addWheelMember(body, pos3);
    const wheelRight = addWheelMember(body, pos4);
    const pinholeRight = addPinholeMember(body, pos5);
    const attachment = addFixedPoint(pos6);

    world.addComponent(body, new RigidBodyComponent([pinholeLeft, wheelLeft, wheelRight, pinholeRight]));

    const t0 = tangentFromSphereToPoint(pos1, pos0, radius, axis, false);
    const t2 = tangentFromPointToSphere(pos2, pos3, radius, axis, true);
    const t3 = tangentFromSphereToSphere(pos3, radius, true, pos4, radius, true, axis);
    const t4 = tangentFromSphereToPoint(pos5, pos4, radius, axis, true);

    const jointIds = [
      addJoint(hybrid, pinholeExternal, t0.a_sphere, t0.a_attach),
      addJoint(pinholeExternal, pinholeLeft, pos1, pos2),
      addJoint(pinholeLeft, wheelLeft, t2.a_attach, t2.a_sphere),
      addJoint(wheelLeft, wheelRight, t3.a_sphere, t3.b_sphere),
      addJoint(wheelRight, pinholeRight, t4.a_sphere, t4.a_attach),
      addJoint(pinholeRight, attachment, pos5, pos6, pos5.distanceTo(pos6) * 0.2),
    ];

    const pathEntity = world.createEntity();
    const path = new CablePathComponent(
      world,
      jointIds,
      ['hybrid', 'pinhole', 'pinhole', 'rolling', 'rolling', 'pinhole', 'attachment'],
      [false, true, true, true, true, true, true],
      Infinity,
      [0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0],
      0.0,
      0.0,
    );
    world.addComponent(pathEntity, path);

    const initialRestLengths = jointIds.map((jointId) => (
      world.getComponent(jointId, CableJointComponent).restLength
    ));
    const systems = [
      new PrevFinalPosSystem(),
      new PrevFinalOrientationSystem(),
      new AngularMovementSystem(),
      new RigidBodySyncSystem(),
      new CableAttachmentUpdateSystem(false),
      new CableAttachmentCacheSystem(),
      new CableFrictionSystem(),
      new PBDCableConstraintSolver(),
      new PBDAngularVelocityUpdateSystem(),
    ];

    for (let step = 0; step < 16; step++) {
      for (const system of systems) {
        system.update(world, dt);
      }
    }

    const leftMember = world.getComponent(wheelLeft, RigidBodyMemberComponent);
    const rightMember = world.getComponent(wheelRight, RigidBodyMemberComponent);
    const finalRestLengths = jointIds.map((jointId) => (
      world.getComponent(jointId, CableJointComponent).restLength
    ));
    const finalTotalRestLength = finalRestLengths.reduce((sum, value) => sum + value, 0.0)
      + path.stored.reduce((sum, value) => sum + value, 0.0);

    expect(Math.abs(rightMember.localOrientation.z) + Math.abs(rightMember.localOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(Math.abs(leftMember.localOrientation.z) + Math.abs(leftMember.localOrientation.w - 1.0)).toBeGreaterThan(1e-4);
    expect(Math.abs(finalRestLengths[4] - initialRestLengths[4])).toBeGreaterThan(1e-4);
    expect(Math.abs(finalRestLengths[3] - initialRestLengths[3])).toBeGreaterThan(1e-4);
    expect(finalRestLengths[1]).toBeLessThan(initialRestLengths[1]);
    expect(finalTotalRestLength).toBeCloseTo(path.totalRestLength, 8);
  });
});
