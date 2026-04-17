import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  StepperMotorComponent,
  StepperMotorSystem,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { AngularMovementSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  SpoolStateComponent,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('slideprinter 3D stepper closed-loop mode', () => {
  test('snaps position-controlled steppers onto the locked spool axis', () => {
    const world = new World();
    const system = new StepperMotorSystem();
    const stepperEntity = world.createEntity();
    const stepper = new StepperMotorComponent(0.8, 0.2);
    stepper.closedLoop = true;
    const orient = new OrientationComponent();
    const swing = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), 0.45);
    const twist = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), -0.4);
    orient.quaternion.multiplyQuaternions(swing, twist).normalize();

    world.addComponent(stepperEntity, stepper);
    world.addComponent(stepperEntity, new SpoolStateComponent('A'));
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.2, -0.1, 0.7));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));

    system.update(world, 0.1);

    const updatedOrient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    const spoolState = world.getComponent(stepperEntity, SpoolStateComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrient.quaternion);
    expect(worldAxis.x).toBeCloseTo(0.0, 12);
    expect(worldAxis.y).toBeCloseTo(0.0, 12);
    expect(worldAxis.z).toBeCloseTo(1.0, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrient.quaternion)).toBeCloseTo(0.6, 12);
    expect(angVel.omega.length()).toBeCloseTo(0.0, 12);
  });

  test('keeps open-loop steppers on the torque model path by default', () => {
    const world = new World();
    const system = new StepperMotorSystem();
    const stepperEntity = world.createEntity();
    const orient = new OrientationComponent();
    orient.quaternion.setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), 0.2);

    world.addComponent(stepperEntity, new StepperMotorComponent(1.1, 0.0, 0.5, 1, 0.0));
    world.addComponent(stepperEntity, new SpoolStateComponent('A'));
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));

    system.update(world, 0.1);

    const updatedOrient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    const spoolState = world.getComponent(stepperEntity, SpoolStateComponent);
    const worldAxis = getSpoolWorldAxis(spoolState, updatedOrient.quaternion);
    expect(getSpoolRotationAngle(spoolState, updatedOrient.quaternion)).toBeCloseTo(0.2, 12);
    expect(Math.abs(angVel.omega.dot(worldAxis))).toBeGreaterThan(0.01);
  });

  test('applies equal-and-opposite motor reaction torque to the host rigid body', () => {
    const world = new World();
    const system = new StepperMotorSystem();

    const bodyEntity = world.createEntity();
    world.addComponent(bodyEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(bodyEntity, new MomentOfInertiaComponent(4.0));

    const stepperEntity = world.createEntity();
    const orient = new OrientationComponent();
    orient.quaternion.setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), 0.2);

    world.addComponent(stepperEntity, new StepperMotorComponent(1.1, 0.0, 0.5, 1, 0.0));
    world.addComponent(stepperEntity, new SpoolStateComponent('A'));
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));
    world.addComponent(stepperEntity, new RigidBodyMemberComponent(bodyEntity));

    system.update(world, 0.1);

    const spoolAngularVelocity = world.getComponent(stepperEntity, AngularVelocityComponent).omega;
    const bodyAngularVelocity = world.getComponent(bodyEntity, AngularVelocityComponent).omega;
    expect(spoolAngularVelocity.z).toBeGreaterThan(0.0);
    expect(bodyAngularVelocity.z).toBeCloseTo(-0.25 * spoolAngularVelocity.z, 12);
  });

  test('snaps rigid-body-member spools in local spool space', () => {
    const world = new World();
    const system = new StepperMotorSystem();

    const bodyEntity = world.createEntity();
    const bodyOrientation = new OrientationComponent();
    bodyOrientation.quaternion.setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), 0.45);
    world.addComponent(bodyEntity, bodyOrientation);

    const stepperEntity = world.createEntity();
    const stepper = new StepperMotorComponent(0.8, 0.2);
    stepper.closedLoop = true;
    const localOrientation = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), -0.2);
    const worldOrientation = new Quaternion()
      .multiplyQuaternions(bodyOrientation.quaternion, localOrientation)
      .normalize();
    const orient = new OrientationComponent();
    orient.quaternion.set(worldOrientation);
    const spoolState = new SpoolStateComponent('A', null, bodyOrientation.quaternion.clone());

    world.addComponent(stepperEntity, stepper);
    world.addComponent(stepperEntity, spoolState);
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.2, -0.1, 0.7));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));
    world.addComponent(stepperEntity, new RigidBodyMemberComponent(bodyEntity, null, localOrientation));

    system.update(world, 0.1);

    const member = world.getComponent(stepperEntity, RigidBodyMemberComponent);
    const updatedOrient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    const localSpoolState = {
      axisLocal: spoolState.axisLocal,
      referenceOrientation: new Quaternion(),
    };
    expect(getSpoolRotationAngle(localSpoolState, member.localOrientation)).toBeCloseTo(0.6, 12);
    expect(getSpoolRotationAngle(spoolState, updatedOrient.quaternion)).toBeCloseTo(0.6, 12);
    expect(angVel.omega.length()).toBeCloseTo(0.0, 12);
  });

  test('does not double-integrate rigid-body-member spool rotation in AngularMovementSystem', () => {
    const world = new World();
    const stepperSystem = new StepperMotorSystem();
    const angularMovementSystem = new AngularMovementSystem();

    const bodyEntity = world.createEntity();
    const bodyOrientation = new OrientationComponent();
    bodyOrientation.quaternion.setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), 0.45);
    world.addComponent(bodyEntity, bodyOrientation);

    const stepperEntity = world.createEntity();
    const localOrientation = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), 0.2);
    const worldOrientation = new Quaternion()
      .multiplyQuaternions(bodyOrientation.quaternion, localOrientation)
      .normalize();
    const orient = new OrientationComponent();
    orient.quaternion.set(worldOrientation);
    const spoolState = new SpoolStateComponent('A', null, bodyOrientation.quaternion.clone());

    world.addComponent(stepperEntity, new StepperMotorComponent(1.1, 0.0, 0.5, 1, 0.0));
    world.addComponent(stepperEntity, spoolState);
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));
    world.addComponent(stepperEntity, new RigidBodyMemberComponent(bodyEntity, null, localOrientation));

    stepperSystem.update(world, 0.1);

    const member = world.getComponent(stepperEntity, RigidBodyMemberComponent);
    const localSpoolState = {
      axisLocal: spoolState.axisLocal,
      referenceOrientation: new Quaternion(),
    };
    const angleAfterStepper = getSpoolRotationAngle(localSpoolState, member.localOrientation);
    const worldAngleAfterStepper = getSpoolRotationAngle(spoolState, orient.quaternion);
    expect(angleAfterStepper).toBeGreaterThan(0.2);

    angularMovementSystem.update(world, 0.1);

    expect(getSpoolRotationAngle(localSpoolState, member.localOrientation)).toBeCloseTo(angleAfterStepper, 12);
    expect(getSpoolRotationAngle(spoolState, orient.quaternion)).toBeCloseTo(worldAngleAfterStepper, 12);
  });
});
