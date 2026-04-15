import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  StepperMotorComponent,
  StepperMotorSystem,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  SpoolStateComponent,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('slideprinter 3D stepper closed-loop mode', () => {
  test('snaps position-controlled steppers onto the local spool axis while preserving tilt', () => {
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
    expect(getSpoolRotationAngle(spoolState, updatedOrient.quaternion)).toBeCloseTo(0.6, 12);
    expect(angVel.omega.dot(worldAxis)).toBeCloseTo(0.0, 12);
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
});
