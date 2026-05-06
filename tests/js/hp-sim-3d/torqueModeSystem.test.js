import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  StepperMotorComponent,
  StepperMotorSystem,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import {
  TorqueModeSystem,
  readTorqueModeCableLoadTorque,
} from '../../../hp-sim-3d/app/torqueModeSystem.js';
import {
  SpoolStateComponent,
  getSpoolRotationAngle,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('3D torque mode system', () => {
  function addTorqueModeStepper(world, targetTorque = 0.5) {
    const entity = world.createEntity();
    const stepper = new StepperMotorComponent();
    stepper.torqueMode = true;
    stepper.targetTorque = targetTorque;

    world.addComponent(entity, stepper);
    world.addComponent(entity, new SpoolStateComponent('A'));
    world.addComponent(entity, new OrientationComponent());
    world.addComponent(entity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(entity, new MomentOfInertiaComponent(2.0));
    return entity;
  }

  test('StepperMotorSystem leaves torque-mode steppers to TorqueModeSystem', () => {
    const world = new World();
    const entity = addTorqueModeStepper(world);
    const stepperSystem = new StepperMotorSystem();

    stepperSystem.update(world, 0.1);

    const spoolState = world.getComponent(entity, SpoolStateComponent);
    const orientation = world.getComponent(entity, OrientationComponent);
    const angularVelocity = world.getComponent(entity, AngularVelocityComponent);
    expect(getSpoolRotationAngle(spoolState, orientation.quaternion)).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.length()).toBeCloseTo(0.0, 12);
  });

  test('applies target torque to angular velocity', () => {
    const world = new World();
    const entity = addTorqueModeStepper(world, 0.5);
    const torqueModeSystem = new TorqueModeSystem();

    torqueModeSystem.update(world, 0.1);

    const angularVelocity = world.getComponent(entity, AngularVelocityComponent);
    expect(angularVelocity.omega.x).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.y).toBeCloseTo(0.0, 12);
    expect(angularVelocity.omega.z).toBeCloseTo(0.025, 12);
  });

  test('subtracts signed cable load torque from the spool ODE', () => {
    const world = new World();
    const entity = addTorqueModeStepper(world, 0.5);
    world.setResource('torqueModeCableLoadTorques', new Map([[entity, -0.2]]));
    const torqueModeSystem = new TorqueModeSystem();

    torqueModeSystem.update(world, 0.1);

    const angularVelocity = world.getComponent(entity, AngularVelocityComponent);
    expect(readTorqueModeCableLoadTorque(world, entity)).toBeCloseTo(-0.2, 12);
    expect(angularVelocity.omega.z).toBeCloseTo(0.015, 12);
  });

  test('applies reaction torque and integrates rigid-body member spool twist', () => {
    const world = new World();
    const bodyEntity = world.createEntity();
    world.addComponent(bodyEntity, new OrientationComponent());
    world.addComponent(bodyEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));

    const entity = addTorqueModeStepper(world, 0.5);
    world.addComponent(bodyEntity, new RigidBodyComponent([entity]));
    world.addComponent(entity, new RigidBodyMemberComponent(bodyEntity));

    const torqueModeSystem = new TorqueModeSystem();
    torqueModeSystem.update(world, 0.1);

    const member = world.getComponent(entity, RigidBodyMemberComponent);
    const spoolState = world.getComponent(entity, SpoolStateComponent);
    const orientation = world.getComponent(entity, OrientationComponent);
    const bodyAngularVelocity = world.getComponent(bodyEntity, AngularVelocityComponent);
    const localSpoolState = {
      axisLocal: spoolState.axisLocal,
      referenceOrientation: world.getComponent(bodyEntity, OrientationComponent).quaternion,
    };
    expect(getSpoolRotationAngle(localSpoolState, member.localOrientation)).toBeCloseTo(0.0025, 12);
    expect(getSpoolRotationAngle(spoolState, orientation.quaternion)).toBeCloseTo(0.0025, 12);
    expect(bodyAngularVelocity.omega.z).toBeCloseTo(-0.025, 12);
  });

  test('does not treat external cable load as motor-body reaction torque', () => {
    const world = new World();
    const bodyEntity = world.createEntity();
    world.addComponent(bodyEntity, new OrientationComponent());
    world.addComponent(bodyEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));

    const entity = addTorqueModeStepper(world, 0.5);
    world.addComponent(bodyEntity, new RigidBodyComponent([entity]));
    world.addComponent(entity, new RigidBodyMemberComponent(bodyEntity));
    world.setResource('torqueModeCableLoadTorques', new Map([[entity, -0.2]]));

    const torqueModeSystem = new TorqueModeSystem();
    torqueModeSystem.update(world, 0.1);

    const spoolAngularVelocity = world.getComponent(entity, AngularVelocityComponent);
    const bodyAngularVelocity = world.getComponent(bodyEntity, AngularVelocityComponent);
    expect(spoolAngularVelocity.omega.z).toBeCloseTo(0.015, 12);
    expect(bodyAngularVelocity.omega.z).toBeCloseTo(-0.025, 12);
  });
});
