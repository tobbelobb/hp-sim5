import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidGroupComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  StepperMotorComponent,
  StepperMotorSystem,
  isStepperClosedLoopEnabled,
} from '../../../example_apps/js/slideprinter/slideprinter_common.js';

describe('slideprinter stepper closed-loop mode', () => {
  test('enables closed-loop mode from either the component flag or the world resource', () => {
    expect(isStepperClosedLoopEnabled({ getResource: () => false }, { closedLoop: false })).toBe(false);
    expect(isStepperClosedLoopEnabled({ getResource: () => true }, { closedLoop: false })).toBe(true);
    expect(isStepperClosedLoopEnabled({ getResource: () => false }, { closedLoop: true })).toBe(true);
  });

  test('snaps position-controlled steppers to the rigid-group target when closed loop is enabled', () => {
    const world = new World();
    const system = new StepperMotorSystem();
    const stepperEntity = world.createEntity();
    const rigidGroupEntity = world.createEntity();
    const stepper = new StepperMotorComponent(0.8, 0.2);
    stepper.closedLoop = true;
    const rigidGroup = new RigidGroupComponent([stepperEntity]);
    rigidGroup.prevAngle = 0.1;

    world.addComponent(stepperEntity, stepper);
    world.addComponent(stepperEntity, new OrientationComponent(-0.4));
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.7));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));
    world.addComponent(rigidGroupEntity, rigidGroup);

    system.update(world, 0.1);

    const orient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    expect(orient.angle).toBeCloseTo(0.7, 12);
    expect(angVel.angularVelocity).toBeCloseTo(0.0, 12);
  });

  test('keeps open-loop steppers on the torque model path by default', () => {
    const world = new World();
    const system = new StepperMotorSystem();
    const stepperEntity = world.createEntity();

    world.addComponent(stepperEntity, new StepperMotorComponent(1.1, 0.0, 0.5, 1, 0.0));
    world.addComponent(stepperEntity, new OrientationComponent(0.2));
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.0));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));

    system.update(world, 0.1);

    const orient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    expect(orient.angle).toBeCloseTo(0.2, 12);
    expect(Math.abs(angVel.angularVelocity)).toBeGreaterThan(0.01);
  });
});
