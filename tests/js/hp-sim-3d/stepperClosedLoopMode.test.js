import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  RigidGroupComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  StepperMotorComponent,
  StepperMotorSystem,
} from '../../../hp-sim-3d/app/hangprinter_common.js';

function getPlanarAngle(quaternion) {
  const axis = quaternion.transformVector(new Vector3(1.0, 0.0, 0.0));
  return Math.atan2(axis.y, axis.x);
}

describe('slideprinter 3D stepper closed-loop mode', () => {
  test('snaps position-controlled steppers to the rigid-group target when closed loop is enabled', () => {
    const world = new World();
    const system = new StepperMotorSystem();
    const stepperEntity = world.createEntity();
    const rigidGroupEntity = world.createEntity();
    const stepper = new StepperMotorComponent(0.8, 0.2);
    stepper.closedLoop = true;
    const rigidGroup = new RigidGroupComponent([stepperEntity]);
    rigidGroup.prevAngle = 0.1;
    const orient = new OrientationComponent();
    orient.quaternion.setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), -0.4);

    world.addComponent(stepperEntity, stepper);
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.0, 0.0, 0.7));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));
    world.addComponent(rigidGroupEntity, rigidGroup);

    system.update(world, 0.1);

    const updatedOrient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    expect(getPlanarAngle(updatedOrient.quaternion)).toBeCloseTo(0.7, 12);
    expect(angVel.omega.z).toBeCloseTo(0.0, 12);
  });

  test('keeps open-loop steppers on the torque model path by default', () => {
    const world = new World();
    const system = new StepperMotorSystem();
    const stepperEntity = world.createEntity();
    const orient = new OrientationComponent();
    orient.quaternion.setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), 0.2);

    world.addComponent(stepperEntity, new StepperMotorComponent(1.1, 0.0, 0.5, 1, 0.0));
    world.addComponent(stepperEntity, orient);
    world.addComponent(stepperEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(stepperEntity, new MomentOfInertiaComponent(1.0));

    system.update(world, 0.1);

    const updatedOrient = world.getComponent(stepperEntity, OrientationComponent);
    const angVel = world.getComponent(stepperEntity, AngularVelocityComponent);
    expect(getPlanarAngle(updatedOrient.quaternion)).toBeCloseTo(0.2, 12);
    expect(Math.abs(angVel.omega.z)).toBeGreaterThan(0.01);
  });
});
