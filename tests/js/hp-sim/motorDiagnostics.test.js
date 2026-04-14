import {
  World,
  MachineTagComponent,
  OrientationComponent,
  RigidGroupComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
  SpoolStateComponent,
  StepperMotorComponent,
} from '../../../example_apps/js/slideprinter/slideprinter_common.js';
import { getMachineMotorDiagnostics } from '../../../hp-sim/app/motor-diagnostics.js';

describe('getMachineMotorDiagnostics', () => {
  test('reports per-axis missed steps in the machine local frame', () => {
    const world = new World();
    const groupEntity = world.createEntity();
    const axisA = world.createEntity();
    const axisB = world.createEntity();
    const axisC = world.createEntity();
    const stepAngle = (Math.PI * 2) / 50;

    const group = new RigidGroupComponent([axisA]);
    group.prevAngle = 0.2;
    world.addComponent(groupEntity, group);

    const stepperA = new StepperMotorComponent(0.8, 0.1);
    const stepperB = new StepperMotorComponent(0.4, 0.0);
    const stepperC = new StepperMotorComponent(1.0, 0.0);
    stepperC.torqueMode = true;

    world.addComponent(axisA, new MachineTagComponent('machine-1'));
    world.addComponent(axisA, new SpoolStateComponent('A'));
    world.addComponent(axisA, stepperA);
    world.addComponent(axisA, new OrientationComponent(0.9 - (3 * stepAngle)));

    world.addComponent(axisB, new MachineTagComponent('machine-1'));
    world.addComponent(axisB, new SpoolStateComponent('B'));
    world.addComponent(axisB, stepperB);
    world.addComponent(axisB, new OrientationComponent(0.4 + (2 * stepAngle)));

    world.addComponent(axisC, new MachineTagComponent('machine-1'));
    world.addComponent(axisC, new SpoolStateComponent('C'));
    world.addComponent(axisC, stepperC);
    world.addComponent(axisC, new OrientationComponent(-5.0));

    const diagnostics = getMachineMotorDiagnostics(world, 'machine-1');

    expect(diagnostics).toEqual({
      totalMissedSteps: 5,
      motors: [
        { axis: 'A', missedSteps: 3 },
        { axis: 'B', missedSteps: 2 },
        { axis: 'C', missedSteps: 0 },
      ],
    });
  });
});
