import {
  World,
  EncoderComponent,
  MachineTagComponent,
  OrientationComponent,
  RigidGroupComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  SpoolStateComponent,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  StepperMotorComponent,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { getMachineMotorDiagnostics } from '../../../hp-sim-3d/app/motor-diagnostics.js';

describe('getMachineMotorDiagnostics (3D)', () => {
  test('reports per-axis missed steps from encoder angles', () => {
    const world = new World();
    const groupEntity = world.createEntity();
    const axisA = world.createEntity();
    const axisB = world.createEntity();
    const stepAngle = (Math.PI * 2) / 50;

    const group = new RigidGroupComponent([axisA]);
    group.prevAngle = 0.15;
    world.addComponent(groupEntity, group);

    const stepperA = new StepperMotorComponent(0.75, 0.1);
    const stepperB = new StepperMotorComponent(0.25, 0.0);

    world.addComponent(axisA, new MachineTagComponent('machine-1'));
    world.addComponent(axisA, new SpoolStateComponent('A'));
    world.addComponent(axisA, stepperA);
    world.addComponent(axisA, new OrientationComponent());
    world.addComponent(axisA, new EncoderComponent(0.8 - (4 * stepAngle)));

    world.addComponent(axisB, new MachineTagComponent('machine-1'));
    world.addComponent(axisB, new SpoolStateComponent('B'));
    world.addComponent(axisB, stepperB);
    world.addComponent(axisB, new OrientationComponent());
    world.addComponent(axisB, new EncoderComponent(0.25 + (1 * stepAngle)));

    const diagnostics = getMachineMotorDiagnostics(world, 'machine-1');

    expect(diagnostics).toEqual({
      totalMissedSteps: 5,
      motors: [
        { axis: 'A', missedSteps: 4 },
        { axis: 'B', missedSteps: 1 },
      ],
    });
  });
});
