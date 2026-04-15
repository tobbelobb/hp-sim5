import {
  World,
  EncoderComponent,
  MachineTagComponent,
  OrientationComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  SpoolStateComponent,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  StepperMotorComponent,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { getMachineMotorDiagnostics } from '../../../hp-sim-3d/app/motor-diagnostics.js';

describe('getMachineMotorDiagnostics (3D)', () => {
  test('reports per-axis missed steps from local spool rotation angles', () => {
    const world = new World();
    const axisA = world.createEntity();
    const axisB = world.createEntity();
    const stepAngle = (Math.PI * 2) / 50;

    const stepperA = new StepperMotorComponent(0.75, 0.1);
    const stepperB = new StepperMotorComponent(0.25, 0.0);

    world.addComponent(axisA, new MachineTagComponent('machine-1'));
    world.addComponent(axisA, new SpoolStateComponent('A'));
    world.addComponent(axisA, stepperA);
    const orientA = new OrientationComponent();
    orientA.quaternion.setFromAxisAngle({ x: 0.0, y: 0.0, z: 1.0 }, 0.65 - (4 * stepAngle));
    world.addComponent(axisA, orientA);

    world.addComponent(axisB, new MachineTagComponent('machine-1'));
    world.addComponent(axisB, new SpoolStateComponent('B'));
    world.addComponent(axisB, stepperB);
    const orientB = new OrientationComponent();
    orientB.quaternion.setFromAxisAngle({ x: 0.0, y: 0.0, z: 1.0 }, 0.25 + (1 * stepAngle));
    world.addComponent(axisB, orientB);

    const diagnostics = getMachineMotorDiagnostics(world, 'machine-1');

    expect(diagnostics).toEqual({
      totalMissedSteps: 5,
      motors: [
        { axis: 'A', missedSteps: 4 },
        { axis: 'B', missedSteps: 1 },
      ],
    });
  });

  test('ignores full-turn encoder offsets when computing missed steps', () => {
    const world = new World();
    const axisA = world.createEntity();

    world.addComponent(axisA, new MachineTagComponent('machine-1'));
    world.addComponent(axisA, new SpoolStateComponent('A'));
    world.addComponent(axisA, new StepperMotorComponent((2 * Math.PI) + 0.25, 0.0));
    const orientA = new OrientationComponent();
    orientA.quaternion.setFromAxisAngle({ x: 0.0, y: 0.0, z: 1.0 }, 0.25);
    world.addComponent(axisA, orientA);
    world.addComponent(axisA, new EncoderComponent((2 * Math.PI) + 0.25));

    const diagnostics = getMachineMotorDiagnostics(world, 'machine-1');

    expect(diagnostics).toEqual({
      totalMissedSteps: 0,
      motors: [
        { axis: 'A', missedSteps: 0 },
      ],
    });
  });

  test('still reports local error after removing a full-turn encoder offset', () => {
    const world = new World();
    const axisA = world.createEntity();
    const stepAngle = (Math.PI * 2) / 50;

    world.addComponent(axisA, new MachineTagComponent('machine-1'));
    world.addComponent(axisA, new SpoolStateComponent('A'));
    world.addComponent(axisA, new StepperMotorComponent(0.25, 0.0));
    const orientA = new OrientationComponent();
    orientA.quaternion.setFromAxisAngle({ x: 0.0, y: 0.0, z: 1.0 }, 0.25 + stepAngle);
    world.addComponent(axisA, orientA);
    world.addComponent(axisA, new EncoderComponent((2 * Math.PI) + 0.25 + stepAngle));

    const diagnostics = getMachineMotorDiagnostics(world, 'machine-1');

    expect(diagnostics).toEqual({
      totalMissedSteps: 1,
      motors: [
        { axis: 'A', missedSteps: 1 },
      ],
    });
  });

  test('unwraps the fallback quaternion angle near the target when no encoder is present', () => {
    const world = new World();
    const axisA = world.createEntity();

    world.addComponent(axisA, new MachineTagComponent('machine-1'));
    world.addComponent(axisA, new SpoolStateComponent('A'));
    world.addComponent(axisA, new StepperMotorComponent((2 * Math.PI) + 0.25, 0.0));
    const orientA = new OrientationComponent();
    orientA.quaternion.setFromAxisAngle({ x: 0.0, y: 0.0, z: 1.0 }, 0.25);
    world.addComponent(axisA, orientA);

    const diagnostics = getMachineMotorDiagnostics(world, 'machine-1');

    expect(diagnostics).toEqual({
      totalMissedSteps: 0,
      motors: [
        { axis: 'A', missedSteps: 0 },
      ],
    });
  });
});
