import {
  World,
  MachineTagComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  SpoolStateComponent,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  StepperMotorComponent,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { STEPPER_CLOSED_LOOP_RESOURCE } from '../../../hp-sim-3d/app/hangprinter_runtime.js';
import {
  FULL_STEPS_PER_SLIP,
  getMachineMotorDiagnostics,
  resetMachineMotorDiagnostics,
} from '../../../hp-sim-3d/app/motor-diagnostics.js';

// The physics engine counts slips; these tests set its outputs directly.
describe('getMachineMotorDiagnostics (3D)', () => {
  function addStepper(world, machineId, axis, slips, netSlips) {
    const entity = world.createEntity();
    const stepper = new StepperMotorComponent(0.0, 0.0);
    stepper.missedSteps = slips;
    stepper.currentMissedSteps = netSlips;
    world.addComponent(entity, new MachineTagComponent(machineId));
    world.addComponent(entity, new SpoolStateComponent(axis));
    world.addComponent(entity, stepper);
    return stepper;
  }

  test('reports slips per axis in full steps for one machine', () => {
    const world = new World();
    addStepper(world, 'machine-1', 'B', 1, -1);
    addStepper(world, 'machine-1', 'A', 3, 1);
    addStepper(world, 'machine-2', 'A', 7, 7);

    expect(getMachineMotorDiagnostics(world, 'machine-1')).toEqual({
      totalMissedSteps: 4 * FULL_STEPS_PER_SLIP,
      motors: [
        { axis: 'A', closedLoop: false, slips: 3, netSlips: 1, missedSteps: 3 * FULL_STEPS_PER_SLIP },
        { axis: 'B', closedLoop: false, slips: 1, netSlips: -1, missedSteps: FULL_STEPS_PER_SLIP },
      ],
    });
  });

  test('marks motors as closed loop when the world runs servos', () => {
    const world = new World();
    addStepper(world, 'machine-1', 'A', 0, 0);
    world.setResource(STEPPER_CLOSED_LOOP_RESOURCE, true);

    expect(getMachineMotorDiagnostics(world, 'machine-1').motors[0].closedLoop).toBe(true);
  });

  test('reset restarts the slip count but keeps the encoder baseline and net offset', () => {
    const world = new World();
    const a = addStepper(world, 'machine-1', 'A', 5, 5);
    const other = addStepper(world, 'machine-2', 'A', 2, 2);
    a.missedStepEncoderOffset = 6.28;

    resetMachineMotorDiagnostics(world, 'machine-1');

    expect(a.missedSteps).toBe(0);
    expect(a.currentMissedSteps).toBe(5);
    expect(a.missedStepEncoderOffset).toBe(6.28);
    expect(other.missedSteps).toBe(2);
  });
});
