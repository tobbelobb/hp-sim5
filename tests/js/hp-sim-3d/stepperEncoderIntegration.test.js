import {
  World,
  EncoderComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { EncoderUpdateSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';
import {
  StepperMotorComponent,
  StepperMotorSystem,
} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import {
  SpoolStateComponent,
  composeSpoolOrientation,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('stepper and encoder integration', () => {
  test('preserves unwrapped encoder turn history across stepper updates', () => {
    const world = new World();
    const entity = world.createEntity();
    const spoolState = new SpoolStateComponent('A');
    const orientation = new OrientationComponent();
    const encoder = new EncoderComponent();
    const stepperSystem = new StepperMotorSystem();
    const encoderSystem = new EncoderUpdateSystem();
    const degToRad = Math.PI / 180.0;

    world.addComponent(entity, new StepperMotorComponent());
    world.addComponent(entity, spoolState);
    world.addComponent(entity, orientation);
    world.addComponent(entity, encoder);
    world.addComponent(entity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(entity, new MomentOfInertiaComponent(1.0));

    orientation.quaternion.set(composeSpoolOrientation(spoolState, null, 170.0 * degToRad));
    stepperSystem.update(world, 0.0);
    encoderSystem.update(world, 0.0);
    expect(encoder.angle).toBeCloseTo(170.0 * degToRad, 6);

    orientation.quaternion.set(composeSpoolOrientation(spoolState, null, 190.0 * degToRad));
    stepperSystem.update(world, 0.0);
    encoderSystem.update(world, 0.0);
    expect(encoder.angle).toBeCloseTo(190.0 * degToRad, 6);

    orientation.quaternion.set(composeSpoolOrientation(spoolState, null, 370.0 * degToRad));
    stepperSystem.update(world, 0.0);
    encoderSystem.update(world, 0.0);
    expect(encoder.angle).toBeCloseTo(370.0 * degToRad, 6);
  });
});
