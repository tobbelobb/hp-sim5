import fs from 'node:fs';
import { createHeadlessWorld } from '../../../hp-sim-3d/app/headlessWorld.js';
import { createFeatureFlagsController } from '../../../hp-sim-3d/app/featureFlagsController.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

test('the motor strength slider scales every holding torque from its machine value', () => {
  const { world } = createHeadlessWorld(fs.readFileSync('public/usd_scenes/cubecorners.usda', 'utf8'));
  try {
    const listeners = {};
    const slider = { value: '1', addEventListener: (type, listener) => { listeners[type] = listener; } };
    const label = { textContent: '' };
    const featureFlags = createFeatureFlagsController({ world, state: {}, toggles: { motorStrength: slider, motorStrengthValue: label } });
    featureFlags.bindUi();
    const torques = () => world.query([StepperMotorComponent])
      .map(id => world.getComponent(id, StepperMotorComponent).holdingTorque);
    expect(torques()).toEqual(Array(8).fill(1));
    for (const strength of ['2.5', '0.25', '1']) {
      slider.value = strength;
      listeners.input();
      expect(torques()).toEqual(Array(8).fill(Number(strength)));
      expect(label.textContent).toBe(`${Number(strength)} Nm`);
    }
    slider.value = '2';
    listeners.input();
    // A rebuilt scene gets fresh steppers at their machine value.
    for (const id of world.query([StepperMotorComponent])) {
      const stepper = world.getComponent(id, StepperMotorComponent);
      stepper.holdingTorque = stepper.machineHoldingTorque;
      delete stepper.machineHoldingTorque;
    }
    featureFlags.syncMotorStrengthState();
    expect(torques()).toEqual(Array(8).fill(2));
    expect(label.textContent).toBe('2 Nm');
    // Mixed motor ratings must be shown as a torque range, including after a
    // machine is loaded while a strength override is active.
    const stepper = world.getComponent(world.query([StepperMotorComponent])[0], StepperMotorComponent);
    stepper.machineHoldingTorque = 0.5;
    featureFlags.syncMotorStrengthState();
    expect(label.textContent).toBe('1–2 Nm');
    slider.value = '0.25';
    listeners.input();
    expect(label.textContent).toBe('0.125–0.25 Nm');
  } finally {
    world.getResource('cableSolverBackend')?.dispose();
  }
});
