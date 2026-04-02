import {
  CLOSED_LOOP_MOTOR_TOGGLE_KEYS,
  setClosedLoopMotorFeatureFlags,
} from '../../../hp-sim/app/closed-loop-flags.js';
import { STEPPER_CLOSED_LOOP_RESOURCE } from '../../../examples/js/slideprinter/slideprinter_common.js';

describe('setClosedLoopMotorFeatureFlags', () => {
  test('writes the closed-loop motor resource as enabled', () => {
    const resources = new Map();
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setClosedLoopMotorFeatureFlags(world, true);

    expect(CLOSED_LOOP_MOTOR_TOGGLE_KEYS).toEqual([STEPPER_CLOSED_LOOP_RESOURCE]);
    expect(world.setResource).toHaveBeenCalledTimes(CLOSED_LOOP_MOTOR_TOGGLE_KEYS.length);
    expect(resources.get(STEPPER_CLOSED_LOOP_RESOURCE)).toBe(true);
  });

  test('writes the closed-loop motor resource as disabled', () => {
    const resources = new Map();
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setClosedLoopMotorFeatureFlags(world, false);

    expect(resources.get(STEPPER_CLOSED_LOOP_RESOURCE)).toBe(false);
  });

  test('returns cleanly when world is missing', () => {
    expect(() => setClosedLoopMotorFeatureFlags(null, true)).not.toThrow();
    expect(() => setClosedLoopMotorFeatureFlags(undefined, true)).not.toThrow();
  });
});
