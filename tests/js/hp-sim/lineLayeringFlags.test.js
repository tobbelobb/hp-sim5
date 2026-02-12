import {
  LINE_LAYERING_FEATURE_KEYS,
  setLineLayeringFeatureFlags,
} from '../../../hp-sim/assets/line-layering-flags.js';

describe('setLineLayeringFeatureFlags', () => {
  test('writes all layering resources as enabled', () => {
    const resources = new Map();
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setLineLayeringFeatureFlags(world, true);

    expect(world.setResource).toHaveBeenCalledTimes(LINE_LAYERING_FEATURE_KEYS.length);
    for (const key of LINE_LAYERING_FEATURE_KEYS) {
      expect(resources.get(key)).toBe(true);
    }
  });

  test('writes all layering resources as disabled', () => {
    const resources = new Map();
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setLineLayeringFeatureFlags(world, false);

    expect(world.setResource).toHaveBeenCalledTimes(LINE_LAYERING_FEATURE_KEYS.length);
    for (const key of LINE_LAYERING_FEATURE_KEYS) {
      expect(resources.get(key)).toBe(false);
    }
  });

  test('returns cleanly when world is missing', () => {
    expect(() => setLineLayeringFeatureFlags(null, true)).not.toThrow();
    expect(() => setLineLayeringFeatureFlags(undefined, true)).not.toThrow();
  });
});
