import {
  LINE_LAYERING_TOGGLE_KEYS,
  setLineLayeringFeatureFlags,
} from '../../../hp-sim/app/line-layering-flags.js';

const CORE_CABLE_MECHANICS_KEYS = Object.freeze([
  'layeringAttachmentUpdatePoints',
  'layeringMergeJoints',
  'layeringSplitJoints',
  'layeringHybridLinkStates',
]);

describe('setLineLayeringFeatureFlags', () => {
  test('writes all line layering toggle resources as enabled', () => {
    const resources = new Map();
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setLineLayeringFeatureFlags(world, true);

    expect(world.setResource).toHaveBeenCalledTimes(LINE_LAYERING_TOGGLE_KEYS.length);
    for (const key of LINE_LAYERING_TOGGLE_KEYS) {
      expect(resources.get(key)).toBe(true);
    }
  });

  test('writes all line layering toggle resources as disabled', () => {
    const resources = new Map();
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setLineLayeringFeatureFlags(world, false);

    expect(world.setResource).toHaveBeenCalledTimes(LINE_LAYERING_TOGGLE_KEYS.length);
    for (const key of LINE_LAYERING_TOGGLE_KEYS) {
      expect(resources.get(key)).toBe(false);
    }
  });

  test('does not overwrite core cable mechanics flags when disabling layering', () => {
    const resources = new Map();
    for (const key of CORE_CABLE_MECHANICS_KEYS) {
      resources.set(key, true);
    }
    const world = {
      setResource: jest.fn((key, value) => {
        resources.set(key, value);
      }),
    };

    setLineLayeringFeatureFlags(world, false);

    for (const key of CORE_CABLE_MECHANICS_KEYS) {
      expect(resources.get(key)).toBe(true);
    }
  });

  test('returns cleanly when world is missing', () => {
    expect(() => setLineLayeringFeatureFlags(null, true)).not.toThrow();
    expect(() => setLineLayeringFeatureFlags(undefined, true)).not.toThrow();
  });
});
