import {
  angleToLength,
} from '../../primitives/uncalibrated_actions.mjs';
import {
  combinations,
  generateSweepConfigs,
  MACHINE_CONFIGS,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from '../../behaviors/sweep_data_collection.mjs';

describe('collect_sweep_data CLI helpers', () => {
  test('generateSweepConfigs', () => {
    const slideConfigs = generateSweepConfigs('slideprinter');
    expect(slideConfigs).toHaveLength(3);

    const hp4Configs = generateSweepConfigs('hangprinter_4');
    expect(hp4Configs).toHaveLength(6);
  });

  test('combinations', () => {
    const result = combinations([0, 1, 2, 3], 2);
    expect(result).toHaveLength(6);
  });

  test('selectRepresentativeConfigs', () => {
    const allConfigs = generateSweepConfigs('hangprinter_4');
    const selected = selectRepresentativeConfigs(allConfigs, MACHINE_CONFIGS.hangprinter_4, 6);
    expect(selected).toHaveLength(6);
    const drives = new Set(selected.map((c) => c.driveAnchor));
    expect(drives.size).toBeGreaterThanOrEqual(3);
  });

  test('validateSweepConfig', () => {
    const config = { numAnchors: 3, dimensions: 2, forbiddenSensors: [], axes: ['X', 'Y', 'Z'] };
    validateSweepConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 2 }, config);
    expect(() => validateSweepConfig({ fixedAnchors: [0, 0], driveAnchor: 1, sensorAnchor: 2 }, config))
      .toThrow(/duplicate/i);
    expect(() => validateSweepConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 3 }, config))
      .toThrow(/range/i);
  });

  test('angleToLength', () => {
    const mmPerDeg = [0.5, 1.0];
    expect(angleToLength(10, 0, mmPerDeg)).toBe(5);
    expect(angleToLength(10, 1, mmPerDeg)).toBe(10);
    expect(angleToLength(10, 2, mmPerDeg)).toBe(0);
  });
});
