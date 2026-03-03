import {
  angleToLength,
} from '../../primitives/uncalibrated_actions.mjs';
import {
  buildM666AdjustmentCommand,
  combinations,
  generateSweepConfigs,
  MACHINE_CONFIGS,
  resolveForcedBaseRadii,
  resolveForcedBuildupFactor,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from '../../behaviors/sweep_data_collection.mjs';
import { parseBridgeArgs } from '../../primitives/gcode_bridge.mjs';

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

  test('resolveForcedBuildupFactor defaults to no Q adjustment', () => {
    expect(resolveForcedBuildupFactor({})).toBeNull();
  });

  test('resolveForcedBuildupFactor preserves current Q when requested', () => {
    expect(resolveForcedBuildupFactor({ preserveBuildupFactor: true })).toBeNull();
  });

  test('resolveForcedBuildupFactor uses explicit Q override', () => {
    expect(resolveForcedBuildupFactor({ forceBuildupFactor: 0.63661977 })).toBeCloseTo(0.63661977);
  });

  test('resolveForcedBaseRadii normalizes radii to the machine axis count', () => {
    expect(resolveForcedBaseRadii({ forceBaseRadii: [30], numAnchors: 3 })).toEqual([30, 30, 30]);
    expect(resolveForcedBaseRadii({ forceBaseRadii: [30, 31, 32, 33], numAnchors: 3 })).toEqual([30, 31, 32]);
  });

  test('buildM666AdjustmentCommand composes only provided fields', () => {
    expect(buildM666AdjustmentCommand({
      forcedBaseRadii: null,
      forcedBuildupFactor: null,
    })).toBeNull();
    expect(buildM666AdjustmentCommand({
      forcedBaseRadii: [30, 30, 30],
      forcedBuildupFactor: null,
    })).toBe('M666 R30:30:30');
    expect(buildM666AdjustmentCommand({
      forcedBaseRadii: null,
      forcedBuildupFactor: 0.63661977,
    })).toBe('M666 Q0.63661977');
    expect(buildM666AdjustmentCommand({
      forcedBaseRadii: [30, 30, 30],
      forcedBuildupFactor: 0.63661977,
    })).toBe('M666 R30:30:30 Q0.63661977');
  });

  test('parseBridgeArgs captures spool override flags', () => {
    const args = parseBridgeArgs([
      '--force-buildup-factor', '0.5',
      '--force-base-radii', '30,31,32',
      '--preserve-buildup-factor',
    ]);
    expect(args.forceBuildupFactor).toBe('0.5');
    expect(args.forceBaseRadii).toBe('30,31,32');
    expect(args.preserveBuildupFactor).toBe(true);
  });
});
