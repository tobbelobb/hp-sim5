import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {
  angleToLength,
} from '../../primitives/uncalibrated_actions.mjs';
import { buildRrfSimulatorArgs } from '../../primitives/encoder_utils.mjs';
import { normalizeMachineType, resolveRrfSimulatorConfig } from '../../primitives/machine_type.mjs';
import {
  buildM666AdjustmentCommand,
  buildGlobalForceModes,
  buildSymmetricPulloutModes,
  combinations,
  generateSweepConfigs,
  MACHINE_CONFIGS,
  resolveFixedTargets,
  resolveForcedBaseRadii,
  resolveForcedBuildupFactor,
  resolveKlipperMmPerDegreeFromConfig,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from '../../behaviors/sweep_data_collection.mjs';
import { parseBridgeArgs } from '../../cli/collect_sweep_data.mjs';

describe('collect_sweep_data CLI helpers', () => {
  test('normalizeMachineType maps hangprinter aliases to hangprinter_4', () => {
    expect(normalizeMachineType('hangprinter_4')).toBe('hangprinter_4');
    expect(normalizeMachineType('hp4')).toBe('hangprinter_4');
    expect(normalizeMachineType('hp3')).toBe('hangprinter_4');
    expect(normalizeMachineType('hangprinter_3')).toBe('hangprinter_4');
  });

  test('resolveRrfSimulatorConfig uses hp3 configs for hangprinter_4 family', () => {
    expect(resolveRrfSimulatorConfig('hangprinter_4')).toBe('sys/config_hp3.g');
    expect(resolveRrfSimulatorConfig('hp4')).toBe('sys/config_hp3.g');
    expect(resolveRrfSimulatorConfig('hp3')).toBe('sys/config_hp3.g');
    expect(resolveRrfSimulatorConfig('hangprinter_3')).toBe('sys/config_hp3.g');
    expect(resolveRrfSimulatorConfig('hangprinter_4', { preferLineLayerConfig: true })).toBe('sys/config_hp3_w_line_layers.g');
  });

  test('buildRrfSimulatorArgs uses machine-aware config selection', () => {
    expect(buildRrfSimulatorArgs(8081, { machineType: 'slideprinter' })).toEqual([
      '--vsd', 'RRF/run/vsd', '-c', 'sys/config_slideprinter.g', '--server', '-p', '8081',
    ]);
    expect(buildRrfSimulatorArgs(8081, { machineType: 'hp3' })).toEqual([
      '--vsd', 'RRF/run/vsd', '-c', 'sys/config_hp3.g', '--server', '-p', '8081',
    ]);
    expect(buildRrfSimulatorArgs(8081, { machineType: 'hangprinter_4', preferLineLayerConfig: true })).toEqual([
      '--vsd', 'RRF/run/vsd', '-c', 'sys/config_hp3_w_line_layers.g', '--server', '-p', '8081',
    ]);
  });

  test('hangprinter_4 uses the visible axes from the selected hp3 simulator config', () => {
    const configPath = path.resolve(
      process.cwd(),
      'RRF',
      'run',
      'vsd',
      resolveRrfSimulatorConfig('hangprinter_4'),
    );
    const configText = readFileSync(configPath, 'utf8');
    const m584Line = configText
      .split(/\r?\n/)
      .find((line) => line.trim().startsWith('M584 '));

    expect(m584Line).toContain('X40.0');
    expect(m584Line).toContain('Y41.0');
    expect(m584Line).toContain('Z42.0');
    expect(m584Line).toContain('U43.0');
    expect(MACHINE_CONFIGS.hangprinter_4.axes).toEqual(['X', 'Y', 'Z', 'U']);
  });

  test('generateSweepConfigs', () => {
    const slideConfigs = generateSweepConfigs('slideprinter');
    expect(slideConfigs).toHaveLength(3);

    const hp4Configs = generateSweepConfigs('hangprinter_4');
    expect(hp4Configs).toHaveLength(3);
    expect(hp4Configs.every((cfg) => cfg.fixedAnchors.includes(3))).toBe(true);
    expect(hp4Configs.every((cfg) => cfg.driveAnchor !== 3 && cfg.sensorAnchor !== 3)).toBe(true);
  });

  test('combinations', () => {
    const result = combinations([0, 1, 2, 3], 2);
    expect(result).toHaveLength(6);
  });

  test('selectRepresentativeConfigs', () => {
    const allConfigs = generateSweepConfigs('hangprinter_4');
    const selected = selectRepresentativeConfigs(allConfigs, MACHINE_CONFIGS.hangprinter_4, 6);
    expect(selected).toHaveLength(3);
    expect(selected.every((cfg) => cfg.fixedAnchors.includes(3))).toBe(true);
  });

  test('validateSweepConfig', () => {
    const config = { numAnchors: 3, dimensions: 2, mustBeInFixedSet: [], axes: ['X', 'Y', 'Z'] };
    validateSweepConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 2 }, config);
    expect(() => validateSweepConfig({ fixedAnchors: [0, 0], driveAnchor: 1, sensorAnchor: 2 }, config))
      .toThrow(/duplicate/i);
    expect(() => validateSweepConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 3 }, config))
      .toThrow(/range/i);
    expect(() => validateSweepConfig(
      { fixedAnchors: [0, 1], driveAnchor: 3, sensorAnchor: 2 },
      MACHINE_CONFIGS.hangprinter_4,
    )).toThrow(/must remain fixed/i);
  });

  test('resolveFixedTargets respects per-anchor fixed bounds', () => {
    expect(resolveFixedTargets([0, 3], [25, 25], null, MACHINE_CONFIGS.hangprinter_4)).toEqual([25, 0]);
    expect(resolveFixedTargets([3, 2], null, 30, MACHINE_CONFIGS.hangprinter_4)).toEqual([0, 30]);
  });

  test('buildGlobalForceModes keeps forbidden anchors in position mode', () => {
    expect(buildGlobalForceModes(
      ['40.0', '41.0', '42.0', '43.0'],
      0.3182,
      MACHINE_CONFIGS.hangprinter_4.mustBeInFixedSet,
    )).toEqual([0.3182, 0.3182, 0.3182, 'position']);
  });

  test('buildSymmetricPulloutModes uses max force on free anchors', () => {
    expect(buildSymmetricPulloutModes({
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      movingAnchors: new Set([0]),
      fixedAnchors: [0, 3],
      forbiddenForceAnchors: MACHINE_CONFIGS.hangprinter_4.mustBeInFixedSet,
      forceMid: 0.3182,
      forceMax: 9.2035,
    })).toEqual(['position', 9.2035, 9.2035, 'position']);
  });

  test('angleToLength', () => {
    const mmPerDeg = [0.5, 1.0];
    expect(angleToLength(10, 0, mmPerDeg)).toBe(5);
    expect(angleToLength(10, 1, mmPerDeg)).toBe(10);
    expect(angleToLength(10, 2, mmPerDeg)).toBe(0);
  });

  test('resolveKlipperMmPerDegreeFromConfig maps m569 addresses and machine axes', () => {
    const tempDir = mkdtempSync(path.join(os.tmpdir(), 'hp-sim5-klipper-mmpd-'));
    const configPath = path.join(tempDir, 'printer.cfg');
    writeFileSync(configPath, `
[printer]
kinematics: winch
winch_mechanical_advantage: 2, 1, 1, 1

[stepper_a]
step_pin: gpiochip1/gpio0
dir_pin: gpiochip1/gpio1
enable_pin: gpiochip1/gpio2
rotation_distance: 360
m569_address: 40.0

[stepper_b]
step_pin: gpiochip1/gpio3
dir_pin: gpiochip1/gpio4
enable_pin: gpiochip1/gpio5
rotation_distance: 180
m569_address: 41.0

[stepper_c]
step_pin: gpiochip1/gpio6
dir_pin: gpiochip1/gpio7
enable_pin: gpiochip1/gpio8
rotation_distance: 90
m569_address: 42.0

[stepper_d]
step_pin: gpiochip1/gpio9
dir_pin: gpiochip1/gpio10
enable_pin: gpiochip1/gpio11
rotation_distance: 45
m569_address: 43.0
`, 'utf8');

    try {
      const mmPerDeg = resolveKlipperMmPerDegreeFromConfig({
        motorIds: ['40.0', '41.0', '42.0', '43.0'],
        axes: ['X', 'Y', 'Z', 'U'],
        configPath,
      });
      expect(mmPerDeg).toHaveLength(4);
      expect(mmPerDeg[0]).toBeCloseTo(0.5); // 360/(360*2)
      expect(mmPerDeg[1]).toBeCloseTo(0.5); // 180/(360*1)
      expect(mmPerDeg[2]).toBeCloseTo(0.25); // 90/(360*1)
      expect(mmPerDeg[3]).toBeCloseTo(0.125); // 45/(360*1)
    } finally {
      rmSync(tempDir, { recursive: true, force: true });
    }
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
      '--sensor-collection-force', '7',
    ]);
    expect(args.forceBuildupFactor).toBe('0.5');
    expect(args.forceBaseRadii).toBe('30,31,32');
    expect(args.preserveBuildupFactor).toBe(true);
    expect(args.sensorCollectionForce).toBe('7');
  });
});
