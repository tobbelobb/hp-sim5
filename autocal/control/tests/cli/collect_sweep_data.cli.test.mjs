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
  computeSafeProbeMaxForceN,
  extractM669Anchors,
  generateSweepConfigs,
  MACHINE_CONFIGS,
  probeCompliance,
  probeComplianceWithAntiphase,
  resolveComplianceProbeMultiples,
  resolveFixedTargets,
  resolveForcedBaseRadii,
  resolveForcedBuildupFactor,
  resolveKlipperMmPerDegreeFromConfig,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from '../../behaviors/sweep_data_collection.mjs';
import { parseBridgeArgs } from '../../cli/collect_sweep_data.mjs';

describe('collect_sweep_data CLI helpers', () => {
  test('solver iterations accept a bounded integer override and otherwise preserve the machine setting', () => {
    expect(parseBridgeArgs([]).solverIterations).toBeNull();
    expect(parseBridgeArgs(['--solver-iterations', '8']).solverIterations).toBe(8);
    for (const value of ['0', '-1', '1.5', '65', 'bad']) {
      expect(() => parseBridgeArgs(['--solver-iterations', value])).toThrow('integer');
    }
  });

  test('open-loop motors are opt-in', () => {
    expect(parseBridgeArgs([]).openLoopMotors).toBe(false);
    expect(parseBridgeArgs(['--sim', '--open-loop-motors']).openLoopMotors).toBe(true);
  });

  test('rejects retired zero-tension collection', () => {
    expect(() => parseBridgeArgs(['--project-zero-tension'])).toThrow('retired');
  });

  test('machine aliases select their own config in both layering modes', () => {
    const cases = [
      ['hp3', 'hangprinter_3', 'hp3'], ['hangprinter_3', 'hangprinter_3', 'hp3'],
      ['hp4', 'hangprinter_4', 'hp4'], ['hangprinter_4', 'hangprinter_4', 'hp4'],
      ['v3', 'hangprinter_3', 'hp3'], ['v4', 'hangprinter_4', 'hp4'],
      ['four_high_anchors', 'skycam', 'skycam'], ['skycam', 'skycam', 'skycam'],
      ['cube_corners', 'cubecorners', 'cubecorners'], ['cubecorners', 'cubecorners', 'cubecorners'],
      ['slideprinter', 'slideprinter', 'slideprinter'],
    ];
    for (const [input, canonical, name] of cases) {
      expect(normalizeMachineType(input)).toBe(canonical);
      expect(resolveRrfSimulatorConfig(input)).toBe(`sys/config_${name}.g`);
      expect(resolveRrfSimulatorConfig(input, { preferLineLayerConfig: true })).toBe(`sys/config_${name}_w_line_layers.g`);
      expect(MACHINE_CONFIGS[canonical]).toBeDefined();
    }
  });

  test('buildRrfSimulatorArgs uses machine-aware config selection', () => {
    expect(buildRrfSimulatorArgs(8081, { machineType: 'slideprinter' })).toEqual([
      '--vsd', 'RRF/run/vsd', '-c', 'sys/config_slideprinter.g', '--server', '-p', '8081',
    ]);
    expect(buildRrfSimulatorArgs(8081, { machineType: 'hp3' })).toEqual([
      '--vsd', 'RRF/run/vsd', '-c', 'sys/config_hp3.g', '--server', '-p', '8081',
    ]);
    expect(buildRrfSimulatorArgs(8081, { machineType: 'hangprinter_4', preferLineLayerConfig: true })).toEqual([
      '--vsd', 'RRF/run/vsd', '-c', 'sys/config_hp4_w_line_layers.g', '--server', '-p', '8081',
    ]);
  });

  test('hangprinter_4 uses the visible axes from the selected hp4 simulator config', () => {
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

  test('buildSymmetricPulloutModes limits free-anchor preload from the start force', () => {
    expect(buildSymmetricPulloutModes({
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      movingAnchors: new Set([0]),
      fixedAnchors: [0, 3],
      forbiddenForceAnchors: MACHINE_CONFIGS.hangprinter_4.mustBeInFixedSet,
      forceMid: 0.3182,
      forceMax: 9.2035,
    })).toEqual(['position', 6.364, 6.364, 'position']);
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
    })).toBe('M666 Q0.63661977 K0 V0');
    expect(buildM666AdjustmentCommand({
      forcedBaseRadii: [30, 30, 30],
      forcedBuildupFactor: 0.63661977,
    })).toBe('M666 R30:30:30 Q0.63661977 K0 V0');
    expect(buildM666AdjustmentCommand({ forcedBuildupFactor: 0 })).toBe('M666 Q0');
  });

  test('parseBridgeArgs captures spool override and compliance-probe flags', () => {
    const args = parseBridgeArgs([
      '--force-buildup-factor', '0.5',
      '--force-base-radii', '30,31,32',
      '--preserve-buildup-factor',
      '--sensor-collection-force', '7',
      '--compliance-probe', '2,4,1',
    ]);
    expect(args.forceBuildupFactor).toBe('0.5');
    expect(args.forceBaseRadii).toBe('30,31,32');
    expect(args.preserveBuildupFactor).toBe(true);
    expect(args.sensorCollectionForce).toBe('7');
    expect(args.complianceProbe).toBe('2,4,1');
    expect(args.noComplianceProbe).toBe(false);

    const disabled = parseBridgeArgs(['--no-compliance-probe']);
    expect(disabled.noComplianceProbe).toBe(true);
    expect(disabled.complianceProbe).toBeNull();
  });

  test('resolveComplianceProbeMultiples is opt-in on hardware, default-on in simulation, and rejects multipliers < 1', () => {
    // Physical machines default to opt-in ([]).
    expect(resolveComplianceProbeMultiples({
      machineType: 'hangprinter_3',
      m666: { S: 20000 },
      complianceProbe: null,
      noComplianceProbe: false,
    })).toEqual([]);
    expect(resolveComplianceProbeMultiples({
      machineType: 'hangprinter_3',
      m666: { S: 20000 },
      complianceProbe: null,
      noComplianceProbe: false,
      isSimulation: false,
    })).toEqual([]);
    // Simulation enables [2, 3, 1] by default for flex-enabled machines.
    expect(resolveComplianceProbeMultiples({
      machineType: 'hangprinter_3',
      m666: { S: 20000 },
      complianceProbe: null,
      noComplianceProbe: false,
      isSimulation: true,
    })).toEqual([2, 3, 1]);
    expect(resolveComplianceProbeMultiples({
      machineType: 'cubecorners',
      m666: { S: 118500 },
      complianceProbe: null,
      noComplianceProbe: false,
      isSimulation: true,
    })).toEqual([2, 3, 1]);
    expect(resolveComplianceProbeMultiples({
      machineType: 'hangprinter_3',
      m666: { S: 0 },
      complianceProbe: null,
      noComplianceProbe: false,
      isSimulation: true,
    })).toEqual([]);
    expect(resolveComplianceProbeMultiples({
      machineType: 'hangprinter_3',
      m666: { S: 20000 },
      complianceProbe: null,
      noComplianceProbe: true,
      isSimulation: true,
    })).toEqual([]);
    // Explicit opt-in works on both hardware and simulation when all multipliers >= 1.0.
    expect(resolveComplianceProbeMultiples({
      machineType: 'hangprinter_3',
      m666: { S: 20000 },
      complianceProbe: '1.5,3,1',
      noComplianceProbe: false,
      isSimulation: false,
    })).toEqual([1.5, 3, 1]);
    // Disable tokens remain accepted.
    for (const token of ['0', 'off', 'false', 'none', 'disabled']) {
      expect(resolveComplianceProbeMultiples({
        machineType: 'hangprinter_3',
        m666: { S: 20000 },
        complianceProbe: token,
        noComplianceProbe: false,
        isSimulation: true,
      })).toEqual([]);
    }
    // Custom multipliers below 1.0 (which unload sensor tension below baseline) must be rejected.
    for (const bad of ['0.2,2,1', '0.9', '2,-1', 'abc']) {
      expect(() => resolveComplianceProbeMultiples({
        machineType: 'hangprinter_3',
        m666: { S: 20000 },
        complianceProbe: bad,
        noComplianceProbe: false,
        isSimulation: true,
      })).toThrow(/multipliers must be finite numbers >= 1\.0/i);
    }
  });

  test('computeSafeProbeMaxForceN skips missing/non-finite limits and caps held-line overload', () => {
    const anchors = [
      [0, -1900, -280],
      [1645.45, 950, -280],
      [-1645.45, 950, -280],
      [0, 0, 1900],
    ];
    // Missing or NaN M666 X must return null (never Infinity).
    expect(computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3],
      maxForceByAnchor: [NaN, 40, 40, 40],
      anchors,
      lengthsMm: [0, 0, 0, 0],
    })).toBeNull();
    expect(computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3],
      maxForceByAnchor: null,
      maxSensorForceN: NaN,
    })).toBeNull();

    // When a held line (e.g. drive anchor 1) has a tighter limit than the sensor,
    // full-machine static equilibrium must cap the sensor force below 0.85 * X_sensor.
    const capTightHeld = computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3],
      maxForceByAnchor: [40, 20, 40, 40],
      forceLow: 0.5,
      anchors,
      lengthsMm: [0, 0, 0, 0],
      weightN: 0.8,
    });
    expect(capTightHeld).not.toBeNull();
    expect(capTightHeld).toBeLessThan(0.85 * 40);
    expect(capTightHeld).toBeLessThanOrEqual(0.85 * 20 + 0.5);

    // Fail closed when anchor geometry, lengths, conditioning, or tautness cannot be established.
    expect(computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3],
      maxForceByAnchor: [40, 40, 40, 40],
      anchors: null,
      lengthsMm: [0, 0, 0, 0],
    })).toBeNull();
    expect(computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3],
      maxForceByAnchor: [40, 40, 40, 40],
      anchors,
      lengthsMm: null,
    })).toBeNull();
    // Coplanar (singular K_sys) anchors must fail closed.
    expect(computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3],
      maxForceByAnchor: [40, 40, 40, 40],
      anchors: [
        [0, -1900, 0],
        [1645.45, 950, 0],
        [-1645.45, 950, 0],
        [0, 500, 0],
      ],
      lengthsMm: [0, 0, 0, 0],
    })).toBeNull();

    // Statically indeterminate held sets (more than dims locked cables, e.g. 4 held in 3D)
    // have an unobservable baseline prestress nullspace ker(U_h) and MUST fail closed.
    const hp5Anchors = [
      [0, -1900, -280],
      [1900, 0, -280],
      [0, 1900, -280],
      [-1900, 0, -280],
      [0, 0, 1900],
    ];
    const hp5Pos = [0, 600, 0];
    const hp5Lengths = hp5Anchors.map((a) => (
      Math.hypot(a[0] - hp5Pos[0], a[1] - hp5Pos[1], a[2] - hp5Pos[2])
      - Math.hypot(a[0], a[1], a[2])
    ));
    expect(computeSafeProbeMaxForceN({
      baseForce: 5,
      sensorAnchor: 2,
      heldAnchors: [0, 1, 3, 4],
      maxForceByAnchor: [40, 60, 60, 60, 60],
      forceLow: 0.1,
      anchors: hp5Anchors,
      lengthsMm: hp5Lengths,
      weightN: 0.8,
    })).toBeNull();

    // Determinate HP5 sweep pose (3 held [0, 1, 4], 1 constant-force [2], 1 sensor [3])
    // has a unique static equilibrium baseline and succeeds cleanly.
    const hp5SweepPos = [0, -400, 200];
    const hp5SweepLengths = hp5Anchors.map((a) => (
      Math.hypot(a[0] - hp5SweepPos[0], a[1] - hp5SweepPos[1], a[2] - hp5SweepPos[2])
      - Math.hypot(a[0], a[1], a[2])
    ));
    const capHp5Determinate = computeSafeProbeMaxForceN({
      baseForce: 4,
      sensorAnchor: 3,
      heldAnchors: [0, 1, 4],
      constantForceAnchors: [2],
      constantForces: { 2: 0.05 },
      maxForceByAnchor: [40, 40, 40, 40, 40],
      forceLow: 0.05,
      anchors: hp5Anchors,
      lengthsMm: hp5SweepLengths,
      mechanicalAdvantageByAnchor: [2, 2, 2, 2, 4],
      strandsByAnchor: [2, 2, 2, 2, 4],
      extraLengthByAnchor: [429.1, 429.1, 429.1, 429.1, 352.3],
      weightN: 0.8,
    });
    expect(capHp5Determinate).not.toBeNull();
    expect(capHp5Determinate).toBeGreaterThan(12);
    expect(capHp5Determinate).toBeLessThanOrEqual(0.85 * 40);
  });

  test('extractM669Anchors maps canonical RRF anchor letters instead of G-code axis names', () => {
    const m669Hp5 = {
      A: [0, -1900, -280],
      B: [1900, 0, -280],
      C: [0, 1900, -280],
      D: [-1900, 0, -280],
      I: [0, 0, 1900],
    };
    expect(extractM669Anchors(m669Hp5, 5, 3)).toEqual([
      [0, -1900, -280],
      [1900, 0, -280],
      [0, 1900, -280],
      [-1900, 0, -280],
      [0, 0, 1900],
    ]);
    expect(extractM669Anchors({ A: [0, -1900, -280] }, 4, 3)).toBeNull();
  });

  test('probeCompliance propagates force-restoration failures and probeComplianceWithAntiphase verifies phase', async () => {
    const anchors = [
      [0, -1900, -280],
      [1645.45, 950, -280],
      [-1645.45, 950, -280],
      [0, 0, 1900],
    ];
    const makeSimClock = () => {
      let nowMs = 0;
      return {
        now: () => nowMs,
        sleep: async (ms) => { nowMs += Math.max(Number(ms) || 10, 10); },
      };
    };
    const baseOpts = {
      axes: ['X', 'Y', 'Z', 'U'],
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      mmPerDeg: [0.5, 0.5, 0.5, 0.5],
      feed: 1400,
      speedup: 1000,
      settleOptions: { stableWindowMs: 20, pollIntervalMs: 10, timeoutMs: 2000 },
      driveAnchor: 1,
      sensorAnchor: 2,
      fixedSet: new Set([0, 3]),
      forbidden: new Set([3]),
      measurementSensorForces: {},
      forceLow: 0.5,
      multiples: [2],
      maxForceByAnchor: [40, 40, 40, 40],
      m669Anchors: anchors,
      weightN: 0.8,
      stepsPerRev: 200,
      lastPoint: { sensor_force_n: 5, l_drive: 100, mu: [0, 0, 0, 0], sigma: [0, 0, 0, 0] },
      prevPoint: { sensor_force_n: 5, l_drive: 50, mu: [0, -100, -50, 0], sigma: [0, 0, 0, 0] },
      noiseOptions: { sampleCount: 2, sampleIntervalMs: 1, speedup: 1000 },
    };

    // 1. Restoration failure inside finally must NOT be swallowed.
    let failOnRestore = false;
    const sendRestoreFail = async (cmd) => {
      if (cmd.startsWith('M569.4') && cmd.includes('T0.0:0.0:5:0.0')) {
        failOnRestore = true;
        throw new Error('CAN bus lost during restore');
      }
      if (cmd.startsWith('M569.3')) return { reply: '0.0 0.0 -0.2 0.0' };
      return { reply: 'ok' };
    };
    sendRestoreFail.simulationClock = makeSimClock();
    await expect(probeCompliance(sendRestoreFail, baseOpts)).rejects.toThrow(/Failed to restore compliance probe base force/);
    expect(failOnRestore).toBe(true);

    // 2. Movement failure during antiphase nudge must propagate instead of being swallowed.
    const sendMoveFail = async (cmd) => {
      if (cmd.startsWith('G1 ')) throw new Error('motor stall on nudge');
      if (cmd.startsWith('M569.3')) return { reply: '0.0 0.0 -0.2 0.0' };
      return { reply: 'ok' };
    };
    sendMoveFail.simulationClock = makeSimClock();
    await expect(probeComplianceWithAntiphase(sendMoveFail, { ...baseOpts, multiples: [2, 1] }))
      .rejects.toThrow(/motor stall on nudge/);

    // 3. When nudge achieves ~180 deg electrical shift (3.6 deg for J=200), both probes are paired & verified.
    let nudged = false;
    const sendAntiphaseOk = async (cmd) => {
      if (cmd.startsWith('G1 ')) nudged = true;
      if (cmd.startsWith('M569.3')) {
        return { reply: nudged ? '0.0 -7.2 -3.6 0.0' : '0.0 0.0 -0.2 0.0' };
      }
      return { reply: 'ok' };
    };
    sendAntiphaseOk.simulationClock = makeSimClock();
    const paired = await probeComplianceWithAntiphase(sendAntiphaseOk, { ...baseOpts, multiples: [2, 1] });
    expect(paired).toHaveLength(2);
    expect(paired[0].pair_id).toBe(paired[1].pair_id);
    expect(paired[0].phase_role).toBe('primary');
    expect(paired[1].phase_role).toBe('antiphase');
    expect(paired[0].antiphase_verified).toBe(true);
    expect(paired[1].antiphase_verified).toBe(true);
    expect(paired[1].achieved_electrical_phase_deg).toBeCloseTo(180, 0);

    // 4. When nudge fails to reach antiphase (only 0.4 deg = 20 deg electrical), skip second probe.
    let shortNudged = false;
    const sendShortNudge = async (cmd) => {
      if (cmd.startsWith('G1 ')) shortNudged = true;
      if (cmd.startsWith('M569.3')) {
        return { reply: shortNudged ? '0.0 -1.0 -0.4 0.0' : '0.0 0.0 -0.2 0.0' };
      }
      return { reply: 'ok' };
    };
    sendShortNudge.simulationClock = makeSimClock();
    const unpaired = await probeComplianceWithAntiphase(sendShortNudge, { ...baseOpts, multiples: [2, 1] });
    expect(unpaired).toHaveLength(1);
    expect(unpaired[0].antiphase_verified).toBe(false);

    // 5. Direct probeCompliance calls must also reject multipliers below 1.0.
    await expect(probeCompliance(sendAntiphaseOk, { ...baseOpts, multiples: [0.2, 2] }))
      .rejects.toThrow(/multipliers must be finite numbers >= 1\.0/i);
  });
});

for (const type of ['hangprinter_5', 'hangprinter_5_pulley_reduced']) {
  test(`${type} bootstrap reels in its held low line and frees the opposing line`, () => {
    const config = MACHINE_CONFIGS[type];
    const sweeps = generateSweepConfigs(type);
    expect(sweeps).toHaveLength(4);
    for (const sweep of sweeps) {
      const low = sweep.fixedAnchors.find(axis => axis !== 4);
      expect(new Set([sweep.driveAnchor, sweep.sensorAnchor])).toEqual(new Set([(low+1)%4, (low+3)%4]));
      expect(resolveFixedTargets(sweep.fixedAnchors, null, 400, config)).toEqual([-400, 0]);
    }
    expect(resolveRrfSimulatorConfig(type)).toContain(type === 'hangprinter_5' ? 'config_hp5.g' : 'config_hp5_pulley_reduced.g');
  });
}

test('redundant pullout keeps the opposing unused winch compliant while reeling a fixed line', () => {
  expect(buildSymmetricPulloutModes({ motorIds: ['40','41','42','43','44'],
    movingAnchors: new Set([0]), fixedAnchors: [0,4], activeAnchors: [1,3],
    forbiddenForceAnchors: [4], forceLow: .02, forceMid: .5, forceMax: 2 }))
    .toEqual(['position', 2, .02, 2, 'position']);
});
