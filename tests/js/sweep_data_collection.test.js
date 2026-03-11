import {
  computeInitialForceSweepStepMm,
  expandSubSweepsForConfig,
  generateSweepConfigs,
  MACHINE_CONFIGS,
  resolveFixedTargets,
  scaleMeasuredMaxTravelMm,
} from '../../autocal/control/behaviors/sweep_data_collection.mjs';

describe('sweep config generation', () => {
  test('slideprinter sweeps cover both drive/sensor directions per fixed anchor', () => {
    const sweeps = generateSweepConfigs('slideprinter');
    expect(sweeps).toHaveLength(3);
    const roles = sweeps.map((cfg) => [cfg.fixedAnchors.join(','), cfg.driveAnchor, cfg.sensorAnchor]);
    expect(roles).toEqual(
      expect.arrayContaining([
        ['0', 1, 2],
        ['1', 0, 2],
        ['2', 0, 1],
      ]),
    );

    const expanded = expandSubSweepsForConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 2 }, MACHINE_CONFIGS.slideprinter);
    expect(expanded).toEqual(
      expect.arrayContaining([
        expect.objectContaining({ driveAnchor: 1, sensorAnchor: 2 }),
        expect.objectContaining({ driveAnchor: 2, sensorAnchor: 1 }),
      ]),
    );
  });

  test('forbidden sensor anchors stay out of sensor role', () => {
    const sweeps = generateSweepConfigs('hangprinter_4');
    const sensors = new Set(sweeps.map((cfg) => cfg.sensorAnchor));
    expect(sensors.has(3)).toBe(false);

    const cfg = sweeps.find((entry) => entry.fixedAnchors.includes(0) && entry.fixedAnchors.includes(3));
    expect(cfg).toBeDefined();
    const subSweeps = expandSubSweepsForConfig(cfg, MACHINE_CONFIGS.hangprinter_4);
    expect(subSweeps).toHaveLength(2);
    expect(subSweeps).toEqual(
      expect.arrayContaining([
        expect.objectContaining({ driveAnchor: 1, sensorAnchor: 2 }),
        expect.objectContaining({ driveAnchor: 2, sensorAnchor: 1 }),
      ]),
    );
    expect(subSweeps.every((entry) => entry.sensorAnchor !== 3)).toBe(true);
  });

  test('hangprinter_4 fixed targets stay within physical sign bounds', () => {
    expect(resolveFixedTargets([0, 3], [25, -25], null, MACHINE_CONFIGS.hangprinter_4)).toEqual([25, -25]);
    expect(resolveFixedTargets([0, 3], [-25, 25], null, MACHINE_CONFIGS.hangprinter_4)).toEqual([0, 0]);
  });
});

describe('force sweep initial rewind step', () => {
  test('uses base radius over a quarter turn', () => {
    expect(computeInitialForceSweepStepMm(30)).toBeCloseTo(30 * Math.PI / 2);
  });

  test('falls back to legacy step when base radius is unavailable', () => {
    expect(computeInitialForceSweepStepMm(Number.NaN)).toBe(100);
    expect(computeInitialForceSweepStepMm(0)).toBe(100);
  });

  test('hangprinter_4 size tuning scales measured max travel down', () => {
    expect(scaleMeasuredMaxTravelMm(MACHINE_CONFIGS.hangprinter_4, 100)).toBeCloseTo(80);
    expect(scaleMeasuredMaxTravelMm(MACHINE_CONFIGS.slideprinter, 100)).toBeCloseTo(100);
  });
});
