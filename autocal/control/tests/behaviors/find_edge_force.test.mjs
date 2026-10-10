import { findEdgeForce } from '../../behaviors/force_tuning.mjs';

describe('findEdgeForce', () => {
  test('recorded HP3 curve selects 2.3957N before the former 12.8849N plateau', async () => {
    const travel = [1.14, 17.41, 54.69, 110.45, 194.05, 289.79, 412.23,
      529.61, 639.22, 831.35, 1027.63, 1174.73, 1237.66, 1249.35];
    const result = await findEdgeForce(null, {
      forceStart: 0.22727151462399986, capForceLimit: 20,
      trialFn: async () => ({ travelDeg: travel.shift(), moved: true, stalled: false }),
    });
    expect(result.forceEdge).toBeCloseTo(2.39574867159);
    expect(result.samples).toHaveLength(10);
    expect(result.dMax).toBeCloseTo(831.35);
  });

  test('stops before the hockey stick with identical observation windows', async () => {
    const calls = [];
    const result = await findEdgeForce(null, {
      forceStart: 0.1, capForceLimit: 20,
      trialFn: async (force, label, options) => {
        calls.push({ force, options });
        return { travelDeg: 100 * force / (1 + force), moved: true, stalled: false };
      },
    });
    expect(result.forceEdge).toBeGreaterThan(0.3);
    expect(result.forceEdge).toBeLessThan(1);
    expect(calls.at(-1).force).toBeLessThan(2);
    expect(result.dMax).toBeLessThan(100);
    expect(result.knee.complianceRatio).toBe(0.5);
    expect(calls.every(c => c.options.sampleWindowMs === 30000 && !c.options.waitForStall)).toBe(true);
  });

  test('one small incremental gain does not confirm a knee', async () => {
    const travel = [10, 20, 21, 61, 141];
    const result = await findEdgeForce(null, {
      forceStart: 1, capForceLimit: 16, bracketFactor: 2,
      trialFn: async () => ({ travelDeg: travel.shift(), moved: true }),
    });
    expect(result.forceEdge).toBeNull();
    expect(result.samples).toHaveLength(5);
  });

  test('never promotes the force cap when the curve is still linear', async () => {
    const result = await findEdgeForce(null, {
      forceStart: 0.1, capForceLimit: 2,
      trialFn: async force => ({ travelDeg: force * 100, moved: true }),
    });
    expect(result.forceEdge).toBeNull();
    expect(result.reason).toContain('no comfortable knee');
  });

  test('ignores unmoved points and stops at a travel reversal', async () => {
    const trials = [
      { travelDeg: 100, moved: false },
      { travelDeg: 10, moved: true },
      { travelDeg: 50, moved: true },
      { travelDeg: 20, moved: true },
    ];
    const result = await findEdgeForce(null, {
      forceStart: 1, capForceLimit: 20, bracketFactor: 2,
      trialFn: async () => trials.shift(),
    });
    expect(result.forceEdge).toBe(4);
    expect(result.samples).toHaveLength(4);
  });

  test('stops ramping when the fixed anchor slips', async () => {
    const result = await findEdgeForce(null, {
      forceStart: 1, capForceLimit: 20, bracketFactor: 2,
      trialFn: async force => ({ travelDeg: force * 100, moved: true,
        fixedDriftDeg: force >= 4 ? -2 : 0 }),
    });
    expect(result.forceEdge).toBe(2);
    expect(result.samples.at(-1).force).toBe(4);
    expect(result.reason).toContain('fixed-anchor slip');
  });
});
