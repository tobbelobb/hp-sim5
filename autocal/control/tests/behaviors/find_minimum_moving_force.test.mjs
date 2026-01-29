import { findMinimumMovingForce, FORCE_TUNING_CONSTANTS } from '../../behaviors/force_tuning.mjs';

describe('findMinimumMovingForce', () => {
  test('finds threshold within tolerance', async () => {
    const threshold = 0.8;
    const calls = [];
    const trialFn = async (force) => {
      calls.push(force);
      return { moved: force >= threshold };
    };
    let returnCalls = 0;
    const returnToOriginFn = async () => {
      returnCalls += 1;
    };

    const result = await findMinimumMovingForce(() => {}, {
      motorIds: ['40.0', '41.0'],
      axes: ['A', 'B'],
      mmPerDeg: [1, 1],
      baseLow: 0.2,
      capForceLimit: 2.0,
      trialFn,
      returnToOriginFn,
    });

    expect(result.forceStart).toBeGreaterThanOrEqual(threshold);
    const allowed = threshold * (1 + FORCE_TUNING_CONSTANTS.AUTO_TUNE_RELATIVE_TOLERANCE)
      + FORCE_TUNING_CONSTANTS.AUTO_TUNE_ABSOLUTE_TOLERANCE;
    expect(result.forceStart).toBeLessThanOrEqual(allowed);
    expect(calls.length).toBeGreaterThan(0);
    expect(returnCalls).toBe(1);
  });
});
