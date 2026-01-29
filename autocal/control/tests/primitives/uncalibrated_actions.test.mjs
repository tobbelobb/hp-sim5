import { calculateReturnOrder, waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';

describe('uncalibrated_actions', () => {
  test('calculateReturnOrder', () => {
    const order = calculateReturnOrder({
      fixedAnchors: [2],
      currentLengths: [5, 12, 1],
    });
    expect(order).toEqual([1, 0, 2]);
  });

  test('waitForStableEncoders', async () => {
    const send = async () => ({ reply: '1 1 1' });
    const result = await waitForStableEncoders(send, ['40.0', '41.0', '42.0'], 1, {
      pollIntervalMs: 1,
      stableWindowMs: 2,
      toleranceDeg: 0.01,
    });
    expect(result.anglesDeg).toEqual([1, 1, 1]);
    expect(result.samples).toBeGreaterThanOrEqual(2);
  });
});
