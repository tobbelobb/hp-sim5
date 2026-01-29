import { waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';

describe('waitForStableEncoders', () => {
  test('resolves once encoder values stabilize', async () => {
    const send = async () => ({ reply: '0 0' });

    const result = await waitForStableEncoders(send, ['40.0', '41.0'], 1, {
      pollIntervalMs: 1,
      stableWindowMs: 2,
    });

    expect(result.anglesDeg).toEqual([0, 0]);
    expect(result.samples).toBeGreaterThanOrEqual(2);
  });
});
