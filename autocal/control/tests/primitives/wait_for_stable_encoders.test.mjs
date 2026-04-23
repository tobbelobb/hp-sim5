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

  test('accepts an options object in the third argument slot', async () => {
    const send = async () => ({ reply: '0 0' });

    const result = await waitForStableEncoders(send, ['40.0', '41.0'], {
      speedup: 10,
      pollIntervalMs: 10,
      stableWindowMs: 20,
    });

    expect(result.anglesDeg).toEqual([0, 0]);
    expect(result.samples).toBeGreaterThanOrEqual(2);
  });

  test('treats 10 seconds of vibration without drift as stable', async () => {
    const readings = [-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5];
    let pollIdx = 0;
    let nowMs = 0;
    const send = async () => {
      const reading = readings[Math.min(pollIdx, readings.length - 1)];
      pollIdx += 1;
      return { reply: `${reading}` };
    };

    const result = await waitForStableEncoders(send, ['40.0'], 1, {
      pollIntervalMs: 1000,
      stableWindowMs: 2000,
      toleranceDeg: 0.1,
      sleepFn: async (ms) => {
        nowMs += ms;
      },
      nowFn: () => nowMs,
    });

    expect(result.anglesDeg).toEqual([-0.5]);
    expect(result.elapsedMs).toBe(10000);
    expect(result.samples).toBeGreaterThanOrEqual(10);
  });

  test('does not treat drift as vibration stability', async () => {
    const readings = [0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0];
    let pollIdx = 0;
    let nowMs = 0;
    const send = async () => {
      const reading = readings[Math.min(pollIdx, readings.length - 1)];
      pollIdx += 1;
      return { reply: `${reading}` };
    };

    await expect(waitForStableEncoders(send, ['40.0'], 1, {
      pollIntervalMs: 1000,
      stableWindowMs: 2000,
      toleranceDeg: 0.1,
      timeoutMs: 12000,
      sleepFn: async (ms) => {
        nowMs += ms;
      },
      nowFn: () => nowMs,
    })).rejects.toThrow('Timed out waiting for encoder stability after 12000ms');
  });
});
