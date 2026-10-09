import { waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';

describe('waitForStableEncoders', () => {
  async function replay(readingAt, { pollIntervalMs = 500, ...options } = {}) {
    let nowMs = 0;
    const send = async () => ({ reply: readingAt(nowMs) });
    return waitForStableEncoders(send, ['40.0', '41.0'], 1, {
      pollIntervalMs,
      stableWindowMs: 1500,
      toleranceDeg: 0.1,
      timeoutMs: 15000,
      sleepFn: async (ms) => { nowMs += ms; },
      nowFn: () => nowMs,
      ...options,
    });
  }

  test('uses the recent quiet window while retaining older vibration history', async () => {
    const result = await replay((ms) => `${ms < 1000 ? 10 : 0} 0`);
    expect(result.elapsedMs).toBe(2500);
    expect(result.anglesDeg).toEqual([0, 0]);
  });

  test('requires a full quiet window on every motor', async () => {
    const result = await replay((ms) => `0 ${ms < 3000 ? 30 - ms / 100 : 0}`);
    expect(result.elapsedMs).toBe(4500);
  });

  test('covers the quiet-window boundary when polls have jitter', async () => {
    const result = await replay((ms) => `${ms < 1100 ? 10 : 0} 0`, {
      pollIntervalMs: 510,
    });
    expect(result.elapsedMs).toBe(3060);
  });

  test('keeps the previous observation when transport exceeds the entire history window', async () => {
    const result = await replay(() => '0 0', {
      speedup: 25, pollIntervalMs: 20, stableWindowMs: 60, vibrationWindowMs: 200,
      sleepFn: undefined,
      // The requested speed must not shorten the selected clock's window.
      ...(() => {
        let now = 0;
        return { nowFn: () => now, sleepFn: async () => { now += 272; } };
      })(),
    });
    expect(result.elapsedMs).toBe(272);
    expect(result.samples).toBe(2);
  });

  test('stationary vibration covers a boundary between polls', async () => {
    const result = await replay(ms => `${Math.round(ms / 510) % 2 ? .5 : -.5} 0`, {
      pollIntervalMs: 510, stableWindowMs: 1020, vibrationWindowMs: 5000,
      toleranceDeg: .1,
    });
    expect(result.elapsedMs).toBe(5100);
  });

  test('hardware settling has a finite default timeout', async () => {
    let now = 0;
    const send = async () => ({ reply: `${now} 0` });
    await expect(waitForStableEncoders(send, ['A', 'B'], 25, {
      nowFn: () => now, sleepFn: async ms => { now += ms; },
    })).rejects.toThrow('after 30000ms');
  });

  test('a stalled simulation reaches the wall deadline and reports progress', async () => {
    let wall = 0;
    const progress = [];
    const send = async () => ({ reply: '0 0' });
    send.simulationClock = { now: () => 0, sleep: async () => { wall += 1000; } };
    await expect(waitForStableEncoders(send, ['A', 'B'], 25, {
      wallTimeoutMs: 6000, wallNowFn: () => wall, onProgress: message => progress.push(message),
    })).rejects.toThrow('wall-clock deadline');
    expect(progress[0]).toContain('5.0s wall, 0.0s simulation');
  });

  test('restarts the quiet window after a malformed encoder reply', async () => {
    const result = await replay((ms) => (ms === 2000 ? 'invalid' : `${ms < 1000 ? 10 : 0} 0`));
    expect(result.elapsedMs).toBe(4000);
  });

  test('rejects drift that fits the short-window range but exceeds the long-window rate', async () => {
    await expect(replay((ms) => `${ms * 0.0008} 0`, {
      toleranceDeg: 1.5,
    })).rejects.toThrow('Timed out waiting for encoder stability');
  });

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
      vibrationWindowMs: 10000,
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
