import { runForceTrial } from '../../behaviors/force_tuning.mjs';

describe('runForceTrial', () => {
  test('returns to origin before releasing a moved pullout to idle force', async () => {
    const commands = [];
    let activeForce = 0;
    let atOrigin = true;
    let nowMs = 0;

    const send = async (line) => {
      commands.push(line);
      if (line.startsWith('M569.4 ')) {
        const forces = line.split(' T')[1].split(':').map((value) => Number(value));
        activeForce = Number.isFinite(forces[0]) ? forces[0] : 0;
      }
      if (line.startsWith('G1 H2 ')) {
        atOrigin = true;
      }
      if (line.startsWith('M569.3 ')) {
        if (!atOrigin || activeForce >= 1.0) {
          atOrigin = false;
          return { reply: '10 5 0' };
        }
        return { reply: '0 0 0' };
      }
      return { reply: '' };
    };

    const result = await runForceTrial(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      activeAnchor: 0,
      fixedAnchor: 2,
      restAnchors: [1],
      idleForce: 0.01,
      testForce: 1.0,
      speedup: 1,
      sampleWindowMs: 1,
      sampleIntervalMs: 1,
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      feed: 1000,
      settleOptions: {
        pollIntervalMs: 1,
        stableWindowMs: 2,
        vibrationWindowMs: 2,
        sleepFn: async (ms) => {
          nowMs += Math.max(1, ms);
        },
        nowFn: () => {
          nowMs += 1;
          return nowMs;
        },
      },
      thresholds: {
        thetaActThr: 0.5,
        thetaResThr: 0.3,
        sumResidualThr: 0.8,
        thetaOtherByAnchor: new Map([[1, 0.5]]),
      },
    });

    const testForceIndex = commands.findIndex((line) => line === 'M569.4 P40.0:41.0:42.0 T1:0.01:0.0');
    const returnMoveIndex = commands.findIndex((line) => line.startsWith('G1 H2 '));
    const idleReleaseIndex = commands.findIndex((line, idx) => (
      idx > testForceIndex && line === 'M569.4 P40.0:41.0:42.0 T0.01:0.01:0.0'
    ));

    expect(result.moved).toBe(true);
    expect(returnMoveIndex).toBeGreaterThan(testForceIndex);
    expect(idleReleaseIndex).toBeGreaterThan(returnMoveIndex);
  });
});
