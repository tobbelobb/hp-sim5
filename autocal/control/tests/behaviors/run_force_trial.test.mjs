import { runForceTrial } from '../../behaviors/force_tuning.mjs';
import { projectSimulationWallDeadline as projectForceTrialWallDeadline } from '../../primitives/simulation_deadline.mjs';

describe('projectForceTrialWallDeadline', () => {
  const base = { wallDeadlineMs: 120000, hardWallDeadlineMs: 600000, remainingClockMs: 25000 };

  test('extends the budget by the measured wall time per simulated second', () => {
    // The clock takes 2400 ms of wall time to advance 500 ms, so the remaining
    // 25 s of simulated window needs 120 s more than the 5 s safety margin.
    expect(projectForceTrialWallDeadline({
      ...base, clockAdvancedMs: 500, wallElapsedMs: 2400, nowMs: 100000,
    })).toBe(225000);
  });

  test('never shortens an existing deadline or exceeds the hard cap', () => {
    expect(projectForceTrialWallDeadline({
      ...base, clockAdvancedMs: 500, wallElapsedMs: 10, nowMs: 100000,
    })).toBe(120000);
    expect(projectForceTrialWallDeadline({
      ...base, clockAdvancedMs: 1, wallElapsedMs: 100000, nowMs: 100000,
    })).toBe(600000);
  });

  test('leaves the deadline unchanged when the clock does not advance', () => {
    for (const clockAdvancedMs of [0, -5, Number.NaN]) {
      expect(projectForceTrialWallDeadline({
        ...base, clockAdvancedMs, wallElapsedMs: 2400, nowMs: 100000,
      })).toBe(120000);
    }
  });

  test('leaves the deadline unchanged when no simulated budget remains', () => {
    expect(projectForceTrialWallDeadline({
      ...base, remainingClockMs: 0, clockAdvancedMs: 500, wallElapsedMs: 2400, nowMs: 100000,
    })).toBe(120000);
  });
});

describe('runForceTrial', () => {
  test('ends the probe and holds before returning when a fixed motor slips', async () => {
    let time = 0;
    let loaded = false;
    const angles = [0, 0, 0];
    const commands = [];
    const send = async line => {
      commands.push(line);
      if (line.startsWith('M569.4')) {
        loaded = line.includes(' T1:');
      }
      if (line.startsWith('G1 H2')) {
        for (const match of line.matchAll(/([XYZ])(-?[\d.]+)/g)) {
          angles[['X', 'Y', 'Z'].indexOf(match[1])] += Number(match[2]);
        }
      }
      return { reply: angles.join(' ') };
    };
    send.simulationClock = { now: () => time, sleep: async ms => {
      time += ms;
      if (loaded) angles.splice(0, 3, 10, 5, 2);
    } };
    const result = await runForceTrial(send, {
      motorIds: ['A', 'B', 'C'], activeAnchor: 0, fixedAnchor: 2, restAnchors: [1],
      testForce: 1, sampleWindowMs: 30000, axes: ['X', 'Y', 'Z'], mmPerDeg: [1, 1, 1],
    });
    expect(result.fixedDriftDeg).toBe(2);
    expect(time).toBeLessThan(30000);
    const testIndex = commands.findIndex(line => line.includes(' T1:'));
    const holdIndex = commands.findIndex((line, idx) => idx > testIndex && line.endsWith('T0.0:0.0:0.0'));
    const returnIndex = commands.findIndex(line => line.startsWith('G1 H2'));
    expect(holdIndex).toBeGreaterThan(testIndex);
    expect(returnIndex).toBeGreaterThan(holdIndex);
  });

  test('a continuously moving trial is held before return settling, without releasing it to idle first', async () => {
    let nowMs = 0;
    const angles = [0, 0, 0];
    let modes = [0, 0, 0];
    const commands = [];
    const send = async line => {
      commands.push(line);
      if (line.startsWith('M569.4')) modes = line.split(' T')[1].split(':').map(Number);
      if (line.startsWith('G1 H2')) {
        const axes = ['X', 'Y', 'Z'];
        for (const match of line.matchAll(/([XYZ])(-?[\d.]+)/g)) angles[axes.indexOf(match[1])] += Number(match[2]);
      }
      return { reply: angles.join(' ') };
    };
    send.simulationClock = {
      now: () => nowMs,
      sleep: async ms => {
        // Both observed motors keep moving for as long as the pullout force is applied.
        if (modes[0] > .02) { angles[0] += ms / 500; angles[1] += ms / 1000; }
        nowMs += ms;
      },
    };
    const result = await runForceTrial(send, {
      motorIds: ['40.0', '41.0', '42.0'], activeAnchor: 0, fixedAnchor: 2, restAnchors: [1],
      idleForce: .011363575731199994, testForce: .6236330361282555, speedup: 25,
      sampleWindowMs: 1000, stallTimeoutMs: 30000, waitForStall: true,
      axes: ['X', 'Y', 'Z'], mmPerDeg: [1, 1, 1], feed: 1000,
    });
    expect(result.moved).toBe(true);
    expect(result.stalled).toBe(false);
    const trial = commands.findIndex(line => line.includes('T0.6236330361282555:'));
    const hold = commands.findIndex((line, index) => index > trial && line.endsWith('T0.0:0.0:0.0'));
    const move = commands.findIndex(line => line.startsWith('G1 H2'));
    const idle = commands.findIndex((line, index) => index > trial && line.includes('T0.011363575731199994:0.011363575731199994:'));
    expect(hold).toBeGreaterThan(trial);
    expect(move).toBeGreaterThan(hold);
    expect(idle).toBeGreaterThan(move);
    expect(angles).toEqual([0, 0, 0]);
  });

  test('requested 25x never shortens the observed force trial window', async () => {
    let simulationMs = 0;
    const forceChanges = [];
    const send = async line => {
      if (line.startsWith('M569.4')) forceChanges.push(simulationMs);
      return { reply: '0 0' };
    };
    send.simulationClock = { now: () => simulationMs, sleep: async ms => { simulationMs += ms; } };
    await runForceTrial(send, {
      motorIds: ['A', 'B'], activeAnchor: 0, fixedAnchor: 1,
      speedup: 25, idleForce: .01, testForce: 1,
      sampleWindowMs: 1000, sampleIntervalMs: 100,
    });
    expect(forceChanges).toEqual([0, 1000, 2000]);
  });

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
