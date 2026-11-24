import {
  findBreakawayForce,
  runStaircaseForceStep,
  tuneForce,
} from '../../behaviors/force_tuning.mjs';

function makeMockStaircaseMachine({ breakawayN = 0.5, edgeN = 3.0, slipAtN = Infinity, mmPerDeg = 0.5 } = {}) {
  let nowMs = 0;
  let activeForce = 0;
  let angles = [0, 0, 0, 0];
  const commands = [];
  const targetTravelFor = (force) => {
    if (force < breakawayN) return 0;
    const x = force - breakawayN;
    const scale = Math.max(0.2, edgeN - breakawayN);
    // Smooth saturating response curve whose compliance drops past edgeN.
    return 1500 * (x / (x + scale));
  };
  const send = async (line) => {
    commands.push({ line, nowMs });
    if (line.startsWith('M569.4')) {
      const forces = line.split(' T')[1].split(':').map(Number);
      activeForce = forces[1] ?? 0;
    } else if (line.startsWith('G1 H2')) {
      for (const match of line.matchAll(/([XYZU])(-?[\d.]+)/g)) {
        angles['XYZU'.indexOf(match[1])] += Number(match[2]) / mmPerDeg;
      }
    }
    return { reply: angles.join(' ') };
  };
  send.firmware = 'rrf';
  send.simulationClock = {
    now: () => nowMs,
    sleep: async (ms) => {
      nowMs += ms;
      if (activeForce >= slipAtN) {
        angles[3] = 2.5;
        return;
      }
      const target = targetTravelFor(activeForce);
      const current = Math.abs(angles[1]);
      if (target > current) {
        const step = Math.min(target - current, (ms / 1000) * 250);
        angles[1] -= step;
        angles[2] += step * 0.9;
      }
    },
  };
  return { send, commands, now: () => nowMs, angles: () => angles.slice() };
}

describe('staircase force tuning', () => {
  test('findBreakawayForce steps monotonically without returning to origin and stops on slip', async () => {
    const machine = makeMockStaircaseMachine({ breakawayN: 0.4 });
    const res = await findBreakawayForce(machine.send, {
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      activeAnchor: 1,
      fixedAnchor: 3,
      restAnchors: [0, 2],
      idleForce: 0.01,
      startForce: 0.01,
      capForceLimit: 20,
      originAngles: [0, 0, 0, 0],
    });
    expect(res.forceStart).toBeGreaterThanOrEqual(0.4);
    expect(res.forceStart).toBeLessThanOrEqual(0.4 * 1.4 + 1e-6);
    expect(machine.commands.some((c) => c.line.startsWith('G1'))).toBe(false);
    // All sub-stiction steps together should take well under 30 simulated seconds.
    expect(machine.now()).toBeLessThan(30000);

    const slipping = makeMockStaircaseMachine({ breakawayN: 5.0, slipAtN: 1.0 });
    const slipRes = await findBreakawayForce(slipping.send, {
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      activeAnchor: 1,
      fixedAnchor: 3,
      restAnchors: [0, 2],
      idleForce: 0.01,
      startForce: 0.5,
      capForceLimit: 20,
      originAngles: [0, 0, 0, 0],
    });
    expect(Math.abs(slipRes.fixedDriftDeg)).toBeGreaterThan(1.5);
    expect(slipRes.fixedSlipped).toBe(true);
    expect(slipRes.forceStart).toBeNull();
    const lastForceCmd = slipping.commands.filter((c) => c.line.startsWith('M569.4')).at(-1);
    expect(lastForceCmd?.line).toBe('M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.0:0.0');
  });

  test('runStaircaseForceStep exits early once drive stalls and stops torque on slip', async () => {
    const machine = makeMockStaircaseMachine({ breakawayN: 0.4, edgeN: 2.0 });
    const res = await runStaircaseForceStep(machine.send, {
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      activeAnchor: 1,
      fixedAnchor: 3,
      restAnchors: [0, 2],
      idleForce: 0.02,
      testForce: 1.0,
      originAngles: [0, 0, 0, 0],
      maxStepMs: 15000,
    });
    expect(res.moved).toBe(true);
    expect(res.stalled).toBe(true);
    expect(res.fixedSlipped).toBe(false);
    expect(res.travelDeg).toBeGreaterThan(200);
    expect(machine.now()).toBeLessThan(10000);

    const slipping = makeMockStaircaseMachine({ breakawayN: 0.4, slipAtN: 1.5 });
    const slipRes = await runStaircaseForceStep(slipping.send, {
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      activeAnchor: 1,
      fixedAnchor: 3,
      restAnchors: [0, 2],
      idleForce: 0.02,
      testForce: 2.0,
      originAngles: [0, 0, 0, 0],
      maxStepMs: 15000,
    });
    expect(slipRes.fixedSlipped).toBe(true);
    const lastForceCmd = slipping.commands.filter((c) => c.line.startsWith('M569.4')).at(-1);
    expect(lastForceCmd?.line).toBe('M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.0:0.0');
  });

  test('tuneForce treats breakaway slip as a distinct failure without entering edge ramp', async () => {
    const machine = makeMockStaircaseMachine({ breakawayN: 5.0, slipAtN: 0.5 });
    const tuned = await tuneForce(machine.send, {
      config: { fixedAnchors: [0, 3], driveAnchor: 1, sensorAnchor: 2 },
    }, {
      axes: ['X', 'Y', 'Z', 'U'],
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      mmPerDeg: [0.5, 0.5, 0.5, 0.5],
      feed: 1400,
      speedup: 1,
      forbiddenForceAnchors: [3],
    });
    expect(tuned.tuningMeta.tuning_failed).toBe(true);
    expect(tuned.tuningMeta.fixed_slipped).toBe(true);
    const lastForceCmd = machine.commands.filter((c) => c.line.startsWith('M569.4')).at(-1);
    expect(lastForceCmd?.line).toBe('M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.0:0.0');
  });

  test('tuneForce stops force mode and preserves the original error when an exception occurs', async () => {
    const machine = makeMockStaircaseMachine({ breakawayN: 0.35, edgeN: 2.5 });
    let activeApplied = false;
    const flakySend = async (line) => {
      if (line.startsWith('M569.4')) {
        const forces = line.split(' T')[1].split(':').map(Number);
        if ((forces[1] ?? 0) >= 0.5) activeApplied = true;
      }
      if (activeApplied && line.startsWith('M569.3')) {
        activeApplied = false;
        throw new Error('Encoder transport timeout');
      }
      return machine.send(line);
    };
    flakySend.firmware = 'rrf';
    flakySend.simulationClock = machine.send.simulationClock;

    await expect(tuneForce(flakySend, {
      config: { fixedAnchors: [0, 3], driveAnchor: 1, sensorAnchor: 2 },
    }, {
      axes: ['X', 'Y', 'Z', 'U'],
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      mmPerDeg: [0.5, 0.5, 0.5, 0.5],
      feed: 1400,
      speedup: 1,
      forbiddenForceAnchors: [3],
    })).rejects.toThrow('Encoder transport timeout');
    const lastForceCmd = machine.commands.filter((c) => c.line.startsWith('M569.4')).at(-1);
    expect(lastForceCmd?.line).toBe('M569.4 P40.0:41.0:42.0:43.0 T0.0:0.0:0.0:0.0');

    // Original error is preserved even if cleanup itself fails on a dead link.
    let broken = false;
    const deadLinkSend = async (line) => {
      if (broken) throw new Error('Cleanup transport dead');
      if (line.startsWith('M569.4')) {
        const forces = line.split(' T')[1].split(':').map(Number);
        if ((forces[1] ?? 0) >= 0.4) {
          broken = true;
          throw new Error('Original staircase failure');
        }
      }
      return machine.send(line);
    };
    deadLinkSend.firmware = 'rrf';
    deadLinkSend.simulationClock = machine.send.simulationClock;

    await expect(tuneForce(deadLinkSend, {
      config: { fixedAnchors: [0, 3], driveAnchor: 1, sensorAnchor: 2 },
    }, {
      axes: ['X', 'Y', 'Z', 'U'],
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      mmPerDeg: [0.5, 0.5, 0.5, 0.5],
      feed: 1400,
      speedup: 1,
      forbiddenForceAnchors: [3],
    })).rejects.toThrow('Original staircase failure');
  });

  test('tuneForce finds low/mid/max in one staircase with only one return move at the end', async () => {
    const machine = makeMockStaircaseMachine({ breakawayN: 0.35, edgeN: 2.5 });
    const tuned = await tuneForce(machine.send, {
      config: { fixedAnchors: [0, 3], driveAnchor: 1, sensorAnchor: 2 },
    }, {
      axes: ['X', 'Y', 'Z', 'U'],
      motorIds: ['40.0', '41.0', '42.0', '43.0'],
      mmPerDeg: [0.5, 0.5, 0.5, 0.5],
      feed: 1400,
      speedup: 1,
      forbiddenForceAnchors: [3],
    });
    expect(tuned.forceMid).toBeGreaterThanOrEqual(0.35);
    expect(tuned.forceMid).toBeLessThan(0.6);
    expect(tuned.forceMax).toBeGreaterThan(tuned.forceMid);
    expect(tuned.forceMax).toBeLessThan(6.0);
    expect(tuned.tuningMeta.tuning_failed).toBeUndefined();
    // Exactly one G1 return move at the very end of tuning (already at origin on start).
    const g1Moves = machine.commands.filter((c) => c.line.startsWith('G1 H2'));
    expect(g1Moves).toHaveLength(1);
    expect(machine.angles().every((a) => Math.abs(a) < 0.01)).toBe(true);
    expect(machine.now()).toBeLessThan(120000);
  });
});
