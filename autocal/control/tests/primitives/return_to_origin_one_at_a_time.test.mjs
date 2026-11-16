import { returnMotorsToOriginOneAtATime } from '../../primitives/uncalibrated_actions.mjs';

describe('returnMotorsToOriginOneAtATime', () => {
  test('a zero-distance return leaves every motor held', async () => {
    let nowMs = 0;
    const commands = [];
    const send = async line => { commands.push(line); return { reply: '0 0' }; };
    send.simulationClock = { now: () => nowMs, sleep: async ms => { nowMs += ms; } };
    await returnMotorsToOriginOneAtATime(send, {
      motorIds: ['40.0', '41.0'], axes: ['X', 'Y'], mmPerDeg: [1, 1], feed: 1000,
    });
    expect(commands[0]).toBe('M569.4 P40.0:41.0 T0.0:0.0');
    const modes = commands.filter(line => line.startsWith('M569.4'));
    expect(modes.at(-1)).toBe('M569.4 P40.0:41.0 T0.0:0.0');
    expect(commands.some(line => line.startsWith('G1'))).toBe(false);
  });

  test('returns motors in expected order', async () => {
    const sent = [];
    const angles = [10, 20, 5];
    const send = async (line) => {
      sent.push(line);
      if (line.startsWith('M569.3')) {
        return { reply: angles.join(' ') };
      }
      const move = line.match(/G1 H2 ([ABC])([-\d.]+)/);
      if (move) angles['ABC'.indexOf(move[1])] += Number(move[2]);
      return { reply: '' };
    };
    let nowMs = 0;
    const delayFn = (ms = 0) => {
      nowMs += ms;
      return Promise.resolve();
    };
    const sleepFn = (ms = 0) => {
      nowMs += ms;
      return Promise.resolve();
    };
    const nowFn = () => nowMs;

    await returnMotorsToOriginOneAtATime(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['A', 'B', 'C'],
      mmPerDeg: [1, 1, 1],
      feed: 100,
      speedup: 100,
      midForce: 0.01,
      fixedAnchors: [1],
      delayFn,
      settleOptions: { pollIntervalMs: 1, stableWindowMs: 2, sleepFn, nowFn },
    });

    const moveLines = sent.filter((line) => line.startsWith('G1 H2'));
    const axesOrder = moveLines.map((line) => {
      const match = line.match(/H2\s+([A-Z])/);
      return match ? match[1] : null;
    });

    expect(axesOrder).toEqual(['A', 'C', 'B']);
  });

  test('acknowledged moves with quiet unmoved encoders fail before the next return', async () => {
    const sent = [];
    let now = 0;
    const send = async line => { sent.push(line); return { reply: '100 200' }; };
    send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
    await expect(returnMotorsToOriginOneAtATime(send, {
      motorIds: ['40.0', '41.0'], axes: ['X', 'Y'], mmPerDeg: [1, 1], feed: 1000,
    })).rejects.toThrow('Return on axis Y did not reach its encoder origin');
    expect(sent.filter(line => line.startsWith('G1'))).toHaveLength(1);
    expect(sent.at(-1)).toMatch(/^M569.3/);
  });

  test.each([[1, 0], [1, NaN], [1]])('refuses invalid encoder conversions %s', async (...values) => {
    const send = jest.fn();
    await expect(returnMotorsToOriginOneAtATime(send, {
      motorIds: ['40.0', '41.0'], axes: ['X', 'Y'], mmPerDeg: values, feed: 1000,
    })).rejects.toThrow('positive finite encoder conversions');
    expect(send).not.toHaveBeenCalled();
  });

  test('waits for the final return move to settle before reporting lengths', async () => {
    let moved = false;
    let postMoveReads = 0;
    const send = async (line) => {
      if (line.startsWith('G1 H2')) {
        moved = true;
        return { reply: '' };
      }
      if (line.startsWith('M569.3')) {
        if (!moved) {
          return { reply: '10' };
        }
        postMoveReads += 1;
        if (postMoveReads === 1) {
          return { reply: '3' };
        }
        return { reply: '0' };
      }
      return { reply: '' };
    };
    let nowMs = 0;
    const delayFn = (ms = 0) => {
      nowMs += ms;
      return Promise.resolve();
    };
    const sleepFn = (ms = 0) => {
      nowMs += ms;
      return Promise.resolve();
    };
    const nowFn = () => nowMs;

    const lengths = await returnMotorsToOriginOneAtATime(send, {
      motorIds: ['40.0'],
      axes: ['A'],
      mmPerDeg: [1],
      feed: 100,
      speedup: 100,
      midForce: 0.01,
      delayFn,
      settleOptions: {
        pollIntervalMs: 1,
        stableWindowMs: 2,
        toleranceDeg: 0.5,
        sleepFn,
        nowFn,
      },
    });

    expect(lengths).toEqual([0]);
    expect(postMoveReads).toBeGreaterThan(1);
  });
});

// A shortened carrying line must reach home before low axes are held at home.
test('returns carrying anchors first while keeping them held for the remaining returns', async () => {
  const angles = [-10, 20, -30, 40, -5], moves = [], modes = [];
  let now = 0;
  const send = async line => {
    const move = line.match(/G1 H2 ([XYZUV])([-\d.]+)/);
    if (move) {
      const index = 'XYZUV'.indexOf(move[1]);
      angles[index] += Number(move[2]);
      moves.push(index);
    }
    if (line.startsWith('M569.4')) modes.push(line);
    return { reply: angles.join(' ') };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  await returnMotorsToOriginOneAtATime(send, {
    motorIds: ['40.0','41.0','42.0','43.0','44.0'], axes: [...'XYZUV'],
    mmPerDeg: [1,1,1,1,1], feed: 1000, midForce: .02,
    forbiddenForceAnchors: [4], returnFirstAnchors: [4],
  });
  expect(moves).toEqual([4,3,2,1,0]);
  expect(modes.every(line => line.endsWith(':0.0'))).toBe(true);
});
