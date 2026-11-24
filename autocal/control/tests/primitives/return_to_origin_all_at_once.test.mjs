import { returnMotorsToOrigin, returnMotorsToOriginAllAtOnce } from '../../primitives/uncalibrated_actions.mjs';

describe('returnMotorsToOriginAllAtOnce', () => {
  test('holds ongoing force motion before the first encoder read', async () => {
    let nowMs = 0, held = false;
    const commands = [];
    const send = async line => {
      commands.push(line);
      if (line === 'M569.4 P40.0:41.0 T0.0:0.0') held = true;
      return { reply: held ? '10 20' : `${nowMs} ${nowMs}` };
    };
    send.simulationClock = { now: () => nowMs, sleep: async ms => { nowMs += ms; } };
    await returnMotorsToOriginAllAtOnce(send, {
      motorIds: ['40.0', '41.0'], axes: ['X', 'Y'], mmPerDeg: [1, 1], feed: 1000,
    });
    expect(commands[0]).toBe('M569.4 P40.0:41.0 T0.0:0.0');
    expect(commands.some(line => line.startsWith('G1 H2'))).toBe(true);
  });

  test('returns all motors to origin in a single move', async () => {
    const sent = [];
    const encoderReply = '10 20 5';
    const send = async (line) => {
      sent.push(line);
      if (line.startsWith('M569.3')) {
        return { reply: encoderReply };
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

    await returnMotorsToOriginAllAtOnce(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      feed: 100,
      speedup: 100,
      delayFn,
      settleOptions: { pollIntervalMs: 1, stableWindowMs: 2, sleepFn, nowFn },
    });

    const gcode = sent.filter((line) => line.startsWith('G1 H2'));
    expect(gcode).toHaveLength(1);
    expect(gcode[0]).toContain('X-10.000');
    expect(gcode[0]).toContain('Y-20.000');
    expect(gcode[0]).toContain('Z-5.000');
  });

  test('waits for encoders to settle after the return move before reporting lengths', async () => {
    const sent = [];
    let moved = false;
    let postMoveReads = 0;
    const send = async (line) => {
      sent.push(line);
      if (line.startsWith('G1 H2')) {
        moved = true;
        return { reply: '' };
      }
      if (line.startsWith('M569.3')) {
        if (!moved) {
          return { reply: '10 20 5' };
        }
        postMoveReads += 1;
        if (postMoveReads === 1) {
          return { reply: '3 3 3' };
        }
        return { reply: '0 0 0' };
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

    const lengths = await returnMotorsToOriginAllAtOnce(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      feed: 100,
      speedup: 100,
      delayFn,
      settleOptions: {
        pollIntervalMs: 1,
        stableWindowMs: 2,
        toleranceDeg: 0.5,
        sleepFn,
        nowFn,
      },
    });

    expect(lengths).toEqual([0, 0, 0]);
    expect(postMoveReads).toBeGreaterThan(1);
  });

  test('normalizes multi-axis feed so the fastest motor runs at requested feed', async () => {
    const sent = [];
    const angles = [30, 40];
    let nowMs = 0;
    const send = async (line) => {
      sent.push(line);
      if (line.startsWith('G1 H2')) {
        angles[0] = 0;
        angles[1] = 0;
      }
      return { reply: angles.join(' ') };
    };
    send.simulationClock = { now: () => nowMs, sleep: async (ms) => { nowMs += ms; } };

    await returnMotorsToOriginAllAtOnce(send, {
      motorIds: ['40.0', '41.0'],
      axes: ['X', 'Y'],
      mmPerDeg: [1, 1],
      feed: 1000,
      speedup: 1,
    });
    const move = sent.find((line) => line.startsWith('G1 H2'));
    // Euclidean norm = 50, max axis delta = 40 -> effective feed = 1000 * 50 / 40 = 1250.
    expect(move).toBe('G1 H2 X-30.000 Y-40.000 F1250');
  });
});

describe('returnMotorsToOrigin', () => {
  test('completes in one coordinated move on RRF and corrects stale firmware positions', async () => {
    const sent = [];
    const angles = [0, -100, 120];
    const commands = [0, -110, 120]; // motor 1 has a 10 mm stale offset from saturation
    let nowMs = 0;
    const send = async (line) => {
      sent.push(line);
      if (line.startsWith('G1 H2')) {
        for (const match of line.matchAll(/([XYZ])(-?[\d.]+)/g)) {
          const idx = 'XYZ'.indexOf(match[1]);
          commands[idx] += Number(match[2]);
          angles[idx] = commands[idx];
        }
      }
      return { reply: angles.join(' ') };
    };
    send.firmware = 'rrf';
    send.simulationClock = { now: () => nowMs, sleep: async (ms) => { nowMs += ms; } };

    const lengths = await returnMotorsToOrigin(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      axes: ['X', 'Y', 'Z'],
      mmPerDeg: [1, 1, 1],
      feed: 1400,
    });
    expect(lengths).toEqual([0, 0, 0]);
    // First coordinated move lands at [0, -10, 0], second coordinated move corrects Y to 0.
    expect(sent.filter((line) => line.startsWith('G1 H2'))).toHaveLength(2);
  });

  test('routes Klipper returns to one-at-a-time sequential moves', async () => {
    const sent = [];
    const angles = [10, 20];
    let nowMs = 0;
    const send = async (line) => {
      sent.push(line);
      if (line.startsWith('G1 H2')) {
        for (const match of line.matchAll(/([XY])(-?[\d.]+)/g)) {
          angles['XY'.indexOf(match[1])] += Number(match[2]);
        }
      }
      return { reply: angles.join(' ') };
    };
    send.firmware = 'klipper';
    send.simulationClock = { now: () => nowMs, sleep: async (ms) => { nowMs += ms; } };

    await returnMotorsToOrigin(send, {
      motorIds: ['40.0', '41.0'],
      axes: ['X', 'Y'],
      mmPerDeg: [1, 1],
      feed: 1400,
    });
    const moves = sent.filter((line) => line.startsWith('G1 H2'));
    expect(moves).toHaveLength(2);
    expect(moves.every((line) => (line.match(/[XY]-?[\d.]+/g) || []).length === 1)).toBe(true);
  });
});
