import { returnMotorsToOriginAllAtOnce } from '../../primitives/uncalibrated_actions.mjs';

describe('returnMotorsToOriginAllAtOnce', () => {
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
});
