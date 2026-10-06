import { waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';
import { sampleEncoderNoise } from '../../primitives/encoder_noise.mjs';
import { runMoveWithWait, parseM669 } from '../../primitives/encoder_utils.mjs';

test('all three reported RRF anchor coordinates survive collection', () => {
  expect(parseM669('Kinematics is Hangprinter\nA:0.00, -1900.00, -280.00\nB:1645.45, 950.00, -280.00\n')).toMatchObject({
    A: [0, -1900, -280], B: [1645.45, 950, -280],
  });
});

test('the collector clock advances motion, settling and noise sampling without wall sleeps', async () => {
  let nowMs = 0;
  const delays = [];
  const send = async () => ({ reply: '1 2', motion: [] });
  send.simulationClock = {
    now: () => nowMs,
    sleep: async ms => { delays.push(ms); nowMs += ms; },
    settlingTimeoutMs: 30000,
  };
  await runMoveWithWait(send, 'G1 H2 X1 F60', 1);
  expect(delays).toEqual([1100]);
  const settled = await waitForStableEncoders(send, ['40.0', '41.0'], 1);
  expect(settled.elapsedMs).toBe(1500);
  const noise = await sampleEncoderNoise(send, ['40.0', '41.0'], { sampleCount: 4 });
  expect(noise.durationMs).toBe(100);
  expect(noise.samplingHz).toBe(40);
  expect(noise.muByMotorDeg).toEqual([1, 2]);
});

test('a drifting native encoder fails at the bounded simulation-time deadline', async () => {
  let nowMs = 0;
  const send = async () => ({ reply: String(nowMs / 10) });
  send.simulationClock = {
    now: () => nowMs,
    sleep: async ms => { nowMs += ms; },
    settlingTimeoutMs: 2000,
  };
  await expect(waitForStableEncoders(send, ['40.0'], 1)).rejects.toThrow('Timed out waiting for encoder stability');
  expect(nowMs).toBe(2500);
});
