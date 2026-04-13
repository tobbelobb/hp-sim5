import { runMoveWithWait } from '../../primitives/encoder_utils.mjs';

describe('runMoveWithWait', () => {
  test('skips fallback delay for klipper motion commands that already waited in the bridge', async () => {
    const delays = [];
    const send = async () => ({ hadMotion: true });
    send.firmware = 'klipper';

    await runMoveWithWait(send, 'G1 H2 X10 F600', 10, {
      axes: ['X'],
      delayFn: async (ms) => {
        delays.push(ms);
      },
    });

    expect(delays).toHaveLength(0);
  });

  test('keeps fallback delay for non-klipper bridges', async () => {
    const delays = [];
    const send = async () => ({});
    send.firmware = 'rrf';

    await runMoveWithWait(send, 'G1 X10 F600', 10, {
      axes: ['X'],
      delayFn: async (ms) => {
        delays.push(ms);
      },
    });

    expect(delays).toHaveLength(1);
    expect(delays[0]).toBeCloseTo(110, 6);
  });
});
