import { applyForceModeState } from '../../primitives/uncalibrated_actions.mjs';

describe('applyForceModeState', () => {
  test('formats force state command', async () => {
    const sent = [];
    const send = async (line) => {
      sent.push(line);
      return { reply: '' };
    };

    await applyForceModeState(send, {
      motorIds: ['40.0', '41.0', '42.0'],
      modes: ['position', 0.05, 0],
    });

    expect(sent).toEqual([
      'M569.4 P40.0:41.0:42.0 T0.0:0.05:0.0',
    ]);
  });
});
