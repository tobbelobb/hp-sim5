import { primeEncoders } from '../../primitives/uncalibrated_actions.mjs';

describe('primeEncoders', () => {
  test('sends encoder prime command', async () => {
    const sent = [];
    const send = async (line) => {
      sent.push(line);
      return { reply: '' };
    };

    await primeEncoders(send, {
      motorIds: ['40.0', '41.0'],
    });

    expect(sent).toEqual([
      'M569.3 P40.0:41.0 S',
    ]);
  });
});
