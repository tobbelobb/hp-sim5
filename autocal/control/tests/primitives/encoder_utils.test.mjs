import { parseEncoderReply, parseM92, runMoveWithWait, sendHpSimSettings } from '../../primitives/encoder_utils.mjs';

test('parses RRF steps per mm for every axis', () => {
  expect(parseM92('Steps/mm: X: 16.977, Y: 16.977, Z: 16.977, U: 16.977, E: 95.922'))
    .toEqual({ X: 16.977, Y: 16.977, Z: 16.977, U: 16.977, E: 95.922 });
});

test('scientific encoder values near zero remain one value per motor', () => {
  expect(parseEncoderReply('0 -5.684341886080802e-14 3.1E+2 +.25')).toEqual([0, -5.684341886080802e-14, 310, .25]);
});

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
    expect(delays[0]).toBeCloseTo(1100, 6);
  });
});

test('simulator configuration is acknowledged before collection proceeds', async () => {
  const settings = { closedLoopMotorsEnabled: true, lineLayeringEnabled: true, physicsSubsteps: 4, solverIterations: 8 };
  const bridge = {
    broadcast: jest.fn(),
    sendEncoderRequest: jest.fn(async () => ({ simulationSettings: settings })),
  };
  await expect(sendHpSimSettings(bridge, settings)).resolves.toEqual(settings);
  expect(bridge.broadcast).toHaveBeenCalledWith({ type: 'simulation_settings', ...settings });
  bridge.sendEncoderRequest.mockResolvedValue({});
  await expect(sendHpSimSettings(bridge, settings)).rejects.toThrow('reload the simulator page');
  bridge.sendEncoderRequest.mockResolvedValue({ simulationSettings: { closedLoopMotorsEnabled: false } });
  await expect(sendHpSimSettings(bridge, settings)).rejects.toThrow('did not apply');
});

 test('rejects an unacknowledged physics substep count', async () => {
  const bridge = { broadcast: jest.fn(), sendEncoderRequest: jest.fn(async () => ({
    simulationSettings: { physicsSubsteps: 1 },
  })) };
  await expect(sendHpSimSettings(bridge, { physicsSubsteps: 4 })).rejects.toThrow('physicsSubsteps=4');
});
