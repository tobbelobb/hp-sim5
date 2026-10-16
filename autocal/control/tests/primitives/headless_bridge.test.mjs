import { createHeadlessBridge } from '../../primitives/headless_bridge.mjs';
import { sendHpSimSettings } from '../../primitives/encoder_utils.mjs';

test('headless settings return the applied encoder acknowledgement to the collector', async () => {
  const settings = { physicsSubsteps: 1, closedLoopMotorsEnabled: true, lineLayeringEnabled: false };
  const fetch = jest.spyOn(globalThis, 'fetch').mockImplementation(async (url) => ({
    ok: true,
    json: async () => ({ backend: 'headless-js', collector_time_s: 0,
      ...(url.endsWith('/payload') ? { result: { simulationSettings: settings } } : {}),
    }),
  }));
  try {
    const bridge = await createHeadlessBridge('http://local-simulator');
    await expect(sendHpSimSettings(bridge, settings)).resolves.toEqual(settings);
  } finally { fetch.mockRestore(); }
});
