let connectKlipperApiBridgeWithRetry;

async function importModule() {
  if (connectKlipperApiBridgeWithRetry) {
    return;
  }
  ({ connectKlipperApiBridgeWithRetry } = await import('../../../autocal/control/primitives/klipper_api_connect.mjs'));
}

describe('Klipper API connect retry', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('retries ENOENT until the Klippy socket becomes available', async () => {
    const events = [];
    let attempt = 0;
    const bridge = await connectKlipperApiBridgeWithRetry(() => ({
      connect: async () => {
        attempt += 1;
        events.push(`connect:${attempt}`);
        if (attempt < 3) {
          const err = new Error('connect ENOENT /tmp/klippy_uds');
          err.code = 'ENOENT';
          throw err;
        }
      },
      close: () => {
        events.push(`close:${attempt}`);
      },
    }), {
      timeoutMs: 2000,
      retryDelayMs: 0,
    });

    expect(bridge).toBeDefined();
    expect(events).toEqual([
      'connect:1',
      'close:1',
      'connect:2',
      'close:2',
      'connect:3',
    ]);
  });
});

