let waitForKlippyReady;

async function importModule() {
  if (waitForKlippyReady) {
    return;
  }
  ({ waitForKlippyReady } = await import('../../../autocal/control/primitives/klipper_ready_gate.mjs'));
}

describe('Klipper ready gate', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('waits until webhooks reports ready', async () => {
    const states = ['startup', 'startup', 'ready'];
    const requests = [];
    const apiBridge = {
      request: async (method, params) => {
        requests.push({ method, params });
        return {
          status: {
            webhooks: {
              state: states.shift(),
              state_message: states.length > 0 ? 'Printer is starting' : 'Printer is ready',
            },
          },
        };
      },
    };

    const result = await waitForKlippyReady(apiBridge, { timeoutMs: 1000, pollMs: 0 });

    expect(result.status.webhooks.state).toBe('ready');
    expect(requests).toHaveLength(3);
    expect(requests[0].method).toBe('objects/query');
  });
});

