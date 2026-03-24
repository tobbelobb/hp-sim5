let createSequentialLineQueue;

async function importModule() {
  if (createSequentialLineQueue) {
    return;
  }
  ({ createSequentialLineQueue } = await import('../../../autocal/control/primitives/klipper_stdin_queue.mjs'));
}

describe('Klipper stdin queue', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('prompts immediately when a line is enqueued', async () => {
    const prompts = [];
    const seen = [];
    let releaseFirst;
    const first = new Promise((resolve) => {
      releaseFirst = resolve;
    });

    const queue = createSequentialLineQueue({
      onLine: async (line) => {
        seen.push(line);
        if (line === 'G1 X100') {
          await first;
        }
      },
      onPrompt: () => {
        prompts.push(seen.length);
      },
    });

    queue.enqueue('G1 X100');
    queue.enqueue('G1 Y-200');

    expect(prompts).toEqual([0, 1]);
    expect(seen).toEqual(['G1 X100']);

    releaseFirst();
    await new Promise((resolve) => setImmediate(resolve));
    await Promise.resolve();

    expect(seen).toEqual(['G1 X100', 'G1 Y-200']);
  });
});
