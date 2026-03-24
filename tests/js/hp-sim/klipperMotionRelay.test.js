let createKlipperMotionRelay;

async function importModule() {
  if (createKlipperMotionRelay) {
    return;
  }
  ({ createKlipperMotionRelay } = await import('../../../autocal/control/primitives/klipper_motion_relay.mjs'));
}

async function flushMicrotasks() {
  await new Promise((resolve) => setImmediate(resolve));
  await Promise.resolve();
}

describe('Klipper motion relay', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('translates klipper_parsed step lines into Move commands', async () => {
    const emitted = [];
    const relay = await createKlipperMotionRelay({
      onCommand: (command) => {
        emitted.push({ ...command });
      },
      asapMode: true,
    });

    relay.feedParsedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
    ]);

    relay.close();
    await relay.runPromise;
    await flushMicrotasks();

    expect(emitted.length).toBeGreaterThan(0);
    expect(emitted[0].type).toBe('Move');
    expect(emitted[0].A).toBeCloseTo((2 * Math.PI) / (200 * 16), 12);
  });
});
