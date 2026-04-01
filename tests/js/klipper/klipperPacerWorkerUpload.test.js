describe('Klipper pacer worker upload playback', () => {
  let originalSelf;
  let originalPostMessage;

  beforeEach(() => {
    jest.resetModules();
    jest.useFakeTimers();
    originalSelf = globalThis.self;
    originalPostMessage = globalThis.postMessage;
    globalThis.self = {};
    globalThis.postMessage = jest.fn();
  });

  afterEach(() => {
    jest.useRealTimers();
    globalThis.self = originalSelf;
    globalThis.postMessage = originalPostMessage;
  });

  test('plays an uploaded Klipper text file and emits done when drained', async () => {
    await import('../../../integrations/klipper/klipperPacerWorker.js');

    expect(typeof globalThis.self.onmessage).toBe('function');

    const fakeFile = {
      name: 'draw_squares.txt',
      text: async () => [
        'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
        'set_next_step_dir oid=0 dir=1',
        'queue_step oid=0 interval=60000 count=2 add=0',
      ].join('\n'),
      arrayBuffer: async () => new ArrayBuffer(0),
    };

    globalThis.self.onmessage({ data: { type: 'filename_upload', filename: fakeFile } });
    await Promise.resolve();
    jest.advanceTimersByTime(10);
    await Promise.resolve();

    const postedTypes = globalThis.postMessage.mock.calls.map(([message]) => message?.type);
    expect(postedTypes).toContain('move');
    expect(postedTypes).toContain('done');
  });
});
