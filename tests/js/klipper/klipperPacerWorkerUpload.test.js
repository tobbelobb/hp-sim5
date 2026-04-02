const fs = require('node:fs');
const path = require('node:path');

const workerSource = fs.readFileSync(
  path.resolve(__dirname, '../../../integrations/klipper/klipperPacerWorker.js'),
  'utf8',
);
const bucketedAntialiasingEnabled = /const ENABLE_BUCKETED_ANTIALIASING = true;/.test(workerSource);

describe('Klipper pacer worker upload playback', () => {
  let originalSelf;
  let originalPostMessage;

  const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);

  const getPostedMessages = () => globalThis.postMessage.mock.calls.map(([message]) => message);

  const getPostedCommands = () => getPostedMessages()
    .filter((message) => message?.type === 'move')
    .map((message) => message.command);

  async function loadWorkerModule() {
    await import('../../../integrations/klipper/klipperPacerWorker.js');
    expect(typeof globalThis.self.onmessage).toBe('function');
  }

  async function playUploadedLines(lines, options = {}) {
    const {
      speedScale = 1,
    } = options;
    await loadWorkerModule();

    const fakeFile = {
      name: 'draw_squares.txt',
      text: async () => lines.join('\n'),
      arrayBuffer: async () => new ArrayBuffer(0),
    };

    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: speedScale } });
    globalThis.self.onmessage({ data: { type: 'filename_upload', filename: fakeFile } });
    await Promise.resolve();
    await Promise.resolve();
    return getPostedCommands();
  }

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
    await playUploadedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
    ]);
    const postedTypes = globalThis.postMessage.mock.calls.map(([message]) => message?.type);
    expect(postedTypes).toContain('move');
    expect(postedTypes).toContain('done');
  });

  test('spreads uploaded queue_step timing across the 2ms bucket window', async () => {
    const commands = await playUploadedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
    ]);

    const moves = commands.filter((command) => command?.type === 'Move');
    expect(moves).toHaveLength(2);
    expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(bucketedAntialiasingEnabled ? (5 / 3) : 1, 6);
    expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('holds the tail bucket open across a direction change', async () => {
    const commands = await playUploadedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
      'set_next_step_dir oid=0 dir=0',
      'queue_step oid=0 interval=60000 count=2 add=0',
    ]);

    const moves = commands.filter((command) => command?.type === 'Move');
    if (bucketedAntialiasingEnabled) {
      expect(moves).toHaveLength(3);
      expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(5 / 3, 6);
      expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(2 / 3, 6);
      expect(moves[2].A / STEP_ANGLE_RAD).toBeCloseTo(0, 6);
      return;
    }
    expect(moves).toHaveLength(2);
    expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(1, 6);
    expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(0, 6);
  });

  test('keeps a set_position bucket open until the following queue_step arrives', async () => {
    const commands = await playUploadedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_position oid=0 pos=2',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
    ]);

    expect(commands.map((command) => command?.type)).toEqual([
      'Add to reference',
      'Move',
      'Move',
    ]);
    expect(commands[0].at).toBe(commands[1].at);
    expect(commands[1].A / STEP_ANGLE_RAD).toBeCloseTo(bucketedAntialiasingEnabled ? (5 / 3) : 1, 6);
    expect(commands[2].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('emits identical Move sequences across speed scaling', async () => {
    const lines = [
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
      'queue_step oid=0 interval=50000 count=2 add=0',
    ];
    const baseCommands = await playUploadedLines(lines, {
      speedScale: 1,
    });
    jest.resetModules();
    jest.clearAllTimers();
    globalThis.self = {};
    globalThis.postMessage = jest.fn();
    const fastCommands = await playUploadedLines(lines, {
      speedScale: 2,
    });

    const normalize = (command) => {
      const { at, span, ...rest } = command || {};
      return rest;
    };
    const baseMoves = baseCommands.filter((command) => command?.type === 'Move');
    const fastMoves = fastCommands.filter((command) => command?.type === 'Move');

    expect(fastCommands.map(normalize)).toEqual(baseCommands.map(normalize));
    expect(fastMoves[0].at).toBeLessThan(baseMoves[0].at);
  });

  test('upload pacing uses relative time instead of worker uptime', async () => {
    jest.advanceTimersByTime(200);
    const startMs = performance.now();

    const commands = await playUploadedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
      'queue_step oid=0 interval=60000 count=2 add=0',
    ]);

    const moves = commands.filter((command) => command?.type === 'Move');
    expect(moves.length).toBeGreaterThan(0);
    expect(moves[0].at - startMs).toBeLessThanOrEqual(10);
  });
});
