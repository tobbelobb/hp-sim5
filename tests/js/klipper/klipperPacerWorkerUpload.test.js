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
  const getPostedDiagnostics = () => getPostedMessages()
    .filter((message) => message?.type === 'diagnostic')
    .map((message) => message.diagnostic);

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

  async function playApiBatches(batches, options = {}) {
    const {
      speedScale = 1,
      clockHz = 16000000,
      advanceMs = 20,
      trapqBatches = [],
    } = options;
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: speedScale } });
    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz } });
    for (const batch of trapqBatches) {
      globalThis.self.onmessage({ data: { type: 'api_motion_trapq_batch', batch } });
    }
    for (const batch of batches) {
      globalThis.self.onmessage({ data: { type: 'api_motion_batch', batch } });
    }
    globalThis.self.onmessage({ data: { type: 'api_motion_finish' } });
    jest.advanceTimersByTime(advanceMs);
    jest.runAllTimers();
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

  test('replays API motion batches through the same paced move emission path', async () => {
    const commands = await playApiBatches([
      {
        name: 'stepper_a',
        first_clock: 0,
        start_mcu_position: 0,
        data: [[60000, 2, 0]],
      },
    ]);

    const postedTypes = globalThis.postMessage.mock.calls.map(([message]) => message?.type);
    expect(postedTypes).toContain('move');
    expect(postedTypes).toContain('done');

    const moves = commands.filter((command) => command?.type === 'Move');
    expect(moves).toHaveLength(2);
    expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(bucketedAntialiasingEnabled ? (5 / 3) : 1, 6);
    expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('starts a new API session from the reported MCU position instead of snapping back to zero', async () => {
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: 1 } });

    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 16_000_000 } });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_clock: 0,
          start_mcu_position: 2,
          data: [[60_000, 2, 0]],
        },
      },
    });
    globalThis.self.onmessage({ data: { type: 'api_motion_finish' } });
    jest.runAllTimers();
    await Promise.resolve();
    await Promise.resolve();

    const firstSessionMoveCount = getPostedCommands().length;
    const firstSessionMoves = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(firstSessionMoves.at(-1).A / STEP_ANGLE_RAD).toBeCloseTo(4, 6);

    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 16_000_000 } });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_clock: 0,
          start_mcu_position: 4,
          data: [[60_000, 2, 0]],
        },
      },
    });
    globalThis.self.onmessage({ data: { type: 'api_motion_finish' } });
    jest.runAllTimers();
    await Promise.resolve();
    await Promise.resolve();

    const secondSessionMoves = getPostedCommands()
      .slice(firstSessionMoveCount)
      .filter((command) => command?.type === 'Move');
    expect(secondSessionMoves).toHaveLength(2);
    expect(secondSessionMoves[0].A / STEP_ANGLE_RAD).toBeCloseTo(4 + (5 / 3), 6);
    expect(secondSessionMoves[1].A / STEP_ANGLE_RAD).toBeCloseTo(6, 6);
  });

  test('rebases API motion timing against toolhead trapq instead of the historical first interval', async () => {
    const commands = await playApiBatches([
      {
        name: 'stepper_a',
        first_time: 10.1,
        last_time: 10.1024,
        start_mcu_position: 0,
        data: [
          [100000000, 1, 0],
          [60000, 1, 0],
        ],
      },
    ], {
      clockHz: 50_000_000,
      trapqBatches: [
        {
          name: 'toolhead',
          data: [
            [10.0, 0.2, 0, 0, [0, 0, 0], [1, 0, 0]],
          ],
        },
      ],
      advanceMs: 200,
    });

    const moves = commands.filter((command) => command?.type === 'Move');
    expect(moves.length).toBeGreaterThan(20);
    expect(moves[0].at).toBeLessThan(300);
    expect(moves.at(-1).at).toBeLessThan(300);
  });

  test('starts API playback timing when buffered motion is released, not when the stream opens', async () => {
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: 1 } });
    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 50_000_000 } });

    jest.advanceTimersByTime(500);
    const releaseMs = performance.now();

    globalThis.self.onmessage({
      data: {
        type: 'api_motion_trapq_batch',
        batch: {
          name: 'toolhead',
          data: [
            [10.0, 0.5, 0, 0, [0, 0, 0], [1, 0, 0]],
          ],
        },
      },
    });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 10.1,
          last_time: 10.2,
          start_mcu_position: 0,
          data: [
            [100_000_000, 1, 0],
            [100_000_000, 1, 0],
          ],
        },
      },
    });
    globalThis.self.onmessage({ data: { type: 'api_motion_finish' } });

    const preEmittedMoves = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(preEmittedMoves.length).toBeGreaterThan(0);
    expect(preEmittedMoves[0].at - releaseMs).toBeGreaterThanOrEqual(0);
    expect(preEmittedMoves.at(-1).at - releaseMs).toBeLessThanOrEqual(30);
    jest.advanceTimersByTime(80);
    const earlyMoves = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(earlyMoves.length).toBeGreaterThan(preEmittedMoves.length);
    expect(earlyMoves.at(-1).at - releaseMs).toBeLessThanOrEqual(110);

    jest.advanceTimersByTime(140);
    const lateMoves = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(lateMoves.length).toBeGreaterThan(earlyMoves.length);
    expect(lateMoves.at(-1).at - releaseMs).toBeGreaterThanOrEqual(200);
  });

  test('starts trapq-less API playback from the earliest stepper timestamp before api_motion_finish', async () => {
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: 1 } });
    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 50_000_000 } });

    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 10.1,
          last_time: 13.0,
          start_mcu_position: 0,
          data: [
            [500_000, 290, 0],
          ],
        },
      },
    });

    jest.advanceTimersByTime(20);
    await Promise.resolve();

    const movesBeforeFinish = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(movesBeforeFinish.length).toBeGreaterThan(0);

    globalThis.self.onmessage({ data: { type: 'api_motion_finish' } });
    jest.runAllTimers();
    await Promise.resolve();
    await Promise.resolve();

    const postedTypes = globalThis.postMessage.mock.calls.map(([message]) => message?.type);
    expect(postedTypes).toContain('done');
  });

  test('does not start API playback until the startup preload includes the tail reserve', async () => {
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: 1 } });
    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 50_000_000, debugDiagnostics: true } });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_trapq_batch',
        batch: {
          name: 'toolhead',
          data: [
            [10.0, 4.0, 0, 0, [0, 0, 0], [1, 0, 0]],
          ],
        },
      },
    });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 10.1,
          last_time: 13.0,
          start_mcu_position: 0,
          data: [
            [500_000, 290, 0],
          ],
        },
      },
    });

    jest.advanceTimersByTime(1000);
    await Promise.resolve();
    expect(getPostedCommands()).toHaveLength(0);

    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 13.0,
          last_time: 13.8,
          start_mcu_position: 290,
          data: [
            [500_000, 80, 0],
          ],
        },
      },
    });

    jest.advanceTimersByTime(20);
    await Promise.resolve();
    const moves = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(moves.length).toBeGreaterThan(0);
  });

  test('does not emit a catch-up burst when a later API batch arrives after the current safe horizon', async () => {
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: 1 } });
    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 50_000_000 } });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_trapq_batch',
        batch: {
          name: 'toolhead',
          data: [
            [10.0, 4.5, 0, 0, [0, 0, 0], [1, 0, 0]],
          ],
        },
      },
    });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 10.1,
          last_time: 13.8,
          start_mcu_position: 0,
          data: [
            [500_000, 370, 0],
          ],
        },
      },
    });

    jest.advanceTimersByTime(700);
    await Promise.resolve();

    const movesBeforeSecondBatch = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(movesBeforeSecondBatch.length).toBeGreaterThan(0);

    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 13.8,
          last_time: 14.3,
          start_mcu_position: 370,
          data: [
            [500_000, 50, 0],
          ],
        },
      },
    });

    const movesImmediatelyAfterSecondBatch = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(movesImmediatelyAfterSecondBatch).toHaveLength(movesBeforeSecondBatch.length);

    jest.advanceTimersByTime(20);
    await Promise.resolve();

    const movesAfterPacingResumes = getPostedCommands().filter((command) => command?.type === 'Move');
    expect(movesAfterPacingResumes.length).toBeGreaterThan(movesImmediatelyAfterSecondBatch.length);
  });

  test('emits API pacing summaries when diagnostics are enabled', async () => {
    await loadWorkerModule();
    globalThis.self.onmessage({ data: { type: 'set_speed_scale', value: 1 } });
    globalThis.self.onmessage({ data: { type: 'api_motion_start', clockHz: 50_000_000, debugDiagnostics: true } });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_trapq_batch',
        batch: {
          name: 'toolhead',
          data: [
            [10.0, 4.0, 0, 0, [0, 0, 0], [1, 0, 0]],
          ],
        },
      },
    });
    globalThis.self.onmessage({
      data: {
        type: 'api_motion_batch',
        batch: {
          name: 'stepper_a',
          first_time: 10.1,
          last_time: 13.8,
          start_mcu_position: 0,
          data: [
            [500_000, 370, 0],
          ],
        },
      },
    });

    jest.advanceTimersByTime(3100);
    await Promise.resolve();

    globalThis.self.onmessage({ data: { type: 'api_motion_finish' } });
    jest.runAllTimers();
    await Promise.resolve();
    await Promise.resolve();

    const summary = getPostedDiagnostics().find((diagnostic) => diagnostic?.event === 'summary');
    expect(summary).toBeTruthy();
    expect(summary.config.emitLookaheadMs).toBeGreaterThan(0);
    expect(summary.stats.timerOversleepCount).toBeGreaterThanOrEqual(0);
  });
});
