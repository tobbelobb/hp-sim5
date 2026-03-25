let connectKlipperRaw;
let summarizeKlipperCommand;
let summarizeKlipperBridgeMessage;

class FakeWorker {
  static instances = [];

  constructor(url, options) {
    this.url = url;
    this.options = options;
    this.messages = [];
    this.onmessage = null;
    this.onerror = null;
    this.onmessageerror = null;
    FakeWorker.instances.push(this);
  }

  postMessage(message) {
    this.messages.push(message);
  }

  terminate() {
    this.terminated = true;
  }
}

async function importModules() {
  if (connectKlipperRaw) {
    return;
  }
  ({
    connectKlipperRaw,
  } = await import('../../../examples/js/slideprinter/klipperHandler.js'));
  ({
    summarizeKlipperCommand,
    summarizeKlipperBridgeMessage,
  } = await import('../../../autocal/control/primitives/klipper_raw_observability.mjs'));
}

describe('Klipper raw observability', () => {
  let originalWorker;
  let performanceSpy;

  beforeAll(async () => {
    await importModules();
  });

  beforeEach(() => {
    originalWorker = global.Worker;
    FakeWorker.instances = [];
    global.Worker = FakeWorker;
    performanceSpy = jest.spyOn(performance, 'now').mockImplementation(() => 100);
  });

  afterEach(() => {
    performanceSpy?.mockRestore?.();
    global.Worker = originalWorker;
  });

  test('summarizes raw Klipper commands and bridge payloads for tracing', () => {
    expect(summarizeKlipperCommand({
      type: 'Move',
      at: '12.5',
      span: '0.25',
      A: '1',
      E: 0.5,
      note: 'ignored',
    })).toEqual({
      type: 'Move',
      at: 12.5,
      span: 0.25,
      axes: {
        A: 1,
        E: 0.5,
      },
    });

    expect(summarizeKlipperBridgeMessage({
      action: 'klipper_parsed',
      lines: ['config_stepper oid=0', 'queue_step oid=0 interval=1 count=1 add=0'],
    })).toEqual({
      action: 'klipper_parsed',
      lineCount: 2,
      firstLine: 'config_stepper oid=0',
      lastLine: 'queue_step oid=0 interval=1 count=1 add=0',
    });

    expect(summarizeKlipperBridgeMessage({
      action: 'klipper_clock',
      mcu_clock: '123',
      host_time: 456,
      clock_hz: 50_000_000,
    })).toEqual({
      action: 'klipper_clock',
      mcu_clock: 123,
      host_time: 456,
      clock_hz: 50_000_000,
    });

    expect(summarizeKlipperBridgeMessage({
      action: 'klipper_serial',
      data: 'abcd',
    })).toEqual({
      action: 'klipper_serial',
      payloadLength: 4,
    });
  });

  test('forwards raw dt updates and emits scheduling traces', () => {
    const traces = [];
    const receivedCommands = [];
    const handle = connectKlipperRaw('ws://example.invalid', (command) => {
      receivedCommands.push({ ...command });
    }, {
      dt: 0.02,
      initialSpeedScale: 2,
      traceRaw: true,
      onTrace: (trace) => {
        traces.push(trace);
      },
    });

    expect(FakeWorker.instances).toHaveLength(2);
    const [pacerWorker, timingWorker] = FakeWorker.instances;

    expect(pacerWorker.messages).toContainEqual({
      type: 'connect',
      url: 'ws://example.invalid',
      logMove: false,
      speedScale: 2,
      traceRaw: true,
    });
    expect(pacerWorker.messages).toContainEqual({
      type: 'set_dt',
      dt: 0.02,
    });
    expect(timingWorker.messages[0].type).toBe('init');
    expect(timingWorker.messages[0].sab).toBeInstanceOf(SharedArrayBuffer);

    handle.setDt(0.03);
    expect(pacerWorker.messages).toContainEqual({
      type: 'set_dt',
      dt: 0.03,
    });

    pacerWorker.onmessage?.({
      data: {
        type: 'move',
        command: { type: 'Move', at: 100, span: 0, A: 1 },
      },
    });

    expect(traces.some((trace) => trace.scope === 'worker' && trace.kind === 'move_emitted')).toBe(true);
    expect(traces.some((trace) => trace.scope === 'handler' && trace.kind === 'queue_inserted')).toBe(true);
    expect(traces.some((trace) => trace.scope === 'handler' && trace.kind === 'sleep_armed')).toBe(true);

    timingWorker.onmessage?.({
      data: {
        type: 'wakeup',
        reason: 'timeout',
      },
    });

    expect(receivedCommands).toEqual([
      { type: 'Move', at: 100, span: 0, A: 1 },
    ]);
    expect(traces.some((trace) => trace.scope === 'handler' && trace.kind === 'dispatch')).toBe(true);

    handle.close();
  });
});
