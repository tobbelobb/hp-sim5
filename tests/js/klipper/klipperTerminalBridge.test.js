const { EventEmitter } = require('node:events');

function flushMicrotasks() {
  return new Promise((resolve) => {
    queueMicrotask(resolve);
  });
}

describe('createKlipperTerminalBridge', () => {
  let createKlipperTerminalBridge;
  let mcuClockHzKlipperHost;

  beforeEach(async () => {
    jest.resetModules();
    ({ createKlipperTerminalBridge } = await import('../../../integrations/klipper/klipperTerminalBridge.js'));
    ({ MCU_CLOCK_HZ_KLIPPER_HOST: mcuClockHzKlipperHost } = await import('../../../integrations/klipper/klipperFirmwareModel.js'));
  });

  function createHarness(options = {}) {
    const {
      encoderResolver = null,
      ...bridgeOptions
    } = options;
    const client = {
      subscribe: jest.fn(async (_method, _params, _handler, subscribeOptions = {}) => {
        const result = { header: ['interval', 'count', 'add'] };
        if (typeof subscribeOptions.onResponse === 'function') {
          subscribeOptions.onResponse(result);
        }
        return result;
      }),
      unsubscribe: jest.fn(),
    };
    const klippyState = new EventEmitter();
    klippyState.getSnapshot = () => ({
      motionSources: {
        steppers: [],
      },
    });
    const payloads = [];
    const bridge = createKlipperTerminalBridge({
      client,
      klippyState,
      wsPort: 0,
      motionIdleMs: 5,
      onBroadcast: (payload) => payloads.push(payload),
      encoderResolver,
      ...bridgeOptions,
    });
    return {
      client,
      klippyState,
      bridge,
      payloads,
    };
  }

  test('subscribes and unsubscribes dump_stepper streams from runtime motion source updates', async () => {
    const { client, klippyState, bridge } = createHarness();

    try {
      klippyState.emit('motion-sources-changed', { steppers: ['stepper_b', 'stepper_a'] });
      await flushMicrotasks();
      await flushMicrotasks();

      expect(client.subscribe).toHaveBeenCalledTimes(2);
      expect(client.subscribe).toHaveBeenNthCalledWith(
        1,
        'motion_report/dump_stepper',
        { name: 'stepper_a' },
        expect.any(Function),
        { id: 'sub:motion:stepper:stepper_a', onResponse: expect.any(Function) },
      );
      expect(client.subscribe).toHaveBeenNthCalledWith(
        2,
        'motion_report/dump_stepper',
        { name: 'stepper_b' },
        expect.any(Function),
        { id: 'sub:motion:stepper:stepper_b', onResponse: expect.any(Function) },
      );

      klippyState.emit('motion-sources-changed', { steppers: ['stepper_b'] });
      await flushMicrotasks();
      await flushMicrotasks();

      expect(client.unsubscribe).toHaveBeenCalledWith('sub:motion:stepper:stepper_a');
    } finally {
      bridge.close();
    }
  });

  test('forwards simultaneous stepper batches as a shared API motion session and emits a final reply', async () => {
    const { client, klippyState, bridge, payloads } = createHarness();

    try {
      klippyState.emit('motion-sources-changed', { steppers: ['stepper_a', 'stepper_b'] });
      await flushMicrotasks();
      await flushMicrotasks();

      const handlersByStepper = new Map(
        client.subscribe.mock.calls.map((call) => [call[1].name, call[2]]),
      );

      const result = await bridge.runGcodeCommand('G1 A1 B1', async () => {
        handlersByStepper.get('stepper_a')({
          first_clock: 0,
          start_mcu_position: 0,
          data: [[60000, 2, 0]],
        });
        handlersByStepper.get('stepper_b')({
          first_clock: 0,
          start_mcu_position: 0,
          data: [[60000, 2, 0]],
        });
        bridge.handleGcodeOutput('ok move\n');
      });

      expect(result).toMatchObject({
        reply: 'ok move',
        hadMotion: true,
        printedLiveOutput: true,
      });

      expect(payloads).toHaveLength(5);
      expect(payloads[0]).toEqual({
        type: 'klipper_api_session_start',
        gcode: 'G1 A1 B1',
        clock_hz: mcuClockHzKlipperHost,
      });
      expect(payloads[1]).toEqual({
        type: 'klipper_api_stepper_batch',
        gcode: 'G1 A1 B1',
        batch: {
          name: 'stepper_a',
          first_clock: 0,
          first_time: null,
          last_clock: undefined,
          last_time: null,
          start_mcu_position: 0,
          data: [[60000, 2, 0]],
        },
      });
      expect(payloads[2]).toEqual({
        type: 'klipper_api_stepper_batch',
        gcode: 'G1 A1 B1',
        batch: {
          name: 'stepper_b',
          first_clock: 0,
          first_time: null,
          last_clock: undefined,
          last_time: null,
          start_mcu_position: 0,
          data: [[60000, 2, 0]],
        },
      });
      expect(payloads[3]).toEqual({
        type: 'klipper_api_session_end',
        gcode: 'G1 A1 B1',
      });
      expect(payloads[4]).toEqual({
        type: 'reply',
        gcode: 'G1 A1 B1',
        reply: 'ok move',
      });
    } finally {
      bridge.close();
    }
  });

  test('forwards toolhead trapq batches into the API motion session', async () => {
    const { client, klippyState, bridge, payloads } = createHarness();

    try {
      klippyState.emit('motion-sources-changed', {
        steppers: ['stepper_a'],
        trapq: ['toolhead'],
      });
      await flushMicrotasks();
      await flushMicrotasks();

      const stepperHandler = client.subscribe.mock.calls.find(
        (call) => call[0] === 'motion_report/dump_stepper' && call[1].name === 'stepper_a',
      )[2];
      const trapqHandler = client.subscribe.mock.calls.find(
        (call) => call[0] === 'motion_report/dump_trapq' && call[1].name === 'toolhead',
      )[2];

      await bridge.runGcodeCommand('G1 X100', async () => {
        trapqHandler({
          data: [[1, 0.5, 0, 0, [0, 0, 0], [1, 0, 0]]],
        });
        stepperHandler({
          first_clock: 0,
          start_mcu_position: 0,
          data: [[60000, 1, 0]],
        });
      });

      expect(payloads).toEqual(expect.arrayContaining([
        {
          type: 'klipper_api_trapq_batch',
          gcode: 'G1 X100',
          batch: {
            name: 'toolhead',
            data: [[1, 0.5, 0, 0, [0, 0, 0], [1, 0, 0]]],
          },
        },
      ]));
    } finally {
      bridge.close();
    }
  });

  test('finishes a motion session as soon as stepper coverage reaches the trapq end', async () => {
    const { client, klippyState, bridge } = createHarness({
      motionIdleMs: 250,
    });

    try {
      klippyState.emit('motion-sources-changed', {
        steppers: ['stepper_a'],
        trapq: ['toolhead'],
      });
      await flushMicrotasks();
      await flushMicrotasks();

      const stepperHandler = client.subscribe.mock.calls.find(
        (call) => call[0] === 'motion_report/dump_stepper' && call[1].name === 'stepper_a',
      )[2];
      const trapqHandler = client.subscribe.mock.calls.find(
        (call) => call[0] === 'motion_report/dump_trapq' && call[1].name === 'toolhead',
      )[2];

      const resultPromise = bridge.runGcodeCommand('G1 X100', async () => {
        trapqHandler({
          data: [[10.0, 0.6, 0, 0, [0, 0, 0], [1, 0, 0]]],
        });
        stepperHandler({
          first_clock: 0,
          start_mcu_position: 0,
          last_time: 10.2,
          data: [[60000, 1, 0]],
        });
      });

      await expect(Promise.race([
        resultPromise.then(() => 'resolved'),
        new Promise((resolve) => setTimeout(() => resolve('timeout'), 50)),
      ])).resolves.toBe('timeout');

      stepperHandler({
        first_clock: 60_000,
        start_mcu_position: 1,
        last_time: 10.595,
        data: [[60000, 1, 0]],
      });

      await expect(Promise.race([
        resultPromise,
        new Promise((resolve) => setTimeout(() => resolve('timeout'), 50)),
      ])).resolves.toMatchObject({
        reply: 'ok',
        hadMotion: true,
        printedLiveOutput: false,
      });
    } finally {
      bridge.close();
    }
  });

  test('subscribes to dump_trapq only in debug mode and logs trapq batches', async () => {
    const debugEntries = [];
    const { client, klippyState, bridge } = createHarness({
      debugTrapq: true,
      debugLog: (event, payload) => debugEntries.push({ event, ...payload }),
    });

    try {
      klippyState.emit('motion-sources-changed', {
        steppers: ['stepper_a'],
        trapq: ['toolhead'],
      });
      await flushMicrotasks();
      await flushMicrotasks();

      expect(client.subscribe).toHaveBeenCalledWith(
        'motion_report/dump_trapq',
        { name: 'toolhead' },
        expect.any(Function),
        { id: 'sub:motion:trapq:toolhead', onResponse: expect.any(Function) },
      );

      const trapqHandler = client.subscribe.mock.calls.find(
        (call) => call[0] === 'motion_report/dump_trapq' && call[1].name === 'toolhead',
      )[2];
      trapqHandler({
        data: [[0, 1, 2, 3, 4, 5]],
      });

      expect(debugEntries).toEqual(expect.arrayContaining([
        expect.objectContaining({
          event: 'api-response',
          method: 'motion_report/dump_trapq',
          channel: 'initial',
          name: 'toolhead',
          result: { header: ['interval', 'count', 'add'] },
        }),
        expect.objectContaining({
          event: 'api-response',
          method: 'motion_report/dump_trapq',
          channel: 'async',
          name: 'toolhead',
          params: { data: [[0, 1, 2, 3, 4, 5]] },
        }),
      ]));
    } finally {
      bridge.close();
    }
  });

  test('overrides placeholder M569.3 output with simulated encoder values', async () => {
    const encoderResolver = jest.fn(async ({ axes }) => {
      expect(axes).toEqual(['A', 'B']);
      return [12.3456, -4.4];
    });
    const { bridge, payloads } = createHarness({ encoderResolver });

    try {
      const result = await bridge.runGcodeCommand('M569.3 P40.0:41.0', async () => {
        const forwarded = bridge.handleGcodeOutput('Error: M569.3: Message not received\n');
        expect(forwarded).toEqual([]);
      });

      expect(encoderResolver).toHaveBeenCalledTimes(1);
      expect(result).toMatchObject({
        reply: '[12.35, -4.40, ]',
        hadMotion: false,
        printedLiveOutput: false,
      });
      expect(payloads).toEqual([
        {
          type: 'reply',
          gcode: 'M569.3 P40.0:41.0',
          reply: '[12.35, -4.40, ]',
        },
      ]);
    } finally {
      bridge.close();
    }
  });

  test('tracks M569.3 S references locally across simulated encoder reads', async () => {
    const encoderResolver = jest.fn(async ({ axes }) => {
      expect(axes).toEqual(['A']);
      return [90];
    });
    const { bridge } = createHarness({ encoderResolver });

    try {
      const setReference = await bridge.runGcodeCommand('M569.3 P40.0 S', async () => {
        bridge.handleGcodeOutput('Error: M569.3: Message not received\n');
      });
      const relative = await bridge.runGcodeCommand('M569.3 P40.0', async () => {
        bridge.handleGcodeOutput('Error: M569.3: Message not received\n');
      });

      expect(setReference.reply).toBe('[0.00, ]');
      expect(relative.reply).toBe('[0.00, ]');
      expect(encoderResolver).toHaveBeenCalledTimes(2);
    } finally {
      bridge.close();
    }
  });

  test('falls back to the Klipper placeholder reply when no simulator encoder is available', async () => {
    const encoderResolver = jest.fn(async () => []);
    const { bridge, payloads } = createHarness({ encoderResolver });

    try {
      const result = await bridge.runGcodeCommand('M569.3 P40.0', async () => {
        const forwarded = bridge.handleGcodeOutput('Error: M569.3: Message not received\n');
        expect(forwarded).toEqual([]);
      });

      expect(result).toMatchObject({
        reply: 'Error: M569.3: Message not received',
        printedLiveOutput: false,
      });
      expect(payloads).toEqual([
        {
          type: 'reply',
          gcode: 'M569.3 P40.0',
          reply: 'Error: M569.3: Message not received',
        },
      ]);
    } finally {
      bridge.close();
    }
  });

  test('translates M569.4 torque replies into hp-sim torque mode commands', async () => {
    const { bridge, payloads } = createHarness();

    try {
      const result = await bridge.runGcodeCommand('M569.4 P40.0:41.0 T1.0:2.0', async () => {
        const forwarded = bridge.handleGcodeOutput('-0.039185 Nm, -0.078369 Nm,\n');
        expect(forwarded).toEqual(['-0.039185 Nm, -0.078369 Nm,']);
      });

      expect(result).toMatchObject({
        reply: '-0.039185 Nm, -0.078369 Nm,',
        printedLiveOutput: true,
      });
      expect(payloads).toEqual([
        expect.objectContaining({
          type: 'command',
          gcode: 'M569.4 P40.0:41.0 T1.0:2.0',
          command: expect.objectContaining({
            type: 'SetTorqueMode',
            axis: 'A',
            driver: 40,
            torqueNm: -0.039185,
          }),
        }),
        expect.objectContaining({
          type: 'command',
          gcode: 'M569.4 P40.0:41.0 T1.0:2.0',
          command: expect.objectContaining({
            type: 'SetTorqueMode',
            axis: 'B',
            driver: 41,
            torqueNm: -0.078369,
          }),
        }),
        {
          type: 'reply',
          gcode: 'M569.4 P40.0:41.0 T1.0:2.0',
          reply: '-0.039185 Nm, -0.078369 Nm,',
        },
      ]);
    } finally {
      bridge.close();
    }
  });

  test('translates mixed M569.4 position and torque replies into hp-sim mode changes', async () => {
    const { bridge, payloads } = createHarness();

    try {
      await bridge.runGcodeCommand('M569.4 P40.0:41.0 T0:2.0', async () => {
        bridge.handleGcodeOutput('pos_mode, -0.078369 Nm,\n');
      });

      expect(payloads).toEqual([
        expect.objectContaining({
          type: 'command',
          gcode: 'M569.4 P40.0:41.0 T0:2.0',
          command: expect.objectContaining({
            type: 'SetPositionMode',
            axis: 'A',
            driver: 40,
            torqueNm: 0,
          }),
        }),
        expect.objectContaining({
          type: 'command',
          gcode: 'M569.4 P40.0:41.0 T0:2.0',
          command: expect.objectContaining({
            type: 'SetTorqueMode',
            axis: 'B',
            driver: 41,
            torqueNm: -0.078369,
          }),
        }),
        {
          type: 'reply',
          gcode: 'M569.4 P40.0:41.0 T0:2.0',
          reply: 'pos_mode, -0.078369 Nm,',
        },
      ]);
    } finally {
      bridge.close();
    }
  });
});
