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

  function createHarness() {
    const client = {
      subscribe: jest.fn(async () => ({ header: ['interval', 'count', 'add'] })),
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
        { id: 'sub:motion:stepper:stepper_a' },
      );
      expect(client.subscribe).toHaveBeenNthCalledWith(
        2,
        'motion_report/dump_stepper',
        { name: 'stepper_b' },
        expect.any(Function),
        { id: 'sub:motion:stepper:stepper_b' },
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
});
