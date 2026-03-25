import { EventEmitter } from 'node:events';

let createReplayableJsonBroadcaster;

class FakeSocket extends EventEmitter {
  constructor() {
    super();
    this.readyState = 1;
    this.sent = [];
  }

  send(data) {
    this.sent.push(data);
  }
}

async function importModule() {
  if (createReplayableJsonBroadcaster) {
    return;
  }
  ({ createReplayableJsonBroadcaster } = await import('../../../autocal/control/primitives/replayable_json_broadcaster.mjs'));
}

describe('replayable JSON broadcaster', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('replays buffered payloads to late clients after a reset marker', () => {
    const broadcaster = createReplayableJsonBroadcaster({ maxHistory: 2 });
    broadcaster.broadcast({ action: 'klipper_parsed', lines: ['config_stepper oid=0'] });
    broadcaster.broadcast({ action: 'klipper_clock', mcu_clock: 123 });

    const firstSocket = new FakeSocket();
    broadcaster.register(firstSocket);

    expect(firstSocket.sent).toEqual([
      '{"type":"reset"}',
      '{"action":"klipper_parsed","lines":["config_stepper oid=0"]}',
      '{"action":"klipper_clock","mcu_clock":123}',
    ]);

    broadcaster.broadcast({ action: 'klipper_parsed', lines: ['queue_step oid=0 interval=1 count=1 add=0'] });

    expect(firstSocket.sent).toEqual([
      '{"type":"reset"}',
      '{"action":"klipper_parsed","lines":["config_stepper oid=0"]}',
      '{"action":"klipper_clock","mcu_clock":123}',
      '{"action":"klipper_parsed","lines":["queue_step oid=0 interval=1 count=1 add=0"]}',
    ]);

    const secondSocket = new FakeSocket();
    broadcaster.register(secondSocket);

    expect(secondSocket.sent).toEqual([
      '{"type":"reset"}',
      '{"action":"klipper_clock","mcu_clock":123}',
      '{"action":"klipper_parsed","lines":["queue_step oid=0 interval=1 count=1 add=0"]}',
    ]);
  });
});

