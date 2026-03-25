import { EventEmitter } from 'node:events';

let createJsonBroadcaster;

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
  if (createJsonBroadcaster) {
    return;
  }
  ({ createJsonBroadcaster } = await import('../../../autocal/control/primitives/replayable_json_broadcaster.mjs'));
}

describe('live JSON broadcaster', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('only forwards payloads that were broadcast after a client connected', () => {
    const broadcaster = createJsonBroadcaster();
    broadcaster.broadcast({ action: 'klipper_parsed', lines: ['config_stepper oid=0'] });

    const firstSocket = new FakeSocket();
    broadcaster.register(firstSocket);

    expect(firstSocket.sent).toEqual([]);

    broadcaster.broadcast({ action: 'klipper_clock', mcu_clock: 123 });

    expect(firstSocket.sent).toEqual([
      '{"action":"klipper_clock","mcu_clock":123}',
    ]);

    const secondSocket = new FakeSocket();
    broadcaster.register(secondSocket);

    expect(secondSocket.sent).toEqual([]);

    broadcaster.broadcast({ action: 'klipper_parsed', lines: ['queue_step oid=0 interval=1 count=1 add=0'] });

    expect(firstSocket.sent).toEqual([
      '{"action":"klipper_clock","mcu_clock":123}',
      '{"action":"klipper_parsed","lines":["queue_step oid=0 interval=1 count=1 add=0"]}',
    ]);
    expect(secondSocket.sent).toEqual([
      '{"action":"klipper_parsed","lines":["queue_step oid=0 interval=1 count=1 add=0"]}',
    ]);
  });
});
