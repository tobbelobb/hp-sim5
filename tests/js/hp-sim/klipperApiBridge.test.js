import { EventEmitter } from 'node:events';

let KlipperApiBridge;

class FakeSocket extends EventEmitter {
  constructor() {
    super();
    this.writes = [];
    this.ended = false;
    this.destroyed = false;
  }

  write(chunk) {
    this.writes.push(typeof chunk === 'string' ? chunk : chunk.toString('utf8'));
    return true;
  }

  end() {
    this.ended = true;
    this.emit('close');
  }

  destroy() {
    this.destroyed = true;
  }
}

async function importModule() {
  if (KlipperApiBridge) {
    return;
  }
  ({ KlipperApiBridge } = await import('../../../autocal/control/primitives/klipper_api_bridge.mjs'));
}

describe('KlipperApiBridge', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('frames gcode/script requests with the Klipper 0x03 delimiter', async () => {
    const socket = new FakeSocket();
    const bridge = new KlipperApiBridge({
      socketPath: '/tmp/test.sock',
      connectImpl: () => {
        process.nextTick(() => socket.emit('connect'));
        return socket;
      },
    });

    await bridge.connect();
    const pending = bridge.sendGcodeLine('G90');

    expect(socket.writes).toEqual([
      '{"id":1,"method":"gcode/script","params":{"script":"G90"}}\x03',
    ]);

    socket.emit('data', Buffer.from('{"id":1,"result":{"ok":true}}\x03'));

    await expect(pending).resolves.toEqual({ ok: true });
  });

  test('can append M400 so one-shot calls wait for motion completion', async () => {
    const socket = new FakeSocket();
    const bridge = new KlipperApiBridge({
      socketPath: '/tmp/test.sock',
      connectImpl: () => {
        process.nextTick(() => socket.emit('connect'));
        return socket;
      },
    });

    await bridge.connect();
    const pending = bridge.sendGcodeLine('G1 X10', { waitForMotion: true });

    expect(socket.writes).toEqual([
      '{"id":1,"method":"gcode/script","params":{"script":"G1 X10\\nM400"}}\x03',
    ]);

    socket.emit('data', Buffer.from('{"id":1,"result":{"ok":true}}\x03'));

    await expect(pending).resolves.toEqual({ ok: true });
  });

  test('delivers asynchronous subscribe messages to onMessage', async () => {
    const socket = new FakeSocket();
    const asyncMessages = [];
    const bridge = new KlipperApiBridge({
      socketPath: '/tmp/test.sock',
      onMessage: (msg) => {
        asyncMessages.push(msg);
      },
      connectImpl: () => {
        process.nextTick(() => socket.emit('connect'));
        return socket;
      },
    });

    await bridge.connect();
    const subPromise = bridge.subscribeStepperDump('stepper_a');

    expect(socket.writes).toEqual([
      '{"id":1,"method":"motion_report/dump_stepper","params":{"name":"stepper_a","response_template":{"stepper_name":"stepper_a"}}}\x03',
    ]);

    socket.emit('data', Buffer.from('{"id":1,"result":{"header":["interval","count","add"]}}\x03'));
    await expect(subPromise).resolves.toEqual({ header: ['interval', 'count', 'add'] });

    socket.emit('data', Buffer.from('{"stepper_name":"stepper_a","params":{"data":[[10,2,0]]}}\x03'));

    expect(asyncMessages).toEqual([
      { stepper_name: 'stepper_a', params: { data: [[10, 2, 0]] } },
    ]);
  });
});
