import { EventEmitter } from 'node:events';

let connectWebSocketWithRetry;

class FakeSocket extends EventEmitter {
  close() {
    this.closed = true;
  }
}

async function importModule() {
  if (connectWebSocketWithRetry) {
    return;
  }
  ({ connectWebSocketWithRetry } = await import('../../../autocal/control/primitives/klipper_ws_connect.mjs'));
}

describe('Klipper websocket connection retry', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('retries ECONNREFUSED until the websocket becomes available', async () => {
    const attempts = [];
    let attemptCount = 0;
    const WebSocketImpl = class extends FakeSocket {
      constructor(url) {
        super();
        attempts.push(url);
        attemptCount += 1;
        process.nextTick(() => {
          if (attemptCount < 3) {
            const err = new Error('connect ECONNREFUSED 127.0.0.1:8770');
            err.code = 'ECONNREFUSED';
            this.emit('error', err);
          } else {
            this.emit('open');
          }
        });
      }
    };

    const socket = await connectWebSocketWithRetry('ws://127.0.0.1:8770', {
      WebSocketImpl,
      timeoutMs: 2000,
      retryDelayMs: 0,
    });

    expect(socket).toBeInstanceOf(WebSocketImpl);
    expect(attempts).toEqual([
      'ws://127.0.0.1:8770',
      'ws://127.0.0.1:8770',
      'ws://127.0.0.1:8770',
    ]);
  });
});

