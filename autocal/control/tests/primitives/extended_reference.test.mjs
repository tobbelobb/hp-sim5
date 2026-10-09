import { EventEmitter } from 'node:events';
import { createReferenceLogger } from '../../primitives/extended_reference.mjs';

class Socket extends EventEmitter {
  static current;
  constructor() { super(); Socket.current = this; this.readyState = 0; this.sent = []; }
  send(data, callback) { this.sent.push(JSON.parse(data)); callback?.(); }
  open() { this.readyState = 1; this.emit('open'); }
  ack() { this.emit('message', JSON.stringify({ type: 'event_ack' })); }
  close() { this.readyState = 3; }
}
const logger = options => createReferenceLogger('ws://test', { WebSocketClass: Socket, ...options });

test('events enqueue without blocking commands; completion drains all acknowledgements', async () => {
  const log = logger();
  const clock = { backend: 'browser-js', now: () => 123, observedWallMs: 456 };
  expect(log.emit('gcode_send', { line: 'G1 X1' }, clock)).toBeUndefined();
  expect(log.emit('gcode_reply', { reply: 'ok' }, clock)).toBeUndefined();
  const socket = Socket.current;
  let complete = false;
  const completion = log.close().then(() => { complete = true; });
  socket.open();
  expect(socket.sent.map(event => event.sequence)).toEqual([1, 2]);
  expect(socket.sent[0]).toMatchObject({ sim_time_s: .123, sim_time_observed_wall_ms: 456 });
  socket.ack(); await Promise.resolve();
  expect(complete).toBe(false);
  socket.ack(); await completion;
  expect(complete).toBe(true);
});

test('queue overflow fails capture instead of silently dropping events', async () => {
  const log = logger({ maxEvents: 1 });
  log.emit('first', {});
  expect(() => log.emit('second', {})).toThrow('incomplete');
  await expect(log.close()).rejects.toThrow('limit');
});

test('invalid transport setup fails immediately and does not leave cleanup waiting', async () => {
  const log = logger({ WebSocketClass: class { constructor() { throw new Error('Invalid URL'); } } });
  expect(() => log.emit('event', {})).toThrow('Invalid URL');
  await expect(log.close()).rejects.toThrow('Invalid URL');
});

test('a recorder that stops acknowledging has a bounded failure', async () => {
  jest.useFakeTimers();
  try {
    const log = logger({ ackTimeoutMs: 100 });
    log.emit('event', {}); Socket.current.open();
    const drained = expect(log.close()).rejects.toThrow('timed out');
    jest.advanceTimersByTime(100);
    await drained;
    expect(() => log.emit('later', {})).toThrow('timed out');
  } finally { jest.useRealTimers(); }
});
