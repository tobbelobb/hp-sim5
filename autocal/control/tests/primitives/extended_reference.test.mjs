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

test('queue overflow degrades capture without failing the command', async () => {
  const log = logger({ maxEvents: 1 });
  log.emit('first', {});
  expect(() => log.emit('second', {})).not.toThrow();
  expect(log.failure().message).toMatch('limit');
  await expect(log.close()).resolves.toBeUndefined();
});

test('invalid transport setup degrades immediately without leaving cleanup waiting', async () => {
  const log = logger({ WebSocketClass: class { constructor() { throw new Error('Invalid URL'); } } });
  expect(() => log.emit('event', {})).not.toThrow();
  expect(log.failure().message).toBe('Invalid URL');
  await expect(log.close()).resolves.toBeUndefined();
});

test('unserializable events degrade capture without throwing', async () => {
  const log = logger();
  const payload = {}; payload.self = payload;
  expect(() => log.emit('event', payload)).not.toThrow();
  expect(log.failure()).toBeInstanceOf(TypeError);
  await expect(log.close()).resolves.toBeUndefined();
});

test('a recorder that stops acknowledging disables capture non-fatally', async () => {
  jest.useFakeTimers();
  try {
    const log = logger({ ackTimeoutMs: 100 });
    log.emit('event', {}); Socket.current.open();
    jest.advanceTimersByTime(100);
    expect(log.failure().message).toMatch('timed out');
    expect(() => log.emit('later', {})).not.toThrow();
    await expect(log.close()).resolves.toBeUndefined();
  } finally { jest.useRealTimers(); }
});
