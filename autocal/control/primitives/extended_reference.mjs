import WebSocket from 'ws';
import fs from 'node:fs/promises';
import { randomUUID } from 'node:crypto';

// Bounded, lossless queue: control calls enqueue; completion waits for every acknowledgement.
export function createReferenceLogger(url = process.env.AUTOCAL_REFERENCE_WS, {
  WebSocketClass = WebSocket, maxBytes = 8 * 1024 * 1024, maxEvents = 4096, ackTimeoutMs = 10000,
} = {}) {
  let socket, timer, error;
  let sequence = 0, bytes = 0, inFlight = 0;
  const queue = [], waiters = [];
  const session = randomUUID();
  function fail(reason) {
    error ||= reason;
    clearTimeout(timer);
    for (const waiter of waiters.splice(0)) waiter.reject(error);
    socket?.close();
  }
  function watch() {
    clearTimeout(timer);
    if (queue.length) timer = setTimeout(() => fail(new Error('Reference recorder acknowledgement timed out')), ackTimeoutMs);
  }
  function pump() {
    if (error || socket?.readyState !== 1) return;
    while (inFlight < queue.length && inFlight < 64) {
      try { socket.send(queue[inFlight++], sendError => { if (sendError) fail(sendError); }); }
      catch (reason) { fail(reason); return; }
    }
  }
  function emit(kind, payload, clock) {
    if (!url) return;
    if (error) throw error;
    const event = { version: 1, type: 'autocal_event', source: 'collector', kind, payload,
      wall_time_ms: Date.now(), wall_time_source: 'collector.Date.now',
      monotonic_ms: performance.now(), pid: process.pid, session, sequence: ++sequence,
      sim_time_s: clock ? clock.now() / 1000 : null,
      sim_time_source: clock ? `collector.${clock.source ?? clock.backend}` : 'unavailable',
      sim_time_observed_wall_ms: clock?.observedWallMs ?? null };
    const data = JSON.stringify(event);
    const size = Buffer.byteLength(data);
    if (bytes + size > maxBytes || queue.length >= maxEvents) {
      fail(new Error('Reference recorder queue exceeded its limit; capture is incomplete'));
      throw error;
    }
    queue.push(data); bytes += size;
    if (!socket) {
      try { socket = new WebSocketClass(url, { handshakeTimeout: 5000 }); }
      catch (reason) { fail(reason); throw reason; }
      socket.on('open', pump);
      socket.on('error', fail);
      socket.on('close', () => { if (queue.length) fail(new Error('Reference recorder disconnected before draining')); });
      socket.on('message', data => {
        if (error) return;
        try {
          if (JSON.parse(data).type !== 'event_ack' || !inFlight) throw new Error('Invalid reference acknowledgement');
          bytes -= Buffer.byteLength(queue.shift()); inFlight -= 1;
          watch(); pump();
          if (!queue.length) for (const waiter of waiters.splice(0)) waiter.resolve();
        } catch (reason) { fail(reason); }
      });
    }
    if (queue.length === 1) watch();
    pump();
    if (error) throw error;
  }
  async function flush() {
    if (error) throw error;
    if (queue.length) await new Promise((resolve, reject) => waiters.push({ resolve, reject }));
  }
  async function artifact(file, clock) {
    if (url) emit('artifact', { path: file, content: await fs.readFile(file, 'utf8') }, clock);
  }
  return { emit, artifact, flush, async close() { await flush(); socket?.close(); } };
}
