import WebSocket from 'ws';
import fs from 'node:fs/promises';
import { randomUUID } from 'node:crypto';

// Each acknowledged event has its source timestamp, independent of arrival order.
export function createReferenceLogger(url = process.env.AUTOCAL_REFERENCE_WS) {
  let socket;
  let connected;
  let sequence = 0;
  const session = randomUUID();
  async function emit(kind, payload, clock) {
    if (!url) return;
    const event = { version: 1, type: 'autocal_event', source: 'collector', kind, payload,
      wall_time_ms: Date.now(), wall_time_source: 'collector.Date.now',
      monotonic_ms: performance.now(), pid: process.pid, session, sequence: ++sequence,
      sim_time_s: clock ? clock.now() / 1000 : null,
      sim_time_source: clock ? `collector.${clock.backend}` : 'unavailable' };
    if (!socket) {
      socket = new WebSocket(url, { handshakeTimeout: 5000 });
      connected = new Promise((resolve, reject) => {
        socket.once('open', resolve);
        socket.once('error', reject);
      });
    }
    await connected;
    await new Promise((resolve, reject) => {
      const timeout = setTimeout(() => finish(new Error('Reference recorder acknowledgement timed out')), 10000);
      const onMessage = data => {
        try {
          if (JSON.parse(data).type !== 'event_ack') throw new Error('Invalid reference acknowledgement');
          finish();
        } catch (error) { finish(error); }
      };
      const onClose = () => finish(new Error('Reference recorder disconnected'));
      const finish = error => {
        clearTimeout(timeout);
        socket.off('message', onMessage);
        socket.off('close', onClose);
        socket.off('error', finish);
        error ? reject(error) : resolve();
      };
      socket.once('message', onMessage);
      socket.once('close', onClose);
      socket.once('error', finish);
      socket.send(JSON.stringify(event), error => { if (error) finish(error); });
    });
  }
  async function artifact(file, clock) {
    if (url) await emit('artifact', { path: file, content: await fs.readFile(file, 'utf8') }, clock);
  }
  return { emit, artifact, close: () => socket?.close() };
}
