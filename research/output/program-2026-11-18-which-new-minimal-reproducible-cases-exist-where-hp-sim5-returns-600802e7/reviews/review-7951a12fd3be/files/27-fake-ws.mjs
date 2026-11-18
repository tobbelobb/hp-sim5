// Fake `ws` module: an in-process WebSocketServer/client pair with no sockets.
// Used only through a module loader hook so the real rrfSimulatorBridge runs.
import { EventEmitter } from 'node:events';

class FakeSocket extends EventEmitter {
  constructor(url = 'ws://fake') {
    super();
    this.url = url;
    this.readyState = 1; // OPEN, matching ws
    this.sent = [];
    this.terminated = false;
    this.closed = false;
  }
  send(data) { this.sent.push(JSON.parse(data)); }
  close(code = 1000, reason = '') {
    if (this.closed) return;
    this.closed = true;
    this.readyState = 3;
    this.emit('close', code, Buffer.from(reason));
  }
  terminate() { this.close(1006, 'terminated'); }
  // test helper: deliver a client->server message
  receive(payload) { this.emit('message', Buffer.from(JSON.stringify(payload))); }
}

export class WebSocketServer extends EventEmitter {
  static instances = [];
  constructor(options = {}) {
    super();
    this.options = options;
    this.clients = new Set();
    this.closed = false;
    WebSocketServer.instances.push(this);
  }
  // test helper: attach a fake browser client
  connectClient() {
    const socket = new FakeSocket();
    this.clients.add(socket);
    socket.on('close', () => this.clients.delete(socket));
    this.emit('connection', socket);
    return socket;
  }
  close() { this.closed = true; for (const c of [...this.clients]) c.close(1001, 'server closing'); }
}

export const WebSocket = FakeSocket;
export default { WebSocketServer, WebSocket };
