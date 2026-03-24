import net from 'node:net';

const DEFAULT_CLIENT_INFO = {
  program: 'klipper_api_bridge',
  version: 'v0.1',
};

function encodeRequest(message) {
  return `${JSON.stringify(message)}\x03`;
}

function toTextChunk(chunk) {
  if (typeof chunk === 'string') {
    return chunk;
  }
  if (Buffer.isBuffer(chunk)) {
    return chunk.toString('utf8');
  }
  return String(chunk);
}

export class KlipperApiBridge {
  constructor(options = {}) {
    this.socketPath = options.socketPath || process.env.KLIPPY_UDS || '/tmp/klippy_uds';
    this.clientInfo = options.clientInfo || DEFAULT_CLIENT_INFO;
    this.connectImpl = typeof options.connectImpl === 'function'
      ? options.connectImpl
      : (socketPath) => net.createConnection({ path: socketPath });
    this.onMessage = typeof options.onMessage === 'function' ? options.onMessage : null;
    this.onError = typeof options.onError === 'function' ? options.onError : null;
    this.onConnect = typeof options.onConnect === 'function' ? options.onConnect : null;
    this.onClose = typeof options.onClose === 'function' ? options.onClose : null;
    this.socket = null;
    this.isConnected = false;
    this.connectPromise = null;
    this.pendingRequests = new Map();
    this.nextRequestId = 1;
    this.buffer = '';
    this.closed = false;
  }

  async connect() {
    if (this.connectPromise) {
      return this.connectPromise;
    }

    this.closed = false;
    this.connectPromise = new Promise((resolve, reject) => {
      let socket;
      try {
        socket = this.connectImpl(this.socketPath);
      } catch (err) {
        this.connectPromise = null;
        reject(err);
        return;
      }
      if (!socket || typeof socket.on !== 'function' || typeof socket.write !== 'function') {
        this.connectPromise = null;
        reject(new Error('Klipper API bridge connectImpl must return a socket-like object'));
        return;
      }

      this.socket = socket;

      const settleConnect = () => {
        this.isConnected = true;
        if (this.onConnect) {
          try {
            this.onConnect();
          } catch (err) {
            this._emitError(err);
          }
        }
        resolve(this);
      };

      const settleError = (err) => {
        this._rejectPending(err);
        this.connectPromise = null;
        this.socket = null;
        this.isConnected = false;
        if (this.onError) {
          try {
            this.onError(err);
          } catch (_listenerErr) {
            // Ignore listener failures; the original error still propagates.
          }
        }
        reject(err);
      };

      socket.on('connect', settleConnect);
      socket.on('data', (chunk) => this._handleData(chunk));
      socket.on('error', settleError);
      socket.on('close', () => {
        this.socket = null;
        this.isConnected = false;
        this.connectPromise = null;
        this.closed = true;
        this._rejectPending(new Error('Klipper API socket closed'));
        if (this.onClose) {
          try {
            this.onClose();
          } catch (err) {
            this._emitError(err);
          }
        }
      });
    });

    return this.connectPromise;
  }

  async request(method, params = {}, options = {}) {
    if (!this.isConnected) {
      await this.connect();
    }

    const expectResponse = options.expectResponse !== false;
    const requestId = expectResponse ? this.nextRequestId++ : null;
    const payload = requestId == null
      ? { method, params }
      : { id: requestId, method, params };

    this._write(payload);

    if (!expectResponse) {
      return null;
    }

    return new Promise((resolve, reject) => {
      this.pendingRequests.set(requestId, { resolve, reject });
    });
  }

  async sendGcodeLine(line) {
    const script = typeof line === 'string' ? line.trim() : '';
    if (!script) {
      return null;
    }
    return this.request('gcode/script', { script });
  }

  async subscribeTerminalOutput() {
    return this.request('gcode/subscribe_output', { response_template: {} });
  }

  async subscribeStepperDump(stepperName) {
    return this.request('motion_report/dump_stepper', {
      name: stepperName,
      response_template: {},
    });
  }

  close() {
    this.closed = true;
    const socket = this.socket;
    this.socket = null;
    this.isConnected = false;
    this.connectPromise = null;
    this._rejectPending(new Error('Klipper API bridge closed'));
    if (socket) {
      try {
        socket.end();
      } catch (_err) {
        // Ignore close failures.
      }
      try {
        socket.destroy?.();
      } catch (_err) {
        // Ignore close failures.
      }
    }
  }

  _emitError(err) {
    if (this.onError) {
      try {
        this.onError(err);
      } catch (_listenerErr) {
        // Ignore listener failures.
      }
    }
  }

  _rejectPending(err) {
    if (this.pendingRequests.size === 0) {
      return;
    }
    for (const [, pending] of this.pendingRequests.entries()) {
      try {
        pending.reject(err);
      } catch (_listenerErr) {
        // Ignore listener failures.
      }
    }
    this.pendingRequests.clear();
  }

  _write(message) {
    if (!this.socket) {
      throw new Error('Klipper API socket is not connected');
    }
    this.socket.write(encodeRequest(message));
  }

  _handleData(chunk) {
    this.buffer += toTextChunk(chunk);
    let splitIndex;
    while ((splitIndex = this.buffer.indexOf('\x03')) >= 0) {
      const rawMessage = this.buffer.slice(0, splitIndex);
      this.buffer = this.buffer.slice(splitIndex + 1);
      if (!rawMessage) {
        continue;
      }
      let msg;
      try {
        msg = JSON.parse(rawMessage);
      } catch (err) {
        this._emitError(new Error(`Failed to parse Klipper API message: ${err.message}`));
        continue;
      }
      this._handleMessage(msg);
    }
  }

  _handleMessage(msg) {
    if (msg && Object.prototype.hasOwnProperty.call(msg, 'id')) {
      const pending = this.pendingRequests.get(msg.id);
      if (pending) {
        this.pendingRequests.delete(msg.id);
        if (msg.error) {
          const errorMessage = msg.error?.message || 'Klipper request failed';
          const error = new Error(errorMessage);
          if (msg.error?.error) {
            error.code = msg.error.error;
          }
          pending.reject(error);
        } else {
          pending.resolve(msg.result ?? {});
        }
        return;
      }
    }

    if (this.onMessage) {
      try {
        this.onMessage(msg);
      } catch (err) {
        this._emitError(err);
      }
    }
  }
}
