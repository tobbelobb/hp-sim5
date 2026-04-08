import net from 'node:net';
import { EventEmitter } from 'node:events';

const FRAME_DELIMITER = 0x03;
const DEFAULT_CONNECT_TIMEOUT_MS = 2_000;
const DEFAULT_RECONNECT_DELAY_MS = 200;
const DEFAULT_MAX_RECONNECT_DELAY_MS = 2_000;
const DEFAULT_REQUEST_TIMEOUT_MS = 15_000;

function buildRequestError(message, extras = {}) {
  const error = new Error(message);
  Object.assign(error, extras);
  return error;
}

function isSocketClosedError(error) {
  return error?.code === 'EPIPE' || error?.code === 'ERR_STREAM_DESTROYED';
}

export class KlippyApiClient extends EventEmitter {
  constructor({
    socketPath,
    connectTimeoutMs = DEFAULT_CONNECT_TIMEOUT_MS,
    reconnectDelayMs = DEFAULT_RECONNECT_DELAY_MS,
    maxReconnectDelayMs = DEFAULT_MAX_RECONNECT_DELAY_MS,
    requestTimeoutMs = DEFAULT_REQUEST_TIMEOUT_MS,
  } = {}) {
    super();
    if (!socketPath) {
      throw new Error('KlippyApiClient requires a socketPath.');
    }
    this.socketPath = socketPath;
    this.connectTimeoutMs = connectTimeoutMs;
    this.reconnectDelayMs = reconnectDelayMs;
    this.maxReconnectDelayMs = maxReconnectDelayMs;
    this.requestTimeoutMs = requestTimeoutMs;

    this.socket = null;
    this.socketBuffer = Buffer.alloc(0);
    this.pendingRequests = new Map();
    this.subscriptions = new Map();
    this.nextRequestId = 1;
    this.connecting = false;
    this.connected = false;
    this.closed = false;
    this.reconnectTimer = null;
    this.reconnectAttempt = 0;
    this.connectionGeneration = 0;
  }

  start() {
    if (this.closed) {
      throw new Error('KlippyApiClient is closed.');
    }
    this._ensureConnected();
    return this.waitForConnection();
  }

  async waitForConnection(timeoutMs = 0) {
    if (this.closed) {
      throw new Error('KlippyApiClient is closed.');
    }
    if (this.connected && this.socket && !this.socket.destroyed) {
      return;
    }
    this._ensureConnected();
    await new Promise((resolve, reject) => {
      let timer = null;
      const cleanup = () => {
        this.off('connected', onConnected);
        this.off('closed', onClosed);
        if (timer) {
          clearTimeout(timer);
        }
      };
      const onConnected = () => {
        cleanup();
        resolve();
      };
      const onClosed = () => {
        cleanup();
        reject(new Error('KlippyApiClient is closed.'));
      };
      this.on('connected', onConnected);
      this.on('closed', onClosed);
      if (timeoutMs > 0) {
        timer = setTimeout(() => {
          cleanup();
          reject(buildRequestError(
            `Timed out waiting for Klippy API socket at ${this.socketPath}.`,
            { code: 'ETIMEDOUT' },
          ));
        }, timeoutMs);
      }
      if (this.connected && this.socket && !this.socket.destroyed) {
        cleanup();
        resolve();
      }
    });
  }

  async request(method, params = {}, options = {}) {
    if (!method) {
      throw new Error('KlippyApiClient.request() requires a method.');
    }
    const id = options.id ?? `req:${this.nextRequestId++}`;
    const timeoutMs = Number.isFinite(options.timeoutMs)
      ? Math.max(1, options.timeoutMs)
      : this.requestTimeoutMs;

    let hasAttemptedWrite = false;
    while (!this.closed) {
      await this.waitForConnection(timeoutMs);
      try {
        const result = await this._sendRequest({ id, method, params, timeoutMs });
        hasAttemptedWrite = true;
        return result;
      } catch (error) {
        if (!hasAttemptedWrite && isSocketClosedError(error)) {
          continue;
        }
        throw error;
      }
    }
    throw new Error('KlippyApiClient is closed.');
  }

  async subscribe(method, params = {}, asyncHandler = null, options = {}) {
    if (!method) {
      throw new Error('KlippyApiClient.subscribe() requires a method.');
    }
    const id = options.id ?? `sub:${this.nextRequestId++}`;
    const subscription = {
      id,
      method,
      params: { ...params },
      asyncHandler: typeof asyncHandler === 'function' ? asyncHandler : null,
      onResponse: typeof options.onResponse === 'function' ? options.onResponse : null,
    };
    this.subscriptions.set(id, subscription);
    const result = await this._sendSubscription(subscription);
    return result;
  }

  unsubscribe(id) {
    return this.subscriptions.delete(id);
  }

  close() {
    if (this.closed) {
      return;
    }
    this.closed = true;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    const socket = this.socket;
    this.socket = null;
    this.connected = false;
    this.connecting = false;
    if (socket && !socket.destroyed) {
      socket.destroy();
    }
    this._rejectPendingRequests(new Error('KlippyApiClient closed.'));
    this.emit('closed');
    this.removeAllListeners();
  }

  async _sendSubscription(subscription) {
    const responseTemplate = {
      ...(subscription.params.response_template || {}),
      q: subscription.id,
    };
    const requestParams = {
      ...subscription.params,
      response_template: responseTemplate,
    };
    const result = await this.request(subscription.method, requestParams, {
      id: subscription.id,
    });
    if (subscription.onResponse) {
      subscription.onResponse(result);
    }
    return result;
  }

  async _sendRequest({ id, method, params, timeoutMs }) {
    if (this.pendingRequests.has(id)) {
      throw new Error(`Klippy request id "${id}" is already in flight.`);
    }
    const socket = this.socket;
    if (!socket || socket.destroyed || !this.connected) {
      throw buildRequestError('Klippy API socket is not connected.', { code: 'ERR_STREAM_DESTROYED' });
    }
    const payload = JSON.stringify({ id, method, params });
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingRequests.delete(id);
        reject(buildRequestError(
          `Klippy request "${method}" timed out after ${timeoutMs}ms.`,
          { code: 'ETIMEDOUT' },
        ));
      }, timeoutMs);
      this.pendingRequests.set(id, {
        method,
        resolve: (value) => {
          clearTimeout(timer);
          resolve(value);
        },
        reject: (error) => {
          clearTimeout(timer);
          reject(error);
        },
      });
      socket.write(`${payload}${String.fromCharCode(FRAME_DELIMITER)}`, (error) => {
        if (!error) {
          return;
        }
        const pending = this.pendingRequests.get(id);
        if (!pending) {
          return;
        }
        this.pendingRequests.delete(id);
        clearTimeout(timer);
        reject(error);
      });
    });
  }

  _ensureConnected() {
    if (this.closed || this.connected || this.connecting) {
      return;
    }
    if (this.reconnectTimer) {
      return;
    }
    this._connectSocket();
  }

  _connectSocket() {
    if (this.closed || this.connected || this.connecting) {
      return;
    }
    this.connecting = true;
    this.socketBuffer = Buffer.alloc(0);

    const socket = net.createConnection(this.socketPath);
    let connectTimer = null;
    let didConnect = false;

    const cleanupConnectTimer = () => {
      if (connectTimer) {
        clearTimeout(connectTimer);
        connectTimer = null;
      }
    };

    connectTimer = setTimeout(() => {
      socket.destroy(buildRequestError(
        `Timed out connecting to Klippy API socket at ${this.socketPath}.`,
        { code: 'ETIMEDOUT' },
      ));
    }, this.connectTimeoutMs);

    socket.on('connect', () => {
      cleanupConnectTimer();
      didConnect = true;
      this.socket = socket;
      this.connecting = false;
      this.connected = true;
      this.reconnectAttempt = 0;
      this.connectionGeneration += 1;
      const generation = this.connectionGeneration;
      this.emit('connected');
      this._resubscribeAll(generation).catch((error) => {
        this.emit('error', error);
      });
    });

    socket.on('data', (chunk) => {
      this._handleSocketData(chunk);
    });

    socket.on('error', (error) => {
      cleanupConnectTimer();
      this.emit('socket-error', error);
    });

    socket.on('close', () => {
      cleanupConnectTimer();
      const wasConnected = didConnect && this.connected && this.socket === socket;
      if (this.socket === socket) {
        this.socket = null;
      }
      this.connected = false;
      this.connecting = false;
      this.socketBuffer = Buffer.alloc(0);
      this._rejectPendingRequests(new Error('Klippy API socket disconnected.'));
      if (wasConnected) {
        this.emit('disconnected');
      }
      if (!this.closed) {
        this._scheduleReconnect();
      }
    });
  }

  _scheduleReconnect() {
    if (this.closed || this.connected || this.connecting || this.reconnectTimer) {
      return;
    }
    const delay = Math.min(
      this.maxReconnectDelayMs,
      this.reconnectDelayMs * (2 ** this.reconnectAttempt),
    );
    this.reconnectAttempt += 1;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this._connectSocket();
    }, delay);
  }

  _handleSocketData(chunk) {
    this.socketBuffer = Buffer.concat([this.socketBuffer, chunk]);
    while (true) {
      const frameIndex = this.socketBuffer.indexOf(FRAME_DELIMITER);
      if (frameIndex === -1) {
        return;
      }
      const frame = this.socketBuffer.subarray(0, frameIndex);
      this.socketBuffer = this.socketBuffer.subarray(frameIndex + 1);
      if (frame.length === 0) {
        continue;
      }
      let message = null;
      try {
        message = JSON.parse(frame.toString());
      } catch (error) {
        this.emit('error', buildRequestError('Failed to parse Klippy API response.', {
          cause: error,
          rawMessage: frame.toString(),
        }));
        continue;
      }
      this._routeMessage(message);
    }
  }

  _routeMessage(message) {
    const asyncId = message?.q;
    if (asyncId != null && this.subscriptions.has(asyncId)) {
      const subscription = this.subscriptions.get(asyncId);
      if (subscription?.asyncHandler) {
        subscription.asyncHandler(message.params || {}, message);
      }
      this.emit('async-message', message);
      return;
    }

    const requestId = message?.id;
    if (requestId != null && this.pendingRequests.has(requestId)) {
      const pending = this.pendingRequests.get(requestId);
      this.pendingRequests.delete(requestId);
      if (message.error) {
        pending.reject(buildRequestError(
          message.error.message || `Klippy request "${pending.method}" failed.`,
          { code: message.error.error, data: message.error },
        ));
        return;
      }
      pending.resolve(message.result || {});
      return;
    }

    this.emit('message', message);
  }

  async _resubscribeAll(connectionGeneration) {
    if (this.subscriptions.size === 0) {
      return;
    }
    for (const subscription of this.subscriptions.values()) {
      if (this.closed || !this.connected || this.connectionGeneration !== connectionGeneration) {
        return;
      }
      try {
        await this._sendSubscription(subscription);
      } catch (error) {
        this.emit('subscription-error', {
          subscriptionId: subscription.id,
          method: subscription.method,
          error,
        });
      }
    }
  }

  _rejectPendingRequests(error) {
    for (const pending of this.pendingRequests.values()) {
      pending.reject(error);
    }
    this.pendingRequests.clear();
  }
}

