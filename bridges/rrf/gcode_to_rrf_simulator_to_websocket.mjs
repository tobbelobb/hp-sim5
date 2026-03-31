#!/usr/bin/env node
import { WebSocketServer } from 'ws';
import { RrfHttpBridge } from '../http/rrfHttpBridge.js';

const MAX_PENDING_WS_PAYLOADS = 5000;
const DEFAULT_ENCODER_TIMEOUT_MS = 2000;

function buildWsHelpers({ wsPort, quiet = false, onClientChange }) {
  const wss = wsPort ? new WebSocketServer({ port: wsPort }) : null;
  const pendingWsPayloads = [];
  let waitingForClientNoticePrinted = false;
  const pendingEncoderRequests = new Map();
  let encoderRequestSeq = 1;

  const getReadyWsClients = () => (wss
    ? Array.from(wss.clients).filter((client) => client.readyState === 1)
    : []);

  const hasReadyWsClients = () => getReadyWsClients().length > 0;

  const enqueuePendingPayload = (payload) => {
    if (!wss || !payload) {
      return;
    }
    pendingWsPayloads.push(payload);
    const overflow = pendingWsPayloads.length - MAX_PENDING_WS_PAYLOADS;
    if (overflow > 0) {
      pendingWsPayloads.splice(0, overflow);
    }
    if (!waitingForClientNoticePrinted && !quiet) {
      console.log('Waiting for hp-sim WebSocket client to connect before streaming...');
      waitingForClientNoticePrinted = true;
    }
  };

  const broadcast = (payload) => {
    if (!payload || !wss) {
      return;
    }
    const readyClients = getReadyWsClients();
    if (readyClients.length === 0) {
      enqueuePendingPayload(payload);
      return;
    }
    const data = JSON.stringify(payload);
    readyClients.forEach((client) => {
      client.send(data);
    });
  };

  const flushPendingPayloads = () => {
    if (!hasReadyWsClients() || pendingWsPayloads.length === 0) {
      return;
    }
    const batch = pendingWsPayloads.splice(0, pendingWsPayloads.length);
    if (!quiet) {
      console.log(`Streaming ${batch.length} queued message${batch.length === 1 ? '' : 's'} to hp-sim client.`);
    }
    batch.forEach((payload) => broadcast(payload));
    waitingForClientNoticePrinted = false;
  };

  const handleIncomingWsMessage = (data) => {
    if (!data) {
      return;
    }
    let payload = null;
    try {
      payload = typeof data === 'string' ? JSON.parse(data) : JSON.parse(data.toString());
    } catch (_err) {
      return;
    }
    if (payload?.type === 'encoder_response' && payload.requestId != null) {
      const pending = pendingEncoderRequests.get(payload.requestId);
      if (pending) {
        pendingEncoderRequests.delete(payload.requestId);
        pending.resolve(payload);
      }
    }
  };

  const sendEncoderRequest = (axes, timeoutMs = DEFAULT_ENCODER_TIMEOUT_MS) => {
    const readyClients = getReadyWsClients();
    if (readyClients.length === 0) {
      throw new Error('Message not received');
    }
    const requestId = encoderRequestSeq++;
    const payload = { type: 'encoder_request', requestId, axes };
    const data = JSON.stringify(payload);
    readyClients.forEach((client) => {
      try {
        client.send(data);
      } catch (_err) {
        /* ignore send errors; timeout will handle missing responses */
      }
    });
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        pendingEncoderRequests.delete(requestId);
        reject(new Error('Message not received'));
      }, Math.max(1, timeoutMs));
      pendingEncoderRequests.set(requestId, {
        resolve: (value) => {
          clearTimeout(timeout);
          pendingEncoderRequests.delete(requestId);
          resolve(value);
        },
        reject: (err) => {
          clearTimeout(timeout);
          pendingEncoderRequests.delete(requestId);
          reject(err);
        },
      });
    });
  };

  if (wss) {
    wss.on('connection', (socket) => {
      flushPendingPayloads();
      if (onClientChange) {
        onClientChange(true);
      }
      socket.on('message', (data) => handleIncomingWsMessage(data));
      socket.on('close', () => {
        if (onClientChange) {
          onClientChange(hasReadyWsClients());
        }
      });
    });
  }

  const waitForHpSimConnection = (timeoutMs = 0) => {
    if (!wss || hasReadyWsClients()) {
      return Promise.resolve();
    }
    let settled = false;
    const resolveOnce = (resolve) => {
      if (settled) {
        return;
      }
      settled = true;
      resolve();
    };

    return new Promise((resolve) => {
      const cleanup = () => {
        wss.off('connection', onConnection);
      };

      const onConnection = () => {
        cleanup();
        resolveOnce(resolve);
      };

      wss.on('connection', onConnection);

      const timer = timeoutMs > 0
        ? setTimeout(() => {
          cleanup();
          resolveOnce(resolve);
        }, timeoutMs)
        : null;

      process.nextTick(() => {
        if (settled) {
          return;
        }
        if (hasReadyWsClients()) {
          if (timer) {
            clearTimeout(timer);
          }
          cleanup();
          resolveOnce(resolve);
        }
      });
    });
  };

  const close = () => {
    if (wss) {
      wss.close();
    }
    pendingEncoderRequests.clear();
    pendingWsPayloads.splice(0, pendingWsPayloads.length);
  };

  return {
    wss,
    hasReadyWsClients,
    getReadyWsClients,
    broadcast,
    flushPendingPayloads,
    sendEncoderRequest,
    waitForHpSimConnection,
    close,
  };
}

export function createGcodeBridge({
  server = process.env.RRF_SERVER_URL || 'http://localhost:8080',
  wsPort = 8790,
  quiet = false,
  onClientChange = null,
  encoderTimeoutMs = DEFAULT_ENCODER_TIMEOUT_MS,
} = {}) {
  const helpers = buildWsHelpers({ wsPort, quiet, onClientChange });
  let currentGcode = null;

  const encoderResolver = helpers.wss
    ? async ({ axes, timeoutMs }) => {
      const timeout = Number.isFinite(timeoutMs)
        ? Math.max(1, Math.min(timeoutMs, 5000))
        : encoderTimeoutMs;
      const response = await helpers.sendEncoderRequest(axes, timeout);
      if (response && Array.isArray(response.anglesDeg)) {
        return response.anglesDeg;
      }
      if (Array.isArray(response?.angles)) {
        return response.angles;
      }
      return [];
    }
    : null;

  const bridge = new RrfHttpBridge({
    baseUrl: server.replace(/\/$/, ''),
    remoteSpoolSystem: {
      addCommand: (command) => {
        if (!command) {
          return;
        }
        helpers.broadcast({ type: 'command', command, gcode: currentGcode });
      },
    },
    encoderResolver,
  });

  const sendGcodeLine = async (line, options = {}) => {
    const trimmed = line?.trim?.();
    if (!trimmed) {
      return null;
    }
    currentGcode = trimmed;
    const result = await bridge.sendGCode(trimmed, options);
    helpers.broadcast({ type: 'reply', gcode: trimmed, reply: result.reply });
    currentGcode = null;
    return result;
  };

  return {
    bridge,
    ...helpers,
    sendGcodeLine,
  };
}
