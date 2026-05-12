export function normalizeWsUrl(raw) {
  if (!raw || typeof raw !== 'string') {
    return null;
  }
  const trimmed = raw.trim();
  if (!trimmed) {
    return null;
  }
  if (trimmed.startsWith('ws://') || trimmed.startsWith('wss://')) {
    return trimmed;
  }
  const cleaned = trimmed.replace(/^\/+/, '');
  return `ws://${cleaned}`;
}

export function createExternalCommandSocket({
  url,
  WebSocketCtor = globalThis.WebSocket,
  onPayload,
  onOpen,
  logger = console,
  reconnectDelayMs = 5000,
  logEveryAttempts = 10,
} = {}) {
  let socket = null;
  let connecting = false;
  let reconnectTimer = null;
  let attemptCount = 0;

  function clearReconnectTimer() {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
  }

  function scheduleReconnect(reason = '') {
    if (!url || !WebSocketCtor || socket || connecting || reconnectTimer) {
      return;
    }
    const attempts = Math.max(1, attemptCount);
    if (attempts % logEveryAttempts === 0) {
      const suffix = reason ? ` (${reason})` : '';
      logger.log(
        `hp-sim-3d: tried external G-code stream ${attempts} times${suffix}. Continuing to retry every ${Math.round(reconnectDelayMs)}ms`
      );
    }
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      connect();
    }, reconnectDelayMs);
  }

  function send(payload) {
    if (!socket || typeof socket.send !== 'function') {
      return false;
    }
    if (WebSocketCtor && socket.readyState !== WebSocketCtor.OPEN) {
      return false;
    }
    socket.send(typeof payload === 'string' ? payload : JSON.stringify(payload));
    return true;
  }

  function connect() {
    if (!url || socket || connecting || !WebSocketCtor) {
      return;
    }
    clearReconnectTimer();
    connecting = true;
    attemptCount += 1;
    try {
      socket = new WebSocketCtor(url);
    } catch (_err) {
      socket = null;
      connecting = false;
      scheduleReconnect('failed to open');
      return;
    }
    socket.addEventListener('open', () => {
      connecting = false;
      const attemptsBeforeConnect = attemptCount;
      attemptCount = 0;
      if (attemptsBeforeConnect > 1) {
        logger.log(`hp-sim-3d: external G-code stream connected after ${attemptsBeforeConnect} attempts:`, url);
      } else {
        logger.log('hp-sim-3d: external G-code stream connected:', url);
      }
      onOpen?.();
    });
    socket.addEventListener('message', (event) => {
      try {
        const payload = typeof event.data === 'string' ? JSON.parse(event.data) : event.data;
        onPayload?.(payload);
      } catch (err) {
        logger.warn('hp-sim-3d: failed to process external G-code payload.', err);
      }
    });
    socket.addEventListener('close', () => {
      socket = null;
      connecting = false;
      scheduleReconnect('connection closed');
    });
    socket.addEventListener('error', () => {
      try {
        socket?.close();
      } catch (_err) {
        // Ignore socket close errors.
      }
    });
  }

  return {
    connect,
    send,
    get socket() {
      return socket;
    },
    get connecting() {
      return connecting;
    },
  };
}

