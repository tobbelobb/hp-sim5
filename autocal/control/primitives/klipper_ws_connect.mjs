const RETRYABLE_SOCKET_ERROR_CODES = new Set([
  'ECONNREFUSED',
  'EHOSTUNREACH',
  'ENETUNREACH',
]);

function isRetryableSocketError(err) {
  if (!err) {
    return false;
  }
  if (RETRYABLE_SOCKET_ERROR_CODES.has(err.code)) {
    return true;
  }
  const message = typeof err.message === 'string' ? err.message : '';
  return message.includes('ECONNREFUSED') || message.includes('EHOSTUNREACH') || message.includes('ENETUNREACH');
}

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export async function connectWebSocketWithRetry(url, {
  WebSocketImpl,
  timeoutMs = 10000,
  retryDelayMs = 100,
} = {}) {
  if (typeof WebSocketImpl !== 'function') {
    throw new Error('WebSocketImpl is required');
  }

  const deadline = Date.now() + Math.max(1, timeoutMs);

  while (true) {
    const socket = new WebSocketImpl(url);
    const result = await new Promise((resolve, reject) => {
      const cleanup = () => {
        socket.removeListener?.('open', onOpen);
        socket.removeListener?.('error', onError);
      };
      const onOpen = () => {
        cleanup();
        resolve(socket);
      };
      const onError = (err) => {
        cleanup();
        reject(err);
      };
      socket.on?.('open', onOpen);
      socket.on?.('error', onError);
    }).catch((err) => ({ error: err, socket }));

    if (result && !result.error) {
      return result;
    }

    const { error } = result;
    try {
      result.socket?.close?.();
    } catch (_err) {
      // ignore close errors
    }

    if (!isRetryableSocketError(error) || Date.now() >= deadline) {
      throw error;
    }

    await wait(retryDelayMs);
  }
}

