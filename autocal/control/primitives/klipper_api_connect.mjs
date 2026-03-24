const RETRYABLE_SOCKET_ERROR_CODES = new Set([
  'ENOENT',
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
  return message.includes('ENOENT')
    || message.includes('ECONNREFUSED')
    || message.includes('EHOSTUNREACH')
    || message.includes('ENETUNREACH');
}

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export async function connectKlipperApiBridgeWithRetry(createBridge, {
  timeoutMs = 15000,
  retryDelayMs = 100,
} = {}) {
  if (typeof createBridge !== 'function') {
    throw new Error('createBridge is required');
  }

  const deadline = Date.now() + Math.max(1, timeoutMs);
  let lastError = null;

  while (Date.now() < deadline) {
    const bridge = createBridge();
    try {
      await bridge.connect();
      return bridge;
    } catch (err) {
      lastError = err;
      try {
        bridge.close?.();
      } catch (_closeErr) {
        // Ignore close failures while retrying.
      }
      if (!isRetryableSocketError(err)) {
        throw err;
      }
      await wait(retryDelayMs);
    }
  }

  throw lastError || new Error('Timed out connecting to Klipper API socket');
}

