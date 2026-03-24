const READY_STATES = new Set(['ready']);
const SHUTDOWN_STATES = new Set(['shutdown']);

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export async function waitForKlippyReady(apiBridge, {
  timeoutMs = 30000,
  pollMs = 200,
} = {}) {
  if (!apiBridge || typeof apiBridge.request !== 'function') {
    throw new Error('apiBridge is required');
  }

  const deadline = Date.now() + Math.max(1, timeoutMs);
  let lastMessage = 'Klippy is not ready';

  while (Date.now() < deadline) {
    const result = await apiBridge.request('objects/query', {
      objects: {
        webhooks: ['state', 'state_message'],
      },
    });
    const status = result?.status?.webhooks || {};
    const state = status.state;
    if (typeof status.state_message === 'string' && status.state_message.trim()) {
      lastMessage = status.state_message.trim();
    }
    if (READY_STATES.has(state)) {
      return result;
    }
    if (SHUTDOWN_STATES.has(state)) {
      throw new Error(lastMessage || 'Klippy entered shutdown state');
    }
    await wait(pollMs);
  }

  throw new Error(lastMessage || 'Timed out waiting for Klippy to become ready');
}

