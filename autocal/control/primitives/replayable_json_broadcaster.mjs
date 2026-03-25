const DEFAULT_HISTORY_LIMIT = 5000;
const RESET_MESSAGE = JSON.stringify({ type: 'reset' });

function isSocketOpen(socket) {
  if (!socket || typeof socket.send !== 'function') {
    return false;
  }
  if (typeof socket.readyState !== 'number') {
    return true;
  }
  return socket.readyState === 1;
}

function safeSend(socket, data) {
  if (!isSocketOpen(socket)) {
    return false;
  }
  try {
    socket.send(data);
    return true;
  } catch (_err) {
    return false;
  }
}

export function createReplayableJsonBroadcaster({ maxHistory = DEFAULT_HISTORY_LIMIT } = {}) {
  const clients = new Set();
  const history = [];

  const pruneHistory = () => {
    const overflow = history.length - Math.max(0, maxHistory);
    if (overflow > 0) {
      history.splice(0, overflow);
    }
  };

  const replayHistory = (socket) => {
    safeSend(socket, RESET_MESSAGE);
    for (const payload of history) {
      safeSend(socket, payload);
    }
  };

  const register = (socket) => {
    if (!socket || typeof socket.on !== 'function') {
      return;
    }
    clients.add(socket);
    const onClose = () => {
      clients.delete(socket);
      if (typeof socket.off === 'function') {
        socket.off('close', onClose);
      } else if (typeof socket.removeListener === 'function') {
        socket.removeListener('close', onClose);
      }
    };
    socket.on('close', onClose);
    replayHistory(socket);
  };

  const broadcast = (payload) => {
    if (!payload) {
      return;
    }
    let data = null;
    try {
      data = JSON.stringify(payload);
    } catch (_err) {
      return;
    }
    history.push(data);
    pruneHistory();
    for (const socket of clients) {
      safeSend(socket, data);
    }
  };

  const close = () => {
    clients.clear();
    history.length = 0;
  };

  return {
    register,
    broadcast,
    close,
    clients,
    history,
  };
}

