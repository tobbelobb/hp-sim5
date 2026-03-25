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

export function createJsonBroadcaster() {
  const clients = new Set();

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
    for (const socket of clients) {
      safeSend(socket, data);
    }
  };

  const close = () => {
    clients.clear();
  };

  return {
    register,
    broadcast,
    close,
    clients,
  };
}
