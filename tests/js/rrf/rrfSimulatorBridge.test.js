import net from 'node:net';
import { WebSocket } from 'ws';

import { createGcodeBridge } from '../../../integrations/rrf/rrfSimulatorBridge.mjs';

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function getFreePort() {
  return new Promise((resolve, reject) => {
    const server = net.createServer();
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      const port = typeof address === 'object' && address ? address.port : null;
      server.close((err) => {
        if (err) {
          reject(err);
          return;
        }
        resolve(port);
      });
    });
    server.on('error', reject);
  });
}

async function waitFor(predicate, timeoutMs = 1000) {
  const startMs = Date.now();
  // eslint-disable-next-line no-constant-condition
  while (true) {
    if (predicate()) {
      return;
    }
    if (Date.now() - startMs > timeoutMs) {
      throw new Error(`Timed out after ${timeoutMs}ms`);
    }
    // eslint-disable-next-line no-await-in-loop
    await sleep(10);
  }
}

async function waitForOpen(socket) {
  if (socket.readyState === WebSocket.OPEN) {
    return;
  }
  await new Promise((resolve, reject) => {
    socket.once('open', resolve);
    socket.once('error', reject);
  });
}

async function closeSocket(socket) {
  if (!socket || socket.readyState === WebSocket.CLOSED) {
    return;
  }
  await new Promise((resolve) => {
    socket.once('close', resolve);
    socket.close();
  });
}

function createEncoderClient(url, anglesDeg) {
  const socket = new WebSocket(url);
  const requests = [];
  socket.on('message', (data) => {
    const payload = JSON.parse(data.toString());
    if (payload?.type !== 'encoder_request') {
      return;
    }
    requests.push(payload);
    socket.send(JSON.stringify({
      type: 'encoder_response',
      requestId: payload.requestId,
      anglesDeg,
    }));
  });
  return {
    socket,
    requests,
  };
}

describe('createGcodeBridge encoder websocket routing', () => {
  test('sends encoder requests only to the most recently connected client', async () => {
    const wsPort = await getFreePort();
    const bridgeContext = createGcodeBridge({
      server: 'http://127.0.0.1:9',
      wsPort,
      quiet: true,
    });
    const wsUrl = `ws://127.0.0.1:${wsPort}`;
    const firstClient = createEncoderClient(wsUrl, [11]);
    const secondClient = createEncoderClient(wsUrl, [22]);

    try {
      await waitForOpen(firstClient.socket);
      await waitForOpen(secondClient.socket);
      await waitFor(() => bridgeContext.getReadyWsClients().length === 2);

      const response = await bridgeContext.sendEncoderRequest(['X'], 500);

      expect(response.anglesDeg).toEqual([22]);
      expect(firstClient.requests).toHaveLength(0);
      expect(secondClient.requests).toHaveLength(1);
    } finally {
      await closeSocket(firstClient.socket);
      await closeSocket(secondClient.socket);
      bridgeContext.close();
    }
  });

  test('falls back to an older ready client when the preferred client disconnects', async () => {
    const wsPort = await getFreePort();
    const bridgeContext = createGcodeBridge({
      server: 'http://127.0.0.1:9',
      wsPort,
      quiet: true,
    });
    const wsUrl = `ws://127.0.0.1:${wsPort}`;
    const firstClient = createEncoderClient(wsUrl, [11]);
    const secondClient = createEncoderClient(wsUrl, [22]);

    try {
      await waitForOpen(firstClient.socket);
      await waitForOpen(secondClient.socket);
      await waitFor(() => bridgeContext.getReadyWsClients().length === 2);

      await closeSocket(secondClient.socket);
      await waitFor(() => bridgeContext.getReadyWsClients().length === 1);

      const response = await bridgeContext.sendEncoderRequest(['X'], 500);

      expect(response.anglesDeg).toEqual([11]);
      expect(firstClient.requests).toHaveLength(1);
      expect(secondClient.requests).toHaveLength(0);
    } finally {
      await closeSocket(firstClient.socket);
      await closeSocket(secondClient.socket);
      bridgeContext.close();
    }
  });
});
