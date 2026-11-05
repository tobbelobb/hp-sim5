import net from 'node:net';
import http from 'node:http';
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
  test('simulation-clock refresh tolerates a browser frame longer than two seconds', async () => {
    const wsPort = await getFreePort();
    const context = createGcodeBridge({ wsPort, sim: true, quiet: true });
    const socket = new WebSocket(`ws://127.0.0.1:${wsPort}`);
    socket.on('message', data => {
      const request = JSON.parse(data.toString());
      if (request.type !== 'encoder_request' || request.axes.length) return;
      setTimeout(() => socket.send(JSON.stringify({
        type: 'encoder_response', requestId: request.requestId,
        simulationClock: { time_ms: 123, generation: 0 },
      })), 2100);
    });
    try {
      await waitForOpen(socket);
      await context.simulationClock.refresh();
      expect(context.simulationClock.now()).toBe(123);
      await expect(context.sendEncoderRequest(['A'], 10)).rejects.toThrow('encoder request (A) within 10 ms');
    } finally {
      await closeSocket(socket);
      context.close();
    }
  });

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

// Exercise the production HTTP-to-WebSocket encoder path with an HTTP firmware stub.
async function withCommandFixture(options, onRequest, check) {
  const firmware = http.createServer((_request, response) => response.end('ok'));
  await new Promise(resolve => firmware.listen(0, '127.0.0.1', resolve));
  const wsPort = await getFreePort();
  const context = createGcodeBridge({ server: `http://127.0.0.1:${firmware.address().port}`, wsPort, quiet: true, ...options });
  const socket = new WebSocket(`ws://127.0.0.1:${wsPort}`);
  const timers = [];
  const later = (callback, ms) => timers.push(setTimeout(callback, ms));
  socket.on('message', data => {
    const request = JSON.parse(data.toString());
    if (request.type === 'encoder_request') onRequest(request, socket, later);
  });
  try {
    await waitForOpen(socket);
    await check(context, socket);
  } finally {
    timers.forEach(clearTimeout);
    await closeSocket(socket);
    context.close();
    await new Promise(resolve => firmware.close(resolve));
  }
}

const encoderCommand = 'M569.3 P40.0:41.0:42.0:43.0';
const finiteEncoderValues = [12.34, -56.78, 90.12, -34.56];

describe('M569.3 encoder reply budgets', () => {
  test('honors an explicit long budget for a valid reply after six seconds', async () => {
    await withCommandFixture({ encoderTimeoutMs: 120000 }, (request, socket, later) => {
      later(() => socket.send(JSON.stringify({ type: 'encoder_response', requestId: request.requestId, anglesDeg: finiteEncoderValues })), 6000);
    }, async context => {
      const result = await context.sendGcodeLine(encoderCommand, { timeout: 120000 });
      expect(result.reply).toBe('[12.34, -56.78, 90.12, -34.56, ]');
    });
  }, 15000);

  test('retains the effective five-second command default when no encoder budget is supplied', async () => {
    await withCommandFixture({}, () => {}, async context => {
      const started = Date.now();
      const result = await context.sendGcodeLine(encoderCommand, { timeout: 120000 });
      expect(result.reply).toMatch(/within 5000 ms/);
      expect(Date.now() - started).toBeGreaterThanOrEqual(4800);
    });
  }, 12000);

  test('a shorter command deadline bounds an explicit long encoder budget', async () => {
    await withCommandFixture({ encoderTimeoutMs: 120000 }, () => {}, async context => {
      const started = Date.now();
      const result = await context.sendGcodeLine(encoderCommand, { timeout: 80 });
      expect(result.reply).toMatch(/within 80 ms/);
      expect(Date.now() - started).toBeLessThan(2000);
    });
  });

  test('a missing reply fails within the configured short encoder budget', async () => {
    await withCommandFixture({ encoderTimeoutMs: 60 }, () => {}, async context => {
      const result = await context.sendGcodeLine(encoderCommand, { timeout: 1000 });
      expect(result.reply).toMatch(/within 60 ms/);
    });
  });

  test('a pending request remains bounded when its client disconnects', async () => {
    await withCommandFixture({ encoderTimeoutMs: 80 }, (_request, socket, later) => {
      later(() => socket.close(), 20);
    }, async context => {
      const result = await context.sendGcodeLine(encoderCommand, { timeout: 120000 });
      expect(result.reply).toMatch(/within 80 ms/);
    });
  });

  test('a disconnected simulator still fails immediately', async () => {
    await withCommandFixture({ encoderTimeoutMs: 120000 }, () => {}, async (context, socket) => {
      await closeSocket(socket);
      await waitFor(() => context.getReadyWsClients().length === 0);
      const result = await context.sendGcodeLine(encoderCommand, { timeout: 120000 });
      expect(result.reply).toMatch(/disconnected; no WebSocket client/);
    });
  });

  test('concurrent replies route by request ID and ignore an unknown ID', async () => {
    const requests = [];
    await withCommandFixture({}, request => requests.push(request), async (context, socket) => {
      let firstResolved = false;
      const first = context.sendEncoderRequest(['A'], 1000).then(result => { firstResolved = true; return result; });
      const second = context.sendEncoderRequest(['B'], 1000);
      await waitFor(() => requests.length === 2);
      expect(requests[0].requestId).not.toBe(requests[1].requestId);
      socket.send(JSON.stringify({ type: 'encoder_response', requestId: 999999, anglesDeg: [99] }));
      socket.send(JSON.stringify({ type: 'encoder_response', requestId: requests[1].requestId, anglesDeg: [22] }));
      expect((await second).anglesDeg).toEqual([22]);
      expect(firstResolved).toBe(false);
      socket.send(JSON.stringify({ type: 'encoder_response', requestId: requests[0].requestId, anglesDeg: [11] }));
      expect((await first).anglesDeg).toEqual([11]);
    });
  });
});
