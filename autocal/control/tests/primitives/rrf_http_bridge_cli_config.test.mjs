import http from 'node:http';
import {
  buildRrfHttpBridgeWsHint,
  waitForRrfSimulator,
} from '../../../../bridges/rrf/http/rrf_http_bridge_cli_config.mjs';

describe('rrf_http_bridge_cli_config', () => {
  test('buildRrfHttpBridgeWsHint formats the websocket query parameter', () => {
    expect(buildRrfHttpBridgeWsHint(8790)).toBe('?gcode_ws=ws://localhost:8790');
    expect(buildRrfHttpBridgeWsHint(9001, { host: '127.0.0.1' })).toBe('?gcode_ws=ws://127.0.0.1:9001');
  });

  test('waitForRrfSimulator probes POST /machine/code with M115', async () => {
    let seenMethod = null;
    let seenUrl = null;
    let seenBody = null;
    const server = http.createServer((req, res) => {
      const chunks = [];
      req.on('data', (chunk) => chunks.push(chunk));
      req.on('end', () => {
        seenMethod = req.method;
        seenUrl = req.url;
        seenBody = Buffer.concat(chunks).toString('utf8');
        res.writeHead(200, { 'Content-Type': 'text/plain' });
        res.end('ok');
      });
    });

    await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
    const address = server.address();
    const baseUrl = `http://127.0.0.1:${address.port}`;

    try {
      await waitForRrfSimulator(baseUrl, 1000);
      expect(seenMethod).toBe('POST');
      expect(seenUrl).toBe('/machine/code');
      expect(seenBody.trim()).toBe('M115');
    } finally {
      await new Promise((resolve) => server.close(resolve));
    }
  });
});
