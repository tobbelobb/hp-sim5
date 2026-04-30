import http from 'node:http';
import {
  buildRrfHttpBridgeLaunchSpec,
  buildRrfHttpBridgeWsHint,
  waitForRrfSimulator,
} from '../../../../integrations/rrf/rrf_http_bridge_cli_config.mjs';
import { parseArgs } from '../../../../integrations/rrf/rrf_terminal.mjs';

describe('rrf_http_bridge_cli_config', () => {
  test('buildRrfHttpBridgeWsHint formats the websocket query parameter', () => {
    expect(buildRrfHttpBridgeWsHint(8790)).toBe('?gcode_ws=ws://localhost:8790');
    expect(buildRrfHttpBridgeWsHint(9001, { host: '127.0.0.1' })).toBe('?gcode_ws=ws://127.0.0.1:9001');
  });

  test('buildRrfHttpBridgeLaunchSpec forwards server script arguments', () => {
    const spec = buildRrfHttpBridgeLaunchSpec({
      cwd: '/repo',
      startScript: './scripts/rrf_server.sh',
      startScriptArgs: ['-c', 'config_hp4.g'],
    });

    expect(spec.args).toEqual(['/repo/scripts/rrf_server.sh', '-c', 'config_hp4.g']);
  });

  test('rrf_terminal parses RRF autostart config options', () => {
    expect(parseArgs(['-c', 'config_hp4.g']).startScriptArgs).toEqual(['-c', 'config_hp4.g']);
    expect(parseArgs(['--config=sys/config_hp5.g']).startScriptArgs).toEqual(['-c', 'sys/config_hp5.g']);
  });

  test('rrf_terminal parses RRF autostart machine options independent of order', () => {
    expect(parseArgs(['--line-layers', '-m', 'skycam']).startScriptArgs).toEqual([
      '-m',
      'skycam',
      '--line-layers',
    ]);
    expect(parseArgs(['--machineType=hangprinter_3', '--no-line-layers']).startScriptArgs).toEqual([
      '-m',
      'hangprinter_3',
      '--no-line-layers',
    ]);
  });

  test('rrf_terminal leaves custom start script args empty unless autostart options are explicit', () => {
    const oldValue = process.env.RRF_HTTP_BRIDGE_START_SCRIPT;
    process.env.RRF_HTTP_BRIDGE_START_SCRIPT = './scripts/custom_rrf.sh';
    try {
      expect(parseArgs([]).startScriptArgs).toEqual([]);
      expect(parseArgs(['-m', 'hp4', '--buildup']).startScriptArgs).toEqual([
        '-m',
        'hp4',
        '--buildup',
      ]);
    } finally {
      if (oldValue === undefined) {
        delete process.env.RRF_HTTP_BRIDGE_START_SCRIPT;
      } else {
        process.env.RRF_HTTP_BRIDGE_START_SCRIPT = oldValue;
      }
    }
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
