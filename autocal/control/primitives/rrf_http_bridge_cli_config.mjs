import path from 'node:path';
import { spawn } from 'node:child_process';
import { waitForRrfSimulator } from './encoder_utils.mjs';

export const DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT =
  process.env.RRF_HTTP_BRIDGE_START_SCRIPT || './scripts/rrf_server_hp3_w_buildup.sh';
export const RRF_HTTP_BRIDGE_WS_QUERY_PARAM = 'gcode_ws';

const SERVER_UNAVAILABLE_CODES = new Set([
  'ECONNREFUSED',
  'ECONNRESET',
  'EHOSTUNREACH',
  'ENETUNREACH',
  'EPERM',
]);

export function buildRrfHttpBridgeLaunchSpec({
  cwd = process.cwd(),
  startScript = DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT,
  shell = '/bin/bash',
} = {}) {
  return {
    command: shell,
    args: [path.resolve(cwd, startScript)],
    options: {
      cwd,
      detached: true,
      stdio: 'ignore',
    },
  };
}

export function buildRrfHttpBridgeWsHint(wsPort, { host = 'localhost' } = {}) {
  return `?${RRF_HTTP_BRIDGE_WS_QUERY_PARAM}=ws://${host}:${wsPort}`;
}

export function isRrfServerUnavailableError(error) {
  if (!error) {
    return false;
  }
  const message = typeof error.message === 'string' ? error.message : String(error);
  if (/fetch failed/i.test(message)) {
    return true;
  }
  const code = error.code || error.cause?.code || null;
  return SERVER_UNAVAILABLE_CODES.has(code);
}

export async function ensureRrfHttpBridgeServer({
  serverUrl,
  cwd = process.cwd(),
  startScript = DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT,
  shell = '/bin/bash',
  onInfo = null,
  spawnImpl = spawn,
  waitForServer = waitForRrfSimulator,
} = {}) {
  if (!serverUrl) {
    throw new Error('Missing server URL for rrf_http_bridge autostart.');
  }
  const spec = buildRrfHttpBridgeLaunchSpec({ cwd, startScript, shell });
  if (typeof onInfo === 'function') {
    onInfo(`rrf_simulator is not responding at ${serverUrl}. Starting ${startScript}...`);
  }
  const child = spawnImpl(spec.command, spec.args, spec.options);
  if (typeof child?.unref === 'function') {
    child.unref();
  }
  await waitForServer(serverUrl);
  if (typeof onInfo === 'function') {
    onInfo(`rrf_simulator is ready at ${serverUrl}.`);
  }
  return child;
}
