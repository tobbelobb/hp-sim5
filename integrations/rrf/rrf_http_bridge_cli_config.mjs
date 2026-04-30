import path from 'node:path';
import { spawn } from 'node:child_process';

export const DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT =
  process.env.RRF_HTTP_BRIDGE_START_SCRIPT || './scripts/rrf_server.sh';
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
  startScriptArgs = [],
  shell = '/bin/bash',
} = {}) {
  return {
    command: shell,
    args: [path.resolve(cwd, startScript), ...startScriptArgs],
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

export async function waitForRrfSimulator(baseUrl, timeoutMs = 7000) {
  const endpoint = `${baseUrl.replace(/\/$/, '')}/machine/code`;
  const deadline = Date.now() + Math.max(1, timeoutMs);
  while (Date.now() < deadline) {
    try {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), 1500);
      const res = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain' },
        body: 'M115',
        signal: controller.signal,
      });
      clearTimeout(timer);
      if (res.ok) {
        await res.text();
        return;
      }
    } catch (_err) {
      /* try again */
    }
    await new Promise((resolve) => setTimeout(resolve, 250));
  }
  throw new Error(`rrf_simulator at ${baseUrl} did not become ready in time`);
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
  startScriptArgs = [],
  shell = '/bin/bash',
  onInfo = null,
  spawnImpl = spawn,
  waitForServer = waitForRrfSimulator,
} = {}) {
  if (!serverUrl) {
    throw new Error('Missing server URL for rrf_http_bridge autostart.');
  }
  const spec = buildRrfHttpBridgeLaunchSpec({ cwd, startScript, startScriptArgs, shell });
  if (typeof onInfo === 'function') {
    const renderedArgs = startScriptArgs.length > 0 ? ` ${startScriptArgs.join(' ')}` : '';
    onInfo(`rrf_simulator is not responding at ${serverUrl}. Starting ${startScript}${renderedArgs}...`);
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

export function stopRrfHttpBridgeServer(child) {
  if (!child || typeof child.pid !== 'number' || child.pid <= 0) {
    return false;
  }
  try {
    process.kill(-child.pid, 'SIGTERM');
    return true;
  } catch (_err) {
    try {
      child.kill('SIGTERM');
      return true;
    } catch (_innerErr) {
      return false;
    }
  }
}
