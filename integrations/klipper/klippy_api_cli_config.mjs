import net from 'node:net';
import path from 'node:path';
import { spawn } from 'node:child_process';

export const DEFAULT_KLIPPY_API_START_SCRIPT =
  process.env.KLIPPY_API_START_SCRIPT || './scripts/run_klippy_api_mode.sh';
export const DEFAULT_KLIPPY_SOCKET_PATH =
  process.env.KLIPPY_SOCKET_PATH || '/tmp/klippy_uds';
export const DEFAULT_KLIPPY_LOG_PATH =
  process.env.KLIPPY_LOG_PATH || '/tmp/klipper.log';
export const DEFAULT_KLIPPY_CONFIG_PATH =
  process.env.KLIPPY_CONFIG_PATH
  || './examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg';
export const DEFAULT_KLIPPY_PYTHON =
  process.env.KLIPPY_PYTHON || './.venv/bin/python';

const SOCKET_UNAVAILABLE_CODES = new Set([
  'ECONNREFUSED',
  'ECONNRESET',
  'ENOENT',
  'ENOTSOCK',
  'EPERM',
]);

function resolvePathLike(value, cwd) {
  if (!value) {
    return value;
  }
  return path.isAbsolute(value) ? value : path.resolve(cwd, value);
}

export function buildKlippyApiLaunchSpec({
  cwd = process.cwd(),
  startScript = DEFAULT_KLIPPY_API_START_SCRIPT,
  socketPath = DEFAULT_KLIPPY_SOCKET_PATH,
  logPath = DEFAULT_KLIPPY_LOG_PATH,
  configPath = DEFAULT_KLIPPY_CONFIG_PATH,
  python = DEFAULT_KLIPPY_PYTHON,
  shell = '/bin/bash',
} = {}) {
  return {
    command: shell,
    args: [path.resolve(cwd, startScript)],
    options: {
      cwd,
      detached: true,
      stdio: 'ignore',
      env: {
        ...process.env,
        KLIPPY_SOCKET_PATH: resolvePathLike(socketPath, cwd),
        KLIPPY_LOG_PATH: resolvePathLike(logPath, cwd),
        KLIPPY_CONFIG_PATH: resolvePathLike(configPath, cwd),
        KLIPPY_PYTHON: resolvePathLike(python, cwd),
      },
    },
  };
}

export function isKlippySocketUnavailableError(error) {
  if (!error) {
    return false;
  }
  return SOCKET_UNAVAILABLE_CODES.has(error.code)
    || SOCKET_UNAVAILABLE_CODES.has(error.cause?.code);
}

export async function waitForKlippySocket(socketPath, timeoutMs = 7_000) {
  const deadline = Date.now() + Math.max(1, timeoutMs);
  while (Date.now() < deadline) {
    try {
      await new Promise((resolve, reject) => {
        const socket = net.createConnection(socketPath);
        const cleanup = () => {
          socket.removeAllListeners();
        };
        socket.once('connect', () => {
          cleanup();
          socket.end();
          resolve();
        });
        socket.once('error', (error) => {
          cleanup();
          socket.destroy();
          reject(error);
        });
      });
      return;
    } catch (error) {
      if (!isKlippySocketUnavailableError(error)) {
        throw error;
      }
    }
    await new Promise((resolve) => setTimeout(resolve, 150));
  }
  throw new Error(`Klippy API socket at ${socketPath} did not become ready in time.`);
}

export async function ensureKlippyApiServer({
  socketPath = DEFAULT_KLIPPY_SOCKET_PATH,
  cwd = process.cwd(),
  startScript = DEFAULT_KLIPPY_API_START_SCRIPT,
  logPath = DEFAULT_KLIPPY_LOG_PATH,
  configPath = DEFAULT_KLIPPY_CONFIG_PATH,
  python = DEFAULT_KLIPPY_PYTHON,
  shell = '/bin/bash',
  onInfo = null,
  spawnImpl = spawn,
  waitForSocket = waitForKlippySocket,
} = {}) {
  const spec = buildKlippyApiLaunchSpec({
    cwd,
    startScript,
    socketPath,
    logPath,
    configPath,
    python,
    shell,
  });
  if (typeof onInfo === 'function') {
    onInfo(`Klippy is not responding on ${socketPath}. Starting ${startScript}...`);
  }
  const child = spawnImpl(spec.command, spec.args, spec.options);
  if (typeof child?.unref === 'function') {
    child.unref();
  }
  await waitForSocket(socketPath);
  if (typeof onInfo === 'function') {
    onInfo(`Klippy API socket is ready on ${socketPath}.`);
  }
  return child;
}

export function stopKlippyApiServer(child) {
  if (!child || typeof child.pid !== 'number' || child.pid <= 0) {
    return false;
  }
  try {
    process.kill(-child.pid, 'SIGTERM');
    return true;
  } catch (_error) {
    try {
      child.kill('SIGTERM');
      return true;
    } catch (_innerError) {
      return false;
    }
  }
}
