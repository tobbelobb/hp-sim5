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
  || './public/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg';
export const DEFAULT_KLIPPY_PYTHON =
  process.env.KLIPPY_PYTHON || './.venv/bin/python';
export const DEFAULT_KLIPPY_HOST_MCU_BIN =
  process.env.KLIPPY_HOST_MCU_BIN || './public/klipper/linux_mcu/klipper.elf';
export const DEFAULT_KLIPPY_HOST_MCU_SERIAL =
  process.env.KLIPPY_HOST_MCU_SERIAL || '/tmp/klipper_host_mcu';
export const DEFAULT_KLIPPY_HOST_MCU_REALTIME =
  process.env.KLIPPY_HOST_MCU_REALTIME || '0';
export const DEFAULT_KLIPPY_MOTION_QUEUE_SG_LOW_TIME =
  process.env.KLIPPY_MOTION_QUEUE_SG_LOW_TIME || null;
export const DEFAULT_KLIPPY_MOTION_QUEUE_SG_HIGH_TIME =
  process.env.KLIPPY_MOTION_QUEUE_SG_HIGH_TIME || null;

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

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function tokenizeCommandLine(commandLine) {
  return String(commandLine)
    .trim()
    .split(/\s+/)
    .filter(Boolean);
}

function hasArgPair(argv, flag, expectedValue) {
  for (let i = 0; i < argv.length - 1; i += 1) {
    if (argv[i] === flag && argv[i + 1] === expectedValue) {
      return true;
    }
  }
  return false;
}

async function listProcesses({ spawnImpl = spawn } = {}) {
  return new Promise((resolve, reject) => {
    const child = spawnImpl('ps', ['-eo', 'pid=,args='], {
      stdio: ['ignore', 'pipe', 'ignore'],
    });
    let stdout = '';
    child.once('error', reject);
    child.stdout.on('data', (chunk) => {
      stdout += chunk.toString();
    });
    child.once('close', (code) => {
      if (code !== 0) {
        reject(new Error(`ps exited with code ${code}`));
        return;
      }
      const processes = stdout
        .split(/\r?\n/)
        .map((line) => line.trim())
        .filter(Boolean)
        .map((line) => {
          const firstSpace = line.indexOf(' ');
          if (firstSpace === -1) {
            return null;
          }
          const pid = Number.parseInt(line.slice(0, firstSpace), 10);
          const commandLine = line.slice(firstSpace + 1).trim();
          if (!Number.isFinite(pid) || !commandLine) {
            return null;
          }
          return {
            pid,
            commandLine,
            argv: tokenizeCommandLine(commandLine),
          };
        })
        .filter(Boolean);
      resolve(processes);
    });
  });
}

export async function findKlippyApiProcess({
  socketPath = DEFAULT_KLIPPY_SOCKET_PATH,
  cwd = process.cwd(),
  listProcessesImpl = listProcesses,
} = {}) {
  const resolvedSocketPath = resolvePathLike(socketPath, cwd);
  const processes = await listProcessesImpl();
  return processes.find((proc) => (
    proc.argv.some((arg) => arg.endsWith('/klippy.py') || arg === 'klippy.py')
    && hasArgPair(proc.argv, '-a', resolvedSocketPath)
  )) || null;
}

export async function terminateKlippyApiProcess({
  socketPath = DEFAULT_KLIPPY_SOCKET_PATH,
  cwd = process.cwd(),
  signal = 'SIGTERM',
  timeoutMs = 5_000,
  pollMs = 100,
  listProcessesImpl = listProcesses,
  killImpl = process.kill,
} = {}) {
  const target = await findKlippyApiProcess({
    socketPath,
    cwd,
    listProcessesImpl,
  });
  if (!target) {
    return false;
  }
  killImpl(target.pid, signal);
  const deadline = Date.now() + Math.max(1, timeoutMs);
  while (Date.now() < deadline) {
    const active = await findKlippyApiProcess({
      socketPath,
      cwd,
      listProcessesImpl,
    });
    if (!active || active.pid !== target.pid) {
      return true;
    }
    await sleep(pollMs);
  }
  throw new Error(`Timed out waiting for stale Klippy process ${target.pid} to exit.`);
}

export function buildKlippyApiLaunchSpec({
  cwd = process.cwd(),
  startScript = DEFAULT_KLIPPY_API_START_SCRIPT,
  socketPath = DEFAULT_KLIPPY_SOCKET_PATH,
  logPath = DEFAULT_KLIPPY_LOG_PATH,
  configPath = DEFAULT_KLIPPY_CONFIG_PATH,
  python = DEFAULT_KLIPPY_PYTHON,
  hostMcuBin = DEFAULT_KLIPPY_HOST_MCU_BIN,
  hostMcuSerial = DEFAULT_KLIPPY_HOST_MCU_SERIAL,
  hostMcuRealtime = DEFAULT_KLIPPY_HOST_MCU_REALTIME,
  motionQueueStepGenLowTime = DEFAULT_KLIPPY_MOTION_QUEUE_SG_LOW_TIME,
  motionQueueStepGenHighTime = DEFAULT_KLIPPY_MOTION_QUEUE_SG_HIGH_TIME,
  shell = '/bin/bash',
} = {}) {
  const env = {
    ...process.env,
    KLIPPY_SOCKET_PATH: resolvePathLike(socketPath, cwd),
    KLIPPY_LOG_PATH: resolvePathLike(logPath, cwd),
    KLIPPY_CONFIG_PATH: resolvePathLike(configPath, cwd),
    KLIPPY_PYTHON: resolvePathLike(python, cwd),
    KLIPPY_HOST_MCU_BIN: resolvePathLike(hostMcuBin, cwd),
    KLIPPY_HOST_MCU_SERIAL: resolvePathLike(hostMcuSerial, cwd),
    KLIPPY_HOST_MCU_REALTIME: String(hostMcuRealtime),
  };
  if (motionQueueStepGenLowTime !== null && motionQueueStepGenLowTime !== undefined) {
    env.KLIPPY_MOTION_QUEUE_SG_LOW_TIME = String(motionQueueStepGenLowTime);
  }
  if (motionQueueStepGenHighTime !== null && motionQueueStepGenHighTime !== undefined) {
    env.KLIPPY_MOTION_QUEUE_SG_HIGH_TIME = String(motionQueueStepGenHighTime);
  }
  return {
    command: shell,
    args: [path.resolve(cwd, startScript)],
    options: {
      cwd,
      detached: true,
      stdio: 'ignore',
      env,
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
  hostMcuBin = DEFAULT_KLIPPY_HOST_MCU_BIN,
  hostMcuSerial = DEFAULT_KLIPPY_HOST_MCU_SERIAL,
  hostMcuRealtime = DEFAULT_KLIPPY_HOST_MCU_REALTIME,
  motionQueueStepGenLowTime = DEFAULT_KLIPPY_MOTION_QUEUE_SG_LOW_TIME,
  motionQueueStepGenHighTime = DEFAULT_KLIPPY_MOTION_QUEUE_SG_HIGH_TIME,
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
    hostMcuBin,
    hostMcuSerial,
    hostMcuRealtime,
    motionQueueStepGenLowTime,
    motionQueueStepGenHighTime,
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
