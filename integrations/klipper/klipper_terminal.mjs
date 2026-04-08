#!/usr/bin/env node
import path from 'node:path';
import readline from 'node:readline';
import { fileURLToPath } from 'node:url';
import { KlippyApiClient } from './klippyApiClient.js';
import { parseArgs } from './klipperTerminalArgs.js';
import {
  DEFAULT_KLIPPY_API_START_SCRIPT,
  DEFAULT_KLIPPY_CONFIG_PATH,
  DEFAULT_KLIPPY_LOG_PATH,
  DEFAULT_KLIPPY_SOCKET_PATH,
  ensureKlippyApiServer,
  stopKlippyApiServer,
  waitForKlippySocket,
} from './klippy_api_cli_config.mjs';

const CLIENT_INFO = {
  program: 'klipper_terminal',
  version: 'step1',
};

const PROMPT_CONNECTED = 'gcode> ';
const PROMPT_DISCONNECTED = '\x1b[90mdisconnected>\x1b[0m ';

function printHelp() {
  console.log(`Usage: node integrations/klipper/klipper_terminal.mjs [options]

Send G-code to a Klippy API socket. Step 1 implements process management and
API transport only; WebSocket simulator fan-out flags are accepted but not yet
active.

Options:
  --socket <path>          Klippy API socket path (default: ${DEFAULT_KLIPPY_SOCKET_PATH})
  --config <path>          Klippy printer config for autostart
                           (default: ${DEFAULT_KLIPPY_CONFIG_PATH})
  --log-path <path>        Klippy log path for autostart (default: ${DEFAULT_KLIPPY_LOG_PATH})
  --start-script <path>    Launcher script for autostart
                           (default: ${DEFAULT_KLIPPY_API_START_SCRIPT})
  --ws-port <port>         Reserved for future hp-sim fan-out (default: 8790)
  --no-ws                  Disable future WebSocket fan-out
  --cmd, -c <GCODE>        Send G-code and exit
  --quiet, -q              Only print command results
  --keep-alive             Do not stop a terminal-managed klippy on exit
  --help, -h               Show this help`);
}
export { parseArgs } from './klipperTerminalArgs.js';

function buildRuntime(args) {
  const client = new KlippyApiClient({
    socketPath: args.socketPath,
  });

  let rl = null;
  let promptConnectedState = false;
  let promptEverRendered = false;
  let managedKlippyProcess = null;
  let autoStartInFlight = null;
  let shuttingDown = false;
  let primePromise = null;
  let latestInfo = null;
  let latestObjectsList = null;

  const interactivePromptEnabled = () => rl && process.stdin.isTTY && !args.quiet;

  const updatePromptForConnectionState = (connected, { forcePrompt = false } = {}) => {
    promptConnectedState = connected;
    if (!interactivePromptEnabled()) {
      return;
    }
    const nextPrompt = connected ? PROMPT_CONNECTED : PROMPT_DISCONNECTED;
    if (rl.getPrompt() !== nextPrompt) {
      rl.setPrompt(nextPrompt);
    }
    if (forcePrompt || !promptEverRendered) {
      promptEverRendered = true;
      rl.prompt();
      return;
    }
    rl.prompt(true);
  };

  const promptIfInteractive = () => {
    if (!interactivePromptEnabled()) {
      return;
    }
    promptEverRendered = true;
    rl.prompt();
  };

  const primeConnection = async () => {
    const nextPrime = (async () => {
      latestInfo = await client.request('info', { client_info: CLIENT_INFO });
      latestObjectsList = await client.request('objects/list', {});
      return {
        info: latestInfo,
        objectsList: latestObjectsList,
      };
    })();
    primePromise = nextPrime;
    try {
      return await nextPrime;
    } finally {
      if (primePromise === nextPrime) {
        primePromise = null;
      }
    }
  };

  const waitForPrimedConnection = async () => {
    await client.waitForConnection();
    if (primePromise) {
      return primePromise;
    }
    return primeConnection();
  };

  const waitForKlippyReady = async (timeoutMs = 15_000) => {
    const deadline = Date.now() + Math.max(1, timeoutMs);
    while (Date.now() < deadline) {
      await waitForPrimedConnection();
      if (latestInfo?.state === 'ready') {
        return latestInfo;
      }
      if (latestInfo?.state === 'shutdown' || latestInfo?.state === 'error') {
        throw new Error(latestInfo.state_message || `Klippy is in ${latestInfo.state} state.`);
      }
      await new Promise((resolve) => setTimeout(resolve, 200));
      latestInfo = await client.request('info', {});
    }
    const details = latestInfo?.state_message || latestInfo?.state || 'unknown state';
    throw new Error(`Klippy did not become ready within ${timeoutMs}ms (${details}).`);
  };

  const ensureKlippyServerReady = async ({ forceStart = false } = {}) => {
    if (autoStartInFlight) {
      return autoStartInFlight;
    }
    autoStartInFlight = (async () => {
      if (!forceStart) {
        try {
          await waitForKlippySocket(args.socketPath, 750);
          return managedKlippyProcess;
        } catch (_error) {
          // Fall through to autostart.
        }
      }
      if (!managedKlippyProcess) {
        managedKlippyProcess = await ensureKlippyApiServer({
          socketPath: args.socketPath,
          configPath: args.configPath,
          logPath: args.logPath,
          startScript: args.startScript,
          onInfo: args.quiet ? null : (message) => console.log(message),
        });
        return managedKlippyProcess;
      }
      await waitForKlippySocket(args.socketPath);
      return managedKlippyProcess;
    })();
    try {
      return await autoStartInFlight;
    } finally {
      autoStartInFlight = null;
    }
  };

  const sendGcodeLine = async (line) => {
    const trimmed = line?.trim?.();
    if (!trimmed) {
      promptIfInteractive();
      return true;
    }
    if (!args.quiet) {
      console.log(`> ${trimmed}`);
    }
    try {
      await waitForPrimedConnection();
      await waitForKlippyReady();
      await client.request('gcode/script', { script: trimmed });
      console.log('ok');
      return true;
    } catch (error) {
      console.error(`Error sending "${trimmed}": ${error.message}`);
      return false;
    } finally {
      promptIfInteractive();
    }
  };

  const shutdown = (exitCode = 0) => {
    if (shuttingDown) {
      return;
    }
    shuttingDown = true;
    try {
      client.close();
    } catch (_error) {
      // Ignore shutdown errors.
    }
    if (managedKlippyProcess && !args.keepAlive) {
      stopKlippyApiServer(managedKlippyProcess);
      managedKlippyProcess = null;
    }
    process.exit(exitCode);
  };

  client.on('connected', () => {
    updatePromptForConnectionState(true);
    primeConnection().catch((error) => {
      if (!shuttingDown) {
        console.error(`Failed to query Klippy state: ${error.message}`);
      }
    });
  });

  client.on('disconnected', () => {
    updatePromptForConnectionState(false);
  });

  client.on('socket-error', (error) => {
    if (shuttingDown || args.quiet) {
      return;
    }
    if (error?.code === 'ENOENT' || error?.code === 'ECONNREFUSED') {
      return;
    }
    console.error(`Klippy socket error: ${error.message}`);
  });

  const main = async () => {
    await ensureKlippyServerReady();
    client.start();
    await waitForPrimedConnection();

    if (!args.noWs && args.wsPort > 0 && !args.quiet) {
      console.log('WebSocket fan-out is reserved for a later klipper_terminal step; continuing in API-only mode.');
    }

    if (args.command) {
      const success = await sendGcodeLine(args.command);
      shutdown(success ? 0 : 1);
      return;
    }

    rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
      terminal: process.stdin.isTTY,
    });

    if (process.stdin.isTTY && !args.quiet) {
      updatePromptForConnectionState(promptConnectedState, { forcePrompt: true });
    }

    rl.on('line', (line) => {
      sendGcodeLine(line).catch((error) => {
        console.error(`Unexpected terminal error: ${error.message}`);
      });
    });

    rl.on('close', () => {
      shutdown(0);
    });

    if (!args.quiet) {
      const objectCount = latestObjectsList?.objects?.length ?? 0;
      const state = latestInfo?.state || 'unknown';
      console.log(`Connected to Klippy on ${args.socketPath} (${state}, ${objectCount} objects).`);
    }
  };

  return {
    client,
    main,
    shutdown,
  };
}

export async function main(argv = process.argv.slice(2)) {
  const args = parseArgs(argv);
  if (args.help) {
    printHelp();
    return 0;
  }
  const runtime = buildRuntime(args);
  activeRuntime = runtime;
  await runtime.main();
  return 0;
}

const entryPath = process.argv[1] ? path.resolve(process.argv[1]) : null;
const modulePath = fileURLToPath(import.meta.url);
let activeRuntime = null;

if (entryPath && entryPath === modulePath) {
  main().catch((error) => {
    console.error(`Failed to start klipper_terminal: ${error.message}`);
    process.exit(1);
  });
}

function handleSignal(exitCode) {
  if (activeRuntime) {
    activeRuntime.shutdown(exitCode);
    return;
  }
  process.exit(exitCode);
}

process.on('SIGINT', () => handleSignal(130));
process.on('SIGTERM', () => handleSignal(143));
process.on('SIGHUP', () => handleSignal(129));

process.on('SIGINT', () => process.exit(130));
process.on('SIGTERM', () => process.exit(143));
process.on('SIGHUP', () => process.exit(129));
