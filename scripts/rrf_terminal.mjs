#!/usr/bin/env node
import readline from 'node:readline';
import { createGcodeBridge } from '../bridges/rrf/gcode_to_rrf_simulator_to_websocket.mjs';
import { waitForRrfSimulator } from '../autocal/control/primitives/encoder_utils.mjs';
import {
  DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT,
  buildRrfHttpBridgeWsHint,
  ensureRrfHttpBridgeServer,
  isRrfServerUnavailableError,
  stopRrfHttpBridgeServer,
} from '../bridges/rrf/http/rrf_http_bridge_cli_config.mjs';

function printHelp() {
  console.log(`Usage: node scripts/rrf_command_prompt.mjs [options]

Send single-line G-code to an rrf_simulator HTTP server and stream the resulting
motion commands over WebSocket for hp-sim to visualize.

Options:
  --server, --rrf <url>    Base URL of rrf_simulator (default: http://localhost:8080)
  --ws-port <port>         Port for the WebSocket fan-out (0 to disable, default: 8790)
  --no-ws                  Disable WebSocket fan-out entirely
  --cmd, -c <GCODE>        Send one G-code line and exit
  --quiet, -q              Only print replies (suppress prompts and extra logs)
  Auto-start launcher      ${DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT}
  --help, -h               Show this help`);
}

export function parseArgs(argv) {
  const envServer = process.env.RRF_SERVER_URL;
  const args = {
    server: envServer || 'http://localhost:8080',
    wsPort: 8790,
    noWs: false,
    command: null,
    quiet: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--server' || arg === '--rrf') {
      args.server = argv[++i] || args.server;
      args.serverExplicit = true;
    } else if (arg === '--ws-port') {
      const value = parseInt(argv[++i], 10);
      if (Number.isFinite(value) && value > 0) {
        args.wsPort = value;
      } else {
        args.wsPort = 0;
      }
    } else if (arg === '--cmd' || arg === '-c') {
      args.command = argv[++i] || null;
    } else if (arg === '--no-ws') {
      args.noWs = true;
    } else if (arg === '--quiet' || arg === '-q') {
      args.quiet = true;
    } else if (arg === '--help' || arg === '-h') {
      args.help = true;
    }
  }

  return args;
}

const args = parseArgs(process.argv.slice(2));
if (args.help) {
  printHelp();
  process.exit(0);
}

const bridgeContext = createGcodeBridge({
  server: args.server,
  wsPort: args.noWs ? 0 : args.wsPort,
  quiet: args.quiet,
  onClientChange: (connected) => updatePromptForConnectionState(connected),
});

const PROMPT_CONNECTED = 'gcode> ';
const PROMPT_DISCONNECTED = '\x1b[90mdisconnected>\x1b[0m ';
let promptConnectedState = !bridgeContext.wss;
let promptEverRendered = false;

let managedRrfServer = null;
let autoStartInFlight = null;
let shuttingDown = false;

const sendQueue = [];
let processingQueue = false;
let rl = null;

const interactivePromptEnabled = () => rl && process.stdin.isTTY && !args.quiet;

function updatePromptForConnectionState(connected, { forcePrompt = false } = {}) {
  promptConnectedState = connected;
  if (!interactivePromptEnabled()) {
    return;
  }
  const targetPrompt = connected ? PROMPT_CONNECTED : PROMPT_DISCONNECTED;
  if (rl.getPrompt() !== targetPrompt) {
    rl.setPrompt(targetPrompt);
  }
  if (forcePrompt || !promptEverRendered) {
    promptEverRendered = true;
    rl.prompt();
  } else {
    rl.prompt(true);
  }
}

const promptIfInteractive = () => {
  if (interactivePromptEnabled()) {
    promptEverRendered = true;
    rl.prompt();
  }
};

async function ensureRrfServerReady({ forceStart = false } = {}) {
  if (autoStartInFlight) {
    return autoStartInFlight;
  }
  autoStartInFlight = (async () => {
    if (!forceStart) {
      try {
        await waitForRrfSimulator(args.server, 600);
        return managedRrfServer;
      } catch (_err) {
        // Fall through to autostart.
      }
    }
    if (!managedRrfServer) {
      managedRrfServer = await ensureRrfHttpBridgeServer({
        serverUrl: args.server,
        onInfo: args.quiet ? null : (message) => console.log(message),
      });
      return managedRrfServer;
    }
    await waitForRrfSimulator(args.server);
    return managedRrfServer;
  })();
  try {
    return await autoStartInFlight;
  } finally {
    autoStartInFlight = null;
  }
}

async function processQueue() {
  if (processingQueue) {
    return;
  }
  processingQueue = true;
  while (sendQueue.length > 0) {
    const next = sendQueue.shift();
    // eslint-disable-next-line no-await-in-loop
    await next();
  }
  processingQueue = false;
}

async function handleGcodeLine(line) {
  const trimmed = line.trim();
  if (trimmed.length === 0) {
    promptIfInteractive();
    return;
  }
  if (!args.quiet) {
    console.log(`> ${trimmed}`);
  }
  try {
    let result;
    try {
      result = await bridgeContext.sendGcodeLine(trimmed);
    } catch (err) {
      if (!isRrfServerUnavailableError(err)) {
        throw err;
      }
      await ensureRrfServerReady({ forceStart: true });
      result = await bridgeContext.sendGcodeLine(trimmed);
    }
    console.log(result.reply.trim());
  } catch (err) {
    console.error(`Error sending "${trimmed}": ${err.message}`);
  } finally {
    promptIfInteractive();
  }
}

function enqueueLine(line) {
  sendQueue.push(() => handleGcodeLine(line));
  processQueue().catch((err) => {
    console.error('Unexpected error while processing queue:', err);
  });
}

async function runOneShot() {
  await ensureRrfServerReady();
  await bridgeContext.waitForHpSimConnection();
  await handleGcodeLine(args.command);
  shutdown(0);
}

function shutdown(exitCode = 0) {
  if (shuttingDown) {
    return;
  }
  shuttingDown = true;
  try {
    bridgeContext.close();
  } catch (_err) {
    // Ignore shutdown errors.
  }
  if (managedRrfServer) {
    stopRrfHttpBridgeServer(managedRrfServer);
    managedRrfServer = null;
  }
  process.exit(exitCode);
}

async function main() {
  if (args.command) {
    await runOneShot();
    return;
  }

  await ensureRrfServerReady();
  rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
    terminal: process.stdin.isTTY,
  });

  if (process.stdin.isTTY && !args.quiet) {
    updatePromptForConnectionState(promptConnectedState, { forcePrompt: true });
  }

  rl.on('line', (line) => {
    enqueueLine(line);
  });

  rl.on('close', () => {
    shutdown(0);
  });

  if (bridgeContext.wss && !args.quiet) {
    console.log(`WebSocket feed ready on ws://localhost:${args.wsPort}`);
    console.log(`Open hp-sim with ${buildRrfHttpBridgeWsHint(args.wsPort)} to follow along.`);
  }
}

main().catch((err) => {
  console.error('Failed to start rrf_command_prompt:', err);
  shutdown(1);
});

process.on('SIGINT', () => shutdown(130));
process.on('SIGTERM', () => shutdown(143));
process.on('SIGHUP', () => shutdown(129));
