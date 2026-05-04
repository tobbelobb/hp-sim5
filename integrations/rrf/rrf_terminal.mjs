#!/usr/bin/env node
import path from 'node:path';
import fs from 'node:fs';
import readline from 'node:readline';
import { createGcodeBridge } from './rrfSimulatorBridge.mjs';
import { parseRrfMotorAxisMapFromConfigText } from './rrfFirmwareModel.js';
import { waitForRrfSimulator } from '../../autocal/control/primitives/encoder_utils.mjs';
import {
  DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT,
  buildRrfHttpBridgeWsHint,
  ensureRrfHttpBridgeServer,
  isRrfServerUnavailableError,
  stopRrfHttpBridgeServer,
} from './rrf_http_bridge_cli_config.mjs';

function printHelp() {
  console.log(`Usage: node integrations/rrf/rrf_terminal.mjs [options]

Send single-line G-code to an rrf_simulator HTTP server and stream the resulting
motion commands over WebSocket for hp-sim to visualize.

Options:
  --server, --rrf <url>    Base URL of rrf_simulator (default: http://localhost:8080)
  --ws-port <port>         Port for the WebSocket fan-out (0 to disable, default: 8790)
  --no-ws                  Disable WebSocket fan-out entirely
  --cmd <GCODE>            Send one G-code line and exit
  -c, --config <CONFIG>    RRF config file for autostart (config_name.g or sys/config_name.g)
  -m, --machineType <TYPE> Machine type for autostart when --config is omitted
  --buildup, --line-layers Use config_<machine>_w_line_layers.g for autostart
  --no-buildup, --no_buildup, no_buildup, --no-line-layers
                           Use config_<machine>.g for autostart
  --quiet, -q              Only print replies (suppress prompts and extra logs)
  Auto-start launcher      ${DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT}
  --help, -h               Show this help`);
}

function takeValue(argv, index, optionName) {
  const value = argv[index + 1];
  if (!value) {
    throw new Error(`${optionName} requires an argument`);
  }
  return value;
}

function normalizeLineLayerSuffix(lineLayerArg) {
  return lineLayerArg === '--buildup' || lineLayerArg === '--line-layers'
    ? '_w_line_layers'
    : '';
}

function resolveRrfConfigPath(configValue) {
  if (!configValue) {
    return null;
  }
  const candidates = [];
  if (path.isAbsolute(configValue)) {
    candidates.push(configValue);
  } else {
    candidates.push(path.resolve(process.cwd(), configValue));
    candidates.push(path.resolve(process.cwd(), 'RRF/run/vsd/sys', configValue));
    candidates.push(path.resolve(process.cwd(), 'RRF/run/vsd', configValue));
  }
  return candidates.find((candidate) => fs.existsSync(candidate)) || candidates[0] || null;
}

function readDriverToAxisFromConfig(configPath) {
  if (!configPath) {
    return null;
  }
  try {
    const configText = fs.readFileSync(configPath, 'utf8');
    const driverToAxis = parseRrfMotorAxisMapFromConfigText(configText);
    return driverToAxis.size > 0 ? driverToAxis : null;
  } catch (err) {
    if (!args?.quiet) {
      console.warn(`rrf_terminal.mjs: unable to read RRF motor map from ${configPath}: ${err.message}`);
    }
    return null;
  }
}

export function parseArgs(argv) {
  const envServer = process.env.RRF_SERVER_URL;
  const hasCustomStartScript = Boolean(process.env.RRF_HTTP_BRIDGE_START_SCRIPT);
  let simulatorConfig = null;
  let machineType = 'hp3';
  let lineLayerArg = '--line-layers';
  let hasAutostartOption = false;
  const args = {
    server: envServer || 'http://localhost:8080',
    wsPort: 8790,
    noWs: false,
    command: null,
    startScript: DEFAULT_RRF_HTTP_BRIDGE_START_SCRIPT,
    startScriptArgs: [],
    quiet: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--server' || arg === '--rrf') {
      args.server = takeValue(argv, i, arg);
      i += 1;
      args.serverExplicit = true;
    } else if (arg === '--ws-port') {
      const value = parseInt(takeValue(argv, i, arg), 10);
      i += 1;
      if (Number.isFinite(value) && value > 0) {
        args.wsPort = value;
      } else {
        args.wsPort = 0;
      }
    } else if (arg === '--cmd') {
      args.command = takeValue(argv, i, arg);
      i += 1;
    } else if (arg === '-c' || arg === '--config') {
      simulatorConfig = takeValue(argv, i, arg);
      hasAutostartOption = true;
      i += 1;
    } else if (arg.startsWith('--config=')) {
      simulatorConfig = arg.slice('--config='.length);
      hasAutostartOption = true;
    } else if (arg === '-m' || arg === '--machineType') {
      machineType = takeValue(argv, i, arg);
      hasAutostartOption = true;
      i += 1;
    } else if (arg.startsWith('--machineType=')) {
      machineType = arg.slice('--machineType='.length);
      hasAutostartOption = true;
    } else if (arg === '--buildup' || arg === '--line-layers') {
      lineLayerArg = arg;
      hasAutostartOption = true;
    } else if (
      arg === '--no-buildup'
      || arg === '--no_buildup'
      || arg === 'no_buildup'
      || arg === '--no-line-layers'
    ) {
      lineLayerArg = arg;
      hasAutostartOption = true;
    } else if (arg === '--no-ws') {
      args.noWs = true;
    } else if (arg === '--quiet' || arg === '-q') {
      args.quiet = true;
    } else if (arg === '--help' || arg === '-h') {
      args.help = true;
    } else {
      throw new Error(`unknown option: ${arg}`);
    }
  }

  if (simulatorConfig) {
    args.startScriptArgs = ['-c', simulatorConfig];
    args.configPath = resolveRrfConfigPath(simulatorConfig);
  } else if (hasAutostartOption || !hasCustomStartScript) {
    args.startScriptArgs = ['-m', machineType, lineLayerArg];
    args.configPath = resolveRrfConfigPath(`config_${machineType}${normalizeLineLayerSuffix(lineLayerArg)}.g`);
  }

  return args;
}

const PROMPT_CONNECTED = 'gcode> ';
const PROMPT_DISCONNECTED = '\x1b[90mdisconnected>\x1b[0m ';
let args = null;
let bridgeContext = null;
let promptConnectedState = true;
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
        startScript: args.startScript,
        startScriptArgs: args.startScriptArgs,
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

export async function runCli(argv = process.argv.slice(2)) {
  try {
    args = parseArgs(argv);
  } catch (err) {
    console.error(`rrf_terminal.mjs: ${err.message}`);
    console.error('Try node integrations/rrf/rrf_terminal.mjs --help for usage.');
    return 2;
  }
  if (args.help) {
    printHelp();
    return 0;
  }

  bridgeContext = createGcodeBridge({
    server: args.server,
    wsPort: args.noWs ? 0 : args.wsPort,
    quiet: args.quiet,
    onClientChange: (connected) => updatePromptForConnectionState(connected),
    driverToAxis: readDriverToAxisFromConfig(args.configPath),
  });
  promptConnectedState = !bridgeContext.wss;
  await main();
  return 0;
}

const isMain = path.basename(process.argv[1] || '') === 'rrf_terminal.mjs';

if (isMain) {
  runCli()
    .then((exitCode) => {
      if (Number.isInteger(exitCode) && exitCode !== 0) {
        process.exitCode = exitCode;
      }
    })
    .catch((err) => {
      console.error(`Failed to start rrf_command_prompt: ${err.message}`);
      shutdown(1);
    });
}

process.on('SIGINT', () => shutdown(130));
process.on('SIGTERM', () => shutdown(143));
process.on('SIGHUP', () => shutdown(129));
