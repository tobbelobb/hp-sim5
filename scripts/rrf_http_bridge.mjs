#!/usr/bin/env node
import readline from 'node:readline';
import { createGcodeBridge, parseBridgeArgs } from './gcode_bridge.mjs';

function printHelp() {
  console.log(`Usage: node scripts/rrf_http_bridge.mjs [options]

Send single-line G-code to an rrf_simulator HTTP server and stream the resulting
motion commands over WebSocket for hp-sim to visualize.

Options:
  --server, --rrf <url>    Base URL of rrf_simulator (default: http://localhost:8080)
  --ws-port <port>         Port for the WebSocket fan-out (0 to disable, default: 8790)
  --no-ws                  Disable WebSocket fan-out entirely
  --cmd, -c <GCODE>        Send one G-code line and exit
  --quiet, -q              Only print replies (suppress prompts and extra logs)
  --help, -h               Show this help`);
}

const args = parseBridgeArgs(process.argv.slice(2));
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

let currentGcode = null;
const bridge = bridgeContext.bridge;

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
  currentGcode = trimmed;
  if (!args.quiet) {
    console.log(`> ${trimmed}`);
  }
  try {
    const result = await bridgeContext.sendGcodeLine(trimmed);
    console.log(result.reply.trim());
  } catch (err) {
    console.error(`Error sending "${trimmed}": ${err.message}`);
  } finally {
    currentGcode = null;
    promptIfInteractive();
  }
}

function enqueueLine(line) {
  sendQueue.push(() => handleGcodeLine(line));
  processQueue().catch((err) => {
    console.error('Unexpected error while processing queue:', err);
  });
}

function sendEncoderRequest(axes, timeoutMs = ENCODER_REQUEST_TIMEOUT_MS) {
  const readyClients = getReadyWsClients();
  if (readyClients.length === 0) {
    throw new Error('Message not received');
  }
  const requestId = encoderRequestSeq++;
  const payload = { type: 'encoder_request', requestId, axes };
  const data = JSON.stringify(payload);
  readyClients.forEach((client) => {
    try {
      client.send(data);
    } catch (_err) {
      // Ignore send errors; timeout will handle missing responses
    }
  });
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      pendingEncoderRequests.delete(requestId);
      reject(new Error('Message not received'));
    }, Math.max(1, timeoutMs));
    pendingEncoderRequests.set(requestId, {
      resolve: (value) => {
        clearTimeout(timeout);
        pendingEncoderRequests.delete(requestId);
        resolve(value);
      },
      reject: (err) => {
        clearTimeout(timeout);
        pendingEncoderRequests.delete(requestId);
        reject(err);
      },
    });
  });
}

async function runOneShot() {
  await bridgeContext.waitForHpSimConnection();
  await handleGcodeLine(args.command);
  bridgeContext.close();
  process.exit(0);
}

if (args.command) {
  runOneShot().catch((err) => {
    console.error('Failed to send command:', err);
    process.exit(1);
  });
} else {
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
    bridgeContext.close();
    process.exit(0);
  });

  if (bridgeContext.wss && !args.quiet) {
    console.log(`WebSocket feed ready on ws://localhost:${args.wsPort}`);
    console.log(`Open hp-sim with ?gcode_ws=ws://localhost:${args.wsPort} to follow along.`);
  }
}
