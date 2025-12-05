#!/usr/bin/env node
import readline from 'node:readline';
import { WebSocketServer } from 'ws';
import { RrfHttpBridge } from '../examples/js/slideprinter/rrfHttpBridge.js';

function parseArgs(argv) {
  const args = {
    server: process.env.RRF_SERVER_URL || 'http://localhost:8080',
    wsPort: 8790,
    command: null,
    quiet: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--server' || arg === '--rrf') {
      args.server = argv[++i] || args.server;
    } else if (arg === '--ws-port') {
      const value = parseInt(argv[++i], 10);
      if (Number.isFinite(value) && value > 0) {
        args.wsPort = value;
      } else {
        args.wsPort = 0;
      }
    } else if (arg === '--cmd' || arg === '-c') {
      args.command = argv[++i] || null;
    } else if (arg === '--quiet' || arg === '-q') {
      args.quiet = true;
    } else if (arg === '--help' || arg === '-h') {
      args.help = true;
    }
  }

  return args;
}

function printHelp() {
  console.log(`Usage: node scripts/rrf_http_bridge.mjs [options]

Send single-line G-code to an rrf_simulator HTTP server and stream the resulting
motion commands over WebSocket for hp-sim to visualize.

Options:
  --server, --rrf <url>    Base URL of rrf_simulator (default: http://localhost:8080)
  --ws-port <port>         Port for the WebSocket fan-out (0 to disable, default: 8790)
  --cmd, -c <GCODE>        Send one G-code line and exit
  --quiet, -q              Only print replies (suppress prompts and extra logs)
  --help, -h               Show this help`);
}

const args = parseArgs(process.argv.slice(2));
if (args.help) {
  printHelp();
  process.exit(0);
}

const wss = args.wsPort
  ? new WebSocketServer({ port: args.wsPort })
  : null;

const pendingWsPayloads = [];
const MAX_PENDING_WS_PAYLOADS = 5000;
let waitingForClientNoticePrinted = false;

const hasReadyWsClients = () => Boolean(
  wss && Array.from(wss.clients).some((client) => client.readyState === 1)
);

const broadcast = (payload) => {
  if (!payload || !wss) {
    return;
  }
  const readyClients = [];
  wss.clients.forEach((client) => {
    if (client.readyState === 1) {
      readyClients.push(client);
    }
  });
  if (readyClients.length === 0) {
    enqueuePendingPayload(payload);
    return;
  }
  const data = JSON.stringify(payload);
  readyClients.forEach((client) => {
    client.send(data);
  });
};

function enqueuePendingPayload(payload) {
  if (!wss || !payload) {
    return;
  }
  pendingWsPayloads.push(payload);
  const overflow = pendingWsPayloads.length - MAX_PENDING_WS_PAYLOADS;
  if (overflow > 0) {
    pendingWsPayloads.splice(0, overflow);
  }
  if (!waitingForClientNoticePrinted && !args.quiet) {
    console.log('Waiting for hp-sim WebSocket client to connect before streaming...');
    waitingForClientNoticePrinted = true;
  }
}

function flushPendingPayloads() {
  if (!hasReadyWsClients() || pendingWsPayloads.length === 0) {
    return;
  }
  const batch = pendingWsPayloads.splice(0, pendingWsPayloads.length);
  if (!args.quiet) {
    console.log(`Streaming ${batch.length} queued message${batch.length === 1 ? '' : 's'} to hp-sim client.`);
  }
  batch.forEach((payload) => broadcast(payload));
  waitingForClientNoticePrinted = false;
}

if (wss) {
  if (!args.quiet) {
    console.log(`WebSocket feed ready on ws://localhost:${args.wsPort}`);
    console.log(`Open hp-sim with ?gcode_ws=ws://localhost:${args.wsPort} to follow along.`);
  }
  wss.on('connection', (socket) => {
    if (!args.quiet) {
      console.log('hp-sim connected to WebSocket feed.');
    }
    flushPendingPayloads();
    socket.on('close', () => {
      if (!args.quiet) {
        console.log('hp-sim disconnected from WebSocket feed.');
      }
    });
  });
}

let currentGcode = null;
const bridge = new RrfHttpBridge({
  baseUrl: args.server.replace(/\/$/, ''),
  remoteSpoolSystem: {
    addCommand: (command) => {
      if (!command) {
        return;
      }
      broadcast({ type: 'command', command, gcode: currentGcode });
    },
  },
});

const sendQueue = [];
let processingQueue = false;

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
    return;
  }
  currentGcode = trimmed;
  if (!args.quiet) {
    console.log(`> ${trimmed}`);
  }
  try {
    const result = await bridge.sendGCode(trimmed);
    console.log(result.reply.trim());
    broadcast({ type: 'reply', gcode: trimmed, reply: result.reply });
  } catch (err) {
    console.error(`Error sending "${trimmed}": ${err.message}`);
  } finally {
    currentGcode = null;
  }
}

function enqueueLine(line) {
  sendQueue.push(() => handleGcodeLine(line));
  processQueue().catch((err) => {
    console.error('Unexpected error while processing queue:', err);
  });
}

async function runOneShot() {
  await handleGcodeLine(args.command);
  if (wss) {
    wss.close();
  }
  process.exit(0);
}

if (args.command) {
  runOneShot().catch((err) => {
    console.error('Failed to send command:', err);
    process.exit(1);
  });
} else {
  const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
    terminal: process.stdin.isTTY,
  });

  if (process.stdin.isTTY && !args.quiet) {
    rl.setPrompt('gcode> ');
    rl.prompt();
  }

  rl.on('line', (line) => {
    enqueueLine(line);
    if (process.stdin.isTTY && !args.quiet) {
      rl.prompt();
    }
  });

  rl.on('close', () => {
    if (wss) {
      wss.close();
    }
    process.exit(0);
  });
}
