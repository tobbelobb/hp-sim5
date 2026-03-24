#!/usr/bin/env node
import readline from 'node:readline';
import { KlipperApiBridge } from '../autocal/control/primitives/klipper_api_bridge.mjs';

function printHelp() {
  console.log(`Usage: node scripts/klipper_api_bridge.mjs [options]

Send individual G-code lines to a live Klipper instance over the API socket.

Options:
  --socket <path>          Unix domain socket path (default: /tmp/klippy_uds)
  --cmd, -c <GCODE>        Send one G-code line and exit
  --quiet, -q              Only print Klipper terminal output
  --help, -h               Show this help`);
}

function parseArgs(argv) {
  const out = {
    socketPath: process.env.KLIPPY_UDS || '/tmp/klippy_uds',
    command: null,
    quiet: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--help' || arg === '-h') {
      out.help = true;
      continue;
    }
    if (arg === '--quiet' || arg === '-q') {
      out.quiet = true;
      continue;
    }
    if (arg === '--socket') {
      out.socketPath = argv[i + 1] || out.socketPath;
      i += 1;
      continue;
    }
    if (arg.startsWith('--socket=')) {
      out.socketPath = arg.slice('--socket='.length) || out.socketPath;
      continue;
    }
    if (arg === '--cmd' || arg === '-c') {
      out.command = argv[i + 1] || '';
      i += 1;
      continue;
    }
    if (arg.startsWith('--cmd=')) {
      out.command = arg.slice('--cmd='.length);
      continue;
    }
  }

  return out;
}

const args = parseArgs(process.argv.slice(2));

if (args.help) {
  printHelp();
  process.exit(0);
}

const bridge = new KlipperApiBridge({
  socketPath: args.socketPath,
  onMessage: (msg) => {
    const response = msg?.params?.response;
    if (typeof response === 'string' && response.length > 0) {
      console.log(response);
      return;
    }
    if (!args.quiet && msg?.params && Object.keys(msg.params).length > 0) {
      console.log(JSON.stringify(msg.params));
    }
  },
  onError: (err) => {
    console.error(err.message);
  },
});

const sendQueue = [];
let processingQueue = false;

async function processQueue() {
  if (processingQueue) {
    return;
  }
  processingQueue = true;
  try {
    while (sendQueue.length > 0) {
      const next = sendQueue.shift();
      // eslint-disable-next-line no-await-in-loop
      await next();
    }
  } finally {
    processingQueue = false;
  }
}

function enqueueLine(line) {
  sendQueue.push(() => handleLine(line));
  processQueue().catch((err) => {
    console.error(err.message);
  });
}

async function handleLine(line) {
  const trimmed = line.trim();
  if (!trimmed) {
    return;
  }
  if (!args.quiet) {
    console.log(`> ${trimmed}`);
  }
  await bridge.sendGcodeLine(trimmed);
}

async function runOneShot() {
  await startupPromise;
  await bridge.sendGcodeLine(args.command);
  bridge.close();
  process.exit(0);
}

const startupPromise = (async () => {
  await bridge.connect();
  await bridge.subscribeTerminalOutput();
  await bridge.request('info', {
    client_info: {
      program: 'klipper_api_bridge',
      version: 'v0.1',
    },
  });
})();

if (args.command) {
  runOneShot().catch((err) => {
    console.error(`Failed to send command: ${err.message}`);
    bridge.close();
    process.exit(1);
  });
} else {
  const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
    terminal: process.stdin.isTTY,
  });

  startupPromise.catch((err) => {
    console.error(`Failed to connect to Klipper: ${err.message}`);
    process.exit(1);
  });

  rl.on('line', (line) => {
    startupPromise.then(() => {
      enqueueLine(line);
    }).catch((err) => {
      console.error(`Failed to connect to Klipper: ${err.message}`);
    });
  });

  rl.on('close', () => {
    bridge.close();
    process.exit(0);
  });
}
