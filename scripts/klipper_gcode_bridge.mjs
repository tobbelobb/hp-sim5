#!/usr/bin/env node
import fs from 'node:fs';
import path from 'node:path';
import readline from 'node:readline';
import { spawn } from 'node:child_process';
import WebSocket, { WebSocketServer } from 'ws';
import { KlipperApiBridge } from '../autocal/control/primitives/klipper_api_bridge.mjs';
import { createKlipperMotionRelay } from '../autocal/control/primitives/klipper_motion_relay.mjs';
import {
  buildKlippySpawnSpec,
  buildMcuBridgeSpawnSpec,
  parseKlipperGcodeBridgeArgs,
} from '../autocal/control/primitives/klipper_gcode_bridge_config.mjs';
import { connectWebSocketWithRetry } from '../autocal/control/primitives/klipper_ws_connect.mjs';

function printHelp() {
  console.log(`Usage: node scripts/klipper_gcode_bridge.mjs [options]

Start a local Klipper stack, accept G-code on stdin, and stream motion into hp-sim-3d.

Options:
  --config <path>          Klipper printer.cfg (default: examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg)
  --socket <path>          Klipper API socket path (default: /tmp/klippy_uds)
  --ws-port <port>         hp-sim websocket port (default: 8790)
  --bridge-ws-port <port>  Internal klipper_linux_mcu_bridge websocket port (default: 8770)
  --raw-path <path>        Raw MCU PTY path (default: /tmp/klipper_host_mcu_raw)
  --host-path <path>       Host-facing MCU PTY path (default: /tmp/klipper_host_mcu)
  --mcu-bin <path>         klipper_mcu binary (default: examples/klipper/linux_mcu/klipper.elf)
  --dict <path>            Klipper dictionary (default: examples/klipper/linux_mcu/klipper.dict)
  --klipper-py <path>      Klipper Python package dir (default: ./klipper/klippy/)
  --klippy-python <path>   Python interpreter for klippy.py (default: ~/klippy-env/bin/python)
  --cmd, -c <GCODE>        Send one G-code line and exit
  --quiet, -q              Reduce console output
  --help, -h               Show this help`);
}

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function waitForPath(filePath, timeoutMs = 10000) {
  const absPath = path.isAbsolute(filePath) ? filePath : path.resolve(process.cwd(), filePath);
  const deadline = Date.now() + timeoutMs;
  return new Promise((resolve, reject) => {
    const tick = () => {
      if (fs.existsSync(absPath)) {
        resolve(absPath);
        return;
      }
      if (Date.now() > deadline) {
        reject(new Error(`Timed out waiting for ${absPath}`));
        return;
      }
      setTimeout(tick, 50);
    };
    tick();
  });
}

function spawnLogged(command, args, options = {}) {
  const child = spawn(command, args, {
    stdio: ['ignore', 'pipe', 'pipe'],
    ...options,
  });
  return child;
}

function prefixStream(prefix, stream, quiet = false) {
  if (!stream) {
    return;
  }
  stream.setEncoding('utf8');
  let buffer = '';
  stream.on('data', (chunk) => {
    buffer += chunk;
    let newlineIndex;
    while ((newlineIndex = buffer.indexOf('\n')) >= 0) {
      const line = buffer.slice(0, newlineIndex);
      buffer = buffer.slice(newlineIndex + 1);
      if (!quiet) {
        console.log(`[${prefix}] ${line}`);
      }
    }
  });
  stream.on('end', () => {
    if (buffer && !quiet) {
      console.log(`[${prefix}] ${buffer}`);
    }
  });
}

const args = parseKlipperGcodeBridgeArgs(process.argv.slice(2));

if (args.help) {
  printHelp();
  process.exit(0);
}

const externalClients = new Set();
let externalServer = null;
let klippyProc = null;
let mcuBridgeProc = null;
let bridgeSocket = null;
let apiBridge = null;
let motionRelay = null;
let currentGcode = null;
let shuttingDown = false;

const broadcastExternal = (payload) => {
  if (!externalServer || !payload) {
    return;
  }
  const data = JSON.stringify(payload);
  for (const socket of externalClients) {
    if (socket.readyState === WebSocket.OPEN) {
      try {
        socket.send(data);
      } catch (_err) {
        // Ignore send failures; the socket close handler will clean up.
      }
    }
  }
};

function closeSocket(socket) {
  if (!socket) {
    return;
  }
  try {
    socket.close();
  } catch (_err) {
    // Ignore close failures.
  }
}

async function startExternalServer() {
  externalServer = new WebSocketServer({ port: args.wsPort });
  externalServer.on('connection', (socket) => {
    externalClients.add(socket);
    socket.on('close', () => {
      externalClients.delete(socket);
    });
    socket.on('message', (raw) => {
      try {
        const payload = JSON.parse(raw.toString());
        if (payload?.type === 'set_speed_scale' && Number.isFinite(payload.value) && motionRelay) {
          motionRelay.setSpeedScale(payload.value);
        }
        if (payload?.type === 'set_asap_mode' && motionRelay) {
          motionRelay.setAsapMode(Boolean(payload.enable));
        }
      } catch (_err) {
        // Ignore client messages we do not understand.
      }
    });
  });
}

async function startMcuBridge() {
  const spec = buildMcuBridgeSpawnSpec(args);
  mcuBridgeProc = spawnLogged(spec.command, spec.args, { cwd: process.cwd() });
  prefixStream('mcu-bridge:out', mcuBridgeProc.stdout, args.quiet);
  prefixStream('mcu-bridge:err', mcuBridgeProc.stderr, args.quiet);
  await waitForPath(args.hostPath);
}

async function connectBridgeSocket() {
  const url = `ws://127.0.0.1:${args.bridgeWsPort}`;
  return connectWebSocketWithRetry(url, {
    WebSocketImpl: WebSocket,
    timeoutMs: 10000,
    retryDelayMs: 100,
  });
}

async function startMotionRelay() {
  motionRelay = await createKlipperMotionRelay({
    onCommand: (command) => {
      broadcastExternal({ type: 'command', command, gcode: currentGcode });
    },
  });
}

async function startKlippy() {
  const spec = buildKlippySpawnSpec(args);
  klippyProc = spawnLogged(spec.command, spec.args, { cwd: process.cwd() });
  prefixStream('klippy:out', klippyProc.stdout, args.quiet);
  prefixStream('klippy:err', klippyProc.stderr, args.quiet);
}

async function startApiBridge() {
  apiBridge = new KlipperApiBridge({
    socketPath: args.socketPath,
    onMessage: (msg) => {
      const response = msg?.params?.response;
      if (typeof response === 'string' && response.trim()) {
        broadcastExternal({ type: 'reply', gcode: currentGcode, reply: response });
        if (!args.quiet) {
          console.log(response);
        }
      }
    },
    onError: (err) => {
      if (!args.quiet) {
        console.error(err.message);
      }
    },
  });
  await apiBridge.connect();
  await apiBridge.subscribeTerminalOutput();
  await apiBridge.request('info', {
    client_info: {
      program: 'klipper_gcode_bridge',
      version: 'v0.1',
    },
  });
}

async function bootstrap() {
  await startExternalServer();
  await startMcuBridge();
  bridgeSocket = await connectBridgeSocket();
  bridgeSocket.on('message', (raw) => {
    try {
      const msg = JSON.parse(raw.toString());
      if (msg?.action === 'klipper_parsed' && Array.isArray(msg.lines) && motionRelay) {
        motionRelay.feedParsedLines(msg.lines);
      }
    } catch (_err) {
      // Ignore non-JSON messages.
    }
  });
  bridgeSocket.on('close', () => {
    if (!shuttingDown && !args.quiet) {
      console.warn('Internal Klipper bridge websocket closed.');
    }
  });
  await startMotionRelay();
  await startKlippy();
  await startApiBridge();
}

async function shutdown(exitCode = 0) {
  if (shuttingDown) {
    return;
  }
  shuttingDown = true;
  try {
    if (motionRelay) {
      motionRelay.close();
    }
  } catch (_err) {
    // Ignore.
  }
  try {
    closeSocket(bridgeSocket);
  } catch (_err) {
    // Ignore.
  }
  try {
    if (apiBridge) {
      apiBridge.close();
    }
  } catch (_err) {
    // Ignore.
  }
  for (const socket of externalClients) {
    closeSocket(socket);
  }
  externalClients.clear();
  try {
    externalServer?.close();
  } catch (_err) {
    // Ignore.
  }
  for (const child of [klippyProc, mcuBridgeProc]) {
    if (!child) {
      continue;
    }
    try {
      child.kill('SIGTERM');
    } catch (_err) {
      // Ignore.
    }
  }
  await wait(100);
  process.exit(exitCode);
}

function attachProcessHandlers() {
  process.on('SIGINT', () => {
    shutdown(0).catch(() => process.exit(1));
  });
  process.on('SIGTERM', () => {
    shutdown(0).catch(() => process.exit(1));
  });
}

const stdinQueue = [];
let stdinProcessing = false;
let rl = null;

async function processStdinQueue() {
  if (stdinProcessing) {
    return;
  }
  stdinProcessing = true;
  try {
    while (stdinQueue.length > 0) {
      const next = stdinQueue.shift();
      // eslint-disable-next-line no-await-in-loop
      await next();
    }
  } finally {
    stdinProcessing = false;
  }
}

function enqueueLine(line) {
  stdinQueue.push(() => handleLine(line));
  processStdinQueue().catch((err) => {
    if (!args.quiet) {
      console.error(err.message);
    }
  });
}

async function handleLine(line) {
  const trimmed = line.trim();
  if (!trimmed) {
    return;
  }
  currentGcode = trimmed;
  if (!args.quiet) {
    console.log(`> ${trimmed}`);
  }
  try {
    await apiBridge.sendGcodeLine(trimmed);
    await wait(150);
  } catch (err) {
    console.error(`Error sending "${trimmed}": ${err.message}`);
  } finally {
    currentGcode = null;
  }
}

async function runOneShot() {
  await bootstrap();
  await handleLine(args.command);
  await shutdown(0);
}

async function runInteractive() {
  await bootstrap();
  rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
    terminal: process.stdin.isTTY,
  });

  rl.on('line', (line) => {
    enqueueLine(line);
  });

  rl.on('close', () => {
    shutdown(0).catch(() => process.exit(1));
  });

  if (process.stdin.isTTY && !args.quiet) {
    rl.setPrompt('gcode> ');
    rl.prompt();
  }
}

attachProcessHandlers();

if (args.command) {
  runOneShot().catch((err) => {
    console.error(`Failed to run bridge: ${err.message}`);
    shutdown(1).catch(() => process.exit(1));
  });
} else {
  runInteractive().catch((err) => {
    console.error(`Failed to start bridge: ${err.message}`);
    shutdown(1).catch(() => process.exit(1));
  });
}
