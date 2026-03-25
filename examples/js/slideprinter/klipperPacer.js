import { KlipperCommander } from './klipperCommander.js';
import { createKlipperSerialDecoder, SerialLineDecoder, decodeBase64Chunk } from './klipperSerialParser.js';
import { normalizeSpeedScale } from '../../../autocal/control/primitives/klipper_live_timing.mjs';

// This worker turns live parsed Klipper MCU output into the same bucketed
// Move commands used by KlipperCommander. That keeps the raw bridge on the
// smoother code path instead of reconstructing individual step wakeups.

let ws = null;
let commander = null;
let commanderStream = null;
let commanderRunPromise = null;
let serialLineDecoder = null;
let LOG_MOVE = false;
let moveLogSeq = 0;
let speedScale = 1.0;
let extruderPosMm = 0;

const axisOrder = ['A', 'B', 'C', 'D', 'E'];
const EXTRUDER_AXIS = 'E';
const EXTRUDER_ROTATION_DISTANCE_MM = 33.5;
const STEPS_PER_REV = 200;
const MICROSTEPS = 16;
const EXTRUDER_MM_PER_STEP = EXTRUDER_ROTATION_DISTANCE_MM / (STEPS_PER_REV * MICROSTEPS);

const serialDecoder = createKlipperSerialDecoder();

class PushableTextStream {
  constructor() {
    this.chunks = [];
    this.waiters = [];
    this.closed = false;
    this.error = null;
  }

  pipeThrough() {
    return this;
  }

  getReader() {
    return {
      read: () => this._read(),
    };
  }

  push(chunk) {
    if (this.closed || this.error || typeof chunk !== 'string' || chunk.length === 0) {
      return;
    }
    if (this.waiters.length > 0) {
      const waiter = this.waiters.shift();
      waiter.resolve({ value: chunk, done: false });
      return;
    }
    this.chunks.push(chunk);
  }

  close() {
    if (this.closed) {
      return;
    }
    this.closed = true;
    while (this.waiters.length > 0) {
      const waiter = this.waiters.shift();
      waiter.resolve({ value: undefined, done: true });
    }
  }

  fail(error) {
    if (this.closed || this.error) {
      return;
    }
    this.error = error || new Error('PushableTextStream failed');
    while (this.waiters.length > 0) {
      const waiter = this.waiters.shift();
      waiter.reject(this.error);
    }
  }

  _read() {
    if (this.error) {
      return Promise.reject(this.error);
    }
    if (this.chunks.length > 0) {
      return Promise.resolve({ value: this.chunks.shift(), done: false });
    }
    if (this.closed) {
      return Promise.resolve({ value: undefined, done: true });
    }
    return new Promise((resolve, reject) => {
      this.waiters.push({ resolve, reject });
    });
  }
}

function resetRuntimeState() {
  try {
    resumeCommander();
  } catch (_err) {
    // Ignore resume failures during teardown.
  }
  try {
    commanderStream?.close();
  } catch (_err) {
    // Ignore stream shutdown errors.
  }
  commander = null;
  commanderStream = null;
  commanderRunPromise = null;
  serialLineDecoder = null;
  moveLogSeq = 0;
  extruderPosMm = 0;
}

function resumeCommander() {
  if (!commander) {
    return;
  }
  commander.isPaused = false;
  if (commander.resolveResume) {
    commander.resolveResume();
    commander.resolveResume = null;
  }
}

function buildMoveLogEntry(command, atMs, spanMs) {
  try {
    const axes = {};
    for (const axis of axisOrder) {
      if (axis === EXTRUDER_AXIS) {
        axes[axis] = extruderPosMm;
      } else {
        axes[axis] = commander?.axisAngles?.get(axis) || 0;
      }
    }
    return {
      seq: moveLogSeq++,
      at_ms: atMs,
      span_ms: spanMs,
      axes,
      axis_units: {
        spool: 'radians',
        [EXTRUDER_AXIS]: 'millimeters',
      },
      command: { ...command },
    };
  } catch (err) {
    console.error('Failed to build move log entry:', err);
    return null;
  }
}

function ensureCommander() {
  if (commander) {
    return commander;
  }
  if (!commanderStream) {
    commanderStream = new PushableTextStream();
  }

  commander = new KlipperCommander();
  commander.setSpeedScale(speedScale);
  commander.sendCommand = async (command) => {
    if (!command || typeof command !== 'object') {
      return;
    }

    const emittedCommand = { ...command, at: performance.now(), span: 0 };
    if (Number.isFinite(emittedCommand.E)) {
      extruderPosMm += emittedCommand.E;
    }

    if (LOG_MOVE) {
      const logEntry = buildMoveLogEntry(emittedCommand, emittedCommand.at, emittedCommand.span);
      if (logEntry) {
        postMessage({ type: 'move', command: emittedCommand, logEntry });
        return;
      }
    }

    postMessage({ type: 'move', command: emittedCommand });
  };

  commanderRunPromise = commander.run(commanderStream);
  commanderRunPromise.catch((err) => {
    console.error('KlipperPacer commander failed:', err);
    try {
      postMessage({ type: 'error', message: err?.message || String(err) });
    } catch (_postErr) {
      // Ignore error reporting failures.
    }
  });

  return commander;
}

function pushParsedLines(lines) {
  if (!Array.isArray(lines) || lines.length === 0) {
    return;
  }
  ensureCommander();
  commanderStream.push(`${lines.join('\n')}\n`);
}

const ensureSerialDecoder = () => {
  if (!serialLineDecoder) {
    serialLineDecoder = new SerialLineDecoder(serialDecoder);
  }
  return serialLineDecoder;
};

const feedSerialChunk = (chunk) => {
  try {
    const decoder = ensureSerialDecoder();
    const lines = decoder.push(chunk);
    if (lines.length > 0) {
      pushParsedLines(lines);
    }
  } catch (err) {
    console.error('KlipperPacer serial decode failed', err);
    try {
      postMessage({ type: 'error', message: err?.message || 'Serial decode failed' });
    } catch (_postErr) {
      // Ignore error reporting failures.
    }
  }
};

const handleBinaryPayload = (payload) => {
  if (!payload) {
    return;
  }
  if (typeof Blob !== 'undefined' && payload instanceof Blob) {
    payload.arrayBuffer().then((buf) => feedSerialChunk(new Uint8Array(buf))).catch((err) => {
      console.error('KlipperPacer failed to read Blob payload', err);
      try {
        postMessage({ type: 'error', message: err?.message || 'Failed to read Blob payload' });
      } catch (_postErr) {
        // Ignore error reporting failures.
      }
    });
    return;
  }
  if (payload instanceof ArrayBuffer) {
    feedSerialChunk(new Uint8Array(payload));
    return;
  }
  if (ArrayBuffer.isView(payload)) {
    feedSerialChunk(new Uint8Array(payload.buffer, payload.byteOffset, payload.byteLength));
  }
};

const connect = (url) => {
  resetRuntimeState();
  ensureCommander();

  ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => console.log(`connected to ${url}`);
  ws.onmessage = (event) => {
    if (typeof event.data === 'string') {
      try {
        const msg = JSON.parse(event.data);
        if (msg && msg.action === 'klipper_parsed' && Array.isArray(msg.lines)) {
          pushParsedLines(msg.lines);
          return;
        }
        if (msg && msg.action === 'klipper_serial') {
          const chunk = decodeBase64Chunk(msg.data || msg.chunk || msg.payload);
          if (chunk) {
            feedSerialChunk(chunk);
          }
          return;
        }
        if (msg && msg.action === 'klipper_clock') {
          // The bucketed live path no longer needs live clock remapping.
          return;
        }
      } catch (_err) {
        // Fall through to the debug log below.
      }
      console.log(`unhandled text: ${event.data.slice(0, 100)}`);
      return;
    }
    handleBinaryPayload(event.data);
  };
  ws.onerror = (err) => {
    console.error('websocket error:', err);
    try {
      postMessage({ type: 'error', message: err?.message || 'websocket error' });
    } catch (_postErr) {
      // Ignore error reporting failures.
    }
  };
  ws.onclose = () => {
    console.log('connection closed');
    resumeCommander();
    if (serialLineDecoder) {
      const remaining = serialLineDecoder.flush();
      if (remaining.length > 0) {
        pushParsedLines(remaining);
      }
      serialLineDecoder = null;
    }
    try {
      commanderStream?.close();
    } catch (_err) {
      // Ignore stream shutdown errors.
    }
    commanderStream = null;
    commanderRunPromise = null;
    ws = null;
    postMessage({ type: 'closed' });
  };
};

self.onmessage = (e) => {
  const { type, ...data } = e.data || {};
  if (type === 'connect') {
    LOG_MOVE = Boolean(data.logMove);
    speedScale = normalizeSpeedScale(data.speedScale, 1.0);
    if (LOG_MOVE) {
      moveLogSeq = 0;
      try {
        console.log('KlipperPacer move logging enabled');
      } catch (_err) {
        // Ignore console failures in worker contexts.
      }
    }
    connect(data.url);
    return;
  }

  if (type === 'close') {
    if (ws) {
      ws.close();
    }
    return;
  }

  if (type === 'set_speed_scale') {
    const nextScale = normalizeSpeedScale(data.value, 1.0);
    speedScale = nextScale;
    if (commander) {
      commander.setSpeedScale(nextScale);
      commander.accumulatedWaitMs = 0.0;
    }
    return;
  }

  if (type === 'set_asap_mode') {
    if (commander) {
      commander.setAsapMode(Boolean(data.enable));
    }
    return;
  }

  if (type === 'set_fast_mode') {
    if (commander) {
      commander.fastMode = Boolean(data.enable);
      if (!commander.fastMode) {
        commander.accumulatedWaitMs = 0.0;
      }
    }
    return;
  }

  if (type === 'set_dt') {
    if (commander && Number.isFinite(data.dt) && data.dt > 0) {
      commander.setDt(data.dt);
    }
    return;
  }

  if (type === 'pause') {
    if (commander) {
      commander.isPaused = true;
    }
    return;
  }

  if (type === 'resume') {
    resumeCommander();
  }
};
