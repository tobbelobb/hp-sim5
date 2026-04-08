import { createKlipperSerialDecoder, SerialLineDecoder, decodeBase64Chunk } from './klipperSerialDecoder.js';
import { detectFileFormat, FileFormat, isKlipperFormat } from '../shared/fileFormatUtils.js';
import {
  KlipperBucketedMotionCore,
  applyBucketedParsedLine,
} from './klipperMotionCore.js';
import {
  DEFAULT_AXIS_ORDER,
  MCU_CLOCK_HZ_KLIPPER_HOST,
} from './klipperFirmwareModel.js';

// This is a worker script for Klipper pacing.

let ws = null;
let DEBUG = false;
let firstSeqSeen = null;
let expectedSeq = null;

const axisOrder = DEFAULT_AXIS_ORDER;
const BUCKET_INTERVAL_MS = 2;
const ENABLE_BUCKETED_ANTIALIASING = true;

let LOG_MOVE = false;
let moveLogSeq = 0;
let moveLogAxes = Object.fromEntries(axisOrder.map((axis) => [axis, 0]));

const serialDecoder = createKlipperSerialDecoder();
let serialLineDecoder = null;

const ensureSerialDecoder = () => {
  if (!serialLineDecoder) {
    serialLineDecoder = new SerialLineDecoder(serialDecoder);
  }
  return serialLineDecoder;
};

const motionCore = new KlipperBucketedMotionCore({
  axisOrder,
  bucketIntervalMs: BUCKET_INTERVAL_MS,
  clockHz: MCU_CLOCK_HZ_KLIPPER_HOST,
  enableBucketedAntialiasing: ENABLE_BUCKETED_ANTIALIASING,
});

let pacerTimer = null;
let pacerNextDeadlineMs = null;
let isPaused = false;
let speedScale = 1;
let asapMode = false;
let inputComplete = false;
let donePosted = false;
let pendingLookaheadLine = null;
let deferBucketFlush = false;
let logicalClockAnchorMs = 0;
let logicalClockAnchorWorkerMs = 0;
let logicalClockPaused = false;

const getSpeedScale = () => (
  Number.isFinite(speedScale) && speedScale > 0 ? speedScale : 1
);

const scaleDelayMs = (delayMs) => {
  if (asapMode) {
    return 0;
  }
  const safeDelay = Math.max(0, Number(delayMs) || 0);
  return safeDelay / getSpeedScale();
};

const getPacerIntervalMs = () => {
  if (asapMode) {
    return 0;
  }
  return scaleDelayMs(BUCKET_INTERVAL_MS);
};

const currentLogicalPlaybackMs = (now = performance.now()) => {
  if (logicalClockPaused) {
    return logicalClockAnchorMs;
  }
  return logicalClockAnchorMs + Math.max(0, now - logicalClockAnchorWorkerMs) * getSpeedScale();
};

const syncLogicalPlaybackClock = (now = performance.now()) => {
  logicalClockAnchorMs = currentLogicalPlaybackMs(now);
  logicalClockAnchorWorkerMs = now;
};

const setLogicalPlaybackPaused = (paused, now = performance.now()) => {
  syncLogicalPlaybackClock(now);
  logicalClockPaused = Boolean(paused);
};

const resetLogicalPlaybackClock = () => {
  logicalClockAnchorMs = 0;
  logicalClockAnchorWorkerMs = performance.now();
  logicalClockPaused = false;
};

const resolveBucketAtMs = (bucketIdx) => {
  const now = performance.now();
  if (asapMode) {
    return now;
  }
  const targetLogicalMs = (bucketIdx + 1) * BUCKET_INTERVAL_MS;
  const logicalNow = currentLogicalPlaybackMs(now);
  if (targetLogicalMs <= logicalNow) {
    return now;
  }
  return now + ((targetLogicalMs - logicalNow) / getSpeedScale());
};

const isBucketDueNow = (bucketIdx, now = performance.now()) => {
  if (asapMode) {
    return true;
  }
  const targetLogicalMs = (bucketIdx + 1) * BUCKET_INTERVAL_MS;
  return targetLogicalMs <= currentLogicalPlaybackMs(now);
};

const hasBucketWork = () => pendingLookaheadLine !== null || motionCore.hasPendingBuckets();
const hasPlaybackWork = () => hasBucketWork();

const resetMoveLogAxes = () => {
  moveLogAxes = Object.fromEntries(axisOrder.map((axis) => [axis, 0]));
};

const applyCommandToMoveLogAxes = (command) => {
  if (!command || command.type !== 'Move') {
    return;
  }
  for (const axis of axisOrder) {
    if (axis === 'E') {
      if (Number.isFinite(command.E)) {
        moveLogAxes.E += command.E;
      }
      continue;
    }
    if (Number.isFinite(command[axis])) {
      moveLogAxes[axis] = command[axis];
    }
  }
};

const buildMoveLogEntry = (command) => {
  try {
    applyCommandToMoveLogAxes(command);
    return {
      seq: moveLogSeq++,
      at_ms: command.at,
      span_ms: command.span,
      axes: { ...moveLogAxes },
      axis_units: {
        spool: 'radians',
        E: 'millimeters',
      },
      command: { ...command },
    };
  } catch (error) {
    console.error('Failed to build move log entry:', error);
    return null;
  }
};

const postMoveCommand = (command) => {
  const scheduled = {
    span: 0,
    at: performance.now(),
    ...command,
  };
  const logEntry = LOG_MOVE && scheduled.type === 'Move'
    ? buildMoveLogEntry(scheduled)
    : null;
  if (LOG_MOVE && logEntry) {
    postMessage({ type: 'move', command: scheduled, logEntry });
  } else {
    postMessage({ type: 'move', command: scheduled });
  }
};

const resetRuntimeState = () => {
  if (pacerTimer) {
    clearTimeout(pacerTimer);
    pacerTimer = null;
  }
  pacerNextDeadlineMs = null;
  motionCore.reset();
  resetLogicalPlaybackClock();
  resetMoveLogAxes();
  serialLineDecoder = null;
  inputComplete = false;
  donePosted = false;
  pendingLookaheadLine = null;
  deferBucketFlush = false;
  firstSeqSeen = null;
  expectedSeq = null;
};

const flushReadyBuckets = (force = false, holdBackOneBucket = false) => {
  const commands = motionCore.flushCommands({
    force,
    holdBackOneBucket,
    canEmitBucket: force ? null : (bucketIdx) => isBucketDueNow(bucketIdx),
    buildTiming: (bucketIdx) => ({
      at: resolveBucketAtMs(bucketIdx),
      span: scaleDelayMs(BUCKET_INTERVAL_MS),
    }),
  });
  for (const command of commands) {
    postMoveCommand(command);
  }
  return commands.length;
};

const flushReadyBucketsIfEnabled = (force = false, holdBackOneBucket = false) => {
  if (deferBucketFlush && !force) {
    return 0;
  }
  return flushReadyBuckets(force, holdBackOneBucket);
};

const maybePostDone = () => {
  if (!inputComplete || donePosted || isPaused) {
    return false;
  }
  if (hasPlaybackWork()) {
    return false;
  }
  donePosted = true;
  pacerNextDeadlineMs = null;
  if (pacerTimer) {
    clearTimeout(pacerTimer);
    pacerTimer = null;
  }
  postMessage({ type: 'done' });
  return true;
};

const pacerLoop = () => {
  try {
    if (isPaused) {
      return;
    }
    flushReadyBuckets(false, false);
  } catch (error) {
    console.error('KlipperPacer pacer error:', error);
  }
};

const scheduleNextPacer = (now = performance.now()) => {
  if (isPaused || (inputComplete && donePosted)) {
    return;
  }
  const intervalMs = getPacerIntervalMs();
  if (intervalMs <= 0) {
    pacerNextDeadlineMs = now;
  } else if (pacerNextDeadlineMs === null) {
    pacerNextDeadlineMs = now + intervalMs;
  } else {
    while (pacerNextDeadlineMs <= now) {
      pacerNextDeadlineMs += intervalMs;
    }
  }
  const delay = intervalMs <= 0 ? 0 : Math.max(0, pacerNextDeadlineMs - now);
  pacerTimer = setTimeout(pacerTick, delay);
};

const pacerTick = () => {
  pacerTimer = null;
  if (isPaused) {
    return;
  }
  pacerLoop();
  if (maybePostDone()) {
    return;
  }
  if (hasPlaybackWork()) {
    scheduleNextPacer();
  }
};

const ensurePacerRunning = () => {
  if (isPaused || (inputComplete && donePosted)) {
    return;
  }
  if (pacerTimer == null) {
    const now = performance.now();
    pacerNextDeadlineMs = now;
    scheduleNextPacer(now);
  }
};

const setPaused = (paused) => {
  isPaused = Boolean(paused);
  setLogicalPlaybackPaused(isPaused);
  if (isPaused) {
    if (pacerTimer) {
      clearTimeout(pacerTimer);
      pacerTimer = null;
    }
    pacerNextDeadlineMs = null;
    return;
  }
  if (maybePostDone()) {
    return;
  }
  if (hasPlaybackWork()) {
    const now = performance.now();
    pacerNextDeadlineMs = now;
    scheduleNextPacer(now);
  }
};

const setSpeedScale = (value) => {
  const now = performance.now();
  const logicalNow = currentLogicalPlaybackMs(now);
  speedScale = value;
  logicalClockAnchorMs = logicalNow;
  logicalClockAnchorWorkerMs = now;
  if (!isPaused && hasPlaybackWork()) {
    if (pacerTimer) {
      clearTimeout(pacerTimer);
      pacerTimer = null;
    }
    pacerNextDeadlineMs = now;
    scheduleNextPacer(now);
  }
};

const setAsapMode = (enabled) => {
  const nextAsapMode = Boolean(enabled);
  if (asapMode === nextAsapMode) {
    return;
  }
  syncLogicalPlaybackClock();
  asapMode = nextAsapMode;
  if (!asapMode) {
    logicalClockAnchorWorkerMs = performance.now();
    return;
  }
  if (!isPaused && hasPlaybackWork()) {
    if (pacerTimer) {
      clearTimeout(pacerTimer);
      pacerTimer = null;
    }
    pacerNextDeadlineMs = performance.now();
    scheduleNextPacer(pacerNextDeadlineMs);
  }
};

const processParsedLine = (line, nextLine = null) => {
  applyBucketedParsedLine(motionCore, line);
  flushReadyBucketsIfEnabled(false, nextLine !== null);
};

const pushParsedLine = (line) => {
  if (typeof line !== 'string' || line.length === 0) {
    return;
  }
  if (pendingLookaheadLine !== null) {
    processParsedLine(pendingLookaheadLine, line);
  }
  pendingLookaheadLine = line;
};

const flushPendingParsedLine = (forceBuckets = true) => {
  if (pendingLookaheadLine !== null) {
    processParsedLine(pendingLookaheadLine, null);
    pendingLookaheadLine = null;
  }
  if (forceBuckets) {
    flushReadyBuckets(true, false);
  }
};

const processSerialLines = (lines) => {
  if (!Array.isArray(lines) || lines.length === 0) {
    return;
  }
  for (const line of lines) {
    if (typeof line === 'string' && line.length > 0) {
      pushParsedLine(line);
    }
  }
  if (inputComplete) {
    flushPendingParsedLine();
    maybePostDone();
  }
};

const feedSerialChunk = (chunk) => {
  try {
    const decoder = ensureSerialDecoder();
    const lines = decoder.push(chunk);
    if (lines.length) {
      processSerialLines(lines);
    }
  } catch (error) {
    console.error('KlipperPacer serial decode failed', error);
    try {
      postMessage({ type: 'error', message: error?.message || 'Serial decode failed' });
    } catch (_error) {}
  }
};

const handleBinaryPayload = (payload) => {
  if (!payload) {
    return;
  }
  if (typeof Blob !== 'undefined' && payload instanceof Blob) {
    payload.arrayBuffer().then((buffer) => {
      feedSerialChunk(new Uint8Array(buffer));
    }).catch((error) => {
      console.error('KlipperPacer failed to read Blob payload', error);
      try {
        postMessage({ type: 'error', message: error?.message || 'Failed to read Blob payload' });
      } catch (_error) {}
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
  ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => console.log(`connected to ${url}`);
  ws.onmessage = (event) => {
    if (typeof event.data === 'string') {
      try {
        const message = JSON.parse(event.data);
        if (message && message.action === 'klipper_parsed' && Array.isArray(message.lines)) {
          if (firstSeqSeen === null && typeof message.seq === 'number') {
            firstSeqSeen = message.seq;
            if (DEBUG) {
              console.log(`first klipper_parsed seq=${firstSeqSeen} count=${message.count}`);
            }
          }
          if (typeof message.seq === 'number' && typeof message.count === 'number') {
            if (expectedSeq === null) {
              expectedSeq = message.seq;
            }
            if (message.seq !== expectedSeq && DEBUG) {
              console.error(`seq discontinuity: expected ${expectedSeq}, got ${message.seq}`);
            }
            expectedSeq = message.seq + message.count;
          }
          for (const line of message.lines) {
            pushParsedLine(line);
          }
          return;
        }
        if (message && message.action === 'klipper_serial') {
          const chunk = decodeBase64Chunk(message.data || message.chunk || message.payload);
          if (chunk) {
            feedSerialChunk(chunk);
          }
          return;
        }
        if (message && message.action === 'klipper_clock') {
          return;
        }
      } catch (_error) {}
      console.log(`unhandled text: ${event.data.slice(0, 100)}`);
    } else {
      handleBinaryPayload(event.data);
    }
  };
  ws.onerror = (error) => console.error('websocket error:', error);
  ws.onclose = () => {
    console.log('connection closed');
    if (serialLineDecoder) {
      const remaining = serialLineDecoder.flush();
      processSerialLines(remaining);
    }
    flushPendingParsedLine();
    resetRuntimeState();
    ws = null;
    postMessage({ type: 'closed' });
  };
};

const finishUploadInput = ({ forceBucketFlush = true } = {}) => {
  if (serialLineDecoder) {
    const remaining = serialLineDecoder.flush();
    processSerialLines(remaining);
    serialLineDecoder = null;
  }
  flushPendingParsedLine(forceBucketFlush);
  inputComplete = true;
  if (!forceBucketFlush && !ws) {
    flushReadyBuckets(true, false);
  }
  if (!maybePostDone() && !isPaused && hasPlaybackWork()) {
    ensurePacerRunning();
  }
};

const processUploadedText = async (file) => {
  const text = await file.text();
  const lines = text.split(/\r?\n/g);
  for (const rawLine of lines) {
    const line = typeof rawLine === 'string' ? rawLine.trim() : '';
    if (!line) {
      continue;
    }
    pushParsedLine(line);
  }
};

const processUploadedSerial = async (file) => {
  const buffer = await file.arrayBuffer();
  feedSerialChunk(new Uint8Array(buffer));
};

const processUploadedFile = async (file, explicitFormat = null) => {
  if (!file || typeof file.text !== 'function' || typeof file.arrayBuffer !== 'function') {
    postMessage({ type: 'error', message: 'Klipper raw upload requires a File or Blob-like object.' });
    return;
  }
  const format = explicitFormat || detectFileFormat(file.name) || FileFormat.MCU_TEXT;
  if (!isKlipperFormat(format)) {
    postMessage({ type: 'error', message: `Unsupported Klipper raw upload format: ${format}` });
    return;
  }
  resetRuntimeState();
  deferBucketFlush = true;
  ws = null;
  try {
    if (format === FileFormat.MCU_SERIAL) {
      await processUploadedSerial(file);
    } else {
      await processUploadedText(file);
    }
    resetLogicalPlaybackClock();
    finishUploadInput({ forceBucketFlush: false });
    deferBucketFlush = false;
  } catch (error) {
    deferBucketFlush = false;
    console.error('KlipperPacer upload processing failed', error);
    postMessage({ type: 'error', message: error?.message || 'Failed to process Klipper upload' });
  }
};

self.onmessage = (event) => {
  const { type, ...data } = event.data;
  if (type === 'connect') {
    LOG_MOVE = Boolean(data.logMove);
    moveLogSeq = 0;
    resetMoveLogAxes();
    if (LOG_MOVE) {
      try {
        console.log('KlipperPacer move logging enabled');
      } catch (_error) {}
    }
    connect(data.url);
  } else if (type === 'filename_upload') {
    LOG_MOVE = Boolean(data.logMove);
    moveLogSeq = 0;
    resetMoveLogAxes();
    processUploadedFile(data.filename, data.format);
  } else if (type === 'set_speed_scale') {
    setSpeedScale(data.value);
  } else if (type === 'set_asap_mode') {
    setAsapMode(data.enable);
  } else if (type === 'pause') {
    setPaused(true);
  } else if (type === 'resume') {
    setPaused(false);
  } else if (type === 'disconnect' || type === 'close') {
    try {
      ws?.close();
    } catch (_error) {}
    ws = null;
    resetRuntimeState();
    postMessage({ type: 'closed' });
  }
};
