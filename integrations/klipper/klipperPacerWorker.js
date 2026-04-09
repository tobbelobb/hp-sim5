import { createKlipperSerialDecoder, SerialLineDecoder, decodeBase64Chunk } from './klipperSerialDecoder.js';
import { detectFileFormat, FileFormat, isKlipperFormat } from '../shared/fileFormatUtils.js';
import {
  KlipperBucketedMotionCore,
  applyBucketedParsedLine,
} from './klipperMotionCore.js';
import {
  API_MOTION_STARTUP_BUFFER_MS,
  KlipperApiSessionTimelineBuffer,
} from './klipperApiSessionTimeline.js';
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
// Klipper's BulkHelper delivers motion_report batches every 500ms, and the
// browser pacer needs materially more than one batch of slack to avoid
// stalling when a batch lands a little late.
const API_MOTION_TAIL_RESERVE_MS = 900;
const API_MOTION_STARTUP_QUIET_WINDOW_MS = 900;
const API_MOTION_INITIAL_PRELOAD_MS = API_MOTION_STARTUP_BUFFER_MS + API_MOTION_TAIL_RESERVE_MS;
const API_MOTION_EMIT_LOOKAHEAD_MS = 24;
const PACER_DIAGNOSTIC_RECORD_EPSILON_MS = 1;
const PACER_DIAGNOSTIC_OVERSLEEP_REPORT_MS = 8;
const PACER_DIAGNOSTIC_CLAMP_REPORT_MS = 2;
const PACER_DIAGNOSTIC_DEDUP_MS = 25;
const PACER_DIAGNOSTIC_MAX_EVENT_REPORTS = 8;

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
let apiStreamActive = false;
let apiStreamClockHz = MCU_CLOCK_HZ_KLIPPER_HOST;
let apiStreamStartTick = null;
let apiTimelineBuffer = null;
let apiStartupTimer = null;
let apiPlaybackStarted = false;
let pacerSessionSeq = 0;
let pacerDiagnostics = null;

const roundDiagnosticMs = (value) => (
  Number.isFinite(value) ? Math.round(value * 1000) / 1000 : null
);

const createPacerDiagnostics = (sessionType) => ({
  sessionId: ++pacerSessionSeq,
  sessionType,
  eventReports: 0,
  summaryPosted: false,
  timerOversleepCount: 0,
  maxTimerOversleepMs: 0,
  lastTimerReportWorkerMs: null,
  lastTimerReportOversleepMs: 0,
  horizonClampCount: 0,
  maxHorizonClampMs: 0,
  lastClampWorkerMs: null,
  lastClampSchedulableMs: null,
});

const emitPacerDiagnostic = (event, details = {}) => {
  if (!pacerDiagnostics) {
    return;
  }
  postMessage({
    type: 'diagnostic',
    diagnostic: {
      source: 'klipper_pacer',
      sessionId: pacerDiagnostics.sessionId,
      sessionType: pacerDiagnostics.sessionType,
      event,
      ...details,
    },
  });
};

const maybeEmitPacerEvent = (event, now, details = {}, {
  minSpacingMs = PACER_DIAGNOSTIC_DEDUP_MS,
  lastReportKey,
  maxValueKey = null,
  eventValue = null,
  minReportValue = null,
} = {}) => {
  if (!pacerDiagnostics || pacerDiagnostics.eventReports >= PACER_DIAGNOSTIC_MAX_EVENT_REPORTS) {
    return;
  }
  if (Number.isFinite(minReportValue) && !(eventValue >= minReportValue)) {
    return;
  }
  const lastReportAt = pacerDiagnostics[lastReportKey];
  const maxValue = maxValueKey ? pacerDiagnostics[maxValueKey] : null;
  const spacedOut = !Number.isFinite(lastReportAt) || (now - lastReportAt) >= minSpacingMs;
  const newMax = Number.isFinite(eventValue) && (!Number.isFinite(maxValue) || eventValue > maxValue);
  if (!spacedOut && !newMax) {
    return;
  }
  pacerDiagnostics[lastReportKey] = now;
  if (maxValueKey && Number.isFinite(eventValue)) {
    pacerDiagnostics[maxValueKey] = Math.max(maxValue || 0, eventValue);
  }
  pacerDiagnostics.eventReports += 1;
  emitPacerDiagnostic(event, details);
};

const noteTimerOversleep = (now) => {
  if (!pacerDiagnostics || !Number.isFinite(pacerNextDeadlineMs)) {
    return;
  }
  const oversleepMs = Math.max(0, now - pacerNextDeadlineMs);
  if (oversleepMs < PACER_DIAGNOSTIC_RECORD_EPSILON_MS) {
    return;
  }
  pacerDiagnostics.timerOversleepCount += 1;
  pacerDiagnostics.maxTimerOversleepMs = Math.max(
    pacerDiagnostics.maxTimerOversleepMs,
    oversleepMs,
  );
  maybeEmitPacerEvent('timer_oversleep', now, {
    oversleepMs: roundDiagnosticMs(oversleepMs),
    deadlineMs: roundDiagnosticMs(pacerNextDeadlineMs),
    observedWorkerMs: roundDiagnosticMs(now),
  }, {
    lastReportKey: 'lastTimerReportWorkerMs',
    maxValueKey: 'lastTimerReportOversleepMs',
    eventValue: oversleepMs,
    minReportValue: PACER_DIAGNOSTIC_OVERSLEEP_REPORT_MS,
  });
};

const noteHorizonClamp = ({
  now,
  logicalNow,
  schedulableLogicalMs,
}) => {
  if (!pacerDiagnostics) {
    return;
  }
  const clampMs = logicalNow - schedulableLogicalMs;
  if (clampMs < PACER_DIAGNOSTIC_RECORD_EPSILON_MS) {
    return;
  }
  const distinctClamp = (
    !Number.isFinite(pacerDiagnostics.lastClampWorkerMs)
    || (now - pacerDiagnostics.lastClampWorkerMs) >= PACER_DIAGNOSTIC_DEDUP_MS
    || !Number.isFinite(pacerDiagnostics.lastClampSchedulableMs)
    || Math.abs(pacerDiagnostics.lastClampSchedulableMs - schedulableLogicalMs) >= 1
  );
  if (distinctClamp) {
    pacerDiagnostics.horizonClampCount += 1;
    pacerDiagnostics.lastClampSchedulableMs = schedulableLogicalMs;
  }
  pacerDiagnostics.maxHorizonClampMs = Math.max(
    pacerDiagnostics.maxHorizonClampMs,
    clampMs,
  );
  maybeEmitPacerEvent('horizon_clamp', now, {
    clampMs: roundDiagnosticMs(clampMs),
    logicalPlaybackMs: roundDiagnosticMs(logicalNow),
    schedulableLogicalMs: roundDiagnosticMs(schedulableLogicalMs),
  }, {
    lastReportKey: 'lastClampWorkerMs',
    eventValue: clampMs,
    minReportValue: PACER_DIAGNOSTIC_CLAMP_REPORT_MS,
  });
};

const postPacerDiagnosticSummary = () => {
  if (!pacerDiagnostics || pacerDiagnostics.summaryPosted) {
    return;
  }
  pacerDiagnostics.summaryPosted = true;
  emitPacerDiagnostic('summary', {
    stats: {
      timerOversleepCount: pacerDiagnostics.timerOversleepCount,
      maxTimerOversleepMs: roundDiagnosticMs(pacerDiagnostics.maxTimerOversleepMs),
      horizonClampCount: pacerDiagnostics.horizonClampCount,
      maxHorizonClampMs: roundDiagnosticMs(pacerDiagnostics.maxHorizonClampMs),
    },
    config: {
      bucketIntervalMs: BUCKET_INTERVAL_MS,
      startupBufferMs: API_MOTION_STARTUP_BUFFER_MS,
      tailReserveMs: API_MOTION_TAIL_RESERVE_MS,
      startupPreloadMs: API_MOTION_INITIAL_PRELOAD_MS,
      emitLookaheadMs: API_MOTION_EMIT_LOOKAHEAD_MS,
    },
  });
};

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

const setLogicalPlaybackTime = (logicalMs, now = performance.now()) => {
  logicalClockAnchorMs = Math.max(0, Number(logicalMs) || 0);
  logicalClockAnchorWorkerMs = now;
};

const getApiTailReserveTicks = () => (
  (apiStreamActive && !inputComplete)
    ? Math.round((API_MOTION_TAIL_RESERVE_MS / 1000) * apiStreamClockHz)
    : 0
);

const getApiSchedulableLogicalMs = () => {
  if (!apiStreamActive || inputComplete) {
    return null;
  }
  const maxKnownTick = motionCore.getMaxKnownTick();
  if (!Number.isFinite(maxKnownTick)) {
    return null;
  }
  const schedulableTick = Math.max(0, maxKnownTick - getApiTailReserveTicks());
  return (schedulableTick / apiStreamClockHz) * 1000;
};

const clampApiLogicalPlaybackHorizon = (now = performance.now()) => {
  if (!apiStreamActive || inputComplete || asapMode || logicalClockPaused) {
    return;
  }
  const schedulableLogicalMs = getApiSchedulableLogicalMs();
  if (!Number.isFinite(schedulableLogicalMs)) {
    return;
  }
  const logicalNow = currentLogicalPlaybackMs(now);
  if (logicalNow > schedulableLogicalMs) {
    noteHorizonClamp({
      now,
      logicalNow,
      schedulableLogicalMs,
    });
    setLogicalPlaybackTime(schedulableLogicalMs, now);
  }
};

const resolveBucketTargetLogicalMs = (bucketIdx) => {
  if (apiStreamActive && Number.isFinite(apiStreamStartTick)) {
    const bucketEndTick = motionCore.getBucketEndTick(bucketIdx);
    const tickOffset = Math.max(0, bucketEndTick - apiStreamStartTick);
    return (tickOffset / apiStreamClockHz) * 1000;
  }
  return (bucketIdx + 1) * BUCKET_INTERVAL_MS;
};

const resolveBucketAtMs = (bucketIdx, now = performance.now()) => {
  if (asapMode) {
    return now;
  }
  clampApiLogicalPlaybackHorizon(now);
  const targetLogicalMs = resolveBucketTargetLogicalMs(bucketIdx);
  const logicalNow = currentLogicalPlaybackMs(now);
  if (targetLogicalMs <= logicalNow) {
    return now;
  }
  return now + ((targetLogicalMs - logicalNow) / getSpeedScale());
};

const getBucketEmitLookaheadMs = () => (
  apiStreamActive && !asapMode ? API_MOTION_EMIT_LOOKAHEAD_MS : 0
);

const isBucketReadyToEmit = (bucketIdx, now = performance.now(), emitLookaheadMs = 0) => {
  if (asapMode) {
    return true;
  }
  clampApiLogicalPlaybackHorizon(now);
  const targetLogicalMs = resolveBucketTargetLogicalMs(bucketIdx);
  return targetLogicalMs <= (currentLogicalPlaybackMs(now) + Math.max(0, emitLookaheadMs));
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
  apiStreamActive = false;
  apiStreamClockHz = MCU_CLOCK_HZ_KLIPPER_HOST;
  apiStreamStartTick = null;
  apiTimelineBuffer = null;
  apiPlaybackStarted = false;
  pacerDiagnostics = null;
  if (apiStartupTimer) {
    clearTimeout(apiStartupTimer);
    apiStartupTimer = null;
  }
  firstSeqSeen = null;
  expectedSeq = null;
};

const flushReadyBuckets = (force = false, holdBackOneBucket = false) => {
  const now = performance.now();
  clampApiLogicalPlaybackHorizon(now);
  const maxKnownTick = motionCore.getMaxKnownTick();
  const apiTailReserveTicks = getApiTailReserveTicks();
  const emitLookaheadMs = force ? 0 : getBucketEmitLookaheadMs();
  const commands = motionCore.flushCommands({
    force,
    forceThreshold: inputComplete,
    holdBackOneBucket,
    canEmitBucket: force ? null : (bucketIdx) => {
      if (!isBucketReadyToEmit(bucketIdx, now, emitLookaheadMs)) {
        return false;
      }
      if (apiTailReserveTicks > 0 && Number.isFinite(maxKnownTick)) {
        return motionCore.getBucketEndTick(bucketIdx) <= (maxKnownTick - apiTailReserveTicks);
      }
      return true;
    },
    buildTiming: (bucketIdx) => ({
      at: resolveBucketAtMs(bucketIdx, now),
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
  postPacerDiagnosticSummary();
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
  const now = performance.now();
  pacerTimer = null;
  if (isPaused) {
    return;
  }
  noteTimerOversleep(now);
  pacerLoop();
  if (maybePostDone()) {
    return;
  }
  if (hasPlaybackWork()) {
    scheduleNextPacer(now);
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

const startApiMotionStream = ({ clockHz = MCU_CLOCK_HZ_KLIPPER_HOST } = {}) => {
  resetRuntimeState();
  apiStreamActive = true;
  apiStreamClockHz = Number.isFinite(clockHz) && clockHz > 0 ? clockHz : MCU_CLOCK_HZ_KLIPPER_HOST;
  ws = null;
  pacerDiagnostics = createPacerDiagnostics('api_motion');
  apiTimelineBuffer = new KlipperApiSessionTimelineBuffer({
    clockHz: apiStreamClockHz,
    // Prepay the startup window and the live tail reserve before the first
    // bucket is allowed to move, otherwise playback can stall on the first
    // reserve boundary even when Klipper is streaming smoothly.
    startupBufferMs: API_MOTION_INITIAL_PRELOAD_MS,
  });
};

const flushBufferedApiMotion = ({ forceStart = false } = {}) => {
  if (!apiStreamActive) {
    startApiMotionStream();
  }
  if (!apiTimelineBuffer) {
    return;
  }
  const readyBatches = apiTimelineBuffer.flushReady({ forceStart });
  if (readyBatches.length === 0) {
    return;
  }
  if (!apiPlaybackStarted || !Number.isFinite(apiStreamStartTick)) {
    apiStreamStartTick = 0;
    apiPlaybackStarted = true;
    resetLogicalPlaybackClock();
  }
  for (const batch of readyBatches) {
    motionCore.consumeStepperBatch({
      stepperName: batch.name,
      firstClock: batch.timeline_start_tick,
      startMcuPosition: batch.start_mcu_position,
      data: batch.data,
    });
  }
  flushReadyBucketsIfEnabled(false, false);
  if (!isPaused && hasPlaybackWork()) {
    ensurePacerRunning();
  }
};

const scheduleApiStartupFlush = () => {
  if (apiStartupTimer) {
    clearTimeout(apiStartupTimer);
  }
  apiStartupTimer = setTimeout(() => {
    apiStartupTimer = null;
    // Re-check without forcing: the initial preload must include the startup
    // window and tail reserve. Short sessions are still released by
    // api_motion_finish(), which force-starts once the stream is complete.
    flushBufferedApiMotion({ forceStart: false });
  }, API_MOTION_STARTUP_QUIET_WINDOW_MS);
};

const consumeApiMotionBatch = (batch = {}) => {
  if (!apiStreamActive) {
    startApiMotionStream();
  }
  if (!apiTimelineBuffer) {
    return;
  }
  apiTimelineBuffer.consumeStepperBatch(batch);
  if (apiTimelineBuffer.canStart(false)) {
    flushBufferedApiMotion({ forceStart: false });
    return;
  }
  scheduleApiStartupFlush();
};

const consumeApiTrapqBatch = (batch = {}) => {
  if (!apiStreamActive) {
    startApiMotionStream();
  }
  if (!apiTimelineBuffer) {
    return;
  }
  apiTimelineBuffer.consumeTrapqBatch(batch);
  if (apiTimelineBuffer.canStart(false)) {
    flushBufferedApiMotion({ forceStart: false });
  }
};

const finishApiMotionStream = () => {
  if (apiStartupTimer) {
    clearTimeout(apiStartupTimer);
    apiStartupTimer = null;
  }
  flushBufferedApiMotion({ forceStart: true });
  inputComplete = true;
  if (!maybePostDone() && !isPaused && hasPlaybackWork()) {
    ensurePacerRunning();
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
  } else if (type === 'api_motion_start') {
    LOG_MOVE = Boolean(data.logMove);
    moveLogSeq = 0;
    resetMoveLogAxes();
    startApiMotionStream({ clockHz: data.clockHz });
  } else if (type === 'api_motion_batch') {
    consumeApiMotionBatch(data.batch);
  } else if (type === 'api_motion_trapq_batch') {
    consumeApiTrapqBatch(data.batch);
  } else if (type === 'api_motion_finish') {
    finishApiMotionStream();
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
