import { createKlipperSerialDecoder, SerialLineDecoder, decodeBase64Chunk } from './klipperSerialDecoder.js';
import { detectFileFormat, FileFormat, isKlipperFormat } from '../shared/fileFormatUtils.js';
import {
  computeQueueStepDurationTicks,
  recordAnchoredStepSequence,
  recordDistributedSequence,
} from './motionUtils.js';
import {
  DEFAULT_AXIS_ORDER,
  EXTRUDER_MM_PER_STEP_KLIPPER,
  MCU_CLOCK_HZ_KLIPPER_HOST,
  STEP_ANGLE_RAD,
} from './klipperFirmwareModel.js';

// This is a worker script for Klipper pacing.

// --- Globals for worker state ---
let ws = null;
let DEBUG = false;
let firstSeqSeen = null; // Debug: first server seq index we receive
let expectedSeq = null;  // Debug: detect gaps in parsed-line stream

const axisOrder = DEFAULT_AXIS_ORDER;
const axisAngles = new Map(axisOrder.map(a => [a, 0.0]));
const stepAngle = STEP_ANGLE_RAD;
// Treat oid 4 as the extruder stepper and convert its steps to filament mm.
// Use rotation_distance from the example config (33.5 mm per full rotation).
// mm per microstep = rotation_distance / (stepsPerRev * microsteps)
const EXTRUDER_AXIS = 'E';
const spoolAxisOrder = axisOrder.filter((axis) => axis !== EXTRUDER_AXIS);
let extruderPosMm = 0;

// Throttle outgoing Move messages to avoid overwhelming the browser.
// Pacer will run at this interval and emit at most once per interval.
const PACER_INTERVAL_MS = 2;
const MIN_EMIT_INTERVAL_MS = PACER_INTERVAL_MS;
// False keeps exact per-step bucketing without distributeEvenly smoothing.
const ENABLE_BUCKETED_ANTIALIASING = true;
const BUCKET_INTERVAL_MS = PACER_INTERVAL_MS;
const TICKS_PER_BUCKET = Math.max(1, Math.round(MCU_CLOCK_HZ_KLIPPER_HOST * (BUCKET_INTERVAL_MS / 1000)));

// Aggregation buffer across pacer ticks
let lastEmitMs = 0;
let aggMove = { type: 'Move' };
let aggFirstStepTimeMs = null;
let aggLastStepTimeMs = null;
let aggHasAny = false;

let LOG_MOVE = false;
let moveLogSeq = 0;

const serialDecoder = createKlipperSerialDecoder();
let serialLineDecoder = null;

const ensureSerialDecoder = () => {
  if (!serialLineDecoder) {
    serialLineDecoder = new SerialLineDecoder(serialDecoder);
  }
  return serialLineDecoder;
};

const FULL_WRAP = 0x100000000;
const HALF_WRAP = FULL_WRAP / 2;

const wrap32 = (value) => {
  if (!Number.isFinite(value)) return 0;
  let v = value % FULL_WRAP;
  if (v < 0) v += FULL_WRAP;
  return v;
};

class ClockModel {
  constructor() {
    this.reset();
  }

  reset() {
    this.clockHz = 50_000_000;
    this.lastRawTick = null;
    this.lastMcuTick = null;
    this.sampleWorkerMs = null;
  }

  ticksToMs(ticks) {
    return (ticks / this.clockHz) * 1000.0;
  }

  getLastRawTick() {
    return this.lastRawTick;
  }

  currentMcuTick() {
    return this.lastMcuTick;
  }

  isReady() {
    return this.lastRawTick !== null && this.lastMcuTick !== null &&
      this.sampleWorkerMs !== null;
  }

  updateFromMessage(msg, receiveMs) {
    if (msg && typeof msg.clock_hz === 'number' && Number.isFinite(msg.clock_hz) && msg.clock_hz > 0) {
      this.clockHz = msg.clock_hz;
    }
    const clockVal = Number(msg?.mcu_clock);
    if (!Number.isFinite(clockVal)) {
      return;
    }
    const unwrapped = this._updateUnwrapped(clockVal);
    if (!Number.isFinite(unwrapped)) {
      return;
    }
    this.lastMcuTick = unwrapped;
    this.sampleWorkerMs = receiveMs;
  }

  _updateUnwrapped(rawTick) {
    if (!Number.isFinite(rawTick)) {
      return null;
    }
    if (this.lastRawTick === null || this.lastMcuTick === null) {
      this.lastRawTick = rawTick;
      this.lastMcuTick = rawTick;
      return this.lastMcuTick;
    }
    let diff = rawTick - this.lastRawTick;
    if (diff > HALF_WRAP) diff -= FULL_WRAP;
    else if (diff < -HALF_WRAP) diff += FULL_WRAP;
    this.lastRawTick = rawTick;
    this.lastMcuTick += diff;
    return this.lastMcuTick;
  }

  mapRawClock(rawTick) {
    if (!Number.isFinite(rawTick)) {
      return null;
    }
    if (this.lastRawTick === null || this.lastMcuTick === null) {
      return null;
    }
    let diff = rawTick - this.lastRawTick;
    if (diff > HALF_WRAP) diff -= FULL_WRAP;
    else if (diff < -HALF_WRAP) diff += FULL_WRAP;
    return this.lastMcuTick + diff;
  }

  mcuToWorkerMs(rawTick) {
    const mapped = this.mapRawClock(rawTick);
    if (mapped === null || this.sampleWorkerMs === null || this.lastMcuTick === null) {
      return null;
    }
    const deltaTicks = mapped - this.lastMcuTick;
    return this.sampleWorkerMs + (deltaTicks / this.clockHz) * 1000.0;
  }
}

const clockModel = new ClockModel();
const ticksToMs = (ticks) => clockModel.ticksToMs(ticks);
let pacerTimer = null;       // setTimeout handle for pacer
let pacerNextDeadlineMs = null; // Absolute deadline for the next pacer wakeup
let isPaused = false;
let speedScale = 1;
let asapMode = false;
let inputComplete = false;
let donePosted = false;

const axisState = new Map(axisOrder.map(a => [a, {
  segments: [],
  nextWakeTimeMs: null,
  intervalTicks: null,
  addTicks: 0,
  remaining: 0,
  dirSign: 1,
  lastTick: 0,
  baseAngle: 0,
  hasSteps: false,
  // Direction used for the currently executing segment
  activeDirSign: 1,
  baseClockRaw: null,
  nextStepClockRaw: null,
}]));

const bucketSteps = new Map();
const bucketExtrusion = new Map();
const bucketAddToReference = new Map();
const bucketActiveAxes = new Set();

let nextBucketToEmit = 0;
let maxBucketSeen = -1;
let pendingLookaheadLine = null;
let deferBucketFlush = false;
let logicalClockAnchorMs = 0;
let logicalClockAnchorWorkerMs = 0;
let logicalClockPaused = false;

const getSpeedScale = () => (
  Number.isFinite(speedScale) && speedScale > 0 ? speedScale : 1
);

const scaleDelayMs = (delayMs) => {
  if (asapMode) return 0;
  const safeDelay = Math.max(0, Number(delayMs) || 0);
  return safeDelay / getSpeedScale();
};

const getPacerIntervalMs = () => {
  if (asapMode) {
    return 0;
  }
  return scaleDelayMs(PACER_INTERVAL_MS);
};

const getMinEmitIntervalMs = () => {
  if (asapMode) {
    return 0;
  }
  return scaleDelayMs(MIN_EMIT_INTERVAL_MS);
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

const workerToLogicalPlaybackMs = (workerTargetMs, now = performance.now()) => {
  const logicalNow = currentLogicalPlaybackMs(now);
  if (!Number.isFinite(workerTargetMs)) {
    return logicalNow;
  }
  return logicalNow + Math.max(0, workerTargetMs - now);
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

const hasAxisWork = () => {
  for (const axis of axisOrder) {
    const st = axisState.get(axis);
    if (!st) continue;
    if (st.nextWakeTimeMs !== null || st.remaining > 0 || st.segments.length > 0) {
      return true;
    }
  }
  return false;
};

const hasBucketWork = () => {
  if (pendingLookaheadLine !== null) {
    return true;
  }
  return nextBucketToEmit <= maxBucketSeen;
};

const hasPlaybackWork = () => hasAxisWork() || hasBucketWork() || aggHasAny;

const ensureBucketMap = (axis) => {
  let map = bucketSteps.get(axis);
  if (!map) {
    map = new Map();
    bucketSteps.set(axis, map);
  }
  return map;
};

const recordBucketSteps = (axis, bucketIdx, delta) => {
  const bucketMap = ensureBucketMap(axis);
  bucketMap.set(bucketIdx, (bucketMap.get(bucketIdx) || 0) + delta);
};

const recordBucketExtrusion = (bucketIdx, delta) => {
  bucketExtrusion.set(bucketIdx, (bucketExtrusion.get(bucketIdx) || 0) + delta);
};

const updateMaxBucketSeen = (bucketIdx) => {
  maxBucketSeen = Math.max(maxBucketSeen, bucketIdx);
};

const markBucketAxisActive = (axis) => {
  if (!axis) {
    return;
  }
  const st = axisState.get(axis);
  if (!st) {
    return;
  }
  bucketActiveAxes.add(axis);
  if (axis !== EXTRUDER_AXIS && !axisAngles.has(axis)) {
    axisAngles.set(axis, st.baseAngle || 0);
  }
};

const resetRuntimeState = () => {
  if (pacerTimer) {
    clearTimeout(pacerTimer);
    pacerTimer = null;
  }
  pacerNextDeadlineMs = null;
  clockModel.reset();
  resetLogicalPlaybackClock();
  bucketSteps.clear();
  bucketExtrusion.clear();
  bucketAddToReference.clear();
  bucketActiveAxes.clear();
  nextBucketToEmit = 0;
  maxBucketSeen = -1;
  pendingLookaheadLine = null;
  deferBucketFlush = false;
  for (const axis of axisOrder) {
    axisAngles.set(axis, 0.0);
    const st = axisState.get(axis);
    if (!st) continue;
    st.segments.length = 0;
    st.nextWakeTimeMs = null;
    st.intervalTicks = null;
    st.addTicks = 0;
    st.remaining = 0;
    st.dirSign = 1;
    st.lastTick = 0;
    st.baseAngle = 0;
    st.hasSteps = false;
    st.activeDirSign = 1;
    st.baseClockRaw = null;
    st.nextStepClockRaw = null;
  }
  extruderPosMm = 0;
  aggMove = { type: 'Move' };
  aggFirstStepTimeMs = null;
  aggLastStepTimeMs = null;
  aggHasAny = false;
  lastEmitMs = 0;
  serialLineDecoder = null;
  inputComplete = false;
  donePosted = false;
  firstSeqSeen = null;
  expectedSeq = null;
};

const resolveScaledWakeTimeMs = (nextStepRaw, fallbackDelayMs, fallbackBaseMs = performance.now()) => {
  const workerNow = performance.now();
  if (!ENABLE_BUCKETED_ANTIALIASING) {
    if (asapMode) {
      return currentLogicalPlaybackMs(workerNow);
    }
    const mapped = nextStepRaw !== null ? clockModel.mcuToWorkerMs(nextStepRaw) : null;
    if (mapped !== null) {
      return workerToLogicalPlaybackMs(mapped, workerNow);
    }
    const logicalBaseMs = Number.isFinite(fallbackBaseMs)
      ? fallbackBaseMs
      : currentLogicalPlaybackMs(workerNow);
    return logicalBaseMs + Math.max(0, Number(fallbackDelayMs) || 0);
  }
  if (asapMode) {
    return workerNow;
  }
  const mapped = nextStepRaw !== null ? clockModel.mcuToWorkerMs(nextStepRaw) : null;
  if (mapped !== null) {
    return workerNow + scaleDelayMs(mapped - workerNow);
  }
  return fallbackBaseMs + scaleDelayMs(fallbackDelayMs);
};

const postMoveCommand = (command, atMs = performance.now(), spanMs = 0) => {
  const scheduled = {
    ...command,
    at: atMs,
    span: spanMs,
  };
  const logEntry = LOG_MOVE && scheduled.type === 'Move'
    ? buildMoveLogEntry(scheduled, atMs, spanMs)
    : null;
  if (LOG_MOVE && logEntry) {
    postMessage({ type: 'move', command: scheduled, logEntry });
  } else {
    postMessage({ type: 'move', command: scheduled });
  }
  return scheduled;
};

const emitAggregatedMove = (atMs = performance.now()) => {
  if (!aggHasAny) {
    return false;
  }
  const spanMs = (aggFirstStepTimeMs != null && aggLastStepTimeMs != null)
    ? Math.max(0, aggLastStepTimeMs - aggFirstStepTimeMs)
    : 0;
  postMoveCommand(aggMove, atMs, spanMs);
  aggMove = { type: 'Move' };
  aggFirstStepTimeMs = null;
  aggLastStepTimeMs = null;
  aggHasAny = false;
  lastEmitMs = performance.now();
  return true;
};

const maybePostDone = () => {
  if (!inputComplete || donePosted || isPaused) {
    return false;
  }
  if (hasAxisWork() || hasBucketWork()) {
    return false;
  }
  if (aggHasAny) {
    emitAggregatedMove();
  }
  if (hasAxisWork() || hasBucketWork() || aggHasAny) {
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
  const previousScale = getSpeedScale();
  speedScale = value;
  logicalClockAnchorMs = logicalNow;
  logicalClockAnchorWorkerMs = now;
  if (asapMode || previousScale === getSpeedScale()) {
    return;
  }
  if (ENABLE_BUCKETED_ANTIALIASING) {
    for (const axis of axisOrder) {
      const st = axisState.get(axis);
      if (!st || st.nextWakeTimeMs === null) continue;
      const remainingMs = Math.max(0, st.nextWakeTimeMs - now);
      const logicalRemainingMs = remainingMs * previousScale;
      st.nextWakeTimeMs = now + (logicalRemainingMs / getSpeedScale());
    }
  }
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
  const now = performance.now();
  const wakeTimeNow = ENABLE_BUCKETED_ANTIALIASING ? now : currentLogicalPlaybackMs(now);
  for (const axis of axisOrder) {
    const st = axisState.get(axis);
    if (!st || st.nextWakeTimeMs === null) continue;
    st.nextWakeTimeMs = wakeTimeNow;
  }
  if (!isPaused && hasPlaybackWork()) {
    if (pacerTimer) {
      clearTimeout(pacerTimer);
      pacerTimer = null;
    }
    pacerNextDeadlineMs = now;
    scheduleNextPacer(now);
  }
};

const ensureAxisForOid = (oid) => {
  const n = Number(oid);
  if (!Number.isInteger(n) || n < 0 || n >= axisOrder.length) return undefined;
  return axisOrder[n];
};

const parseKv = (line) => {
  const out = {};
  const parts = line.split(/\s+/);
  for (const p of parts) {
    const m = p.match(/^([a-zA-Z_]+)=(.+)$/);
    if (m) {
      const k = m[1];
      let v = m[2];
      const n = Number(v);
      out[k] = Number.isNaN(n) ? v : n;
    }
  }
  return out;
};

const buildMoveLogEntry = (move, atMs, spanMs) => {
  try {
    const commandCopy = { ...move };
    const axes = {};
    for (const axis of axisOrder) {
      if (axis === EXTRUDER_AXIS) {
        axes[axis] = extruderPosMm;
      } else {
        axes[axis] = axisAngles.get(axis) || 0;
      }
    }
    const entry = {
      seq: moveLogSeq++,
      at_ms: atMs,
      span_ms: spanMs,
      axes,
      axis_units: {
        spool: 'radians',
        [EXTRUDER_AXIS]: 'millimeters',
      },
      command: commandCopy,
    };
    return entry;
  } catch (err) {
    console.error('Failed to build move log entry:', err);
    return null;
  }
};

const readyBucketThreshold = (force = false) => {
  if (force) {
    return maxBucketSeen + 1;
  }
  if (maxBucketSeen < 0) {
    return null;
  }
  if (bucketActiveAxes.size === 0) {
    return null;
  }
  let hasBlockingAxis = false;
  let minBucket = Infinity;
  for (const axis of bucketActiveAxes) {
    const st = axisState.get(axis);
    if (!st) {
      continue;
    }
    if (!st.hasSteps && st.lastTick === 0) {
      continue;
    }
    hasBlockingAxis = true;
    const bucket = Math.floor(st.lastTick / TICKS_PER_BUCKET);
    if (bucket < minBucket) {
      minBucket = bucket;
    }
  }
  if (!hasBlockingAxis) {
    return maxBucketSeen + 1;
  }
  return minBucket;
};

const emitBucketedMove = (bucketIdx) => {
  const atMs = resolveBucketAtMs(bucketIdx);
  const spanMs = scaleDelayMs(BUCKET_INTERVAL_MS);

  const addRefEntry = bucketAddToReference.get(bucketIdx);
  if (addRefEntry) {
    const addCmd = { type: 'Add to reference' };
    let hasDelta = false;
    for (const [axis, delta] of Object.entries(addRefEntry)) {
      if (!Number.isFinite(delta) || delta === 0) {
        continue;
      }
      addCmd[axis] = delta;
      hasDelta = true;
    }
    if (hasDelta) {
      postMoveCommand(addCmd, atMs, 0);
    }
    bucketAddToReference.delete(bucketIdx);
  }

  let changed = false;
  const moveCmd = { type: 'Move' };

  for (const axis of spoolAxisOrder) {
    const st = axisState.get(axis);
    if (!axisAngles.has(axis)) {
      axisAngles.set(axis, st?.baseAngle || 0);
    }
    const axisMap = bucketSteps.get(axis);
    const deltaSteps = axisMap ? axisMap.get(bucketIdx) || 0 : 0;
    if (deltaSteps !== 0) {
      const current = axisAngles.get(axis) || 0;
      const newAngle = current + deltaSteps * stepAngle;
      axisAngles.set(axis, newAngle);
      changed = true;
    }
    moveCmd[axis] = axisAngles.get(axis) || 0;
    if (axisMap) {
      axisMap.delete(bucketIdx);
      if (axisMap.size === 0) {
        bucketSteps.delete(axis);
      }
    }
  }

  const extrusionDelta = bucketExtrusion.get(bucketIdx) || 0;
  if (extrusionDelta !== 0) {
    extruderPosMm += extrusionDelta;
    moveCmd.E = extrusionDelta;
    changed = true;
  }
  if (bucketExtrusion.has(bucketIdx)) {
    bucketExtrusion.delete(bucketIdx);
  }

  if (!changed && bucketIdx === 0) {
    for (const axis of spoolAxisOrder) {
      if ((axisAngles.get(axis) || 0) !== 0) {
        changed = true;
        break;
      }
    }
  }

  if (changed) {
    postMoveCommand(moveCmd, atMs, spanMs);
  }
};

const flushReadyBuckets = (force = false, holdBackOneBucket = false) => {
  const threshold = readyBucketThreshold(force);
  if (threshold == null) {
    return;
  }
  let upperBound = Math.min(threshold, maxBucketSeen + 1);
  if (!force && holdBackOneBucket && upperBound > nextBucketToEmit) {
    upperBound -= 1;
  }
  while (nextBucketToEmit < upperBound) {
    emitBucketedMove(nextBucketToEmit);
    nextBucketToEmit += 1;
  }
};

const flushReadyBucketsIfEnabled = (force = false, holdBackOneBucket = false) => {
  if (deferBucketFlush && !force) {
    return;
  }
  flushReadyBuckets(force, holdBackOneBucket);
};

const pacerLoop = () => {
  try {
    if (isPaused) {
      return;
    }
    const workerNow = performance.now();
    const playbackNow = ENABLE_BUCKETED_ANTIALIASING
      ? workerNow
      : currentLogicalPlaybackMs(workerNow);
    let any = false;
    let loopMinStepTimeMs = Infinity;
    let loopMaxStepTimeMs = -Infinity;

    for (const axis of axisOrder) {
      const st = axisState.get(axis);
      if (!st) continue;

      let stepsApplied = 0;
      let lastStepTimeMs = null;
      while (st.nextWakeTimeMs !== null && st.nextWakeTimeMs <= playbackNow) {
        const thisStepTimeMs = st.nextWakeTimeMs;
        const stepRaw = st.nextStepClockRaw;
        stepsApplied += st.activeDirSign;
        lastStepTimeMs = workerNow;
        st.remaining -= 1;
        if (stepRaw !== null) {
          st.baseClockRaw = wrap32(stepRaw);
        }
        if (st.remaining > 0) {
          st.intervalTicks = Math.max(1, (st.intervalTicks || 1) + (st.addTicks || 0));
          const nextRaw = stepRaw !== null ? wrap32(stepRaw + st.intervalTicks) : null;
          st.nextStepClockRaw = nextRaw;
          st.nextWakeTimeMs = resolveScaledWakeTimeMs(nextRaw, ticksToMs(st.intervalTicks), thisStepTimeMs);
        } else {
          const nextSeg = st.segments.shift();
          if (nextSeg) {
            st.intervalTicks = Math.max(1, nextSeg.intervalTicks);
            st.addTicks = nextSeg.addTicks;
            st.remaining = nextSeg.remaining;
            st.activeDirSign = nextSeg.dirSign;
            const baseRaw = st.baseClockRaw !== null ? st.baseClockRaw : ensureBaseClockRaw(st);
            const nextRaw = wrap32(baseRaw + st.intervalTicks);
            st.nextStepClockRaw = nextRaw;
            st.nextWakeTimeMs = resolveScaledWakeTimeMs(nextRaw, ticksToMs(st.intervalTicks), thisStepTimeMs);
          } else {
            st.nextWakeTimeMs = null;
            st.intervalTicks = null;
            st.addTicks = 0;
            st.remaining = 0;
            st.nextStepClockRaw = null;
          }
        }
      }
      if (stepsApplied !== 0) {
        const newAngle = (axisAngles.get(axis) || 0) + stepsApplied * stepAngle;
        axisAngles.set(axis, newAngle);
        if (axis === EXTRUDER_AXIS) {
          const e_mm = stepsApplied * EXTRUDER_MM_PER_STEP_KLIPPER;
          extruderPosMm += e_mm;
          aggMove.E = (aggMove.E || 0) + e_mm;
        } else {
          aggMove[axis] = newAngle;
        }
        if (lastStepTimeMs !== null) {
          if (lastStepTimeMs < loopMinStepTimeMs) loopMinStepTimeMs = lastStepTimeMs;
          if (lastStepTimeMs > loopMaxStepTimeMs) loopMaxStepTimeMs = lastStepTimeMs;
          if (aggFirstStepTimeMs === null || lastStepTimeMs < aggFirstStepTimeMs) aggFirstStepTimeMs = lastStepTimeMs;
          if (aggLastStepTimeMs === null || lastStepTimeMs > aggLastStepTimeMs) aggLastStepTimeMs = lastStepTimeMs;
        }
        aggHasAny = true;
        any = true;
      }
    }

    while (nextBucketToEmit <= maxBucketSeen && isBucketDueNow(nextBucketToEmit, workerNow)) {
      emitBucketedMove(nextBucketToEmit);
      nextBucketToEmit += 1;
      any = true;
    }

    if (any) {
      // Extend aggregation window with this loop's min/max if available
      if (loopMinStepTimeMs < Infinity) {
        if (aggFirstStepTimeMs === null || loopMinStepTimeMs < aggFirstStepTimeMs) aggFirstStepTimeMs = loopMinStepTimeMs;
      }
      if (loopMaxStepTimeMs > -Infinity) {
        if (aggLastStepTimeMs === null || loopMaxStepTimeMs > aggLastStepTimeMs) aggLastStepTimeMs = loopMaxStepTimeMs;
      }
    }

    // Throttled emit
    if (aggHasAny) {
      const sinceLast = workerNow - lastEmitMs;
      if (sinceLast >= getMinEmitIntervalMs()) {
        emitAggregatedMove((aggLastStepTimeMs != null) ? aggLastStepTimeMs : workerNow);
      }
    }

    // Fixed-rate pacer; next wake is scheduled explicitly after this loop
  } catch (e) {
    console.error('KlipperPacer pacer error:', e);
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
    // Reset baseline so first schedule is anchored to the current clock value
    pacerNextDeadlineMs = now;
    scheduleNextPacer(now);
  }
};

const ensureBaseClockRaw = (st) => {
  if (st.baseClockRaw !== null) return st.baseClockRaw;
  const lastRaw = clockModel.getLastRawTick();
  const fallback = Number.isFinite(lastRaw) ? lastRaw : 0;
  st.baseClockRaw = wrap32(fallback);
  return st.baseClockRaw;
};

const enqueueSegment = (axis, intervalTicks, count, addTicks) => {
  const st = axisState.get(axis);
  if (!st) return;
  const seg = {
    intervalTicks: Math.max(1, Number(intervalTicks) || 1),
    addTicks: Number(addTicks) || 0,
    remaining: Math.max(0, Number(count) || 0),
    dirSign: st.dirSign,
  };

  if (seg.remaining <= 0) return;

  if (st.nextWakeTimeMs === null) {
    const baseRaw = ensureBaseClockRaw(st);
    const firstStepRaw = wrap32(baseRaw + seg.intervalTicks);
    st.intervalTicks = seg.intervalTicks;
    st.addTicks = seg.addTicks;
    st.remaining = seg.remaining;
    st.activeDirSign = seg.dirSign;
    st.nextStepClockRaw = firstStepRaw;
    const now = performance.now();
    const fallbackBaseMs = ENABLE_BUCKETED_ANTIALIASING ? now : currentLogicalPlaybackMs(now);
    st.nextWakeTimeMs = resolveScaledWakeTimeMs(firstStepRaw, ticksToMs(seg.intervalTicks), fallbackBaseMs);
  } else {
    st.segments.push(seg);
  }
  ensurePacerRunning();
};

const handleParsedLineLegacy = (line) => {
  const has = (name) => line.includes(name + ' ');
  const sliceAfter = (name) => {
    const i = line.indexOf(name);
    return i >= 0 ? line.slice(i + name.length) : '';
  };

  if (has('config_stepper')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('config_stepper'));
    const axis = ensureAxisForOid(kv.oid);
    if (axis) console.log(`Klipper map: oid ${kv.oid} -> axis ${axis}`);
    else console.log(`Klipper failed to map oid: ${kv.oid}`);
    return;
  }
  if (has('set_next_step_dir')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('set_next_step_dir'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const st = axisState.get(axis);
    if (!st) return;
    st.dirSign = (Number(kv.dir) === 0) ? -1 : 1;
    return;
  }
  if (has('reset_step_clock')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('reset_step_clock'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const st = axisState.get(axis);
    if (!st) return;
    const clk = Number(kv.clock);
    if (!Number.isFinite(clk)) return;
    st.baseClockRaw = wrap32(clk);
    st.nextStepClockRaw = null;
    st.nextWakeTimeMs = null;
    st.intervalTicks = null;
    st.addTicks = 0;
    st.remaining = 0;
    st.segments.length = 0;
    st.activeDirSign = st.dirSign;
    return;
  }
  if (has('set_position')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('set_position'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const steps = Number(kv.pos);
    if (!Number.isFinite(steps)) return;
    const newAngle = steps * stepAngle;
    const currentAngle = axisAngles.get(axis) || 0;
    const delta = newAngle - currentAngle;
    axisAngles.set(axis, newAngle);
    if (axis === EXTRUDER_AXIS) {
      extruderPosMm = steps * EXTRUDER_MM_PER_STEP_KLIPPER;
    }
    const st = axisState.get(axis);
    if (st) {
      st.segments.length = 0;
      st.nextWakeTimeMs = null;
      st.intervalTicks = null;
      st.addTicks = 0;
      st.remaining = 0;
      st.baseClockRaw = null;
      st.nextStepClockRaw = null;
      // After a set_position, align the active direction with the latest pending direction
      st.activeDirSign = st.dirSign;
    }
    postMessage({ type: 'move', command: { type: 'Add to reference', [axis]: delta, at: performance.now() } });
    // Ensure pacer is running; fixed-rate loop will pick up new schedule
    ensurePacerRunning();
    return;
  }
  if (has('queue_step')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('queue_step'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const count = Number(kv.count) || 0;
    const interval = Number(kv.interval) || 1;
    const add = ('add' in kv) ? Number(kv.add) : 0;
    enqueueSegment(axis, interval, count, add);
    return;
  }
};

const handleParsedLineBucketed = (line, nextLine = null) => {
  const has = (name) => line.includes(name + ' ');
  const sliceAfter = (name) => {
    const i = line.indexOf(name);
    return i >= 0 ? line.slice(i + name.length) : '';
  };
  const holdBackOneBucket = nextLine !== null;

  if (has('config_stepper')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('config_stepper'));
    const axis = ensureAxisForOid(kv.oid);
    if (axis) console.log(`Klipper map: oid ${kv.oid} -> axis ${axis}`);
    else console.log(`Klipper failed to map oid: ${kv.oid}`);
    return;
  }
  if (has('set_next_step_dir')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('set_next_step_dir'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const st = axisState.get(axis);
    if (!st) return;
    st.dirSign = (Number(kv.dir) === 0) ? -1 : 1;
    flushReadyBucketsIfEnabled(false, holdBackOneBucket);
    return;
  }
  if (has('reset_step_clock')) {
    if (DEBUG) console.log(line);
    flushReadyBucketsIfEnabled(false, holdBackOneBucket);
    return;
  }
  if (has('set_position')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('set_position'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const steps = Number(kv.pos);
    if (!Number.isFinite(steps)) return;
    if (axis === EXTRUDER_AXIS) {
      extruderPosMm = steps * EXTRUDER_MM_PER_STEP_KLIPPER;
      flushReadyBucketsIfEnabled(false, holdBackOneBucket);
      return;
    }
    const st = axisState.get(axis);
    if (!st) return;
    const newAngle = steps * stepAngle;
    const delta = newAngle - st.baseAngle;
    if (delta !== 0) {
      const bucketIdx = Math.floor(st.lastTick / TICKS_PER_BUCKET);
      const entry = bucketAddToReference.get(bucketIdx) || {};
      entry[axis] = (entry[axis] || 0) + delta;
      bucketAddToReference.set(bucketIdx, entry);
      updateMaxBucketSeen(bucketIdx);
    }
    st.baseAngle = newAngle;
    markBucketAxisActive(axis);
    flushReadyBucketsIfEnabled(false, holdBackOneBucket);
    return;
  }
  if (has('queue_step')) {
    if (DEBUG) console.log(line);
    const kv = parseKv(sliceAfter('queue_step'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const st = axisState.get(axis);
    if (!st) return;
    const count = Number(kv.count) || 0;
    const interval = Number(kv.interval) || 1;
    const add = ('add' in kv) ? Number(kv.add) : 0;
    if (count <= 0) {
      flushReadyBucketsIfEnabled(false, holdBackOneBucket);
      return;
    }
    if (axis === EXTRUDER_AXIS) {
      st.lastTick = recordAnchoredStepSequence({
        startTick: st.lastTick,
        intervalTicks: interval,
        count,
        addTicks: add,
        stepValue: st.dirSign * EXTRUDER_MM_PER_STEP_KLIPPER,
        bucketSize: TICKS_PER_BUCKET,
        accumulate: (bucketIdx, delta) => {
          recordBucketExtrusion(bucketIdx, delta);
        },
        setMaxBucket: updateMaxBucketSeen,
      });
    } else if (!ENABLE_BUCKETED_ANTIALIASING) {
      st.lastTick = recordAnchoredStepSequence({
        startTick: st.lastTick,
        intervalTicks: interval,
        count,
        addTicks: add,
        stepValue: st.dirSign,
        bucketSize: TICKS_PER_BUCKET,
        accumulate: (bucketIdx, delta) => {
          recordBucketSteps(axis, bucketIdx, delta);
        },
        setMaxBucket: updateMaxBucketSeen,
      });
    } else {
      const totalDurationTicks = computeQueueStepDurationTicks(interval, count, add);
      recordDistributedSequence({
        axis,
        startTick: st.lastTick,
        durationTicks: totalDurationTicks,
        totalValue: st.dirSign * count,
        bucketSize: TICKS_PER_BUCKET,
        accumulateStepBucket: (bucketAxis, bucketIdx, delta) => {
          recordBucketSteps(bucketAxis, bucketIdx, delta);
        },
        accumulateExtrusionBucket: (bucketIdx, delta) => {
          recordBucketExtrusion(bucketIdx, delta);
        },
        setMaxBucket: updateMaxBucketSeen,
      });
      st.lastTick += totalDurationTicks;
    }
    st.hasSteps = true;
    markBucketAxisActive(axis);
    flushReadyBucketsIfEnabled(false, holdBackOneBucket);
  }
};

const processParsedLine = (line, nextLine = null) => {
  handleParsedLineBucketed(line, nextLine);
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
  if (!Array.isArray(lines) || lines.length === 0) return;
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
  } catch (err) {
    console.error('KlipperPacer serial decode failed', err);
    try {
      postMessage({ type: 'error', message: err?.message || 'Serial decode failed' });
    } catch (_) {}
  }
};

const handleBinaryPayload = (payload) => {
  if (!payload) return;
  if (typeof Blob !== 'undefined' && payload instanceof Blob) {
    payload.arrayBuffer().then((buf) => feedSerialChunk(new Uint8Array(buf))).catch((err) => {
      console.error('KlipperPacer failed to read Blob payload', err);
      try {
        postMessage({ type: 'error', message: err?.message || 'Failed to read Blob payload' });
      } catch (_) {}
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
        const msg = JSON.parse(event.data);
        if (msg && msg.action === 'klipper_parsed' && Array.isArray(msg.lines)) {
          if (firstSeqSeen === null && typeof msg.seq === 'number') {
            firstSeqSeen = msg.seq;
            if (DEBUG) console.log(`first klipper_parsed seq=${firstSeqSeen} count=${msg.count}`);
          }
          if (typeof msg.seq === 'number' && typeof msg.count === 'number') {
            if (expectedSeq === null) expectedSeq = msg.seq;
            if (msg.seq !== expectedSeq && DEBUG) {
              console.error(`seq discontinuity: expected ${expectedSeq}, got ${msg.seq}`);
            }
            expectedSeq = msg.seq + msg.count;
          }
          for (const line of msg.lines) pushParsedLine(line);
          return;
        } else if (msg && msg.action === 'klipper_serial') {
          const chunk = decodeBase64Chunk(msg.data || msg.chunk || msg.payload);
          if (chunk) {
            feedSerialChunk(chunk);
          }
          return;
        } else if (msg && msg.action === 'klipper_clock') {
          const receiveMs = performance.now();
          clockModel.updateFromMessage(msg, receiveMs);
          if (clockModel.isReady()) {
            for (const axis of axisOrder) {
              const st = axisState.get(axis);
              if (!st || st.nextStepClockRaw === null || st.nextWakeTimeMs === null) continue;
              if (clockModel.mcuToWorkerMs(st.nextStepClockRaw) !== null) {
                const fallbackBaseMs = ENABLE_BUCKETED_ANTIALIASING
                  ? receiveMs
                  : currentLogicalPlaybackMs(receiveMs);
                st.nextWakeTimeMs = resolveScaledWakeTimeMs(st.nextStepClockRaw, 0, fallbackBaseMs);
              }
            }
          }
          return;
        }
      } catch (_) { /* not json, fall through */ }
      console.log(`unhandled text: ${event.data.slice(0, 100)}`);
    } else {
      handleBinaryPayload(event.data);
    }
  };
  ws.onerror = (err) => console.error('websocket error:', err);
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
  if (!maybePostDone() && !isPaused && hasPlaybackWork()) {
    ensurePacerRunning();
  }
};

const processUploadedText = async (file) => {
  const text = await file.text();
  const lines = text.split(/\r?\n/g);
  for (const rawLine of lines) {
    const line = typeof rawLine === 'string' ? rawLine.trim() : '';
    if (!line) continue;
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
    // Local uploads should start their playback clock after parsing completes,
    // otherwise high speed factors can make early buckets look overdue instantly.
    resetLogicalPlaybackClock();
    finishUploadInput({ forceBucketFlush: false });
    deferBucketFlush = false;
  } catch (err) {
    deferBucketFlush = false;
    console.error('KlipperPacer upload processing failed', err);
    postMessage({ type: 'error', message: err?.message || 'Failed to process Klipper upload' });
  }
};

self.onmessage = (e) => {
  const { type, ...data } = e.data;
  if (type === 'connect') {
    LOG_MOVE = Boolean(data.logMove);
    if (LOG_MOVE) {
      moveLogSeq = 0;
      try {
        console.log('KlipperPacer move logging enabled');
      } catch (_) { /* console may be unavailable in some worker contexts */ }
    }
    connect(data.url);
  } else if (type === 'filename_upload') {
    LOG_MOVE = Boolean(data.logMove);
    moveLogSeq = 0;
    processUploadedFile(data.filename, data.format);
  } else if (type === 'set_speed_scale') {
    setSpeedScale(data.value);
  } else if (type === 'set_asap_mode') {
    setAsapMode(data.enable);
  } else if (type === 'pause') {
    setPaused(true);
  } else if (type === 'resume') {
    setPaused(false);
  } else if (type === 'close') {
    if (ws) {
      ws.close();
    } else {
      resetRuntimeState();
      postMessage({ type: 'closed' });
    }
  }
};
