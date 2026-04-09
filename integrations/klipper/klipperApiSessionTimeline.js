import { computeQueueStepDurationTicks } from './motionUtils.js';

export const API_MOTION_STARTUP_BUFFER_MS = 1000;

function normalizeBatchTime(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : null;
}

function getStepperFirstTime(batch = {}) {
  return normalizeBatchTime(batch.first_time ?? batch.first_step_time);
}

function getStepperLastTime(batch = {}) {
  return normalizeBatchTime(batch.last_time ?? batch.last_step_time);
}

function getTrapqFirstTime(batch = {}) {
  const first = Array.isArray(batch.data) ? batch.data[0] : null;
  return Array.isArray(first) ? normalizeBatchTime(first[0]) : null;
}

function getTrapqLastTime(batch = {}) {
  const last = Array.isArray(batch.data) ? batch.data.at(-1) : null;
  if (!Array.isArray(last)) {
    return null;
  }
  const start = normalizeBatchTime(last[0]);
  const duration = normalizeBatchTime(last[1]);
  if (!Number.isFinite(start)) {
    return null;
  }
  return start + (Number.isFinite(duration) ? duration : 0);
}

function normalizeBatchEntry(entry) {
  if (Array.isArray(entry)) {
    return {
      interval: Number(entry[0]) || 0,
      count: Number(entry[1]) || 0,
      add: Number(entry[2]) || 0,
    };
  }
  if (entry && typeof entry === 'object') {
    return {
      interval: Number(entry.interval) || 0,
      count: Number(entry.count) || 0,
      add: Number(entry.add) || 0,
    };
  }
  return {
    interval: 0,
    count: 0,
    add: 0,
  };
}

function splitFirstEntryWithAdjustedInterval(rawData, adjustedFirstInterval) {
  const entries = Array.isArray(rawData) ? rawData.map((entry) => normalizeBatchEntry(entry)) : [];
  if (entries.length === 0) {
    return [];
  }
  const first = entries[0];
  const signedCount = Number(first.count) || 0;
  const direction = signedCount < 0 ? -1 : 1;
  const absCount = Math.abs(signedCount);
  const next = [];
  const normalizedFirstInterval = Math.max(1, Math.round(adjustedFirstInterval));

  if (absCount <= 0) {
    return entries.slice(1).map((entry) => [entry.interval, entry.count, entry.add]);
  }

  if (absCount === 1) {
    next.push([normalizedFirstInterval, signedCount, 0]);
  } else {
    next.push([normalizedFirstInterval, direction, 0]);
    next.push([
      Math.max(1, first.interval + first.add),
      direction * (absCount - 1),
      first.add,
    ]);
  }

  for (const entry of entries.slice(1)) {
    next.push([entry.interval, entry.count, entry.add]);
  }
  return next;
}

function computeBatchLastTick(startTick, data = []) {
  let tick = Number(startTick) || 0;
  for (const rawEntry of Array.isArray(data) ? data : []) {
    const entry = normalizeBatchEntry(rawEntry);
    const stepCount = Math.abs(entry.count);
    if (stepCount <= 0) {
      continue;
    }
    tick += computeQueueStepDurationTicks(entry.interval, stepCount, entry.add);
  }
  return tick;
}

export function rebaseApiStepperBatch(rawBatch = {}, {
  sessionStartTime = null,
  clockHz,
  previousLastStepTick = null,
} = {}) {
  const data = Array.isArray(rawBatch.data) ? rawBatch.data : [];
  if (data.length === 0) {
    return null;
  }

  const stepperFirstTime = getStepperFirstTime(rawBatch);
  const baseOriginTick = Number.isFinite(previousLastStepTick) ? previousLastStepTick : 0;
  let adjustedFirstInterval = null;

  if (Number.isFinite(stepperFirstTime) && Number.isFinite(sessionStartTime)) {
    const firstStepTick = Math.round((stepperFirstTime - sessionStartTime) * clockHz);
    adjustedFirstInterval = Math.max(1, firstStepTick - baseOriginTick);
  } else {
    adjustedFirstInterval = Math.max(1, Number(rawBatch.data?.[0]?.[0]) || 0);
  }

  const rebasedData = splitFirstEntryWithAdjustedInterval(data, adjustedFirstInterval);
  if (rebasedData.length === 0) {
    return null;
  }

  return {
    ...rawBatch,
    first_clock: baseOriginTick,
    data: rebasedData,
    timeline_start_tick: baseOriginTick,
    timeline_last_tick: computeBatchLastTick(baseOriginTick, rebasedData),
  };
}

export class KlipperApiSessionTimelineBuffer {
  constructor({
    clockHz,
    startupBufferMs = API_MOTION_STARTUP_BUFFER_MS,
  } = {}) {
    this.defaultClockHz = clockHz;
    this.startupBufferMs = startupBufferMs;
    this.reset({ clockHz });
  }

  reset({ clockHz = this.defaultClockHz } = {}) {
    this.clockHz = Number.isFinite(clockHz) && clockHz > 0 ? clockHz : this.defaultClockHz;
    this.pendingStepperBatches = [];
    this.sessionStartTime = null;
    this.fallbackStartTime = null;
    this.maxObservedTime = null;
    this.started = false;
    this.lastTickByStepper = new Map();
  }

  setClockHz(clockHz) {
    if (Number.isFinite(clockHz) && clockHz > 0) {
      this.clockHz = clockHz;
      if (!Number.isFinite(this.defaultClockHz)) {
        this.defaultClockHz = clockHz;
      }
    }
  }

  consumeTrapqBatch(batch = {}) {
    const firstTime = getTrapqFirstTime(batch);
    if (Number.isFinite(firstTime)) {
      this.sessionStartTime = Number.isFinite(this.sessionStartTime)
        ? Math.min(this.sessionStartTime, firstTime)
        : firstTime;
    }
    const lastTime = getTrapqLastTime(batch);
    if (Number.isFinite(lastTime)) {
      this.maxObservedTime = Number.isFinite(this.maxObservedTime)
        ? Math.max(this.maxObservedTime, lastTime)
        : lastTime;
    }
  }

  consumeStepperBatch(batch = {}) {
    this.pendingStepperBatches.push(batch);
    const firstTime = getStepperFirstTime(batch);
    if (Number.isFinite(firstTime)) {
      this.fallbackStartTime = Number.isFinite(this.fallbackStartTime)
        ? Math.min(this.fallbackStartTime, firstTime)
        : firstTime;
    }
    const lastTime = getStepperLastTime(batch) ?? firstTime;
    if (Number.isFinite(lastTime)) {
      this.maxObservedTime = Number.isFinite(this.maxObservedTime)
        ? Math.max(this.maxObservedTime, lastTime)
        : lastTime;
    }
  }

  hasPendingStepperBatches() {
    return this.pendingStepperBatches.length > 0;
  }

  getBufferedDurationMs() {
    const startTime = this._resolveStartTime(false);
    if (!Number.isFinite(startTime) || !Number.isFinite(this.maxObservedTime)) {
      return 0;
    }
    return Math.max(0, (this.maxObservedTime - startTime) * 1000);
  }

  canStart(force = false) {
    if (this.started) {
      return true;
    }
    if (!this.hasPendingStepperBatches()) {
      return false;
    }
    if (force) {
      return true;
    }
    const startTime = this._resolveStartTime(false);
    if (!Number.isFinite(startTime)) {
      return false;
    }
    return this.getBufferedDurationMs() >= this.startupBufferMs;
  }

  flushReady({ forceStart = false } = {}) {
    if (!this.canStart(forceStart)) {
      return [];
    }
    this.started = true;
    this.sessionStartTime = this._resolveStartTime(true);
    const ready = [];
    while (this.pendingStepperBatches.length > 0) {
      const rawBatch = this.pendingStepperBatches.shift();
      const previousLastStepTick = this.lastTickByStepper.get(rawBatch.name) ?? null;
      const rebased = rebaseApiStepperBatch(rawBatch, {
        sessionStartTime: this.sessionStartTime,
        clockHz: this.clockHz,
        previousLastStepTick,
      });
      if (!rebased) {
        continue;
      }
      this.lastTickByStepper.set(rawBatch.name, rebased.timeline_last_tick);
      ready.push(rebased);
    }
    return ready;
  }

  _resolveStartTime(force) {
    if (Number.isFinite(this.sessionStartTime)) {
      return this.sessionStartTime;
    }
    if (force && Number.isFinite(this.fallbackStartTime)) {
      return this.fallbackStartTime;
    }
    return null;
  }
}
