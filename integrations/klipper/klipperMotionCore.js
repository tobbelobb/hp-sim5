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

const FULL_WRAP = 0x100000000;
const HALF_WRAP = FULL_WRAP / 2;
const EXTRUDER_AXIS = 'E';

export function wrap32(value) {
  if (!Number.isFinite(value)) {
    return 0;
  }
  let next = value % FULL_WRAP;
  if (next < 0) {
    next += FULL_WRAP;
  }
  return next;
}

export function parseKlipperKv(line) {
  const out = {};
  for (const part of String(line || '').trim().split(/\s+/)) {
    const match = part.match(/^([a-zA-Z_]+)=(.+)$/);
    if (!match) {
      continue;
    }
    const [, key, rawValue] = match;
    const numeric = Number(rawValue);
    out[key] = Number.isNaN(numeric) ? rawValue : numeric;
  }
  return out;
}

export function mapOidToAxis(oid, axisOrder = DEFAULT_AXIS_ORDER) {
  const index = Number(oid);
  if (!Number.isInteger(index) || index < 0 || index >= axisOrder.length) {
    return null;
  }
  return axisOrder[index];
}

export function mapStepperNameToAxis(stepperName) {
  const normalized = String(stepperName || '').trim().toLowerCase();
  if (!normalized) {
    return null;
  }
  if (normalized === 'extruder' || normalized.startsWith('extruder_stepper')) {
    return EXTRUDER_AXIS;
  }
  const match = normalized.match(/^stepper_([a-z])$/);
  if (!match) {
    return null;
  }
  return match[1].toUpperCase();
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

export class KlipperClockModel {
  constructor({ clockHz = MCU_CLOCK_HZ_KLIPPER_HOST } = {}) {
    this.defaultClockHz = clockHz;
    this.reset();
  }

  reset() {
    this.clockHz = this.defaultClockHz;
    this.lastRawTick = null;
    this.lastMcuTick = null;
    this.sampleWorkerMs = null;
  }

  isReady() {
    return this.lastRawTick !== null
      && this.lastMcuTick !== null
      && this.sampleWorkerMs !== null;
  }

  getCurrentMcuTick() {
    return this.lastMcuTick;
  }

  ticksToMs(ticks) {
    return (Number(ticks) || 0) / this.clockHz * 1000.0;
  }

  updateFromMessage(message, receiveMs) {
    if (message && Number.isFinite(message.clock_hz) && message.clock_hz > 0) {
      this.clockHz = message.clock_hz;
    }
    const rawTick = Number(message?.mcu_clock);
    if (!Number.isFinite(rawTick)) {
      return null;
    }
    const unwrapped = this._updateUnwrapped(rawTick);
    this.sampleWorkerMs = receiveMs;
    return unwrapped;
  }

  unwrapRawClock(rawTick) {
    if (!Number.isFinite(rawTick)) {
      return null;
    }
    if (this.lastRawTick === null || this.lastMcuTick === null) {
      return rawTick;
    }
    let diff = rawTick - this.lastRawTick;
    if (diff > HALF_WRAP) {
      diff -= FULL_WRAP;
    } else if (diff < -HALF_WRAP) {
      diff += FULL_WRAP;
    }
    return this.lastMcuTick + diff;
  }

  unwrappedToWorkerMs(unwrappedTick) {
    if (!Number.isFinite(unwrappedTick)
      || !Number.isFinite(this.lastMcuTick)
      || !Number.isFinite(this.sampleWorkerMs)) {
      return null;
    }
    return this.sampleWorkerMs + ((unwrappedTick - this.lastMcuTick) / this.clockHz) * 1000.0;
  }

  mcuToWorkerMs(rawTick) {
    const unwrapped = this.unwrapRawClock(rawTick);
    if (!Number.isFinite(unwrapped)) {
      return null;
    }
    return this.unwrappedToWorkerMs(unwrapped);
  }

  _updateUnwrapped(rawTick) {
    if (this.lastRawTick === null || this.lastMcuTick === null) {
      this.lastRawTick = rawTick;
      this.lastMcuTick = rawTick;
      return this.lastMcuTick;
    }
    let diff = rawTick - this.lastRawTick;
    if (diff > HALF_WRAP) {
      diff -= FULL_WRAP;
    } else if (diff < -HALF_WRAP) {
      diff += FULL_WRAP;
    }
    this.lastRawTick = rawTick;
    this.lastMcuTick += diff;
    return this.lastMcuTick;
  }
}

export class KlipperBucketedMotionCore {
  constructor({
    axisOrder = DEFAULT_AXIS_ORDER,
    bucketIntervalMs = 2,
    clockHz = MCU_CLOCK_HZ_KLIPPER_HOST,
    enableBucketedAntialiasing = true,
  } = {}) {
    this.axisOrder = [...axisOrder];
    this.bucketIntervalMs = bucketIntervalMs;
    this.clockHz = clockHz;
    this.enableBucketedAntialiasing = enableBucketedAntialiasing;
    this.spoolAxisOrder = this.axisOrder.filter((axis) => axis !== EXTRUDER_AXIS);
    this.ticksPerBucket = Math.max(1, Math.round(this.clockHz * (this.bucketIntervalMs / 1000)));
    this.reset();
  }

  reset() {
    this.axisStates = new Map(this.axisOrder.map((axis) => [axis, {
      dirSign: 1,
      lastTick: 0,
      baseAngle: 0,
      hasSteps: false,
    }]));
    this.axisAngles = new Map(this.axisOrder.map((axis) => [axis, 0.0]));
    this.bucketSteps = new Map();
    this.bucketExtrusion = new Map();
    this.bucketAddToReference = new Map();
    this.bucketActiveAxes = new Set();
    this.nextBucketToEmit = 0;
    this.maxBucketSeen = -1;
    this.extruderPosMm = 0;
  }

  hasPendingBuckets() {
    return this.nextBucketToEmit <= this.maxBucketSeen;
  }

  getMaxKnownTick() {
    let maxTick = null;
    for (const state of this.axisStates.values()) {
      if (!state || !state.hasSteps) {
        continue;
      }
      if (!Number.isFinite(maxTick) || state.lastTick > maxTick) {
        maxTick = state.lastTick;
      }
    }
    return maxTick;
  }

  getTicksPerBucket() {
    return this.ticksPerBucket;
  }

  getBucketEndTick(bucketIdx) {
    return (bucketIdx + 1) * this.ticksPerBucket;
  }

  getCurrentAxes() {
    const axes = {};
    for (const axis of this.spoolAxisOrder) {
      axes[axis] = this.axisAngles.get(axis) || 0;
    }
    axes[EXTRUDER_AXIS] = this.extruderPosMm;
    return axes;
  }

  setAxisDirection(axis, dirSign) {
    const state = this._ensureAxisState(axis);
    if (!state) {
      return;
    }
    state.dirSign = dirSign >= 0 ? 1 : -1;
  }

  setAxisReferencePosition(axis, absoluteValue, tick = null) {
    const state = this._ensureAxisState(axis);
    if (!state) {
      return;
    }
    if (axis === EXTRUDER_AXIS) {
      this.extruderPosMm = absoluteValue;
      return;
    }
    const delta = absoluteValue - state.baseAngle;
    if (delta !== 0) {
      const referenceTick = Number.isFinite(tick) ? tick : state.lastTick;
      const bucketIdx = Math.floor(referenceTick / this.ticksPerBucket);
      const entry = this.bucketAddToReference.get(bucketIdx) || {};
      entry[axis] = (entry[axis] || 0) + delta;
      this.bucketAddToReference.set(bucketIdx, entry);
      this._updateMaxBucketSeen(bucketIdx);
    }
    state.baseAngle = absoluteValue;
    this._markAxisActive(axis);
  }

  recordQueueSteps({
    axis,
    startTick = null,
    intervalTicks,
    count,
    addTicks = 0,
    useCurrentDirection = false,
  }) {
    const state = this._ensureAxisState(axis);
    if (!state) {
      return state?.lastTick ?? 0;
    }
    const signedCount = Number(count) || 0;
    const stepCount = Math.abs(signedCount);
    const originTick = Number.isFinite(startTick) ? startTick : state.lastTick;
    state.lastTick = originTick;
    if (stepCount <= 0) {
      return state.lastTick;
    }

    const direction = signedCount < 0
      ? -1
      : (useCurrentDirection ? state.dirSign : 1);
    if (axis === EXTRUDER_AXIS) {
      state.lastTick = recordAnchoredStepSequence({
        startTick: originTick,
        intervalTicks,
        count: stepCount,
        addTicks,
        stepValue: direction * EXTRUDER_MM_PER_STEP_KLIPPER,
        bucketSize: this.ticksPerBucket,
        accumulate: (bucketIdx, delta) => {
          this.bucketExtrusion.set(bucketIdx, (this.bucketExtrusion.get(bucketIdx) || 0) + delta);
        },
        setMaxBucket: (bucketIdx) => this._updateMaxBucketSeen(bucketIdx),
      });
    } else if (!this.enableBucketedAntialiasing) {
      state.lastTick = recordAnchoredStepSequence({
        startTick: originTick,
        intervalTicks,
        count: stepCount,
        addTicks,
        stepValue: direction,
        bucketSize: this.ticksPerBucket,
        accumulate: (bucketIdx, delta) => {
          const bucketMap = this._ensureBucketMap(axis);
          bucketMap.set(bucketIdx, (bucketMap.get(bucketIdx) || 0) + delta);
        },
        setMaxBucket: (bucketIdx) => this._updateMaxBucketSeen(bucketIdx),
      });
    } else {
      const totalDurationTicks = computeQueueStepDurationTicks(intervalTicks, stepCount, addTicks);
      recordDistributedSequence({
        axis,
        startTick: originTick,
        durationTicks: totalDurationTicks,
        totalValue: direction * stepCount,
        bucketSize: this.ticksPerBucket,
        accumulateStepBucket: (bucketAxis, bucketIdx, delta) => {
          const bucketMap = this._ensureBucketMap(bucketAxis);
          bucketMap.set(bucketIdx, (bucketMap.get(bucketIdx) || 0) + delta);
        },
        accumulateExtrusionBucket: (bucketIdx, delta) => {
          this.bucketExtrusion.set(bucketIdx, (this.bucketExtrusion.get(bucketIdx) || 0) + delta);
        },
        setMaxBucket: (bucketIdx) => this._updateMaxBucketSeen(bucketIdx),
      });
      state.lastTick = originTick + totalDurationTicks;
    }

    state.hasSteps = true;
    this._markAxisActive(axis);
    return state.lastTick;
  }

  consumeStepperBatch({
    axis = null,
    stepperName = null,
    firstClock,
    startMcuPosition,
    data = [],
  }) {
    const resolvedAxis = axis || mapStepperNameToAxis(stepperName);
    const state = this._ensureAxisState(resolvedAxis);
    if (!state) {
      return;
    }
    const startTick = Number(firstClock);
    if (!Number.isFinite(startTick)) {
      return;
    }

    const absolutePosition = resolvedAxis === EXTRUDER_AXIS
      ? (Number(startMcuPosition) || 0) * EXTRUDER_MM_PER_STEP_KLIPPER
      : (Number(startMcuPosition) || 0) * STEP_ANGLE_RAD;
    if (!state.hasSteps) {
      this.setAxisReferencePosition(resolvedAxis, absolutePosition, startTick);
    }
    state.lastTick = startTick;

    let nextTick = startTick;
    for (const rawEntry of Array.isArray(data) ? data : []) {
      const entry = normalizeBatchEntry(rawEntry);
      if (resolvedAxis !== EXTRUDER_AXIS) {
        this.setAxisDirection(resolvedAxis, entry.count < 0 ? -1 : 1);
      }
      nextTick = this.recordQueueSteps({
        axis: resolvedAxis,
        startTick: nextTick,
        intervalTicks: entry.interval,
        count: resolvedAxis === EXTRUDER_AXIS ? entry.count : Math.abs(entry.count),
        addTicks: entry.add,
        useCurrentDirection: resolvedAxis !== EXTRUDER_AXIS,
      });
    }
  }

  flushCommands({
    force = false,
    forceThreshold = false,
    holdBackOneBucket = false,
    canEmitBucket = null,
    buildTiming = null,
  } = {}) {
    const threshold = this._readyBucketThreshold(force || forceThreshold);
    if (threshold == null) {
      return [];
    }
    let upperBound = Math.min(threshold, this.maxBucketSeen + 1);
    if (!force && holdBackOneBucket && upperBound > this.nextBucketToEmit) {
      upperBound -= 1;
    }

    const emitted = [];
    while (this.nextBucketToEmit < upperBound) {
      if (typeof canEmitBucket === 'function' && !canEmitBucket(this.nextBucketToEmit)) {
        break;
      }
      const timing = typeof buildTiming === 'function'
        ? buildTiming(this.nextBucketToEmit) || {}
        : {};
      emitted.push(...this._emitBucket(this.nextBucketToEmit, timing));
      this.nextBucketToEmit += 1;
    }
    return emitted;
  }

  _ensureAxisState(axis) {
    if (!axis) {
      return null;
    }
    let state = this.axisStates.get(axis);
    if (!state) {
      state = {
        dirSign: 1,
        lastTick: 0,
        baseAngle: 0,
        hasSteps: false,
      };
      this.axisStates.set(axis, state);
      this.axisAngles.set(axis, 0.0);
      if (axis !== EXTRUDER_AXIS && !this.spoolAxisOrder.includes(axis)) {
        this.spoolAxisOrder.push(axis);
      }
    }
    return state;
  }

  _ensureBucketMap(axis) {
    let bucketMap = this.bucketSteps.get(axis);
    if (!bucketMap) {
      bucketMap = new Map();
      this.bucketSteps.set(axis, bucketMap);
    }
    return bucketMap;
  }

  _markAxisActive(axis) {
    if (!axis || axis === EXTRUDER_AXIS) {
      return;
    }
    this.bucketActiveAxes.add(axis);
  }

  _readyBucketThreshold(force = false) {
    if (force) {
      return this.maxBucketSeen + 1;
    }
    if (this.maxBucketSeen < 0 || this.bucketActiveAxes.size === 0) {
      return null;
    }
    let hasBlockingAxis = false;
    let minBucket = Infinity;
    for (const axis of this.bucketActiveAxes) {
      const state = this.axisStates.get(axis);
      if (!state) {
        continue;
      }
      if (!state.hasSteps && state.lastTick === 0) {
        continue;
      }
      hasBlockingAxis = true;
      const bucketIdx = Math.floor(state.lastTick / this.ticksPerBucket);
      if (bucketIdx < minBucket) {
        minBucket = bucketIdx;
      }
    }
    if (!hasBlockingAxis) {
      return this.maxBucketSeen + 1;
    }
    return minBucket;
  }

  _emitBucket(bucketIdx, timing) {
    const scheduled = [];
    const at = Number.isFinite(timing?.at) ? timing.at : 0;
    const span = Number.isFinite(timing?.span) ? timing.span : 0;

    const addRefEntry = this.bucketAddToReference.get(bucketIdx);
    if (addRefEntry) {
      const addCommand = { type: 'Add to reference' };
      let hasDelta = false;
      for (const [axis, delta] of Object.entries(addRefEntry)) {
        if (!Number.isFinite(delta) || delta === 0) {
          continue;
        }
        addCommand[axis] = delta;
        hasDelta = true;
      }
      if (hasDelta) {
        scheduled.push({
          ...addCommand,
          at,
          span: 0,
        });
      }
      this.bucketAddToReference.delete(bucketIdx);
    }

    let changed = false;
    const moveCommand = { type: 'Move' };
    for (const axis of this.spoolAxisOrder) {
      const bucketMap = this.bucketSteps.get(axis);
      const deltaSteps = bucketMap ? bucketMap.get(bucketIdx) || 0 : 0;
      if (deltaSteps !== 0) {
        const currentAngle = this.axisAngles.get(axis) || 0;
        const nextAngle = currentAngle + deltaSteps * STEP_ANGLE_RAD;
        this.axisAngles.set(axis, nextAngle);
        changed = true;
      }
      moveCommand[axis] = this.axisAngles.get(axis) || 0;
      if (bucketMap) {
        bucketMap.delete(bucketIdx);
        if (bucketMap.size === 0) {
          this.bucketSteps.delete(axis);
        }
      }
    }

    const extrusionDelta = this.bucketExtrusion.get(bucketIdx) || 0;
    if (extrusionDelta !== 0) {
      this.extruderPosMm += extrusionDelta;
      moveCommand[EXTRUDER_AXIS] = extrusionDelta;
      changed = true;
    }
    if (this.bucketExtrusion.has(bucketIdx)) {
      this.bucketExtrusion.delete(bucketIdx);
    }

    if (!changed && bucketIdx === 0) {
      for (const axis of this.spoolAxisOrder) {
        if ((this.axisAngles.get(axis) || 0) !== 0) {
          changed = true;
          break;
        }
      }
    }

    if (changed) {
      scheduled.push({
        ...moveCommand,
        at,
        span,
      });
    }
    return scheduled;
  }

  _updateMaxBucketSeen(bucketIdx) {
    this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
  }
}

export function applyBucketedParsedLine(core, line) {
  const text = String(line || '');
  const hasCommand = (name) => text.includes(`${name} `);
  const sliceAfter = (name) => {
    const index = text.indexOf(name);
    return index >= 0 ? text.slice(index + name.length) : '';
  };

  if (hasCommand('config_stepper')) {
    return;
  }
  if (hasCommand('set_next_step_dir')) {
    const kv = parseKlipperKv(sliceAfter('set_next_step_dir'));
    const axis = mapOidToAxis(kv.oid, core.axisOrder);
    if (axis) {
      core.setAxisDirection(axis, Number(kv.dir) === 0 ? -1 : 1);
    }
    return;
  }
  if (hasCommand('reset_step_clock')) {
    return;
  }
  if (hasCommand('set_position')) {
    const kv = parseKlipperKv(sliceAfter('set_position'));
    const axis = mapOidToAxis(kv.oid, core.axisOrder);
    if (!axis) {
      return;
    }
    const steps = Number(kv.pos);
    if (!Number.isFinite(steps)) {
      return;
    }
    const absolutePosition = axis === EXTRUDER_AXIS
      ? steps * EXTRUDER_MM_PER_STEP_KLIPPER
      : steps * STEP_ANGLE_RAD;
    const axisState = core.axisStates.get(axis);
    core.setAxisReferencePosition(axis, absolutePosition, axisState?.lastTick ?? 0);
    return;
  }
  if (hasCommand('queue_step')) {
    const kv = parseKlipperKv(sliceAfter('queue_step'));
    const axis = mapOidToAxis(kv.oid, core.axisOrder);
    if (!axis) {
      return;
    }
    const axisState = core.axisStates.get(axis);
    core.recordQueueSteps({
      axis,
      startTick: axisState?.lastTick ?? 0,
      intervalTicks: Number(kv.interval) || 1,
      count: Number(kv.count) || 0,
      addTicks: 'add' in kv ? Number(kv.add) || 0 : 0,
      useCurrentDirection: true,
    });
  }
}
