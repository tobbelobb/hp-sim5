import { createKlipperSerialDecoder, SerialLineDecoder, decodeBase64Chunk } from './klipperSerialParser.js';
import { normalizeSpeedScale, resolveRawMoveDispatchTime, scaleDurationMs } from '../../../autocal/control/primitives/klipper_live_timing.mjs';

// This is a worker script for Klipper pacing.

// --- Globals for worker state ---
let ws = null;
let DEBUG = false;
let firstSeqSeen = null; // Debug: first server seq index we receive
let expectedSeq = null;  // Debug: detect gaps in parsed-line stream

const axisOrder = ['A', 'B', 'C', 'D', 'E'];
const axisAngles = new Map(axisOrder.map(a => [a, 0.0]));
const stepsPerRev = 200;
const microsteps = 16;
const stepAngle = (2 * Math.PI) / (stepsPerRev * microsteps);
// Treat oid 4 as the extruder stepper and convert its steps to filament mm.
// Use rotation_distance from the example config (33.5 mm per full rotation).
// mm per microstep = rotation_distance / (stepsPerRev * microsteps)
const EXTRUDER_OID = 4; // assumed stable mapping
const EXTRUDER_AXIS = 'E';
const EXTRUDER_ROTATION_DISTANCE_MM = 33.5;
const EXTRUDER_MM_PER_STEP = EXTRUDER_ROTATION_DISTANCE_MM / (stepsPerRev * microsteps);
let extruderPosMm = 0;

// Throttle outgoing Move messages to avoid overwhelming the browser.
// Pacer will run at this interval and emit at most once per interval.
const PACER_INTERVAL_MS = 0.50;
const MIN_EMIT_INTERVAL_MS = PACER_INTERVAL_MS;

// Aggregation buffer across pacer ticks
let lastEmitMs = 0;
let aggMove = { type: 'Move' };
let aggFirstStepTimeMs = null;
let aggLastStepTimeMs = null;
let aggHasAny = false;

let LOG_MOVE = false;
let moveLogSeq = 0;
let speedScale = 1.0;

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
    return this.sampleWorkerMs + scaleDurationMs((deltaTicks / this.clockHz) * 1000.0, speedScale);
  }
}

const clockModel = new ClockModel();
const ticksToMs = (ticks) => clockModel.ticksToMs(ticks);
let pacerTimer = null;       // setTimeout handle for pacer
let pacerNextDeadlineMs = null; // Absolute deadline for the next pacer wakeup

const axisState = new Map(axisOrder.map(a => [a, {
  segments: [],
  nextWakeTimeMs: null,
  intervalTicks: null,
  addTicks: 0,
  remaining: 0,
  dirSign: 1,
  // Direction used for the currently executing segment
  activeDirSign: 1,
  baseClockRaw: null,
  nextStepClockRaw: null,
}]));

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

const pacerLoop = () => {
  try {
    const now = performance.now();
    let any = false;
    let loopMinStepTimeMs = Infinity;
    let loopMaxStepTimeMs = -Infinity;

    for (const axis of axisOrder) {
      const st = axisState.get(axis);
      if (!st) continue;

      let stepsApplied = 0;
      let lastStepTimeMs = null;
      while (st.nextWakeTimeMs !== null && st.nextWakeTimeMs <= now) {
        const thisStepTimeMs = st.nextWakeTimeMs;
        const stepRaw = st.nextStepClockRaw;
        stepsApplied += st.activeDirSign;
        lastStepTimeMs = thisStepTimeMs;
        st.remaining -= 1;
        if (stepRaw !== null) {
          st.baseClockRaw = wrap32(stepRaw);
        }
        if (st.remaining > 0) {
          st.intervalTicks = Math.max(1, (st.intervalTicks || 1) + (st.addTicks || 0));
          const nextRaw = stepRaw !== null ? wrap32(stepRaw + st.intervalTicks) : null;
          st.nextStepClockRaw = nextRaw;
          const mapped = nextRaw !== null ? clockModel.mcuToWorkerMs(nextRaw) : null;
          st.nextWakeTimeMs = mapped !== null ? mapped : thisStepTimeMs + ticksToMs(st.intervalTicks);
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
            const mapped = clockModel.mcuToWorkerMs(nextRaw);
            st.nextWakeTimeMs = mapped !== null ? mapped : thisStepTimeMs + ticksToMs(st.intervalTicks);
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
          const e_mm = stepsApplied * EXTRUDER_MM_PER_STEP;
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
      const sinceLast = now - lastEmitMs;
      if (sinceLast >= MIN_EMIT_INTERVAL_MS) {
        const atMs = resolveRawMoveDispatchTime(aggFirstStepTimeMs, now);
        const spanMs = (aggFirstStepTimeMs != null && aggLastStepTimeMs != null)
          ? Math.max(0, aggLastStepTimeMs - aggFirstStepTimeMs)
          : 0;
        aggMove.at = atMs;
        aggMove.span = spanMs;
        const logEntry = LOG_MOVE ? buildMoveLogEntry(aggMove, atMs, spanMs) : null;
        if (LOG_MOVE && logEntry) {
          postMessage({ type: 'move', command: aggMove, logEntry });
        } else {
          postMessage({ type: 'move', command: aggMove });
        }
        // Reset aggregation
        aggMove = { type: 'Move' };
        aggFirstStepTimeMs = null;
        aggLastStepTimeMs = null;
        aggHasAny = false;
        lastEmitMs = now;
      }
    }

    // Fixed-rate pacer; next wake is scheduled explicitly after this loop
  } catch (e) {
    console.error('KlipperPacer pacer error:', e);
  }
};

const scheduleNextPacer = (now = performance.now()) => {
  if (pacerNextDeadlineMs === null) {
    pacerNextDeadlineMs = now + PACER_INTERVAL_MS;
  } else {
    while (pacerNextDeadlineMs <= now) {
      pacerNextDeadlineMs += PACER_INTERVAL_MS;
    }
  }
  const delay = Math.max(0, pacerNextDeadlineMs - now);
  pacerTimer = setTimeout(pacerTick, delay);
};

const pacerTick = () => {
  pacerTimer = null;
  pacerLoop();
  scheduleNextPacer();
};

const ensurePacerRunning = () => {
  if (pacerTimer == null) {
    const now = performance.now();
    // Reset baseline so first schedule is anchored to the current clock value
    pacerNextDeadlineMs = now;
    scheduleNextPacer(now);
  }
};

const remapScheduledWakeTimes = () => {
  const now = performance.now();
  for (const axis of axisOrder) {
    const st = axisState.get(axis);
    if (!st) continue;
    if (st.nextStepClockRaw !== null && clockModel.isReady()) {
      const mapped = clockModel.mcuToWorkerMs(st.nextStepClockRaw);
      if (mapped !== null) {
        st.nextWakeTimeMs = mapped;
        continue;
      }
    }
    if (st.nextWakeTimeMs !== null) {
      st.nextWakeTimeMs = now + scaleDurationMs(st.nextWakeTimeMs - now, speedScale);
    }
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
    const mapped = clockModel.mcuToWorkerMs(firstStepRaw);
    const now = performance.now();
    st.nextWakeTimeMs = mapped !== null ? mapped : now + scaleDurationMs(ticksToMs(seg.intervalTicks), speedScale);
  } else {
    st.segments.push(seg);
  }
  ensurePacerRunning();
};

const handleParsedLine = (line) => {
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
      extruderPosMm = steps * EXTRUDER_MM_PER_STEP;
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

const processSerialLines = (lines) => {
  if (!Array.isArray(lines) || lines.length === 0) return;
  for (const line of lines) {
    if (typeof line === 'string' && line.length > 0) {
      handleParsedLine(line);
    }
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
  serialLineDecoder = null;
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
          for (const line of msg.lines) handleParsedLine(line);
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
              const mapped = clockModel.mcuToWorkerMs(st.nextStepClockRaw);
              if (mapped !== null) {
                st.nextWakeTimeMs = mapped;
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
    if (pacerTimer) {
      clearTimeout(pacerTimer);
      pacerTimer = null;
    }
    pacerNextDeadlineMs = null;
    clockModel.reset();
    for (const axis of axisOrder) {
      const st = axisState.get(axis);
      if (!st) continue;
      st.segments.length = 0;
      st.nextWakeTimeMs = null;
      st.intervalTicks = null;
      st.addTicks = 0;
      st.remaining = 0;
      st.dirSign = 1;
      st.activeDirSign = 1;
      st.baseClockRaw = null;
      st.nextStepClockRaw = null;
    }
    aggMove = { type: 'Move' };
    aggFirstStepTimeMs = null;
    aggLastStepTimeMs = null;
    aggHasAny = false;
    if (serialLineDecoder) {
      const remaining = serialLineDecoder.flush();
      processSerialLines(remaining);
    }
    serialLineDecoder = null;
    postMessage({ type: 'closed' });
  };
};

self.onmessage = (e) => {
  const { type, ...data } = e.data;
  if (type === 'connect') {
    LOG_MOVE = Boolean(data.logMove);
    speedScale = normalizeSpeedScale(data.speedScale, 1.0);
    if (LOG_MOVE) {
      moveLogSeq = 0;
      try {
        console.log('KlipperPacer move logging enabled');
      } catch (_) { /* console may be unavailable in some worker contexts */ }
    }
    connect(data.url);
  } else if (type === 'close') {
    if (ws) ws.close();
  } else if (type === 'set_speed_scale') {
    const nextScale = normalizeSpeedScale(data.value, 1.0);
    if (nextScale !== speedScale) {
      speedScale = nextScale;
      remapScheduledWakeTimes();
    }
  }
};
