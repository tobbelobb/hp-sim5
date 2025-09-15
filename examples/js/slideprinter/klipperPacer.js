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

let clockHz = 50_000_000; // Default; will auto-update from bridge
const ticksToMs = (ticks) => (ticks / clockHz) * 1000.0;
const bufferAheadMs = 15.0; // Buffer to smooth out network jitter
const pacerIntervalMs = 1.0; // engine can consume at max 500 Hz
let pacerTimer = null;       // setInterval handle for pacer
let startedBaseTimeMs = null;
let firstTickOffset = null;     // Global baseline: min first-interval across all axes
let hasEmittedAnyStep = false;  // Lock baseline once stepping begins

const axisState = new Map(axisOrder.map(a => [a, {
  segments: [],
  nextWakeTimeMs: null,
  intervalTicks: null,
  addTicks: 0,
  remaining: 0,
  dirSign: 1,
  // Direction used for the currently executing segment
  activeDirSign: 1,
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

const pacerLoop = () => {
  try {
    const now = performance.now();
    const move = { type: 'Move' };
    let any = false;

    for (const axis of axisOrder) {
      const st = axisState.get(axis);
      if (!st) continue;

      let stepsApplied = 0;
      while (st.nextWakeTimeMs !== null && st.nextWakeTimeMs <= now) {
        // Step using the direction captured for the active segment
        stepsApplied += st.activeDirSign;
        st.remaining -= 1;
        if (st.remaining > 0) {
          st.intervalTicks = Math.max(1, (st.intervalTicks || 1) + (st.addTicks || 0));
          st.nextWakeTimeMs += ticksToMs(st.intervalTicks);
        } else {
          const nextSeg = st.segments.shift();
          if (nextSeg) {
            st.intervalTicks = Math.max(1, nextSeg.intervalTicks);
            st.addTicks = nextSeg.addTicks;
            st.remaining = nextSeg.remaining;
            st.activeDirSign = nextSeg.dirSign;
            st.nextWakeTimeMs += ticksToMs(st.intervalTicks);
          } else {
            st.nextWakeTimeMs = null;
            st.intervalTicks = null;
            st.addTicks = 0;
            st.remaining = 0;
          }
        }
      }
      if (stepsApplied !== 0) {
        hasEmittedAnyStep = true; // lock baseline thereafter
        const newAngle = (axisAngles.get(axis) || 0) + stepsApplied * stepAngle;
        axisAngles.set(axis, newAngle);
        if (axis === EXTRUDER_AXIS) {
          const e_mm = stepsApplied * EXTRUDER_MM_PER_STEP;
          move[axis] = e_mm;
        } else {
          move[axis] = newAngle;
        }
        any = true;
      }
    }

    if (any) {
      postMessage({ type: 'move', command: move });
    }

    // Fixed-rate pacer; next wake is handled by setInterval
  } catch (e) {
    console.error('KlipperPacer pacer error:', e);
  }
};

const ensurePacerRunning = () => {
  if (pacerTimer !== null) return;
  pacerTimer = setInterval(pacerLoop, pacerIntervalMs);
};

const maybeAdjustBaseline = (firstIntervalTicks) => {
  if (hasEmittedAnyStep) return; // don't rebase after we start
  const fi = Math.max(1, Number(firstIntervalTicks) || 1);
  if (firstTickOffset === null) {
    firstTickOffset = fi;
    return;
  }
  if (fi < firstTickOffset) {
    const old = firstTickOffset;
    firstTickOffset = fi;
    // Shift all scheduled wake times earlier by the delta so the earliest
    // planned step happens near startedBaseTimeMs.
    if (startedBaseTimeMs !== null) {
      const shiftMs = ticksToMs(old - fi);
      for (const axis of axisOrder) {
        const st = axisState.get(axis);
        if (st && st.nextWakeTimeMs !== null) st.nextWakeTimeMs -= shiftMs;
      }
    }
  }
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

  if (st.nextWakeTimeMs === null) {
    // Establish global baseline so the earliest queued step across axes starts soon.
    maybeAdjustBaseline(seg.intervalTicks);
    if (startedBaseTimeMs === null) startedBaseTimeMs = performance.now() + bufferAheadMs;

    // Schedule first step relative to global baseline (subtract earliest offset).
    const baseTicks = firstTickOffset === null ? seg.intervalTicks : Math.max(1, seg.intervalTicks - firstTickOffset);
    st.intervalTicks = seg.intervalTicks;
    st.addTicks = seg.addTicks;
    st.remaining = seg.remaining;
    st.activeDirSign = seg.dirSign;
    st.nextWakeTimeMs = startedBaseTimeMs + ticksToMs(baseTicks);
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
    const st = axisState.get(axis);
    if (st) {
      st.segments.length = 0;
      st.nextWakeTimeMs = null;
      st.intervalTicks = null;
      st.addTicks = 0;
      st.remaining = 0;
      // After a set_position, align the active direction with the latest pending direction
      st.activeDirSign = st.dirSign;
    }
    postMessage({ type: 'move', command: { type: 'Add to reference', [axis]: delta } });
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

const connect = (url) => {
  ws = new WebSocket(url);
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
        } else if (msg && msg.action === 'klipper_clock' && typeof msg.clock_hz === 'number' && isFinite(msg.clock_hz) && msg.clock_hz > 0) {
          const oldHz = clockHz;
          const newHz = msg.clock_hz;
          if (DEBUG) console.log(`clock update: ${oldHz} -> ${newHz}`);
          if (newHz !== oldHz) {
            // Scale any pending nextWakeTimeMs so remaining time adjusts smoothly
            const now = performance.now();
            const scale = oldHz / newHz;
            for (const axis of axisOrder) {
              const st = axisState.get(axis);
              if (!st || st.nextWakeTimeMs === null) continue;
              const rem = Math.max(0, st.nextWakeTimeMs - now);
              st.nextWakeTimeMs = now + rem * scale;
            }
            clockHz = newHz;
          }
          return;
        }
      } catch (_) { /* not json, fall through */ }
      console.log(`unhandled text: ${event.data.slice(0, 100)}`);
    }
  };
  ws.onerror = (err) => console.error('websocket error:', err);
  ws.onclose = () => {
    console.log('connection closed');
    if (pacerTimer) {
      clearInterval(pacerTimer);
      pacerTimer = null;
    }
    postMessage({ type: 'closed' });
  };
};

self.onmessage = (e) => {
  const { type, ...data } = e.data;
  if (type === 'connect') {
    DEBUG = !!data.debug;
    connect(data.url);
  } else if (type === 'close') {
    if (ws) ws.close();
  }
};
