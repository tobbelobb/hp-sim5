// This is a worker script for Klipper pacing.

// --- Logging helpers ---
const log = (...args) => postMessage({ type: 'log', args });
const error = (...args) => postMessage({ type: 'error', args });

// --- Globals for worker state ---
let ws = null;
let DEBUG = true;
let firstSeqSeen = null; // Debug: first server seq index we receive

const axisOrder = ['A', 'B', 'C', 'D'];
const axisAngles = new Map(axisOrder.map(a => [a, 0.0]));
const stepsPerRev = 200;
const microsteps = 16;
const stepAngle = (2 * Math.PI) / (stepsPerRev * microsteps);

const clockHz = 16_000_000; // Match bridge default
const ticksToMs = (ticks) => (ticks / clockHz) * 1000.0;
const bufferAheadMs = 0.0; // Buffer to smooth out network jitter
const pacerIntervalMs = 2.0;
let pacerTimer = null;
let startedBaseTimeMs = null;

const axisState = new Map(axisOrder.map(a => [a, {
  segments: [],
  nextWakeTimeMs: null,
  intervalTicks: null,
  addTicks: 0,
  remaining: 0,
  dirSign: 1,
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
        stepsApplied += st.dirSign;
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
        const newAngle = (axisAngles.get(axis) || 0) + stepsApplied * stepAngle;
        axisAngles.set(axis, newAngle);
        move[axis] = newAngle;
        any = true;
      }
    }

    if (any) {
      postMessage({ type: 'move', command: move });
    }
  } catch (e) {
    error('KlipperPacer pacer error:', e);
  }
};

const ensurePacerRunning = () => {
  if (pacerTimer !== null) return;
  pacerTimer = setInterval(pacerLoop, pacerIntervalMs);
};

const enqueueSegment = (axis, intervalTicks, count, addTicks) => {
  const st = axisState.get(axis);
  if (!st) return;
  const seg = {
    intervalTicks: Math.max(1, Number(intervalTicks) || 1),
    addTicks: Number(addTicks) || 0,
    remaining: Math.max(0, Number(count) || 0),
  };

  if (st.nextWakeTimeMs === null) {
    if (startedBaseTimeMs === null) startedBaseTimeMs = performance.now() + bufferAheadMs;
    st.intervalTicks = seg.intervalTicks;
    st.addTicks = seg.addTicks;
    st.remaining = seg.remaining;
    st.nextWakeTimeMs = startedBaseTimeMs + ticksToMs(st.intervalTicks);
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
    if (DEBUG) log(line);
    const kv = parseKv(sliceAfter('config_stepper'));
    const axis = ensureAxisForOid(kv.oid);
    if (axis) log(`Klipper map: oid ${kv.oid} -> axis ${axis}`);
    else log(`Klipper failed to map oid: ${kv.oid}`);
    return;
  }
  if (has('set_next_step_dir')) {
    if (DEBUG) log(line);
    const kv = parseKv(sliceAfter('set_next_step_dir'));
    const axis = ensureAxisForOid(kv.oid);
    if (!axis) return;
    const st = axisState.get(axis);
    if (!st) return;
    st.dirSign = (Number(kv.dir) === 0) ? -1 : 1;
    return;
  }
  if (has('set_position')) {
    if (DEBUG) log(line);
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
    }
    postMessage({ type: 'move', command: { type: 'Add to reference', [axis]: delta } });
    return;
  }
  if (has('queue_step')) {
    if (DEBUG) log(line);
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
  ws.onopen = () => log(`connected to ${url}`);
  ws.onmessage = (event) => {
    if (typeof event.data === 'string') {
      try {
        const msg = JSON.parse(event.data);
        if (msg && msg.action === 'klipper_parsed' && Array.isArray(msg.lines)) {
          if (firstSeqSeen === null && typeof msg.seq === 'number') {
            firstSeqSeen = msg.seq;
            if (DEBUG) log(`first klipper_parsed seq=${firstSeqSeen} count=${msg.count}`);
          }
          for (const line of msg.lines) handleParsedLine(line);
          return;
        }
      } catch (_) { /* not json, fall through */ }
      log(`unhandled text: ${event.data.slice(0, 100)}`);
    }
  };
  ws.onerror = (err) => error('websocket error:', err);
  ws.onclose = () => {
    log('connection closed');
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
