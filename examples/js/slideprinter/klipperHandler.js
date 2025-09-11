// KlipperHandler: Connects to a raw-bytes WebSocket and logs incoming frames.
// This is a first step to confirm we receive MCU bytes from Klipper.

// Minimal translator from parsed Klipper MCU messages -> Move commands.
 // Assumptions:
 //  - Uses a fixed mapping: oid 0->A, 1->B, 2->C, 3->D.
 //  - Direction is set by 'set_next_step_dir'. 'add' adjusts interval (acceleration), not direction.
//  - stepAngle = 2*pi / (stepsPerRev * microsteps).
 // This is a naive approximation intended for early visualization only.

const DEBUG = true;

export function connectKlipperRaw(url, onCommand /* function(command) */) {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';

  // Simple stepper mapping and position tracking
  const axisOrder = ['A', 'B', 'C', 'D'];
  const axisAngles = new Map(axisOrder.map(a => [a, 0.0]));
  const stepsPerRev = 200;   // default full steps per rev
  const microsteps = 16;     // default microsteps
  const stepAngle = (2 * Math.PI) / (stepsPerRev * microsteps);

  // --- Pacing to emulate Klipper MCU timing ---
  // AVR default clock; Klipper queue_step 'interval' is in clock ticks
  const clockHz = 8_000_000;
  const ticksToMs = (ticks) => (ticks / clockHz) * 1000.0;
  const bufferAheadMs = 0.0;   // try to stay ahead
  const pacerIntervalMs = 2.0;   // coalesce updates at ~500Hz
  let pacerTimer = null;
  let startedBaseTimeMs = null;

  // Per-axis queued segments and scheduler state
  // Segment: { intervalTicks, addTicks, remaining }
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

  // Start/update pacer that applies due steps and emits coalesced Move updates
  const ensurePacerRunning = () => {
    if (pacerTimer !== null) return;
    pacerTimer = setInterval(() => {
      try {
        const now = performance.now();
        const move = { type: 'Move' };
        let any = false;

        for (const axis of axisOrder) {
          const st = axisState.get(axis);
          if (!st) continue;

          let stepsApplied = 0;
          while (st.nextWakeTimeMs !== null && st.nextWakeTimeMs <= now) {
            // Apply one step at scheduled time
            stepsApplied += st.dirSign;

            // Advance scheduling within/after segment
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

        if (any && typeof onCommand === 'function'){
          //console.log(move);
          onCommand(move);
        }
      } catch (e) {
        console.error('KlipperHandler pacer error:', e);
      }
    }, pacerIntervalMs);
  };

  const enqueueSegment = (axis, intervalTicks, count, addTicks) => {
    const st = axisState.get(axis);
    if (!st) return;

    // Direction is provided by 'set_next_step_dir'; do not infer from 'add'
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
    // Be tolerant to prefixes (e.g. "tx: queue_step ...", "mcu: config_stepper ...")
    const has = (name) => line.includes(name + ' ');
    const sliceAfter = (name) => {
      const i = line.indexOf(name);
      return i >= 0 ? line.slice(i + name.length) : '';
    };

    // Example formats we anticipate:
    //  "queue_step oid=4 interval=123 count=10 add=1"
    //  "config_stepper oid=4 step_pin=P.. dir_pin=P.."
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
      const dir = Number(kv.dir);
      st.dirSign = (dir === 0) ? -1 : 1;
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
      // Reset scheduler for this axis to avoid stale queued steps
      const st = axisState.get(axis);
      if (st) {
        st.segments.length = 0;
        st.nextWakeTimeMs = null;
        st.intervalTicks = null;
        st.addTicks = 0;
        st.remaining = 0;
      }
      if (typeof onCommand === 'function') {
        onCommand({ type: 'Add to reference', [axis]: delta });
      }
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
    // TODO: handle sync variants -> Add to reference
  };

  ws.onopen = () => {
    console.log(`KlipperHandler: connected to ${url}`);
  };

  ws.onmessage = (event) => {
    if (event.data instanceof ArrayBuffer) {
      const bytes = new Uint8Array(event.data);
      // Print as a compact hex string (truncate for readability)
      let hex = '';
      const maxLen = Math.min(bytes.length, 128);
      for (let i = 0; i < maxLen; i++) {
        hex += bytes[i].toString(16).padStart(2, '0');
      }
      if (bytes.length > maxLen) {
        hex += '…';
      }
    } else if (typeof event.data === 'string') {
      // Try to parse JSON payloads emitted by the Python bridge
      try {
        const msg = JSON.parse(event.data);
        if (msg && msg.action === 'klipper_parsed' && Array.isArray(msg.lines)) {
          for (const line of msg.lines) handleParsedLine(line);
          return;
        }
      } catch (_) {
        // Not JSON; fall through to plain text logging
      }
      if (DEBUG) console.log(`KlipperHandler (text): ${event.data.slice(0, 200)}${event.data.length > 200 ? '…' : ''}`);
    } else {
      console.log('KlipperHandler: received frame of unknown type');
    }
  };

  ws.onerror = (err) => {
    console.error('KlipperHandler websocket error:', err);
  };

  ws.onclose = () => {
    console.log('KlipperHandler: connection closed');
    if (pacerTimer !== null) {
      clearInterval(pacerTimer);
      pacerTimer = null;
    }
  };

  return {
    ws,
    close: () => ws.close(),
  };
}
