// KlipperHandler: Connects to a raw-bytes WebSocket and logs incoming frames.
// This is a first step to confirm we receive MCU bytes from Klipper.

// Minimal translator from parsed Klipper MCU messages -> Move commands.
// Assumptions:
//  - Maps first four stepper oids seen to axes A,B,C,D.
//  - Interprets 'queue_step oid=N count=K add={-1|1}' as K steps in the
//    indicated direction; updates absolute angle per axis by stepAngle*K*add.
//  - stepAngle = 2*pi / (stepsPerRev * microsteps).
// This is a naive approximation intended for early visualization only.

export function connectKlipperRaw(url, onCommand /* function(command) */) {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';

  // Simple stepper mapping and position tracking
  const axisOrder = ['A', 'B', 'C', 'D'];
  const oidToAxis = new Map();
  const axisAngles = new Map(axisOrder.map(a => [a, 0.0]));
  const stepsPerRev = 200;   // default full steps per rev
  const microsteps = 16;     // default microsteps
  const stepAngle = (2 * Math.PI) / (stepsPerRev * microsteps);

  const ensureAxisForOid = (oid) => {
    if (oidToAxis.has(oid)) return oidToAxis.get(oid);
    const used = new Set(oidToAxis.values());
    const axis = axisOrder.find(a => !used.has(a));
    if (axis) oidToAxis.set(oid, axis);
    return axis;
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
      const kv = parseKv(sliceAfter('config_stepper'));
      const axis = ensureAxisForOid(kv.oid);
      if (axis) console.log(`Klipper map: oid ${kv.oid} -> axis ${axis}`);
      return;
    }
    if (has('queue_step')) {
      const kv = parseKv(sliceAfter('queue_step'));
      const axis = ensureAxisForOid(kv.oid);
      if (!axis) return;
      const count = Number(kv.count) || 0;
      const add = ('add' in kv) ? Number(kv.add) : 1;
      const delta = stepAngle * count * (add >= 0 ? 1 : -1);
      const newAngle = (axisAngles.get(axis) || 0) + delta;
      axisAngles.set(axis, newAngle);
      if (typeof onCommand === 'function') {
        onCommand({ type: 'Move', [axis]: newAngle });
      }
      return;
    }
    // TODO: handle set_position / sync variants -> Add to reference
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
      console.log(`KlipperHandler: ${bytes.length} bytes: ${hex}`);
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
      console.log(`KlipperHandler (text): ${event.data.slice(0, 200)}${event.data.length > 200 ? '…' : ''}`);
    } else {
      console.log('KlipperHandler: received frame of unknown type');
    }
  };

  ws.onerror = (err) => {
    console.error('KlipperHandler websocket error:', err);
  };

  ws.onclose = () => {
    console.log('KlipperHandler: connection closed');
  };

  return {
    ws,
    close: () => ws.close(),
  };
}
