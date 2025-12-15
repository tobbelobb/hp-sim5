import { spawn } from 'node:child_process';
import { STEP_CLOCK_HZ } from '../examples/js/slideprinter/rrfMotionUtils.js';

export const DEFAULT_FEED = 2000;
export const DEFAULT_RRF_PORT = 8081;
export const DEFAULT_RRF_SERVER = `http://localhost:${DEFAULT_RRF_PORT}`;
export const RRF_SIM_BINARY = './RRF/build/rrf_simulator';
export const RRF_SIM_ARGS = ['--vsd', 'RRF/run/vsd', '-c', 'sys/config_slideprinter.g', '--server', '-p'];

export function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export function parseValueList(value) {
  if (value == null) {
    return [];
  }
  if (Array.isArray(value)) {
    return value.map((v) => (Number.isFinite(v) ? v : NaN));
  }
  if (typeof value === 'number') {
    return [value];
  }
  return value
    .split(':')
    .map((part) => parseFloat(part))
    .filter((v) => Number.isFinite(v));
}

export function parseM666(reply) {
  if (typeof reply !== 'string') {
    return {};
  }
  const values = {};
  const regex = /([A-Z])\s*([-0-9.:]+)/g;
  let match = regex.exec(reply);
  while (match) {
    const key = match[1];
    const rawVal = match[2];
    values[key] = rawVal.includes(':')
      ? parseValueList(rawVal)
      : parseFloat(rawVal);
    match = regex.exec(reply);
  }
  return values;
}

export function parseEncoderReply(reply) {
  if (typeof reply !== 'string') {
    return [];
  }
  const matches = reply.match(/-?[0-9]+(?:\.[0-9]+)?/g);
  if (!matches) {
    return [];
  }
  return matches.map((v) => parseFloat(v));
}

function getArrayValue(values, idx, fallback = 1) {
  if (Array.isArray(values) && Number.isFinite(values[idx])) {
    return values[idx];
  }
  if (Number.isFinite(values)) {
    return values;
  }
  return fallback;
}

export function computeMmPerDegree(m666Values, axisIdx) {
  const radii = m666Values.R;
  const mechAdv = m666Values.U;
  const linesPerSpool = m666Values.O;
  const motorGear = m666Values.L;
  const spoolGear = m666Values.H;

  const r = getArrayValue(radii, axisIdx, null);
  if (!Number.isFinite(r) || r === 0) {
    return null;
  }
  const ma = Math.max(getArrayValue(mechAdv, axisIdx, 1), 1e-6);
  const lines = Math.max(getArrayValue(linesPerSpool, axisIdx, 1), 1e-6);
  const motorGearTeeth = Math.max(getArrayValue(motorGear, axisIdx, 1), 1e-6);
  const spoolGearTeeth = Math.max(getArrayValue(spoolGear, axisIdx, 1), 1e-6);
  const gearFactor = spoolGearTeeth / motorGearTeeth;

  return (2 * Math.PI * r) / (gearFactor * ma * lines * 360.0);
}

export function motionDurationSeconds(parsed) {
  if (!parsed?.motion || parsed.motion.length === 0) {
    return null;
  }
  let minStart = Number.POSITIVE_INFINITY;
  let maxEnd = Number.NEGATIVE_INFINITY;
  for (const item of parsed.motion) {
    if (!item || item.type !== 'Motion') {
      continue;
    }
    const start = Number.isFinite(item.timestamp) ? item.timestamp : 0;
    const end = start
      + (Number.isFinite(item.accelTicks) ? item.accelTicks : 0)
      + (Number.isFinite(item.steadyTicks) ? item.steadyTicks : 0)
      + (Number.isFinite(item.decelTicks) ? item.decelTicks : 0);
    minStart = Math.min(minStart, start);
    maxEnd = Math.max(maxEnd, end);
  }
  if (!Number.isFinite(minStart) || !Number.isFinite(maxEnd) || maxEnd < minStart) {
    return null;
  }
  const totalTicks = maxEnd - minStart;
  if (!Number.isFinite(totalTicks) || totalTicks <= 0) {
    return null;
  }
  return totalTicks / STEP_CLOCK_HZ;
}

export function estimateMoveLengthMm(gcode, axes = null) {
  if (typeof gcode !== 'string') {
    return 0;
  }
  const axisChars = Array.isArray(axes) && axes.length > 0
    ? axes.map((c) => String(c).toUpperCase()).join('')
    : 'XYZABCDIJKL';
  const coords = {};
  const regex = new RegExp(`\\b([${axisChars}])(-?[0-9]+(?:\\.[0-9]+)?)`, 'gi');
  let match = regex.exec(gcode);
  while (match) {
    coords[match[1].toUpperCase()] = parseFloat(match[2]);
    match = regex.exec(gcode);
  }
  const values = Object.values(coords);
  if (values.length === 0) {
    return 0;
  }
  const sumSquares = values.reduce((acc, val) => acc + (Number.isFinite(val) ? val * val : 0), 0);
  return Math.sqrt(sumSquares);
}

export async function runMoveWithWait(sendFn, gcode, speedup = 1, {
  defaultFeed = DEFAULT_FEED,
  axes = null,
  delayFn = sleep,
} = {}) {
  const result = await sendFn(gcode);
  const durationSeconds = motionDurationSeconds(result);
  const divisor = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  if (Number.isFinite(durationSeconds) && durationSeconds > 0) {
    await delayFn((durationSeconds * 1000) / divisor);
    return;
  }
  const feedMatch = gcode.match(/\bF([0-9]+(?:\.[0-9]+)?)/i);
  const feed = feedMatch ? parseFloat(feedMatch[1]) : defaultFeed;
  const dist = estimateMoveLengthMm(gcode, axes);
  if (Number.isFinite(feed) && feed > 0 && Number.isFinite(dist)) {
    await delayFn((((dist / (feed / 60)) + 0.1) * 1000) / divisor);
  } else {
    await delayFn(500 / divisor);
  }
}

export function buildRrfSimulatorArgs(port) {
  return [...RRF_SIM_ARGS, port.toString()];
}

export async function startRrfSimulator({ port = DEFAULT_RRF_PORT, debug = false } = {}) {
  const args = buildRrfSimulatorArgs(port);
  const child = spawn(RRF_SIM_BINARY, args, { stdio: ['ignore', 'pipe', 'pipe'] });
  if (debug && child?.stdout) {
    child.stdout.on('data', (data) => {
      process.stdout.write(`[rrf_sim] ${data}`);
    });
  }
  if (child?.stderr) {
    child.stderr.on('data', (data) => {
      const prefix = debug ? '[rrf_sim] ' : '';
      process.stderr.write(`${prefix}${data}`);
    });
  }
  return child;
}

export async function waitForRrfSimulator(baseUrl, timeoutMs = 7000) {
  const endpoint = `${baseUrl.replace(/\/$/, '')}/machine/code`;
  const deadline = Date.now() + Math.max(1, timeoutMs);
  while (Date.now() < deadline) {
    try {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), 1500);
      const res = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain' },
        body: 'M115',
        signal: controller.signal,
      });
      clearTimeout(timer);
      if (res.ok) {
        await res.text();
        return;
      }
    } catch (_err) {
      /* try again */
    }
    // eslint-disable-next-line no-await-in-loop
    await sleep(250);
  }
  throw new Error(`rrf_simulator at ${baseUrl} did not become ready in time`);
}

export function stopProcess(proc) {
  if (!proc || proc.killed) {
    return;
  }
  try {
    proc.kill('SIGTERM');
  } catch (_err) {
    /* best effort */
  }
}

export async function sendHpSimReset(bridgeCtx, { quiet = false } = {}) {
  if (!bridgeCtx?.broadcast) {
    return;
  }
  bridgeCtx.broadcast({ type: 'reset' });
  if (!quiet) {
    console.log('Sent hp-sim reset request.');
  }
  await sleep(50);
}

export async function sendHpSimSpeedScale(bridgeCtx, scale, { quiet = false } = {}) {
  if (!bridgeCtx?.broadcast || !Number.isFinite(scale) || scale <= 0 || scale === 1) {
    return;
  }
  bridgeCtx.broadcast({ type: 'set_speed_scale', value: scale });
  if (!quiet) {
    console.log(`Requested hp-sim speed scale: ${scale}x`);
  }
  await sleep(25);
}

export async function sendHpSimPositionTraceMode(bridgeCtx, enabled, { quiet = false } = {}) {
  if (!bridgeCtx?.broadcast) {
    return;
  }
  bridgeCtx.broadcast({ type: 'position_trace_mode', enabled: Boolean(enabled) });
  if (!quiet) {
    console.log(`Requested hp-sim position trace mode: ${enabled ? 'on' : 'off'}.`);
  }
  await sleep(25);
}
