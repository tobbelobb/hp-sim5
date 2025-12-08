#!/usr/bin/env node
import fs from 'node:fs/promises';
import { spawn } from 'node:child_process';
import { createGcodeBridge, parseBridgeArgs } from './gcode_bridge.mjs';
import { STEP_CLOCK_HZ } from '../examples/js/slideprinter/rrfMotionUtils.js';

const DEFAULT_FEED = 2000;
const DEFAULT_WAIT_FOR_WS_MS = 5000;
const DEFAULT_SETTLE_MS = 1000;
const DEFAULT_SENSOR_AXIS_IDX = 2; // Z motor is sensor by default
const DEFAULT_TORQUE = 0.05;
const DEFAULT_RRF_PORT = 8081;
const DEFAULT_RRF_SERVER = `http://localhost:${DEFAULT_RRF_PORT}`;
const RRF_SIM_BINARY = './RRF/build/rrf_simulator';
const RRF_SIM_ARGS = ['--vsd', 'RRF/run/vsd', '-c', 'sys/config_slideprinter.g', '--server', '-p'];
const AXES = ['X', 'Y', 'Z'];
const MOTOR_IDS = ['40.0', '41.0', '42.0'];

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function parseM92(reply) {
  if (typeof reply !== 'string') {
    return {};
  }
  const result = {};
  const regex = /([XYZE])\s*:\s*([0-9.+-]+)/gi;
  let match = regex.exec(reply);
  while (match) {
    result[match[1].toUpperCase()] = parseFloat(match[2]);
    match = regex.exec(reply);
  }
  return result;
}

function parseValueList(value) {
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

function parseM666(reply) {
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

function parseEncoderReply(reply) {
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

function computeMmPerDegree(m666Values, axisIdx) {
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

function motionDurationSeconds(parsed) {
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

function estimateMoveLengthMm(gcode) {
  if (typeof gcode !== 'string') {
    return 0;
  }
  const coords = {};
  const regex = /\b([XYZ])(-?[0-9]+(?:\.[0-9]+)?)/gi;
  let match = regex.exec(gcode);
  while (match) {
    coords[match[1].toUpperCase()] = parseFloat(match[2]);
    match = regex.exec(gcode);
  }
  const dx = coords.X ?? 0;
  const dy = coords.Y ?? 0;
  const dz = coords.Z ?? 0;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function buildRrfSimulatorArgs(port) {
  return [...RRF_SIM_ARGS, port.toString()];
}

async function startRrfSimulator({ port = DEFAULT_RRF_PORT, debug = false } = {}) {
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

async function waitForRrfSimulator(baseUrl, timeoutMs = 7000) {
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
    await sleep(250);
  }
  throw new Error(`rrf_simulator at ${baseUrl} did not become ready in time`);
}

function stopProcess(proc) {
  if (!proc || proc.killed) {
    return;
  }
  try {
    proc.kill('SIGTERM');
  } catch (_err) {
    /* best effort */
  }
}

function parseTorqueValue(token) {
  if (typeof token !== 'string') {
    return null;
  }
  const match = token.trim().match(/^T\s*([-+]?[0-9]*\.?[0-9]+)/i);
  if (!match) {
    return null;
  }
  const torque = parseFloat(match[1]);
  return Number.isFinite(torque) ? torque : null;
}

function parsePointLine(line, lineNumber) {
  const trimmed = line.trim();
  if (trimmed.length === 0 || trimmed.startsWith('#') || trimmed.startsWith('//')) {
    return null;
  }
  const match = trimmed.match(/^\[([^\]]+)\]\s*$/);
  if (!match) {
    console.warn(`Skipping malformed line ${lineNumber}: "${line}"`);
    return null;
  }
  const parts = match[1].split(',').map((part) => part.trim()).filter((part) => part.length > 0);
  if (parts.length !== AXES.length) {
    console.warn(`Skipping line ${lineNumber}: expected ${AXES.length} entries, got ${parts.length}`);
    return null;
  }
  const rawParts = [...parts];
  const targets = [];
  let sensor = null;
  for (let i = 0; i < parts.length; i += 1) {
    const torque = parseTorqueValue(parts[i]);
    if (torque !== null) {
      sensor = { axisIdx: i, torque };
      targets.push(null);
      continue;
    }
    const value = parseFloat(parts[i]);
    if (!Number.isFinite(value)) {
      console.warn(`Skipping line ${lineNumber}: unable to parse value "${parts[i]}"`);
      return null;
    }
    targets.push(value);
  }
  return { targets, sensor, rawParts, lineNumber };
}

async function loadMeasurementPoints(filePath) {
  const content = await fs.readFile(filePath, 'utf8');
  const points = [];
  const lines = content.split(/\r?\n/);
  for (let i = 0; i < lines.length; i += 1) {
    const parsed = parsePointLine(lines[i], i + 1);
    if (parsed) {
      points.push(parsed);
    }
  }
  if (points.length === 0) {
    throw new Error(`No measurement coordinates found in ${filePath}`);
  }
  return points;
}

function formatPoint(point) {
  if (!point?.rawParts) {
    return 'unknown';
  }
  return `[${point.rawParts.join(', ')}]`;
}

function buildRelativeMove(currentPosition, point, feed) {
  if (!point?.targets || !Array.isArray(currentPosition)) {
    return null;
  }
  const parts = [];
  for (let i = 0; i < point.targets.length; i += 1) {
    const target = point.targets[i];
    if (!Number.isFinite(target)) {
      continue;
    }
    const delta = target - (currentPosition[i] ?? 0);
    currentPosition[i] = target;
    if (Math.abs(delta) < 1e-6) {
      continue;
    }
    parts.push(`${AXES[i]}${delta.toFixed(3)}`);
  }
  if (parts.length === 0) {
    return null;
  }
  return `G1 H2 ${parts.join(' ')} F${feed}`;
}

async function ensureSensorState(sendFn, desiredSensor, state) {
  if (!state) {
    return;
  }
  const desiredAxis = desiredSensor?.axisIdx ?? null;
  const desiredTorque = desiredSensor?.torque;
  const sameAxis = desiredAxis === state.axisIdx;
  const sameTorque = (desiredTorque == null && state.torque == null)
    || (Number.isFinite(desiredTorque) && Number.isFinite(state.torque)
      && Math.abs(desiredTorque - state.torque) < 1e-9);
  if (sameAxis && sameTorque) {
    return;
  }
  if (state.axisIdx != null && state.axisIdx !== desiredAxis) {
    const motorId = MOTOR_IDS[state.axisIdx];
    await sendFn(`M569.4 P${motorId} T0`, { suppressMotionProcessing: true });
  }
  if (desiredAxis != null) {
    const motorId = MOTOR_IDS[desiredAxis];
    const torque = Number.isFinite(desiredTorque) ? desiredTorque : DEFAULT_TORQUE;
    await sendFn(`M569.4 P${motorId} T${torque}`);
  }
  state.axisIdx = desiredAxis;
  state.torque = desiredTorque;
}

function computeReverseDistanceMm(angleDeg, axisIdx, mmPerDeg, m666Values, stepsPerMm) {
  const factor = Number.isFinite(mmPerDeg?.[axisIdx]) ? mmPerDeg[axisIdx] : null;
  if (factor !== null) {
    return -angleDeg * factor;
  }
  const radius = getArrayValue(m666Values?.R, axisIdx, null);
  if (Number.isFinite(radius)) {
    const simpleMmPerDeg = (2 * Math.PI * radius) / 360.0;
    return -angleDeg * simpleMmPerDeg;
  }
  const stepsPerMmAxis = stepsPerMm?.[AXES[axisIdx]];
  if (Number.isFinite(stepsPerMmAxis) && stepsPerMmAxis !== 0) {
    return -(angleDeg / 360) * (1 / stepsPerMmAxis);
  }
  return 0;
}

async function restorePositions(sendFn, {
  feed,
  anglesDeg,
  mmPerDeg,
  m666Values,
  stepsPerMm,
  sensorState,
  speedup = 1,
}) {
  const hasAngles = Array.isArray(anglesDeg) && anglesDeg.length > 0;
  const reverseMm = hasAngles
    ? AXES.map((_, idx) => computeReverseDistanceMm(
      anglesDeg[idx] ?? 0,
      idx,
      mmPerDeg,
      m666Values,
      stepsPerMm,
    ))
    : [];
  const moveParts = [];
  if (hasAngles) {
    for (let i = 0; i < reverseMm.length; i += 1) {
      if (sensorState?.axisIdx === i) {
        continue;
      }
      const dist = reverseMm[i];
      if (!Number.isFinite(dist) || Math.abs(dist) < 1e-5) {
        continue;
      }
      moveParts.push(`${AXES[i]}${dist.toFixed(3)}`);
    }
  }
  if (moveParts.length > 0) {
    await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup);
  }
  if (sensorState?.axisIdx != null) {
    const motorId = MOTOR_IDS[sensorState.axisIdx];
    await sendFn(`M569.4 P${motorId} T0`, { suppressMotionProcessing: true });
    const encoderReply = await sendFn(`M569.3 P${motorId}`, { suppressMotionProcessing: true });
    const encoderAngles = parseEncoderReply(encoderReply?.reply);
    const angle = encoderAngles[0] ?? 0;
    const dist = computeReverseDistanceMm(angle, sensorState.axisIdx, mmPerDeg, m666Values, stepsPerMm);
    if (Number.isFinite(dist) && Math.abs(dist) > 1e-5) {
      await runMoveWithWait(sendFn, `G1 H2 ${AXES[sensorState.axisIdx]}${dist.toFixed(3)} F${feed}`, speedup);
    }
    sensorState.axisIdx = null;
    sensorState.torque = null;
  }
}

async function sendHpSimReset(bridgeCtx, { quiet = false } = {}) {
  if (!bridgeCtx?.broadcast) {
    return;
  }
  bridgeCtx.broadcast({ type: 'reset' });
  if (!quiet) {
    console.log('Sent hp-sim reset request.');
  }
  await sleep(50);
}

async function sendHpSimSpeedScale(bridgeCtx, scale, { quiet = false } = {}) {
  if (!bridgeCtx?.broadcast || !Number.isFinite(scale) || scale <= 0 || scale === 1) {
    return;
  }
  bridgeCtx.broadcast({ type: 'set_speed_scale', value: scale });
  if (!quiet) {
    console.log(`Requested hp-sim speed scale: ${scale}x`);
  }
  await sleep(25);
}

async function main() {
  const args = parseBridgeArgs(process.argv.slice(2));
  const dx = Number.isFinite(parseFloat(args.dx)) ? parseFloat(args.dx) : 10;
  const dy = Number.isFinite(parseFloat(args.dy)) ? parseFloat(args.dy) : 10;
  const feed = Number.isFinite(parseFloat(args.feed)) ? parseFloat(args.feed) : DEFAULT_FEED;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : DEFAULT_WAIT_FOR_WS_MS;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const debug = !!args.debug;
  let success = false;
  const settleMs = Number.isFinite(parseFloat(args.settleMs))
    ? Math.max(0, parseFloat(args.settleMs))
    : DEFAULT_SETTLE_MS;
  const speedup = Number.isFinite(parseFloat(args.speedup)) && parseFloat(args.speedup) > 0
    ? parseFloat(args.speedup)
    : 1;

  let measurementPoints = null;
  const requestedPointsFile = args.pointsFile;
  if (requestedPointsFile) {
    measurementPoints = await loadMeasurementPoints(requestedPointsFile);
    console.log(`Loaded ${measurementPoints.length} measurement point(s) from ${requestedPointsFile}`);
  } else {
    try {
      await fs.access('measurement_points.txt');
      measurementPoints = await loadMeasurementPoints('measurement_points.txt');
      console.log(`Loaded ${measurementPoints.length} measurement point(s) from measurement_points.txt`);
    } catch (err) {
      if (err?.code !== 'ENOENT') {
        console.warn(`Could not load measurement_points.txt: ${err.message}`);
      }
      /* keep fallback */
    }
  }

  if (!measurementPoints) {
    measurementPoints = [{
      targets: [dx, dy, null],
      sensor: { axisIdx: DEFAULT_SENSOR_AXIS_IDX, torque: DEFAULT_TORQUE },
      rawParts: [dx, dy, `T${DEFAULT_TORQUE}`],
    }];
    console.log('No measurement_points.txt supplied; using single default coordinate.');
  }

  const targetServer = args.serverExplicit ? args.server : DEFAULT_RRF_SERVER;
  const shouldSpawnRrf = !args.noSpawnRrfSimulator && !args.serverExplicit;
  let rrfProcess = null;
  if (shouldSpawnRrf) {
    try {
      console.log(`Starting rrf_simulator at ${targetServer}...`);
      rrfProcess = await startRrfSimulator({ port: DEFAULT_RRF_PORT, debug });
      await waitForRrfSimulator(targetServer);
    } catch (err) {
      throw new Error(`Unable to start rrf_simulator: ${err?.message || err}`);
    }
  }

  const bridgeCtx = createGcodeBridge({
    server: targetServer,
    wsPort: args.noWs ? 0 : args.wsPort,
    quiet: args.quiet,
    encoderTimeoutMs,
  });

  const send = async (line, options = {}) => {
    const res = await bridgeCtx.sendGcodeLine(line, options);
    if (debug && res?.reply) {
      console.log(res.reply.trim());
    }
    return res;
  };

  if (!args.noWs) {
    await bridgeCtx.waitForHpSimConnection(waitForWsMs);
    if (!args.noHpSimReset) {
      await sendHpSimReset(bridgeCtx, { quiet: args.quiet });
    }
    if (speedup !== 1) {
      await sendHpSimSpeedScale(bridgeCtx, speedup, { quiet: args.quiet });
    }
  }

  let originalQ = null;
  const measurements = [];
  const sensorState = { axisIdx: null, torque: null };
  const currentPosition = [0, 0, 0];
  let lastAnglesDeg = null;

  try {
    const m92Reply = await send('M92', { suppressMotionProcessing: true });
    const stepsPerMm = parseM92(m92Reply?.reply);

    const m666Reply = await send('M666', { suppressMotionProcessing: true });
    const m666Values = parseM666(m666Reply?.reply);
    originalQ = Number.isFinite(m666Values.Q) ? m666Values.Q : null;
    const mmPerDeg = AXES.map((_, i) => computeMmPerDegree(m666Values, i));

    await send('G92 X0 Y0 Z0', { suppressMotionProcessing: true });
    await send('G91', { suppressMotionProcessing: true });
    await send('M666 Q0', { suppressMotionProcessing: true });
    await send('M569.3 P40.0:41.0:42.0 S', { suppressMotionProcessing: true });

    for (const point of measurementPoints) {
      const desiredSensor = point.sensor
        || (sensorState.axisIdx != null ? sensorState : { axisIdx: DEFAULT_SENSOR_AXIS_IDX, torque: DEFAULT_TORQUE });
      // eslint-disable-next-line no-await-in-loop
      await ensureSensorState(send, desiredSensor, sensorState);
      const move = buildRelativeMove(currentPosition, point, feed);
    if (move) {
      // eslint-disable-next-line no-await-in-loop
      await runMoveWithWait(send, move, speedup);
    }
    // eslint-disable-next-line no-await-in-loop
    await sleep(Math.max(0, settleMs / (speedup > 0 ? speedup : 1)));
      // eslint-disable-next-line no-await-in-loop
      const encoderMid = await send('M569.3 P40.0:41.0:42.0');
      const anglesDeg = parseEncoderReply(encoderMid?.reply);
      lastAnglesDeg = anglesDeg;
      measurements.push({
        point: formatPoint(point),
        targets: point.targets,
        sensorAxis: sensorState.axisIdx != null ? AXES[sensorState.axisIdx] : null,
        torque: sensorState.torque ?? desiredSensor?.torque ?? null,
        anglesDeg,
      });
      console.log(`Measured ${measurements.length}/${measurementPoints.length} at ${formatPoint(point)}: ${anglesDeg.join(', ')}`);
    }

    await restorePositions(send, {
      feed,
      anglesDeg: lastAnglesDeg,
      mmPerDeg,
      m666Values,
      stepsPerMm,
      sensorState,
      speedup,
    });

    if (originalQ !== null) {
      await send(`M666 Q${originalQ}`, { suppressMotionProcessing: true });
    }

    if (args.outputFile) {
      await fs.writeFile(args.outputFile, JSON.stringify(measurements, null, 2));
      console.log(`Saved ${measurements.length} measurement${measurements.length === 1 ? '' : 's'} to ${args.outputFile}`);
    }

    if (debug) {
      console.log('Steps/mm:', stepsPerMm);
      console.log('M666:', m666Values);
    }
    success = true;
  } catch (err) {
    console.error(`Failed to complete sequence: ${err?.message || err}`);
    if (sensorState?.axisIdx != null) {
      try {
        const motorId = MOTOR_IDS[sensorState.axisIdx];
        await send(`M569.4 P${motorId} T0`, { suppressMotionProcessing: true });
      } catch (_err) {
        /* best effort */
      }
    }
    if (originalQ !== null) {
      await send(`M666 Q${originalQ}`, { suppressMotionProcessing: true });
    }
  } finally {
    if (rrfProcess && !args.persistRrfSimulator) {
      stopProcess(rrfProcess);
    }
    bridgeCtx.close();
    process.exit(success ? 0 : 1);
  }
}

async function runMoveWithWait(sendFn, gcode, speedup = 1) {
  const result = await sendFn(gcode);
  const durationSeconds = motionDurationSeconds(result);
  const divisor = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  if (Number.isFinite(durationSeconds) && durationSeconds > 0) {
    await sleep((durationSeconds * 1000) / divisor);
  } else {
    const feedMatch = gcode.match(/\bF([0-9]+(?:\.[0-9]+)?)/i);
    const feed = feedMatch ? parseFloat(feedMatch[1]) : DEFAULT_FEED;
    const dist = estimateMoveLengthMm(gcode);
    if (Number.isFinite(feed) && feed > 0 && Number.isFinite(dist)) {
      await sleep((((dist / (feed / 60)) + 0.1) * 1000) / divisor);
    } else {
      await sleep(500 / divisor);
    }
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
