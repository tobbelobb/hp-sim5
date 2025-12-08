#!/usr/bin/env node
import { createGcodeBridge, parseBridgeArgs } from './gcode_bridge.mjs';
import { STEP_CLOCK_HZ } from '../examples/js/slideprinter/rrfMotionUtils.js';

const DEFAULT_FEED = 600;
const DEFAULT_WAIT_FOR_WS_MS = 5000;

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

async function main() {
  const args = parseBridgeArgs(process.argv.slice(2));
  const dx = Number.isFinite(parseFloat(args.dx)) ? parseFloat(args.dx) : 10;
  const dy = Number.isFinite(parseFloat(args.dy)) ? parseFloat(args.dy) : 10;
  const feed = Number.isFinite(parseFloat(args.feed)) ? parseFloat(args.feed) : DEFAULT_FEED;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : DEFAULT_WAIT_FOR_WS_MS;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const debug = !!args.debug;
  let success = false;

  const bridgeCtx = createGcodeBridge({
    server: args.server,
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
  }

  let originalQ = null;
  try {
    const m92Reply = await send('M92', { suppressMotionProcessing: true });
    const stepsPerMm = parseM92(m92Reply?.reply);

    const m666Reply = await send('M666', { suppressMotionProcessing: true });
    const m666Values = parseM666(m666Reply?.reply);
    originalQ = Number.isFinite(m666Values.Q) ? m666Values.Q : null;

    await send('G92 X0 Y0 Z0', { suppressMotionProcessing: true });
    await send('M666 Q0', { suppressMotionProcessing: true });
    await send('M569.3 P40.0:41.0:42.0 S', { suppressMotionProcessing: true });
    await send('M569.4 P42.0 T0.05');

    await runMoveWithWait(send, `G1 H2 X${dx} Y${dy} F${feed}`);

    await sleep(1000);
    const encoderMid = await send('M569.3 P40.0:41.0:42.0');
    const anglesDeg = parseEncoderReply(encoderMid?.reply);

    const mmPerDeg = [0, 1, 2].map((i) => computeMmPerDegree(m666Values, i));
    const axisOrder = ['X', 'Y', 'Z'];
    const reverseXYMm = anglesDeg.slice(0, 2).map((deg, idx) => {
      const factor = Number.isFinite(mmPerDeg[idx]) ? mmPerDeg[idx] : null;
      if (factor === null) {
        const radius = getArrayValue(m666Values.R, idx, null);
        if (Number.isFinite(radius)) {
          const simpleMmPerDeg = (2 * Math.PI * radius) / 360.0;
          return -deg * simpleMmPerDeg;
        }
        const stepsPerMmAxis = axisOrder[idx] ? stepsPerMm?.[axisOrder[idx]] : null;
        if (Number.isFinite(stepsPerMmAxis) && stepsPerMmAxis !== 0) {
          return -(deg / 360) * (1 / stepsPerMmAxis);
        }
        return 0;
      }
      return -deg * factor;
    });

    const [a, b] = reverseXYMm;
    await runMoveWithWait(send, `G1 H2 X${a.toFixed(3)} Y${b.toFixed(3)} F${feed}`);
    await send('M569.4 P42.0 T0', { suppressMotionProcessing: true });
    const encoderC = await send('M569.3 P42.0', { suppressMotionProcessing: true });
    const anglesCDeg = parseEncoderReply(encoderC?.reply);
    const cAngle = anglesCDeg[0] ?? 0;
    const cFactor = Number.isFinite(mmPerDeg[2]) ? mmPerDeg[2] : (() => {
      const radius = getArrayValue(m666Values.R, 2, null);
      if (Number.isFinite(radius)) {
        return (2 * Math.PI * radius) / 360.0;
      }
      const stepsPerMmAxis = stepsPerMm?.Z;
      if (Number.isFinite(stepsPerMmAxis) && stepsPerMmAxis !== 0) {
        return 1 / (stepsPerMmAxis * 360);
      }
      return 0;
    })();
    const c = -cAngle * cFactor;
    await runMoveWithWait(send, `G1 H2 Z${c.toFixed(3)} F${feed}`);

    if (originalQ !== null) {
      await send(`M666 Q${originalQ}`, { suppressMotionProcessing: true });
    }
    console.log('Captured encoder reading:', anglesDeg);
    if (debug) {
      console.log('Reverse XY (mm):', reverseXYMm.map((v) => Number.isFinite(v) ? v.toFixed(3) : 'nan'));
      console.log('Captured C encoder:', cAngle);
      console.log('Reverse Z (mm):', Number.isFinite(c) ? c.toFixed(3) : 'nan');
      console.log('Steps/mm:', stepsPerMm);
      console.log('M666:', m666Values);
    }
    success = true;
  } catch (err) {
    console.error(`Failed to complete sequence: ${err?.message || err}`);
    if (originalQ !== null) {
      await send(`M666 Q${originalQ}`, { suppressMotionProcessing: true });
    }
  } finally {
    bridgeCtx.close();
    process.exit(success ? 0 : 1);
  }
}

async function runMoveWithWait(sendFn, gcode) {
  const result = await sendFn(gcode);
  const durationSeconds = motionDurationSeconds(result);
  if (Number.isFinite(durationSeconds) && durationSeconds > 0) {
    await sleep(durationSeconds * 1000);
  } else {
    const feedMatch = gcode.match(/\bF([0-9]+(?:\.[0-9]+)?)/i);
    const feed = feedMatch ? parseFloat(feedMatch[1]) : DEFAULT_FEED;
    const dist = estimateMoveLengthMm(gcode);
    if (Number.isFinite(feed) && feed > 0 && Number.isFinite(dist)) {
      await sleep(((dist / (feed / 60)) + 0.1) * 1000);
    } else {
      await sleep(500);
    }
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
