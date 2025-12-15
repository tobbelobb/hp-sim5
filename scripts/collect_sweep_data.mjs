#!/usr/bin/env node
import fs from 'node:fs/promises';
import path from 'node:path';
import readline from 'node:readline';
import { pathToFileURL } from 'node:url';
import { createGcodeBridge, parseBridgeArgs } from './gcode_bridge.mjs';
import {
  DEFAULT_FEED,
  DEFAULT_RRF_PORT,
  computeMmPerDegree,
  parseEncoderReply,
  parseM666,
  runMoveWithWait,
  sendHpSimReset,
  sendHpSimSpeedScale,
  sendHpSimPositionTraceMode,
  sleep as baseSleep,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from './encoder_utils.mjs';

const MACHINE_CONFIGS = {
  slideprinter: { numAnchors: 3, dimensions: 2, axes: ['X', 'Y', 'Z'], forbiddenSensors: [] },
  hangprinter_4: { numAnchors: 4, dimensions: 3, axes: ['A', 'B', 'C', 'D'], forbiddenSensors: [3] },
  hangprinter_5: { numAnchors: 5, dimensions: 3, axes: ['A', 'B', 'C', 'D', 'I'], forbiddenSensors: [4] },
  cubecorners: { numAnchors: 8, dimensions: 3, axes: ['A', 'B', 'C', 'D', 'I', 'J', 'K', 'L'], forbiddenSensors: [] },
  skycam: { numAnchors: 4, dimensions: 3, axes: ['A', 'B', 'C', 'D'], forbiddenSensors: [] },
};

const MOTOR_IDS_BY_MACHINE = {
  slideprinter: ['40.0', '41.0', '42.0'],
  hangprinter_4: ['40.0', '41.0', '42.0', '43.0'],
  hangprinter_5: ['40.0', '41.0', '42.0', '43.0', '44.0'],
  cubecorners: ['40.0', '41.0', '42.0', '43.0', '44.0', '45.0', '46.0', '47.0'],
  skycam: ['40.0', '41.0', '42.0', '43.0'],
};

const DEFAULT_SWEEP_RANGE_MM = 50;
const DEFAULT_SWEEP_POINTS = 21;
const DEFAULT_MAX_SWEEPS = 6;
const DEFAULT_TORQUE = 0.05;
const DEFAULT_SETTLE_MS = 200;
const DEFAULT_SAMPLE_RATE_HZ = 40;
const DEFAULT_SUPER_SWEEP_RANGE_MM = 0;
const DEFAULT_SUPER_SWEEP_POINTS = 1;
const DEFAULT_SWEEP_METHOD = 'position';
const DEFAULT_TORQUE_LOW = 0.01;
const DEFAULT_TORQUE_MIN = 0.01;
const DEFAULT_TORQUE_MAX = 0.1;
const DEFAULT_TORQUE_STEP = 0.01;
const DEFAULT_RAMP_WAIT_MS = 1000;
const DEFAULT_SWAP_WAIT_MS = 2000;
const DEFAULT_STABILITY_POLL_MS = 500;
const DEFAULT_STABILITY_WINDOW_MS = 2000;
const DEFAULT_STABILITY_TOLERANCE_DEG = 1.0;
const DATASET_VERSION = '2.0';

const SOURCE_FILE_LABEL = 'scripts/collect_sweep_data.mjs';
let stepGcodeMode = false;
let pendingPreSendDelayMs = 0;
let stepReadline = null;

function getSourceLineFromStack(stack, { skipMatches = 1 } = {}) {
  if (!stack) {
    return null;
  }
  const matches = [...stack.matchAll(/collect_sweep_data\.mjs:(\d+):\d+/g)];
  if (matches.length === 0) {
    return null;
  }
  const idx = matches.length > skipMatches ? skipMatches : 0;
  const line = parseInt(matches[idx][1], 10);
  return Number.isFinite(line) ? line : null;
}

async function waitForEnter(message) {
  if (!process.stdin.isTTY) {
    console.log(message);
    return;
  }
  if (!stepReadline) {
    stepReadline = readline.createInterface({ input: process.stdin, output: process.stdout });
  }
  await new Promise((resolve) => stepReadline.question(message, resolve));
}

function sleep(ms) {
  const delayMs = Number(ms);
  if (!Number.isFinite(delayMs) || delayMs <= 0) {
    return Promise.resolve();
  }
  if (stepGcodeMode) {
    pendingPreSendDelayMs += delayMs;
    return Promise.resolve();
  }
  return baseSleep(delayMs);
}

async function waitForStableEncoders(sendFn, motorIds, options = {}) {
  const {
    speedup = 1,
    pollIntervalMs = DEFAULT_STABILITY_POLL_MS,
    stableWindowMs = DEFAULT_STABILITY_WINDOW_MS,
    toleranceDeg = DEFAULT_STABILITY_TOLERANCE_DEG,
    timeoutMs = null,
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return { anglesDeg: [], samples: 0, elapsedMs: 0 };
  }

  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const pollMs = Math.max(10, pollIntervalMs / timeScale);
  const windowMs = Math.max(pollMs * 2, stableWindowMs / timeScale);
  const tol = Math.max(0, Number.isFinite(toleranceDeg) ? toleranceDeg : DEFAULT_STABILITY_TOLERANCE_DEG);
  const startMs = Date.now();
  const samples = [];

  const isStable = () => {
    if (samples.length < 2) {
      return false;
    }
    const windowSpan = samples[samples.length - 1].timestampMs - samples[0].timestampMs;
    if (windowSpan < windowMs) {
      return false;
    }
    for (let motorIdx = 0; motorIdx < motorIds.length; motorIdx += 1) {
      let minVal = Number.POSITIVE_INFINITY;
      let maxVal = Number.NEGATIVE_INFINITY;
      for (let i = 0; i < samples.length; i += 1) {
        const v = samples[i].anglesDeg[motorIdx];
        if (!Number.isFinite(v)) {
          return false;
        }
        minVal = Math.min(minVal, v);
        maxVal = Math.max(maxVal, v);
      }
      if (maxVal - minVal > tol + 1e-9) {
        return false;
      }
    }
    return true;
  };

  // Poll until encoders have stayed within tolerance for the full stable window.
  // eslint-disable-next-line no-constant-condition
  while (true) {
    // eslint-disable-next-line no-await-in-loop
    const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const anglesDeg = parseEncoderReply(encoderReply?.reply);
    const nowMs = Date.now();

    if (anglesDeg.length === motorIds.length && anglesDeg.every((v) => Number.isFinite(v))) {
      samples.push({ timestampMs: nowMs, anglesDeg });
      const cutoff = nowMs - 2*windowMs;
      while (samples.length > 0 && samples[0].timestampMs < cutoff) {
        samples.shift();
      }
    } else {
      samples.length = 0;
    }

    if (isStable()) {
      return {
        anglesDeg: samples[samples.length - 1].anglesDeg.slice(),
        samples: samples.length,
        elapsedMs: nowMs - startMs,
      };
    }

    if (Number.isFinite(timeoutMs) && timeoutMs > 0 && nowMs - startMs > timeoutMs) {
      throw new Error(`Timed out waiting for encoder stability after ${Math.round(timeoutMs)}ms`);
    }

    // eslint-disable-next-line no-await-in-loop
    await sleep(pollMs);
  }
}

function printHelp() {
  console.log(`Usage: node scripts/collect_sweep_data.mjs [options]

Collect circular sweep data where N-1 anchors are fixed, one anchor drives, and another is in torque/sensor mode.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --sweepRange <mm>          Sweep half-range in mm (default: ${DEFAULT_SWEEP_RANGE_MM})
  --sweepPoints <count>      Number of points per sweep (default: ${DEFAULT_SWEEP_POINTS})
  --superSweepRange <mm>     Fixed-anchor half-range in mm (default: ${DEFAULT_SUPER_SWEEP_RANGE_MM})
  --superSweepPoints <count> Number of fixed-length samples per sweep config (default: ${DEFAULT_SUPER_SWEEP_POINTS})
  --maxSweeps <count>        Max sweeps when auto-generating configs (default: ${DEFAULT_MAX_SWEEPS})
  --feed <mm/min>            Feed rate for drive moves (default: ${DEFAULT_FEED})
  --torque <Nm>              Torque for sensor motor (default: ${DEFAULT_TORQUE})
  --settleMs <ms>            Deprecated (was fixed settle time; now waits for encoder stability)
  --speedup <scale>          hp-sim speed scale (default: 1)
  --continuous               Use continuous sweep mode (records at --sample-rate)
  --sample-rate <Hz>         Sample rate for continuous mode (default: ${DEFAULT_SAMPLE_RATE_HZ})
  --sweep-method <name>      Sweep method: position | torque-ramp (default: ${DEFAULT_SWEEP_METHOD})
  --torque-low <Nm>          torque-ramp: low/idle torque (default: ${DEFAULT_TORQUE_LOW})
  --torque-min <Nm>          torque-ramp: start torque (default: ${DEFAULT_TORQUE_MIN})
  --torque-max <Nm>          torque-ramp: end torque (default: ${DEFAULT_TORQUE_MAX})
  --torque-step <Nm>         torque-ramp: torque increment (default: ${DEFAULT_TORQUE_STEP})
  --ramp-wait-ms <ms>        Deprecated (was fixed wait; now waits for encoder stability)
  --swap-wait-ms <ms>        Deprecated (was fixed wait; now waits for encoder stability)
  --sweep-config-file <file> Provide explicit sweep configs ([fixed] drive sensor per line)
  --debug-sweep              Print planned sweep permutations before collecting
  --trace                    Tell hp-sim to plot a trace of its movements
  --output-file <path>       Output JSON path (default: sweep_data_<machine>_<timestamp>.json)
  --observability-file <path> Sidecar histogram JSON path (default: <output>.obs.json)
  --server, --rrf <url>      RRF server URL (default: http://localhost:${DEFAULT_RRF_PORT})
  --port <port>              Port for spawned rrf_simulator (default: ${DEFAULT_RRF_PORT})
  --no-spawn-rrf-simulator   Do not start rrf_simulator automatically
  --no-ws                    Disable hp-sim websocket bridge
  --ws-port <port>           WebSocket port (default: 8790)
  --no-hp-sim-reset          Do not reset hp-sim on start
  --wait-ws <ms>             Wait for hp-sim websocket connection (default: 0)
  --debug                    Verbose logging (includes G-code replies)
  --debug-gcode              Echo sent G-code
  --debug-gcode-responses    Echo G-code responses
  --step-gcode               Pause before each G-code; press Enter to send (prints source line + skipped waits)

Examples:
  node scripts/collect_sweep_data.mjs --machineType slideprinter --sweepRange 100 --sweepPoints 41 --output-file sweep.json
  node scripts/collect_sweep_data.mjs --machineType hangprinter_4 --sweep-config-file scripts/sweep_configs/hangprinter_4.txt --debug-sweep`);
}

function range(n) {
  return Array.from({ length: n }, (_, i) => i);
}

function combinations(arr, k) {
  if (k === 0) {
    return [[]];
  }
  if (!Array.isArray(arr) || arr.length < k) {
    return [];
  }
  const [first, ...rest] = arr;
  const withFirst = combinations(rest, k - 1).map((c) => [first, ...c]);
  const withoutFirst = combinations(rest, k);
  return [...withFirst, ...withoutFirst];
}

function permutations(arr, k) {
  if (k === 0) {
    return [[]];
  }
  const result = [];
  for (let i = 0; i < arr.length; i += 1) {
    const rest = [...arr.slice(0, i), ...arr.slice(i + 1)];
    for (const perm of permutations(rest, k - 1)) {
      result.push([arr[i], ...perm]);
    }
  }
  return result;
}

function buildSweepValues(rangeMm, points) {
  const count = Number.isFinite(points) ? Math.max(1, points) : 1;
  const span = Number.isFinite(rangeMm) ? Math.abs(rangeMm) : 0;
  if (count <= 1 || span < 1e-9) {
    return [0];
  }
  if (count === 2) {
    return [-span, span];
  }
  const step = (2 * span) / (count - 1);
  return Array.from({ length: count }, (_, idx) => -span + idx * step);
}

function buildDrivePositions(rangeMm, points) {
  const count = Math.max(2, points);
  const span = Math.max(0, rangeMm);
  if (span < 1e-9) {
    return Array.from({ length: count }, () => 0);
  }
  const start = -span;
  const step = (2 * span) / (count - 1);
  return Array.from({ length: count }, (_, idx) => start + idx * step);
}

function generateFixedLengthCombos(fixedCount, rangeMm, points) {
  if (!Number.isFinite(fixedCount) || fixedCount <= 0) {
    return [[]];
  }
  const values = buildSweepValues(rangeMm, points);
  const combos = [];
  const current = new Array(fixedCount);

  function helper(idx) {
    if (idx >= fixedCount) {
      combos.push([...current]);
      return;
    }
    for (const value of values) {
      current[idx] = value;
      helper(idx + 1);
    }
  }

  helper(0);
  return combos;
}

function generateSweepConfigs(machineType, forbiddenSensors = null) {
  const config = MACHINE_CONFIGS[machineType];
  if (!config) {
    throw new Error(`Unknown machine type: ${machineType}`);
  }
  const n = config.numAnchors;
  const k = config.dimensions - 1;
  const sensorBlock = new Set(forbiddenSensors ?? config.forbiddenSensors ?? []);
  const fixedCombinations = combinations(range(n), k);
  const configs = [];

  for (const fixed of fixedCombinations) {
    const fixedSet = new Set(fixed);
    const freeAnchors = range(n).filter((idx) => !fixedSet.has(idx));
    for (const [drive, sensor] of permutations(freeAnchors, 2)) {
      if (sensorBlock.has(sensor)) {
        continue;
      }
      configs.push({
        fixedAnchors: fixed,
        driveAnchor: drive,
        sensorAnchor: sensor,
      });
    }
  }

  return configs;
}

function selectRepresentativeConfigs(allConfigs, numAnchors, maxSweeps, forbiddenSensors = []) {
  const selected = [];
  const usedAsDrive = new Set();
  const usedAsSensor = new Set();
  const sensorBlock = new Set(forbiddenSensors);

  for (const cfg of allConfigs) {
    if (sensorBlock.has(cfg.sensorAnchor)) {
      continue;
    }
    if (usedAsDrive.has(cfg.driveAnchor) && usedAsSensor.has(cfg.sensorAnchor)) {
      continue;
    }
    selected.push(cfg);
    usedAsDrive.add(cfg.driveAnchor);
    usedAsSensor.add(cfg.sensorAnchor);
    if (selected.length >= maxSweeps) {
      break;
    }
  }

  if (selected.length < maxSweeps) {
    const remaining = allConfigs.filter((c) => !selected.includes(c) && !sensorBlock.has(c.sensorAnchor));
    selected.push(...remaining.slice(0, Math.max(0, maxSweeps - selected.length)));
  }

  return selected;
}

function angleToLength(angleDeg, axisIdx, mmPerDeg) {
  const factor = Array.isArray(mmPerDeg) ? mmPerDeg[axisIdx] : null;
  if (!Number.isFinite(factor)) {
    return 0;
  }
  return angleDeg * factor;
}

function parseSweepMethod(raw, fallback = DEFAULT_SWEEP_METHOD) {
  const value = (raw || fallback || '').toString().trim().toLowerCase();
  if (value === 'torque-ramp' || value === 'torqueramp' || value === 'torque_ramp') {
    return 'torque-ramp';
  }
  if (value === 'position' || value === 'pos' || value === 'drive') {
    return 'position';
  }
  return fallback;
}

function generateTorqueRampPairsFromConfigs(sweepConfigs) {
  const grouped = new Map();
  for (const cfg of sweepConfigs) {
    const fixedKey = (cfg.fixedAnchors || []).join(',');
    const pair = [cfg.driveAnchor, cfg.sensorAnchor].slice().sort((a, b) => a - b);
    const pairKey = `${pair[0]},${pair[1]}`;
    const key = `${fixedKey}|${pairKey}`;
    const entry = grouped.get(key) || { fixedAnchors: cfg.fixedAnchors, pairAnchors: pair };
    grouped.set(key, entry);
  }
  return Array.from(grouped.values());
}

function buildTorqueRampValues(minTorque, maxTorque, stepTorque) {
  const minVal = Number.isFinite(minTorque) ? minTorque : DEFAULT_TORQUE_MIN;
  const maxVal = Number.isFinite(maxTorque) ? maxTorque : DEFAULT_TORQUE_MAX;
  const stepVal = Number.isFinite(stepTorque) ? stepTorque : DEFAULT_TORQUE_STEP;
  if (stepVal <= 0) {
    return [minVal];
  }
  const start = Math.min(minVal, maxVal);
  const end = Math.max(minVal, maxVal);
  const values = [];
  for (let t = start; t <= end + 1e-12; t += stepVal) {
    values.push(Math.round(t * 1e9) / 1e9);
  }
  if (values.length === 0) {
    return [start];
  }
  const last = values[values.length - 1];
  if (Math.abs(last - end) > 1e-9) {
    values.push(end);
  }
  return values;
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

function computeAssumedLineTensionN(m666Values, axisIdx, motorTorqueNm) {
  if (!Number.isFinite(motorTorqueNm)) {
    return null;
  }
  const radiiMm = m666Values?.R;
  const mechAdv = m666Values?.U;
  const linesPerSpool = m666Values?.O;
  const motorGear = m666Values?.L;
  const spoolGear = m666Values?.H;

  const rMm = getArrayValue(radiiMm, axisIdx, null);
  if (!Number.isFinite(rMm) || Math.abs(rMm) < 1e-12) {
    return null;
  }
  const rM = rMm / 1000.0;
  const ma = Math.max(getArrayValue(mechAdv, axisIdx, 1), 1e-6);
  const lines = Math.max(getArrayValue(linesPerSpool, axisIdx, 1), 1e-6);
  const motorGearTeeth = Math.max(getArrayValue(motorGear, axisIdx, 1), 1e-6);
  const spoolGearTeeth = Math.max(getArrayValue(spoolGear, axisIdx, 1), 1e-6);
  const gearFactor = spoolGearTeeth / motorGearTeeth;
  const tension = (motorTorqueNm * gearFactor * ma * lines) / rM;
  return Number.isFinite(tension) ? tension : null;
}

async function getCurrentLengths(sendFn, motorIds, mmPerDeg) {
  const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
  const anglesDeg = parseEncoderReply(encoderReply?.reply);
  return anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
}

async function prepareTorqueRampPositioning(sendFn, task, options) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    fixedTargets = [],
    torqueLow = DEFAULT_TORQUE_LOW,
    feed = DEFAULT_FEED,
    speedup = 1,
    currentPositions = [],
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return currentPositions.slice();
  }
  if (!Array.isArray(currentPositions) || currentPositions.length !== motorIds.length) {
    throw new Error('currentPositions must match motorIds length');
  }

  await sendFn(`M569.4 P${motorIds.join(':')} T0.0`);
  const measured = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    currentPositions[idx] = Number.isFinite(measured[idx]) ? measured[idx] : 0;
  }

  const pairAnchors = task.pairAnchors || [];
  if (pairAnchors.length === 2) {
    const lowTorque = Number.isFinite(torqueLow) ? torqueLow : DEFAULT_TORQUE_LOW;
    await sendFn(`M569.4 P${motorIds[pairAnchors[0]]} T${lowTorque}`);
    await sendFn(`M569.4 P${motorIds[pairAnchors[1]]} T${lowTorque}`);
  }

  const moveParts = [];
  const formatDelta = (axis, delta) => `${axis}${delta.toFixed(3)}`;

  const fixedAnchors = task.fixedAnchors || [];
  for (let i = 0; i < fixedAnchors.length; i += 1) {
    const anchorIdx = fixedAnchors[i];
    const target = Number.isFinite(fixedTargets[i]) ? fixedTargets[i] : 0;
    const delta = target - (currentPositions[anchorIdx] ?? 0);
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatDelta(axes[anchorIdx], delta));
    }
    currentPositions[anchorIdx] = target;
  }

  for (const anchorIdx of pairAnchors) {
    const target = 0;
    const delta = target - (currentPositions[anchorIdx] ?? 0);
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatDelta(axes[anchorIdx], delta));
    }
    currentPositions[anchorIdx] = target;
  }

  if (moveParts.length > 0) {
    await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup, { axes, delayFn: sleep });
  }
  return currentPositions.slice();
}

async function prepareSweepPositioning(sendFn, sweepConfig, options) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    torque,
    driveStartPos,
    fixedTargets = [],
    feed = DEFAULT_FEED,
    speedup = 1,
    currentPositions = [],
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return currentPositions.slice();
  }
  if (!Array.isArray(currentPositions) || currentPositions.length !== motorIds.length) {
    throw new Error('currentPositions must match motorIds length');
  }

  await sendFn(`M569.4 P${motorIds.join(':')} T0.0`);
  const measured = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    currentPositions[idx] = Number.isFinite(measured[idx]) ? measured[idx] : 0;
  }

  const driveAnchor = sweepConfig.driveAnchor;
  const sensorMotorId = motorIds[sweepConfig.sensorAnchor];
  const moveParts = [];

  const formatDelta = (axis, delta) => `${axis}${delta.toFixed(3)}`;
  const driveAxis = axes[driveAnchor];
  const driveDelta = driveStartPos - (currentPositions[driveAnchor] ?? 0);
  if (Math.abs(driveDelta) > 1e-6) {
    moveParts.push(formatDelta(driveAxis, driveDelta));
  }
  currentPositions[driveAnchor] = driveStartPos;

  const fixedAnchors = sweepConfig.fixedAnchors || [];
  for (let i = 0; i < fixedAnchors.length; i += 1) {
    const anchorIdx = fixedAnchors[i];
    const target = Number.isFinite(fixedTargets[i]) ? fixedTargets[i] : 0;
    const delta = target - (currentPositions[anchorIdx] ?? 0);
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatDelta(axes[anchorIdx], delta));
    }
    currentPositions[anchorIdx] = target;
  }

  await sendFn(`M569.4 P${sensorMotorId} T${torque}`);

  if (moveParts.length > 0) {
    await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup, { axes, delayFn: sleep });
  }

  return currentPositions.slice();
}

function parseSweepConfigLine(line, lineNumber) {
  const trimmed = line.split('#')[0].trim();
  if (trimmed.length === 0) {
    return null;
  }
  const match = trimmed.match(/^\[([^\]]+)]\s+([0-9]+)\s+([0-9]+)\s*$/);
  if (!match) {
    throw new Error(`Malformed line ${lineNumber}: "${line}"`);
  }
  const fixedAnchors = match[1]
    .split(',')
    .map((part) => part.trim())
    .filter((part) => part.length > 0)
    .map((v) => parseInt(v, 10));
  const driveAnchor = parseInt(match[2], 10);
  const sensorAnchor = parseInt(match[3], 10);
  if (fixedAnchors.some((v) => Number.isNaN(v)) || Number.isNaN(driveAnchor) || Number.isNaN(sensorAnchor)) {
    throw new Error(`Invalid anchor indices on line ${lineNumber}: "${line}"`);
  }
  return { fixedAnchors, driveAnchor, sensorAnchor };
}

async function loadSweepConfigFile(filePath, machineConfig) {
  const content = await fs.readFile(filePath, 'utf8');
  const lines = content.split(/\r?\n/);
  const configs = [];
  for (let i = 0; i < lines.length; i += 1) {
    const parsed = parseSweepConfigLine(lines[i], i + 1);
    if (!parsed) {
      continue;
    }
    const { fixedAnchors, driveAnchor, sensorAnchor } = parsed;
    const all = [...fixedAnchors, driveAnchor, sensorAnchor];
    if (all.some((idx) => idx < 0 || idx >= machineConfig.numAnchors)) {
      throw new Error(`Anchor index out of range on line ${i + 1}`);
    }
    if (machineConfig.forbiddenSensors?.includes(sensorAnchor)) {
      throw new Error(`Forbidden sensor anchor on line ${i + 1}`);
    }
    configs.push(parsed);
  }
  if (configs.length === 0) {
    throw new Error(`No sweep configurations found in ${filePath}`);
  }
  return configs;
}

function validateSweepConfig(config, machineConfig) {
  const errors = [];
  const requiredFixed = machineConfig.dimensions - 1;
  if (config.fixedAnchors.length !== requiredFixed) {
    errors.push(`expected ${requiredFixed} fixed anchors, got ${config.fixedAnchors.length}`);
  }
  const all = [...config.fixedAnchors, config.driveAnchor, config.sensorAnchor];
  if (all.some((idx) => idx < 0 || idx >= machineConfig.numAnchors)) {
    errors.push('anchor index out of range');
  }
  if (new Set(all).size !== config.fixedAnchors.length + 2) {
    errors.push('duplicate anchor roles in sweep config');
  }
  if (machineConfig.forbiddenSensors?.includes(config.sensorAnchor)) {
    errors.push('forbidden sensor anchor');
  }
  if (errors.length > 0) {
    throw new Error(`Invalid sweep config (fix [${config.fixedAnchors.join(', ')}], drive ${config.driveAnchor}, sensor ${config.sensorAnchor}): ${errors.join('; ')}`);
  }
}

async function performSweep(sendFn, sweepConfig, options) {
  const {
    axes,
    motorIds,
    sweepRangeMm,
    sweepPoints,
    feed,
    speedup,
    mmPerDeg,
    datasetStartMs,
    drivePositions,
  } = options;
  const { driveAnchor, sensorAnchor } = sweepConfig;
  const driveAxis = axes[driveAnchor];
  const path = Array.isArray(drivePositions) && drivePositions.length === sweepPoints
    ? drivePositions
    : buildDrivePositions(sweepRangeMm, sweepPoints);
  const dataPoints = [];

  for (let i = 0; i < sweepPoints; i += 1) {
    if (i > 0) {
      const delta = path[i] - path[i - 1];
      if (Math.abs(delta) > 1e-9) {
        // eslint-disable-next-line no-await-in-loop
        await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${delta} F${feed}`, speedup, { axes, delayFn: sleep });
      }
    }
    // eslint-disable-next-line no-await-in-loop
    const stable = await waitForStableEncoders(sendFn, motorIds, { speedup });
    const anglesDeg = stable.anglesDeg;
    const lengths = anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
    dataPoints.push({
      l_drive: lengths[driveAnchor],
      l_sensor: lengths[sensorAnchor],
      timestamp_ms: Date.now() - datasetStartMs,
      raw_angles_deg: anglesDeg,
      drive_setpoint_mm: path[i],
    });
  }

  return dataPoints;
}

async function performContinuousSweep(sendFn, sweepConfig, options) {
  const {
    axes,
    motorIds,
    sweepRangeMm,
    feed,
    sampleRateHz,
    mmPerDeg,
    datasetStartMs,
    driveStartMm,
    driveEndMm,
  } = options;
  const { driveAnchor, sensorAnchor } = sweepConfig;
  const driveAxis = axes[driveAnchor];
  const dataPoints = [];
  let collecting = true;
  const samplingInterval = 1000 / Math.max(1, sampleRateHz);

  const sampler = (async () => {
    while (collecting) {
      // eslint-disable-next-line no-await-in-loop
      const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
      const angles = parseEncoderReply(encoderReply?.reply);
      const lengths = angles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
      dataPoints.push({
        l_drive: lengths[driveAnchor],
        l_sensor: lengths[sensorAnchor],
        timestamp_ms: Date.now() - datasetStartMs,
        raw_angles_deg: angles,
      });
      // eslint-disable-next-line no-await-in-loop
      await sleep(samplingInterval);
    }
  })();

  try {
    const leg = driveEndMm - driveStartMm;
    if (Math.abs(leg) > 1e-9) {
      await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${leg} F${feed}`, 1, { axes, delayFn: sleep });
      await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${-leg} F${feed}`, 1, { axes, delayFn: sleep });
    }
  } finally {
    collecting = false;
    try {
      await sampler;
    } catch (_err) {
      /* best effort */
    }
  }

  return dataPoints;
}

async function performTorqueRampSweep(sendFn, task, options) {
  const {
    axes,
    motorIds,
    mmPerDeg,
    datasetStartMs,
    speedup,
    fixedAnchors,
    fixedLengths,
    feed,
    torqueLow,
    torqueMin,
    torqueMax,
    torqueStep,
    m666Values,
  } = options;

  const pairAnchors = task.pairAnchors || [];
  if (pairAnchors.length !== 2) {
    throw new Error('torque-ramp sweep requires exactly two free anchors');
  }
  const [anchorA, anchorB] = pairAnchors.slice().sort((a, b) => a - b);
  const rampTorques = buildTorqueRampValues(torqueMin, torqueMax, torqueStep);
  const sweepStartLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);

  const dataPoints = [];
  const phases = [
    { drive: anchorA, sensor: anchorB, phase: 'A_low_B_ramp' },
    { drive: anchorB, sensor: anchorA, phase: 'B_low_A_ramp' },
  ];

  for (let phaseIdx = 0; phaseIdx < phases.length; phaseIdx += 1) {
    const phase = phases[phaseIdx];
    const driveMotorId = motorIds[phase.drive];
    const sensorMotorId = motorIds[phase.sensor];
    // eslint-disable-next-line no-await-in-loop
    await sendFn(`M569.4 P${driveMotorId} T${torqueLow}`);
    // eslint-disable-next-line no-await-in-loop
    await sendFn(`M569.4 P${sensorMotorId} T${rampTorques[0]}`);

    for (let i = 0; i < rampTorques.length; i += 1) {
      const sensorTorque = rampTorques[i];
      if (i > 0) {
        // eslint-disable-next-line no-await-in-loop
        await sendFn(`M569.4 P${sensorMotorId} T${sensorTorque}`);
      }

      // eslint-disable-next-line no-await-in-loop
      const stable = await waitForStableEncoders(sendFn, motorIds, { speedup });
      const anglesDeg = stable.anglesDeg;
      const lengths = anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));

      const torqueByAnchor = new Map([
        [phase.drive, torqueLow],
        [phase.sensor, sensorTorque],
      ]);
      const torqueA = torqueByAnchor.get(anchorA) ?? 0;
      const torqueB = torqueByAnchor.get(anchorB) ?? 0;

      dataPoints.push({
        l_drive: lengths[anchorA],
        l_sensor: lengths[anchorB],
        timestamp_ms: Date.now() - datasetStartMs,
        raw_angles_deg: anglesDeg,
        fixed_anchors: fixedAnchors,
        fixed_lengths: fixedLengths,
        torque_drive_nm: torqueA,
        torque_sensor_nm: torqueB,
        assumed_tension_drive_n: computeAssumedLineTensionN(m666Values, anchorA, torqueA),
        assumed_tension_sensor_n: computeAssumedLineTensionN(m666Values, anchorB, torqueB),
        phase: phase.phase,
        torque_step_index: i,
        torque_step_count: rampTorques.length,
      });
    }

    if (phaseIdx === 0) {
      const firstRampingAnchor = phase.sensor;
      const axis = axes?.[firstRampingAnchor];
      if (!axis) {
        throw new Error('torque-ramp requires axes mapping for reposition step');
      }

      // Put the first ramping motor into position mode and move it back to its sweep start position.
      // eslint-disable-next-line no-await-in-loop
      await sendFn(`M569.4 P${motorIds[firstRampingAnchor]} T0.0`);
      // eslint-disable-next-line no-await-in-loop
      const lengthsAfter = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
      const current = lengthsAfter[firstRampingAnchor] ?? 0;
      const target = sweepStartLengths[firstRampingAnchor] ?? 0;
      const delta = target - current;
      if (Math.abs(delta) > 1e-6) {
        // eslint-disable-next-line no-await-in-loop
        await runMoveWithWait(sendFn, `G1 H2 ${axis}${delta.toFixed(3)} F${feed}`, speedup, { axes, delayFn: sleep });
      }
    }
  }

  {
    const lastPhase = phases[phases.length - 1];
    const lastDriveAnchor = lastPhase.drive;
    const lastSensorAnchor = lastPhase.sensor;
    const axis = axes?.[lastDriveAnchor];
    if (!axis) {
      throw new Error('torque-ramp requires axes mapping for reposition step');
    }

    // Drop the last ramping motor back to low torque and move the last drive motor back to its sweep start position.
    await sendFn(`M569.4 P${motorIds[lastSensorAnchor]} T${torqueLow}`);
    await sendFn(`M569.4 P${motorIds[lastDriveAnchor]} T0.0`);
    const lengthsAfter = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
    const current = lengthsAfter[lastDriveAnchor] ?? 0;
    const target = sweepStartLengths[lastDriveAnchor] ?? 0;
    const delta = target - current;
    if (Math.abs(delta) > 1e-6) {
      await runMoveWithWait(sendFn, `G1 H2 ${axis}${delta.toFixed(3)} F${feed}`, speedup, { axes, delayFn: sleep });
    }
    await sendFn(`M569.4 P${motorIds[lastDriveAnchor]} T${torqueLow}`);
  }

  return dataPoints;
}

function buildHistogram(dataPoints, bucketCount = 20) {
  if (!Array.isArray(dataPoints) || dataPoints.length === 0) {
    return [];
  }
  const drives = dataPoints.map((p) => p.l_drive);
  const sensors = dataPoints.map((p) => p.l_sensor);
  const minDrive = Math.min(...drives);
  const maxDrive = Math.max(...drives);
  if (!Number.isFinite(minDrive) || !Number.isFinite(maxDrive) || maxDrive - minDrive < 1e-9) {
    return [];
  }
  const bucketWidth = (maxDrive - minDrive) / bucketCount;
  const buckets = Array.from({ length: bucketCount }, (_, idx) => ({
    start: minDrive + idx * bucketWidth,
    end: minDrive + (idx + 1) * bucketWidth,
    count: 0,
    mean_drive: 0,
    mean_sensor: 0,
    var_drive: 0,
    var_sensor: 0,
  }));

  for (let i = 0; i < dataPoints.length; i += 1) {
    const drive = drives[i];
    const sensor = sensors[i];
    const idx = Math.min(
      buckets.length - 1,
      Math.max(0, Math.floor((drive - minDrive) / bucketWidth)),
    );
    const bucket = buckets[idx];
    bucket.count += 1;
    const deltaDrive = drive - bucket.mean_drive;
    bucket.mean_drive += deltaDrive / bucket.count;
    const deltaSensor = sensor - bucket.mean_sensor;
    bucket.mean_sensor += deltaSensor / bucket.count;
    bucket.var_drive += deltaDrive * (drive - bucket.mean_drive);
    bucket.var_sensor += deltaSensor * (sensor - bucket.mean_sensor);
  }

  for (const bucket of buckets) {
    if (bucket.count > 1) {
      bucket.var_drive /= bucket.count - 1;
      bucket.var_sensor /= bucket.count - 1;
    } else {
      bucket.var_drive = 0;
      bucket.var_sensor = 0;
    }
  }
  return buckets;
}

async function writeObservabilitySidecar(targetPath, sweeps) {
  if (!sweeps || sweeps.length === 0 || !targetPath) {
    return null;
  }
  const obsFile = targetPath.toLowerCase().endsWith('.json') ? targetPath : `${targetPath}.json`;
  const payload = {
    generated_at: new Date().toISOString(),
    sweeps: sweeps.map((sweep) => ({
      id: sweep.id,
      histogram: buildHistogram(sweep.data_points),
    })),
  };
  const dir = path.dirname(obsFile);
  if (dir && dir !== '.') {
    await fs.mkdir(dir, { recursive: true });
  }
  await fs.writeFile(obsFile, JSON.stringify(payload, null, 2));
  return obsFile;
}

function buildG92Command(axes) {
  const parts = (axes || []).map((axis) => `${axis}0`);
  if (parts.length === 0) {
    return 'G92';
  }
  return `G92 ${parts.join(' ')}`;
}

async function main() {
  const args = parseBridgeArgs(process.argv.slice(2));
  if (args.help) {
    printHelp();
    process.exit(0);
  }
  stepGcodeMode = !!args.stepGcode;
  const machineType = (args.machineType || 'slideprinter').toLowerCase();
  const machineConfig = MACHINE_CONFIGS[machineType];
  if (!machineConfig) {
    console.error(`Unknown machine type: ${machineType}`);
    process.exit(1);
  }
  const motorIds = MOTOR_IDS_BY_MACHINE[machineType];
  if (!motorIds || motorIds.length !== machineConfig.numAnchors) {
    console.error(`Motor ID mapping missing or mismatched for ${machineType}`);
    process.exit(1);
  }
  if (motorIds.length !== machineConfig.axes.length) {
    console.error(`Motor ID count (${motorIds.length}) does not match axes count (${machineConfig.axes.length}) for ${machineType}`);
    process.exit(1);
  }
  const sweepRangeInput = Number.isFinite(parseFloat(args.sweepRange))
    ? parseFloat(args.sweepRange)
    : DEFAULT_SWEEP_RANGE_MM;
  const sweepRangeMm = Math.abs(sweepRangeInput);
  const sweepPoints = Number.isFinite(parseInt(args.sweepPoints, 10))
    ? parseInt(args.sweepPoints, 10)
    : DEFAULT_SWEEP_POINTS;
  const superSweepRangeMm = Number.isFinite(parseFloat(args.superSweepRange))
    ? Math.abs(parseFloat(args.superSweepRange))
    : DEFAULT_SUPER_SWEEP_RANGE_MM;
  const superSweepPoints = Number.isFinite(parseInt(args.superSweepPoints, 10))
    ? Math.max(1, parseInt(args.superSweepPoints, 10))
    : DEFAULT_SUPER_SWEEP_POINTS;
  const maxSweeps = Number.isFinite(parseInt(args.maxSweeps, 10))
    ? parseInt(args.maxSweeps, 10)
    : DEFAULT_MAX_SWEEPS;
  if (sweepPoints < 2) {
    console.error('sweepPoints must be at least 2');
    process.exit(1);
  }
  const maxSweepCount = Math.max(1, maxSweeps);
  const feed = Number.isFinite(parseFloat(args.feed)) ? parseFloat(args.feed) : DEFAULT_FEED;
  const torque = Number.isFinite(parseFloat(args.torque)) ? parseFloat(args.torque) : DEFAULT_TORQUE;
  const sweepMethod = parseSweepMethod(args.sweepMethod, DEFAULT_SWEEP_METHOD);
  const torqueLow = Number.isFinite(parseFloat(args.torqueLow)) ? parseFloat(args.torqueLow) : DEFAULT_TORQUE_LOW;
  const torqueMin = Number.isFinite(parseFloat(args.torqueMin)) ? parseFloat(args.torqueMin) : DEFAULT_TORQUE_MIN;
  const torqueMax = Number.isFinite(parseFloat(args.torqueMax)) ? parseFloat(args.torqueMax) : DEFAULT_TORQUE_MAX;
  const torqueStep = Number.isFinite(parseFloat(args.torqueStep)) ? parseFloat(args.torqueStep) : DEFAULT_TORQUE_STEP;
  const rampWaitMs = Number.isFinite(parseFloat(args.rampWaitMs)) ? Math.max(0, parseFloat(args.rampWaitMs)) : DEFAULT_RAMP_WAIT_MS;
  const swapWaitMs = Number.isFinite(parseFloat(args.swapWaitMs)) ? Math.max(0, parseFloat(args.swapWaitMs)) : DEFAULT_SWAP_WAIT_MS;
  const settleMs = Number.isFinite(parseFloat(args.settleMs))
    ? Math.max(0, parseFloat(args.settleMs))
    : DEFAULT_SETTLE_MS;
  const speedup = Number.isFinite(parseFloat(args.speedup)) && parseFloat(args.speedup) > 0
    ? parseFloat(args.speedup)
    : 1;
  const sampleRateHz = Number.isFinite(parseFloat(args.sampleRate))
    ? parseFloat(args.sampleRate)
    : DEFAULT_SAMPLE_RATE_HZ;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : 0;

  if (sweepMethod === 'torque-ramp' && args.continuous) {
    console.error('torque-ramp sweep method does not support --continuous');
    process.exit(1);
  }

  let sweepConfigs = null;
  if (args.sweepConfigFile) {
    sweepConfigs = await loadSweepConfigFile(args.sweepConfigFile, machineConfig);
  } else {
    sweepConfigs = generateSweepConfigs(machineType);
    if (sweepConfigs.length > maxSweepCount) {
      sweepConfigs = selectRepresentativeConfigs(
        sweepConfigs,
        machineConfig.numAnchors,
        maxSweepCount,
        machineConfig.forbiddenSensors,
      );
    }
  }

  for (const cfg of sweepConfigs) {
    validateSweepConfig(cfg, machineConfig);
  }

  if (args.debugSweep) {
    console.log(`Planned sweeps (${sweepConfigs.length}):`);
    for (const sweep of sweepConfigs) {
      console.log(`  fix [${sweep.fixedAnchors.join(', ')}], drive ${sweep.driveAnchor}, sensor ${sweep.sensorAnchor}`);
    }
  }

  const targetPort = Number.isFinite(parseInt(args.port, 10)) ? parseInt(args.port, 10) : DEFAULT_RRF_PORT;
  const targetServer = args.serverExplicit ? args.server : `http://localhost:${targetPort}`;
  const shouldSpawnRrf = !args.noSpawnRrfSimulator && !args.serverExplicit;
  let rrfProcess = null;

  if (shouldSpawnRrf) {
    try {
      console.log(`Starting rrf_simulator at ${targetServer}...`);
      rrfProcess = await startRrfSimulator({ port: targetPort, debug: args.debug });
      await waitForRrfSimulator(targetServer);
    } catch (err) {
      console.error(`Unable to start rrf_simulator: ${err?.message || err}`);
      process.exit(1);
    }
  }

  const bridgeCtx = createGcodeBridge({
    server: targetServer,
    wsPort: args.noWs ? 0 : args.wsPort,
    quiet: args.quiet,
    encoderTimeoutMs,
  });

  const send = async (line, options = {}) => {
    const trimmed = line?.trim?.();
    if (args.stepGcode && trimmed) {
      const normalPreWaitMs = pendingPreSendDelayMs;
      pendingPreSendDelayMs = 0;
      const sourceLine = getSourceLineFromStack(new Error().stack, { skipMatches: 1 });
      const sourceLabel = sourceLine ? `${SOURCE_FILE_LABEL}:${sourceLine}` : SOURCE_FILE_LABEL;
      await waitForEnter(
        `Send: ${trimmed}\n  Enter to send (normal pre-wait ${Math.round(normalPreWaitMs)}ms; from ${sourceLabel}) `,
      );
    } else if (args.debugGcode && trimmed) {
      console.log(`[rrf_gcode] ${trimmed}`);
    }
    const res = await bridgeCtx.sendGcodeLine(line, options);
    if (args.stepGcode) {
      const reply = res?.reply?.trim?.() || '';
      console.log(reply.length > 0 ? `[rrf_reply] ${reply}` : '[rrf_reply] <empty>');
    } else if ((args.debugGcodeResponses || args.debug) && res?.reply) {
      const reply = res.reply.trim();
      if (reply.length > 0) {
        console.log(`[rrf_reply] ${reply}`);
      }
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
    if (args.trace) {
      await sendHpSimPositionTraceMode(bridgeCtx, true, { quiet: args.quiet });
    }
  }

  let success = false;
  const sweeps = [];
  const datasetStartMs = Date.now();
  const currentPositions = Array.from({ length: motorIds.length }, () => 0);
  const baseDrivePositions = buildDrivePositions(sweepRangeMm, sweepPoints);
  const reverseDrivePositions = [...baseDrivePositions].reverse();
  const sweepTasks = [];

  const sweepGroups = sweepMethod === 'torque-ramp'
    ? generateTorqueRampPairsFromConfigs(sweepConfigs)
    : null;
  const taskSources = sweepGroups ?? sweepConfigs;
  for (const config of taskSources) {
    const combos = generateFixedLengthCombos(config.fixedAnchors.length, superSweepRangeMm, superSweepPoints);
    for (let comboIdx = 0; comboIdx < combos.length; comboIdx += 1) {
      sweepTasks.push({
        config,
        fixedTargets: combos[comboIdx] || [],
        fixedComboIndex: comboIdx,
        fixedComboCount: combos.length,
      });
    }
  }

  const plannedPositions = Array.from({ length: motorIds.length }, () => 0);
  const plannedSweeps = sweepTasks.map((task) => {
    if (sweepMethod === 'torque-ramp') {
      const pairAnchors = task.config.pairAnchors;
      return {
        ...task,
        pairAnchors,
      };
    }
    const driveIdx = task.config.driveAnchor;
    const currentDrivePos = plannedPositions[driveIdx] ?? 0;
    const forwardDist = Math.abs(currentDrivePos - baseDrivePositions[0]);
    const reverseDist = Math.abs(currentDrivePos - reverseDrivePositions[0]);
    const useReverse = reverseDist + 1e-9 < forwardDist;
    const drivePath = useReverse ? reverseDrivePositions : baseDrivePositions;
    const driveStartPos = drivePath[0];
    const driveEndPos = drivePath[drivePath.length - 1];
    const directionLabel = useReverse ? 'decreasing' : 'increasing';
    plannedPositions[driveIdx] = driveEndPos;
    return {
      ...task,
      drivePath,
      driveStartPos,
      driveEndPos,
      directionLabel,
    };
  });

  const totalPlannedSweeps = plannedSweeps.length || sweepConfigs.length;
  let sweepOrdinal = 0;

  try {
    const m666BeforeReply = await send('M666');
    const m666Before = parseM666(m666BeforeReply?.reply);
    const m669Reply = await send('M669');
    const m669Values = parseM666(m669Reply?.reply);
    const m92Reply = await send('M92');
    const m92Values = parseM666(m92Reply?.reply);

    await send('M666 Q0'); // Set buildup factor to zero for data collection
    const m666AfterReply = await send('M666');
    const m666After = parseM666(m666AfterReply?.reply);

    const mmPerDeg = machineConfig.axes.map((_, idx) => computeMmPerDegree(m666After, idx));
    const missingAxes = mmPerDeg
      .map((val, idx) => (Number.isFinite(val) ? null : machineConfig.axes[idx]))
      .filter(Boolean);
    if (missingAxes.length > 0) {
      console.warn(`Warning: missing mm/deg calibration for axes [${missingAxes.join(', ')}]; lengths will default to 0 on those axes.`);
    }

    await send(buildG92Command(machineConfig.axes));
    await send('G91'); // Use relative coordinates
    await send(`M569.3 P${motorIds.join(':')} S`); // Set encoder reference point

    for (const plan of plannedSweeps) {
      const {
        config: sweepConfig,
        fixedTargets,
        fixedComboIndex,
        fixedComboCount,
      } = plan;
      sweepOrdinal += 1;
      const fixedDesc = fixedTargets.length > 0
        ? fixedTargets.map((val) => val.toFixed(3)).join(', ')
        : '';
      const fixedInfo = fixedDesc.length > 0 ? `, fixed targets [${fixedDesc}]` : '';
      if (sweepMethod === 'torque-ramp') {
        const pairAnchors = plan.pairAnchors;
        const forbidden = new Set(machineConfig.forbiddenSensors ?? []);
        if (pairAnchors.some((idx) => forbidden.has(idx))) {
          console.warn(`Skipping sweep (torque-ramp needs torque-capable anchors): fix [${sweepConfig.fixedAnchors.join(', ')}], pair [${pairAnchors.join(', ')}]`);
          continue;
        }
        const canonicalPair = pairAnchors.slice().sort((a, b) => a - b);
        console.log(`\nSweep ${sweepOrdinal}/${totalPlannedSweeps}: fix [${sweepConfig.fixedAnchors.join(', ')}], pair [${canonicalPair.join(', ')}]${fixedInfo}, method torque-ramp`);

        const preparedPositions = await prepareTorqueRampPositioning(send, {
          fixedAnchors: sweepConfig.fixedAnchors,
          pairAnchors: canonicalPair,
        }, {
          motorIds,
          axes: machineConfig.axes,
          mmPerDeg,
          fixedTargets,
          torqueLow,
          feed,
          speedup,
          currentPositions,
        });

        const fixedLengths = sweepConfig.fixedAnchors.map((idx, anchorIdx) => {
          if (Number.isFinite(preparedPositions[idx])) {
            return preparedPositions[idx];
          }
          return Number.isFinite(fixedTargets[anchorIdx]) ? fixedTargets[anchorIdx] : 0;
        });

        const dataPoints = await performTorqueRampSweep(send, {
          pairAnchors: canonicalPair,
        }, {
          axes: machineConfig.axes,
          motorIds,
          mmPerDeg,
          datasetStartMs,
          speedup,
          fixedAnchors: sweepConfig.fixedAnchors,
          fixedLengths,
          feed,
          torqueLow,
          torqueMin,
          torqueMax,
          torqueStep,
          rampWaitMs,
          swapWaitMs,
          m666Values: m666After,
        });

        sweeps.push({
          id: `sweep_${String(sweepOrdinal).padStart(3, '0')}`,
          fixed_anchors: sweepConfig.fixedAnchors,
          fixed_lengths: fixedLengths,
          drive_anchor: canonicalPair[0],
          sensor_anchor: canonicalPair[1],
          drive_range: null,
          data_points: dataPoints,
          metadata: {
            sweep_method: 'torque-ramp',
            feed_rate: feed,
            fixed_combo_index: fixedComboIndex,
            fixed_combo_count: fixedComboCount,
            torque_low_nm: torqueLow,
            torque_min_nm: torqueMin,
            torque_max_nm: torqueMax,
            torque_step_nm: torqueStep,
            ramp_wait_ms: rampWaitMs,
            swap_wait_ms: swapWaitMs,
            speedup,
          },
        });
        console.log(`  Collected ${dataPoints.length} points`);
        continue;
      }

      const {
        drivePath,
        driveStartPos,
        driveEndPos,
        directionLabel,
      } = plan;

      console.log(`\nSweep ${sweepOrdinal}/${totalPlannedSweeps}: fix [${sweepConfig.fixedAnchors.join(', ')}], drive ${sweepConfig.driveAnchor}, sensor ${sweepConfig.sensorAnchor}${fixedInfo}, drive order ${directionLabel}`);

      const preparedPositions = await prepareSweepPositioning(send, sweepConfig, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        torque,
        driveStartPos,
        fixedTargets,
        feed,
        speedup,
        currentPositions,
      });

      const dataPoints = args.continuous
        ? await performContinuousSweep(send, sweepConfig, {
          axes: machineConfig.axes,
          motorIds,
          sweepRangeMm,
          feed,
          sampleRateHz,
          mmPerDeg,
          datasetStartMs,
          driveStartMm: driveStartPos,
          driveEndMm: driveEndPos,
        })
        : await performSweep(send, sweepConfig, {
          axes: machineConfig.axes,
          motorIds,
          sweepRangeMm,
          sweepPoints,
          feed,
          settleMs,
          speedup,
          mmPerDeg,
          datasetStartMs,
          drivePositions: drivePath,
        });

      const fixedLengths = sweepConfig.fixedAnchors.map((idx, anchorIdx) => {
        if (Number.isFinite(preparedPositions[idx])) {
          return preparedPositions[idx];
        }
        return Number.isFinite(fixedTargets[anchorIdx]) ? fixedTargets[anchorIdx] : 0;
      });

      sweeps.push({
        id: `sweep_${String(sweepOrdinal).padStart(3, '0')}`,
        fixed_anchors: sweepConfig.fixedAnchors,
        fixed_lengths: fixedLengths,
        drive_anchor: sweepConfig.driveAnchor,
        sensor_anchor: sweepConfig.sensorAnchor,
        drive_range: { start: -sweepRangeMm, end: sweepRangeMm, unit: 'mm' },
        data_points: dataPoints,
        metadata: {
          sweep_method: 'position',
          feed_rate: feed,
          torque,
          settle_ms: settleMs,
          fixed_combo_index: fixedComboIndex,
          fixed_combo_count: fixedComboCount,
          sample_rate_hz: args.continuous ? sampleRateHz : undefined,
          drive_direction: directionLabel,
        },
      });
      console.log(`  Collected ${dataPoints.length} points`);
    }

    const dataset = {
      version: DATASET_VERSION,
      machine_type: machineType,
      num_anchors: machineConfig.numAnchors,
      dimensions: machineConfig.dimensions,
      timestamp: new Date().toISOString(),
      config: {
        angles_unit: 'deg',
        lengths_unit: 'mm',
        m666: m666After,
        m666_before: m666Before,
        m669: m669Values,
        m92: m92Values,
        mm_per_degree: mmPerDeg,
        notes: {
          buildup_factor_forced: 0,
        },
      },
      sweeps,
    };

    const outputFile = args.outputFile || `sweep_data_${machineType}_${Date.now()}.json`;
    const outputDir = path.dirname(outputFile);
    if (outputDir && outputDir !== '.') {
      await fs.mkdir(outputDir, { recursive: true });
    }
    await fs.writeFile(outputFile, JSON.stringify(dataset, null, 2));
    console.log(`\nSaved ${sweeps.length} sweeps to ${outputFile}`);

    const obsTarget = args.observabilityFile === null
      ? `${outputFile.replace(/\.json$/i, '')}.obs.json`
      : args.observabilityFile;
    const sidecarPath = await writeObservabilitySidecar(obsTarget, sweeps);
    if (sidecarPath) {
      console.log(`Saved histogram sidecar to ${sidecarPath}`);
    }

    success = true;
  } catch (err) {
    console.error(`Failed to collect sweeps: ${err?.message || err}`);
	  } finally {
	    if (rrfProcess && !args.persistRrfSimulator) {
	      stopProcess(rrfProcess);
	    }
	    if (stepReadline) {
	      stepReadline.close();
	      stepReadline = null;
	    }
	    bridgeCtx.close();
	    process.exit(success ? 0 : 1);
	  }
	}

export {
  angleToLength,
  combinations,
  generateSweepConfigs,
  loadSweepConfigFile,
  permutations,
  selectRepresentativeConfigs,
  validateSweepConfig,
};

const isMain = import.meta.url === pathToFileURL(process.argv[1] || '').href;
if (isMain) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
