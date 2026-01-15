import fs from 'node:fs/promises';
import path from 'node:path';
import {
  computeMmPerDegree,
  parseEncoderReply,
  parseM666,
  runMoveWithWait,
  sleep as baseSleep,
} from './encoder_utils.mjs';
import {
  angleToLength,
  applyForceModeState,
  getCurrentLengths,
  primeEncoders,
  returnMotorsToOriginAllAtOnce,
  returnMotorsToOriginOneAtATime,
  waitForStableEncoders,
} from './uncalibrated_actions.mjs';
import { FORCE_TUNING_DEFAULTS, tuneForce } from './force_tuning.mjs';

export const MACHINE_CONFIGS = {
  slideprinter: { numAnchors: 3, dimensions: 2, axes: ['X', 'Y', 'Z'], forbiddenSensors: [] },
  hangprinter_4: { numAnchors: 4, dimensions: 3, axes: ['A', 'B', 'C', 'D'], forbiddenSensors: [3] },
  hangprinter_5: { numAnchors: 5, dimensions: 3, axes: ['A', 'B', 'C', 'D', 'I'], forbiddenSensors: [4] },
  cubecorners: { numAnchors: 8, dimensions: 3, axes: ['A', 'B', 'C', 'D', 'I', 'J', 'K', 'L'], forbiddenSensors: [] },
  skycam: { numAnchors: 4, dimensions: 3, axes: ['A', 'B', 'C', 'D'], forbiddenSensors: [] },
};

export const MOTOR_IDS_BY_MACHINE = {
  slideprinter: ['40.0', '41.0', '42.0'],
  hangprinter_4: ['40.0', '41.0', '42.0', '43.0'],
  hangprinter_5: ['40.0', '41.0', '42.0', '43.0', '44.0'],
  cubecorners: ['40.0', '41.0', '42.0', '43.0', '44.0', '45.0', '46.0', '47.0'],
  skycam: ['40.0', '41.0', '42.0', '43.0'],
};

const DEFAULT_SWEEP_POINTS = 21;
const DEFAULT_MAX_SWEEPS = 6;
const DEFAULT_FEED = 1400;
const DEFAULT_FORCE_MID_N = 0.05;
const DEFAULT_SETTLE_MS = 200;
const DEFAULT_SAMPLE_RATE_HZ = 40;
const DEFAULT_SUPER_SWEEP_RANGE_MM = 0;
const DEFAULT_SUPER_SWEEP_POINTS = 1;
const DATASET_VERSION = '2.0';

export const SWEEP_DEFAULTS = {
  DEFAULT_SWEEP_POINTS,
  DEFAULT_MAX_SWEEPS,
  DEFAULT_FEED,
  DEFAULT_FORCE_MID_N,
  DEFAULT_SETTLE_MS,
  DEFAULT_SAMPLE_RATE_HZ,
  DEFAULT_SUPER_SWEEP_RANGE_MM,
  DEFAULT_SUPER_SWEEP_POINTS,
  DATASET_VERSION,
};

function range(n) {
  return Array.from({ length: n }, (_, i) => i);
}

export function combinations(arr, k) {
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

export function permutations(arr, k) {
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

function parseFixedTargetsSpec(spec, fixedCount) {
  if (!spec) {
    return null;
  }
  if (!Number.isFinite(fixedCount) || fixedCount <= 0) {
    return [[]];
  }
  const raw = spec.toString().trim();
  if (raw.length === 0) {
    return null;
  }
  const groups = raw.split(';').map((entry) => entry.trim()).filter(Boolean);
  if (groups.length === 0) {
    return null;
  }
  const combos = [];
  for (const group of groups) {
    const values = group.split(',').map((entry) => parseFloat(entry.trim()));
    if (values.some((val) => !Number.isFinite(val))) {
      continue;
    }
    combos.push(values);
  }
  return combos.length > 0 ? combos : null;
}

function normalizeFixedTargetCombos(combos, fixedCount) {
  if (!Array.isArray(combos)) {
    return null;
  }
  const normalized = combos.map((combo) => {
    if (!Array.isArray(combo)) {
      return [];
    }
    const trimmed = combo.slice(0, fixedCount).map((val) => (Number.isFinite(val) ? val : 0));
    if (trimmed.length < fixedCount) {
      const missing = fixedCount - trimmed.length;
      for (let i = 0; i < missing; i += 1) {
        trimmed.push(0);
      }
    }
    return trimmed;
  });
  return normalized.length > 0 ? normalized : null;
}

function parseMaxTravelValue(spec) {
  if (spec == null) {
    return null;
  }
  const raw = spec.toString().trim();
  if (raw.length === 0) {
    return null;
  }
  if (raw.includes(',') || raw.includes(';')) {
    return null;
  }
  const value = parseFloat(raw);
  return Number.isFinite(value) ? value : null;
}

function resolveFixedLengthCombos(fixedCount, rangeMm, points, fixedTargetsSpec) {
  const targetCombos = normalizeFixedTargetCombos(
    parseFixedTargetsSpec(fixedTargetsSpec, fixedCount),
    fixedCount,
  );
  if (targetCombos) {
    return targetCombos;
  }
  return generateFixedLengthCombos(fixedCount, rangeMm, points);
}

export function generateSweepConfigs(machineType, forbiddenSensors = null) {
  const machineConfig = MACHINE_CONFIGS[machineType];
  if (!machineConfig) {
    return [];
  }
  const numAnchors = machineConfig.numAnchors;
  const allAnchors = range(numAnchors);
  const forbidden = forbiddenSensors || machineConfig.forbiddenSensors || [];
  const configs = [];

  for (const fixedAnchors of combinations(allAnchors, numAnchors - 2)) {
    const freeAnchors = allAnchors.filter((idx) => !fixedAnchors.includes(idx));
    for (const [driveAnchor, sensorAnchor] of permutations(freeAnchors, 2)) {
      if (forbidden.includes(sensorAnchor)) {
        continue;
      }
      configs.push({ fixedAnchors, driveAnchor, sensorAnchor });
    }
  }

  return configs;
}

function selectSizeTunePair(sweepConfigs, forbiddenSensors = []) {
  if (!Array.isArray(sweepConfigs) || sweepConfigs.length === 0) {
    return null;
  }
  const forbidden = new Set(forbiddenSensors ?? []);
  for (const cfg of sweepConfigs) {
    const drive = cfg?.driveAnchor;
    const sensor = cfg?.sensorAnchor;
    if (!Number.isFinite(drive) || !Number.isFinite(sensor)) {
      continue;
    }
    if (forbidden.has(drive) || forbidden.has(sensor)) {
      continue;
    }
    if (drive === sensor) {
      continue;
    }
    return [drive, sensor];
  }
  return null;
}

export function selectRepresentativeConfigs(allConfigs, numAnchors, maxSweeps, forbiddenSensors = []) {
  if (!Array.isArray(allConfigs) || allConfigs.length <= maxSweeps) {
    return allConfigs || [];
  }
  const axes = range(numAnchors);
  const forbidden = new Set(forbiddenSensors ?? []);
  const combos = combinations(axes, Math.max(1, numAnchors - 1));
  const chosen = [];

  for (const fixedAnchors of combos) {
    const freeAnchors = axes.filter((idx) => !fixedAnchors.includes(idx));
    for (const perm of permutations(freeAnchors, 2)) {
      const driveAnchor = perm[0];
      const sensorAnchor = perm[1];
      if (forbidden.has(sensorAnchor)) {
        continue;
      }
      const found = allConfigs.find((cfg) =>
        cfg.driveAnchor === driveAnchor
        && cfg.sensorAnchor === sensorAnchor
        && cfg.fixedAnchors.join(',') === fixedAnchors.join(','));
      if (found) {
        chosen.push(found);
        if (chosen.length >= maxSweeps) {
          return chosen;
        }
      }
    }
  }

  return chosen.length > 0 ? chosen : allConfigs.slice(0, maxSweeps);
}


async function prepareSweepPositioning(sendFn, sweepConfig, options) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    forceLow,
    forceMid,
    fixedTargets = [],
    feed = DEFAULT_FEED,
    speedup = 1,
  } = options;
  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  const measured = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  const moveParts = [];
  const formatDelta = (axis, delta) => `${axis}${delta.toFixed(3)}`;

  const fixedAnchors = sweepConfig.fixedAnchors || [];
  const movingAnchors = new Set();
  for (let i = 0; i < fixedAnchors.length; i += 1) {
    const anchorIdx = fixedAnchors[i];
    const target = Number.isFinite(fixedTargets[i]) ? fixedTargets[i] : 0;
    const delta = target - (measured[anchorIdx] ?? 0);
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatDelta(axes[anchorIdx], delta));
      movingAnchors.add(anchorIdx);
    }
  }

  if (moveParts.length > 0) {
    const preMoveModes = motorIds.map((_, idx) => (
      movingAnchors.has(idx) ? 'position' : forceMid
    ));
    await applyForceModeState(sendFn, { motorIds, modes: preMoveModes });
    await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup, { axes });
    const postMoveModes = motorIds.map((_, idx) => (
      movingAnchors.has(idx) ? 'position' : forceLow
    ));
    await applyForceModeState(sendFn, { motorIds, modes: postMoveModes });
    await waitForStableEncoders(sendFn, motorIds, { speedup });
  }

  return;
}

async function measureMaxTravelMm(sendFn, options = {}) {
  const {
    motorIds,
    mmPerDeg,
    forceLow,
    forceMax,
    pairAnchors,
    forbiddenForceAnchors = [],
    speedup = 1,
  } = options;

  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return null;
  }
  if (!Array.isArray(pairAnchors) || pairAnchors.length !== 2) {
    return null;
  }
  if (!Number.isFinite(forceLow) || !Number.isFinite(forceMax)) {
    return null;
  }

  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const pairSet = new Set(pairAnchors);

  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  const startLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);

  // Apply 75% of max force in three steps with 100 ms between
  let modesPullPair = motorIds.map((_, idx) => {
    if (forbidden.has(idx)) {
      return 'position';
    }
    if (pairSet.has(idx)) {
      return forceMax*0.25;
    }
    return forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: modesPullPair });
  baseSleep(100);
  modesPullPair = motorIds.map((_, idx) => {
    if (forbidden.has(idx)) {
      return 'position';
    }
    if (pairSet.has(idx)) {
      return forceMax*0.5;
    }
    return forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: modesPullPair });
  baseSleep(100);
  modesPullPair = motorIds.map((_, idx) => {
    if (forbidden.has(idx)) {
      return 'position';
    }
    if (pairSet.has(idx)) {
      return forceMax*0.75;
    }
    return forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: modesPullPair });
  await waitForStableEncoders(sendFn, motorIds, { speedup });
  const endLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);

  let maxTravel = 0;
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    if (pairSet.has(idx) || forbidden.has(idx)) {
      continue;
    }
    const start = startLengths[idx] ?? 0;
    const end = endLengths[idx] ?? 0;
    if (!Number.isFinite(start) || !Number.isFinite(end)) {
      continue;
    }
    const delta = Math.abs(end - start);
    if (!Number.isFinite(delta)) {
      continue;
    }
    maxTravel = Math.max(maxTravel, delta);
  }

  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });

  return Number.isFinite(maxTravel) ? maxTravel : null;
}

function parseSweepConfigLine(line, lineNumber) {
  const trimmed = line.trim();
  if (!trimmed || trimmed.startsWith('#')) {
    return null;
  }
  const parts = trimmed.split(/\s+/);
  if (parts.length < 3) {
    throw new Error(`Invalid sweep config line ${lineNumber}: expected fixedAnchors drive sensor`);
  }
  const fixedToken = parts[0].replace(/^\[/, '').replace(/\]$/, '');
  const fixedAnchors = fixedToken.length > 0
    ? fixedToken.split(',').map((val) => parseInt(val, 10))
    : [];
  const driveAnchor = parseInt(parts[1], 10);
  const sensorAnchor = parseInt(parts[2], 10);
  if (fixedAnchors.some((val) => !Number.isFinite(val)) || !Number.isFinite(driveAnchor) || !Number.isFinite(sensorAnchor)) {
    throw new Error(`Invalid sweep config line ${lineNumber}: anchors must be integers`);
  }
  return { fixedAnchors, driveAnchor, sensorAnchor };
}

export async function loadSweepConfigFile(filePath, machineConfig) {
  const text = await fs.readFile(filePath, 'utf8');
  const lines = text.split(/\r?\n/);
  const configs = [];
  for (let i = 0; i < lines.length; i += 1) {
    const line = lines[i];
    const parsed = parseSweepConfigLine(line, i + 1);
    if (parsed) {
      validateSweepConfig(parsed, machineConfig);
      configs.push(parsed);
    }
  }
  if (configs.length === 0) {
    throw new Error(`No sweep configurations found in ${filePath}`);
  }
  return configs;
}

export function validateSweepConfig(config, machineConfig) {
  const errors = [];
  if (!Array.isArray(config.fixedAnchors) || config.fixedAnchors.length > machineConfig.numAnchors - 2) {
    errors.push('fixed anchors count exceeds available anchors');
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

async function performForceSweep(sendFn, sweepConfig, options) {
  const {
    axes,
    motorIds,
    sweepPoints,
    feed,
    speedup,
    mmPerDeg,
    datasetStartMs,
    forceLow,
    forceMid,
    forceMax,
    fixedAnchors,
    forbiddenForceAnchors = [],
  } = options;
  const { driveAnchor, sensorAnchor } = sweepConfig;
  const driveAxis = axes[driveAnchor];
  if (!driveAxis) {
    throw new Error('force sweep requires drive axis mapping');
  }
  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const fixedSet = new Set(fixedAnchors ?? []);

  waitForStableEncoders(sendFn, motorIds, { speedup });
  const initialLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  const driveStartPointMm = initialLengths[driveAnchor] ?? 0;

  const modesPullout = motorIds.map((_, idx) => {
    if (idx === driveAnchor) {
      return forceMax;
    }
    if (fixedSet.has(idx) || forbidden.has(idx)) {
      return 'position';
    }
    return forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: modesPullout });
  await waitForStableEncoders(sendFn, motorIds, { speedup });

  const modesRelax = motorIds.map((_, idx) => {
    if (idx === driveAnchor) {
      return forceLow;
    }
    if (fixedSet.has(idx) || forbidden.has(idx)) {
      return 'position';
    }
    return forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: modesRelax });
  const forceStable = await waitForStableEncoders(sendFn, motorIds, { speedup });
  const endAngles = forceStable.anglesDeg;
  const endLengths = endAngles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));

  const driveEndPointMm = endLengths[driveAnchor] ?? driveStartPointMm;

  const dataPoints = [];
  const recordPoint = (angles, driveSetpointMm, stepIndex, stepCount) => {
    const lengths = angles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
    dataPoints.push({
      l_drive: lengths[driveAnchor],
      l_sensor: lengths[sensorAnchor],
      timestamp_ms: Date.now() - datasetStartMs,
      raw_angles_deg: angles,
      drive_setpoint_mm: driveSetpointMm,
      step_index: stepIndex,
      step_count: stepCount,
    });
    return lengths;
  };

  recordPoint(endAngles, driveEndPointMm, 0, sweepPoints);

  const returnModes = motorIds.map((_, idx) => {
    if (idx === driveAnchor) {
      return 'position';
    }
    if (fixedSet.has(idx) || forbidden.has(idx)) {
      return 'position';
    }
    return forceMid*2.0;
  });
  await applyForceModeState(sendFn, { motorIds, modes: returnModes });

  const steps = Math.max(2, sweepPoints);
  const stepCount = steps - 1;
  const stepDelta = (driveStartPointMm - driveEndPointMm) / Math.max(1, stepCount);
  let currentDrive = driveEndPointMm;
  for (let stepIdx = 1; stepIdx < steps; stepIdx += 1) {
    const target = driveEndPointMm + stepDelta * stepIdx;
    const delta = target - currentDrive;
    if (Math.abs(delta) > 1e-6) {
      // eslint-disable-next-line no-await-in-loop
      await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${delta.toFixed(3)} F${feed}`, speedup, { axes });
    }
    // eslint-disable-next-line no-await-in-loop
    const stable = await waitForStableEncoders(sendFn, motorIds, { speedup });
    const relaxForDataCollectionModes = motorIds.map((_, idx) => {
      if (idx === driveAnchor) {
        return 'position';
      }
      if (fixedSet.has(idx) || forbidden.has(idx)) {
        return 'position';
      }
      return forceLow;
    });
    await applyForceModeState(sendFn, { motorIds, modes: relaxForDataCollectionModes });
    const stableData = await waitForStableEncoders(sendFn, motorIds, { speedup });
    const lengths = recordPoint(stableData.anglesDeg, target, stepIdx, steps);
    currentDrive = lengths[driveAnchor] ?? currentDrive;
    await applyForceModeState(sendFn, { motorIds, modes: returnModes });
    await waitForStableEncoders(sendFn, motorIds, { speedup });
  }

  return { dataPoints, driveRange: { start: driveEndPointMm, end: driveStartPointMm } };
}

export async function collectSweepData(send, context) {
  const {
    args,
    machineType,
    machineConfig,
    motorIds,
    speedup: speedupArg,
  } = context;

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
    throw new Error('sweepPoints must be at least 2');
  }
  const maxSweepCount = Math.max(1, maxSweeps);
  const feed = Number.isFinite(parseFloat(args.feed)) ? parseFloat(args.feed) : DEFAULT_FEED;
  const forceLowProvided = Number.isFinite(parseFloat(args.forceLow));
  const forceMidProvided = Number.isFinite(parseFloat(args.forceMid));
  const forceMaxProvided = Number.isFinite(parseFloat(args.forceMax));
  let forceLow = forceLowProvided ? parseFloat(args.forceLow) : FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_LOW_N;
  let forceMid = forceMidProvided ? parseFloat(args.forceMid) : FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MID_N;
  let forceMax = forceMaxProvided ? parseFloat(args.forceMax) : FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MAX_N;
  const maxTravelSpec = args.maxTravelMm ?? null;
  const fixedTargetsSpec = args.fixedTargets ?? null;
  const maxTravelOverride = parseMaxTravelValue(maxTravelSpec);
  const settleMs = Number.isFinite(parseFloat(args.settleMs))
    ? Math.max(0, parseFloat(args.settleMs))
    : DEFAULT_SETTLE_MS;
  const speedup = Number.isFinite(speedupArg) && speedupArg > 0
    ? speedupArg
    : 1;
  const sampleRateHz = Number.isFinite(parseFloat(args.sampleRate))
    ? parseFloat(args.sampleRate)
    : DEFAULT_SAMPLE_RATE_HZ;
  const explicitTargetsSpec = fixedTargetsSpec ?? (maxTravelOverride === null ? maxTravelSpec : null);

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

  const sweeps = [];
  const datasetStartMs = Date.now();

  const m666BeforeReply = await send('M666');
  const m666Before = parseM666(m666BeforeReply?.reply);
  const m669Reply = await send('M669');
  const m669Values = parseM666(m669Reply?.reply);
  const m92Reply = await send('M92');
  const m92Values = parseM666(m92Reply?.reply);

  await send('M666 Q0');
  const m666AfterReply = await send('M666');
  const m666After = parseM666(m666AfterReply?.reply);

  const mmPerDeg = machineConfig.axes.map((_, idx) => computeMmPerDegree(m666After, idx));
  const missingAxes = mmPerDeg
    .map((val, idx) => (Number.isFinite(val) ? null : machineConfig.axes[idx]))
    .filter(Boolean);
  if (missingAxes.length > 0) {
    console.warn(`Warning: missing mm/deg calibration for axes [${missingAxes.join(', ')}]; lengths will default to 0 on those axes.`);
  }

  await primeEncoders(send, { motorIds, axes: machineConfig.axes });

  const forceArgsProvided = forceLowProvided && forceMidProvided && forceMaxProvided;
  const autoTuneForce = args.autoTuneForce || (!args.noAutoTuneForce && !forceArgsProvided);
  let forceTuningMeta = null;
  if (autoTuneForce) {
    const forbidden = new Set(machineConfig.forbiddenSensors ?? []);
    const autoTuneConfig = sweepConfigs.find((cfg) => !forbidden.has(cfg.driveAnchor)) ?? sweepConfigs[0];
    if (!autoTuneConfig) {
      console.log('; auto-tune force skipped (no sweep configs)');
    } else {
      console.log('; auto-tune force start');
      const tuned = await tuneForce(send, { config: autoTuneConfig }, {
        axes: machineConfig.axes,
        motorIds,
        mmPerDeg,
        feed,
        speedup,
        forbiddenForceAnchors: machineConfig.forbiddenSensors,
        forceLow,
        forceMid,
        forceMax,
        forceMaxProvided,
      });
      forceLow = tuned.forceLow;
      forceMid = tuned.forceMid;
      forceMax = tuned.forceMax;
      forceTuningMeta = tuned.tuningMeta ?? null;
    }
  }


  // A short tighten-release cycle before we set encoder reference points and set pos mode
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => forceMid),
  });
  await waitForStableEncoders(send, motorIds, { speedup });
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => forceMid*0.5),
  });
  await waitForStableEncoders(send, motorIds, { speedup });
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => forceLow),
  });
  await waitForStableEncoders(send, motorIds, { speedup });
  await primeEncoders(send, { motorIds, axes: machineConfig.axes });
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  await waitForStableEncoders(send, motorIds, { speedup });

  const explicitTargetsProvided = explicitTargetsSpec !== null || maxTravelOverride !== null;
  let maxTravelMm = Number.isFinite(maxTravelOverride) ? Math.abs(maxTravelOverride) : null;
  let sizeTunePair = null;
  if (!explicitTargetsProvided) {
    sizeTunePair = selectSizeTunePair(sweepConfigs, machineConfig.forbiddenSensors);
    if (!sizeTunePair) {
      console.log('; size-tune skipped (no force-capable anchor pair)');
    } else {
      maxTravelMm = await measureMaxTravelMm(send, {
        motorIds,
        mmPerDeg,
        forceLow,
        forceMax,
        pairAnchors: sizeTunePair,
        forbiddenForceAnchors: machineConfig.forbiddenSensors,
        speedup,
      });
      if (Number.isFinite(maxTravelMm)) {
        console.log(`; size-tune max travel=${maxTravelMm.toFixed(3)}mm`);
      } else {
        console.log('; size-tune failed; using 0.0mm');
      }
    }
  }
  if (!Number.isFinite(maxTravelMm)) {
    maxTravelMm = 0;
  }
  const maxTravelMeta = explicitTargetsSpec === null && maxTravelOverride === null
    ? maxTravelMm
    : (maxTravelOverride !== null ? maxTravelMm : undefined);

  const sweepTasks = [];
  for (const config of sweepConfigs) {
    const fixedCount = config.fixedAnchors.length;
    let combos = null;
    if (explicitTargetsSpec !== null) {
      combos = resolveFixedLengthCombos(
        fixedCount,
        superSweepRangeMm,
        superSweepPoints,
        explicitTargetsSpec,
      );
    } else if (Number.isFinite(maxTravelMm)) {
      combos = [Array.from({ length: fixedCount }, () => maxTravelMm)];
    } else {
      combos = resolveFixedLengthCombos(
        fixedCount,
        superSweepRangeMm,
        superSweepPoints,
        null,
      );
    }
    for (let comboIdx = 0; comboIdx < combos.length; comboIdx += 1) {
      sweepTasks.push({
        config,
        fixedTargets: combos[comboIdx] || [],
        fixedComboIndex: comboIdx,
        fixedComboCount: combos.length,
      });
    }
  }

  const plannedSweeps = sweepTasks.map((task) => ({ ...task }));

  const totalPlannedSweeps = plannedSweeps.length || sweepConfigs.length;
  let sweepOrdinal = 0;

  for (let planIdx = 0; planIdx < plannedSweeps.length; planIdx += 1) {
    const plan = plannedSweeps[planIdx];
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

    console.log(`\nSweep ${sweepOrdinal}/${totalPlannedSweeps}: fix [${sweepConfig.fixedAnchors.join(', ')}], drive ${sweepConfig.driveAnchor}, sensor ${sweepConfig.sensorAnchor}${fixedInfo}`);

    await prepareSweepPositioning(send, sweepConfig, {
      motorIds,
      axes: machineConfig.axes,
      mmPerDeg,
      forceLow,
      forceMid,
      fixedTargets,
      feed,
      speedup,
    });

    const sweepResult = await performForceSweep(send, sweepConfig, {
      axes: machineConfig.axes,
      motorIds,
      sweepPoints,
      feed,
      speedup,
      mmPerDeg,
      datasetStartMs,
      forceLow,
      forceMid,
      forceMax,
      fixedAnchors: sweepConfig.fixedAnchors,
      forbiddenForceAnchors: machineConfig.forbiddenSensors,
    });
    const dataPoints = sweepResult.dataPoints;

    const lengths = await getCurrentLengths(send, motorIds, mmPerDeg);
    const fixedLengths = sweepConfig.fixedAnchors.map((idx, anchorIdx) => {
      if (Number.isFinite(lengths[idx])) {
        return lengths[idx];
      }
      return Number.isFinite(fixedTargets[anchorIdx]) ? fixedTargets[anchorIdx] : 0;
    });

    sweeps.push({
      id: `sweep_${String(sweepOrdinal).padStart(3, '0')}`,
      fixed_anchors: sweepConfig.fixedAnchors,
      fixed_lengths: fixedLengths,
      drive_anchor: sweepConfig.driveAnchor,
      sensor_anchor: sweepConfig.sensorAnchor,
      drive_range: sweepResult.driveRange ? { ...sweepResult.driveRange, unit: 'mm' } : null,
      data_points: dataPoints,
      metadata: {
        sweep_method: 'position',
        feed_rate: feed,
        sensor_force_n: forceMid,
        settle_ms: settleMs,
        fixed_combo_index: fixedComboIndex,
        fixed_combo_count: fixedComboCount,
        sample_rate_hz: undefined,
      },
    });
    console.log(`  Collected ${dataPoints.length} points`);
    if (fixedComboCount > 1 && fixedComboIndex === fixedComboCount - 1) {
      await returnMotorsToOriginOneAtATime(send, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        feed,
        speedup,
        midForce: forceMid,
        fixedAnchors: sweepConfig.fixedAnchors,
        forbiddenForceAnchors: machineConfig.forbiddenSensors,
      });
    }
  }

  const forceTuning = {
    auto_tuned: autoTuneForce,
    method: autoTuneForce ? 'force-thresholds' : 'manual',
    tuned_at: autoTuneForce ? new Date().toISOString() : undefined,
    force_low_n: forceLow,
    force_mid_n: forceMid,
    force_max_n: forceMax,
    ...(forceTuningMeta ?? {}),
  };

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
      force_tuning: forceTuning,
      max_travel_mm: Number.isFinite(maxTravelMeta) ? maxTravelMeta : undefined,
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

  if (args.returnToOrigin) {
    try {
      await returnMotorsToOriginAllAtOnce(send, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        feed,
        speedup,
      });
      console.log('Returned all motors to encoder origin.');
    } catch (err) {
      console.warn(`Warning: failed to return to encoder origin: ${err?.message || err}`);
    }
  }

  return { sweeps, outputFile };
}
