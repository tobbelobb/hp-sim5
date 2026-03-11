import fs from 'node:fs/promises';
import path from 'node:path';
import {
  computeMmPerDegree,
  parseM666,
  runMoveWithWait,
  sleep as baseSleep,
} from '../primitives/encoder_utils.mjs';
import { ENCODER_NOISE_DEFAULTS } from '../primitives/encoder_noise.mjs';
import {
  angleToLength,
  applyDataPointReturnModes,
  applyForceModeState,
  collectDataPoint,
  getCurrentLengths,
  primeEncoders,
  returnMotorsToOriginAllAtOnce,
  waitForStableEncoders,
} from '../primitives/uncalibrated_actions.mjs';
import { attachDebugState } from '../primitives/debug_trace.mjs';
import { FORCE_TUNING_DEFAULTS, tuneForce } from './force_tuning.mjs';

export const MACHINE_CONFIGS = {
  slideprinter: { numAnchors: 3, dimensions: 2, axes: ['X', 'Y', 'Z'], mustBeInFixedSet: [] },
  // These are the visible G-code axes exposed by the active RRF configs, not the
  // physical anchor labels used in M669/M666 replies.
  hangprinter_4: {
    numAnchors: 4,
    dimensions: 3,
    axes: ['X', 'Y', 'Z', 'U'],
    mustBeInFixedSet: [3],
    fixedTargetBoundsByAnchor: {
      0: { minFixed: 0 },
      1: { minFixed: 0 },
      2: { minFixed: 0 },
      3: { maxFixed: 0 },
    },
    measuredMaxTravelScale: 0.8,
  },
  hangprinter_5: { numAnchors: 5, dimensions: 3, axes: ['X', 'Y', 'Z', 'U', 'V'], mustBeInFixedSet: [4] },
  cubecorners: { numAnchors: 8, dimensions: 3, axes: ['X', 'Y', 'Z', 'U', 'V', 'W', 'A', 'B'], mustBeInFixedSet: [] },
  skycam: { numAnchors: 4, dimensions: 3, axes: ['X', 'Y', 'Z', 'U'], mustBeInFixedSet: [] },
};

export const MOTOR_IDS_BY_MACHINE = {
  slideprinter: ['40.0', '41.0', '42.0'],
  hangprinter_4: ['40.0', '41.0', '42.0', '43.0'],
  hangprinter_5: ['40.0', '41.0', '42.0', '43.0', '44.0'],
  cubecorners: ['40.0', '41.0', '42.0', '43.0', '44.0', '45.0', '46.0', '47.0'],
  skycam: ['40.0', '41.0', '42.0', '43.0'],
};

const DEFAULT_SWEEP_POINTS = 10;
const DEFAULT_MAX_SWEEPS = 6;
const DEFAULT_FEED = 1400;
const DEFAULT_FORCE_MID_N = 0.05;
const DEFAULT_SETTLE_MS = 200;
const DEFAULT_SAMPLE_RATE_HZ = 40;
const DEFAULT_NOISE_SAMPLE_COUNT = ENCODER_NOISE_DEFAULTS.DEFAULT_NOISE_SAMPLE_COUNT;
const DEFAULT_NOISE_SAMPLE_RATE_HZ = ENCODER_NOISE_DEFAULTS.DEFAULT_NOISE_SAMPLE_RATE_HZ;
const DEFAULT_NOISE_MIN_SAMPLES = ENCODER_NOISE_DEFAULTS.DEFAULT_NOISE_MIN_SAMPLES;
const DEFAULT_NOISE_SIGMA_FLOOR_DEG = ENCODER_NOISE_DEFAULTS.DEFAULT_NOISE_SIGMA_FLOOR_DEG;
const DEFAULT_FIXED_TOLERANCE_MM = 0.01;
const DATASET_VERSION = '2.0';

export const SWEEP_DEFAULTS = {
  DEFAULT_SWEEP_POINTS,
  DEFAULT_MAX_SWEEPS,
  DEFAULT_FEED,
  DEFAULT_FORCE_MID_N,
  DEFAULT_SETTLE_MS,
  DEFAULT_SAMPLE_RATE_HZ,
  DEFAULT_NOISE_SAMPLE_COUNT,
  DEFAULT_NOISE_SAMPLE_RATE_HZ,
  DEFAULT_NOISE_MIN_SAMPLES,
  DEFAULT_NOISE_SIGMA_FLOOR_DEG,
  DEFAULT_FIXED_TOLERANCE_MM,
  DATASET_VERSION,
};

/**
 * Sweep collection field reference (input + normalized options).
 * machineType: lowercase key in MACHINE_CONFIGS.
 * machineConfig: { numAnchors, dimensions, axes, mustBeInFixedSet?, fixedTargetBoundsByAnchor? } for machineType.
 * motorIds: array of motor ID strings, length == machineConfig.numAnchors.
 * speedup: positive number, hp-sim speed scale (1 == realtime).
 * sweepPoints: integer >= 2, number of points per sweep.
 * maxSweepCount: integer >= 1, max sweeps selected from generated configs.
 * feed: number, mm/min feed rate for drive moves.
 * settleMs: number >= 0, metadata settle time.
 * forceLow/Mid/Max: numbers, force thresholds in N.
 * forceLowProvided/MidProvided/MaxProvided: booleans for user-provided force args.
 * forceArgsProvided: boolean, true when all force args supplied by the user.
 * fixedTargets: array of numbers or null, per-fixed-anchor deltas in mm.
 * maxTravelMm: number or null, shared fixed-anchor delta in mm.
 * sweepConfigFile: string or null, path to explicit sweep configs file.
 * debugSweep: boolean, print planned sweep configs.
 * autoTuneForce/noAutoTuneForce: booleans, auto-tuning behavior flags.
 * returnToOrigin: boolean, return all motors to origin between sweeps and after collection.
 * outputFile: string or null, output dataset JSON path.
 * projectZeroTension: boolean, project encoder readings to zero tension during collection.
 */

function range(n) {
  return Array.from({ length: n }, (_, i) => i);
}

function parseOptionalNumber(value, label, { integer = false } = {}) {
  if (value == null || value === '') {
    return null;
  }
  const parsed = integer ? parseInt(value, 10) : parseFloat(value);
  if (!Number.isFinite(parsed)) {
    throw new Error(`${label} must be a number.`);
  }
  return parsed;
}

function summarizeNoiseWarnings(noiseStats, { minSamples, sigmaFloorDeg } = {}) {
  if (!noiseStats || typeof noiseStats !== 'object') {
    return [];
  }
  const warnings = [];
  const sampleCount = noiseStats.sampleCount;
  if (Number.isFinite(minSamples) && Number.isFinite(sampleCount) && sampleCount < minSamples) {
    warnings.push('samples_low');
  }
  const sigmas = noiseStats.sigmaByMotorDeg;
  if (Array.isArray(sigmas)) {
    const anyNonFinite = sigmas.some((val) => !Number.isFinite(val));
    if (anyNonFinite) {
      warnings.push('sigma_nonfinite');
    }
    if (Number.isFinite(sigmaFloorDeg)) {
      const belowFloor = sigmas.some((val) => Number.isFinite(val) && val < sigmaFloorDeg);
      if (belowFloor) {
        warnings.push('sigma_below_floor');
      }
    }
  }
  return warnings;
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

function parseFixedTargetsSpec(spec) {
  if (spec == null) {
    return null;
  }
  const raw = spec.toString().trim();
  if (raw.length === 0) {
    return null;
  }
  if (raw.includes(';')) {
    throw new Error('Multiple fixed-target groups are no longer supported (super sweep removed).');
  }
  const values = raw
    .split(',')
    .map((entry) => entry.trim())
    .filter(Boolean)
    .map((entry) => parseFloat(entry));
  if (values.length === 0) {
    return null;
  }
  if (values.some((val) => !Number.isFinite(val))) {
    throw new Error('Fixed targets must be numeric values.');
  }
  return values;
}

function normalizeFixedTargets(values, fixedCount) {
  if (!Array.isArray(values) || fixedCount <= 0) {
    return [];
  }
  const normalized = values.slice(0, fixedCount).map((val) => (Number.isFinite(val) ? val : 0));
  while (normalized.length < fixedCount) {
    normalized.push(0);
  }
  return normalized;
}

function constraintsFor1Dof(machineConfig) {
  const dims = Number.isFinite(machineConfig?.dimensions) ? machineConfig.dimensions : 3;
  const fixed = Math.max(1, Math.floor(dims - 1));
  return Math.min(fixed, Math.max(0, machineConfig?.numAnchors ?? fixed));
}

function getMustBeInFixedSet(machineConfig) {
  if (!Array.isArray(machineConfig?.mustBeInFixedSet)) {
    return [];
  }
  return machineConfig.mustBeInFixedSet
    .map((entry) => Number.parseInt(entry, 10))
    .filter((entry) => Number.isFinite(entry));
}

function getForceForbiddenAnchors(machineConfig) {
  return getMustBeInFixedSet(machineConfig);
}

function getFixedTargetBounds(machineConfig, anchorIdx) {
  if (!machineConfig || typeof machineConfig !== 'object') {
    return null;
  }
  const raw = machineConfig.fixedTargetBoundsByAnchor;
  if (!raw || typeof raw !== 'object') {
    return null;
  }
  const bounds = raw[anchorIdx] ?? raw[String(anchorIdx)];
  if (!bounds || typeof bounds !== 'object') {
    return null;
  }
  const minRaw = bounds.minFixed;
  const maxRaw = bounds.maxFixed;
  const minFixed = Number.isFinite(minRaw) ? Number(minRaw) : null;
  const maxFixed = Number.isFinite(maxRaw) ? Number(maxRaw) : null;
  return { minFixed, maxFixed };
}

function constrainFixedTarget(machineConfig, anchorIdx, value) {
  let target = Number.isFinite(value) ? Number(value) : 0;
  const bounds = getFixedTargetBounds(machineConfig, anchorIdx);
  if (!bounds) {
    return target;
  }
  if (Number.isFinite(bounds.minFixed) && target < bounds.minFixed) {
    target = bounds.minFixed;
  }
  if (Number.isFinite(bounds.maxFixed) && target > bounds.maxFixed) {
    target = bounds.maxFixed;
  }
  return target;
}

function canonicalDriveSensorPair(anchorA, anchorB) {
  return anchorA <= anchorB ? [anchorA, anchorB] : [anchorB, anchorA];
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
    throw new Error('Use --fixed-targets for per-anchor values; --max-travel-mm expects one number.');
  }
  const value = parseFloat(raw);
  if (!Number.isFinite(value)) {
    throw new Error('max-travel-mm must be a numeric value.');
  }
  return value;
}

export function resolveForcedBuildupFactor({ forceBuildupFactor = null, preserveBuildupFactor = false } = {}) {
  if (Number.isFinite(forceBuildupFactor)) {
    return Number(forceBuildupFactor);
  }
  if (preserveBuildupFactor) {
    return null;
  }
  return null;
}

function parseOptionalNumberList(value, label) {
  if (value == null || value === '') {
    return null;
  }
  const rawItems = Array.isArray(value)
    ? value
    : String(value)
      .split(/[,:]/)
      .map((entry) => entry.trim())
      .filter(Boolean);
  if (rawItems.length === 0) {
    return null;
  }
  const out = rawItems.map((entry) => Number.parseFloat(entry));
  if (out.some((entry) => !Number.isFinite(entry))) {
    throw new Error(`${label} must be numeric.`);
  }
  return out;
}

function normalizeBaseRadii(values, numAnchors) {
  if (!Array.isArray(values) || values.length === 0 || !Number.isFinite(numAnchors) || numAnchors <= 0) {
    return null;
  }
  const normalized = values
    .slice(0, numAnchors)
    .map((entry) => Number(entry))
    .filter((entry) => Number.isFinite(entry));
  if (normalized.length === 0) {
    return null;
  }
  while (normalized.length < numAnchors) {
    normalized.push(normalized[0]);
  }
  return normalized;
}

function formatM666Number(value) {
  return Number(value.toFixed(9)).toString();
}

function formatM666Vector(values) {
  if (!Array.isArray(values) || values.length === 0) {
    return null;
  }
  const parts = values.map((entry) => Number(entry)).filter((entry) => Number.isFinite(entry));
  if (parts.length === 0) {
    return null;
  }
  return parts.map((entry) => formatM666Number(entry)).join(':');
}

export function resolveForcedBaseRadii({ forceBaseRadii = null, numAnchors = 0 } = {}) {
  return normalizeBaseRadii(forceBaseRadii, numAnchors);
}

export function buildM666AdjustmentCommand({ forcedBaseRadii = null, forcedBuildupFactor = null } = {}) {
  const fields = [];
  const radiiSpec = formatM666Vector(forcedBaseRadii);
  if (radiiSpec) {
    fields.push(`R${radiiSpec}`);
  }
  if (Number.isFinite(forcedBuildupFactor)) {
    fields.push(`Q${formatM666Number(Number(forcedBuildupFactor))}`);
  }
  if (fields.length === 0) {
    return null;
  }
  return `M666 ${fields.join(' ')}`;
}

// In case spool was fully wound out, wind back in a quarter of a rotation so that line
// attachment point is tangent to the spool.
export function computeInitialForceSweepStepMm(baseRadiusMm) {
  if (!Number.isFinite(baseRadiusMm) || baseRadiusMm <= 0) {
    return 100;
  }
  return baseRadiusMm * Math.PI / 2;
}

export function resolveFixedTargets(fixedAnchors, explicitTargets, maxTravelMm, machineConfig = null) {
  const fixedCount = Array.isArray(fixedAnchors) ? fixedAnchors.length : 0;
  if (fixedCount <= 0) {
    return [];
  }
  let targets = [];
  if (Array.isArray(explicitTargets)) {
    targets = normalizeFixedTargets(explicitTargets, fixedCount);
  } else {
    const value = Number.isFinite(maxTravelMm) ? maxTravelMm : 0;
    targets = Array.from({ length: fixedCount }, () => value);
  }
  return targets.map((target, idx) => constrainFixedTarget(machineConfig, fixedAnchors[idx], target));
}

export function scaleMeasuredMaxTravelMm(machineConfig, measuredMaxTravelMm) {
  if (!Number.isFinite(measuredMaxTravelMm)) {
    return null;
  }
  let scale = Number(machineConfig?.measuredMaxTravelScale);
  if (!Number.isFinite(scale) || scale <= 0) {
    scale = 1;
  } else if (scale > 1) {
    scale = 1;
  }
  return Math.abs(Number(measuredMaxTravelMm)) * scale;
}

function validateSweepCollectionInput(context) {
  if (!context || typeof context !== 'object') {
    throw new Error('collectSweepData requires a context object.');
  }
  const { args = {}, machineType, machineConfig, motorIds, speedup } = context;
  const config = machineConfig ?? MACHINE_CONFIGS[machineType];
  if (!config || !machineType) {
    throw new Error(`Unknown machine type: ${machineType}`);
  }
  if (!Array.isArray(motorIds) || motorIds.length !== config.numAnchors) {
    throw new Error(`Motor ID mapping missing or mismatched for ${machineType}`);
  }
  if (!Array.isArray(config.axes) || motorIds.length !== config.axes.length) {
    throw new Error(`Motor ID count (${motorIds.length}) does not match axes count (${config.axes?.length ?? 0}) for ${machineType}`);
  }

  const sweepPoints = parseOptionalNumber(args.sweepPoints, 'sweepPoints', { integer: true });
  if (Number.isFinite(sweepPoints) && sweepPoints < 2) {
    throw new Error('sweepPoints must be at least 2');
  }
  const maxSweeps = parseOptionalNumber(args.maxSweeps, 'maxSweeps', { integer: true });
  const feed = parseOptionalNumber(args.feed, 'feed');
  const settleMs = parseOptionalNumber(args.settleMs, 'settleMs');
  const forceLow = parseOptionalNumber(args.forceLow, 'force-low');
  const forceMid = parseOptionalNumber(args.forceMid, 'force-mid');
  const forceMax = parseOptionalNumber(args.forceMax, 'force-max');
  const forceBuildupFactor = parseOptionalNumber(args.forceBuildupFactor, 'force-buildup-factor');
  const preserveBuildupFactor = !!args.preserveBuildupFactor;
  const forceBaseRadii = parseOptionalNumberList(
    args.forceBaseRadii ?? args.baseRadii,
    'force-base-radii',
  );
  const noiseSampleCount = parseOptionalNumber(args.noiseSamples ?? args.noiseSampleCount, 'noise-samples', { integer: true });
  const noiseSampleRateHz = parseOptionalNumber(
    args.noiseSampleRateHz ?? args.noiseSampleRate ?? args.noiseSampleHz,
    'noise-sample-rate',
  );
  const noiseSampleIntervalMs = parseOptionalNumber(
    args.noiseSampleIntervalMs ?? args.noiseSampleInterval,
    'noise-sample-interval-ms',
  );
  const noiseMinSamples = parseOptionalNumber(args.noiseMinSamples, 'noise-min-samples', { integer: true });
  const noiseSigmaFloorDeg = parseOptionalNumber(args.noiseSigmaFloorDeg ?? args.noiseSigmaFloor, 'noise-sigma-floor');
  const forceLowProvided = forceLow !== null;
  const forceMidProvided = forceMid !== null;
  const forceMaxProvided = forceMax !== null;
  const fixedTargets = parseFixedTargetsSpec(args.fixedTargets ?? null);
  const maxTravelMm = parseMaxTravelValue(args.maxTravelMm ?? null);
  if (fixedTargets && maxTravelMm !== null) {
    throw new Error('Use either --fixed-targets or --max-travel-mm, not both.');
  }
  if (preserveBuildupFactor && forceBuildupFactor !== null) {
    throw new Error('Use either --preserve-buildup-factor or --force-buildup-factor, not both.');
  }

  return {
    machineType,
    machineConfig: config,
    motorIds,
    speedup: parseOptionalNumber(speedup, 'speedup'),
    sweepPoints,
    maxSweeps,
    feed,
    settleMs,
    forceLow,
    forceMid,
    forceMax,
    forceBuildupFactor,
    preserveBuildupFactor,
    forceBaseRadii,
    noiseSampleCount,
    noiseSampleRateHz,
    noiseSampleIntervalMs,
    noiseMinSamples,
    noiseSigmaFloorDeg,
    forceLowProvided,
    forceMidProvided,
    forceMaxProvided,
    forceArgsProvided: forceLowProvided && forceMidProvided && forceMaxProvided,
    fixedTargets,
    maxTravelMm,
    sweepConfigFile: args.sweepConfigFile ?? null,
    debugSweep: !!args.debugSweep,
    autoTuneForce: !!args.autoTuneForce,
    noAutoTuneForce: !!args.noAutoTuneForce,
    returnToOrigin: !!args.returnToOrigin,
    projectZeroTension: !!args.projectZeroTension,
    outputFile: args.outputFile ?? null,
  };
}

function applySweepDefaults(input) {
  const sweepPoints = input.sweepPoints ?? DEFAULT_SWEEP_POINTS;
  const maxSweepCount = Math.max(1, input.maxSweeps ?? DEFAULT_MAX_SWEEPS);
  const feed = input.feed ?? DEFAULT_FEED;
  const settleMs = Math.max(0, input.settleMs ?? DEFAULT_SETTLE_MS);
  const speedup = Number.isFinite(input.speedup) && input.speedup > 0 ? input.speedup : 1;
  const forceLow = input.forceLow ?? FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_LOW_N;
  const forceMid = input.forceMid ?? FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MID_N;
  const forceMax = input.forceMax ?? FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MAX_N;
  const noiseSampleCount = Math.max(1, input.noiseSampleCount ?? DEFAULT_NOISE_SAMPLE_COUNT);
  let noiseSampleRateHz = Number.isFinite(input.noiseSampleRateHz)
    ? input.noiseSampleRateHz
    : DEFAULT_NOISE_SAMPLE_RATE_HZ;
  let noiseSampleIntervalMs = Number.isFinite(input.noiseSampleIntervalMs)
    ? input.noiseSampleIntervalMs
    : (1000 / Math.max(1e-6, noiseSampleRateHz));
  if (!Number.isFinite(noiseSampleRateHz) || noiseSampleRateHz <= 0) {
    noiseSampleRateHz = DEFAULT_NOISE_SAMPLE_RATE_HZ;
  }
  if (!Number.isFinite(noiseSampleIntervalMs) || noiseSampleIntervalMs <= 0) {
    noiseSampleIntervalMs = 1000 / Math.max(1e-6, noiseSampleRateHz);
  } else {
    noiseSampleRateHz = 1000 / noiseSampleIntervalMs;
  }
  const noiseMinSamples = Math.max(1, input.noiseMinSamples ?? DEFAULT_NOISE_MIN_SAMPLES);
  const noiseSigmaFloorDeg = Math.max(0, input.noiseSigmaFloorDeg ?? DEFAULT_NOISE_SIGMA_FLOOR_DEG);
  const maxTravelMm = Number.isFinite(input.maxTravelMm) ? Math.abs(input.maxTravelMm) : null;
  const autoTuneForce = input.autoTuneForce || (!input.noAutoTuneForce && !input.forceArgsProvided);

  return {
    ...input,
    sweepPoints,
    maxSweepCount,
    feed,
    settleMs,
    speedup,
    forceLow,
    forceMid,
    forceMax,
    forceBuildupFactor: Number.isFinite(input.forceBuildupFactor) ? input.forceBuildupFactor : null,
    preserveBuildupFactor: !!input.preserveBuildupFactor,
    noiseSampleCount,
    noiseSampleRateHz,
    noiseSampleIntervalMs,
    noiseMinSamples,
    noiseSigmaFloorDeg,
    maxTravelMm,
    autoTuneForce,
  };
}

async function resolveSweepConfigs({ sweepConfigFile, machineType, machineConfig, maxSweepCount }) {
  if (sweepConfigFile) {
    return loadSweepConfigFile(sweepConfigFile, machineConfig);
  }
  let sweepConfigs = generateSweepConfigs(machineType);
  if (sweepConfigs.length > maxSweepCount) {
    sweepConfigs = selectRepresentativeConfigs(sweepConfigs, machineConfig, maxSweepCount);
  }
  return sweepConfigs;
}

export function generateSweepConfigs(machineType) {
  const machineConfig = MACHINE_CONFIGS[machineType];
  if (!machineConfig) {
    return [];
  }
  const numAnchors = machineConfig.numAnchors;
  const allAnchors = range(numAnchors);
  const fixedCount = constraintsFor1Dof(machineConfig);
  const mustBeFixed = new Set(getMustBeInFixedSet(machineConfig));
  const configs = [];

  for (const fixedAnchors of combinations(allAnchors, fixedCount)) {
    if ([...mustBeFixed].some((anchorIdx) => !fixedAnchors.includes(anchorIdx))) {
      continue;
    }
    const freeAnchors = allAnchors.filter((idx) => !fixedAnchors.includes(idx));
    for (const pair of combinations(freeAnchors, 2)) {
      const [driveAnchor, sensorAnchor] = canonicalDriveSensorPair(pair[0], pair[1]);
      configs.push({ fixedAnchors, driveAnchor, sensorAnchor });
    }
  }

  return configs;
}

function selectSizeTunePair(sweepConfigs, fixedOnlyAnchors = []) {
  if (!Array.isArray(sweepConfigs) || sweepConfigs.length === 0) {
    return null;
  }
  const fixedOnly = new Set(fixedOnlyAnchors ?? []);
  for (const cfg of sweepConfigs) {
    if (fixedOnly.has(cfg.driveAnchor) || fixedOnly.has(cfg.sensorAnchor)) {
      continue;
    }
    if (cfg.driveAnchor === cfg.sensorAnchor) {
      continue;
    }
    return [cfg.driveAnchor, cfg.sensorAnchor];
  }
  return null;
}

export function selectRepresentativeConfigs(allConfigs, machineConfig, maxSweeps) {
  if (!Array.isArray(allConfigs) || allConfigs.length <= maxSweeps) {
    return allConfigs || [];
  }
  const cfg = machineConfig || {};
  const numAnchors = cfg.numAnchors ?? 0;
  const axes = range(numAnchors);
  const fixedCount = constraintsFor1Dof(cfg);
  const mustBeFixed = new Set(getMustBeInFixedSet(cfg));
  const combos = combinations(axes, fixedCount);
  const chosen = [];

  for (const fixedAnchors of combos) {
    if ([...mustBeFixed].some((anchorIdx) => !fixedAnchors.includes(anchorIdx))) {
      // eslint-disable-next-line no-continue
      continue;
    }
    const freeAnchors = axes.filter((idx) => !fixedAnchors.includes(idx));
    for (const pair of combinations(freeAnchors, 2)) {
      const [driveAnchor, sensorAnchor] = canonicalDriveSensorPair(pair[0], pair[1]);
      const found = allConfigs.find(
        (cfgEntry) =>
          cfgEntry.driveAnchor === driveAnchor
          && cfgEntry.sensorAnchor === sensorAnchor
          && cfgEntry.fixedAnchors.join(',') === fixedAnchors.join(','),
      );
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

export function expandSubSweepsForConfig(sweepConfig, machineConfig) {
  if (!sweepConfig) {
    return [];
  }
  const normalized = normalizeSweepConfig(sweepConfig, machineConfig);
  const mustBeFixed = new Set(getMustBeInFixedSet(machineConfig));
  const fixedSet = new Set(normalized.fixedAnchors || []);
  const candidates = [
    { driveAnchor: normalized.driveAnchor, sensorAnchor: normalized.sensorAnchor },
    { driveAnchor: normalized.sensorAnchor, sensorAnchor: normalized.driveAnchor },
  ];
  const out = [];
  for (const cand of candidates) {
    const { driveAnchor, sensorAnchor } = cand;
    if (!Number.isFinite(driveAnchor) || !Number.isFinite(sensorAnchor)) {
      continue;
    }
    if (driveAnchor === sensorAnchor) {
      continue;
    }
    if (fixedSet.has(driveAnchor) || fixedSet.has(sensorAnchor)) {
      continue;
    }
    if (mustBeFixed.has(driveAnchor) || mustBeFixed.has(sensorAnchor)) {
      continue;
    }
    if (out.some((cfg) => cfg.driveAnchor === driveAnchor && cfg.sensorAnchor === sensorAnchor)) {
      continue;
    }
    out.push({ ...normalized, driveAnchor, sensorAnchor });
  }
  return out;
}

function remapDataPointsToCanonical(dataPoints, canonicalConfig, actualConfig) {
  if (!Array.isArray(dataPoints) || dataPoints.length === 0) {
    return [];
  }
  const sameOrientation =
    canonicalConfig.driveAnchor === actualConfig.driveAnchor
    && canonicalConfig.sensorAnchor === actualConfig.sensorAnchor;
  return dataPoints.map((point) => {
    const p = { ...point };
    if (!sameOrientation) {
      p.l_drive = point.l_sensor;
      p.l_sensor = point.l_drive;
      if (
        Object.prototype.hasOwnProperty.call(point, 'l_drive_mu')
        || Object.prototype.hasOwnProperty.call(point, 'l_sensor_mu')
      ) {
        p.l_drive_mu = point.l_sensor_mu;
        p.l_sensor_mu = point.l_drive_mu;
      }
      if (
        Object.prototype.hasOwnProperty.call(point, 'assumed_tension_drive_n')
        || Object.prototype.hasOwnProperty.call(point, 'assumed_tension_sensor_n')
      ) {
        p.assumed_tension_drive_n = point.assumed_tension_sensor_n;
        p.assumed_tension_sensor_n = point.assumed_tension_drive_n;
      }
      // Keep `source_*` as physical sub-sweep metadata even after remapping
      // l_* into canonical sweep orientation. This means downstream code can
      // observe reversed source_* with already-canonical l_drive/l_sensor.
      p.source_drive_anchor = actualConfig.driveAnchor;
      p.source_sensor_anchor = actualConfig.sensorAnchor;
    }
    return p;
  });
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
    speedup,
  } = options;
  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  const measured = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  const moveParts = [];
  const formatDelta = (axis, delta) => `${axis}${delta.toFixed(3)}`;

  const fixedAnchors = sweepConfig.fixedAnchors || [];
  const fixedSet = new Set(fixedAnchors);
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
      (movingAnchors.has(idx) || fixedSet.has(idx)) ? 'position' : forceMid
    ));
    await applyForceModeState(sendFn, { motorIds, modes: preMoveModes });
    await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup, { axes });
  }

  return;
}

function buildFixedTargetByAnchor(fixedAnchors, fixedTargets, motorCount) {
  if (!Array.isArray(fixedAnchors) || fixedAnchors.length === 0) {
    return null;
  }
  const targets = new Array(motorCount).fill(null);
  for (let i = 0; i < fixedAnchors.length; i += 1) {
    const anchorIdx = fixedAnchors[i];
    if (!Number.isFinite(anchorIdx)) {
      continue;
    }
    const target = Array.isArray(fixedTargets) && Number.isFinite(fixedTargets[i])
      ? fixedTargets[i]
      : 0;
    targets[anchorIdx] = target;
  }
  return targets;
}

async function enforceFixedAnchors(sendFn, options = {}) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    fixedTargetByAnchor,
    feed = DEFAULT_FEED,
    speedup,
    toleranceMm = DEFAULT_FIXED_TOLERANCE_MM,
  } = options;
  if (!Array.isArray(fixedTargetByAnchor) || fixedTargetByAnchor.length === 0) {
    return null;
  }
  const lengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  const moveParts = [];
  for (let idx = 0; idx < fixedTargetByAnchor.length; idx += 1) {
    const target = fixedTargetByAnchor[idx];
    if (!Number.isFinite(target)) {
      continue;
    }
    const mmPer = Array.isArray(mmPerDeg) ? mmPerDeg[idx] : null;
    if (!Number.isFinite(mmPer)) {
      continue;
    }
    const current = lengths[idx];
    if (!Number.isFinite(current)) {
      continue;
    }
    const delta = target - current;
    if (Math.abs(delta) > toleranceMm) {
      const axis = axes[idx];
      if (axis) {
        moveParts.push(`${axis}${delta.toFixed(3)}`);
      }
    }
  }
  if (moveParts.length === 0) {
    return lengths;
  }
  await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup, { axes });
  await waitForStableEncoders(sendFn, motorIds, speedup);
  return getCurrentLengths(sendFn, motorIds, mmPerDeg);
}

async function measureMaxTravelMm(sendFn, options = {}) {
  const {
    motorIds,
    mmPerDeg,
    forceLow,
    forceMax,
    pairAnchors,
    forbiddenForceAnchors = [],
    speedup,
  } = options;

  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const pairSet = new Set(pairAnchors);

  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  const startLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);

  // Apply 75% of max force in three steps with 100 ms between
  const pullFractions = [0.25, 0.5, 0.75];
  for (let i = 0; i < pullFractions.length; i += 1) {
    const fraction = pullFractions[i];
    const modesPullPair = motorIds.map((_, idx) => {
      if (forbidden.has(idx)) {
        return 'position';
      }
      if (pairSet.has(idx)) {
        return forceMax * fraction;
      }
      return forceLow;
    });
    await applyForceModeState(sendFn, { motorIds, modes: modesPullPair });
    if (i < pullFractions.length - 1) {
      baseSleep(100);
    }
  }
  await waitForStableEncoders(sendFn, motorIds, speedup);
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
  const seen = new Set();
  for (let i = 0; i < lines.length; i += 1) {
    const line = lines[i];
    const parsed = parseSweepConfigLine(line, i + 1);
    if (parsed) {
      const normalized = normalizeSweepConfig(parsed, machineConfig);
      const key = `${normalized.fixedAnchors.join(',')}|${normalized.driveAnchor}|${normalized.sensorAnchor}`;
      if (seen.has(key)) {
        // Skip duplicates after canonicalization to avoid repeated sweeps.
        // eslint-disable-next-line no-continue
        continue;
      }
      validateSweepConfig(normalized, machineConfig);
      configs.push(normalized);
      seen.add(key);
    }
  }
  if (configs.length === 0) {
    throw new Error(`No sweep configurations found in ${filePath}`);
  }
  return configs;
}

export function validateSweepConfig(config, machineConfig) {
  const errors = [];
  const fixedCount = constraintsFor1Dof(machineConfig);
  const mustBeFixed = getMustBeInFixedSet(machineConfig);
  if (!Array.isArray(config.fixedAnchors) || config.fixedAnchors.length !== fixedCount) {
    errors.push(`expected ${fixedCount} fixed anchors`);
  }
  const all = [...config.fixedAnchors, config.driveAnchor, config.sensorAnchor];
  if (all.some((idx) => idx < 0 || idx >= machineConfig.numAnchors)) {
    errors.push('anchor index out of range');
  }
  if (new Set(all).size !== config.fixedAnchors.length + 2) {
    errors.push('duplicate anchor roles in sweep config');
  }
  if (mustBeFixed.some((anchorIdx) => !config.fixedAnchors.includes(anchorIdx))) {
    errors.push(`anchors [${mustBeFixed.join(', ')}] must remain fixed`);
  }
  if (errors.length > 0) {
    throw new Error(`Invalid sweep config (fix [${config.fixedAnchors.join(', ')}], drive ${config.driveAnchor}, sensor ${config.sensorAnchor}): ${errors.join('; ')}`);
  }
}

function normalizeSweepConfig(config, machineConfig) {
  if (!config || !machineConfig) {
    return config;
  }
  const fixedAnchors = Array.isArray(config.fixedAnchors)
    ? [...config.fixedAnchors].sort((a, b) => a - b)
    : [];
  const [driveAnchor, sensorAnchor] = canonicalDriveSensorPair(config.driveAnchor, config.sensorAnchor);
  return { ...config, fixedAnchors, driveAnchor, sensorAnchor };
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
    noiseSampleCount,
    noiseSampleIntervalMs,
    noiseMinSamples,
    noiseSigmaFloorDeg,
    fixedAnchors,
    fixedTargets,
    projectZeroTension,
    forbiddenForceAnchors = [],
    baseRadiusMm = null,
  } = options;
  const { driveAnchor, sensorAnchor } = sweepConfig;
  const driveAxis = axes[driveAnchor];
  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const fixedSet = new Set(fixedAnchors ?? []);
  const fixedTargetByAnchor = buildFixedTargetByAnchor(fixedAnchors, fixedTargets, motorIds.length);

  await waitForStableEncoders(sendFn, motorIds, speedup);
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
  await waitForStableEncoders(sendFn, motorIds, speedup);

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
  const forceStable = await waitForStableEncoders(sendFn, motorIds, speedup);
  const endAngles = forceStable.anglesDeg;
  const endLengths = endAngles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));

  const driveEndPointMm = endLengths[driveAnchor] ?? driveStartPointMm;

  const dataPoints = [];
  const recordPoint = (angles, driveSetpointMm, stepIndex, stepCount, extraFields = null) => {
    const lengths = angles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
    const point = {
      l_drive: lengths[driveAnchor],
      l_sensor: lengths[sensorAnchor],
      timestamp_ms: Date.now() - datasetStartMs,
      raw_angles_deg: angles,
      drive_setpoint_mm: driveSetpointMm,
      step_index: stepIndex,
      step_count: stepCount,
    };
    if (extraFields && typeof extraFields === 'object') {
      Object.assign(point, extraFields);
    }
    dataPoints.push(point);
    return lengths;
  };

  //recordPoint(endAngles, driveEndPointMm, 0, sweepPoints);

  await applyForceModeState(sendFn, { motorIds, modes: returnModes });

  const steps = Math.max(2, sweepPoints + 1);
  const stepCount = steps - 1;
  const totalDelta = driveStartPointMm - driveEndPointMm;
  const stepDirection = Math.sign(totalDelta) || 1;
  const initialStepMm = computeInitialForceSweepStepMm(baseRadiusMm) * stepDirection;
  const remainingSteps = Math.max(1, stepCount - 1);
  const stepDelta = (totalDelta - initialStepMm) / remainingSteps;
  let currentDrive = driveEndPointMm;
  for (let stepIdx = 1; stepIdx < steps; stepIdx += 1) {
    const target = stepIdx === 1
      ? driveEndPointMm + initialStepMm
      : driveEndPointMm + initialStepMm + stepDelta * (stepIdx - 1);
    const delta = target - currentDrive;
    if (Number.isFinite(delta) && Math.abs(delta) > 1e-6) {
      // eslint-disable-next-line no-await-in-loop
      await applyDataPointReturnModes(sendFn, {
        motorIds,
        driveAnchor,
        sensorAnchors: [sensorAnchor],
        fixedAnchors,
        forbiddenForceAnchors,
        forceMax,
        forceMin: forceMid,
        forceMid,
        speedup,
      });
      if (fixedTargetByAnchor) {
        // eslint-disable-next-line no-await-in-loop
        await enforceFixedAnchors(sendFn, {
          motorIds,
          axes,
          mmPerDeg,
          fixedTargetByAnchor,
          feed,
          speedup,
        });
      }
      // eslint-disable-next-line no-await-in-loop
      await runMoveWithWait(
        sendFn,
        `G1 H2 ${driveAxis}${delta.toFixed(3)} F${feed}`,
        speedup,
        { axes },
      );
      // eslint-disable-next-line no-await-in-loop
      await waitForStableEncoders(sendFn, motorIds, speedup);
    }
    // eslint-disable-next-line no-await-in-loop
    const collected = await collectDataPoint(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      driveAnchor,
      sensorAnchors: [sensorAnchor],
      fixedAnchors,
      forbiddenForceAnchors,
      forceMax,
      forceMin: forceMid,
      forceMid,
      speedup,
      recordPoint,
      driveSetpointMm: target,
      stepIndex: stepIdx - 1,
      stepCount,
      projectZeroTension: !!projectZeroTension,
      skipReturnModePrep: Number.isFinite(delta) && Math.abs(delta) > 1e-6,
      encoderNoiseOptions: {
        sampleCount: noiseSampleCount,
        sampleIntervalMs: noiseSampleIntervalMs,
        speedup,
      },
    });
    const noiseStats = collected?.noiseStats;
    if (noiseStats && dataPoints.length > 0) {
      const point = dataPoints[dataPoints.length - 1];
      point.mu = noiseStats.muByMotorDeg;
      point.sigma = noiseStats.sigmaByMotorDeg;
      point.sample_count = noiseStats.sampleCount;
      point.sampling_hz = noiseStats.samplingHz;
      if (Number.isFinite(noiseStats.durationMs)) {
        point.sample_duration_ms = noiseStats.durationMs;
      }
      if (Array.isArray(noiseStats.muByMotorDeg)) {
        const muLengths = noiseStats.muByMotorDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
        if (Number.isFinite(muLengths[driveAnchor])) {
          point.l_drive_mu = muLengths[driveAnchor];
        }
        if (Number.isFinite(muLengths[sensorAnchor])) {
          point.l_sensor_mu = muLengths[sensorAnchor];
        }
      }
      const warnings = summarizeNoiseWarnings(noiseStats, {
        minSamples: noiseMinSamples,
        sigmaFloorDeg: noiseSigmaFloorDeg,
      });
      if (warnings.length > 0) {
        point.noise_warnings = warnings;
      }
    }
    const lengths = collected?.lengths;
    if (Array.isArray(lengths)) {
      currentDrive = lengths[driveAnchor] ?? currentDrive;
    }
  }

  return { dataPoints, driveRange: { start: driveEndPointMm, end: driveStartPointMm } };
}

// Collect sweeps for each config once; fixed anchors use explicit targets or size-tuned travel.
export async function collectSweepData(send, context) {
  const input = validateSweepCollectionInput(context);
  const options = applySweepDefaults(input);
  const {
    machineType,
    machineConfig,
    motorIds,
    speedup,
    sweepPoints,
    maxSweepCount,
    feed,
    settleMs,
    noiseSampleCount,
    noiseSampleRateHz,
    noiseSampleIntervalMs,
    noiseMinSamples,
    noiseSigmaFloorDeg,
    fixedTargets: fixedTargetsSpec,
    maxTravelMm: maxTravelOverride,
    debugSweep,
    autoTuneForce,
    returnToOrigin,
    outputFile: outputFileOverride,
    forceMaxProvided,
    forceBuildupFactor,
    preserveBuildupFactor,
    forceBaseRadii,
  } = options;
  let { forceLow, forceMid, forceMax } = options;

  const sweepConfigs = await resolveSweepConfigs({
    sweepConfigFile: options.sweepConfigFile,
    machineType,
    machineConfig,
    maxSweepCount,
  });

  if (debugSweep) {
    console.log(`Planned sweeps (${sweepConfigs.length}):`);
    for (const sweep of sweepConfigs) {
      const sub = expandSubSweepsForConfig(sweep, machineConfig);
      console.log(
        `  fix [${sweep.fixedAnchors.join(', ')}], drive ${sweep.driveAnchor}, sensor ${sweep.sensorAnchor}` +
        (sub.length > 1
          ? ` (sub-sweeps: ${sub.map((cfg) => `[d${cfg.driveAnchor}->s${cfg.sensorAnchor}]`).join(', ')})`
          : ''),
      );
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

  const forcedBuildupFactor = resolveForcedBuildupFactor({
    forceBuildupFactor,
    preserveBuildupFactor,
  });
  const forcedBaseRadii = resolveForcedBaseRadii({
    forceBaseRadii,
    numAnchors: machineConfig.numAnchors,
  });
  const m666AdjustmentCommand = buildM666AdjustmentCommand({
    forcedBaseRadii,
    forcedBuildupFactor,
  });
  let m666Adjusted = null;
  if (typeof m666AdjustmentCommand === 'string' && m666AdjustmentCommand.length > 0) {
    await send(m666AdjustmentCommand);
    const m666AfterReply = await send('M666');
    m666Adjusted = parseM666(m666AfterReply?.reply);
  }

  const m666ForCollection = (m666Adjusted && typeof m666Adjusted === 'object') ? m666Adjusted : m666Before;
  const mmPerDeg = machineConfig.axes.map((_, idx) => computeMmPerDegree(m666ForCollection, idx));
  const missingAxes = mmPerDeg
    .map((val, idx) => (Number.isFinite(val) ? null : machineConfig.axes[idx]))
    .filter(Boolean);
  if (missingAxes.length > 0) {
    console.warn(`Warning: missing mm/deg calibration for axes [${missingAxes.join(', ')}]; lengths will default to 0 on those axes.`);
  }
  attachDebugState(send, { mmPerDeg });

  await primeEncoders(send, { motorIds, axes: machineConfig.axes });

  let forceTuningMeta = null;
  if (autoTuneForce) {
    const forbidden = new Set(getForceForbiddenAnchors(machineConfig));
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
        forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
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
  await waitForStableEncoders(send, motorIds, speedup);
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => forceMid*0.5),
  });
  await waitForStableEncoders(send, motorIds, speedup);
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => forceLow),
  });
  await waitForStableEncoders(send, motorIds, speedup);
  await primeEncoders(send, { motorIds, axes: machineConfig.axes });
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  await waitForStableEncoders(send, motorIds, speedup);

  const useFixedTargets = fixedTargetsSpec !== null;
  let maxTravelMm = Number.isFinite(maxTravelOverride) ? maxTravelOverride : null;
  let sizeTunePair = null;
  if (!useFixedTargets && maxTravelOverride === null) {
    sizeTunePair = selectSizeTunePair(sweepConfigs, getForceForbiddenAnchors(machineConfig));
    if (!sizeTunePair) {
      console.log('; size-tune skipped (no force-capable anchor pair)');
    } else {
      const measuredMaxTravelMm = await measureMaxTravelMm(send, {
        motorIds,
        mmPerDeg,
        forceLow,
        forceMax,
        pairAnchors: sizeTunePair,
        forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
        speedup,
      });
      maxTravelMm = scaleMeasuredMaxTravelMm(machineConfig, measuredMaxTravelMm);
      if (Number.isFinite(maxTravelMm)) {
        if (
          Number.isFinite(measuredMaxTravelMm)
          && Math.abs(measuredMaxTravelMm - maxTravelMm) > 1e-9
        ) {
          console.log(
            `; size-tune max travel raw=${measuredMaxTravelMm.toFixed(3)}mm scaled=${maxTravelMm.toFixed(3)}mm`,
          );
        } else {
          console.log(`; size-tune max travel=${maxTravelMm.toFixed(3)}mm`);
        }
      } else {
        console.log('; size-tune failed; using 0.0mm');
      }
    }
  }
  if (!Number.isFinite(maxTravelMm)) {
    maxTravelMm = 0;
  }
  const maxTravelMeta = useFixedTargets ? undefined : maxTravelMm;
  const totalPlannedSweeps = sweepConfigs.length;
  let sweepOrdinal = 0;

  for (const sweepConfig of sweepConfigs) {
    sweepOrdinal += 1;
    const fixedTargets = resolveFixedTargets(
      sweepConfig.fixedAnchors,
      fixedTargetsSpec,
      maxTravelMm,
      machineConfig,
    );
    const fixedDesc = useFixedTargets
      ? fixedTargets.map((val) => val.toFixed(3)).join(', ')
      : '';
    const fixedInfo = fixedDesc.length > 0 ? `, fixed targets [${fixedDesc}]` : '';

    const subSweeps = expandSubSweepsForConfig(sweepConfig, machineConfig);
    console.log(
      `\nSweep ${sweepOrdinal}/${totalPlannedSweeps}: fix [${sweepConfig.fixedAnchors.join(', ')}],` +
      ` pair ${sweepConfig.driveAnchor}/${sweepConfig.sensorAnchor}${fixedInfo}`,
    );
    if (subSweeps.length === 0) {
      console.warn('  No valid sub-sweeps (fixed-only anchor or invalid config); skipping.');
      continue;
    }

    const aggregatedPoints = [];
    for (let i = 0; i < subSweeps.length; i += 1) {
      const subCfg = subSweeps[i];
      console.log(
        `  Sub-sweep ${i + 1}/${subSweeps.length}: drive ${subCfg.driveAnchor}, sensor ${subCfg.sensorAnchor}`,
      );

      // Re-apply positioning for each sub-sweep to cover the full circle arc in both directions.
      // This ensures fixed anchors are retightened before changing drive/sensor roles.
      // eslint-disable-next-line no-await-in-loop
      await prepareSweepPositioning(send, subCfg, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        forceLow,
        forceMid,
        fixedTargets,
        feed,
        speedup,
      });

      // eslint-disable-next-line no-await-in-loop
      const sweepResult = await performForceSweep(send, subCfg, {
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
        noiseSampleCount,
        noiseSampleIntervalMs,
        noiseMinSamples,
        noiseSigmaFloorDeg,
        fixedAnchors: sweepConfig.fixedAnchors,
        fixedTargets,
        projectZeroTension: options.projectZeroTension,
        forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
        baseRadiusMm: Array.isArray(m666ForCollection?.R)
          ? m666ForCollection.R[subCfg.driveAnchor]
          : m666ForCollection?.R,
      });
      const remapped = remapDataPointsToCanonical(
        sweepResult.dataPoints,
        sweepConfig,
        subCfg,
      );
      aggregatedPoints.push(...remapped);
    }

    const lengths = await getCurrentLengths(send, motorIds, mmPerDeg);
    const fixedLengths = sweepConfig.fixedAnchors.map((idx, anchorIdx) => {
      if (Number.isFinite(lengths[idx])) {
        return lengths[idx];
      }
      return Number.isFinite(fixedTargets[anchorIdx]) ? fixedTargets[anchorIdx] : 0;
    });

    const driveRangeValues = aggregatedPoints
      .map((p) => (Number.isFinite(p?.l_drive) ? p.l_drive : null))
      .filter((v) => v !== null);
    const driveRange = driveRangeValues.length > 0
      ? {
        start: Math.min(...driveRangeValues),
        end: Math.max(...driveRangeValues),
        unit: 'mm',
      }
      : null;

    sweeps.push({
      id: `sweep_${String(sweepOrdinal).padStart(3, '0')}`,
      fixed_anchors: sweepConfig.fixedAnchors,
      fixed_lengths: fixedLengths,
      drive_anchor: sweepConfig.driveAnchor,
      sensor_anchor: sweepConfig.sensorAnchor,
      drive_range: driveRange,
      data_points: aggregatedPoints,
      metadata: {
        sweep_method: 'position',
        feed_rate: feed,
        sensor_force_n: forceMid,
        settle_ms: settleMs,
      },
    });
    console.log(
      `  Collected ${aggregatedPoints.length} points from ${subSweeps.length} sub-sweep${subSweeps.length === 1 ? '' : 's'}`,
    );
    if (returnToOrigin && sweepOrdinal < totalPlannedSweeps) {
      try {
        await returnMotorsToOriginAllAtOnce(send, {
          motorIds,
          axes: machineConfig.axes,
          mmPerDeg,
          feed,
          speedup,
        });
        console.log('Returned all motors to encoder origin (between sweeps).');
      } catch (err) {
        console.warn(`Warning: failed to return to encoder origin: ${err?.message || err}`);
      }
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
  const noiseOriginDeg = Array.isArray(forceTuning?.noise_sigma_deg)
    ? forceTuning.noise_sigma_deg.map((val) => (Number.isFinite(val) ? val : null))
    : null;
  let noiseOriginMm = null;
  if (noiseOriginDeg && Array.isArray(mmPerDeg) && mmPerDeg.length > 0) {
    noiseOriginMm = noiseOriginDeg.map((val, idx) => {
      const mmPer = mmPerDeg[idx];
      if (!Number.isFinite(val) || !Number.isFinite(mmPer)) {
        return null;
      }
      return val * mmPer;
    });
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
      m666_before_data_collection: m666Before,
      m666_adjusted_by_data_collector: m666Adjusted ?? undefined,
      m669: m669Values,
      m92: m92Values,
      mm_per_degree: mmPerDeg,
      notes: {
        buildup_factor_forced: Number.isFinite(forcedBuildupFactor) ? forcedBuildupFactor : undefined,
        base_radii_forced_mm: forcedBaseRadii ?? undefined,
      },
      force_tuning: forceTuning,
      max_travel_mm: Number.isFinite(maxTravelMeta) ? maxTravelMeta : undefined,
      encoder_noise_origin_deg: noiseOriginDeg ?? undefined,
      encoder_noise_origin_mm: noiseOriginMm ?? undefined,
      encoder_noise: {
        method: 'mad',
        units: 'deg',
        sigma_floor_deg: noiseSigmaFloorDeg,
        min_samples: noiseMinSamples,
        sample_count: noiseSampleCount,
        sample_interval_ms: noiseSampleIntervalMs,
        sample_rate_hz: noiseSampleRateHz,
      },
    },
    sweeps,
  };

  const outputFile = outputFileOverride || `sweep_data_${machineType}_${Date.now()}.json`;
  const outputDir = path.dirname(outputFile);
  if (outputDir && outputDir !== '.') {
    await fs.mkdir(outputDir, { recursive: true });
  }
  await fs.writeFile(outputFile, JSON.stringify(dataset, null, 2));
  console.log(`\nSaved ${sweeps.length} sweeps to ${outputFile}`);

  if (returnToOrigin) {
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
