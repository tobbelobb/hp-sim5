import fs from 'node:fs/promises';
import path from 'node:path';
import {
  computeMmPerDegree,
  parseM666,
  parseM92,
  parseM669,
  runMoveWithWait,
  sleep as baseSleep,
} from '../primitives/encoder_utils.mjs';
import {
  buildMotorCatalogFromConfig,
  computeMmPerDegreeFromDescriptor,
  resolveMotorToken,
} from '../../../integrations/klipper/klipperMotorAddressConfig.js';
import { DEFAULT_KLIPPY_CONFIG_PATH } from '../../../integrations/klipper/klippy_api_cli_config.mjs';
import { ENCODER_NOISE_DEFAULTS, sampleEncoderNoise } from '../primitives/encoder_noise.mjs';
import { collectWithSlipRecovery } from './slip_recovery.mjs';
import { probeSpoolInterval, UnmeasurableSweepError } from './spool_reserve.mjs';
import {
  angleToLength,
  assertFixedAnchorAngles,
  applyDataPointPreloadModes,
  applyForceModeState,
  collectDataPoint,
  EncoderStabilityTimeoutError,
  getCurrentLengths,
  primeEncoders,
  returnMotorsToOriginOneAtATime,
  resolveCollectionForce,
  settleOnTargets,
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
      3: { maxFixed: 0 },
    },
    measuredMaxTravelScale: 0.8,
  },
  hangprinter_5: {
    numAnchors: 5, dimensions: 3, axes: ['X', 'Y', 'Z', 'U', 'V'], mustBeInFixedSet: [4],
    fixedTargetBoundsByAnchor: Object.fromEntries(range(5).map(axis => [axis, { maxFixed: 0 }])),
    defaultFixedTargetScale: -1, measuredMaxTravelScale: 0.8,
  },
  cubecorners: { numAnchors: 8, dimensions: 3, axes: ['X', 'Y', 'Z', 'U', 'V', 'W', 'A', 'B'], mustBeInFixedSet: [] },
  skycam: { numAnchors: 4, dimensions: 3, axes: ['X', 'Y', 'Z', 'U'], mustBeInFixedSet: [] },
};

MACHINE_CONFIGS.hangprinter_5_pulley_reduced = MACHINE_CONFIGS.hangprinter_5;
MACHINE_CONFIGS.hangprinter_3 = MACHINE_CONFIGS.hangprinter_4;

export const MOTOR_IDS_BY_MACHINE = {
  slideprinter: ['40.0', '41.0', '42.0'],
  hangprinter_3: ['40.0', '41.0', '42.0', '43.0'],
  hangprinter_4: ['40.0', '41.0', '42.0', '43.0'],
  hangprinter_5: ['40.0', '41.0', '42.0', '43.0', '44.0'],
  hangprinter_5_pulley_reduced: ['40.0', '41.0', '42.0', '43.0', '44.0'],
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
// A fixed anchor is position-controlled, so its encoder reads the loaded
// equilibrium of the cable rather than the commanded position. That residual is
// compliance, and it is stable within a sweep: in the no-buildup reference every
// one of the 9 sweeps had a fixed anchor sitting 0.013-0.029 mm off target, and
// the value was bit-identical across all 20 points of the sweep even though a
// ~0.03 mm correction move was issued before each point. A tolerance below that
// level therefore only buys no-op moves followed by a full settle. 0.05 mm is
// about 5x the 0.02 deg encoder noise floor (0.0105 mm at 30 mm radius) and about
// 16x below the 1.5 deg drift-recovery threshold, so genuine slipping drift is
// still corrected.
const DEFAULT_FIXED_TOLERANCE_MM = 0.05;
// Active workspace traversal needs time to move before the quiet window starts.
// This is measured on the observed simulation clock (wall time on hardware).
const FORCE_TRAVEL_TIMEOUT_MS = 600000;
// A fixed-torque HP4 pull at the production 2 ms step passed the unchanged
// settling check after ~472 s (four substeps: ~120 s). Allow this slow approach
// to equilibrium without relaxing drift/vibration acceptance.
const SIZE_TUNE_TRAVEL_TIMEOUT_MS = 600000;
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

// A natural target, { natural: offset }, holds a line where it settles once the
// other held lines are in place, plus the offset (mm).
const isNaturalTarget = target => Number.isFinite(target?.natural);

function normalizeFixedTargets(values, fixedCount) {
  if (!Array.isArray(values) || fixedCount <= 0) {
    return [];
  }
  const normalized = values.slice(0, fixedCount).map((val) => (Number.isFinite(val) || isNaturalTarget(val) ? val : 0));
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

export function buildGlobalForceModes(motorIds, force, forbiddenForceAnchors = []) {
  const forbidden = new Set((forbiddenForceAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  return motorIds.map((_, idx) => (forbidden.has(idx) ? 'position' : force));
}

export function buildSymmetricPulloutModes({
  motorIds,
  movingAnchors = new Set(),
  fixedAnchors = [],
  forbiddenForceAnchors = [],
  forceMid,
  forceLow,
  activeAnchors = null,
  forceMax,
} = {}) {
  const fixedSet = new Set((fixedAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  const movingSet = movingAnchors instanceof Set
    ? movingAnchors
    : new Set((movingAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  const forbidden = new Set((forbiddenForceAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  // Pullout needs more preload than measurement to traverse the workspace.
  const pullForce = resolveCollectionForce({ forceMid, forceMax, preloadMultiplier: 20 });
  return motorIds.map((_, idx) => {
    if (movingSet.has(idx) || fixedSet.has(idx) || forbidden.has(idx)) return 'position';
    return activeAnchors && !activeAnchors.includes(idx) ? forceLow : pullForce;
  });
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

const GCODE_AXIS_TO_KLIPPER_STEPPER_AXIS = {
  X: 'A',
  Y: 'B',
  Z: 'C',
  U: 'D',
  V: 'E',
  W: 'F',
};

function normalizeKlipperStepperAxis(axis) {
  const upper = String(axis || '').trim().toUpperCase();
  if (!upper) {
    return null;
  }
  return GCODE_AXIS_TO_KLIPPER_STEPPER_AXIS[upper] || upper;
}

export function resolveKlipperMmPerDegreeFromConfig({
  motorIds = [],
  axes = [],
  configPath = DEFAULT_KLIPPY_CONFIG_PATH,
} = {}) {
  const catalog = buildMotorCatalogFromConfig(configPath);
  if (!catalog || !Array.isArray(catalog.stepperDescriptors) || catalog.stepperDescriptors.length === 0) {
    return Array.from({ length: motorIds.length || axes.length }, () => null);
  }

  const byStepperAxis = new Map();
  for (const descriptor of catalog.stepperDescriptors) {
    const stepperAxis = String(descriptor?.axis || '').toUpperCase();
    if (!stepperAxis || byStepperAxis.has(stepperAxis)) {
      continue;
    }
    byStepperAxis.set(stepperAxis, descriptor);
  }

  const count = Math.max(motorIds.length, axes.length);
  const mmPerDeg = Array.from({ length: count }, () => null);
  for (let idx = 0; idx < count; idx += 1) {
    const token = motorIds[idx];
    let descriptor = null;
    if (token != null) {
      const resolved = resolveMotorToken(token, catalog);
      if (resolved?.resolved) {
        descriptor = resolved.resolved;
      }
    }

    if (!descriptor) {
      const axis = normalizeKlipperStepperAxis(axes[idx]);
      if (axis && byStepperAxis.has(axis)) {
        descriptor = byStepperAxis.get(axis);
      }
    }

    mmPerDeg[idx] = computeMmPerDegreeFromDescriptor(descriptor);
  }
  return mmPerDeg;
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

// Initial drive displacement for seeking the reserved part of the arc. The
// sensor's actual half-spindle-turn retraction is checked separately below.
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
    targets = fixedAnchors.map(axis => getMustBeInFixedSet(machineConfig).includes(axis) ? 0
      : value * (machineConfig?.defaultFixedTargetScale ?? 1));
  }
  return targets.map((target, idx) => (isNaturalTarget(target) ? target
    : constrainFixedTarget(machineConfig, fixedAnchors[idx], target)));
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
  const sensorCollectionForce = parseOptionalNumber(
    args.sensorCollectionForce ?? args.sensorForce,
    'sensor-collection-force',
  );
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
    firmware: String(args.firmware || 'rrf').toLowerCase(),
    klipperConfigPath: args.config ?? null,
    motorIds,
    speedup: parseOptionalNumber(speedup, 'speedup'),
    sweepPoints,
    maxSweeps,
    feed,
    settleMs,
    forceLow,
    forceMid,
    forceMax,
    sensorCollectionForce,
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
    sensorForceProfileFile: args.sensorForceProfile ?? null,
    extraSensors: config.numAnchors === 5 && config.dimensions === 3 && !args.noExtraSensors,
    completedSweeps: parseOptionalNumber(args.completedSweeps, 'completed-sweeps', { integer: true }) ?? 0,
    debugSweep: !!args.debugSweep,
    autoTuneForce: !!args.autoTuneForce,
    noAutoTuneForce: !!args.noAutoTuneForce,
    returnToOrigin: !!args.returnToOrigin,
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
  const sensorCollectionForce = Number.isFinite(input.sensorCollectionForce)
    ? input.sensorCollectionForce
    : null;
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
    sensorCollectionForce,
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

// Each entry of the file is one sweep direction: sensor force (N) tabulated
// against the drive line's length change (mm, ascending).
export async function loadSensorForceProfiles(filePath) {
  const profiles = JSON.parse(await fs.readFile(filePath, 'utf8'));
  if (!Array.isArray(profiles) || profiles.length === 0) {
    throw new Error(`Sensor force profile must contain at least one table: ${filePath}`);
  }
  for (const profile of profiles) {
    const { drive_mm: driveMm, force_n: forceN } = profile;
    if (!Array.isArray(driveMm) || !Array.isArray(forceN) || driveMm.length < 2 || driveMm.length !== forceN.length
      || ![...driveMm, ...forceN].every(Number.isFinite) || forceN.some(v => v <= 0)
      || (profile.extra_sensor_force_n !== undefined
        && !(Number.isFinite(profile.extra_sensor_force_n) && profile.extra_sensor_force_n > 0))
      || driveMm.some((v, i) => i > 0 && v <= driveMm[i - 1])) {
      throw new Error(`Invalid sensor force profile in ${filePath}`);
    }
  }
  return profiles;
}

export function sensorForceProfileFor(profiles, driveAnchor, sensorAnchor, fixedAnchors, fixedTargets) {
  const matches = profiles.filter(p => p.drive_anchor === driveAnchor && p.sensor_anchor === sensorAnchor
    && Array.isArray(p.fixed_anchors) && Array.isArray(p.fixed_lengths)
    && p.fixed_anchors.length === fixedAnchors.length && p.fixed_lengths.length === fixedTargets.length
    && fixedAnchors.every((anchor, i) => p.fixed_anchors[i] === anchor
      && Math.abs(p.fixed_lengths[i] - fixedTargets[i]) <= 1e-3));
  if (matches.length !== 1) {
    throw new Error(`Sensor force profile needs exactly one entry for drive ${driveAnchor}, sensor ${sensorAnchor}, fixed [${fixedAnchors}] at [${fixedTargets}]; found ${matches.length}`);
  }
  return matches[0];
}

// Linear interpolation; beyond the table the nearest end value applies.
export function sensorForceAt(profile, driveMm) {
  const { drive_mm: xs, force_n: forces } = profile;
  if (driveMm <= xs[0]) return forces[0];
  if (driveMm >= xs[xs.length - 1]) return forces[forces.length - 1];
  let hi = 1;
  while (xs[hi] < driveMm) hi += 1;
  const t = (driveMm - xs[hi - 1]) / (xs[hi] - xs[hi - 1]);
  return forces[hi - 1] + t * (forces[hi] - forces[hi - 1]);
}

function sweepPairAllowed(config, fixed, drive, sensor) {
  const top = getMustBeInFixedSet(config);
  const lowCount = config.numAnchors - top.length;
  if (top.length !== 1 || top[0] !== config.numAnchors - 1 || lowCount < 4 || lowCount % 2) return true;
  const oppositeFixed = fixed.filter(axis => axis < lowCount).map(axis => (axis + lowCount / 2) % lowCount);
  return !oppositeFixed.includes(drive) && !oppositeFixed.includes(sensor);
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
      if (sweepPairAllowed(machineConfig, fixedAnchors, driveAnchor, sensorAnchor)) {
        configs.push({ fixedAnchors, driveAnchor, sensorAnchor });
      }
    }
  }

  return configs;
}

function selectSizeTunePair(sweepConfigs, fixedOnlyAnchors = [], numAnchors = 0) {
  if (fixedOnlyAnchors.length === 1 && numAnchors > 4) {
    // Size tuning pulls both lines at once: use neighbours, since opposing
    // winches only stretch their lines and report a near-zero workspace.
    return range(numAnchors).filter(axis => !fixedOnlyAnchors.includes(axis)).slice(0, 2);
  }
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


// Moves the held lines to their targets and returns the lengths they hold, in
// fixed-anchor order. Numeric targets move first. With natural targets, every
// free line then idles at the low force, so the effector hangs where the held
// lines leave it, not at the drive's pulled-in end, and the natural-target lines
// move by their offsets from where they settled. A line that cannot reach its target holds where it could
// (settleOnTargets), and that length is returned instead.
async function prepareSweepPositioning(sendFn, sweepConfig, options) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    forceMid,
    forceLow,
    forceMax,
    forbiddenForceAnchors = [],
    fixedTargets = [],
    feed = DEFAULT_FEED,
    speedup,
  } = options;
  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  const fixedAnchors = sweepConfig.fixedAnchors || [];
  const targetOf = idx => (Number.isFinite(fixedTargets[idx]) ? fixedTargets[idx] : 0);
  const numeric = fixedAnchors.filter((_, idx) => !isNaturalTarget(fixedTargets[idx]));
  const natural = numeric.length < fixedAnchors.length;
  const moveHeld = async (targetByAnchor, heldAnchors) => {
    const starts = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
    const targets = motorIds.map((_, idx) => (Number.isFinite(targetByAnchor[idx]) ? targetByAnchor[idx] : null));
    const moving = new Set(targets.flatMap((target, idx) => (target !== null && Math.abs(target - starts[idx]) > 1e-6 ? [idx] : [])));
    if (moving.size === 0) return starts;
    await applyForceModeState(sendFn, {
      motorIds,
      modes: buildSymmetricPulloutModes({
        motorIds, movingAnchors: moving, fixedAnchors: heldAnchors, forbiddenForceAnchors,
        forceMid, forceMax, forceLow, activeAnchors: natural ? [] : [sweepConfig.driveAnchor, sweepConfig.sensorAnchor],
      }),
    });
    const parts = [...moving].map(idx => `${axes[idx]}${(targets[idx] - starts[idx]).toFixed(3)}`);
    await runMoveWithWait(sendFn, `G1 H2 ${parts.join(' ')} F${feed}`, speedup, { axes });
    return settleOnTargets(sendFn, { motorIds, axes, mmPerDeg, targets, starts, feed, speedup });
  };
  const first = new Array(motorIds.length).fill(null);
  fixedAnchors.forEach((anchor, idx) => { if (!isNaturalTarget(fixedTargets[idx])) first[anchor] = targetOf(idx); });
  let lengths = await moveHeld(first, numeric);
  if (natural) {
    const second = new Array(motorIds.length).fill(null);
    fixedAnchors.forEach((anchor, idx) => {
      second[anchor] = isNaturalTarget(fixedTargets[idx]) ? lengths[anchor] + fixedTargets[idx].natural : lengths[anchor];
    });
    lengths = await moveHeld(second, fixedAnchors);
  }
  return fixedAnchors.map(anchor => lengths[anchor]);
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

export async function measureMaxTravelMm(sendFn, options = {}) {
  const {
    motorIds,
    mmPerDeg,
    forceLow,
    forceMax,
    pairAnchors,
    forbiddenForceAnchors = [],
    speedup,
    settleOptions = {},
  } = options;

  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const pairSet = new Set(pairAnchors);

  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  const startLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);

  try {
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
        await (sendFn.simulationClock?.sleep ?? baseSleep)(100);
      }
    }
    await waitForStableEncoders(sendFn, motorIds, speedup, {
      timeoutMs: SIZE_TUNE_TRAVEL_TIMEOUT_MS,
      wallTimeoutMs: SIZE_TUNE_TRAVEL_TIMEOUT_MS + 30000,
      onProgress: message => console.log(`; size-tune travel: ${message.replace(/^; /, '')}`),
      ...settleOptions,
    });
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

    return Number.isFinite(maxTravel) ? maxTravel : null;
  } finally {
    // Also stop the pull when a clock/encoder/transport error aborts measurement.
    await applyForceModeState(sendFn, {
      motorIds,
      modes: motorIds.map(() => 'position'),
    });
  }
}

function parseSweepConfigLine(line, lineNumber) {
  const trimmed = line.trim();
  if (!trimmed || trimmed.startsWith('#')) {
    return null;
  }
  const parts = trimmed.split(/\s+/);
  if (parts.length < 3) {
    throw new Error(`Invalid sweep config line ${lineNumber}: expected fixedAnchors drive sensor [fixed=targets]`);
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
  // Optional per-line held lengths, in the listed fixed-anchor order: "fixed=430,-215".
  // "n", "n+200" or "n-50" holds a line where it settles after the others moved, plus an offset.
  if (parts.length < 4) return { fixedAnchors, driveAnchor, sensorAnchor };
  const fixedTargets = parts[3].replace(/^fixed=/, '').split(',').map((token) => {
    const natural = /^n([+-]\d+(?:\.\d+)?)?$/.exec(token);
    return natural ? { natural: Number(natural[1] ?? 0) } : Number(token);
  });
  if (!parts[3].startsWith('fixed=') || fixedTargets.length !== fixedAnchors.length
      || !fixedTargets.every(target => Number.isFinite(target) || isNaturalTarget(target))) {
    throw new Error(`Invalid sweep config line ${lineNumber}: expected fixed=<one length or n[+-offset] per fixed anchor>`);
  }
  return { fixedAnchors, driveAnchor, sensorAnchor, fixedTargets };
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
      const key = `${normalized.fixedAnchors.join(',')}|${normalized.driveAnchor}|${normalized.sensorAnchor}|${JSON.stringify(normalized.fixedTargets ?? null)}`;
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
  const order = (config.fixedAnchors ?? []).map((_, idx) => idx)
    .sort((a, b) => config.fixedAnchors[a] - config.fixedAnchors[b]);
  const fixedAnchors = order.map(idx => config.fixedAnchors[idx]);
  const [driveAnchor, sensorAnchor] = canonicalDriveSensorPair(config.driveAnchor, config.sensorAnchor);
  const normalized = { ...config, fixedAnchors, driveAnchor, sensorAnchor };
  if (config.fixedTargets) normalized.fixedTargets = order.map(idx => config.fixedTargets[idx]);
  return normalized;
}

// Check the spare encoder on two consecutive ordinary sweep steps. The first
// two readings establish a history. After a plateau or a turning point, wait
// for two resolved steps again. Keep this history local to one loaded sub-sweep.
function spareRangeMoved(points, anchor, sigmaFloorDeg) {
  if (points.length < 3) return false;
  const [a, b, c] = points.slice(-3);
  if (b.step_index !== a.step_index + 1 || c.step_index !== b.step_index + 1) return false;
  const resolvedStep = (from, to) => {
    const change = to.mu[anchor] - from.mu[anchor];
    const noise = Math.hypot(Math.max(sigmaFloorDeg, from.sigma[anchor]),
                             Math.max(sigmaFloorDeg, to.sigma[anchor]));
    return Math.abs(change) > 5 * noise ? Math.sign(change) : 0;
  };
  return resolvedStep(a, b) * resolvedStep(b, c) === 1;
}

export async function performForceSweep(sendFn, sweepConfig, options) {
  const {
    axes,
    motorIds,
    sweepPoints,
    feed,
    speedup,
    mmPerDeg,
    stepMmByAnchor = [],
    datasetStartMs,
    forceLow,
    forceMid,
    forceMax,
    sensorCollectionForce: constantSensorForce = null,
    sensorForceProfile = null,
    extraSensors = false,
    noiseSampleCount,
    noiseSampleIntervalMs,
    noiseMinSamples,
    noiseSigmaFloorDeg,
    fixedAnchors,
    fixedTargets,
    forbiddenForceAnchors = [],
    baseRadiusMm = null,
    sensorSpoolGearRatio = 1,
    probeSpoolReserve = false,
  } = options;
  const { driveAnchor, sensorAnchor } = sweepConfig;
  const driveAxis = axes[driveAnchor];
  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const fixedSet = new Set(fixedAnchors ?? []);
  // Constant-force HP5 collection records both loads from the first measurement.
  // Historical profiles without an extra load retain their original force policy.
  const extraForce = sensorForceProfile ? sensorForceProfile.extra_sensor_force_n
    : extraSensors ? resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce: constantSensorForce }) : null;
  const extraAnchors = Number.isFinite(extraForce) && extraForce > 0
    ? motorIds.map((_, idx) => idx).filter(idx => idx !== driveAnchor && idx !== sensorAnchor
      && !fixedSet.has(idx) && !forbidden.has(idx)) : [];
  const measurementSensors = [sensorAnchor, ...extraAnchors];
  const measurementSensorForces = Object.fromEntries(extraAnchors.map(idx => [idx, extraForce]));
  if (extraAnchors.length) console.log(`; measurement sensors [${measurementSensors.join(', ')}]; `
    + `extra force ${extraForce.toFixed(3)} N; pullout keeps the idle force`);
  const fixedTargetByAnchor = buildFixedTargetByAnchor(fixedAnchors, fixedTargets, motorIds.length);
  const settleOptions = {
    validateAngles: angles => assertFixedAnchorAngles(angles, { fixedTargetByAnchor, mmPerDeg }),
  };

  // Fixed-target preparation leaves the free motors pulling across the
  // workspace. Give that transit the pullout budget before ordinary point waits.
  await waitForStableEncoders(sendFn, motorIds, speedup, {
    ...settleOptions,
    timeoutMs: FORCE_TRAVEL_TIMEOUT_MS,
    wallTimeoutMs: FORCE_TRAVEL_TIMEOUT_MS + 30000,
    onProgress: message => console.log(`; sweep preparation: ${message.replace(/^; /, '')}`),
  });
  const initialLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  let driveStartPointMm = initialLengths[driveAnchor] ?? 0;

  const modesPullout = motorIds.map((_, idx) => {
    if (idx === driveAnchor) {
      return resolveCollectionForce({ forceMid, forceMax, preloadMultiplier: 20 });
    }
    if (fixedSet.has(idx) || forbidden.has(idx)) {
      return 'position';
    }
    return forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: modesPullout });
  try {
    await waitForStableEncoders(sendFn, motorIds, speedup, {
      ...settleOptions,
      timeoutMs: FORCE_TRAVEL_TIMEOUT_MS,
      wallTimeoutMs: FORCE_TRAVEL_TIMEOUT_MS + 30000,
      onProgress: message => console.log(`; sweep pullout: ${message.replace(/^; /, '')}`),
    });
  } catch (error) {
    if (!(error instanceof EncoderStabilityTimeoutError)) throw error;
    // Pullout searches for an endpoint; it is not a measurement. Hold the drive
    // below and require ordinary settling under preload before using that endpoint.
    console.log('; sweep pullout reached travel time limit; holding drive before settling');
  }

  // Capture maximum payout before the stronger preload may take up loose line.
  const pulloutLengths = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  const sensorPayoutMm = pulloutLengths[sensorAnchor];
  const preloadModes = motorIds.map((_, idx) => {
    if (idx === driveAnchor) {
      return 'position';
    }
    if (fixedSet.has(idx) || forbidden.has(idx)) {
      return 'position';
    }
    return measurementSensors.includes(idx) ? forceMid * 2.0 : forceLow;
  });
  await applyForceModeState(sendFn, { motorIds, modes: preloadModes });
  const forceStable = await waitForStableEncoders(sendFn, motorIds, speedup, {
    ...settleOptions,
    timeoutMs: FORCE_TRAVEL_TIMEOUT_MS,
    wallTimeoutMs: FORCE_TRAVEL_TIMEOUT_MS + 30000,
    onProgress: message => console.log(`; sweep pullout preload: ${message.replace(/^; /, '')}`),
  });
  const endAngles = forceStable.anglesDeg;
  const endLengths = endAngles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));

  let driveEndPointMm = endLengths[driveAnchor] ?? driveStartPointMm;
  const rawDriveInterval = [driveEndPointMm, driveStartPointMm];
  const rawDirection = Math.sign(driveStartPointMm - driveEndPointMm);
  if (sensorForceProfile) {
    const lo = Math.max(Math.min(...rawDriveInterval), sensorForceProfile.drive_mm[0]);
    const hi = Math.min(Math.max(...rawDriveInterval), sensorForceProfile.drive_mm.at(-1));
    if (hi - lo <= DEFAULT_FIXED_TOLERANCE_MM) {
      throw new UnmeasurableSweepError('No measured interval lies inside the force profile support', { rawDriveInterval });
    }
    driveEndPointMm = rawDirection >= 0 ? lo : hi;
    driveStartPointMm = rawDirection >= 0 ? hi : lo;
  }
  const totalDelta = driveStartPointMm - driveEndPointMm;
  if (!Number.isFinite(totalDelta) || Math.abs(totalDelta) <= DEFAULT_FIXED_TOLERANCE_MM) {
    throw new UnmeasurableSweepError('No measured drive interval remains for this sweep', { rawDriveInterval });
  }
  const stepDirection = Math.sign(totalDelta);
  const initialStepMm = Math.min(Math.abs(totalDelta), computeInitialForceSweepStepMm(baseRadiusMm));
  const sensorReserveMm = 180 * sensorSpoolGearRatio * mmPerDeg[sensorAnchor];
  if (!(Number.isFinite(sensorPayoutMm) && sensorReserveMm > 0)) {
    throw new Error('Cannot establish the sensor half-turn reserve from encoder gearing');
  }
  const forceAt = target => sensorForceProfile ? sensorForceAt(sensorForceProfile, target)
    : resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce: constantSensorForce });
  let reserveForceActive = null;
  const moveForReserve = async (target, sensorForce = forceAt(target)) => {
    if (reserveForceActive !== sensorForce) {
      await applyDataPointPreloadModes(sendFn, {
        motorIds, driveAnchor, sensorAnchors: measurementSensors, fixedAnchors, forbiddenForceAnchors,
        forceMax, forceMin: forceMid, forceMid, compliantForce: forceLow, sensorCollectionForce: sensorForce, speedup, settleOptions,
      });
      await applyForceModeState(sendFn, { motorIds, modes: motorIds.map((_, idx) => {
        if (idx === driveAnchor || fixedSet.has(idx) || forbidden.has(idx)) return 'position';
        return idx === sensorAnchor ? sensorForce : (measurementSensorForces[idx] ?? forceLow);
      }) });
      reserveForceActive = sensorForce;
    }
    await enforceFixedAnchors(sendFn, { motorIds, axes, mmPerDeg, fixedTargetByAnchor, feed, speedup });
    const current = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
    const delta = target - current[driveAnchor];
    if (Math.abs(delta) > 1e-6) await runMoveWithWait(sendFn,
      `G1 H2 ${driveAxis}${delta.toFixed(3)} F${feed}`, speedup, { axes });
    const stable = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
    const measuredClockMs = sendFn.simulationClock?.now() ?? Date.now();
    const sample = await sampleEncoderNoise(sendFn, motorIds, {
      sampleCount: noiseSampleCount, sampleIntervalMs: noiseSampleIntervalMs, speedup,
    });
    settleOptions.validateAngles(sample.muByMotorDeg);
    if (sample.sampleCount < noiseMinSamples || sample.muByMotorDeg.length !== motorIds.length
        || !sample.muByMotorDeg.every(Number.isFinite)) {
      throw new Error('Insufficient encoder evidence for the spool reserve');
    }
    const reachedMm = sample.muByMotorDeg[driveAnchor] * mmPerDeg[driveAnchor];
    const trackingToleranceMm = DEFAULT_FIXED_TOLERANCE_MM + (stepMmByAnchor[driveAnchor] ?? 0);
    if (Math.abs(reachedMm - target) > trackingToleranceMm) {
      throw new Error(`Drive did not track the spool-reserve probe target: target ${target.toFixed(3)} mm, `
        + `reached ${reachedMm.toFixed(3)} mm, move ${delta.toFixed(3)} mm, tolerance ${trackingToleranceMm.toFixed(3)} mm`);
    }
    return { ...sample, stableData: stable, measuredClockMs };
  };
  let measurementStartMm = driveEndPointMm + stepDirection * initialStepMm;
  let reserved = null;
  if (!probeSpoolReserve) {
    reserved = await moveForReserve(measurementStartMm);
    while (sensorPayoutMm - reserved.muByMotorDeg[sensorAnchor] * mmPerDeg[sensorAnchor] < sensorReserveMm) {
      const remaining = stepDirection * (driveStartPointMm - measurementStartMm);
      if (remaining <= DEFAULT_FIXED_TOLERANCE_MM) {
        throw new Error('Measured sweep is too short to wind the sensor in half a spindle turn');
      }
      measurementStartMm += stepDirection * Math.min(initialStepMm, remaining);
      reserved = await moveForReserve(measurementStartMm);
    }
  }
  const guardedStartMm = measurementStartMm;
  let reserveProbe = { accepted: false, reason: 'initial half-turn reserve' };
  const couplingProbes = [];
  if (!probeSpoolReserve && Math.abs(driveStartPointMm - measurementStartMm) <= DEFAULT_FIXED_TOLERANCE_MM) {
    throw new Error('No measurement interval remains after establishing the sensor reserve');
  }

  // With a profile the sensor force follows the drive setpoint; otherwise it stays constant.
  let sensorCollectionForce = constantSensorForce;
  let measurementClockMs = null;
  const dataPoints = [];
  const validateMeasurementAngles = angles => {
    if (angles.length !== motorIds.length || !angles.every(Number.isFinite)) {
      throw new Error('Incomplete encoder readings cannot establish the sensor reserve');
    }
    settleOptions.validateAngles(angles);
    const force = resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce });
    const tested = reserveProbe.tested_drive_mm;
    const driveMm = angles[driveAnchor] * mmPerDeg[driveAnchor];
    if (sensorForceProfile && (driveMm < sensorForceProfile.drive_mm[0] - DEFAULT_FIXED_TOLERANCE_MM
        || driveMm > sensorForceProfile.drive_mm.at(-1) + DEFAULT_FIXED_TOLERANCE_MM)) {
      throw new Error('Measurement is outside the force profile support');
    }
    const locallyProbed = reserveProbe.accepted && Math.abs(force - reserveProbe.sensor_force_n) < 1e-9
      && driveMm >= Math.min(...tested) - DEFAULT_FIXED_TOLERANCE_MM
      && driveMm <= Math.max(...tested) + DEFAULT_FIXED_TOLERANCE_MM;
    if (!locallyProbed && sensorPayoutMm - angles[sensorAnchor] * mmPerDeg[sensorAnchor] < sensorReserveMm) {
      throw new Error('Sensor left the reserved half-turn interval before measurement');
    }
  };
  const recordPoint = (angles, driveSetpointMm, stepIndex, stepCount, extraFields = null) => {
    validateMeasurementAngles(angles);
    const lengths = angles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
    const point = {
      l_drive: lengths[driveAnchor],
      l_sensor: lengths[sensorAnchor],
      timestamp_ms: (measurementClockMs ?? sendFn.simulationClock?.now() ?? Date.now()) - datasetStartMs,
      clock_time_ms: measurementClockMs ?? sendFn.simulationClock?.now() ?? Date.now(),
      clock_source: sendFn.simulationClock ? 'simulation' : 'wall',
      raw_angles_deg: angles,
      drive_setpoint_mm: driveSetpointMm,
      step_index: stepIndex,
      step_count: stepCount,
      collection_attempt: options.collectionAttempt ?? 0,
      fixed_target_angles_deg: fixedTargetByAnchor.map((target, idx) => (
        Number.isFinite(target) ? target / mmPerDeg[idx] : null
      )),
      fixed_error_angles_deg: fixedTargetByAnchor.map((target, idx) => (
        Number.isFinite(target) ? angles[idx] - target / mmPerDeg[idx] : null
      )),
      sensor_force_n: resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce }),
      ...(extraAnchors.length ? { extra_sensor_ranges: extraAnchors.map(anchor => ({
        anchor, force_n: extraForce, length_mm: lengths[anchor], accepted: false,
      })) } : {}),
      spool_reserve_method: reserveProbe.accepted ? 'local_encoder_coupling' : 'half_turn',
    };
    if (extraFields && typeof extraFields === 'object') {
      Object.assign(point, extraFields);
    }
    dataPoints.push(point);
    return lengths;
  };

  const steps = Math.max(2, sweepPoints + 1);
  const stepCount = steps - 1;
  const remainingSteps = Math.max(1, stepCount - 1);
  const stepDelta = (driveStartPointMm - measurementStartMm) / remainingSteps;
  let currentDrive = measurementStartMm;
  let measurementPreloadActive = false;
  for (let stepIdx = 1; stepIdx < steps; stepIdx += 1) {
    // Later sweeps probe interior points in place, at each intended preload;
    // they do not need a half-turn-long arc in order to establish coupling.
    const target = probeSpoolReserve
      ? driveEndPointMm + totalDelta * stepIdx / (stepCount + 1)
      : measurementStartMm + stepDelta * (stepIdx - 1);
    const delta = target - currentDrive;
    if (sensorForceProfile) sensorCollectionForce = sensorForceAt(sensorForceProfile, target);
    let settledMeasurement = null;
    let settledNoiseStats = null;
    measurementClockMs = null;
    if (probeSpoolReserve || extraAnchors.length) {
      const force = forceAt(target);
      // The half-turn search already sampled the first target under this load.
      let sample = reserved && stepIdx === 1 ? reserved : await moveForReserve(target, force);
      reserveProbe = { accepted: false, reason: 'measured half-turn reserve' };
      const rewind = sensorPayoutMm - sample.muByMotorDeg[sensorAnchor] * mmPerDeg[sensorAnchor];
      const margin = 5 * Math.max(noiseSigmaFloorDeg, sample.sigmaByMotorDeg[sensorAnchor]) * mmPerDeg[sensorAnchor];
      if (probeSpoolReserve && rewind < sensorReserveMm + margin) {
        const radius = Math.min(initialStepMm, Math.abs(target - driveEndPointMm), Math.abs(target - driveStartPointMm)) / 2;
        const tested = await probeSpoolInterval(value => moveForReserve(value, force), {
          start: target, away: target + stepDirection * radius, toward: target - stepDirection * radius,
          driveAnchor, sensorAnchor, mmPerDeg, sigmaFloorDeg: noiseSigmaFloorDeg,
        });
        const { sample: returned, ...evidence } = tested;
        reserveProbe = { ...evidence, sensor_force_n: force };
        couplingProbes.push(reserveProbe);
        if (!reserveProbe.accepted) continue;
        sample = returned;
      }
      // No mode/preload change is allowed between verification and sampling.
      settledMeasurement = sample.stableData;
      settledNoiseStats = sample;
      measurementClockMs = sample.measuredClockMs;
    } else if (Number.isFinite(delta) && Math.abs(delta) > 1e-6) {
      if (!measurementPreloadActive) {
        // eslint-disable-next-line no-await-in-loop
        await applyDataPointPreloadModes(sendFn, {
          motorIds,
          driveAnchor,
          sensorAnchors: measurementSensors,
          fixedAnchors,
          forbiddenForceAnchors,
          forceMax,
          forceMin: forceMid,
          compliantForce: forceLow,
          forceMid,
          sensorCollectionForce,
          speedup,
          settleOptions,
        });
      }
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
      const settled = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
      // Moving the drive leaves the sensor's measurement force unchanged.
      if (measurementPreloadActive && !sensorForceProfile) settledMeasurement = settled;
    }
    // eslint-disable-next-line no-await-in-loop
    const collected = await collectDataPoint(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      driveAnchor,
      sensorAnchors: measurementSensors,
      fixedAnchors,
      forbiddenForceAnchors,
      forceMax,
      forceMin: forceMid,
      compliantForce: forceLow,
      forceMid,
      sensorCollectionForce,
      speedup,
      settleOptions,
      recordPoint,
      driveSetpointMm: target,
      stepIndex: stepIdx - 1,
      stepCount,
      settledMeasurement,
      settledNoiseStats,
      skipReturnModePrep: Number.isFinite(delta) && Math.abs(delta) > 1e-6,
      encoderNoiseOptions: {
        sampleCount: noiseSampleCount,
        sampleIntervalMs: noiseSampleIntervalMs,
        speedup,
      },
    });
    measurementPreloadActive = true;
    const noiseStats = collected?.noiseStats;
    if (noiseStats && dataPoints.length > 0) {
      const point = dataPoints[dataPoints.length - 1];
      point.mu = noiseStats.muByMotorDeg;
      point.sigma = noiseStats.sigmaByMotorDeg;
      point.sample_count = noiseStats.sampleCount;
      point.sampling_hz = noiseStats.samplingHz;
      if (Number.isFinite(noiseStats.durationMs)) {
        point.sample_duration_ms = noiseStats.durationMs;
        point.noise_clock_end_ms = sendFn.simulationClock?.now() ?? Date.now();
        point.noise_clock_start_ms = point.noise_clock_end_ms - noiseStats.durationMs;
      }
      if (Array.isArray(noiseStats.muByMotorDeg)) {
        validateMeasurementAngles(noiseStats.muByMotorDeg);
        point.fixed_error_angles_mu_deg = point.fixed_target_angles_deg.map((target, idx) => (
          Number.isFinite(target) ? noiseStats.muByMotorDeg[idx] - target : null
        ));
        const muLengths = noiseStats.muByMotorDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
        if (Number.isFinite(muLengths[driveAnchor])) {
          point.l_drive_mu = muLengths[driveAnchor];
        }
        if (Number.isFinite(muLengths[sensorAnchor])) {
          point.l_sensor_mu = muLengths[sensorAnchor];
        }
        for (const extra of point.extra_sensor_ranges ?? []) {
          extra.length_mu_mm = muLengths[extra.anchor];
          extra.accepted = spareRangeMoved(dataPoints, extra.anchor, noiseSigmaFloorDeg);
          extra.reason = extra.accepted ? 'encoder followed consecutive sweep steps' : 'insufficient resolved sweep motion';
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
    await options.onPoint?.(dataPoints.at(-1), sweepConfig);
    const lengths = collected?.lengths;
    if (Array.isArray(lengths)) {
      currentDrive = lengths[driveAnchor] ?? currentDrive;
    }
  }
  if (dataPoints.length < Math.min(5, sweepPoints)) {
    throw new UnmeasurableSweepError('Insufficient coupled measurement points in the observed sweep interval',
      { rawDriveInterval, retained_points: dataPoints.length, coupling_probes: couplingProbes });
  }
  const complianceProbe = await probeCompliance(sendFn, {
    motorIds, speedup, settleOptions, driveAnchor, sensorAnchor, fixedSet, forbidden,
    measurementSensorForces, forceLow, lastPoint: dataPoints.at(-1),
    noiseOptions: { sampleCount: noiseSampleCount, sampleIntervalMs: noiseSampleIntervalMs, speedup },
  });
  return { dataPoints, complianceProbe, driveRange: { start: driveEndPointMm, end: driveStartPointMm },
    spoolReserve: { sensor_payout_mm: sensorPayoutMm, required_sensor_rewind_mm: sensorReserveMm,
      raw_measured_drive_interval_mm: rawDriveInterval,
      guarded_sensor_rewind_mm: reserved ? sensorPayoutMm - reserved.muByMotorDeg[sensorAnchor] * mmPerDeg[sensorAnchor] : null,
      guarded_start_drive_mm: reserved ? guardedStartMm : null,
      first_drive_mm: dataPoints[0].drive_setpoint_mm, initial_half_turn_required: !probeSpoolReserve,
      coupling_probes: couplingProbes } };
}

// Research probe of line compliance: at the sweep's last pose, with every
// other line held, step the sensor force and record all encoders at each level.
// HP_SIM5_COMPLIANCE_PROBE lists the levels as multiples of the last point's force.
async function probeCompliance(sendFn, {
  motorIds, speedup, settleOptions, driveAnchor, sensorAnchor, fixedSet, forbidden,
  measurementSensorForces, forceLow, lastPoint, noiseOptions,
}) {
  const multiples = (process.env.HP_SIM5_COMPLIANCE_PROBE ?? '').split(',').filter(Boolean).map(Number);
  if (!multiples.length || !lastPoint) return null;
  const base = lastPoint.sensor_force_n;
  const levels = [{ force_n: base, mu_deg: lastPoint.mu ?? lastPoint.raw_angles_deg, sigma_deg: lastPoint.sigma ?? null }];
  for (const multiple of multiples) {
    const force = base * multiple;
    await applyForceModeState(sendFn, { motorIds, modes: motorIds.map((_, idx) => {
      if (idx === driveAnchor || fixedSet.has(idx) || forbidden.has(idx)) return 'position';
      return idx === sensorAnchor ? force : (measurementSensorForces[idx] ?? forceLow);
    }) });
    await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
    const sample = await sampleEncoderNoise(sendFn, motorIds, noiseOptions);
    levels.push({ force_n: force, mu_deg: sample.muByMotorDeg, sigma_deg: sample.sigmaByMotorDeg,
      clock_ms: sendFn.simulationClock?.now() ?? Date.now() });
  }
  console.log(`; compliance probe on sensor ${sensorAnchor}: ${levels.map(level => level.force_n.toFixed(2)).join(', ')} N`);
  return { drive_anchor: driveAnchor, sensor_anchor: sensorAnchor, levels };
}

export async function collectSweepData(send, context) {
  const input = validateSweepCollectionInput(context);
  const options = applySweepDefaults(input);
  const {
    machineType,
    machineConfig,
    firmware,
    klipperConfigPath,
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
  let { forceLow, forceMid, forceMax, sensorCollectionForce } = options;
  const outputFile = outputFileOverride || `sweep_data_${machineType}_${Date.now()}.json`;

  const sweepConfigs = await resolveSweepConfigs({
    sweepConfigFile: options.sweepConfigFile,
    machineType,
    machineConfig,
    maxSweepCount,
  });
  const sensorForceProfiles = options.sensorForceProfileFile
    ? await loadSensorForceProfiles(options.sensorForceProfileFile)
    : null;
  const profileModels = new Set((sensorForceProfiles ?? []).map(profile => profile.strain_model ?? 'legacy'));
  if (profileModels.size > 1) throw new Error('Cannot mix sensor force profile strain models');
  const profileModel = sensorForceProfiles ? [...profileModels][0] : null;

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
  const rejectedSweeps = [];
  const datasetStartMs = send.simulationClock?.now() ?? Date.now();

  const m666BeforeReply = await send('M666');
  const m666Before = parseM666(m666BeforeReply?.reply);
  const m669Reply = await send('M669');
  const m669Values = parseM669(m669Reply?.reply);
  const m92Reply = await send('M92');
  const m92Values = parseM92(m92Reply?.reply);
  // Moves land on whole motor steps, so position checks must allow one step.
  const stepMmByAnchor = machineConfig.axes.map(axis => (m92Values[axis] > 0 ? 1 / m92Values[axis] : 0));
  // RRF applies G90 to H2 moves as absolute motor coordinates; every H2 move here is a delta.
  if (firmware === 'rrf') await send('G91');

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
  await context.configureSimulator?.({
    lineLayeringEnabled: Boolean(context.args?.lineLayering || (Number.isFinite(Number(m666ForCollection?.Q)) && Number(m666ForCollection.Q) !== 0)),
  });
  let mmPerDeg = machineConfig.axes.map((_, idx) => computeMmPerDegree(m666ForCollection, idx));

  if (firmware === 'klipper' && mmPerDeg.some((value) => !Number.isFinite(value))) {
    const klipperConfigPathResolved = klipperConfigPath || DEFAULT_KLIPPY_CONFIG_PATH;
    const fallbackMmPerDeg = resolveKlipperMmPerDegreeFromConfig({
      motorIds,
      axes: machineConfig.axes,
      configPath: klipperConfigPathResolved,
    });
    mmPerDeg = mmPerDeg.map((value, idx) => (Number.isFinite(value) ? value : fallbackMmPerDeg[idx]));
  }

  const missingAxes = mmPerDeg
    .map((val, idx) => (Number.isFinite(val) ? null : machineConfig.axes[idx]))
    .filter(Boolean);
  if (missingAxes.length > 0) {
    console.warn(`Warning: missing mm/deg calibration for axes [${missingAxes.join(', ')}]; lengths will default to 0 on those axes.`);
  }
  attachDebugState(send, { mmPerDeg });

  //await primeEncoders(send, { motorIds, axes: machineConfig.axes });

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

  // Persist selected forces before sweep positioning, never as an auto-loaded cache.
  const forceTuning = {
    auto_tuned: autoTuneForce,
    method: autoTuneForce ? 'force-thresholds' : 'manual',
    tuned_at: autoTuneForce ? new Date().toISOString() : undefined,
    force_low_n: forceLow,
    force_mid_n: forceMid,
    force_max_n: forceMax,
    pullout_force_n: resolveCollectionForce({ forceMid, forceMax, preloadMultiplier: 20 }),
    ...(forceTuningMeta ?? {}),
  };
  const checkpointFile = `${outputFile}.force-tuning.json`;
  const checkpointStage = !autoTuneForce ? 'manual_force_selection_before_sweeps'
    : forceTuningMeta?.tuning_failed ? 'force_tuning_failed_fallback_selected'
      : 'force_tuning_complete_before_sweeps';
  await fs.mkdir(path.dirname(checkpointFile), { recursive: true });
  await fs.writeFile(`${checkpointFile}.tmp`, JSON.stringify({
    version: '1.0',
    stage: checkpointStage,
    timestamp: new Date().toISOString(),
    machine_type: machineType,
    firmware,
    m666: m666ForCollection,
    mm_per_degree: mmPerDeg,
    requested_sensor_collection_force_n: sensorCollectionForce,
    sensor_collection_force_n: resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce }),
    force_tuning: forceTuning,
  }, null, 2));
  await fs.rename(`${checkpointFile}.tmp`, checkpointFile);
  console.log(`; force-tuning checkpoint saved to ${checkpointFile}`);

  // A short tighten-release cycle before we set encoder reference points and set pos mode
  const forbiddenForceAnchors = getForceForbiddenAnchors(machineConfig);
  await applyForceModeState(send, {
    motorIds,
    modes: buildGlobalForceModes(motorIds, forceMid, forbiddenForceAnchors),
  });
  await waitForStableEncoders(send, motorIds, speedup);
  await applyForceModeState(send, {
    motorIds,
    modes: buildGlobalForceModes(motorIds, forceMid * 0.5, forbiddenForceAnchors),
  });
  await waitForStableEncoders(send, motorIds, speedup);
  await applyForceModeState(send, {
    motorIds,
    modes: buildGlobalForceModes(motorIds, forceLow, forbiddenForceAnchors),
  });
  await waitForStableEncoders(send, motorIds, speedup);
  //await primeEncoders(send, { motorIds, axes: machineConfig.axes });
  await applyForceModeState(send, {
    motorIds,
    modes: motorIds.map(() => 'position'),
  });
  await waitForStableEncoders(send, motorIds, speedup);

  const useFixedTargets = fixedTargetsSpec !== null
    || (sweepConfigs.length > 0 && sweepConfigs.every(cfg => cfg.fixedTargets));
  let maxTravelMm = Number.isFinite(maxTravelOverride) ? maxTravelOverride : null;
  let sizeTunePair = null;
  if (!useFixedTargets && maxTravelOverride === null) {
    sizeTunePair = selectSizeTunePair(sweepConfigs, getForceForbiddenAnchors(machineConfig), machineConfig.numAnchors);
    if (!sizeTunePair) {
      console.log('; size-tune skipped (no force-capable anchor pair)');
    } else {
      const measuredMaxTravelMm = await measureMaxTravelMm(send, {
        motorIds,
        mmPerDeg,
        forceLow,
        forceMax: resolveCollectionForce({ forceMid, forceMax, preloadMultiplier: 20 }),
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
      await returnMotorsToOriginOneAtATime(send, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        feed,
        speedup,
        midForce: forceLow,
        forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
        returnFirstAnchors: machineConfig.numAnchors > machineConfig.dimensions + 1 ? getForceForbiddenAnchors(machineConfig) : [],
      });
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
    let fixedTargets = resolveFixedTargets(
      sweepConfig.fixedAnchors,
      sweepConfig.fixedTargets ?? fixedTargetsSpec,
      maxTravelMm,
      machineConfig,
    );
    const fixedDesc = useFixedTargets
      ? fixedTargets.map((val) => (isNaturalTarget(val) ? `n${val.natural >= 0 ? '+' : ''}${val.natural}` : val.toFixed(3))).join(', ')
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
    const recoveryEvents = [];
    const directionForces = [];
    const spoolReserves = [];
    const complianceProbes = [];
    let rejected = false;
    for (let i = 0; i < subSweeps.length; i += 1) {
      const subCfg = subSweeps[i];
      console.log(
        `  Sub-sweep ${i + 1}/${subSweeps.length}: drive ${subCfg.driveAnchor}, sensor ${subCfg.sensorAnchor}`,
      );

      // Re-apply positioning for each sub-sweep to cover the full circle arc in both directions.
      // This ensures fixed anchors are retightened before changing drive/sensor roles.
      // eslint-disable-next-line no-await-in-loop
      let sweepResult;
      try {
        sweepResult = await collectWithSlipRecovery(send, async (forces, attempt) => {
          if (attempt > 0) {
            await returnMotorsToOriginOneAtATime(send, {
              motorIds, axes: machineConfig.axes, mmPerDeg, feed, speedup,
              midForce: forces.forceLow,
              forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
              returnFirstAnchors: machineConfig.numAnchors > machineConfig.dimensions + 1 ? getForceForbiddenAnchors(machineConfig) : [],
            });
          }
          // Both directions and every retry share the lengths the first positioning held.
          fixedTargets = await prepareSweepPositioning(send, subCfg, {
            motorIds,
            axes: machineConfig.axes,
            mmPerDeg,
            ...forces,
            forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
            fixedTargets,
            feed,
            speedup,
          });

          return performForceSweep(send, subCfg, {
            axes: machineConfig.axes,
            motorIds,
            sweepPoints,
            feed,
            speedup,
            mmPerDeg,
            stepMmByAnchor,
            datasetStartMs,
            ...forces,
            extraSensors: options.extraSensors,
            sensorForceProfile: sensorForceProfiles
              ? sensorForceProfileFor(sensorForceProfiles, subCfg.driveAnchor, subCfg.sensorAnchor,
                sweepConfig.fixedAnchors, fixedTargets)
              : null,
            collectionAttempt: attempt,
            noiseSampleCount,
            noiseSampleIntervalMs,
            noiseMinSamples,
            noiseSigmaFloorDeg,
            fixedAnchors: sweepConfig.fixedAnchors,
            fixedTargets,
            forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
            onPoint: context.onPoint,
            baseRadiusMm: Array.isArray(m666ForCollection?.R)
              ? m666ForCollection.R[subCfg.driveAnchor]
              : m666ForCollection?.R,
            sensorSpoolGearRatio: (Array.isArray(m666ForCollection?.H) ? m666ForCollection.H[subCfg.sensorAnchor] : m666ForCollection?.H ?? 1)
              / (Array.isArray(m666ForCollection?.L) ? m666ForCollection.L[subCfg.sensorAnchor] : m666ForCollection?.L ?? 1),
            probeSpoolReserve: options.completedSweeps >= 3,
          });
        }, {
          motorIds, forceLow, forceMid, forceMax, sensorCollectionForce,
          onRecovery: recovery => context.onRecovery?.({ ...recovery, sweep: subCfg, direction: i }),
        });
      } catch (error) {
        if (!(error instanceof UnmeasurableSweepError) || options.completedSweeps < 3) throw error;
        await applyForceModeState(send, { motorIds, modes: motorIds.map(() => 'position') });
        await returnMotorsToOriginOneAtATime(send, {
          motorIds, axes: machineConfig.axes, mmPerDeg, feed, speedup, midForce: forceLow,
          forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
          returnFirstAnchors: machineConfig.numAnchors > machineConfig.dimensions + 1 ? getForceForbiddenAnchors(machineConfig) : [],
        });
        rejectedSweeps.push({ fixed_anchors: sweepConfig.fixedAnchors, fixed_lengths: fixedTargets,
          drive_anchor: sweepConfig.driveAnchor, sensor_anchor: sweepConfig.sensorAnchor,
          direction: i, reason: error.message, evidence: error.evidence });
        console.log(`; rejected unmeasurable sweep after returning to origin: ${error.message}`);
        rejected = true;
        break;
      }
      ({ forceMid, forceMax } = sweepResult.forces);
      sensorCollectionForce = sweepResult.forces.sensorCollectionForce;
      recoveryEvents.push(...sweepResult.recoveries.map(event => ({ ...event, direction: i })));
      spoolReserves.push({ direction: i, ...sweepResult.spoolReserve });
      if (sweepResult.complianceProbe) complianceProbes.push({ direction: i, ...sweepResult.complianceProbe });
      directionForces.push({ drive_anchor: subCfg.driveAnchor, sensor_anchor: subCfg.sensorAnchor,
        sensor_force_n: resolveCollectionForce(sweepResult.forces),
        pullout_force_n: resolveCollectionForce({ forceMid, forceMax, preloadMultiplier: 20 }) });
      const remapped = remapDataPointsToCanonical(
        sweepResult.dataPoints,
        sweepConfig,
        subCfg,
      );
      aggregatedPoints.push(...remapped);
    }

    if (rejected) continue;

    // Each point's fixed_error_angles_deg is measured from these targets, so a
    // held line's length is target + error. A later encoder read would count
    // the line's drift during the sweep twice.
    const fixedLengths = sweepConfig.fixedAnchors.map((_, anchorIdx) => (
      Number.isFinite(fixedTargets[anchorIdx]) ? fixedTargets[anchorIdx] : 0));

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
        sensor_force_n: resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce }),
        sensor_force_profile: sensorForceProfiles !== null,
        sensor_force_profile_model: profileModel,
        strand_tension_n: sensorForceProfiles?.[0]?.strand_tension_n,
        direction_forces: directionForces,
        slip_recoveries: recoveryEvents,
        spool_reserves: spoolReserves,
        ...(complianceProbes.length ? { compliance_probes: complianceProbes } : {}),
        settle_ms: settleMs,
      },
    });
    console.log(
      `  Collected ${aggregatedPoints.length} points from ${subSweeps.length} sub-sweep${subSweeps.length === 1 ? '' : 's'}`,
    );
    if (returnToOrigin && sweepOrdinal < totalPlannedSweeps) {
      await returnMotorsToOriginOneAtATime(send, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        feed,
        speedup,
        ...(machineConfig.numAnchors > machineConfig.dimensions + 1 ? {
          midForce: forceLow, forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
          returnFirstAnchors: getForceForbiddenAnchors(machineConfig),
        } : {}),
      });
      console.log('Returned all motors to encoder origin (between sweeps).');
    }
  }

  // Recovery may change collection forces after the initial tuning checkpoint.
  Object.assign(forceTuning, {
    force_low_n: forceLow,
    force_mid_n: forceMid,
    force_max_n: forceMax,
    pullout_force_n: resolveCollectionForce({ forceMid, forceMax, preloadMultiplier: 20 }),
  });
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
      extra_sensors_enabled: options.extraSensors,
      noise_model: { project_zero_tension: false },
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
    acquisition_rejections: rejectedSweeps,
  };

  const outputDir = path.dirname(outputFile);
  if (outputDir && outputDir !== '.') {
    await fs.mkdir(outputDir, { recursive: true });
  }
  await fs.writeFile(outputFile, JSON.stringify(dataset, null, 2));
  console.log(`\nSaved ${sweeps.length} sweeps to ${outputFile}`);

  if (returnToOrigin) {
    await returnMotorsToOriginOneAtATime(send, {
      motorIds,
      axes: machineConfig.axes,
      mmPerDeg,
      feed,
      speedup,
      ...(machineConfig.numAnchors > machineConfig.dimensions + 1 ? {
        midForce: forceLow, forbiddenForceAnchors: getForceForbiddenAnchors(machineConfig),
        returnFirstAnchors: getForceForbiddenAnchors(machineConfig),
      } : {}),
    });
    console.log('Returned all motors to encoder origin.');
  }

  return { sweeps, outputFile, collectionOptions: options };
}
