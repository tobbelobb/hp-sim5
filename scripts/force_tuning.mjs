import { parseEncoderReply, runMoveWithWait, sleep as baseSleep } from './encoder_utils.mjs';
import {
  angleToLength,
  applyForceModeState,
  getCurrentLengths,
  primeEncoders,
  returnMotorsToOriginAllAtOnce,
  returnMotorsToOriginOneAtATime,
  waitForStableEncoders,
} from './uncalibrated_actions.mjs';

/**
 * Force tuning input reference (shared by validation/default helpers).
 * motorIds: array of motor ID strings, length > 0 when talking to hardware.
 * axes: array of axis letters aligned with motorIds.
 * mmPerDeg: array of mm/deg values aligned with motorIds.
 * feed: mm/min feed rate for return moves.
 * speedup: positive number, hp-sim speed scale.
 * forbiddenForceAnchors: anchor indices that must stay in position mode.
 * activeAnchor/fixedAnchor: anchor indices for force trials.
 * restAnchors: anchor indices expected to move during trials.
 * idleForce/testForce/baseLow/capForceLimit: force values in N.
 * sampleDurationMs/sampleIntervalMs/sampleWindowMs: timing controls in ms.
 * rampForces/rampStepWaitMs/rebaselineAfterRamp: optional force ramp config.
 * thresholds: movement thresholds from buildMovementThresholds.
 * forceLow/forceMid/forceMax: fallback force levels for tuneForce.
 * forceMaxProvided: boolean, true when forceMax is user-supplied.
 * waitForStall/stallTimeoutMs: trial stopping parameters.
 * forceStart: starting force for edge search.
 * bracketFactor/maxBracketSteps/...: edge-force search tuning.
 */

const DEFAULT_FEED = 3000;
const DEFAULT_FORCE_LOW_N = 0.01;
const DEFAULT_FORCE_MID_N = 0.1;
const DEFAULT_FORCE_MAX_N = 1.0;

const AUTO_TUNE_MIN_FORCE_N = 0.01;
const AUTO_TUNE_MAX_FORCE_N = 20.0;
const AUTO_TUNE_SAMPLE_WINDOW_MS = 10000;
const AUTO_TUNE_NOISE_SAMPLE_MS = 4000;
const AUTO_TUNE_NOISE_SAMPLE_INTERVAL_MS = 200;
const AUTO_TUNE_SAMPLE_INTERVAL_MS = 500;
const AUTO_TUNE_STALL_WINDOW_MS = 2500;
const AUTO_TUNE_MIN_STALL_SPEED_DEG_PER_SEC = 0.05;
const AUTO_TUNE_BRACKET_FACTOR = 1.4;
const AUTO_TUNE_MAX_BRACKET_STEPS = 120;
const AUTO_TUNE_MAX_BISECT_STEPS = 60;
const AUTO_TUNE_RELATIVE_TOLERANCE = 0.1;
const AUTO_TUNE_ABSOLUTE_TOLERANCE = 0.01;
const AUTO_TUNE_EDGE_RATIO = 0.95;
const AUTO_TUNE_IDLE_FORCE_RATIO = 0.05;

export const FORCE_TUNING_DEFAULTS = {
  DEFAULT_FORCE_LOW_N,
  DEFAULT_FORCE_MID_N,
  DEFAULT_FORCE_MAX_N,
};

function clampAutoTuneForce(value) {
  if (!Number.isFinite(value)) {
    return null;
  }
  return Math.min(AUTO_TUNE_MAX_FORCE_N, Math.max(AUTO_TUNE_MIN_FORCE_N, value));
}

function computeMedian(values) {
  if (!Array.isArray(values)) {
    return 0;
  }
  const filtered = values.filter((v) => Number.isFinite(v)).sort((a, b) => a - b);
  if (filtered.length === 0) {
    return 0;
  }
  const mid = Math.floor(filtered.length / 2);
  if (filtered.length % 2 === 1) {
    return filtered[mid];
  }
  return 0.5 * (filtered[mid - 1] + filtered[mid]);
}

export function computeStallSpeedThresholdDegPerSec(sigmaActDeg, sampleIntervalSec) {
  const interval = Number.isFinite(sampleIntervalSec) && sampleIntervalSec > 0 ? sampleIntervalSec : null;
  const noiseSpeed = Number.isFinite(sigmaActDeg) && interval
    ? (6 * sigmaActDeg) / interval
    : 0;
  return Math.max(AUTO_TUNE_MIN_STALL_SPEED_DEG_PER_SEC, noiseSpeed);
}

// Build thresholds for what should be considered a significant movement.
// Basically just avoids having to hard code 0.5 deg as the limit for what counts as a movement.
// It's more portable between machines this way.
export function buildMovementThresholds(noiseSigmaDeg, { activeAnchor, restAnchors = [] } = {}) {
  const sigmaAct = Array.isArray(noiseSigmaDeg) ? noiseSigmaDeg[activeAnchor] : 0;
  // require at least 6σ of active-motor change (with a hard floor of 0.5°).
  const thetaActThr = Math.max(0.5, 6 * (Number.isFinite(sigmaAct) ? sigmaAct : 0));
  // require at least 4σ of residual change (floor 0.3°) after releasing force and settling.
  const thetaResThr = Math.max(0.3, 4 * (Number.isFinite(sigmaAct) ? sigmaAct : 0));

  // The other (non active) motors are also required to have moved for us to record "movement occured".
  const thetaOtherByAnchor = new Map();
  const restSigmas = [];
  for (const anchorIdx of restAnchors) {
    const sigma = Array.isArray(noiseSigmaDeg) ? noiseSigmaDeg[anchorIdx] : 0;
    const thr = Math.max(0.5, 6 * (Number.isFinite(sigma) ? sigma : 0));
    thetaOtherByAnchor.set(anchorIdx, thr);
    restSigmas.push(Number.isFinite(sigma) ? sigma : 0);
  }
  // after releasing and settling, the sum of residual motion across the rest must be significant,
  // scaled by typical noise (median σ).
  const medianRestSigma = computeMedian(restSigmas);
  const sumResidualThr = Math.max(0.8, 6 * medianRestSigma);

  return {
    sigmaAct,
    medianRestSigma,
    thetaActThr,
    thetaResThr,
    thetaOtherByAnchor,
    sumResidualThr,
  };
}

export function buildForceRampValues(startForce, endForce, factor = AUTO_TUNE_BRACKET_FACTOR) {
  const start = Number.isFinite(startForce) ? startForce : 0;
  const end = Number.isFinite(endForce) ? endForce : start;
  if (!Number.isFinite(end) || end <= 0) {
    return [];
  }
  const stepFactor = Number.isFinite(factor) && factor > 1 ? factor : AUTO_TUNE_BRACKET_FACTOR;
  let current = Math.max(start, AUTO_TUNE_MIN_FORCE_N);
  if (end <= current + 1e-12) {
    return [end];
  }
  const values = [];
  while (current < end - 1e-12) {
    values.push(current);
    const next = current * stepFactor;
    if (!(next > current + 1e-12)) {
      break;
    }
    current = Math.min(next, end);
  }
  if (values.length === 0 || Math.abs(values[values.length - 1] - end) > 1e-12) {
    values.push(end);
  }
  return values;
}

export async function findMinimumMovingForce(sendFn, options = {}) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    feed = DEFAULT_FEED,
    speedup,
    baseLow = DEFAULT_FORCE_LOW_N,
    capForceLimit = AUTO_TUNE_MAX_FORCE_N,
    trialFn = null,
    returnToOriginFn = returnMotorsToOriginAllAtOnce,
    maxBracketSteps = AUTO_TUNE_MAX_BRACKET_STEPS,
    maxBisectSteps = AUTO_TUNE_MAX_BISECT_STEPS,
    absTolerance = AUTO_TUNE_ABSOLUTE_TOLERANCE,
    relTolerance = AUTO_TUNE_RELATIVE_TOLERANCE,
  } = options;

  if (typeof trialFn !== 'function') {
    throw new Error('findMinimumMovingForce requires a trialFn');
  }

  const runTrial = async (force, label, trialOptions = {}) => trialFn(force, label, trialOptions);

  let testForce = clampAutoTuneForce(AUTO_TUNE_MIN_FORCE_N) ?? baseLow;
  if (testForce > capForceLimit) {
    testForce = capForceLimit;
  }
  let lastNoMoveForce = null;
  let firstMoveForce = null;

  for (let i = 0; i < maxBracketSteps; i += 1) {
    const result = await runTrial(testForce, `probe ${i + 1}/${maxBracketSteps}`);
    if (result.moved) {
      firstMoveForce = testForce;
      break;
    }
    lastNoMoveForce = testForce;
    const nextForceRaw = testForce * AUTO_TUNE_BRACKET_FACTOR;
    let nextForce = clampAutoTuneForce(nextForceRaw) ?? nextForceRaw;
    if (nextForce > capForceLimit) {
      nextForce = capForceLimit;
    }
    if (!Number.isFinite(nextForce) || nextForce <= testForce + 1e-12) {
      break;
    }
    testForce = nextForce;
  }

  if (!Number.isFinite(firstMoveForce)) {
    return { forceStart: null, lastNoMoveForce, firstMoveForce };
  }

  let low = Number.isFinite(lastNoMoveForce) ? lastNoMoveForce : 0;
  let high = firstMoveForce;

  for (let i = 0; i < maxBisectSteps; i += 1) {
    const width = high - low;
    if (width <= absTolerance || width / Math.max(high, 1e-6) <= relTolerance) {
      break;
    }
    const midRaw = 0.5 * (low + high);
    const mid = clampAutoTuneForce(midRaw) ?? midRaw;
    const result = await runTrial(mid, `bisect-start ${i + 1}/${maxBisectSteps}`);
    if (result.moved) {
      high = mid;
    } else {
      low = mid;
    }
  }

  const forceStart = high;
  if (Array.isArray(axes) && Array.isArray(mmPerDeg) && Array.isArray(motorIds)) {
    await returnToOriginFn(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
    });
  }

  return { forceStart, lastNoMoveForce, firstMoveForce };
}

async function setForceTrialModes(sendFn, motorIds, options = {}) {
  const {
    activeAnchor = null,
    fixedAnchor = null,
    idleForce = DEFAULT_FORCE_LOW_N,
    activeForce = null,
    forbiddenForceAnchors = [],
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return;
  }
  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const idle = Number.isFinite(idleForce) ? idleForce : DEFAULT_FORCE_LOW_N;
  const active = Number.isFinite(activeForce) ? activeForce : idle;
  const modes = motorIds.map((_, idx) => {
    if (idx === fixedAnchor || forbidden.has(idx)) {
      return 'position';
    }
    if (idx === activeAnchor) {
      return active;
    }
    return idle;
  });
  await applyForceModeState(sendFn, { motorIds, modes });
}

export async function calibrateEncoderNoise(sendFn, options = {}) {
  const {
    motorIds,
    fixedAnchor = null,
    idleForce = DEFAULT_FORCE_LOW_N,
    speedup,
    sampleDurationMs = AUTO_TUNE_NOISE_SAMPLE_MS,
    sampleIntervalMs = AUTO_TUNE_NOISE_SAMPLE_INTERVAL_MS,
    forbiddenForceAnchors = [],
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return { sigmaByMotorDeg: [], samples: 0, durationMs: 0 };
  }
  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const durationMs = Math.max(1, sampleDurationMs / timeScale);
  const intervalMs = Math.max(10, sampleIntervalMs / timeScale);
  const sampleCount = Math.max(3, Math.floor(durationMs / intervalMs));

  await setForceTrialModes(sendFn, motorIds, {
    activeAnchor: null,
    fixedAnchor,
    idleForce,
    activeForce: idleForce,
    forbiddenForceAnchors,
  });
  await waitForStableEncoders(sendFn, motorIds, speedup);

  const sums = Array.from({ length: motorIds.length }, () => 0);
  const sumsSq = Array.from({ length: motorIds.length }, () => 0);
  let samples = 0;

  for (let i = 0; i < sampleCount; i += 1) {
    // eslint-disable-next-line no-await-in-loop
    const reply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const angles = parseEncoderReply(reply?.reply);
    if (angles.length === motorIds.length && angles.every((v) => Number.isFinite(v))) {
      for (let idx = 0; idx < angles.length; idx += 1) {
        const v = angles[idx];
        sums[idx] += v;
        sumsSq[idx] += v * v;
      }
      samples += 1;
    }
    // eslint-disable-next-line no-await-in-loop
    await baseSleep(intervalMs);
  }

  const sigmaByMotorDeg = sums.map((sum, idx) => {
    if (samples <= 0) {
      return 0;
    }
    const mean = sum / samples;
    const variance = (sumsSq[idx] / samples) - mean * mean;
    return Math.sqrt(Math.max(0, variance));
  });

  return { sigmaByMotorDeg, samples, durationMs };
}

export async function runForceTrial(sendFn, options = {}) {
  const {
    motorIds,
    activeAnchor,
    fixedAnchor = null,
    restAnchors = [],
    idleForce = DEFAULT_FORCE_LOW_N,
    testForce = DEFAULT_FORCE_LOW_N,
    speedup,
    sampleWindowMs = AUTO_TUNE_SAMPLE_WINDOW_MS,
    sampleIntervalMs = AUTO_TUNE_SAMPLE_INTERVAL_MS,
    rampForces = null,
    rampStepWaitMs = 0,
    rebaselineAfterRamp = false,
    stallWindowMs = AUTO_TUNE_STALL_WINDOW_MS,
    stallSpeedDegPerSec = null,
    waitForStall = false,
    stallTimeoutMs = null,
    thresholds = null,
    axes,
    mmPerDeg,
    feed = DEFAULT_FEED,
    forbiddenForceAnchors = [],
  } = options;

  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return {
      moved: false,
      travelDeg: 0,
      stalled: false,
      deltaEndDeg: [],
      deltaResidualDeg: [],
    };
  }
  if (!Number.isFinite(activeAnchor) || activeAnchor < 0 || activeAnchor >= motorIds.length) {
    return {
      moved: false,
      travelDeg: 0,
      stalled: false,
      deltaEndDeg: [],
      deltaResidualDeg: [],
    };
  }

  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const windowMs = Math.max(1, sampleWindowMs / timeScale);
  const intervalMs = Math.max(20, sampleIntervalMs / timeScale);
  const stallWindowScaledMs = Math.max(intervalMs, stallWindowMs / timeScale);
  const sampleIntervalSec = intervalMs / 1000;
  const maxWindowRawMs = Number.isFinite(stallTimeoutMs) && stallTimeoutMs > 0
    ? stallTimeoutMs
    : (waitForStall ? sampleWindowMs * 3 : sampleWindowMs);
  const maxWindowMs = Math.max(windowMs, maxWindowRawMs / timeScale);
  const stopAfterMs = waitForStall ? maxWindowMs : windowMs;
  const speedThreshold = Number.isFinite(stallSpeedDegPerSec) && stallSpeedDegPerSec > 0
    ? stallSpeedDegPerSec
    : computeStallSpeedThresholdDegPerSec(thresholds?.sigmaAct ?? 0, sampleIntervalSec);

  await setForceTrialModes(sendFn, motorIds, {
    activeAnchor,
    fixedAnchor,
    idleForce,
    activeForce: idleForce,
    forbiddenForceAnchors,
  });
  const stableStart = await waitForStableEncoders(sendFn, motorIds, speedup);
  let startAngles = stableStart.anglesDeg;

  const rampWaitMs = Math.max(0, rampStepWaitMs / timeScale);
  if (Array.isArray(rampForces) && rampForces.length > 0) {
    for (let idx = 0; idx < rampForces.length; idx += 1) {
      const force = rampForces[idx];
      if (!Number.isFinite(force)) {
        continue;
      }
      await sendFn(`M569.4 P${motorIds[activeAnchor]} T${force}`);
      if (rampWaitMs > 0) {
        // eslint-disable-next-line no-await-in-loop
        await baseSleep(rampWaitMs);
      }
    }
    const lastForce = rampForces[rampForces.length - 1];
    if (!Number.isFinite(lastForce) || Math.abs(lastForce - testForce) > 1e-12) {
      await sendFn(`M569.4 P${motorIds[activeAnchor]} T${testForce}`);
    }
  } else {
    await setForceTrialModes(sendFn, motorIds, {
      activeAnchor,
      fixedAnchor,
      idleForce,
      activeForce: testForce,
      forbiddenForceAnchors,
    });
  }

  if (rebaselineAfterRamp) {
    const reply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const angles = parseEncoderReply(reply?.reply);
    if (angles.length === motorIds.length && angles.every((v) => Number.isFinite(v))) {
      startAngles = angles;
    }
  }

  let lastAngles = startAngles;
  let endAngles = startAngles;
  let lastMs = Date.now();
  const startMs = lastMs;
  let stallDurationMs = 0;
  let stalled = false;
  let stallAngle = null;

  while (Date.now() - startMs < stopAfterMs) {
    // eslint-disable-next-line no-await-in-loop
    await baseSleep(intervalMs);
    // eslint-disable-next-line no-await-in-loop
    const reply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const angles = parseEncoderReply(reply?.reply);
    const nowMs = Date.now();
    if (angles.length === motorIds.length && angles.every((v) => Number.isFinite(v))) {
      endAngles = angles;
      const dtSec = Math.max(1e-6, (nowMs - lastMs) / 1000);
      const prevAngle = lastAngles?.[activeAnchor];
      const curAngle = angles?.[activeAnchor];
      if (Number.isFinite(prevAngle) && Number.isFinite(curAngle)) {
        const speed = Math.abs(curAngle - prevAngle) / dtSec;
        if (speed < speedThreshold) {
          stallDurationMs += (nowMs - lastMs);
        } else {
          stallDurationMs = 0;
        }
        if (!stalled && stallDurationMs >= stallWindowScaledMs) {
          stalled = true;
          stallAngle = curAngle;
        }
      }
      lastAngles = angles;
    }
    lastMs = nowMs;
    if (waitForStall && stalled) {
      break;
    }
  }

  await setForceTrialModes(sendFn, motorIds, {
    activeAnchor,
    fixedAnchor,
    idleForce,
    activeForce: idleForce,
    forbiddenForceAnchors,
  });
  const stableResidual = await waitForStableEncoders(sendFn, motorIds, speedup);
  const residualAngles = stableResidual.anglesDeg;

  const deltaEndDeg = endAngles.map((angle, idx) => angle - (startAngles[idx] ?? 0));
  const deltaResidualDeg = residualAngles.map((angle, idx) => angle - (startAngles[idx] ?? 0));
  const activeDelta = deltaEndDeg[activeAnchor] ?? 0;
  const activeResidual = deltaResidualDeg[activeAnchor] ?? 0;

  const thetaActThr = thresholds?.thetaActThr ?? 0.5;
  const thetaResThr = thresholds?.thetaResThr ?? 0.3;
  const sumResidualThr = thresholds?.sumResidualThr ?? 0.8;
  const thetaOtherByAnchor = thresholds?.thetaOtherByAnchor ?? new Map();

  const activeMoved = Math.abs(activeDelta) >= thetaActThr;
  let restMovedCount = 0;
  let restResidualSum = 0;
  for (const anchorIdx of restAnchors) {
    const delta = deltaEndDeg[anchorIdx] ?? 0;
    const thrOther = thetaOtherByAnchor.get(anchorIdx) ?? 0.5;
    if (Math.abs(delta) >= thrOther) {
      restMovedCount += 1;
    }
    restResidualSum += Math.abs(deltaResidualDeg[anchorIdx] ?? 0);
  }
  const residualActiveOk = Math.abs(activeResidual) >= thetaResThr;
  const residualRestOk = restResidualSum >= sumResidualThr;

  const moved = restAnchors.length > 0
    ? (activeMoved && restMovedCount >= 1 && residualActiveOk && residualRestOk)
    : false;

  const travelDeg = Math.abs(
    stalled && Number.isFinite(stallAngle)
      ? stallAngle - (startAngles[activeAnchor] ?? 0)
      : activeDelta,
  );

  if (Array.isArray(axes) && Array.isArray(mmPerDeg) && travelDeg > 2 * thetaActThr) {
    await returnMotorsToOriginOneAtATime(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
      midForce: idleForce,
      fixedAnchors: [fixedAnchor],
      forbiddenForceAnchors,
    });
    await setForceTrialModes(sendFn, motorIds, {
      activeAnchor,
      fixedAnchor,
      idleForce,
      activeForce: idleForce,
      forbiddenForceAnchors,
    });
  }

  return {
    moved,
    travelDeg,
    stalled,
    deltaEndDeg,
    deltaResidualDeg,
  };
}

export async function findEdgeForce(sendFn, options = {}) {
  const {
    forceStart,
    capForceLimit,
    trialFn,

    // Ramp / saturation detection
    bracketFactor = 1.5,                // multiply force each step
    maxBracketSteps = 12,               // safety
    saturationRelTol = 0.10,            // "within 10%"
    minUsefulTravelDeg = 0.5,           // ignore tiny near-noise numbers
    requireMoved = false,               // optional: only accept points where moved=true

    // Curve fit options
    fitEnabled = true,
    fitGridK = [0.5, 1, 2, 4, 8],       // slope candidates in log-space
    fitGridX0Steps = 25,                // coarse grid for x0
    fitRefineIters = 2,                 // refine around best
    fitRefineFactor = 0.5,              // narrower window each refine

    // Optional extra stop: plateau after N stable steps
    plateauStepsRequired = 2,           // 1 means "2 successive within tol" (your default)
  } = options;

  if (typeof trialFn !== 'function') {
    throw new Error('findEdgeForce requires a trialFn');
  }
  if (!Number.isFinite(forceStart) || forceStart <= 0) {
    return { forceEdge: null, dMax: null, reason: 'invalid forceStart', samples: [] };
  }
  const cap = Number.isFinite(capForceLimit) && capForceLimit > 0 ? capForceLimit : Infinity;

  const runTrial = async (force, label) => {
    const res = await trialFn(force, label);
    const travelDeg = Number.isFinite(res?.travelDeg) ? res.travelDeg : NaN;
    const moved = !!res?.moved;
    return { ...res, travelDeg, moved };
  };

  const samples = [];
  const addSample = (force, res) => {
    samples.push({
      force,
      travelDeg: res.travelDeg,
      moved: res.moved,
      stalled: !!res.stalled,
    });
  };

  // --- 1) Bracket saturation safely by ramping up ---
  let F = forceStart;
  if (F > cap) F = cap;

  let lastAccepted = null;
  let stableCount = 0;
  let saturationPair = null;

  for (let i = 0; i < maxBracketSteps; i += 1) {
    const res = await runTrial(F, `edge-ramp ${i + 1}/${maxBracketSteps}`);
    addSample(F, res);

    const D = res.travelDeg;
    const ok =
      Number.isFinite(D) &&
      D >= minUsefulTravelDeg &&
      (!requireMoved || res.moved);

    // If travel isn't meaningful, keep ramping (but don't let it loop forever).
    if (!ok) {
      if (F >= cap - 1e-12) break;
      const nextRaw = F * bracketFactor;
      let next = nextRaw;
      if (next > cap) next = cap;
      if (!Number.isFinite(next) || next <= F + 1e-12) break;
      F = next;
      continue;
    }

    if (lastAccepted) {
      const Dprev = lastAccepted.travelDeg;
      const Dcur = D;
      const denom = Math.max(Math.abs(Dcur), Math.abs(Dprev), 1e-9);
      const relDiff = Math.abs(Dcur - Dprev) / denom;

      if (relDiff <= saturationRelTol) {
        stableCount += 1;
      } else {
        stableCount = 0;
      }

      if (stableCount >= plateauStepsRequired) {
        // Saturation found: (previous, current) is our "within 10%" pair.
        saturationPair = {
          lowForce: lastAccepted.force,
          highForce: F,
          lowTravelDeg: Dprev,
          highTravelDeg: Dcur,
          relDiff,
        };
        break;
      }
    }

    lastAccepted = { force: F, travelDeg: D };

    if (F >= cap - 1e-12) {
      break;
    }

    const nextRaw = F * bracketFactor;
    let next = nextRaw;
    if (next > cap) next = cap;
    if (!Number.isFinite(next) || next <= F + 1e-12) break;
    F = next;

  }

  if (!saturationPair) {
    // Could not detect a plateau safely.
    // Fall back: pick best observed force by "largest travel at lowest force".
    // This is conservative: chooses lowest force among points close to max travel.
    const valid = samples
      .filter((s) => Number.isFinite(s.travelDeg) && s.travelDeg >= minUsefulTravelDeg)
      .sort((a, b) => a.force - b.force);

    if (valid.length === 0) {
      return {
        forceEdge: null,
        dMax: null,
        reason: 'no valid travel measurements during ramp',
        samples,
      };
    }

    const maxD = Math.max(...valid.map((s) => s.travelDeg));
    const target = (1 - saturationRelTol) * maxD;
    const best = valid.find((s) => s.travelDeg >= target) ?? valid[valid.length - 1];

    const dMaxFallback = maxD;

    return {
      forceEdge: best.force,
      dMax: dMaxFallback,
      reason: 'no plateau detected; conservative fallback from best observed',
      samples,
      saturation: null,
      fit: null,
    };
  }

  // Required by you: choose LOWER of the two plateau forces as preferred edge.
  const forceEdge = saturationPair.lowForce;

  // --- 2) Fit S-curve to estimate dMax (optional) ---
  let fit = null;
  let dMax = Math.max(saturationPair.lowTravelDeg, saturationPair.highTravelDeg);

  if (fitEnabled) {
    fit = fitLogisticInLogForce(samples, {
      minUsefulTravelDeg,
      requireMoved,
      fitGridK,
      fitGridX0Steps,
      fitRefineIters,
      fitRefineFactor,
    });
    if (fit?.ok && Number.isFinite(fit.dMax) && fit.dMax > 0) {
      dMax = fit.dMax;
    }
  }

  return {
    forceEdge,
    dMax,
    reason: 'plateau detected by successive-travel tolerance',
    samples,
    saturation: saturationPair,
    fit,
  };
}

/**
 * Fit logistic curve in log-force space:
 *   R(F) = dMax / (1 + exp(-k*(ln(F) - x0)))
 * using grid-search over (k, x0) and closed-form dMax for each (k, x0).
 *
 * Returns {ok, dMax, k, x0, rmse, nUsed}.
 */
function fitLogisticInLogForce(samples, opts = {}) {
  const {
    minUsefulTravelDeg = 0.5,
    requireMoved = false,
    fitGridK = [0.5, 1, 2, 4, 8],
    fitGridX0Steps = 25,
    fitRefineIters = 2,
    fitRefineFactor = 0.5,
  } = opts;

  const pts = samples
    .filter((s) => Number.isFinite(s.force) && s.force > 0 && Number.isFinite(s.travelDeg))
    .filter((s) => s.travelDeg >= minUsefulTravelDeg)
    .filter((s) => (!requireMoved || s.moved));

  if (pts.length < 3) {
    return { ok: false, reason: 'not enough points for fit', nUsed: pts.length };
  }

  // Build arrays in log-force.
  const xs = pts.map((p) => Math.log(p.force));
  const ys = pts.map((p) => p.travelDeg);

  // Define search range for x0 based on observed log-forces.
  let xMin = Math.min(...xs);
  let xMax = Math.max(...xs);

  let best = null;

  const evalCandidate = (k, x0) => {
    // For fixed k,x0, the model is y ≈ dMax * s_i, where s_i = logistic(...)
    // Best dMax in least squares: dMax = (Σ s_i*y_i)/(Σ s_i^2)
    const s = xs.map((x) => 1 / (1 + Math.exp(-k * (x - x0))));
    let num = 0;
    let den = 0;
    for (let i = 0; i < s.length; i += 1) {
      num += s[i] * ys[i];
      den += s[i] * s[i];
    }
    if (den <= 1e-12) return null;
    const dMax = num / den;

    // Penalize non-physical fits.
    if (!Number.isFinite(dMax) || dMax <= 0) return null;

    let err2 = 0;
    for (let i = 0; i < s.length; i += 1) {
      const yHat = dMax * s[i];
      const e = ys[i] - yHat;
      err2 += e * e;
    }
    const rmse = Math.sqrt(err2 / Math.max(1, s.length));
    return { dMax, k, x0, rmse, nUsed: s.length };
  };

  const gridSearch = (x0Lo, x0Hi) => {
    for (const k of fitGridK) {
      for (let j = 0; j <= fitGridX0Steps; j += 1) {
        const t = j / fitGridX0Steps;
        const x0 = x0Lo + t * (x0Hi - x0Lo);
        const cand = evalCandidate(k, x0);
        if (!cand) continue;
        if (!best || cand.rmse < best.rmse) {
          best = cand;
        }
      }
    }
  };

  // Coarse search.
  gridSearch(xMin, xMax);

  // Refine around best x0.
  for (let r = 0; r < fitRefineIters; r += 1) {
    if (!best) break;
    const span = (xMax - xMin) * Math.pow(fitRefineFactor, r + 1);
    const lo = best.x0 - span;
    const hi = best.x0 + span;
    gridSearch(lo, hi);
  }

  if (!best) {
    return { ok: false, reason: 'fit failed', nUsed: pts.length };
  }

  // Optional sanity: dMax should be >= max observed (usually).
  const yMax = Math.max(...ys);
  const dMax = Math.max(best.dMax, yMax);

  return { ok: true, ...best, dMax };
}

export async function tuneForce(sendFn, plan, options = {}) {
  // Validate and apply options
  const motorIds = options.motorIds ?? [];
  const axes = options.axes ?? [];
  const mmPerDeg = options.mmPerDeg ?? [];
  const feed = Number.isFinite(options.feed) ? options.feed : DEFAULT_FEED;
  const speedup = Number.isFinite(options.speedup) ? options.speedup : 1;
  const forbiddenForceAnchors = options.forbiddenForceAnchors ?? [];
  const fixedAnchors = plan?.config?.fixedAnchors ?? [];

  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    console.log('; auto-tune force skipped (no motor IDs available)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'force-thresholds' },
    };
  }

  const driveAnchor = Number.isFinite(plan.config?.driveAnchor)
    ? plan.config.driveAnchor
    : plan.pairAnchors?.[0];
  if (!Number.isFinite(driveAnchor) || driveAnchor < 0 || driveAnchor >= motorIds.length) {
    console.log('; auto-tune force skipped (missing active anchor)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'force-thresholds' },
    };
  }

  const forbidden = new Set(forbiddenForceAnchors ?? []);
  if (forbidden.has(driveAnchor)) {
    console.log('; auto-tune force skipped (active anchor is forbidden)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'force-thresholds' },
    };
  }

  let fixedAnchor = null;
  const fixedCandidates = fixedAnchors
    .filter((anchorIdx) => Number.isFinite(anchorIdx) && anchorIdx !== driveAnchor);
  if (fixedCandidates.length > 0) {
    fixedAnchor = fixedCandidates.find((idx) => forbidden.has(idx)) ?? fixedCandidates[0];
  } else {
    const otherCandidates = range(motorIds.length).filter((idx) => idx !== driveAnchor);
    fixedAnchor = otherCandidates.find((idx) => forbidden.has(idx)) ?? otherCandidates[0] ?? null;
  }
  if (!Number.isFinite(fixedAnchor)) {
    console.log('; auto-tune force skipped (no fixed anchor available)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'force-thresholds' },
    };
  }

  const restAnchors = range(motorIds.length).filter((idx) => idx !== driveAnchor && idx !== fixedAnchor);
  if (restAnchors.length === 0) {
    console.log('; auto-tune force skipped (needs at least one non-fixed anchor)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'force-thresholds' },
    };
  }

  const fallback = {
    forceLow: Number.isFinite(options.forceLow) ? options.forceLow : DEFAULT_FORCE_LOW_N,
    forceMid: Number.isFinite(options.forceMid) ? options.forceMid : DEFAULT_FORCE_MID_N,
    forceMax: Number.isFinite(options.forceMax) ? options.forceMax : DEFAULT_FORCE_MAX_N,
  };
  // End validate and apply options

  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map((_, idx) => (forbiddenForceAnchors.includes(idx) ? 'position' : fallback.forceLow)),
  });
  await waitForStableEncoders(sendFn, motorIds, speedup);

  const baseLow = clampAutoTuneForce(options.forceLow ?? DEFAULT_FORCE_LOW_N) ?? DEFAULT_FORCE_LOW_N;
  const capForceLimit = clampAutoTuneForce(
    options.forceMaxProvided ? options.forceMax : AUTO_TUNE_MAX_FORCE_N,
  ) ?? AUTO_TUNE_MAX_FORCE_N;
  let idleForce = baseLow;

  if (Array.isArray(axes) && Array.isArray(mmPerDeg)) {
    await returnMotorsToOriginOneAtATime(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
      midForce: baseLow,
      fixedAnchors: [fixedAnchor],
      forbiddenForceAnchors,
    });
    await setForceTrialModes(sendFn, motorIds, {
      activeAnchor: driveAnchor,
      fixedAnchor,
      idleForce: baseLow,
      activeForce: baseLow,
      forbiddenForceAnchors,
    });
  }

  const noiseStats = await calibrateEncoderNoise(sendFn, {
    motorIds,
    fixedAnchor,
    idleForce: baseLow,
    speedup,
    forbiddenForceAnchors,
  });
  const thresholds = buildMovementThresholds(noiseStats.sigmaByMotorDeg, {
    activeAnchor: driveAnchor,
    restAnchors,
  });

  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const intervalMs = Math.max(20, AUTO_TUNE_SAMPLE_INTERVAL_MS / timeScale);
  const stallSpeedDegPerSec = computeStallSpeedThresholdDegPerSec(thresholds.sigmaAct, intervalMs / 1000);

  const formatValue = (value, digits = 4) => (Number.isFinite(value) ? value.toFixed(digits) : 'n/a');
  const logTrial = (label, force, result) => {
    const travel = formatValue(result?.travelDeg, 3);
    console.log(`; auto-tune ${label}: test=${formatValue(force)} moved=${result?.moved ? 'yes' : 'no'} travel=${travel}deg stalled=${result?.stalled ? 'yes' : 'no'}`);
  };

  const runTrial = async (force, label, trialOptions = {}) => {
    const result = await runForceTrial(sendFn, {
      motorIds,
      activeAnchor: driveAnchor,
      fixedAnchor,
      restAnchors,
      idleForce,
      testForce: force,
      speedup,
      thresholds,
      axes,
      mmPerDeg,
      feed,
      forbiddenForceAnchors,
      waitForStall: options.waitForStall ?? true,
      stallTimeoutMs: options.stallTimeoutMs,
    });
    if (label) {
      logTrial(label, force, result);
    }
    return result;
  };
  const minForceResult = await findMinimumMovingForce(sendFn, {
    motorIds,
    axes,
    mmPerDeg,
    feed,
    speedup,
    baseLow,
    capForceLimit,
    trialFn: runTrial,
  });
  const { forceStart, lastNoMoveForce, firstMoveForce } = minForceResult;
  if (!Number.isFinite(forceStart)) {
    console.log('; auto-tune force failed to find force-start; using provided/default forces');
    return {
      ...fallback,
      tuningMeta: {
        tuning_failed: true,
        method: 'force-thresholds',
        noise_sigma_deg: noiseStats.sigmaByMotorDeg,
      },
    };
  }
  const idleCandidate = Math.max(baseLow, DEFAULT_FORCE_LOW_N, AUTO_TUNE_IDLE_FORCE_RATIO * forceStart);
  const adjustedIdle = clampAutoTuneForce(idleCandidate) ?? baseLow;
  if (adjustedIdle > idleForce + 1e-12) {
    idleForce = adjustedIdle;
  }

  let capForceUsed = capForceLimit;
  if (capForceUsed < forceStart - 1e-12) {
    console.log('; auto-tune force cap below force-start; using force-start for cap');
    capForceUsed = forceStart;
  }
  const edgeResult = await findEdgeForce(sendFn, {
    forceStart,
    capForceLimit: capForceUsed,
    trialFn: runTrial,
    bracketFactor: AUTO_TUNE_BRACKET_FACTOR,
    maxBracketSteps: AUTO_TUNE_MAX_BRACKET_STEPS,
    saturationRelTol: AUTO_TUNE_RELATIVE_TOLERANCE,
    minUsefulTravelDeg: thresholds.thetaActThr,
  });

  const forceEdge = edgeResult?.forceEdge;
  const dMax = edgeResult?.dMax;

  if (!Number.isFinite(forceEdge) || !Number.isFinite(dMax) || dMax <= 0) {
    const reason = edgeResult?.reason ?? 'unknown';
    console.log(`; auto-tune force failed to measure edge force (${reason}); using capped max`);
    return {
      forceLow: idleForce,
      forceMid: forceStart,
      forceMax: capForceUsed,
      tuningMeta: {
        method: 'force-thresholds',
        tuning_failed: true,
        force_start: forceStart,
        force_cap: capForceUsed,
        edge_reason: reason,
        noise_sigma_deg: noiseStats.sigmaByMotorDeg,
      },
    };
  }

  console.log(
    `; auto-tune selected: idle=${formatValue(idleForce)} start=${formatValue(forceStart)} `
    + `edge=${formatValue(forceEdge)} d_max=${formatValue(dMax, 3)}deg`,
  );

  const tuned = {
    forceLow: idleForce,
    forceMid: forceStart,
    forceMax: forceEdge,
    tuningMeta: {
      method: 'force-thresholds',
      active_anchor: driveAnchor,
      fixed_anchor: fixedAnchor,
      rest_anchors: restAnchors,
      idle_force_initial: baseLow,
      idle_force_final: idleForce,
      force_start: forceStart,
      force_edge: forceEdge,
      force_cap: capForceUsed,
      force_start_bracket_no: lastNoMoveForce,
      force_start_bracket_yes: firstMoveForce,
      d_max_deg: dMax,
      edge_at_cap: Math.abs(forceEdge - capForceUsed) <= 1e-9,
      edge_reason: edgeResult?.reason,
      edge_saturation: edgeResult?.saturation ?? null,
      edge_fit: edgeResult?.fit ?? null,
      edge_samples: edgeResult?.samples ?? [],
      noise_samples: noiseStats.samples,
      noise_duration_ms: noiseStats.durationMs,
      noise_sigma_deg: noiseStats.sigmaByMotorDeg,
      move_thresholds_deg: {
        active: thresholds.thetaActThr,
        residual_active: thresholds.thetaResThr,
        residual_sum: thresholds.sumResidualThr,
        other_by_anchor: Object.fromEntries(thresholds.thetaOtherByAnchor.entries()),
      },
      stall_speed_deg_per_sec: stallSpeedDegPerSec,
      stall_window_ms: AUTO_TUNE_STALL_WINDOW_MS,
    },
  };

  await returnMotorsToOriginOneAtATime(sendFn, {
    motorIds,
    axes,
    mmPerDeg,
    feed,
    speedup,
    midForce: tuned.forceLow,
    fixedAnchors,
    forbiddenForceAnchors,
  });

  return tuned;
}

function range(n) {
  return Array.from({ length: n }, (_, i) => i);
}

export const FORCE_TUNING_CONSTANTS = {
  AUTO_TUNE_MIN_FORCE_N,
  AUTO_TUNE_MAX_FORCE_N,
  AUTO_TUNE_SAMPLE_WINDOW_MS,
  AUTO_TUNE_NOISE_SAMPLE_MS,
  AUTO_TUNE_NOISE_SAMPLE_INTERVAL_MS,
  AUTO_TUNE_SAMPLE_INTERVAL_MS,
  AUTO_TUNE_STALL_WINDOW_MS,
  AUTO_TUNE_MIN_STALL_SPEED_DEG_PER_SEC,
  AUTO_TUNE_BRACKET_FACTOR,
  AUTO_TUNE_MAX_BRACKET_STEPS,
  AUTO_TUNE_MAX_BISECT_STEPS,
  AUTO_TUNE_RELATIVE_TOLERANCE,
  AUTO_TUNE_ABSOLUTE_TOLERANCE,
  AUTO_TUNE_EDGE_RATIO,
  AUTO_TUNE_IDLE_FORCE_RATIO,
};
