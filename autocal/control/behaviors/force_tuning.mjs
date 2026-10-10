import { parseEncoderReply, runMoveWithWait, sleep as baseSleep } from '../primitives/encoder_utils.mjs';
import {
  angleToLength,
  applyForceModeState,
  getCurrentLengths,
  primeEncoders,
  returnMotorsToOriginOneAtATime,
  waitForStableEncoders,
} from '../primitives/uncalibrated_actions.mjs';

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
const AUTO_TUNE_STALL_WINDOW_MS = 25000;
const AUTO_TUNE_MIN_STALL_SPEED_DEG_PER_SEC = 0.05;
const AUTO_TUNE_BRACKET_FACTOR = 1.4;
const AUTO_TUNE_MAX_BRACKET_STEPS = 120;
const AUTO_TUNE_MAX_BISECT_STEPS = 60;
const AUTO_TUNE_RELATIVE_TOLERANCE = 0.1;
const AUTO_TUNE_ABSOLUTE_TOLERANCE = 0.01;
const AUTO_TUNE_COMPLIANCE_RATIO = 0.5;
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
    returnToOriginFn = returnMotorsToOriginOneAtATime,
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
  const durationMs = Math.max(1, sampleDurationMs);
  const intervalMs = Math.max(10, sampleIntervalMs);
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
    await (sendFn.simulationClock?.sleep ?? baseSleep)(intervalMs);
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
    previousTrialMoved = false,
    settleOptions = {},
    wallTimeoutMs = 120000,
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

  const windowMs = Math.max(1, sampleWindowMs);
  const intervalMs = Math.max(20, sampleIntervalMs);
  const effectiveStallWindowMs = Math.max(intervalMs, stallWindowMs);
  const sampleIntervalSec = intervalMs / 1000;
  const maxWindowRawMs = Number.isFinite(stallTimeoutMs) && stallTimeoutMs > 0
    ? stallTimeoutMs
    : (waitForStall ? sampleWindowMs * 3 : sampleWindowMs);
  const maxWindowMs = Math.max(windowMs, maxWindowRawMs);
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
  const stableStart = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
  let startAngles = stableStart.anglesDeg;

  const rampWaitMs = Math.max(0, rampStepWaitMs);
  if (Array.isArray(rampForces) && rampForces.length > 0) {
    for (let idx = 0; idx < rampForces.length; idx += 1) {
      const force = rampForces[idx];
      if (!Number.isFinite(force)) {
        continue;
      }
      await sendFn(`M569.4 P${motorIds[activeAnchor]} T${force}`);
      if (rampWaitMs > 0) {
        // eslint-disable-next-line no-await-in-loop
        await (sendFn.simulationClock?.sleep ?? baseSleep)(rampWaitMs);
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
  await sendFn.simulationClock?.refresh?.();
  const now = sendFn.simulationClock?.now ?? (() => Date.now());
  let lastMs = now();
  const startMs = lastMs;
  let stallDurationMs = 0;
  let stalled = false;
  let fixedDriftDeg = 0;

  const wallStart = Date.now();
  let lastProgress = wallStart;
  while (now() - startMs < stopAfterMs) {
    if (Date.now() - wallStart >= wallTimeoutMs) throw new Error('Force trial exceeded its wall-clock deadline');
    if (Date.now() - lastProgress >= 5000) {
      console.log(`; force trial running: ${((Date.now() - wallStart) / 1000).toFixed(1)}s wall, ${((now() - startMs) / 1000).toFixed(1)}s ${sendFn.simulationClock ? 'simulation' : 'wall'}`);
      lastProgress = Date.now();
    }
    // eslint-disable-next-line no-await-in-loop
    await (sendFn.simulationClock?.sleep ?? baseSleep)(intervalMs, { wallDeadlineMs: wallStart + wallTimeoutMs });
    // eslint-disable-next-line no-await-in-loop
    const reply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const angles = parseEncoderReply(reply?.reply);
    const nowMs = now();
    if (angles.length === motorIds.length && angles.every((v) => Number.isFinite(v))) {
      endAngles = angles;
      if (Number.isFinite(fixedAnchor)) {
        fixedDriftDeg = angles[fixedAnchor] - startAngles[fixedAnchor];
        if (Math.abs(fixedDriftDeg) > 1.5) break;
      }
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
        if (!stalled && stallDurationMs >= effectiveStallWindowMs) {
          stalled = true;
        }
      }
      lastAngles = angles;
    }
    lastMs = nowMs;
    if (waitForStall && stalled) {
      break;
    }
  }

  const deltaEndDeg = endAngles.map((angle, idx) => angle - (startAngles[idx] ?? 0));
  const activeDelta = deltaEndDeg[activeAnchor] ?? 0;

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
  }
  const pulloutMoved = restAnchors.length > 0
    ? (activeMoved && restMovedCount >= 1)
    : activeMoved;

  const travelDeg = Math.abs(activeDelta);

  const canReturnToOrigin = Array.isArray(axes) && Array.isArray(mmPerDeg);
  const shouldReturnToOrigin = canReturnToOrigin && travelDeg > 2 * thetaActThr;
  const returnBeforeIdleRelease = shouldReturnToOrigin && (pulloutMoved || previousTrialMoved);
  let residualAngles = null;
  if (returnBeforeIdleRelease) {
    await returnMotorsToOriginOneAtATime(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
      midForce: idleForce,
      fixedAnchors: [fixedAnchor],
      forbiddenForceAnchors,
      settleOptions,
    });
    await setForceTrialModes(sendFn, motorIds, {
      activeAnchor,
      fixedAnchor,
      idleForce,
      activeForce: idleForce,
      forbiddenForceAnchors,
    });
    residualAngles = endAngles;
  } else {
    await setForceTrialModes(sendFn, motorIds, {
      activeAnchor,
      fixedAnchor,
      idleForce,
      activeForce: idleForce,
      forbiddenForceAnchors,
    });
    const stableResidual = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
    residualAngles = stableResidual.anglesDeg;
  }

  const deltaResidualDeg = residualAngles.map((angle, idx) => angle - (startAngles[idx] ?? 0));
  const activeResidual = deltaResidualDeg[activeAnchor] ?? 0;
  restResidualSum = 0;
  for (const anchorIdx of restAnchors) {
    restResidualSum += Math.abs(deltaResidualDeg[anchorIdx] ?? 0);
  }
  const residualActiveOk = returnBeforeIdleRelease || Math.abs(activeResidual) >= thetaResThr;
  const residualRestOk = returnBeforeIdleRelease || restResidualSum >= sumResidualThr;

  const moved = restAnchors.length > 0
    ? (activeMoved && restMovedCount >= 1 && residualActiveOk && residualRestOk)
    : (activeMoved && residualActiveOk);

  if (!returnBeforeIdleRelease && shouldReturnToOrigin) {
    await returnMotorsToOriginOneAtATime(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
      midForce: idleForce,
      fixedAnchors: [fixedAnchor],
      forbiddenForceAnchors,
      settleOptions,
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
    fixedDriftDeg,
  };
}

/**
 * Find the comfortable edge from incremental compliance (travel gained per N).
 * Stop after two gains below half the best observed compliance, and use the
 * force BEFORE that decline. This keeps collection ahead of the force/travel
 * hockey stick instead of chasing an asymptote or fitting an unobserved dMax.
 * These are matched-duration excursions, not equilibrium workspace estimates.
 */
export async function findEdgeForce(sendFn, options = {}) {
  const {
    forceStart,
    capForceLimit = AUTO_TUNE_MAX_FORCE_N,
    trialFn,
    bracketFactor = AUTO_TUNE_BRACKET_FACTOR,
    maxBracketSteps = AUTO_TUNE_MAX_BRACKET_STEPS,
    complianceRatio = AUTO_TUNE_COMPLIANCE_RATIO,
    minUsefulTravelDeg = 0.5,
  } = options;
  if (typeof trialFn !== 'function') throw new Error('findEdgeForce requires a trialFn');
  if (!(Number.isFinite(forceStart) && forceStart > 0)) {
    return { forceEdge: null, dMax: null, reason: 'invalid forceStart', samples: [] };
  }
  if (!(Number.isFinite(capForceLimit) && capForceLimit > 0 && bracketFactor > 1
      && complianceRatio > 0 && complianceRatio < 1)) {
    throw new Error('Invalid edge-force search limits');
  }

  const samples = [];
  let previous = null;
  let peakCompliance = 0;
  let declineStart = null;
  let declineCount = 0;
  let force = Math.min(forceStart, capForceLimit);
  const finish = reason => ({
    forceEdge: declineStart?.force ?? null,
    // Retained metadata name: maximum OBSERVED excursion, never extrapolated.
    dMax: samples.length ? Math.max(...samples.map(s => s.travelDeg || 0)) : null,
    reason, samples,
    knee: declineStart ? { force: declineStart.force, travelDeg: declineStart.travelDeg,
      peakComplianceDegPerN: peakCompliance, complianceRatio } : null,
  });

  for (let i = 0; i < maxBracketSteps; i += 1) {
    const res = await trialFn(force, `edge-ramp ${i + 1}/${maxBracketSteps}`, {
      // Equal windows keep early settling and recorder throughput from changing
      // the response curve. Continue observing even after a detected stall.
      waitForStall: false,
      sampleWindowMs: AUTO_TUNE_SAMPLE_WINDOW_MS * 3,
    });
    const sample = { force, travelDeg: res?.travelDeg, moved: !!res?.moved,
      stalled: !!res?.stalled, fixedDriftDeg: res?.fixedDriftDeg ?? 0 };
    samples.push(sample);
    if (!Number.isFinite(sample.travelDeg) || !Number.isFinite(sample.fixedDriftDeg)
        || Math.abs(sample.fixedDriftDeg) > 1.5) {
      // A slipping held motor invalidates the geometry. Do not ramp past it.
      declineStart = previous;
      return finish('invalid travel or fixed-anchor slip; stopped at previous valid force');
    }
    if (sample.moved && sample.travelDeg >= minUsefulTravelDeg) {
      if (previous) {
        const gain = sample.travelDeg - previous.travelDeg;
        const compliance = gain / (force - previous.force);
        sample.complianceDegPerN = compliance;
        if (gain <= minUsefulTravelDeg) {
          declineStart ??= previous;
          return finish('travel stopped increasing; stopped before plateau or reversal');
        }
        peakCompliance = Math.max(peakCompliance, compliance);
        if (compliance < complianceRatio * peakCompliance) {
          declineStart ??= previous;
          declineCount += 1;
          if (declineCount >= 2) return finish('incremental compliance declined below comfortable limit');
        } else {
          declineStart = null;
          declineCount = 0;
        }
      }
      previous = sample;
    } else {
      // An unusable point cannot confirm a decline across a gap.
      previous = null;
      declineStart = null;
      declineCount = 0;
    }
    if (force >= capForceLimit) break;
    force = Math.min(force * bracketFactor, capForceLimit);
  }
  // A cap alone does not locate an edge. Let tuneForce use a bounded default.
  declineStart = null;
  return finish('no comfortable knee measured before force cap or trial limit');
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

  const fallback = {
    forceLow: Number.isFinite(options.forceLow) ? options.forceLow : DEFAULT_FORCE_LOW_N,
    forceMid: Number.isFinite(options.forceMid) ? options.forceMid : DEFAULT_FORCE_MID_N,
    forceMax: Number.isFinite(options.forceMax) ? options.forceMax : DEFAULT_FORCE_MAX_N,
  };

  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    console.log('; auto-tune force skipped (no motor IDs available)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'incremental-compliance' },
    };
  }

  const driveAnchor = Number.isFinite(plan.config?.driveAnchor)
    ? plan.config.driveAnchor
    : plan.pairAnchors?.[0];
  if (!Number.isFinite(driveAnchor) || driveAnchor < 0 || driveAnchor >= motorIds.length) {
    console.log('; auto-tune force skipped (missing active anchor)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'incremental-compliance' },
    };
  }

  const forbidden = new Set(forbiddenForceAnchors ?? []);
  if (forbidden.has(driveAnchor)) {
    console.log('; auto-tune force skipped (active anchor is forbidden)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'incremental-compliance' },
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
      tuningMeta: { tuning_failed: true, method: 'incremental-compliance' },
    };
  }

  const restAnchors = range(motorIds.length).filter((idx) => idx !== driveAnchor && idx !== fixedAnchor);
  if (restAnchors.length === 0) {
    console.log('; auto-tune force skipped (needs at least one non-fixed anchor)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'incremental-compliance' },
    };
  }

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

  const intervalMs = Math.max(20, AUTO_TUNE_SAMPLE_INTERVAL_MS);
  const stallSpeedDegPerSec = computeStallSpeedThresholdDegPerSec(thresholds.sigmaAct, intervalMs / 1000);

  const formatValue = (value, digits = 4) => (Number.isFinite(value) ? value.toFixed(digits) : 'n/a');
  const logTrial = (label, force, result) => {
    const travel = formatValue(result?.travelDeg, 3);
    console.log(`; auto-tune ${label}: test=${formatValue(force)} moved=${result?.moved ? 'yes' : 'no'} travel=${travel}deg stalled=${result?.stalled ? 'yes' : 'no'}`);
  };

  let anyTrialMoved = false;
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
      previousTrialMoved: anyTrialMoved,
      ...trialOptions,
    });
    if (result?.moved) {
      anyTrialMoved = true;
    }
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
        method: 'incremental-compliance',
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
    minUsefulTravelDeg: thresholds.thetaActThr,
  });

  const forceEdge = edgeResult?.forceEdge;
  const dMax = edgeResult?.dMax;

  if (!Number.isFinite(forceEdge) || !Number.isFinite(dMax) || dMax <= 0) {
    const reason = edgeResult?.reason ?? 'unknown';
    const safeMax = Math.max(forceStart, Math.min(DEFAULT_FORCE_MAX_N, capForceUsed));
    console.log(`; auto-tune force failed to measure edge force (${reason}); using bounded default ${safeMax}N`);
    return {
      forceLow: idleForce,
      forceMid: forceStart,
      forceMax: safeMax,
      tuningMeta: {
        method: 'incremental-compliance',
        tuning_failed: true,
        force_start: forceStart,
        force_cap: capForceUsed,
        edge_reason: reason,
        edge_samples: edgeResult?.samples ?? [],
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
      method: 'incremental-compliance',
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
      edge_knee: edgeResult?.knee ?? null,
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
  AUTO_TUNE_COMPLIANCE_RATIO,
  AUTO_TUNE_IDLE_FORCE_RATIO,
};
