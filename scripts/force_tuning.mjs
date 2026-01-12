import { parseEncoderReply, runMoveWithWait, sleep as baseSleep } from './encoder_utils.mjs';
import {
  angleToLength,
  applyForceModeState,
  getCurrentLengths,
  primeEncoders,
  returnMotorsToOriginOneAtATime,
  waitForStableEncoders,
} from './uncalibrated_actions.mjs';

const DEFAULT_FEED = 1400;
const DEFAULT_FORCE_LOW_NM = 0.01;
const DEFAULT_FORCE_MIN_NM = 0.01;
const DEFAULT_FORCE_MAX_NM = 0.1;
const DEFAULT_FORCE_STEP_NM = 0.01;
const DEFAULT_FORCE_STEP_DIVISOR = 6;

const AUTO_TUNE_MIN_FORCE_NM = 0.01;
const AUTO_TUNE_MAX_FORCE_NM = 20.0;
const AUTO_TUNE_SAMPLE_WINDOW_MS = 10000;
const AUTO_TUNE_NOISE_SAMPLE_MS = 4000;
const AUTO_TUNE_NOISE_SAMPLE_INTERVAL_MS = 200;
const AUTO_TUNE_SAMPLE_INTERVAL_MS = 500;
const AUTO_TUNE_FORCE_RAMP_WAIT_MS = 300;
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
  DEFAULT_FORCE_LOW_NM,
  DEFAULT_FORCE_MIN_NM,
  DEFAULT_FORCE_MAX_NM,
  DEFAULT_FORCE_STEP_NM,
  DEFAULT_FORCE_STEP_DIVISOR,
};

export function deriveForceStep(minForce, maxForce) {
  const span = Math.abs(maxForce - minForce);
  if (!Number.isFinite(span) || span <= 0) {
    return DEFAULT_FORCE_STEP_NM;
  }
  const step = span / DEFAULT_FORCE_STEP_DIVISOR;
  return step > 0 ? step : DEFAULT_FORCE_STEP_NM;
}

function clampAutoTuneForce(value) {
  if (!Number.isFinite(value)) {
    return null;
  }
  return Math.min(AUTO_TUNE_MAX_FORCE_NM, Math.max(AUTO_TUNE_MIN_FORCE_NM, value));
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

function computeStallSpeedThresholdDegPerSec(sigmaActDeg, sampleIntervalSec) {
  const interval = Number.isFinite(sampleIntervalSec) && sampleIntervalSec > 0 ? sampleIntervalSec : null;
  const noiseSpeed = Number.isFinite(sigmaActDeg) && interval
    ? (6 * sigmaActDeg) / interval
    : 0;
  return Math.max(AUTO_TUNE_MIN_STALL_SPEED_DEG_PER_SEC, noiseSpeed);
}

function buildMovementThresholds(noiseSigmaDeg, { activeAnchor, restAnchors = [] } = {}) {
  const sigmaAct = Array.isArray(noiseSigmaDeg) ? noiseSigmaDeg[activeAnchor] : 0;
  const thetaActThr = Math.max(0.5, 6 * (Number.isFinite(sigmaAct) ? sigmaAct : 0));
  const thetaResThr = Math.max(0.3, 4 * (Number.isFinite(sigmaAct) ? sigmaAct : 0));

  const thetaOtherByAnchor = new Map();
  const restSigmas = [];
  for (const anchorIdx of restAnchors) {
    const sigma = Array.isArray(noiseSigmaDeg) ? noiseSigmaDeg[anchorIdx] : 0;
    const thr = Math.max(0.5, 6 * (Number.isFinite(sigma) ? sigma : 0));
    thetaOtherByAnchor.set(anchorIdx, thr);
    restSigmas.push(Number.isFinite(sigma) ? sigma : 0);
  }
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
  let current = Math.max(start, AUTO_TUNE_MIN_FORCE_NM);
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

async function setForceTrialModes(sendFn, motorIds, options = {}) {
  const {
    activeAnchor = null,
    fixedAnchor = null,
    idleForce = DEFAULT_FORCE_LOW_NM,
    activeForce = null,
    forbiddenForceAnchors = [],
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return;
  }
  const forbidden = new Set(forbiddenForceAnchors ?? []);
  const idle = Number.isFinite(idleForce) ? idleForce : DEFAULT_FORCE_LOW_NM;
  const active = Number.isFinite(activeForce) ? activeForce : idle;

  for (let idx = 0; idx < motorIds.length; idx += 1) {
    const motorId = motorIds[idx];
    if (idx === fixedAnchor || forbidden.has(idx)) {
      await sendFn(`M569.4 P${motorId} T0.0`);
    } else if (idx === activeAnchor) {
      await sendFn(`M569.4 P${motorId} T${active}`);
    } else {
      await sendFn(`M569.4 P${motorId} T${idle}`);
    }
  }
}

export async function calibrateEncoderNoise(sendFn, options = {}) {
  const {
    motorIds,
    fixedAnchor = null,
    idleForce = DEFAULT_FORCE_LOW_NM,
    speedup = 1,
    sampleDurationMs = AUTO_TUNE_NOISE_SAMPLE_MS,
    sampleIntervalMs = AUTO_TUNE_NOISE_SAMPLE_INTERVAL_MS,
    forbiddenForceAnchors = [],
    delayFn = baseSleep,
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
  await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });

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
    await delayFn(intervalMs);
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

async function runForceTrial(sendFn, options = {}) {
  const {
    motorIds,
    activeAnchor,
    fixedAnchor = null,
    restAnchors = [],
    idleForce = DEFAULT_FORCE_LOW_NM,
    testForce = DEFAULT_FORCE_LOW_NM,
    speedup = 1,
    sampleWindowMs = AUTO_TUNE_SAMPLE_WINDOW_MS,
    sampleIntervalMs = AUTO_TUNE_SAMPLE_INTERVAL_MS,
    rampForces = null,
    rampStepWaitMs = AUTO_TUNE_FORCE_RAMP_WAIT_MS,
    rebaselineAfterRamp = false,
    stallWindowMs = AUTO_TUNE_STALL_WINDOW_MS,
    stallSpeedDegPerSec = null,
    thresholds = null,
    axes,
    mmPerDeg,
    feed = DEFAULT_FEED,
    forbiddenForceAnchors = [],
    delayFn = baseSleep,
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
  const stableStart = await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });
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
        await delayFn(rampWaitMs);
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

  while (Date.now() - startMs < windowMs) {
    // eslint-disable-next-line no-await-in-loop
    await delayFn(intervalMs);
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
  }

  await setForceTrialModes(sendFn, motorIds, {
    activeAnchor,
    fixedAnchor,
    idleForce,
    activeForce: idleForce,
    forbiddenForceAnchors,
  });
  const stableResidual = await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });
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

  if (Array.isArray(axes) && Array.isArray(mmPerDeg)) {
    await returnMotorsToOriginOneAtATime(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
      lowForceNm: idleForce,
      fixedAnchors: [fixedAnchor],
      forbiddenForceAnchors,
      delayFn,
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

async function sampleForceTuningVelocityRps(sendFn, motorIds, targetAnchor, options = {}) {
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return 0;
  }
  const anchorIdx = Number.isFinite(targetAnchor) ? targetAnchor : null;
  if (anchorIdx === null || anchorIdx < 0 || anchorIdx >= motorIds.length) {
    return 0;
  }
  const {
    speedup,
    sampleWindowMs = AUTO_TUNE_SAMPLE_WINDOW_MS,
    startAnglesDeg = null,
    delayFn = baseSleep,
  } = options;

  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const windowMs = Math.max(1, sampleWindowMs / timeScale);
  const windowSec = windowMs / 1000;
  if (!Number.isFinite(windowSec) || windowSec <= 0) {
    return 0;
  }

  let startAngles = Array.isArray(startAnglesDeg) ? startAnglesDeg : null;
  if (!startAngles) {
    const startReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    startAngles = parseEncoderReply(startReply?.reply);
  }
  const startAngle = startAngles?.[anchorIdx];
  if (!Number.isFinite(startAngle)) {
    return 0;
  }

  await delayFn(windowMs);

  const endReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
  const endAngles = parseEncoderReply(endReply?.reply);
  const endAngle = endAngles?.[anchorIdx];
  if (!Number.isFinite(endAngle)) {
    return 0;
  }

  const revolutions = Math.abs(endAngle - startAngle) / 360;
  return revolutions / windowSec;
}

async function returnDriveToTarget(sendFn, options) {
  const {
    driveAxis,
    targetLength = 0,
    currentLength = 0,
    segmentCount = 1,
    feed = DEFAULT_FEED,
    speedup = 1,
    axes,
    onSegment = null,
    delayFn = baseSleep,
  } = options;
  if (!driveAxis) {
    throw new Error('returnDriveToTarget requires a drive axis');
  }
  const segments = Math.max(1, Math.floor(Number.isFinite(segmentCount) ? segmentCount : 1));
  const totalDelta = (Number.isFinite(targetLength) ? targetLength : 0)
    - (Number.isFinite(currentLength) ? currentLength : 0);
  const segmentDelta = segments > 0 ? totalDelta / segments : totalDelta;
  const roundToGcodeMm = (value) => Math.round(value * 1000) / 1000;
  let commanded = 0;

  for (let segIdx = 0; segIdx < segments; segIdx += 1) {
    const desired = segIdx === segments - 1
      ? totalDelta - commanded
      : segmentDelta;
    const delta = roundToGcodeMm(desired);
    commanded += delta;
    if (Math.abs(delta) > 1e-6) {
      await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${delta.toFixed(3)} F${feed}`, speedup, { axes, delayFn });
    }
    if (typeof onSegment === 'function') {
      await onSegment(segIdx + 1, segments);
    }
  }
}

async function prepareForceTuningPositioning(sendFn, task, options) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    fixedTargets = [],
    forceLow = DEFAULT_FORCE_LOW_NM,
    feed = DEFAULT_FEED,
    speedup = 1,
    currentPositions = [],
    delayFn = baseSleep,
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return currentPositions.slice();
  }
  if (!Array.isArray(currentPositions) || currentPositions.length !== motorIds.length) {
    throw new Error('currentPositions must match motorIds length');
  }

  const { fixedAnchors = [], driveAnchor, sensorAnchor } = task;
  if (!Number.isFinite(driveAnchor) || !Number.isFinite(sensorAnchor)) {
    throw new Error('prepareForceTuningPositioning requires drive and sensor anchors');
  }

  await sendFn(`M569.4 P${motorIds.join(':')} T0.0`);
  const measured = await getCurrentLengths(sendFn, motorIds, mmPerDeg);
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    currentPositions[idx] = Number.isFinite(measured[idx]) ? measured[idx] : 0;
  }

  const moveParts = [];
  const formatDelta = (axis, delta) => `${axis}${delta.toFixed(3)}`;
  const lowForce = Number.isFinite(forceLow) ? forceLow : DEFAULT_FORCE_LOW_NM;
  await sendFn(`M569.4 P${motorIds[sensorAnchor]} T${lowForce}`);

  for (let i = 0; i < fixedAnchors.length; i += 1) {
    const anchorIdx = fixedAnchors[i];
    const target = Number.isFinite(fixedTargets[i]) ? fixedTargets[i] : 0;
    const delta = target - (currentPositions[anchorIdx] ?? 0);
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatDelta(axes[anchorIdx], delta));
    }
    currentPositions[anchorIdx] = target;
  }

  const tuningAnchors = new Set([driveAnchor, sensorAnchor]);
  for (const anchorIdx of tuningAnchors) {
    const target = 0;
    const delta = target - (currentPositions[anchorIdx] ?? 0);
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatDelta(axes[anchorIdx], delta));
    }
    currentPositions[anchorIdx] = target;
  }

  if (moveParts.length > 0) {
    await runMoveWithWait(sendFn, `G1 H2 ${moveParts.join(' ')} F${feed}`, speedup, { axes, delayFn });
  }

  return currentPositions.slice();
}

async function performForceTuningProbe(sendFn, task, options) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    feed,
    speedup,
    currentPositions,
    forceLow,
    forceTest,
    delayFn = baseSleep,
  } = options;
  const { fixedAnchors = [], driveAnchor, sensorAnchor, fixedTargets = [] } = task;
  if (!Number.isFinite(driveAnchor) || !Number.isFinite(sensorAnchor)) {
    return 0;
  }
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return 0;
  }

  const lowForce = Number.isFinite(forceLow) ? forceLow : DEFAULT_FORCE_LOW_NM;
  const testForce = Number.isFinite(forceTest) ? forceTest : lowForce;
  const driveMotorId = motorIds[driveAnchor];

  await prepareForceTuningPositioning(sendFn, {
    fixedAnchors,
    driveAnchor,
    sensorAnchor,
  }, {
    motorIds,
    axes,
    mmPerDeg,
    fixedTargets,
    forceLow: lowForce,
    feed,
    speedup,
    currentPositions,
    delayFn,
  });

  const stableStart = await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });
  const startAngles = stableStart.anglesDeg;
  const startLengths = startAngles.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));

  await sendFn(`M569.4 P${driveMotorId} T${testForce}`);
  const velocityRps = await sampleForceTuningVelocityRps(sendFn, motorIds, driveAnchor, {
    speedup,
    startAnglesDeg: startAngles,
    delayFn,
  });

  await sendFn(`M569.4 P${driveMotorId} T0.0`);
  const stableCurrent = await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });
  const currentLengths = stableCurrent.anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));

  const driveAxis = axes?.[driveAnchor];
  if (!driveAxis) {
    throw new Error('force-tuning requires axes mapping for reposition step');
  }

  await returnDriveToTarget(sendFn, {
    driveAxis,
    targetLength: startLengths[driveAnchor] ?? 0,
    currentLength: currentLengths[driveAnchor] ?? 0,
    segmentCount: 1,
    feed,
    speedup,
    axes,
    delayFn,
    onSegment: async () => {
      await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });
    },
  });

  await sendFn(`M569.4 P${driveMotorId} T${lowForce}`);
  return velocityRps;
}

export async function autoTuneForceRamp(sendFn, plan, options) {
  const fallback = {
    forceLow: Number.isFinite(options.forceLow) ? options.forceLow : DEFAULT_FORCE_LOW_NM,
    forceMin: Number.isFinite(options.forceMin) ? options.forceMin : DEFAULT_FORCE_MIN_NM,
    forceMax: Number.isFinite(options.forceMax) ? options.forceMax : DEFAULT_FORCE_MAX_NM,
  };

  const motorIds = options.motorIds;
  const axes = options.axes;
  const mmPerDeg = options.mmPerDeg;
  const feed = Number.isFinite(options.feed) ? options.feed : DEFAULT_FEED;
  const speedup = Number.isFinite(options.speedup) ? options.speedup : 1;
  const forbiddenForceAnchors = options.forbiddenForceAnchors ?? [];
  const delayFn = options.delayFn ?? baseSleep;

  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    console.log('; auto-tune force skipped (no motor IDs available)');
    return {
      ...fallback,
      tuningMeta: { tuning_failed: true, method: 'force-thresholds' },
    };
  }

  const fixedAnchors = plan.config?.fixedAnchors ?? [];
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

  const baseLow = clampAutoTuneForce(options.forceLow ?? DEFAULT_FORCE_LOW_NM) ?? DEFAULT_FORCE_LOW_NM;
  const capForceLimit = clampAutoTuneForce(
    options.forceMaxProvided ? options.forceMax : AUTO_TUNE_MAX_FORCE_NM,
  ) ?? AUTO_TUNE_MAX_FORCE_NM;
  let idleForce = baseLow;

  if (Array.isArray(axes) && Array.isArray(mmPerDeg)) {
    await returnMotorsToOriginOneAtATime(sendFn, {
      motorIds,
      axes,
      mmPerDeg,
      feed,
      speedup,
      lowForceNm: baseLow,
      fixedAnchors: [fixedAnchor],
      forbiddenForceAnchors,
      delayFn,
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
    delayFn,
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
    const useRamp = Number.isFinite(force) && force > idleForce + 1e-12;
    const rampForces = trialOptions.rampForces
      ?? (useRamp ? buildForceRampValues(Math.max(idleForce, AUTO_TUNE_MIN_FORCE_NM), force) : null);
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
      rampForces,
      rampStepWaitMs: AUTO_TUNE_FORCE_RAMP_WAIT_MS,
      rebaselineAfterRamp: trialOptions.rebaselineAfterRamp ?? false,
      stallSpeedDegPerSec,
      delayFn,
    });
    if (label) {
      logTrial(label, force, result);
    }
    return result;
  };

  let testForce = clampAutoTuneForce(AUTO_TUNE_MIN_FORCE_NM) ?? baseLow;
  if (testForce > capForceLimit) {
    testForce = capForceLimit;
  }
  let lastNoMoveForce = null;
  let firstMoveForce = null;

  for (let i = 0; i < AUTO_TUNE_MAX_BRACKET_STEPS; i += 1) {
    const result = await runTrial(testForce, `probe ${i + 1}/${AUTO_TUNE_MAX_BRACKET_STEPS}`);
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

  let low = Number.isFinite(lastNoMoveForce) ? lastNoMoveForce : 0;
  let high = firstMoveForce;

  for (let i = 0; i < AUTO_TUNE_MAX_BISECT_STEPS; i += 1) {
    const width = high - low;
    if (width <= AUTO_TUNE_ABSOLUTE_TOLERANCE
      || width / Math.max(high, 1e-6) <= AUTO_TUNE_RELATIVE_TOLERANCE) {
      break;
    }
    const midRaw = 0.5 * (low + high);
    const mid = clampAutoTuneForce(midRaw) ?? midRaw;
    const result = await runTrial(mid, `bisect-start ${i + 1}/${AUTO_TUNE_MAX_BISECT_STEPS}`);
    if (result.moved) {
      high = mid;
    } else {
      low = mid;
    }
  }

  const forceStart = high;
  const idleCandidate = Math.max(baseLow, DEFAULT_FORCE_LOW_NM, AUTO_TUNE_IDLE_FORCE_RATIO * forceStart);
  const adjustedIdle = clampAutoTuneForce(idleCandidate) ?? baseLow;
  if (adjustedIdle > idleForce + 1e-12) {
    idleForce = adjustedIdle;
  }

  let capForceUsed = capForceLimit;
  if (capForceUsed < forceStart - 1e-12) {
    console.log('; auto-tune force cap below force-start; using force-start for cap');
    capForceUsed = forceStart;
  }
  const capResult = await runTrial(capForceUsed, 'cap');
  const dMax = capResult.travelDeg;

  if (!Number.isFinite(dMax) || dMax <= 0) {
    console.log('; auto-tune force failed to measure D_max; using capped max');
    return {
      forceLow: idleForce,
      forceMin: forceStart,
      forceMax: capForceUsed,
      tuningMeta: {
        method: 'force-thresholds',
        tuning_failed: true,
        force_start: forceStart,
        force_cap: capForceUsed,
        noise_sigma_deg: noiseStats.sigmaByMotorDeg,
      },
    };
  }

  const targetTravel = AUTO_TUNE_EDGE_RATIO * dMax;
  let edgeLow = forceStart;
  let edgeHigh = capForceUsed;
  let edgeHighUpdated = false;

  for (let i = 0; i < AUTO_TUNE_MAX_BISECT_STEPS; i += 1) {
    const width = edgeHigh - edgeLow;
    const relWidth = width / Math.max(edgeHigh, 1e-6);
    const stopAbs = width <= AUTO_TUNE_ABSOLUTE_TOLERANCE;
    const stopRel = relWidth <= AUTO_TUNE_RELATIVE_TOLERANCE;
    const capLocked = Math.abs(edgeHigh - capForceUsed) <= 1e-9 && !edgeHighUpdated;
    if (stopAbs || (stopRel && !capLocked)) {
      break;
    }
    const midRaw = 0.5 * (edgeLow + edgeHigh);
    const mid = clampAutoTuneForce(midRaw) ?? midRaw;
    const result = await runTrial(mid, `bisect-edge ${i + 1}/${AUTO_TUNE_MAX_BISECT_STEPS}`);
    if (result.travelDeg >= targetTravel) {
      edgeHigh = mid;
      edgeHighUpdated = true;
    } else {
      edgeLow = mid;
    }
  }

  const forceEdge = edgeHigh;

  console.log(
    `; auto-tune selected: idle=${formatValue(idleForce)} start=${formatValue(forceStart)} `
    + `edge=${formatValue(forceEdge)} d_max=${formatValue(dMax, 3)}deg`,
  );

  return {
    forceLow: idleForce,
    forceMin: forceStart,
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
      d_target_deg: targetTravel,
      edge_ratio: AUTO_TUNE_EDGE_RATIO,
      edge_at_cap: Math.abs(forceEdge - capForceUsed) <= 1e-9,
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
}

export async function tuneForce(sendFn, plan, options = {}) {
  const motorIds = options.motorIds ?? [];
  const axes = options.axes ?? [];
  const mmPerDeg = options.mmPerDeg ?? [];
  const feed = Number.isFinite(options.feed) ? options.feed : DEFAULT_FEED;
  const speedup = Number.isFinite(options.speedup) ? options.speedup : 1;
  const delayFn = options.delayFn ?? baseSleep;
  const forbiddenForceAnchors = options.forbiddenForceAnchors ?? [];
  const fixedAnchors = plan?.config?.fixedAnchors ?? [];
  const forceLow = Number.isFinite(options.forceLow) ? options.forceLow : DEFAULT_FORCE_LOW_NM;

  await primeEncoders(sendFn, { motorIds, axes });
  await applyForceModeState(sendFn, {
    motorIds,
    modes: motorIds.map(() => forceLow),
    defaultForceNm: forceLow,
    forbiddenForceAnchors,
  });
  await waitForStableEncoders(sendFn, motorIds, { speedup, delayFn });

  const tuned = await autoTuneForceRamp(sendFn, plan, {
    ...options,
    forceLow,
    delayFn,
  });

  await returnMotorsToOriginOneAtATime(sendFn, {
    motorIds,
    axes,
    mmPerDeg,
    feed,
    speedup,
    lowForceNm: tuned.forceLow,
    fixedAnchors,
    forbiddenForceAnchors,
    delayFn,
  });

  return tuned;
}

function range(n) {
  return Array.from({ length: n }, (_, i) => i);
}

export const FORCE_TUNING_CONSTANTS = {
  AUTO_TUNE_MIN_FORCE_NM,
  AUTO_TUNE_MAX_FORCE_NM,
  AUTO_TUNE_SAMPLE_WINDOW_MS,
  AUTO_TUNE_NOISE_SAMPLE_MS,
  AUTO_TUNE_NOISE_SAMPLE_INTERVAL_MS,
  AUTO_TUNE_SAMPLE_INTERVAL_MS,
  AUTO_TUNE_FORCE_RAMP_WAIT_MS,
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
