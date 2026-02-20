import { parseEncoderReply, runMoveWithWait, sleep as baseSleep } from '../primitives/encoder_utils.mjs';
import { sampleEncoderNoise } from './encoder_noise.mjs';
import {
  formatCallSite,
  getDebugState,
  parseCallSite,
  updateDebugAngles,
} from '../primitives/debug_trace.mjs';

const DEFAULT_STABILITY_POLL_MS = 500;
const DEFAULT_STABILITY_WINDOW_MS = 2000;
const DEFAULT_STABILITY_TOLERANCE_DEG = 1.0;
const DEFAULT_LOW_FORCE_N = 0.001; // Just pull out more line
const DEFAULT_MID_FORCE_N = 1.0; // Carefully wind in automatically

function formatForceValue(value) {
  const num = Number(value);
  if (!Number.isFinite(num)) {
    return '?';
  }
  const rounded = Math.round(num * 1000) / 1000;
  const asString = rounded.toFixed(3);
  return asString.replace(/\.?0+$/, '');
}

export function buildG92Command(axes) {
  if (!Array.isArray(axes) || axes.length === 0) {
    return 'G92';
  }
  const parts = axes.map((axis) => `${axis}0`);
  return `G92 ${parts.join(' ')}`;
}

export function angleToLength(angleDeg, axisIdx, mmPerDeg) {
  const mmPer = Array.isArray(mmPerDeg) ? mmPerDeg[axisIdx] : null;
  if (!Number.isFinite(mmPer)) {
    return 0;
  }
  return angleDeg * mmPer;
}

export async function waitForStableEncoders(sendFn, motorIds, speedup, options = {}) {
  const debugState = getDebugState(sendFn);
  if (debugState?.enabled) {
    const callSite = parseCallSite(new Error().stack, { skip: 1 });
    console.log(`waitForStableEncoders called from ${formatCallSite(callSite)}`);
  }
  const {
    pollIntervalMs = DEFAULT_STABILITY_POLL_MS,
    stableWindowMs = DEFAULT_STABILITY_WINDOW_MS,
    toleranceDeg = DEFAULT_STABILITY_TOLERANCE_DEG,
    timeoutMs = null,
    sleepFn = baseSleep,
    nowFn = () => Date.now(),
  } = options;
  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const pollMs = pollIntervalMs / timeScale;
  const windowMs = Math.max(pollMs * 2, stableWindowMs / timeScale);
  const tol = Math.max(0, Number.isFinite(toleranceDeg) ? toleranceDeg : DEFAULT_STABILITY_TOLERANCE_DEG);
  const startMs = nowFn();
  const samples = [];

  // This function is only sent samples collected within the current window,
  // so looking at all samples is ok.
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
    const nowMs = nowFn();

    if (anglesDeg.length === motorIds.length && anglesDeg.every((v) => Number.isFinite(v))) {
      samples.push({ timestampMs: nowMs, anglesDeg });
      const cutoff = nowMs - windowMs - pollMs;
      while (samples.length > 0 && samples[0].timestampMs < cutoff) {
        samples.shift();
      }
    } else {
      samples.length = 0;
    }

    if (isStable()) {
      const result = {
        anglesDeg: samples[samples.length - 1].anglesDeg.slice(),
        samples: samples.length,
        elapsedMs: nowMs - startMs,
      };
      updateDebugAngles(sendFn, result.anglesDeg);
      return result;
    }

    if (Number.isFinite(timeoutMs) && timeoutMs > 0 && nowMs - startMs > timeoutMs) {
      throw new Error(`Timed out waiting for encoder stability after ${Math.round(timeoutMs)}ms`);
    }

    // eslint-disable-next-line no-await-in-loop
    await sleepFn(pollMs);
  }
}

export async function getCurrentLengths(sendFn, motorIds, mmPerDeg) {
  const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
  const anglesDeg = parseEncoderReply(encoderReply?.reply);
  return anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
}

export async function primeEncoders(sendFn, { motorIds, axes } = {}) {
  await sendFn(`M569.3 P${motorIds.join(':')} S`);
}

export async function applyForceModeState(sendFn, {
  motorIds,
  modes,
} = {}) {
  const forces = [];
  const normalizedModes = [];
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    const rawMode = Array.isArray(modes) ? modes[idx] : modes;
    const mode = typeof rawMode === 'string' ? rawMode.toLowerCase() : rawMode;
    if (mode === 'position' || mode === 'pos' || mode === 0 || mode === 0.0) {
      forces.push('0.0');
      normalizedModes.push('position');
    } else {
      if (Number.isFinite(mode)) {
        forces.push(`${mode}`);
        normalizedModes.push(mode);
      } else {
        throw new Error(`applyForceModeState can't set force ${mode}`);
      }
    }
  }
  if (forces.length === 0) {
    return;
  }
  const debugState = getDebugState(sendFn);
  if (debugState?.enabled) {
    const callSite = parseCallSite(new Error().stack, { skip: 1 });
    const labels = normalizedModes.map((mode) => (
      mode === 'position' ? 'pos_mode' : `${formatForceValue(mode)} N`
    ));
    console.log(`applyForceModeState [${labels.join(', ')}] from ${formatCallSite(callSite)}`);
    debugState.lastModes = normalizedModes.slice();
  }
  await sendFn(`M569.4 P${motorIds.join(':')} T${forces.join(':')}`);
}

function buildDataPointModes({
  motorIds,
  driveAnchor,
  sensorAnchors,
  fixedAnchors = [],
  forbiddenForceAnchors = [],
  forceMax,
  forceMin,
  forceMid,
} = {}) {
  const fixedSet = new Set((fixedAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  const sensorSet = new Set(
    (Array.isArray(sensorAnchors) ? sensorAnchors : [sensorAnchors])
      .filter((idx) => Number.isFinite(idx)),
  );
  const forbidden = new Set((forbiddenForceAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  const fallbackForce = Number.isFinite(forceMid)
    ? forceMid
    : (Number.isFinite(forceMin) ? forceMin : 0);

  const buildModes = (sensorForce) => motorIds.map((_, idx) => {
    if (idx === driveAnchor) {
      return 'position';
    }
    if (fixedSet.has(idx) || forbidden.has(idx)) {
      return 'position';
    }
    if (sensorSet.has(idx)) {
      return sensorForce;
    }
    return fallbackForce;
  });

  const returnModes = buildModes(
    Number.isFinite(forceMid)
      ? forceMid * 2.0
      : (Number.isFinite(forceMax) ? forceMax : fallbackForce),
  );
  const relaxModes = buildModes(fallbackForce);

  return {
    buildModes,
    returnModes,
    relaxModes,
    fallbackForce,
  };
}

export async function applyDataPointReturnModes(sendFn, options = {}) {
  const { motorIds, speedup, settleOptions = {} } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    throw new Error('applyDataPointReturnModes requires motorIds');
  }
  const { returnModes } = buildDataPointModes(options);
  await applyForceModeState(sendFn, { motorIds, modes: returnModes });
  await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
  return returnModes;
}

export async function collectDataPoint(sendFn, options = {}) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    driveAnchor,
    sensorAnchors,
    fixedAnchors = [],
    forbiddenForceAnchors = [],
    forceMax,
    forceMin,
    forceMid,
    speedup,
    settleOptions = {},
    recordPoint,
    driveSetpointMm,
    stepIndex,
    stepCount,
    restoreToModeWhenFinished,
    projectZeroTension = false,
    skipReturnModePrep = false,
    encoderNoiseOptions = null,
  } = options;

  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    throw new Error('collectDataPoint requires motorIds');
  }
  if (!Array.isArray(axes) || axes.length !== motorIds.length) {
    throw new Error('collectDataPoint requires axes mapping');
  }
  if (!Array.isArray(mmPerDeg) || mmPerDeg.length !== motorIds.length) {
    throw new Error('collectDataPoint requires mmPerDeg mapping');
  }

  const debugState = getDebugState(sendFn);
  if (debugState?.enabled) {
    const callSite = parseCallSite(new Error().stack, { skip: 1 });
    console.log(`collectDataPoint called from ${formatCallSite(callSite)}`);
  }

  const {
    buildModes,
    returnModes,
    relaxModes,
    fallbackForce,
  } = buildDataPointModes({
    motorIds,
    driveAnchor,
    sensorAnchors,
    fixedAnchors,
    forbiddenForceAnchors,
    forceMax,
    forceMin,
    forceMid,
  });

  if (!skipReturnModePrep) {
    await applyForceModeState(sendFn, { motorIds, modes: returnModes });
    await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
  }

  let anglesDeg = null;
  let stableData = null;

  if (projectZeroTension) {
    const maxForce = Number.isFinite(forceMax) ? forceMax : null;
    const minForce = Number.isFinite(forceMin) ? forceMin : null;
    const samples = [];
    if (maxForce !== null && minForce !== null) {
      const steps = 10;
      const stepDelta = steps > 1 ? (minForce - maxForce) / (steps - 1) : 0;
      for (let idx = 0; idx < steps; idx += 1) {
        const f = maxForce + stepDelta * idx;
        await applyForceModeState(sendFn, { motorIds, modes: buildModes(f) });
        // eslint-disable-next-line no-await-in-loop
        const stable = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
        samples.push({ force: f, anglesDeg: stable.anglesDeg });
        stableData = stable;
      }
    }

    const valid = samples.filter((sample) =>
      Number.isFinite(sample.force)
      && Array.isArray(sample.anglesDeg)
      && sample.anglesDeg.length === motorIds.length
      && sample.anglesDeg.every((val) => Number.isFinite(val)),
    );
    const fallbackAngles = valid.length > 0 ? valid[valid.length - 1].anglesDeg : null;
    let projected = null;
    if (valid.length >= 2) {
      const forces = valid.map((sample) => sample.force);
      const meanForce = forces.reduce((acc, v) => acc + v, 0) / forces.length;
      const varForce = forces.reduce((acc, v) => acc + (v - meanForce) ** 2, 0);
      if (varForce > 1e-9) {
        projected = new Array(motorIds.length).fill(0);
        for (let axisIdx = 0; axisIdx < motorIds.length; axisIdx += 1) {
          let meanAngle = 0;
          for (let i = 0; i < valid.length; i += 1) {
            meanAngle += valid[i].anglesDeg[axisIdx];
          }
          meanAngle /= valid.length;
          let cov = 0;
          for (let i = 0; i < valid.length; i += 1) {
            cov += (forces[i] - meanForce) * (valid[i].anglesDeg[axisIdx] - meanAngle);
          }
          const slope = cov / varForce;
          const intercept = meanAngle - slope * meanForce;
          if (!Number.isFinite(intercept)) {
            projected = null;
            break;
          }
          projected[axisIdx] = intercept;
        }
      }
    }
    anglesDeg = projected ?? fallbackAngles;
    if (!Array.isArray(anglesDeg)) {
      stableData = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
      anglesDeg = stableData.anglesDeg;
    }
  } else {
    await applyForceModeState(sendFn, { motorIds, modes: relaxModes });
    stableData = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
    anglesDeg = stableData.anglesDeg;
  }

  let lengths = null;
  if (Array.isArray(anglesDeg)) {
    lengths = anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
    if (typeof recordPoint === 'function') {
      const extraFields = projectZeroTension
        ? { raw_angles_zero_tension_deg: [...anglesDeg] }
        : null;
      recordPoint(anglesDeg, driveSetpointMm, stepIndex, stepCount, extraFields);
    }
  }

  let noiseStats = null;
  if (encoderNoiseOptions) {
    noiseStats = await sampleEncoderNoise(sendFn, motorIds, {
      ...encoderNoiseOptions,
      speedup,
    });
  }

  if (restoreToModeWhenFinished !== undefined) {
    await applyForceModeState(sendFn, { motorIds, modes: restoreToModeWhenFinished });
    await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
  } else {
    await applyForceModeState(sendFn, { motorIds, modes: returnModes });
    await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
  }

  return {
    stableData,
    anglesDeg,
    lengths,
    noiseStats,
  };
}

export function calculateReturnOrder({ fixedAnchors = [], currentLengths = [] } = {}) {
  const fixedSet = new Set((fixedAnchors ?? []).filter((idx) => Number.isFinite(idx)));
  const others = [];
  for (let idx = 0; idx < currentLengths.length; idx += 1) {
    if (fixedSet.has(idx)) {
      continue;
    }
    const length = Math.abs(Number.isFinite(currentLengths[idx]) ? currentLengths[idx] : 0);
    others.push({ idx, length });
  }
  others.sort((a, b) => b.length - a.length);
  const fixedList = (fixedAnchors ?? []).filter((idx) => Number.isFinite(idx) && idx >= 0);
  return [...others.map((entry) => entry.idx), ...fixedList];
}

function pickClosestToOrigin(lengths, exclude = new Set()) {
  let bestIdx = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (let idx = 0; idx < lengths.length; idx += 1) {
    if (exclude.has(idx)) {
      continue;
    }
    const distance = Math.abs(Number.isFinite(lengths[idx]) ? lengths[idx] : 0);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIdx = idx;
    }
  }
  return bestIdx;
}

function formatAxisDelta(axis, delta) {
  return `${axis}${delta.toFixed(3)}`;
}

export async function returnMotorsToOriginOneAtATime(sendFn, options = {}) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    feed,
    speedup = 1,
    delayFn,
    midForce = DEFAULT_MID_FORCE_N,
    fixedAnchors = [],
    forbiddenForceAnchors = [],
    settleOptions = {},
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return [];
  }
  if (!Array.isArray(axes) || axes.length !== motorIds.length) {
    throw new Error('returnMotorsToOriginOneAtATime requires full axes mapping');
  }

  const stableBefore = await waitForStableEncoders(sendFn, motorIds,  speedup, settleOptions);
  const lengths = stableBefore.anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
  const order = calculateReturnOrder({ fixedAnchors, currentLengths: lengths });
  const forbidden = new Set(forbiddenForceAnchors ?? []);

  for (const anchorIdx of order) {
    const axis = axes[anchorIdx];
    if (!axis) {
      throw new Error(`Missing axis mapping for anchor ${anchorIdx}`);
    }
    const stable = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
    const currentLengths = stable.anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
    const fixedIdx = pickClosestToOrigin(currentLengths, new Set([anchorIdx]));
    const modes = currentLengths.map((_, idx) => {
      if (idx === anchorIdx || idx === fixedIdx || forbidden.has(idx)) {
        return 'position';
      }
      return midForce;
    });
    await applyForceModeState(sendFn, {
      motorIds,
      modes
    });

    const current = currentLengths[anchorIdx] ?? 0;
    const delta = -current;
    if (Math.abs(delta) > 1e-6) {
      await runMoveWithWait(
        sendFn,
        `G1 H2 ${formatAxisDelta(axis, delta)} F${feed}`,
        speedup,
        { axes, delayFn },
      );
      await applyForceModeState(sendFn, {
        motorIds,
        modes: motorIds.map(() => 'position')
      });
    }
  }

  return getCurrentLengths(sendFn, motorIds, mmPerDeg);
}

export async function returnMotorsToOriginAllAtOnce(sendFn, options = {}) {
  const {
    motorIds,
    axes,
    mmPerDeg,
    feed,
    speedup,
    delayFn,
    settleOptions = {},
  } = options;
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return [];
  }
  if (!Array.isArray(axes) || axes.length !== motorIds.length) {
    throw new Error('returnMotorsToOriginAllAtOnce requires full axes mapping');
  }

  const stableBefore = await waitForStableEncoders(sendFn, motorIds, speedup, settleOptions);
  const lengths = stableBefore.anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
  const moveParts = [];
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    const axis = axes[idx];
    const current = lengths[idx] ?? 0;
    const delta = -current;
    if (Math.abs(delta) > 1e-6) {
      moveParts.push(formatAxisDelta(axis, delta));
    }
  }

  await sendFn(`M569.4 P${motorIds.join(':')} T0.0`);
  if (moveParts.length > 0) {
    await runMoveWithWait(
      sendFn,
      `G1 H2 ${moveParts.join(' ')} F${feed}`,
      speedup,
      { axes, delayFn },
    );
    await applyForceModeState(sendFn, {
      motorIds,
      modes: motorIds.map(() => 'position')
    });
  }

  return getCurrentLengths(sendFn, motorIds, mmPerDeg);
}
