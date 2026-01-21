import { parseEncoderReply, runMoveWithWait, sleep as baseSleep } from './encoder_utils.mjs';

const DEFAULT_STABILITY_POLL_MS = 500;
const DEFAULT_STABILITY_WINDOW_MS = 2000;
const DEFAULT_STABILITY_TOLERANCE_DEG = 1.0;
const DEFAULT_LOW_FORCE_N = 0.001; // Just pull out more line
const DEFAULT_MID_FORCE_N = 1.0; // Carefully wind in automatically

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
  const {
    pollIntervalMs = DEFAULT_STABILITY_POLL_MS,
    stableWindowMs = DEFAULT_STABILITY_WINDOW_MS,
    toleranceDeg = DEFAULT_STABILITY_TOLERANCE_DEG,
    timeoutMs = null,
  } = options;
  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const pollMs = pollIntervalMs / timeScale;
  const windowMs = Math.max(pollMs * 2, stableWindowMs / timeScale);
  const tol = Math.max(0, Number.isFinite(toleranceDeg) ? toleranceDeg : DEFAULT_STABILITY_TOLERANCE_DEG);
  const startMs = Date.now();
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
    const nowMs = Date.now();

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
    await baseSleep(pollMs);
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
  for (let idx = 0; idx < motorIds.length; idx += 1) {
    const rawMode = Array.isArray(modes) ? modes[idx] : modes;
    const mode = typeof rawMode === 'string' ? rawMode.toLowerCase() : rawMode;
    if (mode === 'position' || mode === 'pos' || mode === 0 || mode === 0.0) {
      forces.push('0.0');
    } else {
      if (Number.isFinite(mode)) {
        forces.push(`${mode}`);
      } else {
        throw new Error(`applyForceModeState can't set force ${mode}`);
      }
    }
  }
  if (forces.length === 0) {
    return;
  }
  await sendFn(`M569.4 P${motorIds.join(':')} T${forces.join(':')}`);
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
        { axes },
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
      { axes },
    );
    await applyForceModeState(sendFn, {
      motorIds,
      modes: motorIds.map(() => 'position')
    });
  }

  return getCurrentLengths(sendFn, motorIds, mmPerDeg);
}
