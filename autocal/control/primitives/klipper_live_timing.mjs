const DEFAULT_EXTERNAL_KLIPPER_RAW_TIME_SCALE = 20;

export function normalizeSpeedScale(value, fallback = 1.0) {
  const numericValue = Number(value);
  if (Number.isFinite(numericValue) && numericValue > 0) {
    return numericValue;
  }
  return Number.isFinite(fallback) && fallback > 0 ? fallback : 1.0;
}

export function resolveExternalKlipperRawTimeScale(currentTimeScale = 1.0) {
  return Math.max(
    normalizeSpeedScale(currentTimeScale, 1.0),
    DEFAULT_EXTERNAL_KLIPPER_RAW_TIME_SCALE,
  );
}

export function scaleDurationMs(durationMs, speedScale) {
  const normalizedScale = normalizeSpeedScale(speedScale, 1.0);
  const ms = Number(durationMs);
  if (!Number.isFinite(ms) || ms <= 0) {
    return 0;
  }
  return ms / normalizedScale;
}

export function scaleDeadlineMs(nowMs, deadlineMs, speedScale) {
  const now = Number(nowMs);
  const deadline = Number(deadlineMs);
  if (!Number.isFinite(now) || !Number.isFinite(deadline)) {
    return null;
  }
  const scaledDuration = scaleDurationMs(deadline - now, speedScale);
  return now + scaledDuration;
}

export function resolveRawMoveDispatchTime(firstStepTimeMs, nowMs = performance.now()) {
  const firstStep = Number(firstStepTimeMs);
  const now = Number(nowMs);
  if (Number.isFinite(firstStep)) {
    return Number.isFinite(now) ? Math.max(firstStep, now) : firstStep;
  }
  return Number.isFinite(now) ? now : performance.now();
}

export function createTimelineNormalizer() {
  let offsetMs = null;

  return {
    normalizeAt(atMs, nowMs = performance.now()) {
      const at = Number(atMs);
      const now = Number(nowMs);
      if (!Number.isFinite(at)) {
        return Number.isFinite(now) ? now : performance.now();
      }
      if (offsetMs === null) {
        offsetMs = Math.max(0, at - (Number.isFinite(now) ? now : performance.now()));
      }
      return at - offsetMs;
    },
    reset() {
      offsetMs = null;
    },
    getOffset() {
      return offsetMs ?? 0;
    },
  };
}
