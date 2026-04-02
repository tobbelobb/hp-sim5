import { distributeEvenly } from '../shared/motionUtils.js';

export function computeQueueStepDurationTicks(intervalTicks, count, addTicks = 0) {
  let totalDurationTicks = 0;
  let nextInterval = Number(intervalTicks) || 0;
  const stepCount = Math.max(0, Number(count) || 0);
  const deltaTicks = Number(addTicks) || 0;

  for (let i = 0; i < stepCount; i += 1) {
    const clampedInterval = Math.max(1, nextInterval);
    totalDurationTicks += clampedInterval;
    nextInterval = Math.max(1, clampedInterval + deltaTicks);
  }

  return totalDurationTicks;
}

export function recordDistributedSequence({
  axis,
  startTick,
  durationTicks,
  totalValue,
  bucketSize,
  accumulateStepBucket,
  accumulateExtrusionBucket,
  setMaxBucket,
}) {
  if (!Number.isFinite(totalValue) || totalValue === 0) {
    return;
  }
  if (!Number.isFinite(durationTicks) || durationTicks <= 0) {
    return;
  }

  distributeEvenly({
    startTick,
    durationTicks,
    totalValue,
    bucketSize,
    accumulate: (bucketIdx, delta) => {
      if (axis === 'E') {
        accumulateExtrusionBucket(bucketIdx, delta);
        return;
      }
      accumulateStepBucket(axis, bucketIdx, delta);
    },
    setMaxBucket,
  });
}

export function recordAnchoredStepSequence({
  startTick,
  intervalTicks,
  count,
  addTicks = 0,
  stepValue,
  bucketSize,
  accumulate,
  setMaxBucket,
}) {
  let tick = Number(startTick) || 0;
  let nextInterval = Number(intervalTicks) || 0;
  const stepCount = Math.max(0, Number(count) || 0);
  const deltaTicks = Number(addTicks) || 0;

  if (stepCount <= 0 || !Number.isFinite(stepValue) || stepValue === 0) {
    return tick;
  }

  for (let i = 0; i < stepCount; i += 1) {
    nextInterval = Math.max(1, nextInterval);
    tick += nextInterval;
    const bucketIdx = Math.floor(tick / bucketSize);
    accumulate(bucketIdx, stepValue);
    if (setMaxBucket) {
      setMaxBucket(bucketIdx);
    }
    nextInterval = Math.max(1, nextInterval + deltaTicks);
  }

  return tick;
}
