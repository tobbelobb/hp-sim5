export function isFloatValue(num) {
    if (!Number.isFinite(num)) {
        return false;
    }
    return Math.abs(num - Math.trunc(num)) > 1e-9;
}

export function createMotionProfile(row) {
    const { accelTicks, steadyTicks, decelTicks, acceleration, deceleration } = row;
    const totalTicks = accelTicks + steadyTicks + decelTicks;
    if (
        totalTicks <= 0
        || !Number.isFinite(acceleration)
        || !Number.isFinite(deceleration)
    ) {
        return null;
    }
    const accelDistanceExTopSpeed = -0.5 * acceleration * accelTicks * accelTicks;
    const decelDistanceExTopSpeed = -0.5 * deceleration * decelTicks * decelTicks;
    const denom = totalTicks;
    if (denom === 0) {
        return null;
    }
    const topSpeed = (1 - accelDistanceExTopSpeed - decelDistanceExTopSpeed) / denom;
    if (!Number.isFinite(topSpeed)) {
        return null;
    }
    const startSpeed = topSpeed - acceleration * accelTicks;
    const accelDistance = accelDistanceExTopSpeed + topSpeed * accelTicks;
    const decelDistance = decelDistanceExTopSpeed + topSpeed * decelTicks;
    let steadyDistance = 1 - accelDistance - decelDistance;
    if (!Number.isFinite(steadyDistance)) {
        steadyDistance = 0;
    }
    if (steadyDistance < 0 && Math.abs(steadyDistance) < 1e-9) {
        steadyDistance = 0;
    }
    const decelStartDistance = accelDistance + steadyDistance;
    const profile = {
        totalTicks,
        accelTicks,
        steadyTicks,
        decelTicks,
        acceleration,
        deceleration,
        startSpeed,
        topSpeed,
        accelDistance,
        steadyDistance,
        decelStartDistance,
    };
    profile.positionAt = (localTime) => {
        if (!Number.isFinite(localTime)) {
            return 0;
        }
        const clamped = Math.min(Math.max(localTime, 0), totalTicks);
        if (clamped <= accelTicks) {
            const distance = profile.startSpeed * clamped + 0.5 * profile.acceleration * clamped * clamped;
            return Math.min(1, Math.max(0, distance));
        }
        if (clamped <= accelTicks + steadyTicks) {
            const t = clamped - accelTicks;
            const distance = profile.accelDistance + profile.topSpeed * t;
            return Math.min(1, Math.max(0, distance));
        }
        const t = clamped - accelTicks - steadyTicks;
        const distance = profile.decelStartDistance + profile.topSpeed * t - 0.5 * profile.deceleration * t * t;
        return Math.min(1, Math.max(0, distance));
    };
    return profile;
}

export function distributeEvenly({
    startTick,
    durationTicks,
    totalValue,
    bucketSize,
    accumulate,
    setMaxBucket,
}) {
    if (!Number.isFinite(totalValue) || totalValue === 0) {
        return;
    }
    if (!Number.isFinite(durationTicks) || durationTicks <= 0 || !Number.isFinite(bucketSize) || bucketSize <= 0) {
        const bucketIdx = Math.floor(startTick / bucketSize);
        accumulate(bucketIdx, totalValue);
        if (setMaxBucket) {
            setMaxBucket(bucketIdx);
        }
        return;
    }

    const endTick = startTick + durationTicks;
    const valuePerTick = totalValue / durationTicks;
    let bucketIdx = Math.floor(startTick / bucketSize);
    let assigned = 0;
    while (true) {
        const bucketStart = bucketIdx * bucketSize;
        const bucketEnd = bucketStart + bucketSize;
        const overlapStart = Math.max(startTick, bucketStart);
        const overlapEnd = Math.min(endTick, bucketEnd);
        if (overlapEnd > overlapStart) {
            const ticks = overlapEnd - overlapStart;
            const delta = valuePerTick * ticks;
            accumulate(bucketIdx, delta);
            assigned += delta;
        }
        if (overlapEnd >= endTick) {
            break;
        }
        bucketIdx += 1;
    }
    const residual = totalValue - assigned;
    if (Math.abs(residual) > 1e-6) {
        const tailBucket = Math.floor((endTick - 1) / bucketSize);
        accumulate(tailBucket, residual);
        if (setMaxBucket) {
            setMaxBucket(tailBucket);
        }
    }
    if (setMaxBucket) {
        setMaxBucket(Math.floor((endTick - 1) / bucketSize));
    }
}

export function distributeWithProfile({
    startTick,
    profile,
    bucketSize,
    accumulateNormalized,
    setMaxBucket,
}) {
    if (!profile || !Number.isFinite(bucketSize) || bucketSize <= 0) {
        return;
    }
    const totalTicks = profile.totalTicks;
    const moveEndTick = startTick + totalTicks;
    const startBucket = Math.floor(startTick / bucketSize);
    const endBucket = Math.floor((moveEndTick - 1) / bucketSize);
    for (let bucket = startBucket; bucket <= endBucket; bucket += 1) {
        const bucketStart = bucket * bucketSize;
        const bucketEnd = bucketStart + bucketSize;
        const overlapStart = Math.max(bucketStart, startTick);
        const overlapEnd = Math.min(bucketEnd, moveEndTick);
        if (overlapEnd <= overlapStart) {
            continue;
        }
        const localStart = overlapStart - startTick;
        const localEnd = overlapEnd - startTick;
        const posStart = profile.positionAt(localStart);
        const posEnd = profile.positionAt(localEnd);
        const delta = posEnd - posStart;
        if (Math.abs(delta) > 1e-12) {
            accumulateNormalized(bucket, delta);
        }
    }
    if (moveEndTick > startTick && setMaxBucket) {
        const maxBucket = Math.floor((moveEndTick - 1) / bucketSize);
        setMaxBucket(maxBucket);
    }
}
