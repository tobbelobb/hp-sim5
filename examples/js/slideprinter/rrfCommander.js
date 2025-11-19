const STEP_CLOCK_HZ = 48_000_000 / 64; // 750 kHz step clock
const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);
const EXTRUDER_MM_PER_STEP = 1 / 95.922; // Matches M92 E95.922 from the RRF config

const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'];
const MOTOR_AXIS_MAP = new Map([
    [40, 'A'],
    [41, 'B'],
    [42, 'C'],
    [43, 'E'],
]);

async function* makeLineIterator(stream) {
    const reader = stream.pipeThrough(new TextDecoderStream()).getReader();
    let buffer = '';
    while (true) {
        const { value, done } = await reader.read();
        if (done) {
            if (buffer.length > 0) {
                yield buffer;
            }
            break;
        }
        buffer += value;
        let newlineIndex;
        while ((newlineIndex = buffer.indexOf('\n')) >= 0) {
            const line = buffer.slice(0, newlineIndex);
            buffer = buffer.slice(newlineIndex + 1);
            yield line;
        }
    }
}

function isFloatValue(num) {
    if (!Number.isFinite(num)) {
        return false;
    }
    return Math.abs(num - Math.trunc(num)) > 1e-9;
}

export class RrfCommander {
    constructor() {
        this.dt = 1 / 500;
        this.isPaused = false;
        this.resolveResume = null;
        this.accumulatedWaitMs = 0.0;
        this.ticksPerBucket = this._computeTicksPerBucket(this.dt);
        this.speedScale = 1.0;
        this.fastMode = false;
        this.asapMode = false;
        this.asapYieldCounter = 0;
        this.asapYieldInterval = 512;
        this.asapYieldBudgetMs = 24;
        this.lastYieldTime = 0;
        this.axisByMotorId = new Map();
        this.valueTypeByMotorId = new Map();
        this._resetState();
    }

    setDt(dt) {
        if (Number.isFinite(dt) && dt > 0) {
            this.dt = dt;
            this.ticksPerBucket = this._computeTicksPerBucket(this.dt);
        }
    }

    setSpeedScale(scale) {
        if (!Number.isFinite(scale) || scale <= 0) {
            this.speedScale = 1.0;
            return;
        }
        this.speedScale = scale;
    }

    setAsapMode(enable) {
        const next = Boolean(enable);
        if (this.asapMode === next) {
            return;
        }
        this.asapMode = next;
        this.asapYieldCounter = 0;
        this.accumulatedWaitMs = 0.0;
        this.lastYieldTime = performance.now();
    }

    async sendCommand(command) {
        postMessage({ action: 'gcode', command });
    }

    async _waitWhilePaused() {
        if (!this.isPaused) {
            return;
        }
        await new Promise(resolve => {
            this.resolveResume = resolve;
        });
        this.resolveResume = null;
    }

    _computeTicksPerBucket(dt) {
        return Math.max(1, Math.round(STEP_CLOCK_HZ * dt));
    }

    _targetWaitMs() {
        const scale = this.speedScale > 0 ? this.speedScale : 1.0;
        return (this.dt / scale) * 1000;
    }

    _resetState() {
        this.axisStates = new Map();
        this.spoolAxisOrder = [];
        this.axisAngles = new Map();
        this.bucketSteps = new Map();
        this.bucketExtrusion = new Map();
        this.bucketAddToReference = new Map();
        this.nextBucketToEmit = 0;
        this.maxBucketSeen = -1;
        this.accumulatedWaitMs = 0.0;
        this.asapYieldCounter = 0;
        this.lastYieldTime = 0;
        this.activeAxes = new Set();
        this.usedAxes = new Set();
        this.axisByMotorId.clear();
        this.valueTypeByMotorId.clear();
    }

    _ensureAxisState(axis) {
        if (!axis) {
            return null;
        }
        let state = this.axisStates.get(axis);
        if (!state) {
            state = {
                lastTick: 0,
                active: false,
                hasSteps: false,
            };
            this.axisStates.set(axis, state);
        }
        return state;
    }

    _ensureBucketMap(axis) {
        let map = this.bucketSteps.get(axis);
        if (!map) {
            map = new Map();
            this.bucketSteps.set(axis, map);
        }
        return map;
    }

    _assignAxisName(motorId) {
        if (this.axisByMotorId.has(motorId)) {
            return this.axisByMotorId.get(motorId);
        }
        let axis = MOTOR_AXIS_MAP.get(motorId);
        if (!axis || this.usedAxes.has(axis)) {
            axis = DEFAULT_AXIS_ORDER.find(candidate => !this.usedAxes.has(candidate));
        }
        if (!axis) {
            axis = `M${motorId}`;
        }
        this.axisByMotorId.set(motorId, axis);
        this.usedAxes.add(axis);
        if (axis !== 'E' && !this.spoolAxisOrder.includes(axis)) {
            this.spoolAxisOrder.push(axis);
        }
        this._ensureAxisState(axis);
        return axis;
    }

    _markAxisActive(axis) {
        const state = this._ensureAxisState(axis);
        if (!state) {
            return;
        }
        state.active = true;
        this.activeAxes.add(axis);
        if (!this.axisAngles.has(axis)) {
            this.axisAngles.set(axis, 0);
        }
    }

    _readyBucketThreshold(force = false) {
        if (force) {
            return this.maxBucketSeen + 1;
        }
        if (this.maxBucketSeen < 0 || this.activeAxes.size === 0) {
            return null;
        }
        let minBucket = Infinity;
        let hasBlockingAxis = false;
        for (const axis of this.activeAxes) {
            const state = this.axisStates.get(axis);
            if (!state) {
                continue;
            }
            if (!state.hasSteps && state.lastTick === 0) {
                continue;
            }
            hasBlockingAxis = true;
            const bucket = Math.floor(state.lastTick / this.ticksPerBucket);
            minBucket = Math.min(minBucket, bucket);
        }
        if (!hasBlockingAxis) {
            return this.maxBucketSeen + 1;
        }
        return minBucket;
    }

    async _flushReadyBuckets(force = false) {
        const threshold = this._readyBucketThreshold(force);
        if (threshold == null) {
            return;
        }
        const upperBound = Math.min(threshold, this.maxBucketSeen + 1);
        while (this.nextBucketToEmit < upperBound) {
            await this._emitBucket(this.nextBucketToEmit);
            this.nextBucketToEmit += 1;
        }
    }

    async _emitBucket(bucketIdx) {
        await this._waitWhilePaused();

        const loopStart = performance.now();

        const addRefEntry = this.bucketAddToReference.get(bucketIdx);
        if (addRefEntry) {
            const addCmd = { type: 'Add to reference' };
            let hasDelta = false;
            for (const [axis, delta] of Object.entries(addRefEntry)) {
                if (!Number.isFinite(delta) || delta === 0) {
                    continue;
                }
                addCmd[axis] = delta;
                hasDelta = true;
            }
            if (hasDelta) {
                await this.sendCommand(addCmd);
            }
            this.bucketAddToReference.delete(bucketIdx);
        }

        let changed = false;
        const moveCmd = { type: 'Move' };

        for (const axis of this.spoolAxisOrder) {
            const state = this.axisStates.get(axis);
            if (!this.axisAngles.has(axis)) {
                this.axisAngles.set(axis, state?.baseAngle || 0);
            }
            const axisMap = this.bucketSteps.get(axis);
            const deltaSteps = axisMap ? axisMap.get(bucketIdx) || 0 : 0;
            if (deltaSteps !== 0) {
                const current = this.axisAngles.get(axis) || 0;
                const newAngle = current + deltaSteps * STEP_ANGLE_RAD;
                this.axisAngles.set(axis, newAngle);
                changed = true;
            }
            moveCmd[axis] = this.axisAngles.get(axis) || 0;
            if (axisMap) {
                axisMap.delete(bucketIdx);
                if (axisMap.size === 0) {
                    this.bucketSteps.delete(axis);
                }
            }
        }

        const extrusionDelta = this.bucketExtrusion.get(bucketIdx) || 0;
        if (extrusionDelta !== 0) {
            moveCmd.E = extrusionDelta;
            changed = true;
        }
        if (this.bucketExtrusion.has(bucketIdx)) {
            this.bucketExtrusion.delete(bucketIdx);
        }

        if (!changed && bucketIdx === 0) {
            for (const axis of this.spoolAxisOrder) {
                if ((this.axisAngles.get(axis) || 0) !== 0) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed) {
            await this.sendCommand(moveCmd);
        }

        if (this.asapMode) {
            this.asapYieldCounter += 1;
            if (!this.lastYieldTime) {
                this.lastYieldTime = performance.now();
            }
            if (this.asapYieldCounter >= this.asapYieldInterval) {
                const now = performance.now();
                if (now - this.lastYieldTime >= this.asapYieldBudgetMs) {
                    await new Promise((resolve) => setTimeout(resolve, 0));
                    this.lastYieldTime = performance.now();
                    this.accumulatedWaitMs = 0.0;
                }
                this.asapYieldCounter = 0;
            }
            return;
        }

        const elapsedMs = performance.now() - loopStart;
        const waitMs = this._targetWaitMs() - elapsedMs;
        if (waitMs > 0) {
            this.accumulatedWaitMs += waitMs;
        }
        const waitScale = this.fastMode ? Math.max(1, this.speedScale) : 1;
        const thresholdMs = 10.0;

        if (this.accumulatedWaitMs > thresholdMs) {
            const sleepMs = Math.max(0, this.accumulatedWaitMs / waitScale);
            await new Promise((resolve) => setTimeout(resolve, sleepMs));
            this.accumulatedWaitMs = 0.0;
            this.lastYieldTime = performance.now();
        } else if (!this.lastYieldTime || performance.now() - this.lastYieldTime > 50) {
            await new Promise((resolve) => setTimeout(resolve, 0));
            this.lastYieldTime = performance.now();
        }
    }

    _recordBucketSteps(axis, bucketIdx, delta) {
        if (!Number.isFinite(delta) || delta === 0) {
            return;
        }
        const axisMap = this._ensureBucketMap(axis);
        axisMap.set(bucketIdx, (axisMap.get(bucketIdx) || 0) + delta);
        this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
    }

    _recordBucketExtrusion(bucketIdx, delta) {
        if (!Number.isFinite(delta) || delta === 0) {
            return;
        }
        this.bucketExtrusion.set(bucketIdx, (this.bucketExtrusion.get(bucketIdx) || 0) + delta);
        this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
    }

    _distributeSegment(startTick, durationTicks, totalValue, accumulate) {
        if (!Number.isFinite(totalValue) || totalValue === 0) {
            return;
        }
        const bucketSize = this.ticksPerBucket;
        if (!Number.isFinite(durationTicks) || durationTicks <= 0 || bucketSize <= 0) {
            const bucketIdx = Math.floor(startTick / bucketSize);
            accumulate(bucketIdx, totalValue);
            this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
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
        }
        this.maxBucketSeen = Math.max(this.maxBucketSeen, Math.floor((endTick - 1) / bucketSize));
    }

    _handleMovement(row) {
        const {
            motorId,
            whenToExecute,
            accelTicks,
            steadyTicks,
            decelTicks,
            steps,
        } = row;

        const axis = this._assignAxisName(motorId);
        const totalTicks = accelTicks + steadyTicks + decelTicks;
        const state = this._ensureAxisState(axis);
        if (state) {
            state.lastTick = Math.max(state.lastTick, whenToExecute + totalTicks);
        }

        if (!this.valueTypeByMotorId.has(motorId)) {
            this.valueTypeByMotorId.set(motorId, isFloatValue(steps) ? 'float' : 'int');
        }

        const axisType = this.valueTypeByMotorId.get(motorId);
        const startTick = whenToExecute;
        if (axis === 'E' || axisType === 'float') {
            const extrusionMm = steps * EXTRUDER_MM_PER_STEP;
            this._distributeSegment(startTick, totalTicks, extrusionMm, (bucketIdx, delta) => {
                this._recordBucketExtrusion(bucketIdx, delta);
            });
        } else {
            this._distributeSegment(startTick, totalTicks, steps, (bucketIdx, delta) => {
                this._recordBucketSteps(axis, bucketIdx, delta);
            });
        }

        if (state && steps !== 0) {
            state.hasSteps = true;
        }
        this._markAxisActive(axis);
    }

    _parseCsvRow(line) {
        const trimmed = line.trim();
        if (!trimmed || trimmed.startsWith('#')) {
            return null;
        }
        const parts = trimmed.split(',');
        if (parts.length < 7) {
            return null;
        }
        const motorId = Number(parts[1]);
        if (!Number.isFinite(motorId)) {
            return null;
        }
        const whenToExecute = Number(parts[2]);
        const accelTicks = Number(parts[3]);
        const steadyTicks = Number(parts[4]);
        const decelTicks = Number(parts[5]);
        const steps = Number(parts[6]);
        if (
            !Number.isFinite(whenToExecute)
            || !Number.isFinite(accelTicks)
            || !Number.isFinite(steadyTicks)
            || !Number.isFinite(decelTicks)
            || !Number.isFinite(steps)
        ) {
            return null;
        }
        return {
            motorId,
            whenToExecute,
            accelTicks,
            steadyTicks,
            decelTicks,
            steps,
        };
    }

    async _parseStream(stream) {
        const lineIterator = makeLineIterator(stream);
        let metadataHandled = false;

        for await (const rawLine of lineIterator) {
            const line = typeof rawLine === 'string' ? rawLine.trim() : '';
            if (!line) {
                continue;
            }
            if (!metadataHandled && line.startsWith('{')) {
                metadataHandled = true;
                continue;
            }
            metadataHandled = true;

            const movement = this._parseCsvRow(line);
            if (!movement) {
                continue;
            }
            this._handleMovement(movement);
            await this._flushReadyBuckets();
        }
    }

    async run(stream) {
        this._resetState();
        try {
            await this._parseStream(stream);
            await this._flushReadyBuckets(true);
            postMessage({ type: 'done' });
        } catch (e) {
            console.error('RrfCommander failed:', e);
            postMessage({ type: 'error', message: e?.message || String(e) });
        }
    }
}

const commander = new RrfCommander();

self.addEventListener('message', async (e) => {
    const { type } = e.data || {};
    switch (type) {
        case 'filename_upload': {
            const file = e.data.filename;
            if (!file || !file.stream) {
                break;
            }
            commander.run(file.stream());
            break;
        }
        case 'filename_fetch': {
            const filename = e.data.filename;
            if (!filename) {
                break;
            }
            try {
                const response = await fetch(filename);
                if (!response.ok || !response.body) {
                    throw new Error(`Failed to fetch ${filename}`);
                }
                await commander.run(response.body);
            } catch (err) {
                console.error('RrfCommander fetch failed:', err);
                postMessage({ type: 'error', message: err.message });
            }
            break;
        }
        case 'set_dt': {
            commander.setDt(e.data.dt);
            break;
        }
        case 'set_speed_scale': {
            commander.setSpeedScale(e.data.value);
            commander.accumulatedWaitMs = 0.0;
            break;
        }
        case 'set_asap_mode': {
            commander.setAsapMode(e.data.enable);
            break;
        }
        case 'set_fast_mode': {
            commander.fastMode = Boolean(e.data.enable);
            if (!commander.fastMode) {
                commander.accumulatedWaitMs = 0.0;
            }
            break;
        }
        case 'pause': {
            commander.isPaused = true;
            break;
        }
        case 'resume': {
            commander.isPaused = false;
            if (commander.resolveResume) {
                commander.resolveResume();
            }
            break;
        }
        default:
            break;
    }
});

console.log('worker: RrfCommander ready');
