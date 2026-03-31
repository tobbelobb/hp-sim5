import { detectFileFormat, FileFormat, isRrfFormat } from './fileFormatUtils.js';
import {
    computeTicksPerBucket,
    createMotionProfile,
    distributeEvenly,
    distributeWithProfile,
    isFloatValue,
    EXTRUDER_MM_PER_STEP,
} from '../../../bridges/rrf/rrfMotionUtils.js';

const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);

const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E', 'I', 'J', 'K', 'L', 'O'];
const MOTOR_AXIS_MAP = new Map([
    [40, 'A'],
    [41, 'B'],
    [42, 'C'],
    [43, 'D'], // D has been injected at position 43 here, instead of E, without testing if that causes problems or not. Just because ABCDEIJKLO is the new default order of can addresses
    [44, 'E'],
    [45, 'I'],
    [46, 'J'],
    [47, 'K'],
    [48, 'L'],
    [49, 'O'],
]);

const UINT32_MULTIPLIER = 0x1_0000_0000;

function readUint64LEAsNumber(view, offset) {
    const lower = view.getUint32(offset, true);
    const upper = view.getUint32(offset + 4, true);
    const value = upper * UINT32_MULTIPLIER + lower;
    if (!Number.isSafeInteger(value)) {
        throw new Error('CAN timestamp exceeds supported precision');
    }
    return value;
}

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
        this.baselineEmitted = false;
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
        return computeTicksPerBucket(dt);
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
        this.baselineEmitted = false;
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

        // If nothing has been emitted yet, ensure we send an explicit baseline at bucket 0
        if (!changed && bucketIdx === 0 && !this.baselineEmitted) {
            changed = true;
        }

        if (changed) {
            if (bucketIdx === 0) {
                this.baselineEmitted = true;
            }
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

    _distributeEvenly(startTick, durationTicks, totalValue, accumulate) {
        distributeEvenly({
            startTick,
            durationTicks,
            totalValue,
            bucketSize: this.ticksPerBucket,
            accumulate,
            setMaxBucket: (bucketIdx) => {
                this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
            },
        });
    }

    _createMotionProfile(row) {
        return createMotionProfile(row);
    }

    _distributeWithProfile(startTick, profile, accumulateNormalized) {
        distributeWithProfile({
            startTick,
            profile,
            bucketSize: this.ticksPerBucket,
            accumulateNormalized,
            setMaxBucket: (bucket) => {
                this.maxBucketSeen = Math.max(this.maxBucketSeen, bucket);
            },
        });
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
        const profile = this._createMotionProfile(row);
        const useProfile = profile && profile.totalTicks > 0;
        const distribute = (bucketIdx, normalizedDelta) => {
            const deltaValue = steps * normalizedDelta;
            if (axis === 'E' || axisType === 'float') {
                this._recordBucketExtrusion(bucketIdx, deltaValue * EXTRUDER_MM_PER_STEP);
            } else {
                this._recordBucketSteps(axis, bucketIdx, deltaValue);
            }
        };
        if (useProfile) {
            this._distributeWithProfile(startTick, profile, distribute);
        } else if (axis === 'E' || axisType === 'float') {
            const extrusionMm = steps * EXTRUDER_MM_PER_STEP;
            this._distributeEvenly(startTick, totalTicks, extrusionMm, (bucketIdx, delta) => {
                this._recordBucketExtrusion(bucketIdx, delta);
            });
        } else {
            this._distributeEvenly(startTick, totalTicks, steps, (bucketIdx, delta) => {
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
        if (parts.length < 9) {
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
        const acceleration = Number(parts[7]);
        const deceleration = Number(parts[8]);
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
            acceleration,
            deceleration,
        };
    }

    _parseTorqueRow(line) {
        if (!line || !line.startsWith('T,')) {
            return null;
        }
        const parts = line.split(',');
        if (parts.length < 3) {
            return null;
        }
        const motorId = Number(parts[1]);
        const torqueNm = Number(parts[2]);
        if (!Number.isFinite(motorId) || !Number.isFinite(torqueNm)) {
            return null;
        }
        return { motorId, torqueNm };
    }

    async _handleTorque(row) {
        const { motorId, torqueNm } = row;
        const axis = this._assignAxisName(motorId);
        if (!axis) {
            return;
        }
        const isPositionMode = Math.abs(torqueNm) < 1e-9;
        const command = isPositionMode
            ? { type: 'SetPositionMode', axis, driver: motorId }
            : { type: 'SetTorqueMode', axis, driver: motorId, torqueNm };
        await this.sendCommand(command);
    }

    async _consumeCsvLines(lineIterable) {
        let metadataHandled = false;
        let lastWhen = null;

        for await (const rawLine of lineIterable) {
            const line = typeof rawLine === 'string' ? rawLine.trim() : '';
            if (!line) {
                continue;
            }
            if (!metadataHandled && line.startsWith('{')) {
                metadataHandled = true;
                continue;
            }
            metadataHandled = true;

            const torque = this._parseTorqueRow(line);
            if (torque) {
                await this._flushReadyBuckets();
                await this._handleTorque(torque);
                continue;
            }

            const movement = this._parseCsvRow(line);
            if (!movement) {
                continue;
            }
            if (lastWhen !== null && movement.whenToExecute !== lastWhen) {
                await this._flushReadyBuckets();
            }
            this._handleMovement(movement);
            lastWhen = movement.whenToExecute;
        }
        await this._flushReadyBuckets(true);
    }

    async _parseCsvStream(stream) {
        const lineIterator = makeLineIterator(stream);
        await this._consumeCsvLines(lineIterator);
    }

    async _readStreamToArrayBuffer(stream) {
        if (!stream || typeof stream.getReader !== 'function') {
            return new ArrayBuffer(0);
        }
        const reader = stream.getReader();
        const chunks = [];
        let totalLength = 0;
        try {
            while (true) {
                const { value, done } = await reader.read();
                if (done) {
                    break;
                }
                if (value == null) {
                    continue;
                }
                let chunk;
                if (value instanceof Uint8Array) {
                    chunk = value;
                } else if (value instanceof ArrayBuffer) {
                    chunk = new Uint8Array(value);
                } else {
                    continue;
                }
                chunks.push(chunk);
                totalLength += chunk.byteLength;
            }
        } finally {
            if (reader.releaseLock) {
                reader.releaseLock();
            }
        }
        if (chunks.length === 0) {
            return new ArrayBuffer(0);
        }
        if (chunks.length === 1) {
            const [single] = chunks;
            if (single.byteOffset === 0 && single.byteLength === single.buffer.byteLength) {
                return single.buffer;
            }
        }
        const merged = new Uint8Array(totalLength);
        let offset = 0;
        for (const chunk of chunks) {
            merged.set(chunk, offset);
            offset += chunk.byteLength;
        }
        return merged.buffer;
    }

    async _parseBinaryStream(stream) {
        const buffer = await this._readStreamToArrayBuffer(stream);
        if (!buffer || buffer.byteLength === 0) {
            return;
        }
        const view = new DataView(buffer);
        const totalLength = view.byteLength;
        let offset = 0;

        const ensureAvailable = (size) => {
            if (offset + size > totalLength) {
                throw new Error('Unexpected end of CAN data');
            }
        };

        let ctxTime = 0;
        let ctxCol3 = 0;
        let ctxCol4 = 0;
        let ctxCol5 = 0;
        let ctxAccel = 0.0;
        let ctxDecel = 0.0;
        let lastWhen = null;

        while (offset < totalLength) {
            ensureAvailable(2);
            const mask = view.getUint8(offset++);
            const count = view.getUint8(offset++);

            if (mask & 1) {
                ensureAvailable(8);
                ctxTime = readUint64LEAsNumber(view, offset);
                offset += 8;
            }
            if (mask & 2) {
                ensureAvailable(4);
                ctxCol3 = view.getInt32(offset, true);
                offset += 4;
            }
            if (mask & 4) {
                ensureAvailable(4);
                ctxCol4 = view.getInt32(offset, true);
                offset += 4;
            }
            if (mask & 8) {
                ensureAvailable(4);
                ctxCol5 = view.getInt32(offset, true);
                offset += 4;
            }
            if (mask & 16) {
                ensureAvailable(4);
                ctxAccel = view.getFloat32(offset, true);
                offset += 4;
            }
            if (mask & 32) {
                ensureAvailable(4);
                ctxDecel = view.getFloat32(offset, true);
                offset += 4;
            }

            for (let i = 0; i < count; i += 1) {
                ensureAvailable(1);
                const headerByte = view.getUint8(offset++);
                const typeCode = (headerByte >> 6) & 0x03;
                const canId = headerByte & 0x3F;
                let value;
                switch (typeCode) {
                    case 0:
                        ensureAvailable(1);
                        value = view.getInt8(offset);
                        offset += 1;
                        break;
                    case 1:
                        ensureAvailable(2);
                        value = view.getInt16(offset, true);
                        offset += 2;
                        break;
                    case 2:
                        ensureAvailable(4);
                        value = view.getInt32(offset, true);
                        offset += 4;
                        break;
                    case 3:
                        ensureAvailable(4);
                        value = view.getFloat32(offset, true);
                        offset += 4;
                        break;
                    default:
                        throw new Error(`Unsupported CAN type code: ${typeCode}`);
                }
                const whenToExecute = ctxTime;
                if (lastWhen !== null && whenToExecute !== lastWhen) {
                    await this._flushReadyBuckets();
                }
                const movement = {
                    motorId: canId,
                    whenToExecute,
                    accelTicks: ctxCol3,
                    steadyTicks: ctxCol4,
                    decelTicks: ctxCol5,
                    steps: value,
                    acceleration: ctxAccel,
                    deceleration: ctxDecel,
                };
                this._handleMovement(movement);
                lastWhen = whenToExecute;
            }
        }
        await this._flushReadyBuckets(true);
    }

    async run(stream, format = FileFormat.RRF_CAN) {
        this._resetState();
        try {
            if (format === FileFormat.RRF_CAN_BINARY) {
                await this._parseBinaryStream(stream);
            } else {
                await this._parseCsvStream(stream);
            }
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
            const format = detectFileFormat(file.name);
            if (!isRrfFormat(format)) {
                postMessage({ type: 'error', message: 'Unsupported file type for RrfCommander' });
                break;
            }
            commander.run(file.stream(), format);
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
                const format = detectFileFormat(filename) || FileFormat.RRF_CAN;
                await commander.run(response.body, isRrfFormat(format) ? format : FileFormat.RRF_CAN);
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
        case 'reset_state': {
            commander._resetState();
            break;
        }
        case 'append_csv': {
            {
                const lines = Array.isArray(e.data.lines)
                    ? e.data.lines
                    : (typeof e.data.lines === 'string' ? e.data.lines.split('\n') : []);
                if (lines.length > 0) {
                    const iterable = (async function* () {
                        for (const line of lines) {
                            yield line;
                        }
                    }());
                    await commander._consumeCsvLines(iterable);
                }
            }
            break;
        }
        default:
            break;
    }
});

console.log('worker: RrfCommander ready');
