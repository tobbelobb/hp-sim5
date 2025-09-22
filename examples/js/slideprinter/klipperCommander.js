import { detectFileFormat, FileFormat, isMcuFormat } from './fileFormatUtils.js';
import { iterateSerialLines, createKlipperSerialDecoder } from './klipperSerialParser.js';

const serialDecoder = createKlipperSerialDecoder();

const STEP_PIN_AXIS_MAP = {
    'gpiochip1/gpio0': 'A',
    'gpiochip1/gpio3': 'B',
    'gpiochip1/gpio6': 'C',
    'gpiochip1/gpio9': 'D',
};

const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E'];
const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16); // 200 steps/rev, 16 microsteps
const MCU_CLOCK_HZ = 50_000_000;
const EXTRUDER_MM_PER_STEP = 33.5 / (200 * 16);

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

function parseKvPairs(text) {
    const out = {};
    for (const part of text.trim().split(/\s+/)) {
        const match = part.match(/^([a-zA-Z_]+)=(.+)$/);
        if (!match) {
            continue;
        }
        const [, key, rawVal] = match;
        const num = Number(rawVal);
        out[key] = Number.isNaN(num) ? rawVal : num;
    }
    return out;
}

class KlipperCommander {
    constructor() {
        this.dt = 1 / 500; // Default to 500 Hz pacing
        this.isPaused = false;
        this.resolveResume = null;
        this.accumulatedWaitMs = 0.0;
        this.ticksPerBucket = this._computeTicksPerBucket(this.dt);
        this._resetState();
    }

    setDt(dt) {
        if (Number.isFinite(dt) && dt > 0) {
            this.dt = dt;
            this.ticksPerBucket = this._computeTicksPerBucket(this.dt);
        }
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
        return Math.max(1, Math.round(MCU_CLOCK_HZ * dt));
    }

    _resetState() {
        this.axisByOid = new Map();
        this.usedAxes = new Set();
        this.axisStates = new Map();
        this.spoolAxisOrder = [];
        this.activeAxes = new Set();
        this.bucketSteps = new Map();
        this.bucketExtrusion = new Map();
        this.bucketAddToReference = new Map();
        this.axisAngles = new Map();
        this.nextBucketToEmit = 0;
        this.maxBucketSeen = -1;
        this.accumulatedWaitMs = 0.0;
    }

    _ensureAxisState(axis) {
        if (!axis) {
            return null;
        }
        let state = this.axisStates.get(axis);
        if (!state) {
            state = {
                dir: 1,
                lastTick: 0,
                baseAngle: 0,
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

    _markAxisActive(axis) {
        const state = this._ensureAxisState(axis);
        if (!state) {
            return;
        }
        state.active = true;
        this.activeAxes.add(axis);
        if (!this.axisAngles.has(axis)) {
            this.axisAngles.set(axis, state.baseAngle || 0);
        }
    }

    _readyBucketThreshold(force = false) {
        if (force) {
            return this.maxBucketSeen + 1;
        }
        if (this.maxBucketSeen < 0) {
            return null;
        }
        if (this.activeAxes.size === 0) {
            return null;
        }
        let hasBlockingAxis = false;
        let minBucket = Infinity;
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
            if (bucket < minBucket) {
                minBucket = bucket;
            }
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

        const elapsedMs = performance.now() - loopStart;
        const waitMs = this.dt * 1000 - elapsedMs;
        if (waitMs > 0) {
            this.accumulatedWaitMs += waitMs;
        }
        if (this.accumulatedWaitMs > 10.0) {
            await new Promise((resolve) => setTimeout(resolve, this.accumulatedWaitMs));
            this.accumulatedWaitMs = 0.0;
        }
    }

    async _parseStream(stream, format = FileFormat.MCU_TEXT) {
        const lineIterator = format === FileFormat.MCU_SERIAL
            ? iterateSerialLines(stream, serialDecoder)
            : makeLineIterator(stream);

        for await (const rawLine of lineIterator) {
            const line = typeof rawLine === 'string' ? rawLine.trim() : '';
            if (line.length === 0 || line.startsWith('#')) {
                continue;
            }

            if (line.startsWith('config_stepper')) {
                const kv = parseKvPairs(line.slice('config_stepper'.length));
                const oid = kv.oid;
                if (oid == null) {
                    continue;
                }
                let axis = STEP_PIN_AXIS_MAP[kv.step_pin];
                if (!axis || this.usedAxes.has(axis)) {
                    axis = DEFAULT_AXIS_ORDER.find((candidate) => !this.usedAxes.has(candidate)) || null;
                }
                if (axis) {
                    this.usedAxes.add(axis);
                    this.axisByOid.set(oid, axis);
                    if (axis !== 'E' && !this.spoolAxisOrder.includes(axis)) {
                        this.spoolAxisOrder.push(axis);
                    }
                    this._ensureAxisState(axis);
                }
                continue;
            }

            if (line.startsWith('set_next_step_dir')) {
                const kv = parseKvPairs(line.slice('set_next_step_dir'.length));
                const axis = this.axisByOid.get(kv.oid);
                if (!axis) {
                    continue;
                }
                const state = this._ensureAxisState(axis);
                state.dir = (Number(kv.dir) === 0) ? -1 : 1;
                continue;
            }

            if (line.startsWith('set_position')) {
                const kv = parseKvPairs(line.slice('set_position'.length));
                const axis = this.axisByOid.get(kv.oid);
                if (!axis) {
                    continue;
                }
                const state = this._ensureAxisState(axis);
                const newAngle = Number(kv.pos || 0) * STEP_ANGLE_RAD;
                const delta = newAngle - state.baseAngle;
                if (delta !== 0) {
                    const bucketIdx = Math.floor(state.lastTick / this.ticksPerBucket);
                    const entry = this.bucketAddToReference.get(bucketIdx) || {};
                    entry[axis] = (entry[axis] || 0) + delta;
                    this.bucketAddToReference.set(bucketIdx, entry);
                    this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
                }
                state.baseAngle = newAngle;
                this._markAxisActive(axis);
                await this._flushReadyBuckets();
                continue;
            }

            if (line.startsWith('queue_step')) {
                const kv = parseKvPairs(line.slice('queue_step'.length));
                const axis = this.axisByOid.get(kv.oid);
                if (!axis) {
                    continue;
                }
                const state = this._ensureAxisState(axis);
                let interval = Number(kv.interval) || 1;
                const count = Number(kv.count) || 0;
                const add = ('add' in kv) ? Number(kv.add) || 0 : 0;
                if (count <= 0) {
                    continue;
                }
                for (let i = 0; i < count; i += 1) {
                    interval = Math.max(1, interval);
                    state.lastTick += interval;
                    const bucketIdx = Math.floor(state.lastTick / this.ticksPerBucket);
                    this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
                    if (axis === 'E') {
                        const current = this.bucketExtrusion.get(bucketIdx) || 0;
                        this.bucketExtrusion.set(bucketIdx, current + state.dir * EXTRUDER_MM_PER_STEP);
                    } else {
                        const bucketMap = this._ensureBucketMap(axis);
                        bucketMap.set(bucketIdx, (bucketMap.get(bucketIdx) || 0) + state.dir);
                    }
                    interval = Math.max(1, interval + add);
                }
                state.hasSteps = true;
                this._markAxisActive(axis);
                await this._flushReadyBuckets();
                continue;
            }
        }
    }

    async run(stream, format = FileFormat.MCU_TEXT) {
        this._resetState();
        try {
            await this._parseStream(stream, format);
            await this._flushReadyBuckets(true);
            postMessage({ type: 'done' });
        } catch (e) {
            console.error('KlipperCommander failed:', e);
            postMessage({ type: 'error', message: e?.message || String(e) });
        }
    }
}

const commander = new KlipperCommander();

self.addEventListener('message', async (e) => {
    const { type } = e.data || {};
    switch (type) {
        case 'filename_upload': {
            const file = e.data.filename;
            if (!file || !file.stream) {
                break;
            }
            const format = detectFileFormat(file.name);
            if (!isMcuFormat(format)) {
                postMessage({ type: 'error', message: 'Unsupported file type for KlipperCommander' });
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
                const format = detectFileFormat(filename) || FileFormat.MCU_TEXT;
                await commander.run(response.body, format);
            } catch (err) {
                console.error('KlipperCommander fetch failed:', err);
                postMessage({ type: 'error', message: err.message });
            }
            break;
        }
        case 'set_dt': {
            commander.setDt(e.data.dt);
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

console.log('worker: KlipperCommander ready');
