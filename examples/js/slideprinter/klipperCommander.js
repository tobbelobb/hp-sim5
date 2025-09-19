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
    }

    setDt(dt) {
        if (Number.isFinite(dt) && dt > 0) {
            this.dt = dt;
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

    _ticksPerBucket() {
        return Math.max(1, Math.round(MCU_CLOCK_HZ * this.dt));
    }

    async _parseStream(stream, format = FileFormat.MCU_TEXT) {
        const lineIterator = format === FileFormat.MCU_SERIAL
            ? iterateSerialLines(stream, serialDecoder)
            : makeLineIterator(stream);
        const axisByOid = new Map();
        const axisStates = new Map(); // axis -> { dir, lastTick, baseAngle }
        const spoolAxisOrder = [];
        const bucketSteps = new Map(); // axis -> Map<bucketIdx, stepDelta>
        const bucketExtrusion = new Map(); // bucketIdx -> extrusion mm
        const bucketAddToReference = new Map(); // bucketIdx -> { axis: deltaAngle }
        let maxBucket = 0;
        const usedAxes = new Set();
        const ticksPerBucket = this._ticksPerBucket();

        const ensureAxisState = (axis) => {
            if (!axisStates.has(axis)) {
                axisStates.set(axis, { dir: 1, lastTick: 0, baseAngle: 0 });
            }
            return axisStates.get(axis);
        };

        const ensureBucketMap = (axis) => {
            if (!bucketSteps.has(axis)) {
                bucketSteps.set(axis, new Map());
            }
            return bucketSteps.get(axis);
        };

        const registerAxis = (axis) => {
            if (axis !== 'E' && !spoolAxisOrder.includes(axis)) {
                spoolAxisOrder.push(axis);
            }
            ensureAxisState(axis);
        };

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
                if (!axis || usedAxes.has(axis)) {
                    axis = DEFAULT_AXIS_ORDER.find(candidate => !usedAxes.has(candidate)) || null;
                }
                if (axis) {
                    usedAxes.add(axis);
                    axisByOid.set(oid, axis);
                    registerAxis(axis);
                }
                continue;
            }

            if (line.startsWith('set_next_step_dir')) {
                const kv = parseKvPairs(line.slice('set_next_step_dir'.length));
                const axis = axisByOid.get(kv.oid);
                if (!axis) {
                    continue;
                }
                const state = ensureAxisState(axis);
                state.dir = (Number(kv.dir) === 0) ? -1 : 1;
                continue;
            }

            if (line.startsWith('set_position')) {
                const kv = parseKvPairs(line.slice('set_position'.length));
                const axis = axisByOid.get(kv.oid);
                if (!axis) {
                    continue;
                }
                const state = ensureAxisState(axis);
                const newAngle = Number(kv.pos || 0) * STEP_ANGLE_RAD;
                const delta = newAngle - state.baseAngle;
                if (delta !== 0) {
                    const bucketIdx = Math.floor(state.lastTick / ticksPerBucket);
                    const entry = bucketAddToReference.get(bucketIdx) || {};
                    entry[axis] = (entry[axis] || 0) + delta;
                    bucketAddToReference.set(bucketIdx, entry);
                    state.baseAngle = newAngle;
                }
                continue;
            }

            if (line.startsWith('queue_step')) {
                const kv = parseKvPairs(line.slice('queue_step'.length));
                const axis = axisByOid.get(kv.oid);
                if (!axis) {
                    continue;
                }
                const state = ensureAxisState(axis);
                let interval = Number(kv.interval) || 1;
                const count = Number(kv.count) || 0;
                const add = ('add' in kv) ? Number(kv.add) || 0 : 0;
                if (count <= 0) {
                    continue;
                }
                const bucketMap = ensureBucketMap(axis);
                for (let i = 0; i < count; i += 1) {
                    interval = Math.max(1, interval);
                    state.lastTick += interval;
                    const bucketIdx = Math.floor(state.lastTick / ticksPerBucket);
                    maxBucket = Math.max(maxBucket, bucketIdx);
                    if (axis === 'E') {
                        const current = bucketExtrusion.get(bucketIdx) || 0;
                        bucketExtrusion.set(bucketIdx, current + state.dir * EXTRUDER_MM_PER_STEP);
                    } else {
                        const prev = bucketMap.get(bucketIdx) || 0;
                        bucketMap.set(bucketIdx, prev + state.dir);
                    }
                    interval = Math.max(1, interval + add);
                }
                continue;
            }
        }

        return {
            spoolAxisOrder,
            axisStates,
            bucketSteps,
            bucketExtrusion,
            bucketAddToReference,
            maxBucket,
        };
    }

    async _emitTimeline({
        spoolAxisOrder,
        axisStates,
        bucketSteps,
        bucketExtrusion,
        bucketAddToReference,
        maxBucket,
    }) {
        if (spoolAxisOrder.length === 0 && bucketExtrusion.size === 0) {
            return;
        }

        this.accumulatedWaitMs = 0.0;

        const axisAngles = {};
        for (const axis of spoolAxisOrder) {
            axisAngles[axis] = axisStates.get(axis)?.baseAngle || 0;
        }

        const addRefMax = bucketAddToReference.size > 0 ? Math.max(...bucketAddToReference.keys()) : 0;
        const totalBuckets = Math.max(maxBucket, addRefMax);

        for (let bucketIdx = 0; bucketIdx <= totalBuckets; bucketIdx += 1) {
            await this._waitWhilePaused();

            const loopStart = performance.now();

            const addRefEntry = bucketAddToReference.get(bucketIdx);
            if (addRefEntry) {
                const addCmd = { type: 'Add to reference' };
                let hasDelta = false;
                for (const axis of Object.keys(addRefEntry)) {
                    const delta = addRefEntry[axis];
                    if (!Number.isFinite(delta) || delta === 0) {
                        continue;
                    }
                    addCmd[axis] = delta;
                    hasDelta = true;
                }
                if (hasDelta) {
                    await this.sendCommand(addCmd);
                }
            }

            let changed = false;
            const moveCmd = { type: 'Move' };

            for (const axis of spoolAxisOrder) {
                const axisBucket = bucketSteps.get(axis);
                const deltaSteps = axisBucket ? (axisBucket.get(bucketIdx) || 0) : 0;
                if (deltaSteps !== 0) {
                    axisAngles[axis] += deltaSteps * STEP_ANGLE_RAD;
                    changed = true;
                }
                moveCmd[axis] = axisAngles[axis];
            }

            const extrusionDelta = bucketExtrusion.get(bucketIdx) || 0;
            if (extrusionDelta !== 0) {
                moveCmd.E = extrusionDelta;
                changed = true;
            }

            if (!changed && bucketIdx === 0) {
                changed = spoolAxisOrder.some(axis => axisAngles[axis] !== 0);
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
                await new Promise(resolve => setTimeout(resolve, this.accumulatedWaitMs));
                this.accumulatedWaitMs = 0.0;
            }
        }
    }

    async run(stream, format = FileFormat.MCU_TEXT) {
        try {
            const parsed = await this._parseStream(stream, format);
            await this._emitTimeline(parsed);
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
