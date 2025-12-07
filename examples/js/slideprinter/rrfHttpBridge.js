import {
    computeTicksPerBucket,
    createMotionProfile,
    distributeEvenly,
    distributeWithProfile,
    isFloatValue,
    EXTRUDER_MM_PER_STEP,
} from './rrfMotionUtils.js';

const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16); // 1.8deg motor, 16x microstepping

export class RrfHttpBridge {
    constructor(options = {}) {
        this.baseUrl = options.baseUrl || 'http://localhost:8080';
        this.remoteSpoolSystem = options.remoteSpoolSystem || null;
        this.onTorqueModeChange = typeof options.onTorqueModeChange === 'function' ? options.onTorqueModeChange : null;
        this.onGCodeReply = typeof options.onGCodeReply === 'function' ? options.onGCodeReply : null;
        this.driverToAxis = options.driverToAxis || {
            40: 'A',
            41: 'B',
            42: 'C',
            43: 'D',
        };
        this.fetchImpl = options.fetchImpl || (typeof fetch !== 'undefined' ? fetch.bind(globalThis) : null);
        if (!this.fetchImpl) {
            throw new Error('Fetch implementation is not available in this environment.');
        }
        this.encoderResolver = typeof options.encoderResolver === 'function' ? options.encoderResolver : null;

        this.dt = Number.isFinite(options.dt) && options.dt > 0 ? options.dt : 1 / 500;
        this.ticksPerBucket = computeTicksPerBucket(this.dt);
        this.valueTypeByMotorId = new Map();
        this.axisStates = new Map();
        this.activeAxes = new Set();
        this.spoolAxisOrder = [];
        this.bucketSteps = new Map();
        this.bucketExtrusion = new Map();
        this.bucketAddToReference = new Map();
        this.maxBucketSeen = -1;
        this.nextBucketToEmit = 0;
        this.baselineEmitted = false;

        this._pendingMotion = [];
        this._axisAngles = new Map();
        this._driverDirections = new Map();
        this._directionFetches = new Map();
        this._encoderReferences = new Map();
    }

    async sendGCode(gcode, options = {}) {
        const {
            timeout = 30000,
            suppressMotionProcessing = false,
            suppressCallbacks = false,
        } = options;

        if (this._isEncoderQuery(gcode) && this.encoderResolver) {
            return this._handleEncoderQuery(gcode, { timeout, suppressCallbacks });
        }

        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), timeout);

        try {
            const response = await this.fetchImpl(`${this.baseUrl}/machine/code`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'text/plain',
                },
                body: gcode,
                signal: controller.signal,
            });
            clearTimeout(timeoutId);

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const text = await response.text();
            const parsed = this._parseResponse(text);
            this._injectTorqueModeFallback(parsed, gcode);

            if (!suppressMotionProcessing && parsed.motion.length > 0) {
                const torqueItems = parsed.motion.filter((item) => item?.type === 'TorqueMode');
                if (torqueItems.length > 0) {
                    await this._primeDriverDirections(torqueItems);
                }
                this._processMotion(parsed.motion);
            }

            if (!suppressCallbacks && this.onGCodeReply) {
                try {
                    this.onGCodeReply(gcode, parsed.reply);
                } catch (_err) {
                    // Ignore listener errors
                }
            }

            this._maybeCacheDirectionFromReply(gcode, parsed.reply);
            return parsed;
        } catch (error) {
            clearTimeout(timeoutId);
            if (error.name === 'AbortError') {
                throw new Error(`G-code command timed out after ${timeout}ms`);
            }
            throw error;
        }
    }

    async _primeDriverDirections(motionItems) {
        const drivers = new Map();
        for (const item of motionItems) {
            if (!item || item.type !== 'TorqueMode') {
                continue;
            }
            const descriptor = this._getTorqueMotorDescriptor(item);
            if (!descriptor) {
                continue;
            }
            const key = this._motorDescriptorKey(descriptor);
            if (!key || drivers.has(key)) {
                continue;
            }
            drivers.set(key, descriptor);
        }

        if (drivers.size === 0) {
            return;
        }

        const tasks = [];
        for (const descriptor of drivers.values()) {
            const key = this._motorDescriptorKey(descriptor);
            if (!key || this._driverDirections.has(key) || this._directionFetches.has(key)) {
                continue;
            }
            tasks.push(this._fetchDriverDirection(descriptor));
        }

        if (tasks.length > 0) {
            await Promise.allSettled(tasks);
        }
    }

    _getTorqueMotorDescriptor(event) {
        if (!event || event.type !== 'TorqueMode') {
            return null;
        }
        const canAddressCandidate = event.can_address ?? event.canAddress;
        if (Number.isFinite(canAddressCandidate)) {
            const driverIndex = Number.isFinite(event.driver) ? event.driver : 0;
            return {
                canAddress: canAddressCandidate,
                driver: driverIndex,
            };
        }
        const fallback = this._normalizeMotorDescriptorValue(event.driver ?? event.motorId);
        return fallback;
    }

    _normalizeMotorDescriptorValue(raw) {
        if (raw == null) {
            return null;
        }
        if (typeof raw === 'object') {
            const canAddressCandidate = raw.can_address ?? raw.canAddress ?? raw.motorId;
            const driverCandidate = raw.driver;
            if (Number.isFinite(canAddressCandidate)) {
                const driverIndex = Number.isFinite(driverCandidate) ? driverCandidate : 0;
                return {
                    canAddress: canAddressCandidate,
                    driver: driverIndex,
                };
            }
            if (Number.isFinite(driverCandidate)) {
                return this._normalizeMotorDescriptorValue(driverCandidate);
            }
            if (Number.isFinite(raw.motorId)) {
                return this._normalizeMotorDescriptorValue(raw.motorId);
            }
            return null;
        }

        const text = typeof raw === 'number'
            ? raw.toString()
            : typeof raw === 'string'
                ? raw.trim()
                : '';
        if (!text) {
            return null;
        }
        const [canPart, driverPart] = text.split('.');
        const canAddress = parseInt(canPart, 10);
        if (!Number.isFinite(canAddress)) {
            return null;
        }
        let driver = 0;
        if (driverPart !== undefined) {
            const parsedDriver = parseInt(driverPart, 10);
            if (Number.isFinite(parsedDriver)) {
                driver = parsedDriver;
            }
        }
        return {
            canAddress,
            driver,
        };
    }

    _motorDescriptorKey(descriptor) {
        if (!descriptor || !Number.isFinite(descriptor.canAddress)) {
            return null;
        }
        const driverIndex = Number.isFinite(descriptor.driver) ? descriptor.driver : 0;
        return `${descriptor.canAddress}.${driverIndex}`;
    }

    _getMotorDescriptorFromInput(input) {
        if (!input) {
            return null;
        }
        if (typeof input === 'object') {
            if (input.type === 'TorqueMode') {
                return this._getTorqueMotorDescriptor(input);
            }
            const canAddressCandidate = input.can_address ?? input.canAddress;
            if (Number.isFinite(canAddressCandidate)) {
                const driverIndex = Number.isFinite(input.driver) ? input.driver : 0;
                return {
                    canAddress: canAddressCandidate,
                    driver: driverIndex,
                };
            }
            if (Number.isFinite(input.motorId)) {
                return this._normalizeMotorDescriptorValue(input.motorId);
            }
            if (typeof input.driver !== 'undefined') {
                return this._normalizeMotorDescriptorValue(input.driver);
            }
            return null;
        }
        return this._normalizeMotorDescriptorValue(input);
    }

    _parseEncoderQuery(gcode) {
        if (typeof gcode !== 'string' || !/^M569\.3\b/i.test(gcode.trim())) {
            return { descriptors: [], setReference: false };
        }
        const pMatch = gcode.match(/P([0-9:\.]+)/i);
        const descriptors = pMatch
            ? pMatch[1]
                .split(':')
                .map((p) => this._normalizeMotorDescriptorValue(p))
                .filter((descriptor) => descriptor && Number.isFinite(descriptor.canAddress))
            : [];
        const setReference = /\bS(?:\s|$|-?[0-9])/i.test(gcode);
        return { descriptors, setReference };
    }

    _isEncoderQuery(gcode) {
        return typeof gcode === 'string' && /^M569\.3\b/i.test(gcode.trim());
    }

    async _handleEncoderQuery(gcode, options = {}) {
        const { timeout = 30000, suppressCallbacks = false } = options;
        const { descriptors, setReference } = this._parseEncoderQuery(gcode);
        if (!descriptors || descriptors.length === 0) {
            return { reply: "Error: M569: missing parameter 'P'", motion: [], rawMotionLines: [] };
        }

        const descriptorEntries = descriptors.map((descriptor) => ({
            descriptor,
            key: this._motorDescriptorKey(descriptor),
            axis: descriptor ? this.driverToAxis[descriptor.canAddress] : null,
        }));
        const axesForQuery = descriptorEntries
            .filter((entry) => entry.axis)
            .map((entry) => entry.axis);

        if (axesForQuery.length === 0) {
            return { reply: 'Error: M569.3: Message not received', motion: [], rawMotionLines: [] };
        }

        let anglesDeg = null;
        try {
            const resolverResult = await this.encoderResolver({
                axes: axesForQuery,
                timeoutMs: timeout,
                gcode,
            });
            if (Array.isArray(resolverResult)) {
                anglesDeg = resolverResult;
            } else if (resolverResult && Array.isArray(resolverResult.anglesDeg)) {
                anglesDeg = resolverResult.anglesDeg;
            }
        } catch (err) {
            const message = err?.message || 'encoder read failed';
            return { reply: `Error: M569.3: ${message}`, motion: [], rawMotionLines: [] };
        }

        if (!anglesDeg || anglesDeg.length === 0) {
            return { reply: 'Error: M569.3: Message not received', motion: [], rawMotionLines: [] };
        }

        const values = [];
        let encoderIdx = 0;
        for (const entry of descriptorEntries) {
            if (!entry.axis) {
                values.push(null);
                continue;
            }
            const rawValue = anglesDeg[encoderIdx++];
            if (setReference && entry.key && Number.isFinite(rawValue)) {
                this._encoderReferences.set(entry.key, rawValue);
            }
            const reference = entry.key && this._encoderReferences.has(entry.key)
                ? this._encoderReferences.get(entry.key)
                : 0;
            const relative = Number.isFinite(rawValue) ? rawValue - reference : null;
            values.push(Number.isFinite(relative) ? relative : null);
        }

        const reply = this._formatEncoderReply(values);
        if (!suppressCallbacks && this.onGCodeReply) {
            try {
                this.onGCodeReply(gcode, reply);
            } catch (_err) {
                // Ignore listener errors
            }
        }
        return { reply, motion: [], rawMotionLines: [] };
    }

    _formatEncoderReply(values) {
        if (!Array.isArray(values) || values.length === 0) {
            return '[ ]';
        }
        const formatValue = (value) => {
            if (!Number.isFinite(value)) {
                return 'nan';
            }
            return value.toFixed(2);
        };
        const content = values.map((v) => `${formatValue(v)}`).join(', ');
        return `[${content}, ]`;
    }

    async _fetchDriverDirection(identifier) {
        const descriptor = this._getMotorDescriptorFromInput(identifier);
        if (!descriptor) {
            return null;
        }
        const key = this._motorDescriptorKey(descriptor);
        if (!key) {
            return null;
        }
        if (this._driverDirections.has(key)) {
            return this._driverDirections.get(key);
        }
        if (this._directionFetches.has(key)) {
            return this._directionFetches.get(key);
        }

        const inflight = (async () => {
            try {
                const result = await this.sendGCode(`M569 P${key}`, {
                    suppressMotionProcessing: true,
                    suppressCallbacks: true,
                });
                const rawReply = result?.reply ?? '';
                const direction = this._parseDriverDirectionFromReply(rawReply);
                if (direction !== null) {
                    this._driverDirections.set(key, direction);
                    return direction;
                }
                if (typeof rawReply === 'string' && rawReply.trim().length === 0) {
                    this._driverDirections.set(key, true); // assume forwards when firmware stays silent
                    return true;
                }
            } catch (err) {
                console.warn(`RrfHttpBridge: failed to fetch direction for driver ${key}: ${err?.message || err}`);
            } finally {
                this._directionFetches.delete(key);
            }
            return null;
        })();

        this._directionFetches.set(key, inflight);
        return inflight;
    }

    _parseDriverDirectionFromReply(reply) {
        if (typeof reply !== 'string' || reply.length === 0) {
            return null;
        }
        const match = reply.match(/runs\s+(forwards?|forward|in\s+reverse|reverse)/i);
        if (!match || !match[1]) {
            return null;
        }
        const descriptor = match[1].toLowerCase();
        if (descriptor.startsWith('forw')) {
            return true;
        }
        if (descriptor.includes('reverse')) {
            return false;
        }
        return null;
    }

    _maybeCacheDirectionFromReply(gcode, reply) {
        if (typeof gcode !== 'string' || !gcode.trim().toUpperCase().startsWith('M569')) {
            return;
        }
        const driverMatch = gcode.match(/P([0-9]+(?:\.[0-9]+)?)/i);
        const descriptor = driverMatch ? this._normalizeMotorDescriptorValue(driverMatch[1]) : null;
        const key = descriptor ? this._motorDescriptorKey(descriptor) : null;
        if (!key) {
            return;
        }
        const direction = this._parseDriverDirectionFromReply(reply);
        if (direction !== null) {
            this._driverDirections.set(key, direction);
        } else if (typeof reply === 'string' && reply.trim().length === 0) {
            this._driverDirections.set(key, true); // silent reply -> assume forwards
        }
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

    _markAxisActive(axis) {
        const state = this._ensureAxisState(axis);
        if (!state) {
            return;
        }
        state.active = true;
        this.activeAxes.add(axis);
        if (!this._axisAngles.has(axis)) {
            this._axisAngles.set(axis, 0);
        }
        if (axis !== 'E' && !this.spoolAxisOrder.includes(axis)) {
            this.spoolAxisOrder.push(axis);
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

    _flushReadyBuckets(force = false) {
        const threshold = this._readyBucketThreshold(force);
        if (threshold == null) {
            return;
        }
        const upperBound = Math.min(threshold, this.maxBucketSeen + 1);
        while (this.nextBucketToEmit < upperBound) {
            this._emitBucket(this.nextBucketToEmit);
            this.nextBucketToEmit += 1;
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

    _emitBucket(bucketIdx) {
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
            if (hasDelta && this.remoteSpoolSystem) {
                this.remoteSpoolSystem.addCommand(addCmd);
            }
            this.bucketAddToReference.delete(bucketIdx);
        }

        let changed = false;
        const moveCmd = { type: 'Move' };

        for (const axis of this.spoolAxisOrder) {
            const state = this.axisStates.get(axis);
            if (!this._axisAngles.has(axis)) {
                this._axisAngles.set(axis, state?.baseAngle || 0);
            }
            const axisMap = this.bucketSteps.get(axis);
            const deltaSteps = axisMap ? axisMap.get(bucketIdx) || 0 : 0;
            if (deltaSteps !== 0) {
                const current = this._axisAngles.get(axis) || 0;
                const newAngle = current + deltaSteps * STEP_ANGLE_RAD;
                this._axisAngles.set(axis, newAngle);
                changed = true;
            }
            moveCmd[axis] = this._axisAngles.get(axis) || 0;
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

        if (!changed && bucketIdx === 0 && !this.baselineEmitted) {
            changed = true;
        }

        if (changed && this.remoteSpoolSystem) {
            if (bucketIdx === 0) {
                this.baselineEmitted = true;
            }
            this.remoteSpoolSystem.addCommand(moveCmd);
        }
    }

    _parseResponse(responseText) {
        const parts = responseText.split('---MOTION---');
        const result = {
            reply: parts[0].trim(),
            motion: [],
            rawMotionLines: [],
        };

        if (parts.length > 1) {
            const motionLines = parts[1].trim().split('\n');
            for (const rawLine of motionLines) {
                const line = rawLine.trim();
                if (!line) {
                    continue;
                }
                result.rawMotionLines.push(line);
                if (line.startsWith('{')) {
                    continue;
                }

                if (line.startsWith('T,')) {
                    const [, driver, torque] = line.split(',');
                    const descriptor = this._normalizeMotorDescriptorValue(driver);
                    if (!descriptor) {
                        continue;
                    }
                    result.motion.push({
                        type: 'TorqueMode',
                        can_address: descriptor.canAddress,
                        driver: descriptor.driver,
                        motorId: descriptor.canAddress,
                        torqueNm: parseFloat(torque),
                    });
                    continue;
                }

                const fields = line.split(',');
                if (fields.length >= 7) {
                    result.motion.push({
                        type: 'Motion',
                        index: parseInt(fields[0], 10),
                        motorId: parseInt(fields[1], 10),
                        timestamp: parseInt(fields[2], 10),
                        accelTicks: parseInt(fields[3], 10),
                        steadyTicks: parseInt(fields[4], 10),
                        decelTicks: parseInt(fields[5], 10),
                        steps: parseFloat(fields[6]),
                        acceleration: fields[7] ? parseFloat(fields[7]) : 0,
                        deceleration: fields[8] ? parseFloat(fields[8]) : 0,
                    });
                }
            }
        }

        return result;
    }

    _parseTorqueModeCommand(gcode) {
        if (typeof gcode !== 'string' || !/^M569\.4\b/i.test(gcode.trim())) {
            return [];
        }
        const pMatch = gcode.match(/P([0-9:\.]+)/i);
        const tMatch = gcode.match(/T(-?[0-9]+(?:\.[0-9]+)?)/i);
        if (!pMatch || !tMatch) {
            return [];
        }
        const torqueNm = parseFloat(tMatch[1]);
        if (!Number.isFinite(torqueNm)) {
            return [];
        }

        const descriptors = pMatch[1]
            .split(':')
            .map((d) => this._normalizeMotorDescriptorValue(d))
            .filter((descriptor) => descriptor && Number.isFinite(descriptor.canAddress));

        return descriptors.map((descriptor) => ({
            type: 'TorqueMode',
            can_address: descriptor.canAddress,
            driver: descriptor.driver,
            motorId: descriptor.canAddress,
            torqueNm,
        }));
    }

    _injectTorqueModeFallback(parsedResponse, gcode) {
        if (!parsedResponse || !gcode || !Array.isArray(parsedResponse.motion)) {
            return;
        }
        const torqueCommands = this._parseTorqueModeCommand(gcode);
        if (torqueCommands.length === 0) {
            return;
        }
        const seenKeys = new Set(
            parsedResponse.motion
                .filter((m) => m?.type === 'TorqueMode')
                .map((m) => {
                    const descriptor = this._getTorqueMotorDescriptor(m);
                    return this._motorDescriptorKey(descriptor);
                })
                .filter((key) => typeof key === 'string'),
        );
        const missing = torqueCommands.filter((cmd) => {
            const descriptor = this._getTorqueMotorDescriptor(cmd);
            const key = this._motorDescriptorKey(descriptor);
            return key && !seenKeys.has(key);
        });
        if (missing.length > 0) {
            parsedResponse.motion.push(...missing);
        }
    }

    _processMotion(motionItems) {
        let lastTimestamp = null;
        for (const item of motionItems) {
            if (item.type === 'TorqueMode') {
                this._flushReadyBuckets();
                this._handleTorqueModeChange(item);
            } else if (item.type === 'Motion') {
                if (lastTimestamp !== null && item.timestamp !== lastTimestamp) {
                    this._flushReadyBuckets();
                }
                this._handleMotionCommand(item);
                lastTimestamp = item.timestamp;
            }
        }
        this._flushReadyBuckets(true);
    }

    _handleTorqueModeChange(event) {
        const descriptor = this._getTorqueMotorDescriptor(event);
        if (!descriptor) {
            console.warn('RrfHttpBridge: TorqueMode event missing motorId');
            return;
        }
        const key = this._motorDescriptorKey(descriptor);
        const torqueNm = event?.torqueNm ?? 0;
        const axis = this.driverToAxis[descriptor.canAddress];

        if (!axis) {
            console.warn(`RrfHttpBridge: Unknown motorId ${descriptor.canAddress}`);
            return;
        }

        const direction = key ? this._driverDirections.get(key) : null;
        const torqueSign = direction === true ? -1 : direction === false ? 1 : null;
        const effectiveTorque = torqueSign === null ? -torqueNm : torqueNm * torqueSign;

        if (this.onTorqueModeChange) {
            try {
                this.onTorqueModeChange(descriptor.canAddress, axis, effectiveTorque);
            } catch (_err) {
                /* ignore listener errors */
            }
        }

        if (this.remoteSpoolSystem) {
            const isPositionMode = Math.abs(effectiveTorque) < 0.0001;
            this.remoteSpoolSystem.addCommand({
                type: isPositionMode ? 'SetPositionMode' : 'SetTorqueMode',
                axis,
                driver: descriptor.canAddress,
                torqueNm: effectiveTorque,
                timestamp: Date.now(),
            });
        }
    }

    _handleMotionCommand(motion) {
        if (!this.remoteSpoolSystem) {
            return;
        }

        const axis = this.driverToAxis[motion.motorId];
        if (!axis) {
            return;
        }

        const totalTicks = motion.accelTicks + motion.steadyTicks + motion.decelTicks;
        const state = this._ensureAxisState(axis);
        if (state) {
            state.lastTick = Math.max(state.lastTick, motion.timestamp + totalTicks);
        }

        if (!this.valueTypeByMotorId.has(motion.motorId)) {
            this.valueTypeByMotorId.set(motion.motorId, isFloatValue(motion.steps) ? 'float' : 'int');
        }
        const axisType = this.valueTypeByMotorId.get(motion.motorId);
        const startTick = motion.timestamp;
        const profile = createMotionProfile(motion);
        const useProfile = profile && profile.totalTicks > 0;
        const distribute = (bucketIdx, normalizedDelta) => {
            const deltaValue = motion.steps * normalizedDelta;
            if (axis === 'E' || axisType === 'float') {
                this._recordBucketExtrusion(bucketIdx, deltaValue * EXTRUDER_MM_PER_STEP);
            } else {
                this._recordBucketSteps(axis, bucketIdx, deltaValue);
            }
        };

        const updateMax = (bucketIdx) => {
            this.maxBucketSeen = Math.max(this.maxBucketSeen, bucketIdx);
        };

        if (useProfile) {
            distributeWithProfile({
                startTick,
                profile,
                bucketSize: this.ticksPerBucket,
                accumulateNormalized: distribute,
                setMaxBucket: updateMax,
            });
        } else if (axis === 'E' || axisType === 'float') {
            distributeEvenly({
                startTick,
                durationTicks: totalTicks,
                totalValue: motion.steps * EXTRUDER_MM_PER_STEP,
                bucketSize: this.ticksPerBucket,
                accumulate: (bucketIdx, delta) => {
                    this._recordBucketExtrusion(bucketIdx, delta);
                },
                setMaxBucket: updateMax,
            });
        } else {
            distributeEvenly({
                startTick,
                durationTicks: totalTicks,
                totalValue: motion.steps,
                bucketSize: this.ticksPerBucket,
                accumulate: (bucketIdx, delta) => {
                    this._recordBucketSteps(axis, bucketIdx, delta);
                },
                setMaxBucket: updateMax,
            });
        }

        if (state && motion.steps !== 0) {
            state.hasSteps = true;
        }
        this._markAxisActive(axis);
    }

    async home() {
        return this.sendGCode('G28');
    }

    async move(x, y, z, feedrate = 1000) {
        const parts = [];
        if (x !== undefined) parts.push(`X${x}`);
        if (y !== undefined) parts.push(`Y${y}`);
        if (z !== undefined) parts.push(`Z${z}`);
        return this.sendGCode(`G1 ${parts.join(' ')} F${feedrate}`);
    }

    async setTorqueMode(driver, torqueNm) {
        return this.sendGCode(`M569.4 P${driver} T${torqueNm}`);
    }

    async setPositionMode(driver) {
        return this.sendGCode(`M569.4 P${driver} T0`);
    }

    async getPosition() {
        const result = await this.sendGCode('M114');
        return result.reply;
    }

    async getFirmwareInfo() {
        return this.sendGCode('M115');
    }
}

export default RrfHttpBridge;
