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

        this._pendingMotion = [];
        this._axisAngles = new Map();
        this._driverDirections = new Map();
        this._directionFetches = new Map();
    }

    async sendGCode(gcode, options = {}) {
        const {
            timeout = 30000,
            suppressMotionProcessing = false,
            suppressCallbacks = false,
        } = options;
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

            if (!suppressMotionProcessing && parsed.motion.length > 0) {
                await this._primeDriverDirections(parsed.motion);
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
        const drivers = new Set();
        for (const item of motionItems) {
            if (item && item.type === 'TorqueMode' && Number.isFinite(item.driver)) {
                drivers.add(item.driver);
            } else if (item && item.type === 'Motion' && Number.isFinite(item.motorId)) {
                drivers.add(item.motorId);
            }
        }

        if (drivers.size === 0) {
            return;
        }

        const tasks = [];
        for (const driver of drivers) {
            if (this._driverDirections.has(driver)) {
                continue;
            }
            tasks.push(this._fetchDriverDirection(driver));
        }

        if (tasks.length > 0) {
            await Promise.allSettled(tasks);
        }
    }

    async _fetchDriverDirection(driver) {
        if (this._driverDirections.has(driver)) {
            return this._driverDirections.get(driver);
        }
        if (this._directionFetches.has(driver)) {
            return this._directionFetches.get(driver);
        }

        const inflight = (async () => {
            try {
                const result = await this.sendGCode(`M569 P${driver}`, {
                    suppressMotionProcessing: true,
                    suppressCallbacks: true,
                });
                const rawReply = result?.reply ?? '';
                const direction = this._parseDriverDirectionFromReply(rawReply);
                if (direction !== null) {
                    this._driverDirections.set(driver, direction);
                    return direction;
                }
                if (typeof rawReply === 'string' && rawReply.trim().length === 0) {
                    this._driverDirections.set(driver, true); // assume forwards when firmware stays silent
                    return true;
                }
            } catch (err) {
                console.warn(`RrfHttpBridge: failed to fetch direction for driver ${driver}: ${err?.message || err}`);
            } finally {
                this._directionFetches.delete(driver);
            }
            return null;
        })();

        this._directionFetches.set(driver, inflight);
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
        const driver = driverMatch ? parseFloat(driverMatch[1]) : null;
        if (!Number.isFinite(driver)) {
            return;
        }
        const direction = this._parseDriverDirectionFromReply(reply);
        if (direction !== null) {
            this._driverDirections.set(driver, direction);
        } else if (typeof reply === 'string' && reply.trim().length === 0) {
            this._driverDirections.set(driver, true); // silent reply -> assume forwards
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
                    result.motion.push({
                        type: 'TorqueMode',
                        driver: parseInt(driver, 10),
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

    _processMotion(motionItems) {
        for (const item of motionItems) {
            if (item.type === 'TorqueMode') {
                this._handleTorqueModeChange(item);
            } else if (item.type === 'Motion') {
                this._handleMotionCommand(item);
            }
        }
    }

    _handleTorqueModeChange(event) {
        const { driver, torqueNm } = event;
        const axis = this.driverToAxis[driver];

        if (!axis) {
            console.warn(`RrfHttpBridge: Unknown driver address ${driver}`);
            return;
        }

        const direction = this._driverDirections.get(driver);
        const torqueSign = direction === true ? -1 : direction === false ? 1 : null;
        const effectiveTorque = torqueSign === null ? torqueNm : torqueNm * torqueSign;

        if (this.onTorqueModeChange) {
            try {
                this.onTorqueModeChange(driver, axis, effectiveTorque);
            } catch (_err) {
                /* ignore listener errors */
            }
        }

        if (this.remoteSpoolSystem) {
            const isPositionMode = Math.abs(effectiveTorque) < 0.0001;
            this.remoteSpoolSystem.addCommand({
                type: isPositionMode ? 'SetPositionMode' : 'SetTorqueMode',
                axis,
                driver,
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

        const deltaAngle = motion.steps * STEP_ANGLE_RAD;
        const prevAngle = this._axisAngles.get(axis) || 0;
        const nextAngle = prevAngle + deltaAngle;
        this._axisAngles.set(axis, nextAngle);

        const command = {
            type: 'Move',
            timestamp: motion.timestamp,
            axes: {
                [axis]: nextAngle,
            },
            _motion: motion,
        };

        this.remoteSpoolSystem.addCommand(command);
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
