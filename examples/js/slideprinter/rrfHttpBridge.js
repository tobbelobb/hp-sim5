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
    }

    async sendGCode(gcode, options = {}) {
        const timeout = options.timeout || 30000;
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

            if (parsed.motion.length > 0) {
                this._processMotion(parsed.motion);
            }

            if (this.onGCodeReply) {
                try {
                    this.onGCodeReply(gcode, parsed.reply);
                } catch (_err) {
                    // Ignore listener errors
                }
            }

            return parsed;
        } catch (error) {
            clearTimeout(timeoutId);
            if (error.name === 'AbortError') {
                throw new Error(`G-code command timed out after ${timeout}ms`);
            }
            throw error;
        }
    }

    _parseResponse(responseText) {
        const parts = responseText.split('---MOTION---');
        const result = {
            reply: parts[0].trim(),
            motion: [],
        };

        if (parts.length > 1) {
            const motionLines = parts[1].trim().split('\n');
            for (const rawLine of motionLines) {
                const line = rawLine.trim();
                if (!line || line.startsWith('{')) {
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

        if (this.onTorqueModeChange) {
            try {
                this.onTorqueModeChange(driver, axis, torqueNm);
            } catch (_err) {
                /* ignore listener errors */
            }
        }

        if (this.remoteSpoolSystem) {
            const isPositionMode = Math.abs(torqueNm) < 0.0001;
            this.remoteSpoolSystem.addCommand({
                type: isPositionMode ? 'SetPositionMode' : 'SetTorqueMode',
                axis,
                driver,
                torqueNm,
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
