# Subtask 5: JavaScript HTTP Bridge (RrfHttpBridge)

## Overview
Create a JavaScript class that bridges the HTTP endpoint to the existing `RemoteSpoolSystem`. It will:
1. POST G-code commands to the server
2. Parse responses (text replies + motion data)
3. Feed motion commands to `RemoteSpoolSystem`
4. Handle torque mode events

## Files to Create/Modify

- `bridges/rrf/http/rrfHttpBridge.js` - New file
- `examples/js/slideprinter/slideprinter_common.js` - Add torque mode handling
- `examples/js/slideprinter/rrfCommander.js` - Optional: share parsing logic

## Implementation Details

### 1. RrfHttpBridge Class

```javascript
// bridges/rrf/http/rrfHttpBridge.js

/**
 * Bridge between HTTP G-code endpoint and RemoteSpoolSystem.
 *
 * Usage:
 *   const bridge = new RrfHttpBridge({
 *     baseUrl: 'http://localhost:8080',
 *     remoteSpoolSystem: myRemoteSpoolSystem,
 *   });
 *
 *   const reply = await bridge.sendGCode('G1 X10 F1000');
 *   const torqueReply = await bridge.sendGCode('M569.4 P40.0 T0.001');
 */

const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16); // 1.8deg motor, 16x microstepping

export class RrfHttpBridge {
    constructor(options = {}) {
        this.baseUrl = options.baseUrl || 'http://localhost:8080';
        this.remoteSpoolSystem = options.remoteSpoolSystem || null;
        this.onTorqueModeChange = options.onTorqueModeChange || null;
        this.onGCodeReply = options.onGCodeReply || null;

        // Driver address to axis mapping (default for Hangprinter/Slideprinter)
        this.driverToAxis = options.driverToAxis || {
            40: 'A',
            41: 'B',
            42: 'C',
            43: 'D'
        };

        // Pending motion commands
        this._pendingMotion = [];
    }

    /**
     * Send a G-code command and wait for response.
     * Motion commands are automatically fed to RemoteSpoolSystem.
     *
     * @param {string} gcode - G-code to execute
     * @param {object} options - Options
     * @param {number} options.timeout - Timeout in ms (default 30000)
     * @returns {Promise<{reply: string, motion: Array}>}
     */
    async sendGCode(gcode, options = {}) {
        const timeout = options.timeout || 30000;

        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), timeout);

        try {
            const response = await fetch(`${this.baseUrl}/machine/code`, {
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

            // Process motion data
            if (parsed.motion.length > 0) {
                this._processMotion(parsed.motion);
            }

            // Notify reply listener
            if (this.onGCodeReply) {
                this.onGCodeReply(gcode, parsed.reply);
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

    /**
     * Parse HTTP response into reply and motion data.
     */
    _parseResponse(responseText) {
        const parts = responseText.split('---MOTION---');

        const result = {
            reply: parts[0].trim(),
            motion: []
        };

        if (parts.length > 1) {
            const motionLines = parts[1].trim().split('\n');

            for (let i = 0; i < motionLines.length; i++) {
                const line = motionLines[i].trim();
                if (!line || line.startsWith('{')) continue; // Skip empty and JSON header

                if (line.startsWith('T,')) {
                    // Torque mode event: T,driver,torqueNm
                    const [_, driver, torque] = line.split(',');
                    result.motion.push({
                        type: 'TorqueMode',
                        driver: parseInt(driver, 10),
                        torqueNm: parseFloat(torque)
                    });
                } else {
                    // Motion command: index,motorId,timestamp,accel,steady,decel,steps[,accel,decel]
                    const p = line.split(',');
                    if (p.length >= 7) {
                        result.motion.push({
                            type: 'Motion',
                            index: parseInt(p[0], 10),
                            motorId: parseInt(p[1], 10),
                            timestamp: parseInt(p[2], 10),
                            accelTicks: parseInt(p[3], 10),
                            steadyTicks: parseInt(p[4], 10),
                            decelTicks: parseInt(p[5], 10),
                            steps: parseFloat(p[6]),
                            acceleration: p[7] ? parseFloat(p[7]) : 0,
                            deceleration: p[8] ? parseFloat(p[8]) : 0
                        });
                    }
                }
            }
        }

        return result;
    }

    /**
     * Process motion data and feed to RemoteSpoolSystem.
     */
    _processMotion(motionItems) {
        for (const item of motionItems) {
            if (item.type === 'TorqueMode') {
                this._handleTorqueModeChange(item);
            } else if (item.type === 'Motion') {
                this._handleMotionCommand(item);
            }
        }
    }

    /**
     * Handle torque mode change event.
     */
    _handleTorqueModeChange(event) {
        const { driver, torqueNm } = event;
        const axis = this.driverToAxis[driver];

        if (!axis) {
            console.warn(`Unknown driver address: ${driver}`);
            return;
        }

        // Notify external listener
        if (this.onTorqueModeChange) {
            this.onTorqueModeChange(driver, axis, torqueNm);
        }

        // Send to RemoteSpoolSystem as special command
        if (this.remoteSpoolSystem) {
            const isPositionMode = Math.abs(torqueNm) < 0.0001;

            this.remoteSpoolSystem.addCommand({
                type: isPositionMode ? 'SetPositionMode' : 'SetTorqueMode',
                axis: axis,
                driver: driver,
                torqueNm: torqueNm,
                timestamp: Date.now()
            });
        }
    }

    /**
     * Handle motion command from server.
     */
    _handleMotionCommand(motion) {
        if (!this.remoteSpoolSystem) return;

        const axis = this.driverToAxis[motion.motorId];
        if (!axis) {
            // Could be extruder or unknown driver
            return;
        }

        // Convert steps to radians
        const deltaAngle = motion.steps * STEP_ANGLE_RAD;

        // Create command in format RemoteSpoolSystem expects
        const command = {
            type: 'Move',
            timestamp: motion.timestamp,
            axes: {
                [axis]: deltaAngle
            },
            // Store original motion data for advanced processing
            _motion: motion
        };

        this.remoteSpoolSystem.addCommand(command);
    }

    /**
     * Convenience methods for common operations.
     */

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
        // Parse position from reply (format varies)
        return result.reply;
    }

    async getFirmwareInfo() {
        return this.sendGCode('M115');
    }
}

export default RrfHttpBridge;
```

### 2. Extend RemoteSpoolSystem for Torque Mode Commands

Add handling for new command types in `slideprinter_common.js`:

```javascript
// In RemoteSpoolSystem._processCommand() around line 349

_processCommand(world, command, simTime) {
    const commandType = command?.type;

    // ... existing Move and Add to reference handling ...

    // NEW: Handle torque mode commands
    if (commandType === 'SetTorqueMode' || commandType === 'SetPositionMode') {
        const axis = command.axis;
        const entities = this.axisToEntity[axis];

        if (entities) {
            for (const entity of entities) {
                const stepper = world.getComponent(entity, StepperMotorComponent);
                if (stepper) {
                    if (commandType === 'SetTorqueMode') {
                        stepper.torqueMode = true;
                        stepper.targetTorque = command.torqueNm;
                    } else {
                        stepper.torqueMode = false;
                        stepper.targetTorque = 0;
                    }
                }
            }
        }

        // Call callback if set
        if (this.onCommandExecuted) {
            this.onCommandExecuted(command, simTime);
        }

        return true;
    }

    // ... rest of existing handling ...
}
```

### 3. Example Usage

```javascript
// examples/js/slideprinter/demo_http_bridge.js

import { RrfHttpBridge } from '../../../bridges/rrf/http/rrfHttpBridge.js';
import { RemoteSpoolSystem } from './slideprinter_common.js';

async function main() {
    // Create physics system
    const spoolSystem = new RemoteSpoolSystem();

    // Create HTTP bridge
    const bridge = new RrfHttpBridge({
        baseUrl: 'http://localhost:8080',
        remoteSpoolSystem: spoolSystem,
        onTorqueModeChange: (driver, axis, torque) => {
            console.log(`Driver ${driver} (${axis}): torque=${torque} Nm`);
        },
        onGCodeReply: (gcode, reply) => {
            console.log(`${gcode} -> ${reply}`);
        }
    });

    // Test connection
    const info = await bridge.getFirmwareInfo();
    console.log('Firmware:', info.reply);

    // Test torque mode
    const result = await bridge.setTorqueMode(40, 0.001);
    console.log('Torque mode result:', result.reply);

    // Back to position mode
    await bridge.setPositionMode(40);

    // Execute a move
    await bridge.sendGCode('G1 X10 Y10 F1000');

    console.log('Queue length:', spoolSystem.getQueueLength());
}

main().catch(console.error);
```

## Testing

### Unit Tests

```javascript
// examples/js/slideprinter/tests/rrfHttpBridge.test.js

import { RrfHttpBridge } from '../../../../bridges/rrf/http/rrfHttpBridge.js';

describe('RrfHttpBridge', () => {
    describe('_parseResponse', () => {
        it('parses simple reply', () => {
            const bridge = new RrfHttpBridge();
            const result = bridge._parseResponse('ok');

            expect(result.reply).toBe('ok');
            expect(result.motion).toHaveLength(0);
        });

        it('parses reply with motion data', () => {
            const bridge = new RrfHttpBridge();
            const response = `0.001000 Nm,
---MOTION---
{"capture_version":1}
0,40,0,20793,0,0,10
1,41,100,20793,1000,0,-33
T,40,0.001000`;

            const result = bridge._parseResponse(response);

            expect(result.reply).toBe('0.001000 Nm,');
            expect(result.motion).toHaveLength(3);

            // Check motion command
            expect(result.motion[0].type).toBe('Motion');
            expect(result.motion[0].motorId).toBe(40);
            expect(result.motion[0].steps).toBe(10);

            // Check torque event
            expect(result.motion[2].type).toBe('TorqueMode');
            expect(result.motion[2].driver).toBe(40);
            expect(result.motion[2].torqueNm).toBeCloseTo(0.001);
        });

        it('parses torque mode response', () => {
            const bridge = new RrfHttpBridge();
            const result = bridge._parseResponse('pos_mode, pos_mode,');

            expect(result.reply).toBe('pos_mode, pos_mode,');
        });
    });

    describe('_handleTorqueModeChange', () => {
        it('calls onTorqueModeChange callback', () => {
            let calledWith = null;
            const bridge = new RrfHttpBridge({
                onTorqueModeChange: (driver, axis, torque) => {
                    calledWith = { driver, axis, torque };
                }
            });

            bridge._handleTorqueModeChange({ driver: 40, torqueNm: 0.001 });

            expect(calledWith).toEqual({ driver: 40, axis: 'A', torque: 0.001 });
        });

        it('adds command to remoteSpoolSystem', () => {
            const commands = [];
            const mockSpoolSystem = {
                addCommand: (cmd) => commands.push(cmd)
            };

            const bridge = new RrfHttpBridge({
                remoteSpoolSystem: mockSpoolSystem
            });

            bridge._handleTorqueModeChange({ driver: 41, torqueNm: 0.05 });

            expect(commands).toHaveLength(1);
            expect(commands[0].type).toBe('SetTorqueMode');
            expect(commands[0].axis).toBe('B');
            expect(commands[0].torqueNm).toBe(0.05);
        });
    });
});
```

### Integration Test

```bash
#!/bin/bash
# tests/test_http_bridge.sh

# Start server
./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 2

# Run Node.js test script
node --experimental-modules examples/js/slideprinter/tests/integration_test.mjs

TEST_RESULT=$?

kill $SERVER_PID

exit $TEST_RESULT
```

```javascript
// examples/js/slideprinter/tests/integration_test.mjs

import { RrfHttpBridge } from '../../../../bridges/rrf/http/rrfHttpBridge.js';

const bridge = new RrfHttpBridge({ baseUrl: 'http://localhost:8080' });

async function test() {
    // Test M115
    const info = await bridge.getFirmwareInfo();
    if (!info.reply.includes('FIRMWARE')) {
        throw new Error('M115 failed');
    }
    console.log('M115: PASS');

    // Test M569.4
    const torque = await bridge.setTorqueMode(40, 0.001);
    if (!torque.reply.includes('0.001000 Nm')) {
        throw new Error(`M569.4 failed: ${torque.reply}`);
    }
    console.log('M569.4 torque: PASS');

    // Test position mode
    const pos = await bridge.setPositionMode(40);
    if (!pos.reply.includes('pos_mode')) {
        throw new Error(`M569.4 pos_mode failed: ${pos.reply}`);
    }
    console.log('M569.4 pos_mode: PASS');

    console.log('All tests passed!');
}

test().catch(err => {
    console.error('Test failed:', err);
    process.exit(1);
});
```

## Validation Criteria

1. `bridge.sendGCode('M115')` returns firmware info
2. `bridge.sendGCode('M569.4 P40.0 T0.001')` returns `"0.001000 Nm,"`
3. Motion data is correctly parsed from response
4. Torque mode events trigger callbacks
5. Commands are added to RemoteSpoolSystem queue
6. Timeout works correctly for slow operations
7. Error responses are handled properly

## Dependencies

- Subtasks 1-4 (Server infrastructure)
- Subtask 6 (Physics torque mode) for actual simulation effect

## Browser vs Node.js Considerations

The code uses `fetch()` which is available in:
- Modern browsers (native)
- Node.js 18+ (native)
- Node.js <18 with `node-fetch` package

For broader compatibility:

```javascript
// At top of bridges/rrf/http/rrfHttpBridge.js
let fetchImpl;
if (typeof fetch === 'undefined') {
    fetchImpl = require('node-fetch');
} else {
    fetchImpl = fetch;
}
// Use fetchImpl instead of fetch
```

## Estimated Complexity
- HTTP communication: Simple (just fetch)
- Response parsing: Simple
- Integration with RemoteSpoolSystem: Moderate
- Testing: Moderate (needs running server)
