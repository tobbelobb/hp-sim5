import { RrfHttpBridge } from '../../integrations/rrf/rrfHttpBridge.js';

const BASE_URL = process.env.RRF_SERVER_URL || 'http://localhost:8080';

async function runTests() {
    console.log('=== Test 4: JavaScript Integration ===');

    const bridge = new RrfHttpBridge({ baseUrl: BASE_URL });

    // Test 1: Connection
    try {
        const info = await bridge.getFirmwareInfo();
        if (!info.reply.includes('FIRMWARE')) {
            throw new Error('M115 missing FIRMWARE');
        }
        console.log('  Connection: OK');
    } catch (e) {
        console.error('  Connection: FAIL -', e.message);
        process.exit(1);
    }

    // Test 2: Torque mode with callback
    let torqueCallback = null;
    bridge.onTorqueModeChange = (driver, axis, torque) => {
        torqueCallback = { driver, axis, torque };
    };

    try {
        const result = await bridge.setTorqueMode(40, 0.001);
        if (!result.reply.includes('Nm')) {
            throw new Error('Unexpected reply: ' + result.reply);
        }
        // Callback may not fire if no motion data is present; still ensure structure is valid when it does.
        if (torqueCallback && torqueCallback.driver !== 40) {
            throw new Error('Torque callback data incorrect');
        }
        console.log('  Torque mode: OK');
    } catch (e) {
        console.error('  Torque mode: FAIL -', e.message);
        process.exit(1);
    }

    // Test 3: Position mode
    try {
        const result = await bridge.setPositionMode(40);
        if (!result.reply.includes('pos_mode')) {
            throw new Error('Unexpected reply: ' + result.reply);
        }
        console.log('  Position mode: OK');
    } catch (e) {
        console.error('  Position mode: FAIL -', e.message);
        process.exit(1);
    }

    // Test 4: Response parsing
    const testResponse = `ok
---MOTION---
{"capture_version":1}
0,40,0,20793,0,0,10
T,41,0.002000`;

    const parsed = bridge._parseResponse(testResponse);
    if (parsed.reply !== 'ok') {
        console.error('  Parsing reply: FAIL');
        process.exit(1);
    }
    if (parsed.motion.length !== 2) {
        console.error('  Parsing motion count: FAIL');
        process.exit(1);
    }
    if (parsed.motion[0].type !== 'Motion' || parsed.motion[1].type !== 'TorqueMode') {
        console.error('  Parsing motion types: FAIL');
        process.exit(1);
    }
    console.log('  Response parsing: OK');

    console.log('PASS: JavaScript integration test');
}

runTests().catch(e => {
    console.error('Test failed:', e);
    process.exit(1);
});
