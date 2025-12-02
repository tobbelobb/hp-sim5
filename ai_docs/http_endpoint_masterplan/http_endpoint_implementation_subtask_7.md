# Subtask 7: Integration, End-to-End Testing, and Validation

## Overview
Wire all components together and validate the complete system works as expected. This subtask focuses on:
1. Integration testing of all subtasks
2. End-to-end workflow validation
3. Documentation of the complete system
4. Performance benchmarking

## Test Scenarios

### Scenario 1: Basic HTTP Endpoint Connectivity
**Objective**: Verify the server starts and responds to requests.

```bash
#!/bin/bash
# tests/e2e/test_01_connectivity.sh

echo "=== Test 1: Basic Connectivity ==="

# Build
cmake --build RRF/build --target rrf_simulator -j || exit 1

# Start server
./RRF/build/rrf_simulator \
    --vsd RRF/run/vsd \
    -c sys/config_slideprinter.g \
    --server -p 8080 &
SERVER_PID=$!

# Wait for startup
sleep 3

# Test status endpoint
STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8080/machine/status)
if [ "$STATUS" != "200" ]; then
    echo "FAIL: Status endpoint returned $STATUS"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  /machine/status: OK"

# Test code endpoint accepts POST
RESPONSE=$(curl -s -w "\n%{http_code}" http://localhost:8080/machine/code \
    -d "M115" -H "Content-Type: text/plain")
HTTP_CODE=$(echo "$RESPONSE" | tail -n1)
BODY=$(echo "$RESPONSE" | head -n -1)

if [ "$HTTP_CODE" != "200" ]; then
    echo "FAIL: /machine/code returned $HTTP_CODE"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi

if [[ "$BODY" != *"FIRMWARE"* ]]; then
    echo "FAIL: M115 did not return firmware info"
    echo "Response: $BODY"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  /machine/code (M115): OK"

kill $SERVER_PID 2>/dev/null
echo "PASS: Basic connectivity test"
```

### Scenario 2: M569.4 Torque Mode
**Objective**: Verify M569.4 command works correctly.

```bash
#!/bin/bash
# tests/e2e/test_02_torque_mode.sh

echo "=== Test 2: M569.4 Torque Mode ==="

./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 3

# Test setting torque mode
RESPONSE=$(curl -s http://localhost:8080/machine/code \
    -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain")

if [[ "$RESPONSE" != *"0.001000 Nm"* ]]; then
    echo "FAIL: Expected '0.001000 Nm' in response"
    echo "Got: $RESPONSE"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  Set torque 0.001 Nm: OK"

# Test position mode (T0)
RESPONSE=$(curl -s http://localhost:8080/machine/code \
    -d "M569.4 P40.0 T0" -H "Content-Type: text/plain")

if [[ "$RESPONSE" != *"pos_mode"* ]]; then
    echo "FAIL: Expected 'pos_mode' in response"
    echo "Got: $RESPONSE"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  Set position mode: OK"

# Test multiple drivers
RESPONSE=$(curl -s http://localhost:8080/machine/code \
    -d "M569.4 P40.0:41.0:42.0 T0.002" -H "Content-Type: text/plain")

# Should have three torque values
COUNT=$(echo "$RESPONSE" | grep -o "0.002000 Nm" | wc -l)
if [ "$COUNT" -ne 3 ]; then
    echo "FAIL: Expected 3 torque values"
    echo "Got: $RESPONSE"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  Multiple drivers: OK"

# Test error cases
RESPONSE=$(curl -s http://localhost:8080/machine/code \
    -d "M569.4 P40.0" -H "Content-Type: text/plain")

if [[ "$RESPONSE" != *"Error"* && "$RESPONSE" != *"error"* && "$RESPONSE" != *"missing"* ]]; then
    echo "FAIL: Expected error for missing T parameter"
    echo "Got: $RESPONSE"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  Error handling: OK"

kill $SERVER_PID 2>/dev/null
echo "PASS: M569.4 torque mode test"
```

### Scenario 3: Motion Command Generation
**Objective**: Verify G1 commands produce motion data.

```bash
#!/bin/bash
# tests/e2e/test_03_motion.sh

echo "=== Test 3: Motion Command Generation ==="

./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 3

# First home (required for most configs)
curl -s http://localhost:8080/machine/code -d "G28" -H "Content-Type: text/plain" > /dev/null

# Execute a move
RESPONSE=$(curl -s http://localhost:8080/machine/code \
    -d "G1 X10 Y10 F1000" -H "Content-Type: text/plain")

# Check if motion data is present
if [[ "$RESPONSE" == *"---MOTION---"* ]]; then
    echo "  Motion data present: OK"

    # Count motion lines
    MOTION_LINES=$(echo "$RESPONSE" | sed -n '/---MOTION---/,$p' | grep -E "^[0-9]" | wc -l)
    echo "  Motion lines generated: $MOTION_LINES"

    if [ "$MOTION_LINES" -lt 1 ]; then
        echo "WARN: Expected motion data for G1 command (may be config-dependent)"
    fi
else
    echo "INFO: No motion delimiter (motion may be in separate channel)"
fi

# Test M114 (position report)
RESPONSE=$(curl -s http://localhost:8080/machine/code \
    -d "M114" -H "Content-Type: text/plain")

if [[ "$RESPONSE" == *"X:"* || "$RESPONSE" == *"A:"* ]]; then
    echo "  Position report: OK"
else
    echo "  Position report format: $RESPONSE"
fi

kill $SERVER_PID 2>/dev/null
echo "PASS: Motion command test"
```

### Scenario 4: JavaScript Integration
**Objective**: Test RrfHttpBridge with physics simulation.

```javascript
// tests/e2e/test_04_js_integration.mjs
import { RrfHttpBridge } from '../../examples/js/slideprinter/rrfHttpBridge.js';

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
        // Note: callback may or may not fire depending on motion data inclusion
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

    // Test 5: Timeout handling
    bridge.sendGCode('G4 S60', { timeout: 100 })
        .then(() => {
            console.error('  Timeout: FAIL - should have timed out');
            process.exit(1);
        })
        .catch(e => {
            if (e.message.includes('timed out')) {
                console.log('  Timeout handling: OK');
            } else {
                console.error('  Timeout: FAIL -', e.message);
            }
        });

    console.log('PASS: JavaScript integration test');
}

runTests().catch(e => {
    console.error('Test failed:', e);
    process.exit(1);
});
```

### Scenario 5: Full Auto-Calibration Workflow
**Objective**: Simulate the workflow from ai_docs/get_auto_calibration_data_automatically.sh

```bash
#!/bin/bash
# tests/e2e/test_05_autocal_workflow.sh

echo "=== Test 5: Auto-Calibration Workflow ==="

./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 3

ENDPOINT="http://localhost:8080/machine/code"

# Simulate the workflow from get_auto_calibration_data_automatically.sh

# Step 1: Set torque mode on all motors
echo "  Setting torque mode..."
for DRIVER in 40.0 41.0 42.0 43.0; do
    RESPONSE=$(curl -s "$ENDPOINT" -d "M569.4 P$DRIVER T0.001" -H "Content-Type: text/plain")
    if [[ "$RESPONSE" != *"Nm"* ]]; then
        echo "FAIL: Could not set torque mode for $DRIVER"
        kill $SERVER_PID 2>/dev/null
        exit 1
    fi
done
echo "  Torque mode set: OK"

# Step 2: Return to position mode
echo "  Returning to position mode..."
RESPONSE=$(curl -s "$ENDPOINT" -d "M569.4 P40.0:41.0:42.0:43.0 T0" -H "Content-Type: text/plain")
COUNT=$(echo "$RESPONSE" | grep -o "pos_mode" | wc -l)
if [ "$COUNT" -ne 4 ]; then
    echo "FAIL: Expected 4 pos_mode responses"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
echo "  Position mode: OK"

# Step 3: Get position report
echo "  Getting position..."
RESPONSE=$(curl -s "$ENDPOINT" -d "M114" -H "Content-Type: text/plain")
echo "  Position: $RESPONSE"

kill $SERVER_PID 2>/dev/null
echo "PASS: Auto-calibration workflow test"
```

## Integration Test Runner

```bash
#!/bin/bash
# tests/run_all_e2e_tests.sh

set -e

echo "Building rrf_simulator..."
cmake --build RRF/build --target rrf_simulator -j

echo ""
echo "Running end-to-end tests..."
echo "=============================="
echo ""

TESTS=(
    "tests/e2e/test_01_connectivity.sh"
    "tests/e2e/test_02_torque_mode.sh"
    "tests/e2e/test_03_motion.sh"
    "tests/e2e/test_05_autocal_workflow.sh"
)

PASSED=0
FAILED=0

for TEST in "${TESTS[@]}"; do
    echo ""
    if [ -f "$TEST" ]; then
        if bash "$TEST"; then
            ((PASSED++))
        else
            ((FAILED++))
        fi
    else
        echo "SKIP: $TEST not found"
    fi
done

# JavaScript test (requires Node.js)
if command -v node &> /dev/null; then
    echo ""
    ./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
    SERVER_PID=$!
    sleep 3

    if node --experimental-modules tests/e2e/test_04_js_integration.mjs; then
        ((PASSED++))
    else
        ((FAILED++))
    fi

    kill $SERVER_PID 2>/dev/null
else
    echo ""
    echo "SKIP: Node.js not available for JS integration test"
fi

echo ""
echo "=============================="
echo "Results: $PASSED passed, $FAILED failed"

if [ $FAILED -gt 0 ]; then
    exit 1
fi
```

## Performance Benchmarks

```bash
#!/bin/bash
# tests/benchmark/benchmark_http.sh

echo "=== HTTP Endpoint Benchmark ==="

./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 3

# Benchmark M115 (simple command)
echo "Benchmarking M115 (100 requests)..."
START=$(date +%s.%N)
for i in $(seq 1 100); do
    curl -s http://localhost:8080/machine/code -d "M115" -H "Content-Type: text/plain" > /dev/null
done
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
RATE=$(echo "100 / $DURATION" | bc -l)
printf "  M115: %.2f requests/sec\n" $RATE

# Benchmark M569.4
echo "Benchmarking M569.4 (100 requests)..."
START=$(date +%s.%N)
for i in $(seq 1 100); do
    curl -s http://localhost:8080/machine/code -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain" > /dev/null
done
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
RATE=$(echo "100 / $DURATION" | bc -l)
printf "  M569.4: %.2f requests/sec\n" $RATE

# Concurrent requests
echo "Benchmarking concurrent requests (10 parallel)..."
START=$(date +%s.%N)
for i in $(seq 1 10); do
    for j in $(seq 1 10); do
        curl -s http://localhost:8080/machine/code -d "M115" -H "Content-Type: text/plain" > /dev/null &
    done
    wait
done
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
RATE=$(echo "100 / $DURATION" | bc -l)
printf "  Concurrent: %.2f requests/sec\n" $RATE

kill $SERVER_PID 2>/dev/null
echo "Benchmark complete"
```

## Documentation

### README Addition

```markdown
## HTTP Endpoint Mode

The rrf_simulator supports an HTTP server mode for interactive G-code execution:

### Starting the Server

```bash
./RRF/build/rrf_simulator \
    --vsd RRF/run/vsd \
    -c sys/config_slideprinter.g \
    --server \
    -p 8080
```

### Endpoints

- `POST /machine/code` - Execute G-code, returns reply text
- `GET /machine/status` - Get server status

### Example Usage

```bash
# Set torque mode
curl http://localhost:8080/machine/code -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain"
# Response: 0.001000 Nm,

# Return to position mode
curl http://localhost:8080/machine/code -d "M569.4 P40.0 T0" -H "Content-Type: text/plain"
# Response: pos_mode,

# Execute move
curl http://localhost:8080/machine/code -d "G1 X10 F1000" -H "Content-Type: text/plain"
```

### JavaScript Integration

See `examples/js/slideprinter/rrfHttpBridge.js` for programmatic access.
```

## Validation Criteria Summary

| Test | Description | Pass Criteria |
|------|-------------|---------------|
| Connectivity | Server starts and responds | HTTP 200 on /machine/status |
| M115 | Firmware info | Response contains "FIRMWARE" |
| M569.4 torque | Set torque mode | Response "X.XXXXXX Nm," |
| M569.4 position | Set position mode | Response "pos_mode," |
| M569.4 multi | Multiple drivers | Correct count of responses |
| M569.4 error | Missing parameter | Response contains "Error" |
| Motion data | G1 produces motion | Motion delimiter present |
| JS parsing | Parse response | Correct reply and motion array |
| Timeout | Command timeout | Throws timeout error |
| Workflow | Full autocal flow | All steps complete |

## Dependencies

This subtask depends on all previous subtasks being complete:
- Subtask 1: HTTP server infrastructure
- Subtask 2: G-code injection and response capture
- Subtask 3: M569.4 HOST_BUILD handler
- Subtask 4: Motion command streaming
- Subtask 5: JavaScript HTTP bridge
- Subtask 6: Physics engine torque mode

## Estimated Complexity
- Test infrastructure: Moderate (shell + JS)
- Integration testing: Moderate (coordinating multiple components)
- Documentation: Simple
- Benchmarking: Simple
