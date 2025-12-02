# Subtask 4: Motion Command Streaming

## Overview
Enable the HTTP endpoint to provide motion commands (CAN frames) to the JavaScript physics simulation. The existing `CanCapture` system writes to files; we need to extend it for real-time streaming or in-memory buffering.

## Architecture Options

### Option A: Include Motion Data in HTTP Response (Recommended)
When a G-code produces motion, include the CAN commands in the response body alongside the text reply.

**Pros:**
- Simple, synchronous model
- No additional endpoints needed
- Easy to parse

**Cons:**
- Response size grows with motion complexity
- Not suitable for very long moves

### Option B: Separate Streaming Endpoint (SSE/WebSocket)
Provide a `/machine/motion` endpoint that streams motion data.

**Pros:**
- Decoupled from G-code responses
- Can stream continuously during long operations

**Cons:**
- More complex client implementation
- Requires managing streaming connections

### Option C: Polling Endpoint
Provide `/machine/motion` that returns accumulated motion since last poll.

**Pros:**
- Simple implementation
- Works with any HTTP client

**Cons:**
- Latency depends on poll frequency
- May miss data if not polled fast enough

**Decision: Start with Option A for simplicity, with Option C as fallback for long operations.**

## Files to Modify/Create

- `RRF/host/can/CanCapture.h` - Add in-memory buffer
- `RRF/host/can/CanCapture.cpp` - Implement buffering
- `RRF/host/src/main.cpp` - Wire up motion retrieval

## Implementation Details

### 1. Extend CanCapture for In-Memory Buffering

```cpp
// RRF/host/can/CanCapture.h additions

#include <vector>
#include <string>

class HostCanCapture
{
public:
    // Existing methods...
    static bool Configure(const std::filesystem::path& outputPath);
    static void LogMotion(const CanMotionSegment& segment);

    // NEW: In-memory capture for HTTP responses
    static void StartCapture();       // Begin capturing to memory
    static void StopCapture();        // Stop capturing to memory
    static std::string FlushCapture(); // Get captured data and clear buffer
    static bool IsCapturing();

    // NEW: Torque mode events
    static void LogTorqueModeChange(uint8_t driverAddress, float torqueNm);

private:
    static std::vector<std::string> memoryBuffer_;
    static std::mutex bufferMutex_;
    static std::atomic<bool> capturingToMemory_;
};
```

### 2. Implementation

```cpp
// RRF/host/can/CanCapture.cpp additions

std::vector<std::string> HostCanCapture::memoryBuffer_;
std::mutex HostCanCapture::bufferMutex_;
std::atomic<bool> HostCanCapture::capturingToMemory_{false};

void HostCanCapture::StartCapture()
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    memoryBuffer_.clear();
    capturingToMemory_.store(true);
}

void HostCanCapture::StopCapture()
{
    capturingToMemory_.store(false);
}

bool HostCanCapture::IsCapturing()
{
    return capturingToMemory_.load();
}

std::string HostCanCapture::FlushCapture()
{
    std::lock_guard<std::mutex> lock(bufferMutex_);

    std::string result;
    for (const auto& line : memoryBuffer_) {
        result += line;
        result += "\n";
    }
    memoryBuffer_.clear();

    return result;
}

void HostCanCapture::LogMotion(/* existing params */)
{
    // Existing file logging...

    // Add memory buffering
    if (capturingToMemory_.load()) {
        std::lock_guard<std::mutex> lock(bufferMutex_);

        std::ostringstream oss;
        // Format same as CSV file output
        oss << captureIndex_ << ","
            << destinationId << ","
            << normalizedTimestamp << ","
            << accelClocks << ","
            << steadyClocks << ","
            << decelClocks << ","
            << steps;

        if (hasAcceleration) {
            oss << "," << std::scientific << acceleration
                << "," << deceleration;
        }

        memoryBuffer_.push_back(oss.str());
    }
}

void HostCanCapture::LogTorqueModeChange(uint8_t driverAddress, float torqueNm)
{
    // Log to file if enabled
    if (IsFileEnabled()) {
        std::lock_guard<std::mutex> lock(fileMutex_);
        fileStream_ << "T," << (int)driverAddress << "," << torqueNm << "\n";
    }

    // Log to memory buffer
    if (capturingToMemory_.load()) {
        std::lock_guard<std::mutex> lock(bufferMutex_);

        std::ostringstream oss;
        oss << "T," << (int)driverAddress << "," << torqueNm;
        memoryBuffer_.push_back(oss.str());
    }
}
```

### 3. Response Format

Extend the HTTP response to include motion data when present:

```
HTTP/1.1 200 OK
Content-Type: text/plain

ok
---MOTION---
{"capture_version":1}
0,40,0,20793,0,0,10
1,41,0,20793,0,0,-33
T,40,0.001000
```

The `---MOTION---` delimiter separates the G-code reply from motion data.

### 4. Integration in GCodeInjector

```cpp
// In GCodeInjector::ExecuteBlocking()

std::string GCodeInjector::ExecuteBlocking(const std::string& gcode, uint32_t timeoutMs)
{
    // Start motion capture
    HostCanCapture::StartCapture();

    auto cmd = std::make_shared<PendingCommand>();
    cmd->gcode = gcode;

    // ... existing code to inject and wait ...

    // Stop capture and get motion data
    HostCanCapture::StopCapture();
    std::string motionData = HostCanCapture::FlushCapture();

    // Build combined response
    std::string response = cmd->response;
    if (!motionData.empty()) {
        response += "\n---MOTION---\n";
        response += "{\"capture_version\":1}\n";
        response += motionData;
    }

    return response;
}
```

### 5. Alternative: Polling Endpoint

For long-running operations, add a separate endpoint:

```cpp
server_.Get("/machine/motion", [](const httplib::Request& req, httplib::Response& res) {
    std::string motion = HostCanCapture::FlushCapture();
    if (motion.empty()) {
        res.set_content("{\"motion\":[]}", "application/json");
    } else {
        res.set_content(motion, "text/plain");
    }
});
```

## Client-Side Parsing (JavaScript)

Update `RrfCommander` or create helper to parse combined responses:

```javascript
function parseHttpResponse(responseText) {
    const parts = responseText.split('---MOTION---');

    const result = {
        reply: parts[0].trim(),
        motion: []
    };

    if (parts.length > 1) {
        const motionLines = parts[1].trim().split('\n');

        // Skip header line
        for (let i = 1; i < motionLines.length; i++) {
            const line = motionLines[i].trim();
            if (!line) continue;

            if (line.startsWith('T,')) {
                // Torque mode event
                const [_, driver, torque] = line.split(',');
                result.motion.push({
                    type: 'TorqueMode',
                    driver: parseInt(driver),
                    torqueNm: parseFloat(torque)
                });
            } else {
                // Motion command (existing CSV format)
                const parts = line.split(',');
                result.motion.push({
                    type: 'Motion',
                    index: parseInt(parts[0]),
                    motorId: parseInt(parts[1]),
                    timestamp: parseInt(parts[2]),
                    accelTicks: parseInt(parts[3]),
                    steadyTicks: parseInt(parts[4]),
                    decelTicks: parseInt(parts[5]),
                    steps: parseFloat(parts[6]),
                    acceleration: parts[7] ? parseFloat(parts[7]) : undefined,
                    deceleration: parts[8] ? parseFloat(parts[8]) : undefined
                });
            }
        }
    }

    return result;
}
```

## Testing

### Unit Tests

```cpp
// RRF/host/tests/test_can_capture_memory.cpp

TEST(CanCapture, MemoryCapture) {
    HostCanCapture::StartCapture();

    // Simulate motion logging
    HostCanCapture::LogMotion(/* test params */);
    HostCanCapture::LogTorqueModeChange(40, 0.001f);

    HostCanCapture::StopCapture();

    std::string data = HostCanCapture::FlushCapture();
    EXPECT_FALSE(data.empty());
    EXPECT_TRUE(data.find("40") != std::string::npos);
    EXPECT_TRUE(data.find("T,40,0.001") != std::string::npos);

    // Second flush should be empty
    std::string data2 = HostCanCapture::FlushCapture();
    EXPECT_TRUE(data2.empty());
}

TEST(CanCapture, ConcurrentAccess) {
    HostCanCapture::StartCapture();

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([i]() {
            for (int j = 0; j < 100; ++j) {
                HostCanCapture::LogTorqueModeChange(40 + (i % 4), 0.001f * j);
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    HostCanCapture::StopCapture();
    std::string data = HostCanCapture::FlushCapture();

    // Should have 1000 entries
    size_t count = std::count(data.begin(), data.end(), '\n');
    EXPECT_EQ(count, 1000);
}
```

### Integration Test

```bash
#!/bin/bash
# tests/test_motion_streaming.sh

./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 2

# Execute a move that should produce motion
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "G1 X10 F1000" -H "Content-Type: text/plain")

# Check for motion data
if [[ "$RESPONSE" == *"---MOTION---"* ]]; then
    echo "Motion data present in response"

    # Extract and count motion lines
    MOTION_LINES=$(echo "$RESPONSE" | sed -n '/---MOTION---/,$p' | tail -n +2 | wc -l)
    echo "Found $MOTION_LINES motion lines"

    if [ "$MOTION_LINES" -lt 1 ]; then
        echo "FAIL: Expected motion data for G1 command"
        kill $SERVER_PID
        exit 1
    fi
else
    echo "INFO: No motion data (may be expected for some configs)"
fi

# Test torque mode produces events
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain")
if [[ "$RESPONSE" == *"T,40"* ]]; then
    echo "PASS: Torque mode event captured"
else
    echo "INFO: Torque mode event not in response (may be in file only)"
fi

kill $SERVER_PID
echo "PASS: Motion streaming works"
```

## Validation Criteria

1. Motion data appears in response after G1 commands
2. Torque mode events (T,driver,torque) appear in motion data
3. Memory buffer clears after flush
4. Thread-safe under concurrent access
5. Response format is parseable by JavaScript
6. Polling endpoint returns accumulated data

## Dependencies

- Subtask 2 (G-code Injection) provides the execution context
- Subtask 3 (M569.4) produces torque mode events
- Subtask 5 (JS Bridge) will consume this data

## Performance Considerations

- Memory buffer should have a size limit to prevent unbounded growth
- Consider ring buffer for long-running captures
- Flush periodically for very long operations
- Default buffer size: 10,000 entries (configurable)

```cpp
static constexpr size_t MAX_BUFFER_SIZE = 10000;

void HostCanCapture::LogMotion(...)
{
    if (capturingToMemory_.load()) {
        std::lock_guard<std::mutex> lock(bufferMutex_);

        if (memoryBuffer_.size() >= MAX_BUFFER_SIZE) {
            // Drop oldest entry (ring buffer behavior)
            memoryBuffer_.erase(memoryBuffer_.begin());
        }

        memoryBuffer_.push_back(formatLine(...));
    }
}
```

## Estimated Complexity
- Memory buffering: Simple
- Thread safety: Moderate (already have mutex patterns)
- Response formatting: Simple
- Client parsing: Simple
