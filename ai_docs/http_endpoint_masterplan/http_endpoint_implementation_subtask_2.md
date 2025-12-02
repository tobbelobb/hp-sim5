# Subtask 2: G-code Command Injection and Response Capture

## Overview
Create infrastructure to inject G-code commands into the running firmware, wait for completion, and capture the response text.

## Files to Modify/Create
- `RRF/host/src/GCodeInjector.h` - New file
- `RRF/host/src/GCodeInjector.cpp` - New file
- `RRF/host/src/main.cpp` - Wire up injector

## Architecture Decision

The existing firmware uses `GCodeBuffer` objects to process G-code. The HTTP buffer is the natural choice for interactive commands. Key insight from `main.cpp`:

```cpp
GCodeBuffer* const fileBuffer = reprap.GetGCodes().FileGCode();
```

Similarly, we can use the HTTP buffer:
```cpp
GCodeBuffer* const httpBuffer = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::HTTP);
```

## Implementation Details

### 1. GCodeInjector Class

```cpp
// RRF/host/src/GCodeInjector.h
#pragma once

#include <string>
#include <mutex>
#include <condition_variable>
#include <queue>

class GCodeInjector
{
public:
    static GCodeInjector& Instance();

    // Submit G-code and block until response is ready
    // Returns the G-code response string
    std::string ExecuteBlocking(const std::string& gcode, uint32_t timeoutMs = 30000);

    // Called from the Spin loop to process pending commands
    void ProcessPending();

    // Called when a response is generated (hook into GCodes reply system)
    void OnResponse(const char* response);

private:
    GCodeInjector() = default;

    struct PendingCommand {
        std::string gcode;
        std::string response;
        bool completed{false};
        std::condition_variable cv;
    };

    std::mutex mutex_;
    std::queue<std::shared_ptr<PendingCommand>> pendingCommands_;
    std::shared_ptr<PendingCommand> activeCommand_;
};
```

### 2. Implementation

```cpp
// RRF/host/src/GCodeInjector.cpp
#include "GCodeInjector.h"
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>

GCodeInjector& GCodeInjector::Instance()
{
    static GCodeInjector instance;
    return instance;
}

std::string GCodeInjector::ExecuteBlocking(const std::string& gcode, uint32_t timeoutMs)
{
    auto cmd = std::make_shared<PendingCommand>();
    cmd->gcode = gcode;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pendingCommands_.push(cmd);
    }

    // Wait for completion
    std::unique_lock<std::mutex> lock(mutex_);
    bool completed = cmd->cv.wait_for(lock,
        std::chrono::milliseconds(timeoutMs),
        [&cmd]() { return cmd->completed; });

    if (!completed) {
        return "Error: Command timed out";
    }

    return cmd->response;
}

void GCodeInjector::ProcessPending()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // If we have an active command that's done, clear it
    if (activeCommand_ && activeCommand_->completed) {
        activeCommand_.reset();
    }

    // Start next command if idle
    if (!activeCommand_ && !pendingCommands_.empty()) {
        activeCommand_ = pendingCommands_.front();
        pendingCommands_.pop();

        // Inject into HTTP buffer
        GCodeBuffer* httpBuffer = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::HTTP);
        if (httpBuffer != nullptr && httpBuffer->IsCompletelyIdle()) {
            httpBuffer->Put(activeCommand_->gcode.c_str());
        }
    }
}

void GCodeInjector::OnResponse(const char* response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (activeCommand_) {
        activeCommand_->response = response;
        activeCommand_->completed = true;
        activeCommand_->cv.notify_all();
    }
}
```

### 3. Hook Response Capture

The firmware generates responses through `GCodes::HandleReply()`. We need to hook this for the HTTP channel. In `RRF/ReprapFirmware/src/GCodes/GCodes.cpp`, find `HandleReply` and add:

```cpp
#if RRF_HOST_BUILD
#include "host/src/GCodeInjector.h"
#endif

void GCodes::HandleReply(GCodeBuffer& gb, GCodeResult rslt, const char* reply) noexcept
{
    // Existing code...

#if RRF_HOST_BUILD
    if (gb.GetChannel() == GCodeChannel::HTTP) {
        GCodeInjector::Instance().OnResponse(reply);
    }
#endif

    // Rest of existing code...
}
```

### 4. Integration with Spin Loop

Modify the spin loop in server mode:

```cpp
std::thread spinThread([&running]() {
    while (running.load()) {
        GCodeInjector::Instance().ProcessPending();  // Check for pending commands
        reprap.Spin();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
});
```

### 5. Wire Up HTTP Endpoint

In the HTTP handler from Subtask 1:

```cpp
server_.Post("/machine/code", [](const httplib::Request& req, httplib::Response& res) {
    std::string gcode = req.body;

    // Trim whitespace
    gcode.erase(0, gcode.find_first_not_of(" \t\n\r"));
    gcode.erase(gcode.find_last_not_of(" \t\n\r") + 1);

    if (gcode.empty()) {
        res.status = 400;
        res.set_content("Error: Empty G-code", "text/plain");
        return;
    }

    std::string response = GCodeInjector::Instance().ExecuteBlocking(gcode);
    res.set_content(response, "text/plain");
});
```

## Alternative: Direct Command Execution

For simpler commands that don't need the full G-code machinery, we could execute directly:

```cpp
std::string ExecuteSimpleCommand(const std::string& gcode)
{
    StringRef reply(replyBuffer, sizeof(replyBuffer));

    // Parse the G-code
    GCodeBuffer tempBuffer(GCodeChannel::HTTP, nullptr, nullptr);
    tempBuffer.Put(gcode.c_str());
    tempBuffer.DecodeCommand();

    // Execute
    GCodeResult result = reprap.GetGCodes().ProcessGCode(tempBuffer, reply);

    return std::string(reply.c_str());
}
```

However, this bypasses state machines and may not work for all commands.

## Testing

### Unit Tests

```cpp
// RRF/host/tests/test_gcode_injector.cpp

TEST(GCodeInjector, ExecutesM115) {
    // M115 returns firmware info
    std::string response = GCodeInjector::Instance().ExecuteBlocking("M115");
    EXPECT_TRUE(response.find("FIRMWARE_NAME") != std::string::npos);
}

TEST(GCodeInjector, HandlesInvalidGCode) {
    std::string response = GCodeInjector::Instance().ExecuteBlocking("XYZZY");
    EXPECT_TRUE(response.find("Error") != std::string::npos ||
                response.find("Unknown") != std::string::npos);
}

TEST(GCodeInjector, TimeoutWorks) {
    // Command that would hang
    auto start = std::chrono::steady_clock::now();
    std::string response = GCodeInjector::Instance().ExecuteBlocking("G4 S60", 100);  // 100ms timeout
    auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, std::chrono::milliseconds(200));
    EXPECT_TRUE(response.find("timeout") != std::string::npos);
}
```

### Integration Test

```bash
#!/bin/bash
# tests/test_gcode_injection.sh

# Start server
./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 2

# Test M115 (firmware info)
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "M115" -H "Content-Type: text/plain")
if [[ "$RESPONSE" != *"FIRMWARE_NAME"* ]]; then
    echo "FAIL: M115 did not return firmware info"
    kill $SERVER_PID
    exit 1
fi

# Test G28 (home - should work even without actual hardware)
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "G28" -H "Content-Type: text/plain")
echo "G28 response: $RESPONSE"

kill $SERVER_PID
echo "PASS: G-code injection works"
```

## Validation Criteria

1. `M115` returns firmware version info
2. `M114` returns current position
3. Invalid G-codes return error messages
4. Commands complete within timeout
5. Multiple sequential commands work correctly
6. Response format matches real firmware

## Dependencies

- Subtask 1 (HTTP Server) provides the HTTP layer
- This subtask enables Subtasks 3-4 to return meaningful responses

## Thread Safety Considerations

- The injector uses mutexes to protect shared state
- Only one command executes at a time (serialized)
- The Spin loop and HTTP handlers run on different threads
- Response callback must be signal-safe (no allocations in critical path)

## Estimated Complexity
- Response hooking: Moderate (need to understand GCodes reply flow)
- Threading: Moderate (condition variables, proper locking)
- Integration: Simple (straightforward wiring)
