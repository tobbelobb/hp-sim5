# Subtask 3: Simplified M569.4 Handler for HOST_BUILD

## Overview
Add a `#if RRF_HOST_BUILD` block that handles M569.4 (torque mode) without requiring actual ODrive hardware. The handler should:
1. Parse P (driver) and T (torque) parameters
2. Track torque mode state per driver
3. Return properly formatted response strings
4. Emit events that the physics simulation can consume

## Files to Modify
- `RRF/ReprapFirmware/src/CAN/CanInterface.cpp` - Add HOST_BUILD handler
- `RRF/host/include/HostTorqueMode.h` - New file for state tracking
- `RRF/host/src/HostTorqueMode.cpp` - Implementation

## Key Design Decisions

### Why Not Use DUAL_CAN Code
The existing `#if DUAL_CAN` code in `HangprinterKinematics.cpp`:
- Sends actual CAN messages to ODrive hardware
- Requires physical ODrive boards responding
- Has complex gear ratio and mechanical advantage calculations
- Is specific to Hangprinter's ODrive integration

For simulation, we need something simpler that:
- Tracks state without hardware
- Emits events the JS physics engine can consume
- Returns the same response format

### Response Format (from ai_docs/M569.4)
```
M569.4 P40.0 T0.001  → "0.001000 Nm,"
M569.4 P40.0 T0      → "pos_mode,"
M569.4 P40.0:41.0 T0 → "pos_mode, pos_mode,"
```

## Implementation Details

### 1. Host Torque Mode State Tracker

```cpp
// RRF/host/include/HostTorqueMode.h
#pragma once

#include <array>
#include <functional>
#include <cstdint>

// Callback when torque mode changes
// Parameters: driverAddress (40-43), torqueNm (0 = position mode)
using TorqueModeCallback = std::function<void(uint8_t driverAddress, float torqueNm)>;

class HostTorqueMode
{
public:
    static HostTorqueMode& Instance();

    // Set torque mode for a driver
    // Returns formatted response string (e.g., "0.001000 Nm," or "pos_mode,")
    const char* SetTorqueMode(uint8_t driverAddress, float torqueNm);

    // Get current torque for a driver (0 = position mode)
    float GetTorque(uint8_t driverAddress) const;

    // Check if driver is in torque mode
    bool IsInTorqueMode(uint8_t driverAddress) const;

    // Register callback for torque mode changes
    void SetCallback(TorqueModeCallback callback);

    // Get all driver states as JSON (for /machine/status)
    std::string GetStatusJson() const;

private:
    HostTorqueMode() = default;

    static constexpr uint8_t MIN_DRIVER = 40;
    static constexpr uint8_t MAX_DRIVER = 43;
    static constexpr size_t NUM_DRIVERS = MAX_DRIVER - MIN_DRIVER + 1;
    static constexpr float MIN_TORQUE_THRESHOLD = 0.0001f;  // Below this = position mode

    std::array<float, NUM_DRIVERS> torques_{};  // 0 = position mode
    TorqueModeCallback callback_;
    char responseBuffer_[32];
};
```

### 2. Implementation

```cpp
// RRF/host/src/HostTorqueMode.cpp
#include "HostTorqueMode.h"
#include <cstdio>
#include <cmath>

HostTorqueMode& HostTorqueMode::Instance()
{
    static HostTorqueMode instance;
    return instance;
}

const char* HostTorqueMode::SetTorqueMode(uint8_t driverAddress, float torqueNm)
{
    if (driverAddress < MIN_DRIVER || driverAddress > MAX_DRIVER) {
        snprintf(responseBuffer_, sizeof(responseBuffer_),
                 "Error: Invalid driver %d", driverAddress);
        return responseBuffer_;
    }

    size_t index = driverAddress - MIN_DRIVER;

    if (std::fabs(torqueNm) < MIN_TORQUE_THRESHOLD) {
        // Switch to position mode
        torques_[index] = 0.0f;
        snprintf(responseBuffer_, sizeof(responseBuffer_), "pos_mode, ");

        if (callback_) {
            callback_(driverAddress, 0.0f);
        }
    } else {
        // Set torque mode
        torques_[index] = torqueNm;
        snprintf(responseBuffer_, sizeof(responseBuffer_), "%.6f Nm, ", torqueNm);

        if (callback_) {
            callback_(driverAddress, torqueNm);
        }
    }

    return responseBuffer_;
}

float HostTorqueMode::GetTorque(uint8_t driverAddress) const
{
    if (driverAddress < MIN_DRIVER || driverAddress > MAX_DRIVER) {
        return 0.0f;
    }
    return torques_[driverAddress - MIN_DRIVER];
}

bool HostTorqueMode::IsInTorqueMode(uint8_t driverAddress) const
{
    return GetTorque(driverAddress) != 0.0f;
}

void HostTorqueMode::SetCallback(TorqueModeCallback callback)
{
    callback_ = callback;
}

std::string HostTorqueMode::GetStatusJson() const
{
    std::string json = "{\"torqueMode\":{";
    for (size_t i = 0; i < NUM_DRIVERS; ++i) {
        if (i > 0) json += ",";
        char buf[64];
        snprintf(buf, sizeof(buf), "\"driver%zu\":{\"address\":%d,\"torqueNm\":%.6f}",
                 i, static_cast<int>(MIN_DRIVER + i), torques_[i]);
        json += buf;
    }
    json += "}}";
    return json;
}
```

### 3. Integration in CanInterface.cpp

Modify `RRF/ReprapFirmware/src/CAN/CanInterface.cpp` at the M569.4 case:

```cpp
case 4:			// set driver torque mode
    {
        const ExpansionBoardData *const boardData = reprap.GetExpansion().GetBoardDetails(driver.boardAddress);
        if (boardData != nullptr && boardData->hasClosedLoop)
        {
            CanMessageGenericConstructor cons(M569Point4Params);
            cons.PopulateFromCommand(gb);
            return cons.SendAndGetResponse(CanMessageType::m569p4, driver.boardAddress, reply);
        }
    }

#if RRF_HOST_BUILD
    // Simplified torque mode for host simulation
    {
        if (!gb.Seen('T')) {
            reply.copy("Error: M569.4 missing parameter 'T'");
            return GCodeResult::error;
        }

        const float torque = gb.GetFValue();
        const char* response = HostTorqueMode::Instance().SetTorqueMode(
            driver.boardAddress, torque);
        reply.cat(response);
        return GCodeResult::ok;
    }
#elif DUAL_CAN
    // Original ODrive code...
    {
        Kinematics& kin = reprap.GetMove().GetKinematics();
        if (kin.GetLegacyType() == KinematicsType::hangprinter)
        {
            gb.MustSee('T');
            const float torque = gb.GetFValue();
            return ((HangprinterKinematics&)kin).SetODrive3TorqueMode(driver, torque, reply);
        }
    }
    return GCodeResult::errorNotSupported;
#else
    return GCodeResult::errorNotSupported;
#endif
```

### 4. Emit Events for Physics Simulation

The callback mechanism allows the simulator to react to torque mode changes. In `main.cpp`:

```cpp
#include "HostTorqueMode.h"

// In server mode initialization:
HostTorqueMode::Instance().SetCallback([](uint8_t driver, float torque) {
    // Log for debugging
    std::cout << "TorqueMode: driver=" << (int)driver
              << " torque=" << torque << " Nm" << std::endl;

    // Emit to CAN capture system for JS consumption
    HostCanCapture::LogTorqueModeChange(driver, torque);
});
```

### 5. Add TorqueMode Event to CAN Capture

Extend `RRF/host/can/CanCapture.cpp`:

```cpp
void HostCanCapture::LogTorqueModeChange(uint8_t driverAddress, float torqueNm)
{
    if (!IsEnabled()) return;

    std::lock_guard<std::mutex> lock(captureMutex_);

    // Write as special event type that RrfCanPlayer can parse
    // Format: T,driverAddress,torqueNm
    captureStream_ << "T," << (int)driverAddress << "," << torqueNm << "\n";
    captureStream_.flush();
}
```

## Testing

### Unit Tests

```cpp
// RRF/host/tests/test_host_torque_mode.cpp

TEST(HostTorqueMode, SetTorqueMode) {
    auto& tm = HostTorqueMode::Instance();

    // Set torque mode
    const char* response = tm.SetTorqueMode(40, 0.001f);
    EXPECT_STREQ(response, "0.001000 Nm, ");
    EXPECT_FLOAT_EQ(tm.GetTorque(40), 0.001f);
    EXPECT_TRUE(tm.IsInTorqueMode(40));
}

TEST(HostTorqueMode, SetPositionMode) {
    auto& tm = HostTorqueMode::Instance();

    // Set torque then back to position
    tm.SetTorqueMode(41, 0.1f);
    const char* response = tm.SetTorqueMode(41, 0.0f);
    EXPECT_STREQ(response, "pos_mode, ");
    EXPECT_FLOAT_EQ(tm.GetTorque(41), 0.0f);
    EXPECT_FALSE(tm.IsInTorqueMode(41));
}

TEST(HostTorqueMode, InvalidDriver) {
    auto& tm = HostTorqueMode::Instance();

    const char* response = tm.SetTorqueMode(99, 0.001f);
    EXPECT_TRUE(strstr(response, "Error") != nullptr);
}

TEST(HostTorqueMode, CallbackFired) {
    auto& tm = HostTorqueMode::Instance();

    bool called = false;
    uint8_t receivedDriver = 0;
    float receivedTorque = 0;

    tm.SetCallback([&](uint8_t d, float t) {
        called = true;
        receivedDriver = d;
        receivedTorque = t;
    });

    tm.SetTorqueMode(42, 0.05f);

    EXPECT_TRUE(called);
    EXPECT_EQ(receivedDriver, 42);
    EXPECT_FLOAT_EQ(receivedTorque, 0.05f);
}
```

### Integration Test via HTTP

```bash
#!/bin/bash
# tests/test_m569_4.sh

./RRF/build/rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &
SERVER_PID=$!
sleep 2

# Test torque mode
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain")
if [[ "$RESPONSE" != *"0.001000 Nm"* ]]; then
    echo "FAIL: Expected '0.001000 Nm', got: $RESPONSE"
    kill $SERVER_PID
    exit 1
fi

# Test position mode
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "M569.4 P40.0 T0" -H "Content-Type: text/plain")
if [[ "$RESPONSE" != *"pos_mode"* ]]; then
    echo "FAIL: Expected 'pos_mode', got: $RESPONSE"
    kill $SERVER_PID
    exit 1
fi

# Test multiple drivers
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "M569.4 P40.0:41.0 T0.002" -H "Content-Type: text/plain")
if [[ "$RESPONSE" != *"0.002000 Nm"*"0.002000 Nm"* ]]; then
    echo "FAIL: Expected two torque values, got: $RESPONSE"
    kill $SERVER_PID
    exit 1
fi

# Test missing T parameter
RESPONSE=$(curl -s http://localhost:8080/machine/code -d "M569.4 P40.0" -H "Content-Type: text/plain")
if [[ "$RESPONSE" != *"Error"* ]]; then
    echo "FAIL: Expected error for missing T, got: $RESPONSE"
    kill $SERVER_PID
    exit 1
fi

kill $SERVER_PID
echo "PASS: M569.4 works correctly"
```

## Validation Criteria

1. `M569.4 P40.0 T0.001` returns `"0.001000 Nm,"`
2. `M569.4 P40.0 T0` returns `"pos_mode,"`
3. `M569.4 P40.0:41.0 T0` returns `"pos_mode, pos_mode,"`
4. Missing P or T returns appropriate error
5. Invalid driver address returns error
6. Callback is invoked on state change
7. State persists between commands

## Dependencies

- Subtask 2 (G-code Injection) for command processing
- Subtask 6 (Physics Torque Mode) will consume the events

## Notes on Mechanical Calculations

The real firmware calculates motor torque from force using:
```cpp
lineTension_N = force_Newton / mechanicalAdvantage_[boardIndex];
spoolTorque_Nm = lineTension_N * spoolRadii_[boardIndex] * 0.001;
motorTorque_Nm = spoolTorque_Nm * motorGearTeeth_[boardIndex] / spoolGearTeeth_[boardIndex];
```

For the simplified HOST_BUILD version, we accept torque directly as the T parameter (already in Nm) and pass it through without transformation. The physics simulation can apply its own mechanical model if needed.

If the full mechanical calculation is desired later, it can be added by reading the M666 parameters (mechanical advantage, gear teeth, spool radii) during config processing.

## Estimated Complexity
- State tracking: Simple
- Response formatting: Simple
- Integration with CanInterface.cpp: Moderate (need to understand preprocessor flow)
- Callback mechanism: Simple
