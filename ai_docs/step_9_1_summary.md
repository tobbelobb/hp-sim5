# Step 9.1 Implementation Summary

## Goal
Create a lightweight host Move/DDA interface that allows CanMotion.cpp to compile and link without pulling in the full RRF Movement subsystem.

## What Was Done

### 1. Host Movement Facades Created

Created minimal facades in `RRF/host/include/Movement/`:

**Move.h**
- Provides basic Move interface that CanMotion and RepRap expect
- Key methods: `DriveStepsPerMm()`, `GetCurrentMachinePosition()`, `GetKinematics()`, `GetMainDDARing()`
- Initialized with reasonable defaults (80 steps/mm per axis)

**DDA.h**
- Minimal DDA class with methods CanMotion::FinishMovement() calls
- Key methods: `IsCheckingEndstops()`, `GetMoveStartTime()`, `GetClocksNeeded()`
- Includes `PrepParams` struct matching the firmware's structure:
  - For S-curve builds: phaseClocks[], accelerations, decelerations, jerk, distances[]
  - For non S-curve: accelClocks, steadyClocks, decelClocks, acceleration, deceleration
  - Common fields: totalDistance, topSpeed, useInputShaping

**DDARing.h**
- Placeholder for managing the DDA ring buffer
- Tracks machine position, scheduled/completed moves, simulation time
- Provides getters for reporting (top speed, acceleration, distance, duration)

**Kinematics/Kinematics.h**
- Stub base class that returns "none" as kinematics name
- Will be extended in Step 9.2 when real kinematics are needed

### 2. Implementations

Created implementations in `RRF/host/movement/`:

**MoveHost.cpp**
- Constructor initializes driveStepsPerMm[] to 80.0 (reasonable default)
- Allocates Kinematics and DDARing instances
- Implements basic position tracking

**DDAHost.cpp**
- Simple constructor initializing DDA state flags

**DDARingHost.cpp**
- Tracks machine position array
- Implements GetCurrentMachinePosition() and UpdateStartCoordinates()

**KinematicsHost.cpp**
- Minimal implementation returning stub diagnostics

### 3. Integration

**RepRap.h modifications**
- Added `#include <Movement/Move.h>`
- Added `Move& GetMove()` accessor method
- Added `Move move;` member variable

**Makefile updates**
- Added movement host files to HOST_SRC:
  - movement/MoveHost.cpp
  - movement/DDAHost.cpp
  - movement/DDARingHost.cpp
  - movement/KinematicsHost.cpp

## Build Result

✅ `make -C RRF/host` succeeds
✅ Produces 951K binary at build/host_rrf_bootstrap
✅ CanMotion.cpp links successfully against host facades
✅ No new dependencies on MCU subsystems

## What This Enables

The lightweight facades allow:
1. CanMotion.cpp to compile and link
2. Move instance to exist in RepRap
3. Basic position tracking
4. PrepParams and DDA structures to exist for future use

## What This Does NOT Do

This is NOT a working motion planner. The facades:
- Do not plan moves or compute acceleration profiles
- Do not transform coordinates via kinematics
- Do not generate motor steps
- Do not call CanMotion::AddAxisMovement() or CanMotion::AddExtruderMovement()
- Do not produce CAN movement packets

## Next Steps for Step 9.2

To actually generate CAN packets from G1 commands, the next developer needs to:

### 1. Decide on Integration Approach

**Option A: Pull in Real RRF Movement Stack**
- Add Movement/Move.cpp, Movement/DDA.cpp, Movement/DriveMovement.cpp to Makefile
- This brings ~10K+ lines of motion planning code
- Will require many additional stubs (EndstopManager, SmartDrivers, PauseState, etc.)
- Advantage: Gets real RRF motion planning semantics
- Disadvantage: High complexity, many dependencies

**Option B: Implement Simplified Motion Planning**
- Keep lightweight facades but extend them with basic planning logic
- Implement simplified DDA::Init() that converts G1 commands to motor steps
- Implement simplified DDA::Prepare() that:
  - Computes acceleration profile (populate PrepParams)
  - Calls CanMotion::StartMovement()
  - Calls CanMotion::AddAxisMovement() for each axis
  - Calls CanMotion::FinishMovement() to emit packets
- Use simplified kinematics (e.g., 1:1 for Cartesian, or basic Hangprinter formulas)
- Advantage: Lower complexity, fewer dependencies
- Disadvantage: May not match RRF motion planning exactly

### 2. Key Missing Pieces

Regardless of approach, you need:

**Coordinate Transform**
- Currently: No coordinate-to-steps transform
- Need: Either real Kinematics or stub transform
- For Cartesian: `steps[X] = position[X] * driveStepsPerMm[X]`
- For Hangprinter: More complex tether-length calculations

**Acceleration Profile Calculation**
- Currently: PrepParams exists but is never populated
- Need: Compute accelClocks, steadyClocks, decelClocks based on:
  - Distance to move
  - Requested feedrate
  - Max acceleration (from M201)
  - Max jerk (from M566)
- Formula: Trapezoidal or S-curve velocity profile

**CanMotion API Calls**
- Currently: CanMotion functions exist but are never called
- Need: In move preparation, call:
  ```cpp
  CanMotion::StartMovement();
  // For each CAN driver:
  CanMotion::AddAxisMovement(params, driverId, steps);
  uint32_t clocks = CanMotion::FinishMovement(dda, startTime, simulating);
  ```

**Driver Configuration**
- Currently: No axis-to-driver mapping
- Need: Parse or stub M584 (axis driver assignment)
- For CAN drivers: Create DriverId with board address and local driver number
- Example: Driver 121.0 = board 121, local driver 0

**Timing Simulation**
- Currently: No simulated clock advancement
- Need: Maintain `currentTime` in step-timer ticks
- Advance by PrepParams.TotalClocks() after each move
- Use for CanMotion::FinishMovement(dda, currentTime, false)

### 3. Suggested Approach for Step 9.2

I recommend **Option B** (simplified planning) for Step 9.2 because:
1. Gets CAN packets flowing faster
2. Easier to debug
3. Can validate CAN capture infrastructure
4. Can later replace with real RRF planning if needed

**Minimal implementation plan:**
1. Add G1 command handler in main.cpp that:
   - Parses G1 X Y Z E F parameters
   - Creates simplified DDA with target positions
2. Implement DDA::Init() stub that:
   - Converts target positions to motor steps (1:1 for now)
   - Calculates distance
3. Implement DDA::Prepare() stub that:
   - Computes simple trapezoidal profile
   - Populates PrepParams
   - Calls CanMotion APIs
4. Add simple time simulation loop
5. Verify CAN log contains movementLinearShaped records

## Files Created

```
RRF/host/include/Movement/
  Move.h              # Move facade header
  DDA.h               # DDA and PrepParams facade
  DDARing.h           # DDARing facade
  Kinematics/
    Kinematics.h      # Kinematics stub

RRF/host/movement/
  MoveHost.cpp        # Move implementation
  DDAHost.cpp         # DDA implementation
  DDARingHost.cpp     # DDARing implementation
  KinematicsHost.cpp  # Kinematics stub
```

## Files Modified

```
RRF/host/include/Platform/RepRap.h   # Added Move member
RRF/host/Makefile                    # Added movement host files
ai_docs/rrf_integration_build_plan.md  # Updated Step 9
```

## Key Learnings

1. **CanMotion is relatively self-contained**: It only needs PrepParams and DDA queries, not the full Move machinery.

2. **PrepParams is the key data structure**: It contains all the timing information (accel/steady/decel clocks) and motion parameters (acceleration, distance, speed) that CanMotion needs to build movement packets.

3. **The real complexity is in DDA::Prepare()**: This 500+ line function does:
   - Kinematics forward transform
   - Acceleration profile calculation
   - DriveMovement allocation and preparation
   - CanMotion API calls
   - Input shaping
   - Pressure advance calculations

4. **Move.cpp is even heavier**: 2000+ lines dealing with:
   - Driver configuration (M569, M584, M92, M201, M203, M566)
   - Endstop management
   - Probe handling
   - Smart driver communication
   - Bed leveling moves
   - Backlash compensation
   - Most of this is not needed for basic CAN packet generation

5. **The host shim strategy works well**: By creating minimal facades that satisfy CanMotion's interface expectations, we avoided pulling in 10K+ lines of firmware code and their transitive dependencies.

## Testing

The binary builds and runs:
```bash
$ ./build/host_rrf_bootstrap --help
Usage: host_rrf_bootstrap [--vsd <path>] [--run <file.gcode>] [--can-log <path|disable>]
```

However, no CAN packets are generated yet because:
- No moves are being planned (G1 commands don't call DDA::Prepare)
- PrepParams is never populated
- CanMotion APIs are never invoked

This is expected - Step 9.1 just laid the groundwork for Step 9.2.
