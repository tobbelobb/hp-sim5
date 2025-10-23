# Overview
We're in the middle of a large feature: ReprapFirmware batch mode on host.
We have divided the work into three main phases:

  Phase 1: Stub or shim hardware dependencies so most of ReprapFirmware's modules compile on our host machine (x86_64).
  Phase 2: Filesystem interaction: Reads a gcode input file and a config.g file and write output movement commands (steps + timing).
  Phase 3: Compile the movement subsystem and link it cleanly into our host binary.

We just arrived at Phase 3.

The approach taken is to define all motors as CAN-attached external drivers, and to capture all the Movement/Motion commands or packets by writing them to a file.
This file will later be used by a simulator to check exactly which motor movements are calculated by ReprapFirmware, and how well they might work on a "real" (simulated) machine.

# Files and Directories
The code lives in the RRF directory.
The main upstream ReprapFirmware code lives inside RRF/ReprapFirmware.
Required upstream libraries live in RRF/CANlib, RRF/RRFLibraries, among others.
We try to change the upstream code as little as possible, mostly we're just fixing include bugs and put #define guards using the RRF_HOST_BUILD variable.
Our code lives in RRF/host/.

# Current Status
The x86_64 host build is very much a bootstrap.
It sets up a virtual SD rooted at a host directory and provides stubs for the platform, storage and object‑model classes.
The executable (host_rrf_bootstrap) accepts a G‑code file via --run and an optional --can‑log path.
When it runs, the program currently does the following:

 - Sets up a virtual SD directory using MassStorage::SetHostRoot and opens the requested G‑code file.
 - Reads the file byte‑by‑byte, decodes G‑codes and handles only a handful of commands. G0 and G1 moves call ProcessLinearMove, which merely updates the userPositions array; G90/G91 switch between absolute/relative positioning and G92 sets the current user position. These handlers do not call the RRF motion planner - they just record new positions.
 - Prints the final user positions after the file has been read.
 - If CAN logging is enabled, opens a log file and uses the HostCanCapture infrastructure. HostCanCapture::LogMotion writes each movementLinearShaped CAN message as a JSON record, recording the destination board, execution time, acceleration and deceleration clocks and, for each driver, the step count or extrusion amount

There are currently no calls to the RRF movement subsystem.
No motor‑command packets are currently generated, and the CAN‑capture code does not see motion messages in its log.

A lot of the platform/host glue is already being compiled. The RRF/host/Makefile pulls in:

 - Host/platform & storage:
   RRF/host/platform/{PlatformHost.cpp, RepRapHost.cpp, GCodesHost.cpp, DebugPrintHost.cpp, TasksHost.cpp, GetFloatFormatStringHost.cpp},
   RRF/host/storage/{HostMassStorage.cpp, HostFileStore.cpp},
   RRF/host/object_model/{ObjectModelHost.cpp, VariableHost.cpp, GlobalVariablesHost.cpp, ObjectExplorationContextHost.cpp},
   RRF/host/networking/MacAddressHost.cpp, so the basic I/O, logging, VSD, object-model scaffolding, etc., are available at link time.
 - RRF core bits used by the host shell:
   G-code buffer/lexer/parser (RRF/ReprapFirmware/src/GCodes/GCodeBuffer/{ExpressionParser.cpp, StringParser.cpp, BinaryParser.cpp, GCodeBuffer.cpp}, machine-state and exceptions, CRC helpers, and RRF/ReprapFirmware/src/CAN/CanMotion.cpp - i.e., the code that would emit CAN movement packets if the motion pipeline is exercised.
 - CAN capture is wired up in the host: RRF/host/can/CanInterfaceHost.cpp calls the capture sink, and RRF/host/src/main.cpp configures the log sink via HostCanCapture::Configure(...) and prints the chosen path when enabled

# High level view of what still needs to be done to reach the goal

 1. Integrate the full RRF motion planner. The current ProcessLinearMove just updates the user positions; it does not call RRF/ReprapFirmware's Move/Stepper classes.
 2. Implement a complete G‑code interpreter. At the moment only a few G‑codes are recognised. To simulate a real printer, the host build must support homing, probing, tool changes and M‑codes that configure speeds, acceleration, jerk, micro‑stepping, etc. Many of these settings come from config.g; reading config.g and applying its commands is necessary so that moves are computed in the correct units and with the correct kinematics.
 3. Handle extruder and other motion types. The CAN logger currently captures only movementLinearShaped packets. RRF can send other motion‑related message types (trapezoidal moves, raw step counts, set‑motor‑current commands, pause/resume commands, etc.). The logger should be extended to handle all relevant CAN packet types, and CanInterfaceHost::SendMotion should be invoked for each of them.
 4. Support external driver configuration. The current host build sets a fixed axis count and does not use config.g to assign axes to specific CAN drivers. For correct simulation, the mapping between logical axes (X/Y/Z/…) and CAN‑based external drivers must be read from the configuration so that step counts and timings are generated for the right driver addresses.
 5. Expand the object model and platform stubs. To execute macros or conditional G‑code, at least a subset of the RRF object model and platform functions needs to be implemented. This includes handling timers, endstop states, heaters/fans, and other non‑motion commands so that real print files can be interpreted without modification.
 6. Add timing and clock simulation. The host build currently does not simulate real‑time execution. When integrating the movement subsystem, you will need to simulate the tick counter or clock so that whenToExecute, accel_clocks, steady_clocks and decel_clocks fields in the movement messages reflect realistic timing.

See the pending steps in ai_docs/rrf_integration_build_plan.md for more details on the roadmap/development plan.
That plan is always up to date and we're just now ending Step 8 and starting on Step 9.
The previous work iteration was "Iteration 8C".

# What to Focus on
We're in the middle of figuring out how to integrate the RRF motion planner (the Mover/Stepper classes), ie point 1. above.
See Step 9 in ai_docs/rrf_integration_build_plan.md for more details.

Here's what the previous coder wrote after making the previous commit:

"""
First exploratory compile of `Movement/Move.cpp` exposed broad dependencies on real MCU services (`RepRap::IsStopped`, `GCodes::ReadMove`, SmartDrivers, PauseState
enums). Conclusion: rather than drag the full Move stack into the host build, we need a dedicated Move/DDA facade that reports the minimum data required for CAN logging
while keeping host shims manageable.

Suggested next steps:

  1. Define a lightweight host Move/DDA interface (types like PrepParams, limited Move API, stubbed kinematics) that satisfies CanMotion.cpp without invoking MCU subsystems.
  2. Extend the existing host GCodes facade with the movement callbacks the façade will need (ReadMove, pause state accessor).
  3. Reattempt compiling Move.cpp/DDA.cpp behind the new façade and tighten stubs (endstop manager, SmartDrivers) so the CAN capture path finally sees real packets.

"""

I think we should heed the previous coder's advice.
Don't pull in the complete G-code interpreter or Move stack just yet but try to look for the core of Movement/Move.cpp and try to at least pull that in.

Divide Step 9 in ai_docs/rrf_integration_build_plan.md into two substeps:
 - Step 9.1: lightweight host Move/DDA interface
 - Step 9.2: full host Move/DDA interface

Then just do Step 9.1 and collect information that will be useful for the next programmer who will have to do Step 9.2.

# Take Notes
All coders working on this feature have read (and sometimes slightly changed) plans from, as well as written implementation notes into ai_docs/rrf_integration_build_plan.md
After you finish coding I expect you to write a little note under Step 8 and/or Step 9 describing your key findings.
