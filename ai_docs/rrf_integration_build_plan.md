Build Plan

  - Preserve the motion/CAN path: keep ReprapFirmware/src/Movement#L1, ReprapFirmware/src/CAN/CanMotion.cpp#L1, ReprapFirmware/src/CAN/CanInterface.cpp#L1, and the G-code front end in ReprapFirmware/src/GCodes/GCodes.cpp#L1 intact while stubbing
    peripherals elsewhere (WiFi/SBC transport in ReprapFirmware/src/Networking#L1, HTTP/UI in ReprapFirmware/src/SBC#L1, SPI/I2C helpers in ReprapFirmware/src/Platform#L1). These pieces produce the step-scheduling CAN frames we must emit, so they get the
    fake HAL plumbing rather than removals.

  1. Baseline research

  - Extract include paths and compiler flags from .cproject#L1 and RepRapFirmware.wiki/Building-RepRapFirmware.md#L1 to understand existing build groups and inter-project dependencies (RepRapFirmware, CoreN2G, FreeRTOS, RRFLibraries, CANlib). Note
    hardware assumptions from CoreN2G/README.md#L3 and CAN timing from CANlib/doc/Duet3CAN-FDProtocol.md#L1.

  2. Makefile skeleton that builds one file

  - Create RRF/host/Makefile that compiles a trivial host stub (e.g. host/main.cpp) plus ReprapFirmware/src/Version.cpp#L1 with g++ -std=gnu++17.
  - Verify include discovery by exporting a generated compile_commands.json snippet or using make VERBOSE=1.

  3. Extend Makefile to compile a firmware TU

  - Add RepRapFirmware/src/RepRapFirmware.cpp#L1 and minimal dependencies, using objdump -p .cproject data to mirror include directories (src, RRFLibraries/src, CANlib/src).
  - Stub out missing symbols with empty host files under host/stubs/ so the compilation links (expect many undefined external references at this stage).

  4. Introduce fake HAL headers

  - Create host equivalents for <Core.h>/<CoreIO.h> interfaces referenced by Platform code, defining data structures but only no-op functions.
  - Provide host implementations for AppInit, AppMain, and clock helpers described in CoreN2G/README.md#L27-45.
  - Adjust Makefile to include host/hal before CoreN2G headers so we can link without pulling MCU objects.

  5. Compile Platform subset

  - Add ReprapFirmware/src/Platform/Platform.cpp#L1 and ReprapFirmware/src/Platform/Board.cpp#L1 with host stubs for GPIO, timers, SPI, I2C; use #ifdef HOST_BUILD wrappers to bypass hardware setup while keeping timing/stepper abstractions.
  - Ensure the fake HAL offers deterministic tick sources (monotonic counter) to back CAN scheduling.

  6. Integrate FreeRTOS replacement

  - Replace FreeRTOS usage with a cooperative loop: implement small scheduler (std::thread + queues) matching the API points RRF uses (TaskHandle, QueueHandle). Provide adapters in host/rtos/ calling into std::condition_variable.
  - Update Makefile target list to exclude actual FreeRTOS sources but compile host/rtos/freertos_shim.cpp.

  7. Bring in G-code pipeline

  - Compile GCodes/GCodes.cpp#L1, GCodes/GCodeBuffer.cpp#L1, and config helpers.
  - Stub mass-storage calls from Storage module to the host filesystem root; map virtual SD to RRF/host/vsd.

  8. CAN message capture

  - Build CANlib/src sources directly (needs only headers) to keep message formats, and implement a host CanInterface class that writes serialized frames to a file or ring buffer before actual bus access.
  - Mirror the Duet3 expansion behaviour by forcing Platform to enumerate all drives as external over CAN (see ReprapFirmware/src/ExpansionManager.cpp#L1) and route CanMotion::SendMove into our capture sink.

  9. Gradual file adds in Makefile

  - Each iteration, add one directory group: (a) Movement/Kinematics, (b) Movement/StepperDrivers, (c) Heating (stub out ADC/PID hardware but keep config parsing), (d) Tools for extruders.
  - After every addition, run make build-host to ensure no new link errors and adjust stubs accordingly.

  10. Echo test via DuetSoftwareFramework

  - Build DSF’s code console client (RRF/DuetSoftwareFramework Makefile) and start it pointing to a mocked UNIX socket implemented in host firmware (expose --socket-file override, feed G-code lines to firmware object).
  - Add regression script that sends M118 hello and confirm the host firmware returns the same string, matching DSF handshake.

  11. Config.g handling

  - Mount DSF virtual SD path into host firmware; invoke existing GCodes::ReadAndExecuteConfig to parse config.g.
  - Add test target that loads minimal config.g and asserts platform state (e.g. steps/mm) via a JSON dump.

  12. Batch mode output

  - Convert captured CAN movement packets into deterministic trace (binary + human-readable). Option 1: write raw frames plus timestamp into .duetcan; option 2: adapt to our .serial schema by translating CanMotion payloads into abstract queue_step
    events.
  - Build CLI switches to choose output format and to run headless (no DSF).

  13. Interfaces to stub long-term

  - WiFi/Ethernet: replace Networking initialisation with no-ops but keep API surfaces for future (only needed for config M commands).
  - SPI/I2C to stepper drivers: use host logging wrappers; the CAN capture path bypasses these.
  - Web control: disable DWC integration by short-circuiting SBC/WebServer modules; rely on DSF WebSocket for external UI if needed.

  14. Validation & next steps

  - Unit-test CAN serialization against CANlib/src/CanMessageFormats.cpp#L1 reference tables.
  - Plan follow-up tasks: integrate timing scale control, implement real-time to simulated-time mapping, expand config coverage, document output format.

  Potential user actions:

  1. Approve creation of host scaffolding directory and stub headers.
  2. Decide whether .duetcan should mirror raw CAN frames or normalized step events.
