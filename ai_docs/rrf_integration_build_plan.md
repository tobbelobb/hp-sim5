# RRF Host Integration Build Plan

This document tracks the incremental plan and key findings while bringing RepRapFirmware into a host (x86\_64) build suitable for generating deterministic motion traces.

## Step 1 — Extract existing build configuration

- **Source:** `RRF/ReprapFirmware/.cproject`, `RRF/RepRapFirmware.wiki/Building-RepRapFirmware.md`, `RRF/CoreN2G/README.md`, `RRF/CANlib/doc/Duet3CAN-FDProtocol.md`
- Cross build targets ship with Eclipse-managed makefiles invoking `arm-none-eabi-g++` with `-mcpu=cortex-m7`, `-mthumb`, floating-point hard ABI, and size-optimised `-Os`.
- Duet 3 MB6HC RTOS configuration exports include search paths for:
  - Core HAL trees (`CoreN2G/src`, `CoreN2G/src/SAM4S_4E_E70/SAME70/**`, CMSIS 5.4.0 headers, ASF drivers)
  - Firmware sources (`ReprapFirmware/src`, `ReprapFirmware/src/Hardware/SAME70`, `ReprapFirmware/src/Config`, `ReprapFirmware/src/Networking/**`)
  - Third-party dependencies (FreeRTOS portable layer, WiFi socket server headers, CANlib, RRFLibraries)
- Preprocessor defines used in the firmware build: `__SAME70Q20B__`, `RTOS`, `DUET3_MB6HC`, `MQTTC_PAL_FILE="Networking/MQTT/mqtt_pal.h"`, `_XOPEN_SOURCE`, and a `noexcept=` compatibility define for the C compiler configuration.
- Link stage expects static libraries produced by CANlib, CoreN2G, RRFLibraries, FreeRTOS alongside the firmware objects, and links against `supc++`.
- Platform exclusions indicate we can ignore SBC/web control during early host bring-up (`sourceEntries` exclude `src/SBC`, `src/Networking/LwipEthernet`, etc.).
- CoreN2G requires the application to supply `main()` that calls `CoreInit()`, and to implement `AppInit`, `AppMain`, pin description queries, and systick hooks. This aligns with the need for a host-side HAL shim.
- CAN protocol spec (Duet3 CAN FD) depends on a 48 MHz step clock and master-scheduled start times. Captured frames must preserve timestamp fields to reconstruct queue_step equivalents.

## Step 2 — Makefile skeleton

- Added `RRF/host/Makefile` to drive a host build with GNU++17, out-of-source objects under `host/build`.
- Minimal entry point lives in `RRF/host/src/main.cpp`; current target `host_rrf_bootstrap` just prints a banner, verifying the toolchain and folder conventions.
- Successful build via `make -C RRF/host` confirms we can extend the Makefile incrementally without cross toolchain dependencies.
- Host make rules intentionally avoid the size-constrained `-Os` flags used on MCU builds; we'll favour clarity and diagnostics on x64 where binary size is irrelevant.

## Step 3 — Add first firmware translation unit

- Makefile now builds `ReprapFirmware/src/Version.cpp` alongside the host entry point, proving we can consume firmware headers on the host toolchain.
- Added shared include flags for `../ReprapFirmware/src` and `../RRFLibraries/src` to satisfy dependencies on `Version.h` and `ecv_duet3d.h`.
- Build flags now declare the Duet 3 MB6HC identity (`__SAME70Q20B__`, `DUET3_MB6HC`, `SAME70=1`) so every subsequent translation unit sees the CAN-capable board configuration instead of the Duet 2 defaults.
- Host bootstrap prints `VERSION`, `DateText`, and `TimeSuffix`, letting us sanity-check that const data links correctly without MCU-specific libraries.

## Step 4 — Fake HAL headers

- Added host shims `RRF/host/include/Core.h` and `RRF/host/include/CoreIO.h` to intercept `<Core.h>` / `<CoreIO.h>` at a higher include priority.
- Current stubs define the processor feature macros, simple `PinMode` enum, dummy watchpoint registers, and no-op GPIO helpers so firmware sources can include them without dragging in MCU-specific ASF headers.
- Declared placeholder heap pointers and time delay helpers; these will evolve alongside the scheduler shim when we bring in FreeRTOS dependencies.

## Step 5 — Compile Platform subset

- First dry-run compile of `ReprapFirmware/src/Platform/Platform.cpp` exposed the MCU-facing dependencies we must neutralise: `Pins.h` expects board macros (`__SAME70Q20B__`, `DUET3_MB6HC`), CoreN2G headers like `Interrupts.h`, `AnalogIn/Out.h`, `UniqueIdBase.h`, and SAME70-specific device files (`Hardware/SAME70/Devices.h` pulls in `AsyncSerial`, `USARTClass`, etc.).
- Introduced initial host stubs for `CoreTypes.h`, `Interrupts.h`, `{AnalogIn,AnalogOut}.h`, `UniqueIdBase.h`, cache control, async serial/USB (`AsyncSerial.h`, `SerialCDC.h`, `Wire.h`, `USARTClass.h`), and provided runtime stand-ins via `host/src/devices_stub.cpp`, plus extra include dirs for `Hardware/SAME70` during exploratory compiles.
- Host stub now mirrors the Duet 3 device namespace (`serialUart1`, `serialUart2`, guarded `serialWiFi`) so firmware sources can pull in the SAME70 board descriptors without size-oriented Duet 2 assumptions.
- Remaining blockers before this file compiles cleanly: large device headers (`AsyncSerial.h`, SAM4E peripherals), feature macros (e.g. `SUPPORT_REMOTE_COMMANDS`, `HAS_AUX_DEVICES`) that we should pin to zero for the host build, and a strategy to compartmentalise DueXn expansion logic so it can log instead of touching I2C/GPIO.


## Step 6 — FreeRTOS shim (pending)

- Replace FreeRTOS usage with a cooperative loop: implement small scheduler (std::thread + queues) matching the API points RRF uses (TaskHandle, QueueHandle). Provide adapters in host/rtos/ calling into std::condition_variable.
- Update Makefile target list to exclude actual FreeRTOS sources but compile host/rtos/freertos_shim.cpp.

## Step 7 - Bring in G-code pipeline (pending)

- Compile GCodes/GCodes.cpp#L1, GCodes/GCodeBuffer.cpp#L1, and config helpers.
- Stub mass-storage calls from Storage module to the host filesystem root; map virtual SD to RRF/host/vsd.

## Step 8 - CAN message capture (pending)

- Build CANlib/src sources directly (needs only headers) to keep message formats, and implement a host CanInterface class that writes serialized frames to a file or ring buffer before actual bus access.
- Mirror the Duet3 expansion behaviour by forcing Platform to enumerate all drives as external over CAN (see ReprapFirmware/src/ExpansionManager.cpp#L1) and route CanMotion::SendMove into our capture sink.

## Step 9 - Gradual file adds in Makefile (pending)

- Each iteration, add one directory group: (a) Movement/Kinematics, (b) Movement/StepperDrivers, (c) Heating (stub out ADC/PID hardware but keep config parsing), (d) Tools for extruders.
- After every addition, run make build-host to ensure no new link errors and adjust stubs accordingly.

## Step 10 - Echo test via DuetSoftwareFramework (pending)

- Build DSF’s code console client (RRF/DuetSoftwareFramework Makefile) and start it pointing to a mocked UNIX socket implemented in host firmware (expose --socket-file override, feed G-code lines to firmware object).
- Add regression script that sends M118 hello and confirm the host firmware returns the same string, matching DSF handshake.

## Step 11 - Config.g handling (pending)

- Mount DSF virtual SD path into host firmware; invoke existing GCodes::ReadAndExecuteConfig to parse config.g.
- Add test target that loads minimal config.g and asserts platform state (e.g. steps/mm) via a JSON dump.

## Step 12 - Batch mode output (pending)

- Convert captured CAN movement packets into deterministic trace (binary + human-readable). Option 1: write raw frames plus timestamp into .duetcan; option 2: adapt to our .serial schema by translating CanMotion payloads into abstract queue_step
  events.
- Build CLI switches to choose output format and to run headless (no DSF).

## Step 13 - Interfaces to stub long-term (pending)

- WiFi/Ethernet: replace Networking initialisation with no-ops but keep API surfaces for future (only needed for config M commands).
- SPI/I2C to stepper drivers: use host logging wrappers; the CAN capture path bypasses these.
- Web control: disable DWC integration by short-circuiting SBC/WebServer modules; rely on DSF WebSocket for external UI if needed.

## Step 14 - Validation & next steps (pending)

- Unit-test CAN serialization against CANlib/src/CanMessageFormats.cpp#L1 reference tables.
- Plan follow-up tasks: integrate timing scale control, implement real-time to simulated-time mapping, expand config coverage, document output format.

Potential user actions:

1. Approve creation of host scaffolding directory and stub headers.
2. Decide whether .duetcan should mirror raw CAN frames or normalized step events.
