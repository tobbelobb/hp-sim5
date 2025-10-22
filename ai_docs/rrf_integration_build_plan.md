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
- Added SAME70-specific HAL shims (`host/include/DmacManager.h`, `host/include/pmc/pmc.h`) and aliased ARM-only intrinsics (`__fp16`, `float16_t`) so CANlib headers parse under x86\_64.
- Current compilation halts in `Pins_Duet3_MB6HC.h`/`Platform.cpp` because MCU-specific enumerations (`TcOutput`, `PwmOutput`, `IRQn`, peripheral IDs) are still undefined and CMSIS/NVIC helpers are absent. Pointer-sized logging in `Platform.cpp` (casting to `uint32_t`) also fails under 64-bit and needs host-side wrappers.
- Host-only `Pins_Host_MB6HC.h` now mirrors key Duet 3 constants (axes/heater limits, CAN counts, serial devices) while assigning `NoPin` placeholders. `RepRapFirmware.h` pulls in `Features_Host.h` to zero most peripherals before defaults fire, and the host Makefile defines `RRF_HOST_BUILD` so board selection flips automatically.
- Added first-wave host stubs (`PinDescription.h`, FreeRTOS headers, CMSIS-style NVIC/sys tick shims, generalized `Pin` conversions) and wired them through `Core.h/CoreIO.h`. Early compile of `Platform.cpp` still fails on larger SAME70/CMSIS symbols (`SysTick`, `MCAN*_IRQn`, DMA priorities), legacy macros (`REG_RSTC_SR`), and FreeRTOS task definitions; next step is to extend the shim set so these headers no longer require the real HAL.
- Reworked host core types to match Duet 3 expectations (`Pin` shrunk to uint8_t) so CAN message unions stay within bounds, and introduced host-only `Pins_Host_MB6HC.h` flags (`USART_SPI`, step timer constants) to keep SAME70 feature paths coherent without pulling the MCU HAL.
- Stubbed RTOS interfaces (`RTOSIface`, `Mutex`, `TaskBase`, `Read/WriteLockedPointer`, FreeRTOS headers) and overrode `Movement/StepTimer` with a chrono-backed shim; force-defined `SRC_MOVEMENT_STEPTIMER_H_` in the host build so firmware headers never include the MCU version.
- Wrapped diagnostics that printed raw object addresses with `#if !RRF_HOST_BUILD` and noted the remaining 64-bit formatting warnings—we still need to convert the bulk of `reply.printf("%u", size_t)` call-sites to use `PRIuPTR`/`PRIu64` once more of Platform is compiling.
- With these shims in place `g++ … Platform.cpp` compiled under the host configuration (warnings only). As we expand coverage the dependent modules (`Move.h`, `FansManager.h`, etc.) will drag in large portions of the runtime, so we should decide whether to stub the next subsystems (GCodes, Heat, Tools) before adding `Platform.cpp` to the Makefile target list.
- Introduced `host/include/Heating/Heat.h`, a host-only immediate-heating stub backed by a lightweight `HeaterState` struct; `SetTemperature` jumps straight to the requested target so motion code can treat heaters as always-ready. The latest `Platform.cpp` probe now fails later in the diagnostic path at the `sizeof(Heater)` print (no host `Heater` type yet), so the next shim sweep must either stub `Heating/Heater.h` or wrap that branch with `#if !RRF_HOST_BUILD`.
- Guarded the `DiagnosticTestType::PrintObjectSizes` branch with `#if !RRF_HOST_BUILD`, restoring the host compile after the Heater stub landed; while touching the Heating shim we also fixed template usage so `HeaterMatchesList` accepts both bed and chamber lists. Re-run `g++ … Platform.cpp` now succeeds (warnings about `%u` formatting remain on the backlog).
- Added host-only `Networking/Network.h` that mirrors the firmware surface while short-circuiting every operation to `GCodeResult::warningNotSupported`, keeps IP/mac/hostname state locally, and releases `OutputBuffer` replies so message dispatch paths do not leak. With that stub in place `g++ … RepRap.cpp` gets past the previous link block and now halts at `CoreNotifyIndices.h`; next action is to supply a host shim for those CoreN2G notification tables (or guard the include) before bringing the full RepRap controller object into the build.
- Introduced `host/include/CoreNotifyIndices.h`, extending the RTOS notification indices so host builds reuse the same slot numbers (`UartTx`, `Sdhc`, `AnalogIn`) without dragging in CoreN2G; this unblocks the RepRap controller compile path.
- Guarded hardware-only flows inside `RepRap.cpp` for the host build: `StartIap` now logs and aborts under `RRF_HOST_BUILD`, watchdog configuration is wrapped with `#if !RRF_HOST_BUILD`, and the spin-loop diagnostics fall back to `nullptr` stacks instead of inline ARM assembly.
- Small shim extensions: `FreeRTOS.h` defines `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`, `Core.h` adds `NvicPriorityHSMCI/Watchdog`, `task.h` and the StepTimer shim expose `pxTaskGetLastStackTop`, and `Heat` reuses `NamedEnum(HeaterStatus, …)` so status formatting retains `.ToBaseType()` helpers.
- Adjusted `Movement/Move.h` to include the host StepTimer shim when `RRF_HOST_BUILD` is set, keeping the MCU header intact for embedded builds while ensuring the chrono-backed timer is visible on x86_64.
- Compiling `RepRap.cpp` under the host flags now succeeds (warnings only) and produces `host/build/rrf_RepRap.o`; the next major subsystem that will need a stub before we add this TU to the Makefile is the fans/print monitoring stack (`FansManager`, `PrintMonitor`), which still expect ADC tach inputs and will choke once we pull their .cpp files into the build.
- Added host-only shims for `FansManager` and `PrintMonitor` (`host/include/Fans/FansManager.h`, `host/include/PrintMonitor/PrintMonitor.h`). Both report success for basic queries while returning `warningNotSupported` for configuration commands, and they track minimal in-memory state so object-model lookups keep working without touching tachometer hardware. With these in place, the standalone `g++ … RepRap.cpp` build continues to succeed (warnings only). The next likely blockers when we try to link will be the filament monitor and tool/extruder paths, which still depend on sensor IO.
- Stubbed the filament-monitor and tool/extruder surfaces for the host build (`host/include/FilamentMonitors/FilamentMonitor.h`, `host/include/Tools/Tool.h`, `host/include/Tools/Filament.h`). The new headers expose the locks and query helpers RepRap expects while collapsing all runtime behaviour to no-ops or `warningNotSupported`, keep tool/filament counts at zero, and return `nullptr` pointers so OM traversals short-circuit cleanly. `g++ … RepRap.cpp` still compiles (warnings only), so the next compile blockers will come from the GCodes/tool runtime rather than missing headers.
- Normalised the remaining `printf` calls that were mixing `%u` with `size_t`/`unsigned long` in `Platform.cpp` and `RepRap.cpp`, casting to explicit host-safe widths where the firmware expects small indices. Host compiles of both translation units now run cleanly wrt `-Wformat`, leaving only the expected macro and unused-parameter diagnostics.


## Step 6 — FreeRTOS shim

- Added a host runtime in `RRF/host/rtos/freertos_shim.cpp` that wraps `std::thread`, `std::mutex`, and `std::condition_variable` to emulate the FreeRTOS surfaces we touch today (tasks, queues, binary/recursive semaphores, task notifications, critical sections, tick/time helpers). Tasks spawn immediately as detached threads; handles map back to their `TaskBase` via the `StaticTask_t::hostContext` pointer.
- Reworked the host headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`) so they declare the real shim entry points instead of inline no-ops, and taught `RTOSIface/RTOSIface.h` to use `std::recursive_timed_mutex` and the shimmed APIs for mutexes, binary semaphores, task registration, and critical-section helpers. `TaskBase::AttachHostHandle` now records the native stack pointers so diagnostics like `pxTaskGetLastStackTop()` have data.
- Updated the host Makefile with `-pthread`, `-I.` for the new headers, automatic object-dir creation, and a new target for `rtos/freertos_shim.cpp`; `make -C RRF/host` builds `build/rtos/freertos_shim.o` and links the bootstrap without pulling in the embedded FreeRTOS sources.
- Known gaps: threads never stop on `vTaskDelete()` (we only flag `deleteRequested`); `vTaskSuspend()/Resume()` remain no-ops; scheduler state is coarse (`RUNNING` only); indefinite delays sleep for a long wall-clock chunk; queue/semaphore timeouts are millisecond-granularity only. We will need a cooperative loop and teardown logic once more subsystems actually schedule work, but this shim is enough to let higher layers compile and to start wiring the G-code pipeline in Step 7.

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

## Step 6 — FreeRTOS shim

- Added `RRF/host/rtos/freertos_shim.cpp` and matching headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `RTOSIface/RTOSIface.h`) to satisfy the firmware’s RTOS interface while routing calls to lightweight host implementations.
- The shim launches detached `std::thread` workers for `xTaskCreate`, exposes stub queues/semaphores, and guards critical sections with a recursive mutex. Threads observe an atomic `deleteRequested` flag so they can terminate cooperatively when firmware code calls `vTaskDelete`.
- Host Makefile now compiles the shim with the bootstrap sources, allowing `Platform.cpp`/`RepRap.cpp` to link without the embedded FreeRTOS port.
- Future refinement: provide deterministic scheduling semantics (avoid 24 h sleeps for `portMAX_DELAY`), implement cleanup for task handles, and add unit tests around queue behaviour.

## Step 7 — Virtual SD & G-code pipeline (in progress)

- Introduced filesystem-backed storage shims: `host/include/Storage/{FileStore,FileWriteBuffer,MassStorage}.h` plus `host/storage/HostFileStore.cpp` and `HostMassStorage.cpp` translate firmware file operations onto `std::filesystem`/`std::fstream`. Volume `0:/` now maps to a configurable host root (`MassStorage::SetHostRoot`) with automatic `sys/`, `gcodes/`, and `firmware/` directory creation.
- Host Makefile pulls in firmware’s `Storage/CRC32.cpp` and RRFLibraries helpers (`StringRef`, `SafeVsnprintf`, `StringFunctions`, `Strnlen`) so `M36`/metadata paths behave like the embedded build. `GetFileInfo` reports size, mtime, and appends a CRC32 token to `generatedBy` pending a cleaner reporting hook.
- CLI updates in `host/src/main.cpp`: `--vsd <path>` selects the virtual SD root, `--run <file>` records a target gcode (execution still stubbed), and MassStorage initialises the directory skeleton on launch.
- Began compiling the G-code stack (`GCodeMachineState`, `GCodeInput`, `GCodeQueue`, `GCodes[2–7]`, `GCodeBuffer` parsers). Host shims grew to cover `Stream::readBytes`, tool-change flags (`TFreeBit/TPreBit/TPostBit`), character helpers (`isAlpha/isDigit/isAlnum`), random number placeholder, and a stub stack-pointer accessor.
- Current link blockers sit in `ExpressionParser`: object-model types (`ExpressionValue`, `GlobalVariables`, `StringHandle`), diagnostics (`SoftwareReset`), and named-enum helpers are still missing from the host target, yielding unresolved externals. Next iteration must stub those pieces (or guard the code paths) before we can instantiate `GCodes` from `main`.
- Once ExpressionParser links, scrub the remaining `%u` vs `size_t` format warnings and continue trimming unused feature macros (`HAS_WIFI_NETWORKING`, `HAS_AUX_DEVICES`, etc.) so the host focus stays on motion planning and file playback.
- Started object-model host shim: added host overrides for `StringHandle`, `ArrayHandle`, freelist, and heap headers so `ExpressionValue` union compiles on x86_64; provided `SoftwareReset` stub, FreeRTOS-aware queue/sem shims, and storage for virtual SD. Implemented minimal `ObjectExplorationContext`, `VariableSet`, and `GlobalVariables` so `ExpressionParser` can resolve symbol references. Linking still fails because the host build lacks stubs for `reprap` accessors (`GetPlatform()`, global variables), numeric helpers (`NumericConverter`, `fastSqrtf`), and platform queries (`Platform::SysFileExists`); these need either host implementations or conditional code paths before GCodes can execute end-to-end.
- Iteration 7A: introduced `platform/PlatformHost.cpp` to supply host-side `Platform` ctor/logging/virtual SD helpers and trimmed the Makefile to just the parser-heavy `GCodeBuffer` units plus `GCodeException`/`GCodeMachineState`. Link still fails because we have not yet provided a host `RepRap` singleton (`reprap` and associated `GCodes` facade), `FileGCodeInput`/`OutputBuffer` adapters, or the object-model vtables the parser expects. Next pass will either stub these dependencies or further reduce the firmware source set so the Expression parser can link in isolation.

- [In progress] Host stubs for RepRap/GCodes: drafted minimal Platform/RepRap/GCodes replacements and began wiring FileGCodeInput host implementations, but ExpressionParser and object-model hooks still pull in heavy dependencies (MassStorage, reprap.GetObjectValueUsingTableNumber, NamedEnumLookup). Build currently fails inside ExpressionParser pending additional guards/stubs.
- Iteration 7B: wired the parser’s file helpers to the host storage shim by including `Storage/FileStore.h`/`MassStorage.h`, added a host implementation of `RepRap::GetObjectValueUsingTableNumber` that returns neutral values (and warns when unsupported paths are queried), and fixed the RRFLibraries host build by aligning `NamedEnumLookup`’s signature with the firmware ABI. `make -C RRF/host` now links through the ExpressionParser object model surface, unblocking the next wave of G-code execution work.
- Iteration 7C: taught the host object-model shim to answer the Hangprinter-critical queries `inputs[].axesRelative` and `move.axes[].userPosition` by parsing the selector string inside `RepRap::GetObjectValueUsingTableNumber` (`RRF/host/platform/RepRapHost.cpp#L24`). Stub `GCodes` now tracks per-input relative mode and a small user-position array (`RRF/host/include/GCodes/GCodes.h#L32`), so expressions reading those paths return deterministic data while every other branch remains guarded as “unsupported”.
- Iteration 7D: replaced the hard-coded `GCodes` placeholders with real state updates driven by the parser. The `--run` CLI path now streams a host-side `FileStore` through `GCodeBuffer`, handles `G90/G91/G92/G0/G1`, toggles `inputs[].axesRelative`, and accumulates `move.axes[].userPosition` in the stub `GCodes` class (`RRF/host/src/main.cpp`, `RRF/host/include/GCodes/GCodes.h`). Executing a sample file under `host_rrf_bootstrap --run` mutates the object-model values as expected, giving us a working loop for later CAN capture work.
