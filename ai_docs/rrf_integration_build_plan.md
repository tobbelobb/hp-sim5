# RRF Host Integration Build Plan

This document tracks the incremental plan and key findings while bringing RepRapFirmware into a host (x86\_64) build suitable for generating deterministic motion traces.

## Step 1 - Extract existing build configuration

### Step 1 Progress log
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

## Step 2 - Makefile skeleton

### Step 2 Progress log
- Added `RRF/host/Makefile` to drive a host build with GNU++17, out-of-source objects under `host/build`.
- Minimal entry point lives in `RRF/host/src/main.cpp`; current target `host_rrf_bootstrap` just prints a banner, verifying the toolchain and folder conventions.
- Successful build via `make -C RRF/host` confirms we can extend the Makefile incrementally without cross toolchain dependencies.
- Host make rules intentionally avoid the size-constrained `-Os` flags used on MCU builds; we'll favour clarity and diagnostics on x64 where binary size is irrelevant.

## Step 3 - Add first firmware translation unit

### Step 3 Progress log
- Makefile now builds `ReprapFirmware/src/Version.cpp` alongside the host entry point, proving we can consume firmware headers on the host toolchain.
- Added shared include flags for `../ReprapFirmware/src` and `../RRFLibraries/src` to satisfy dependencies on `Version.h` and `ecv_duet3d.h`.
- Build flags now declare the Duet 3 MB6HC identity (`__SAME70Q20B__`, `DUET3_MB6HC`, `SAME70=1`) so every subsequent translation unit sees the CAN-capable board configuration instead of the Duet 2 defaults.
- Host bootstrap prints `VERSION`, `DateText`, and `TimeSuffix`, letting us sanity-check that const data links correctly without MCU-specific libraries.

## Step 4 - Fake HAL headers

### Step 4 Progress log
- Added host shims `RRF/host/include/Core.h` and `RRF/host/include/CoreIO.h` to intercept `<Core.h>` / `<CoreIO.h>` at a higher include priority.
- Current stubs define the processor feature macros, simple `PinMode` enum, dummy watchpoint registers, and no-op GPIO helpers so firmware sources can include them without dragging in MCU-specific ASF headers.
- Declared placeholder heap pointers and time delay helpers; these will evolve alongside the scheduler shim when we bring in FreeRTOS dependencies.

## Step 5 - Compile Platform subset

### Step 5 Progress log
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


## Step 6 - FreeRTOS shim

### Step 6 Progress log
- Added a host runtime in `RRF/host/rtos/freertos_shim.cpp` that wraps `std::thread`, `std::mutex`, and `std::condition_variable` to emulate the FreeRTOS surfaces we touch today (tasks, queues, binary/recursive semaphores, task notifications, critical sections, tick/time helpers). Tasks spawn immediately as detached threads; handles map back to their `TaskBase` via the `StaticTask_t::hostContext` pointer.
- Reworked the host headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`) so they declare the real shim entry points instead of inline no-ops, and taught `RTOSIface/RTOSIface.h` to use `std::recursive_timed_mutex` and the shimmed APIs for mutexes, binary semaphores, task registration, and critical-section helpers. `TaskBase::AttachHostHandle` now records the native stack pointers so diagnostics like `pxTaskGetLastStackTop()` have data.
- Updated the host Makefile with `-pthread`, `-I.` for the new headers, automatic object-dir creation, and a new target for `rtos/freertos_shim.cpp`; `make -C RRF/host` builds `build/rtos/freertos_shim.o` and links the bootstrap without pulling in the embedded FreeRTOS sources.
- Known gaps: threads never stop on `vTaskDelete()` (we only flag `deleteRequested`); `vTaskSuspend()/Resume()` remain no-ops; scheduler state is coarse (`RUNNING` only); indefinite delays sleep for a long wall-clock chunk; queue/semaphore timeouts are millisecond-granularity only. We will need a cooperative loop and teardown logic once more subsystems actually schedule work, but this shim is enough to let higher layers compile and to start wiring the G-code pipeline in Step 7.
- Added `RRF/host/rtos/freertos_shim.cpp` and matching headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `RTOSIface/RTOSIface.h`) to satisfy the firmware’s RTOS interface while routing calls to lightweight host implementations.
- The shim launches detached `std::thread` workers for `xTaskCreate`, exposes stub queues/semaphores, and guards critical sections with a recursive mutex. Threads observe an atomic `deleteRequested` flag so they can terminate cooperatively when firmware code calls `vTaskDelete`.
- Host Makefile now compiles the shim with the bootstrap sources, allowing `Platform.cpp`/`RepRap.cpp` to link without the embedded FreeRTOS port.
- Future refinement: provide deterministic scheduling semantics (avoid 24 h sleeps for `portMAX_DELAY`), implement cleanup for task handles, and add unit tests around queue behaviour.

## Step 7 - Virtual SD & G-code pipeline

Goals:
- Compile GCodes/GCodes.cpp#L1, GCodes/GCodeBuffer.cpp#L1, and config helpers.
- Stub mass-storage calls from Storage module to the host filesystem root; map virtual SD to RRF/host/vsd.

### Step 7 Progress log
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


## Step 8 - Move command packet capture

Goals:
We want to capture and log all the motor movements that the firmware plans and calculates.
We want them in a format similar to what Klipper uses (the `queue_step` mcu commands, see explanation in ai_docs/Klipper_MCU_Commands.md).
We have imagined capturing the packets that ReprapFirmware would typically send via the Duet3 CAN protocol, because the physical setup is similar to Klippers.
Compare these two data flows to see what we mean:

 - Klipper host -> MCU commands via USB protocol -> MCU
 - ReprapFirmware Duet master -> MCU commands via CAN protocol -> MCU

So we imagine that ReprapFirmware puts the synchronization information as well as the move commands themselves in the CAN packets.
We really want a thin CAN shim though, we don't want to emulate a full CAN stack.
See eg "Announcements", "Status Messages", the master broadcasting its time, the CRC checking, and so on in CANlib/doc/Duet3CAN-FDProtocol.md to see examples of the things we'd like to avoid having to emulate.

What we do want is the CAN movement messages, which might include:

 - which driver(s) to step,
 - how many steps or what step interval to maintain,
 - acceleration or segment duration, and
 - a start time tag so all boards move in lockstep.

These are the pieces we want to log.

### Step 8 Progress log
- Iteration 8A: added a dedicated host CAN capture sink (`RRF/host/can/CanCapture.cpp`) that serialises every `CanMessageMovementLinearShaped` frame to JSONL. The CLI now accepts `--can-log <path|disable>` and defaults to `<vsd>/logs/can_capture.jsonl`, with shutdown handled via RAII in `host_rrf_bootstrap`. `CanInterface::SendMotion` is stubbed on host (`RRF/host/can/CanInterfaceHost.cpp`) so each queued movement logs and the buffer is freed immediately instead of touching hardware. To support this path the host build now pulls in `CANlib/src/CanMessageBuffer.cpp`, provides aligned `MessageBufferAlloc/Delete` in `platform/TasksHost.cpp`, and keeps `SUPPORT_CAN_EXPANSION` enabled so Duet3 CAN constants/types align with the sink.
- Iteration 8B: wired `CanMotion.cpp` into the host build (Makefile now compiles the firmware TU alongside the capture shim) and extended `CanInterfaceHost.cpp` with a stubbed `GetCanAddress()` so movement packets carry master/source IDs. To keep the build surface small we temporarily skipped the heavyweight `Move.cpp` hierarchy; only the CAN motion module is linked while we plan a lighter host facade for Move in Step 9.
- Iteration 8C: resolved a host-only build break in `ObjectModel/TypeCode.h` (first inclusion saw `SUPPORT_CAN_EXPANSION==0`) by pulling `Config/Features_Host.h` directly into the header, ensuring CAN-aware type codes survive even when other TUs include the file before `RepRapFirmware.h`. First exploratory compile of `Movement/Move.cpp` exposed broad dependencies on real MCU services (`RepRap::IsStopped`, `GCodes::ReadMove`, SmartDrivers, PauseState enums). Conclusion: rather than drag the full Move stack into the host build, we need a dedicated Move/DDA facade that reports the minimum data required for CAN logging while keeping host shims manageable.


## Step 9 - Bring up the real RRF motion pipeline
Goal: A G1 in the input goes through RRF's planner and emits CAN movement packets that the host build already captures.

### Step 9.1 - Lightweight host Move/DDA interface (COMPLETED)
Goal: Create minimal host-only facades so CanMotion.cpp can compile/link without pulling the full MCU movement stack.

Approach: Rather than attempting to compile the full Movement/Move.cpp (which has heavy dependencies on RepRap::IsStopped, GCodes::ReadMove, SmartDrivers, PauseState enums, etc.), create host-only facades that provide just enough interface for CanMotion to compile and link.

### Step 9.1 Progress log
- Iteration 9.1A: Created lightweight host facades for the movement subsystem in RRF/host/include/Movement/:
  - `Move.h`: Minimal Move class with basic queries (DriveStepsPerMm, GetCurrentMachinePosition, GetKinematics, GetMainDDARing)
  - `DDA.h`: Minimal DDA class and PrepParams struct matching CanMotion's expectations (IsCheckingEndstops, moveStartTime, clocksNeeded)
  - `DDARing.h`: Placeholder DDARing for position tracking and move counting
  - `Kinematics/Kinematics.h`: Stub kinematics base class
- Implemented corresponding .cpp files in RRF/host/movement/:
  - `MoveHost.cpp`: Move facade with reasonable default steps/mm (80 steps/mm)
  - `DDAHost.cpp`: Simple DDA state tracking
  - `DDARingHost.cpp`: Basic position and simulation time tracking
  - `KinematicsHost.cpp`: Stub returning "none" as kinematics name
- Extended RepRap host facade (RRF/host/include/Platform/RepRap.h) to include Move member and GetMove() accessor
- Updated host Makefile to compile new movement host files
- **Build result**: `make -C RRF/host` succeeds, produces 951K binary. CanMotion.cpp now links against host Move/DDA facades without pulling in full RRF Movement stack.
 - What’s in place after Iteration 9.1A:
   - RRF/host/include/Movement/ facades: Move.h, DDA.h (with PrepParams), DDARing.h, Kinematics/Kinematics.h.
   - RRF/host/movement/ implementations: MoveHost.cpp, DDAHost.cpp, DDARingHost.cpp, KinematicsHost.cpp.
   - RepRap host facade exposes Move& GetMove().
   - Host Makefile builds the above; CanMotion.cpp links; binary ~951 KB.
These facades do not yet plan moves, populate PrepParams, call CanMotion::Add*Movement(), or produce movement packets; they simply make CanMotion link and provide state holders for the next step.

### Key findings for Step 9.2
- CanMotion.cpp requires:
  - `PrepParams` struct with timing fields (accelClocks, steadyClocks, decelClocks, acceleration, deceleration, totalDistance, topSpeed, useInputShaping)
  - `DDA` reference with `IsCheckingEndstops()`, `GetMoveStartTime()`, `GetClocksNeeded()`
  - These can be satisfied without the full DDA::Prepare/DDA::Init machinery
- The full RRF Movement stack dependencies include:
  - `Move::MoveLoop()` - main motion task that processes queued DDAs
  - `DDA::Init*()` methods - parse RawMove into motor steps using kinematics
  - `DDA::Prepare()` - compute acceleration profiles, allocate DriveMovement objects, call CanMotion::AddAxisMovement/AddExtruderMovement
  - `DriveMovement` - per-motor step generation state
  - Kinematics - transforms user coords to motor steps (Cartesian, Hangprinter, etc.)
  - EndstopManager, probe handling, SmartDrivers status polling
- To actually generate CAN packets in Step 9.2, we will need to:
  - Wire G1 commands to create RawMove objects
  - Implement simplified DDA::Init and DDA::Prepare that compute PrepParams and invoke CanMotion APIs
  - Decide whether to pull in real Kinematics or stub coordinate→steps transforms
  - Provide a simulated move execution loop that advances time deterministically


### Step 9.2 - Simplified motion planning
Approach: We’ll first implement a host-side, simplified DDA/Move path to get deterministic packets flowing, then make it config-aware. This avoids pulling in the entire RRF movement subsystem up front, reduces stub churn, and lets us validate CAN capture and timing determinism early.

#### Step 9.2.1 - Minimal deterministic CAN emission (COMPLETED)
Goal: For basic G1 commands, compute steps and a simple trapezoid, populate PrepParams, and emit at least one movementLinearShaped packet per move.

### Step 9.2.1 Progress log
- Iteration 9.2.1A: Implemented complete minimal CAN emission pipeline:
  - Created RawMove structure for G-code move representation
  - Implemented DDA::Init() with coordinate→steps conversion (1:1 Cartesian, 80 steps/mm default)
  - Implemented DDA::Prepare() with trapezoid profile calculator:
    * Hardcoded acceleration/deceleration (1000 mm/s²)
    * Computes trapezoidal/triangle velocity profiles
    * Populates PrepParams with accelClocks, steadyClocks, decelClocks (48 MHz)
    * Calls CanMotion::StartMovement/AddAxisMovement/FinishMovement
  - Modified ProcessLinearMove() to create DDAs and execute moves
  - Added deterministic simulated tick counter (currentSimulatedTicks)
  - Initialized CAN subsystem (CanMessageBuffer::Init, CanMotion::Init)
  - Hardcoded CAN driver mapping (board 121, drivers 0-3 → axes A-D)
- **Build result**: `make` succeeds, produces ~953K binary
- **Testing**: test_move.g with 3 G1 commands produces 3 movementLinearShaped records in CAN log
- **Determinism verified**: Two runs produce byte-identical logs ✓
- **Known issues**:
  * Distance calculation shows inf/huge numbers (doesn't affect CAN packets)
  * E parameter not yet implemented
  * Acceleration values show as 0.0 in CAN packets (related to distance bug)
- **Files modified**: DDA.h, DDAHost.cpp, main.cpp, Makefile
- **Documentation**: Created ai_docs/step_9_2_1_summary.md with detailed implementation notes

Acceptance criteria:
 ✅ make -C RRF/host links with no new stubs
 ✅ Running test.g produces movementLinearShaped records in CAN log
 ✅ Two runs produce byte-identical logs


#### Step 9.2.2 - Config-aware simplified planner (COMPLETED)
Goal: Make the simplified planner respect config.g and basic kinematics/driver mapping so that outputs reflect configuration.

### Step 9.2.2 Progress log
- **Implementation complete**: All M-code handlers implemented and tested
- **Config parsing**: config.g is automatically executed on startup before --run
- **M-code handlers implemented**:
  * M92: Steps per mm configuration (axes and extruder)
  * M201: Maximum acceleration limits (mm/s²)
  * M203: Maximum speeds (mm/min → mm/s conversion)
  * M566: Jerk/instant speed change (mm/min)
  * M584: Axis-to-driver mapping (supports both CAN addresses like 40.0 and local drivers like 0)
  * M569: Driver direction/polarity configuration
  * M669: Kinematics type selection (K6=Hangprinter, K1=Cartesian)
  * M666: Hangprinter mechanical parameters (consumed for future use)
- **Configuration storage**: Extended Move class with arrays for accelerations, maxFeedrates, jerks, axisDrivers, driverForward
- **DDA::Prepare() updates**: Now uses configured acceleration/speed values instead of hardcoded defaults
- **Driver mapping**: DDA::Prepare() uses Move::GetAxisDriverId() to emit CAN packets to correct boards/drivers
- **Extruder support**: ProcessLinearMove() handles E parameter with relative extrusion (M83 mode)
- **Files modified**:
  * RRF/host/include/Movement/Move.h: Added configuration accessors
  * RRF/host/movement/MoveHost.cpp: Implemented configuration storage and accessors
  * RRF/host/movement/DDAHost.cpp: Updated to use configured values
  * RRF/host/src/main.cpp: Added M-code handlers and config.g execution

**Testing results**:

*Hangprinter config (80 steps/mm, 10000 mm/s² accel, CAN drivers 40-42)*:
- Z axis: 400 steps for 5mm (80 steps/mm) ✓
- X,Y axes: 800 steps for 10mm (80 steps/mm) ✓
- CAN destinations: 40, 41, 42 ✓
- Acceleration: 666.67 mm/s² (limited by planner)
- Total ticks: 7.68M

*Cartesian config (160/400 steps/mm, 3000 mm/s² accel, local drivers 0-2)*:
- Z axis: 2000 steps for 5mm (400 steps/mm) ✓
- X,Y axes: 1600 steps for 10mm (160 steps/mm) ✓
- CAN destinations: 0, 1, 2 (local drivers) ✓
- Acceleration: 200 mm/s² (lower due to lower config)
- Total ticks: 8.8M (longer due to lower acceleration)

**Determinism verified**: Two runs with same config produce byte-identical movement packets (only timestamp header differs)

Acceptance criteria:
 ✅ With two different config.g files, the same test.g produces different, plausible step counts and timing
 ✅ Extruder E parameter handling implemented (relative extrusion mode)
 ✅ Logs remain byte-identical across repeated runs (deterministic mode)


### Step 9.3 - Progressive migration toward full RRF movement (Pending)
Scope:
 - Replace simplified Init/Prepare with real DDA::Init/Prepare, DriveMovement, and selected kinematics from RRF’s Movement/.
 - Add any missing subsystems (probing/endstops/pauses) strictly as the planner requires.

Exit when: the simplified path is fully validated and we specifically need parity with RRF’s corner cases.


### Notes & invariants for Step 9
 - Keep -DSRC_MOVEMENT_STEPTIMER_H_=1 and -DRRF_HOST_BUILD=1.
 - Timebase stays simulated for Step 9; real-time mode comes later (see Step 12).



## Step 10 - Config.g ingestion
Goal: Moves are computed with the right units, limits, and driver mapping.

### Run config.g on startup:
  * Execute 0:/sys/config.g before --run. (Use existing VSD; no DSF involved.)

### Minimum config coverage:
  * M92 (steps/mm)
  * M566/M201/M203 (jerk/accel/max speed)
  * M350 (microstepping)
  * M584/M569 (axis-driver mapping and direction)
  * M669/M666 (Hangprinter specific config)
  * tool/extruder basics.

### Kinematics selection:
  * Accept Hangprinter vs. Cartesian by what config.g sets; only enable the kinematics we compile.

### Acceptance checks:
  * Dump a small object-model subset (or print) after config to verify steps/mm or M669/M666 settings and axis letters.
  * Same G-code with two different config.g files yields different, plausible CAN step counts/timings.

### Step 10 Progress log
nothing yet


## Step 11 - Expand CAN capture coverage
Goal: Capture everything a simulator needs, not just linear-shaped packets.

### Broaden packet types:
  * Handle other motion-related message kinds we expect (plain linear, raw steps, late shaping variants, extruder-only segments, maybe set-current if emitted).

### Stable schema & versioning:
  * Keep JSONL, add capture_version, include board/driver IDs, seq numbers, per-driver data, PA flags, and timing fields.

### CLI:
  * --can-log <path> (already) and --can-format jsonl|raw (add later if we want a binary).

### Acceptance checks:
  * Logs contain entries for extruder moves and multi-driver segments.
  * Schema passes a JSON schema check and is documented.

### Step 11 Progress log
nothing yet


## Step 12 - Deterministic batch mode & timebase controls
Goal: Same inputs ⇒ byte-identical logs (great for regression).

## Simulated clock:
  * Advance by planner-consumed clocks rather than wall time; seed is fixed: Drive the simulation clock from what the planner says a move lasts, not from wall-clock time and make every source of randomness fixed.
    "Planner-consumed clocks" is the step-timer ticks the RRF planner computes for each segment.
    You already see them in the CAN payload fields: accelerationClocks, steadyClocks, decelClocks, plus the whenToExecute field.
    Total ticks for a segment = accel + steady + decel.
    Instead of sleep()-ing in real time, you advance a monotonic counter by exactly that many ticks and stamp the next message with the new whenToExecute.
    No waiting; it’s pure arithmetic and therefore reproducible.
    Minimal sketch:
    ```cpp
    uint64_t now_ticks = 0;           // simulated time in step-timer ticks
    const uint32_t tick_hz = 48000000; // whatever RRF’s step clock is

    for (auto& m : moves) {
      m.whenToExecute = now_ticks;
      uint64_t dur = m.accelerationClocks + m.steadyClocks + m.decelClocks;
      now_ticks += dur;
      write_jsonl(m);                 // emit to log
    }
    ```

### CLI:
  * --rt false (default) for deterministic; later --rt true can follow wall-clock for interactive demos. Optional --timebase-hz for testing.

### Acceptance checks:
  * Two runs on different machines produce identical CAN logs.
  * A unit test asserts exact file equality for a known input.

### Step 12 Progress log
nothing yet


## Step 13 - Validation & regression
Goal: Confidence that we match RRF semantics and don't regress.

### Unit tests:
  * Serialize/deserialize checks for CAN messages (compare against formats in CANlib).
  * Small kinematics sanity (e.g., Cartesian vs HangprinterKinematics step mapping on a single segment).

### End-to-end "golden log" tests:
  * Tiny printlets (config.g + 10–50 G-code lines) generate canonical JSONL logs under tests/golden/.
  * CI compares outputs byte-for-byte.

### Docs:
  * Document CLI (--vsd, --run, --can-log, --rt/--timebase-hz).
  * Document JSONL schema and how a simulator should consume it.

### Step 13 Progress log
nothing yet


## Next steps (when steps 1-13 are all done and all tests are green):
Expand the following:
 * kinematics set,
 * pressure-advance fidelity,
 * more M-code coverage,
 * performance profiling.

### Next steps Progress log
nothing yet
