I'm looking at ReprapFirmware. It's usually built for an embedded platform but I want to make a x86_64 build with:

No FreeRTOS

No reason to be synchronous, we can fake clock ticks and such. We will only run in "batch mode" on x86_64, so no reason to wait for anything or stay synchronous, as long as everything happens in the right order.

No connected boards, but we want to capture packets that would have been sent to external CAN boards, so we need to fake something there

No networking

No connections to other programs such as DuetWebInterface or DSW or anything like that.

No fans or heaters or anything like that. No physical pins or connections to anything at all actually.

These are my current "fake" header files:

$ tree include
include
├── AnalogIn.h
├── AnalogOut.h
├── AsyncSerial.h
├── Cache.h
├── CAN
│   └── CanCapture.h
├── Core.h
├── CoreIO.h
├── CoreNotifyIndices.h
├── CoreTypes.h
├── DmacManager.h
├── Endstops
│   ├── EndstopDefsHost.h
│   └── EndstopsManager.h
├── Fans
│   └── FansManager.h
├── FilamentMonitors
│   └── FilamentMonitor.h
├── FreeRTOS.h
├── GCodes
│   └── GCodesHost.h
├── General
│   └── FreelistManager.h
├── Heating
│   └── Heat.h
├── Interrupts.h
├── Movement
│   ├── DDAHost.h
│   ├── DDAHost_previouslyDDA.h
│   ├── DDARing.h
│   ├── HostPlanner.h
│   ├── Kinematics
│   │   ├── CartesianKinematicsHost.h
│   │   └── Kinematics.h.old
│   ├── MoveHost.h
│   ├── StepperDrivers
│   │   └── SmartDrivers.h
│   ├── StepTimer.h
│   └── StepTimerHost.h
├── Networking
│   └── Network.h
├── PinDescription.h
├── Platform
│   ├── ArrayHandle.h
│   ├── Heap.h
│   ├── OutputMemory.h
│   ├── Platform.h
│   ├── RepRap.h
│   ├── StringHandle.h
│   └── TaskPriorities.h
├── pmc
│   └── pmc.h
├── PrintMonitor
│   └── PrintMonitor.h
├── queue.h
├── RTOSIface
│   └── RTOSIface.h
├── semphr.h
├── SerialCDC.h
├── Storage
│   ├── FileStore.h
│   ├── FileWriteBuffer.h
│   └── MassStorage.h
├── Stream.h
├── task.h
├── Tools
│   ├── Filament.h
│   └── Tool.h
├── UniqueIdBase.h
├── USARTClass.h
└── Wire.h

I just started to try and include the real GCodes.h, DDA.h, Move.h, and EndstopDefs.h.

Here's the most relevant part of ReprapFirmware's include directory tree:

torbjorn[~/repos/hp-sim5/RRF/ReprapFirmware/src]$ tree
.
├── Accelerometers
│   ├── Accelerometers.cpp
│   ├── Accelerometers.h
│   ├── LISAccelerometer.cpp
│   └── LISAccelerometer.h
├── AppNotifyIndices.h
├── CAN
│   ├── CanDriversData.cpp
│   ├── CanDriversData.h
│   ├── CanInterface.cpp
│   ├── CanInterface.h
│   ├── CanMessageGenericConstructor.cpp
│   ├── CanMessageGenericConstructor.h
│   ├── CanMotion.cpp
│   ├── CanMotion.h
│   ├── CommandProcessor.cpp
│   ├── CommandProcessor.h
│   ├── ExpansionManager.cpp
│   └── ExpansionManager.h
├── ClosedLoop
│   ├── ClosedLoop.cpp
│   └── ClosedLoop.h
├── Comms
│   ├── AuxDevice.cpp
│   ├── AuxDevice.h
│   ├── FirmwareUpdater.cpp
│   ├── FirmwareUpdater.h
│   ├── Modbus.h
│   ├── PanelDueUpdater.cpp
│   └── PanelDueUpdater.h
├── Config
│   ├── Configuration.h
│   ├── Features_Host.h
│   ├── Pins.cpp
│   ├── Pins_Duet3_MB6HC.h
│   ├── Pins_Duet3_MB6XD.h
│   ├── Pins_Duet3Mini.h
│   ├── Pins_DuetNG.h
│   ├── Pins_FMDC.h
│   ├── Pins.h
│   ├── Pins_Host_MB6HC.h
│   └── Pins_Pccb.h
├── Endstops
│   ├── Endstop.cpp
│   ├── EndstopDefs.h
│   ├── Endstop.h
│   ├── EndstopsManager.cpp
│   ├── EndstopsManager.h
│   ├── LocalZProbe.cpp
│   ├── LocalZProbe.h
│   ├── RemoteZProbe.cpp
│   ├── RemoteZProbe.h
│   ├── StallDetectionEndstop.cpp
│   ├── StallDetectionEndstop.h
│   ├── SwitchEndstop.cpp
│   ├── SwitchEndstop.h
│   ├── ZProbe.cpp
│   ├── ZProbeEndstop.cpp
│   ├── ZProbeEndstop.h
│   └── ZProbe.h
├── Fans
│   ├── Fan.cpp
│   ├── Fan.h
│   ├── FansManager.cpp
│   ├── FansManager.h
│   ├── LocalFan.cpp
│   ├── LocalFan.h
│   ├── RemoteFan.cpp
│   └── RemoteFan.h
├── FilamentMonitors
│   ├── Duet3DFilamentMonitor.cpp
│   ├── Duet3DFilamentMonitor.h
│   ├── FilamentMonitor.cpp
│   ├── FilamentMonitor.h
│   ├── LaserFilamentMonitor.cpp
│   ├── LaserFilamentMonitor.h
│   ├── PulsedFilamentMonitor.cpp
│   ├── PulsedFilamentMonitor.h
│   ├── RotatingMagnetFilamentMonitor.cpp
│   ├── RotatingMagnetFilamentMonitor.h
│   ├── SimpleFilamentMonitor.cpp
│   └── SimpleFilamentMonitor.h
├── GCodes
│   ├── CollisionAvoider.cpp
│   ├── CollisionAvoider.h
│   ├── GCodeBuffer
│   │   ├── BinaryParser.cpp
│   │   ├── BinaryParser.h
│   │   ├── ExpressionParser.cpp
│   │   ├── ExpressionParser.h
│   │   ├── GCodeBuffer.cpp
│   │   ├── GCodeBuffer.h
│   │   ├── StringParser.cpp
│   │   └── StringParser.h
│   ├── GCodeChannel.h
│   ├── GCodeException.cpp
│   ├── GCodeException.h
│   ├── GCodeFileInfo.cpp
│   ├── GCodeFileInfo.h
│   ├── GCodeInput.cpp
│   ├── GCodeInput.h
│   ├── GCodeMachineState.cpp
│   ├── GCodeMachineState.h
│   ├── GCodeQueue.cpp
│   ├── GCodeQueue.h
│   ├── GCodes2.cpp
│   ├── GCodes3.cpp
│   ├── GCodes4.cpp
│   ├── GCodes5.cpp
│   ├── GCodes6.cpp
│   ├── GCodes7.cpp
│   ├── GCodes.cpp
│   ├── GCodes.h
│   ├── KeepoutZone.cpp
│   ├── KeepoutZone.h
│   ├── ObjectTracker.cpp
│   ├── ObjectTracker.h
│   ├── RestorePoint.cpp
│   ├── RestorePoint.h
│   ├── SimulationMode.h
│   ├── StraightProbeSettings.cpp
│   ├── StraightProbeSettings.h
│   ├── TriggerItem.cpp
│   └── TriggerItem.h
├── Heating
│   ├── FOPDT.cpp
│   ├── FOPDT.h
│   ├── Heat.cpp
│   ├── Heater.cpp
│   ├── Heater.h
│   ├── HeaterMonitor.cpp
│   ├── HeaterMonitor.h
│   ├── Heat.h
│   ├── LocalHeater.cpp
│   ├── LocalHeater.h
│   ├── RemoteHeater.cpp
│   ├── RemoteHeater.h
├── Movement
│   ├── Axis and drive allocation and release.md
│   ├── AxisShaper.cpp
│   ├── AxisShaper.h
│   ├── BedProbing
│   │   ├── Grid.cpp
│   │   ├── Grid.h
│   │   ├── RandomProbePointSet.cpp
│   │   └── RandomProbePointSet.h
│   ├── Coordinate and endpoint management.ods
│   ├── DDA_3rdOrder.cpp
│   ├── DDA.cpp
│   ├── DDA.h
│   ├── DDARing.cpp
│   ├── DDARing.h
│   ├── DriveMovement.cpp
│   ├── DriveMovement.h
│   ├── ExtruderShaper.cpp
│   ├── ExtruderShaper.h
│   ├── HeightControl
│   │   ├── HeightController.cpp
│   │   └── HeightController.h
│   ├── HomingMode.h
│   ├── Kinematics
│   │   ├── CoreKinematics.cpp
│   │   ├── CoreKinematics.h
│   │   ├── FiveBarScaraKinematics.cpp
│   │   ├── FiveBarScaraKinematics.h
│   │   ├── HangprinterKinematics.cpp
│   │   ├── HangprinterKinematics.h
│   │   ├── Kinematics.cpp
│   │   ├── Kinematics.h
│   │   ├── LinearDeltaKinematics.cpp
│   │   ├── LinearDeltaKinematics.h
│   │   ├── PolarKinematics.cpp
│   │   ├── PolarKinematics.h
│   │   ├── RotaryDeltaKinematics.cpp
│   │   ├── RotaryDeltaKinematics.h
│   │   ├── RoundBedKinematics.cpp
│   │   ├── RoundBedKinematics.h
│   │   ├── ScaraKinematics.cpp
│   │   ├── ScaraKinematics.h
│   │   ├── ZLeadscrewKinematics.cpp
│   │   └── ZLeadscrewKinematics.h
│   ├── Move2.cpp
│   ├── Move3.cpp
│   ├── Move.cpp
│   ├── MoveDebugFlags.h
│   ├── Move.h
│   ├── MovementError.cpp
│   ├── MovementError.h
│   ├── MovementProfile.cpp
│   ├── MovementProfile.h
│   ├── MoveSegment.cpp
│   ├── MoveSegment.h
│   ├── MoveTiming.h
│   ├── PhaseStep.cpp
│   ├── PhaseStep.h
│   ├── RawMove.cpp
│   ├── RawMove.h
│   ├── StepperDrivers
│   │   ├── DriverData.cpp
│   │   ├── DriverData.h
│   │   ├── DriverMode.cpp
│   │   ├── DriverMode.h
│   │   ├── SmartDrivers.h
│   │   ├── TMC22xx.cpp
│   │   ├── TMC22xx.h
│   │   ├── TMC2660.cpp
│   │   ├── TMC2660.h
│   │   ├── TMC51xx.cpp
│   │   └── TMC51xx.h
│   ├── StepTimer.cpp
│   ├── StepTimer.h
│   └── Trigonometry.h
├── ObjectModel
│   ├── GlobalVariables.cpp
│   ├── GlobalVariables.h
│   ├── ObjectModel.cpp
│   ├── ObjectModel.h
│   ├── TypeCode.h
│   ├── Variable.cpp
│   └── Variable.h
├── Platform
│   ├── ArrayHandle.cpp
│   ├── ArrayHandle.h
│   ├── AveragingFilter.h
│   ├── Event.cpp
│   ├── Event.h
│   ├── Heap.cpp
│   ├── Heap.h
│   ├── Logger.cpp
│   ├── Logger.h
│   ├── MessageBox.cpp
│   ├── MessageBox.h
│   ├── MessageType.h
│   ├── OutputMemory.cpp
│   ├── OutputMemory.h
│   ├── Platform.cpp
│   ├── Platform.h
│   ├── PortControl.cpp
│   ├── PortControl.h
│   ├── PrintPausedReason.h
│   ├── RepRap.cpp
│   ├── RepRap.h
│   ├── StringHandle.cpp
│   ├── StringHandle.h
│   ├── TaskPriorities.h
│   ├── Tasks.cpp
│   ├── Tasks.h
│   ├── UniqueId.cpp
│   └── UniqueId.h
├── PrintMonitor
│   ├── PrintMonitor.cpp
│   └── PrintMonitor.h
├── RepRapFirmware.cpp
├── RepRapFirmware.h
├── RepRapFirmware.pdp
├── RTOS work pending.txt
├── Tools
│   ├── Filament.cpp
│   ├── Filament.h
│   ├── Spindle.cpp
│   ├── Spindle.h
│   ├── Tool.cpp
│   └── Tool.h
├── Version.cpp
└── Version.h

So I mostly just need a list over which real ReprapFirmware headers to definitely shadow.

If you want to check some details in the ReprapFirmware codebase, please refer to https://github.com/tobbelobb/ReprapFirmware/tree/6e292a37fec641a9d2de68bde3cdccb7ee407788

-----

Of course. Based on your goal to create a "batch mode" x86_64 simulation of ReprapFirmware, your approach of creating fake headers to "shadow" the real hardware and RTOS-dependent parts is the correct one. You want to isolate the core G-code processing and motion planning logic.

Here is a categorized list of the headers and modules you should definitely focus on shadowing.

1. RTOS and Concurrency

This is the most fundamental layer to replace. Your simulation will be single-threaded and sequential.

FreeRTOS.h, task.h, queue.h, semphr.h: You've already correctly identified these. All operating system primitives for task scheduling, mutexes, semaphores, and queues must be stubbed out.

RTOSIface/RTOSIface.h: This is ReprapFirmware's own wrapper around the RTOS. Faking this is crucial. A fake implementation might have empty mutex lock/unlock functions.

Platform/Tasks.h: This manages the creation of the various RRF tasks. Your main simulation loop will replace this entirely by calling the relevant functions in the correct order.

2. Core Hardware Abstraction

This layer interacts directly with the microcontroller's peripherals.

Platform/Platform.h: This is one of the most important files to fake. It handles board-level initialization, timekeeping (millis, micros), interrupt control, and emergency shutdowns. Your PlatformHost.h should provide fake clocks (e.g., a simple incrementing counter for ticks) and stub out any hardware-specific functions.

Core.h, CoreIO.h: These define low-level pin and peripheral access, which you don't have.

DmacManager.h: Manages Direct Memory Access. Not relevant for an x86 build.

Interrupts.h: All interrupt handling needs to be removed. Your code will run sequentially.

Config/Pins_*.h: These files contain the hardware pin mappings for specific Duet boards. You can replace them with a single fake header that provides dummy pin identifiers if needed for compilation.

3. Physical I/O and On-board Devices

Anything that would touch a wire, sensor, or driver chip needs to be faked.

Movement/StepperDrivers/SmartDrivers.h and the specific driver files (TMC2660.h, TMC51xx.h, etc.): The motion planner (DDA.h) will calculate steps, but the code that sends commands (SPI/UART) to the stepper driver ICs must be stubbed.

Movement/StepTimer.h: This is critical. It uses hardware timers to generate the precise step pulse timing. Your StepTimerHost.h will need to simulate this, likely by just immediately processing all steps in a "batch" without any real-time delay.

Heating/Heat.h: Manages heaters and temperature sensors. Your fake implementation can return fixed values or the "active/standby" temperature when queried, and do nothing when commanded to change heater power.

Fans/FansManager.h: Manages all fans. Fake implementations should simply discard any commands to set fan speeds.

Storage/MassStorage.h: This handles the SD card. You may want a fake implementation that redirects file I/O to the host's native filesystem for loading G-code files.

4. Communication

As you noted, all external communication channels should be disabled.

CAN/CanInterface.h: This is the key to your goal of capturing CAN packets. Your fake CanCapture.h should implement the functions defined in the real CanInterface.h (like SendMessage). Instead of writing to hardware, your implementation will write the packet contents to a log file or an in-memory list for later inspection.

Networking/Network.h: Manages the entire TCP/IP stack. It should be completely stubbed out.

SerialCDC.h, AsyncSerial.h, USARTClass.h: These handle USB and other serial communications, which are not needed.

Summary of Strategy

Your current approach is solid. The general strategy is to start with the core logic you want to keep (GCodes.h, Move.h, DDA.h, Kinematics.h) and let the compiler tell you what's missing. When you hit a compiler error because a function or class from a hardware-dependent module is used, you create a fake version of it in your own include path.

Based on the list above, the most important "shadow" headers to get right will be:

Platform.h: To provide the basic environment (clocks, etc.).

StepTimer.h: To consume the output of the motion planner.

CanInterface.h: To capture the CAN traffic as you require.

Heat.h and FansManager.h: To satisfy the dependencies from GCodes that manage tools and temperatures.


----

Please make three lists categorizing the real ReprapFirmware header files:
1: Header files I need to fake (and comment if I already did so or not)
2: Header files I need to use the real version of
3: Header files that exist in the ReprapFirmware repo but will not be relevant because my fake versions from list 1 will never include them.

----

Excellent. This is a classic "hardware-in-the-loop" simulation problem, and your approach is spot on. By creating a well-defined boundary between the core firmware logic and the hardware abstraction layer, you can achieve your goal.

Here are the three lists categorizing the ReprapFirmware headers based on your requirements.

1. Header Files to Fake/Shadow

These are the interfaces between the core logic and the hardware/RTOS. Your fake headers must provide the same class and function signatures as the real ones to satisfy the compiler, but with simplified "batch mode" implementations.

RTOS & Scheduling:

FreeRTOS.h, task.h, queue.h, semphr.h - (You have already faked these)

RTOSIface/RTOSIface.h - The RRF wrapper for RTOS calls. (You have this)

Core Platform & Hardware Abstraction:

Platform/Platform.h - The most critical one. Handles time, pins, and board-specific functions. (You have this)

Platform/Heap.h - The memory manager. (You have this)

Platform/Tasks.h - Task creation and management. (You'll replace this with your own sequential main loop)

Config/Pins_*.h - All of them. You don't have any hardware pins. You can use one fake Pins.h to satisfy includes.

Interrupts.h - Not relevant in a single-threaded simulation. (You have this)

Movement & Hardware:

Movement/StepTimer.h - This is crucial. The real version uses hardware timers to generate step pulses. Your fake version will likely just confirm that steps have been "executed" instantly. (You have StepTimerHost.h)

Movement/StepperDrivers/SmartDrivers.h - The interface for configuring and commanding the stepper driver ICs. (You have this)

Physical I/O:

Endstops/EndstopsManager.h - The top-level manager for endstops and probes. (You have this)

Heating/Heat.h - The top-level manager for all heaters and sensors. (You have this)

Fans/FansManager.h - The top-level manager for fans. (You have this)

Accelerometers/Accelerometers.h - Interface for hardware accelerometers. (You will need to fake this)

Storage/MassStorage.h - For SD card access. You may want your fake to redirect to the host filesystem to read G-code. (You have this)

Communication & Networking:

CAN/CanInterface.h - The low-level CAN hardware interface. Your fake implementation will be the capture utility you wanted.

Networking/Network.h - The entire TCP/IP stack interface. (You have this)

All headers related to serial/USB communication, such as SerialCDC.h and Comms/AuxDevice.h.

2. Header Files to Use (The Real Version)

This is the core, hardware-agnostic logic of the firmware that you want to test and run. These files should be included directly from the real ReprapFirmware source tree.

G-Code Processing:

GCodes/GCodes.h

GCodes/GCodeBuffer/GCodeBuffer.h

GCodes/GCodeMachineState.h

GCodes/GCodeQueue.h

GCodes/GCodeInput.h

And all the GCodes*.cpp implementation files.

Motion Planning:

Movement/Move.h - The high-level motion orchestrator.

Movement/DDA.h - The Digital Differential Analyzer, which generates the steps.

Movement/DDARing.h - The ring buffer for motion commands.

Movement/DriveMovement.h

Kinematics:

Movement/Kinematics/Kinematics.h - The base class for kinematics.

The specific kinematics implementation you need (e.g., CoreKinematics.h, HangprinterKinematics.h, etc.).

Data Model & State:

ObjectModel/ObjectModel.h - The data model that allows different modules to interact.

Platform/RepRap.h - The central class that ties everything together.

Tools/Tool.h and Tools/Filament.h - The high-level representation of tools.

PrintMonitor/PrintMonitor.h - High-level print state tracking.

Config/Configuration.h - Defines firmware features and limits.

CAN Application Logic:

CAN/ExpansionManager.h

CAN/CommandProcessor.h

These modules use the CanInterface.h you are faking. By including the real versions, you ensure that the logic generating the CAN messages is the original, and your fake interface will correctly capture the intended traffic.

3. Irrelevant Header Files (Will Not Be Used)

These are implementation details for hardware that you will never include because you are faking the manager/interface that uses them. You do not need to fake these or include them.

Specific Hardware Implementations:

Movement/StepperDrivers/TMC2660.h, TMC51xx.h, etc. (Your fake SmartDrivers.h won't include them).

Endstops/SwitchEndstop.h, ZProbe.h, LocalZProbe.h (Your fake EndstopsManager.h won't need specific types).

Heating/LocalHeater.h, RemoteHeater.h (Your fake Heat.h won't use them).

Fans/LocalFan.h, RemoteFan.h (Your fake FansManager.h won't use them).

FilamentMonitors/LaserFilamentMonitor.h, PulsedFilamentMonitor.h, etc. (Your fake FilamentMonitor.h won't use them).

Accelerometers/LISAccelerometer.h.

Low-Level Communication Details:

Comms/PanelDueUpdater.h

Comms/FirmwareUpdater.h

Unused Board Configurations:

Any Pins_*.h file for a board you are not targeting can be ignored completely.

----

Ok that's a hefty amount of real header files. I guess I should start by compiling the lowest level .cpp files that I can. In general each header file has a corresponding .cpp file. For list 2, the header files to use the real version of, please walk through a list of .cpp files to compile, starting with the lowest level ones and working our way up.

----

Of course. This is the correct and methodical way to approach the problem. By starting with the lowest-level dependencies (the "leaf nodes" in the dependency tree) and moving up, you can solve compilation and linking errors one layer at a time.

Here is a logical walk-through of the .cpp files to compile from your "List 2", ordered from lowest to highest dependency.

Tier 1: Foundational Utilities and Data Structures

These files have minimal dependencies on other parts of the ReprapFirmware logic. They provide basic data structures and services that many other modules will use.

Platform/ArrayHandle.cpp & Platform/StringHandle.cpp: These are custom handle-based array and string classes. They are fundamental utilities used throughout the firmware.

Platform/OutputMemory.cpp: Manages buffers for responses. It's a self-contained utility.

ObjectModel/Variable.cpp & ObjectModel/ObjectModel.cpp: The object model is a core service that allows different modules to query the machine's state in a structured way. It needs to exist before the modules that use it (like GCodes and Move).

ObjectModel/GlobalVariables.cpp: Manages global variables accessible through the object model.

At this point, you have the basic building blocks for memory and state management.

Tier 2: Core Machine Components (Stateful but not Active)

These modules represent core components of the printer. They have internal logic but are generally acted upon by the higher-level G-code and motion planners.

Tools/Filament.cpp & Tools/Tool.cpp: These define the Tool class, which represents an extruder or other tool head. They depend on your faked Heat.h but are otherwise self-contained logical units.

Movement/Kinematics/Kinematics.cpp: The base class for all kinematics.

Movement/Kinematics/CoreKinematics.cpp (or your specific choice like HangprinterKinematics.cpp): This is a pure mathematical model for converting Cartesian coordinates to motor steps. It has almost no dependencies on other active parts of the firmware, making it a perfect candidate for early compilation.

Now you have the data structures for tools and the mathematical model for your machine's geometry.

Tier 3: The Motion Planning Engine (Bottom-Up)

This is the most complex subsystem. We'll compile it from the inside out, starting with the step generation and working up to the planner.

Movement/DriveMovement.cpp: Represents the state and parameters of a single axis or extruder drive. It's a dependency for DDA.

Movement/DDA.cpp: The Digital Differential Analyzer. This is the heart of the motion system. It takes a target position and speed and calculates the precise step timing for each motor. It depends on Kinematics and DriveMovement.

Movement/DDARing.cpp: The ring buffer that holds and manages the queue of DDA motion blocks. It depends on DDA.h.

Movement/Move.cpp: This is the high-level motion planner. It receives commands (e.g., "move to X, Y at speed F") from the G-code interpreter, generates the appropriate DDA instances, and enqueues them in the DDARing. This is the top of the motion stack.

You now have a complete, commandable motion system, ready to be driven by G-codes.

Tier 4: G-Code Interpretation and High-Level Logic

These modules parse and execute G-code commands, using the motion engine and other components you've just compiled.

GCodes/GCodeBuffer/GCodeBuffer.cpp, ExpressionParser.cpp, StringParser.cpp: These handle the low-level task of parsing G-code strings into a structured, binary format that the interpreter can use.

GCodes/GCodeMachineState.cpp: This important class tracks the modal state of the machine (e.g., current feedrate, relative/absolute mode, coordinate system offsets, etc.).

GCodes/GCodeInput.cpp & GCodes/GCodeQueue.cpp: These manage the queues for incoming G-code commands from different sources (which in your case will just be your main simulation loop).

GCodes/GCodes.cpp (and GCodes2.cpp, GCodes3.cpp, etc.): This is the main G-code interpreter. It's a giant dispatcher that reads a parsed G-code from the buffer and calls the appropriate functions in Move.h, your faked Heat.h, your faked FansManager.h, etc. It's one of the highest-level pieces of logic.

Tier 5: Top-Level Application Objects

These are the final pieces that tie everything together into a cohesive application.

PrintMonitor/PrintMonitor.cpp: A high-level class that monitors the status of a print job.

Platform/RepRap.cpp: This is the "god object". It instantiates and holds pointers to all the major components (GCodes, Move, Platform, etc.) and manages the overall machine state. This should be one of the very last firmware files you compile.

Your own main.cpp: This will be your entry point. It will:

Instantiate your fake platform and hardware classes.

Instantiate the RepRap object.

Call the initialization sequences.

Contain the main loop that reads G-code from a file and feeds it to the GCodes module.

By following this tiered approach, the linker will be much happier, as you'll be compiling A.cpp before you try to link B.cpp which calls functions in A. Good luck
