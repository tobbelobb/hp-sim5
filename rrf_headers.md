1: Header files I need to fake (and comment if I already did so or not)
2: Header files I need to use the real version of
3: Header files that exist in the ReprapFirmware repo but will not be relevant because my fake versions from list 1 will never include them.

# 1: Header Files to Fake/Shadow

### RTOS & Scheduling:
FreeRTOS.h
task.h
queue.h
semphr.h
RTOSIface/RTOSIface.h

### Core Platform & Hardware Abstraction:
Platform/Platform.h
Platform/Heap.h
Platform/Tasks.h - Task creation and management. (You'll replace this with your own sequential main loop)
Config/Pins_\*.h - All of them. You don't have any hardware pins. You can use one fake Pins.h to satisfy includes.
Interrupts.h - Not relevant in a single-threaded simulation. (You have this)

### Movement & Hardware:
Movement/StepTimer.h - Your fake should confirm that steps have been "executed" instantly. (You have StepTimerHost.h)
Movement/StepperDrivers/SmartDrivers.h - The interface for configuring and commanding the stepper driver ICs. (You have this)

### Physical I/O:
Endstops/EndstopsManager.h
Heating/Heat.h
Fans/FansManager.h
Accelerometers/Accelerometers.h - TODO
Storage/MassStorage.h

### Communication & Networking:
CAN/CanInterface.h - Low-level CAN hardware interface. Your fake implementation will be the capture utility you wanted.
Networking/Network.h

### serial/USB communication:
All of them
 SerialCDC.h
 Comms/AuxDevice.h
 Which others? TODO?

# 2: Header Files to Use (The Real Version)

### G-Code Processing:
GCodes/GCodes.h
GCodes/GCodeBuffer/GCodeBuffer.h
GCodes/GCodeMachineState.h
GCodes/GCodeQueue.h
GCodes/GCodeInput.h

### Motion Planning:
Movement/Move.h
Movement/DDA.h
Movement/DDARing.h
Movement/DriveMovement.h

### Kinematics:
Movement/Kinematics/Kinematics.h
CoreKinematics.h
HangprinterKinematics.h
Other specific kinematics implementations if applicable

### Data Model & State:
ObjectModel/ObjectModel.h
Platform/RepRap.h
Tools/Tool.h
Tools/Filament.h
PrintMonitor/PrintMonitor.h
Config/Configuration.h

### CAN Application Logic:
CAN/ExpansionManager.h
CAN/CommandProcessor.h

# 3: Irrelevant Header Files (Will Not Be Used)

### Specific Hardware Implementations:
Movement/StepperDrivers/TMC2660.h, TMC51xx.h, etc. (Your fake SmartDrivers.h won't include them).
Endstops/SwitchEndstop.h, ZProbe.h, LocalZProbe.h (Your fake EndstopsManager.h won't need specific types).
Heating/LocalHeater.h, RemoteHeater.h (Your fake Heat.h won't use them).
Fans/LocalFan.h, RemoteFan.h (Your fake FansManager.h won't use them).
FilamentMonitors/LaserFilamentMonitor.h, PulsedFilamentMonitor.h, etc. (Your fake FilamentMonitor.h won't use them).
Accelerometers/LISAccelerometer.h.

### Low-Level Communication Details:
Comms/PanelDueUpdater.h
Comms/FirmwareUpdater.h

### Unused Board Configurations:
Any Pins_*.h file for a board you are not targeting can be ignored completely.
