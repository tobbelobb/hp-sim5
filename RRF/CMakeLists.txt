cmake_minimum_required(VERSION 3.16)
project(RepRapFirmwareSimulator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

# --- Project Paths (Relative to this CMakeLists.txt) ---
set(SIM_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/host)
set(RRF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ReprapFirmware)
set(RRFLibraries_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/RRFLibraries)
set(CANlib_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CANlib)

# --- Define the Sandbox Directory ---
# All sources will be copied here before compilation.
set(GENERATED_SOURCES_DIR ${CMAKE_BINARY_DIR}/generated_sources)

# --- Define the Executable Target ---
add_executable(rrf_simulator)

# --- Preprocessor Definitions ---
target_compile_definitions(rrf_simulator PRIVATE
  #DUET3_MB6HC=1
    DUET3=1
  #__SAME70Q20B__
    SAME70=1 # Pretend we're this processor, affects array sizes and stuff
    RRF_HOST_BUILD=1
    USE_SBC=0
    SRC_MOVEMENT_STEPTIMER_H_=1
)

# --- List Your Custom Source Files ---
target_sources(rrf_simulator PRIVATE
    ${SIM_ROOT_DIR}/src/main.cpp
    ${SIM_ROOT_DIR}/platform/Heap.cpp
    ${SIM_ROOT_DIR}/platform/PlatformHost.cpp
    ${SIM_ROOT_DIR}/can/CanCapture.cpp
    ${SIM_ROOT_DIR}/can/CanInterfaceHost.cpp
    ${SIM_ROOT_DIR}/storage/MassStorageHost.cpp
    ${SIM_ROOT_DIR}/storage/FileStoreHost.cpp
    ${SIM_ROOT_DIR}/src/devices_stub.cpp
    ${SIM_ROOT_DIR}/networking/MacAddressHost.cpp
    ${SIM_ROOT_DIR}/movement/StepTimer.cpp
    ${SIM_ROOT_DIR}/gcodes/GCodeFileInfoHost.cpp # A bit unsure about this
    ${SIM_ROOT_DIR}/gcodes/FileGCodeInputHost.cpp # A bit unsure about this
    ${SIM_ROOT_DIR}/platform/SoftwareReset.cpp
    ${SIM_ROOT_DIR}/platform/TasksHost.cpp
    ${SIM_ROOT_DIR}/platform/GetFloatFormatStringHost.cpp
    ${SIM_ROOT_DIR}/platform/DebugPrintHost.cpp
    ${SIM_ROOT_DIR}/rtos/freertos_shim.cpp
    ${SIM_ROOT_DIR}/object_model/ObjectExplorationContextHost.cpp
)


# --- List the real ReprapFirmware Source Files ---
# This is the list of .cpp files we want to compile the real version of.
set(RRF_SOURCES_TO_USE
    # Tier 1: Foundational Utilities
    Platform/ArrayHandle.cpp
    Platform/StringHandle.cpp
    Platform/OutputMemory.cpp
    ObjectModel/Variable.cpp
    ObjectModel/ObjectModel.cpp
    ObjectModel/GlobalVariables.cpp

    # Tier 2: Core Machine Components (Stateful but not Active)
    Tools/Filament.cpp
    Tools/Tool.cpp
    Movement/Kinematics/Kinematics.cpp
    Movement/Kinematics/CoreKinematics.cpp
    Movement/Kinematics/HangprinterKinematics.cpp

    # Tier 3: The Motion Planning Engine (Bottom-Up)
    Movement/DriveMovement.cpp
    Movement/DDA.cpp
    Movement/DDARing.cpp
    Movement/Move.cpp

    # Tier 4: G-Code Interpretation and High-Level Logic
    GCodes/GCodeBuffer/GCodeBuffer.cpp
    GCodes/GCodeBuffer/ExpressionParser.cpp
    #GCodes/GCodeBuffer/StringParser.cpp
    #GCodes/GCodeMachineState.cpp
    #GCodes/GCodeInput.cpp
    #GCodes/GCodeQueue.cpp
    #GCodes/GCodes.cpp
    #GCodes/GCodes2.cpp
    #GCodes/GCodes3.cpp

    # Tier 5: Top-Level Application Objects
    #PrintMonitor/PrintMonitor.cpp
    #Platform/RepRap.cpp
)

# NEW LIST DEFINITION
# List of REAL .h files needed by the .cpp files above.
# If compilation fails with "No such file or directory", add the missing header here.
set(RRF_HEADERS_TO_USE
    # Tier 1 Dependencies
    Platform/ArrayHandle.h
    Platform/StringHandle.h
    Platform/OutputMemory.h
    ObjectModel/Variable.h
    ObjectModel/ObjectModel.h
    ObjectModel/GlobalVariables.h
    ObjectModel/TypeCode.h
    Platform/MessageType.h
    Platform/PrintPausedReason.h
    # Tier 2 Dependencies
    Tools/Filament.h
    Tools/Tool.h
    Movement/Kinematics/Kinematics.h
    Movement/Kinematics/CoreKinematics.h
    Movement/Kinematics/HangprinterKinematics.h
    # Tier 3 Dependencies
    Movement/DriveMovement.h
    Movement/DDA.h
    Movement/DDARing.h
    Movement/Move.h
    Movement/RawMove.h
    # Other common headers
    RepRapFirmware.h
    Platform/RepRap.h
    GCodes/GCodeMachineState.h
    GCodes/GCodeException.h
    GCodes/ObjectTracker.h
    GCodes/RestorePoint.h
    GCodes/GCodeChannel.h
    GCodes/GCodeInput.h
    GCodes/CollisionAvoider.h
    GCodes/KeepoutZone.h
    GCodes/TriggerItem.h
    GCodes/StraightProbeSettings.h
    GCodes/SimulationMode.h
    GCodes/GCodeBuffer/GCodeBuffer.h
    GCodes/GCodeBuffer/BinaryParser.h
    GCodes/GCodeBuffer/StringParser.h
    GCodes/GCodeBuffer/ExpressionParser.h
    GCodes/GCodeFileInfo.h
    Config/Configuration.h
    Networking/NetworkDefs.h
    Storage/FileData.h
    Storage/CRC32.h
    Storage/CRC16.h
    Endstops/EndstopDefs.h
    Endstops/ZProbe.h
    Endstops/Endstop.h
    Platform/MessageBox.h
    CAN/ExpansionManager.h
    Platform/UniqueId.h
    Platform/Event.h
    Movement/StepperDrivers/DriverData.h
    Movement/BedProbing/Grid.h
    Movement/HomingMode.h
    Movement/MovementError.h
    Movement/MoveTiming.h
    Movement/AxisShaper.h
    Movement/ExtruderShaper.h
    Movement/MoveSegment.h
    Movement/MovementProfile.h
    Movement/BedProbing/RandomProbePointSet.h
    Movement/MoveDebugFlags.h
    Hardware/IoPorts.h
    Hardware/ExceptionHandlers.h
    Hardware/SoftwareReset.h
    AppNotifyIndices.h
    CAN/CanMotion.h
    CAN/CanDriversData.h
    Movement/Kinematics/ZLeadscrewKinematics.h
    Movement/Kinematics/LinearDeltaKinematics.h
    Movement/Kinematics/RoundBedKinematics.h
    Movement/Kinematics/RotaryDeltaKinematics.h
    Movement/Kinematics/ScaraKinematics.h
    Movement/Kinematics/PolarKinematics.h
    Movement/Kinematics/FiveBarScaraKinematics.h
    Platform/PortControl.h
    PrintMonitor/PrintMonitor.h
)


# List of FAKE headers from your host/include directory that will OVERWRITE the real ones.
# The path here is relative to host/include.
set(SHADOWING_HEADERS_TO_USE
    # Shadowing headers
    FreeRTOS.h
    task.h
    queue.h
    semphr.h
    RTOSIface/RTOSIface.h
    Platform/Platform.h
    Platform/Heap.h
    Platform/Tasks.h
    Config/Pins.h
    Config/Pins_Host.h
    PinDescription.h
    Interrupts.h
    Movement/StepTimer.h
    Movement/StepperDrivers/SmartDrivers.h
    Endstops/EndstopsManager.h
    Heating/Heat.h
    Fans/FansManager.h
    Accelerometers/Accelerometers.h
    Storage/MassStorage.h
    CAN/CanInterface.h
    Networking/Network.h
    SerialCDC.h
    Core.h
    Config/Features_Host.h
    CoreTypes.h
    CoreIO.h
    Storage/FileStore.h
    Storage/FileWriteBuffer.h
    Tools/Spindle.h
    UniqueIdBase.h
    Stream.h
    AnalogIn.h
    AnalogOut.h
    FilamentMonitors/FilamentMonitor.h
    GCodes/GCodes.h # Very thin shim just to include AsyncSerial.h
    AsyncSerial.h
    Platform/TaskPriorities.h
    CoreNotifyIndices.h
)


set(RRF_HEADERS_WE_NEED_WITH_orig_POSTFIX
    # All the members in this list will probably also exist in SHADOWING_HEADERS_TO_USE
    # Sometimes, the shadowing header wants to use the original header for something.
    # In that case it will append .orig to its name when including, like this:
    # include <Path/To/Header.h.orig>
    Platform/Heap.h
    CAN/CanInterface.h
    GCodes/GCodes.h
    Platform/TaskPriorities.h
    Platform/Tasks.h
    GCodes/GCodes.h
    Platform/TaskPriorities.h
)

set(HOST_HEADERS_TO_USE
    # Our own headers
    rtos/host_rtos.h
)

# --- Logic to Prepare the Sandboxed Source Tree ---

set(GENERATED_CPP_FILES "")
set(GENERATED_H_FILES "")

# 1. Process the REAL RRF source files
foreach(rel_path ${RRF_SOURCES_TO_USE})
    set(abs_src_path "${RRF_SOURCE_DIR}/src/${rel_path}")
    set(abs_dest_path "${GENERATED_SOURCES_DIR}/src/${rel_path}")
    list(APPEND GENERATED_CPP_FILES ${abs_dest_path})

    # Get the directory part of the destination path for make_directory
    get_filename_component(dest_dir ${abs_dest_path} DIRECTORY)

    add_custom_command(
        OUTPUT ${abs_dest_path}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${dest_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${abs_src_path}" "${abs_dest_path}"
        DEPENDS ${abs_src_path}
        COMMENT "Copying real source ${rel_path}"
    )
endforeach()

foreach(rel_path ${RRF_HEADERS_TO_USE})
    set(abs_src_path "${RRF_SOURCE_DIR}/src/${rel_path}")
    set(abs_dest_path "${GENERATED_SOURCES_DIR}/src/${rel_path}")
    list(APPEND GENERATED_H_FILES ${abs_dest_path})

    get_filename_component(dest_dir ${abs_dest_path} DIRECTORY)

    add_custom_command(
        OUTPUT ${abs_dest_path}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${dest_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${abs_src_path}" "${abs_dest_path}"
        DEPENDS ${abs_src_path}
        COMMENT "Copying real header ${rel_path}"
    )
endforeach()

foreach(rel_path ${RRF_HEADERS_WE_NEED_WITH_orig_POSTFIX})
    set(abs_src_path "${RRF_SOURCE_DIR}/src/${rel_path}")
    set(abs_dest_path "${GENERATED_SOURCES_DIR}/src/${rel_path}.orig")
    list(APPEND GENERATED_H_FILES ${abs_dest_path})

    get_filename_component(dest_dir ${abs_dest_path} DIRECTORY)

    add_custom_command(
        OUTPUT ${abs_dest_path}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${dest_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${abs_src_path}" "${abs_dest_path}"
        DEPENDS ${abs_src_path}
        COMMENT "Copying real header ${rel_path} into ${rel_path}.orig"
    )
endforeach()

# 2. Process the fake/host headers that will overwrite the real ones
foreach(rel_path ${SHADOWING_HEADERS_TO_USE})
    set(abs_src_path "${SIM_ROOT_DIR}/include/${rel_path}")
    set(abs_dest_path "${GENERATED_SOURCES_DIR}/src/${rel_path}")
    list(APPEND GENERATED_H_FILES ${abs_dest_path})

    get_filename_component(dest_dir ${abs_dest_path} DIRECTORY)

    add_custom_command(
        OUTPUT ${abs_dest_path}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${dest_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${abs_src_path}" "${abs_dest_path}"
        DEPENDS ${abs_src_path}
        COMMENT "Copying FAKE header ${rel_path}"
    )
endforeach()
foreach(rel_path ${HOST_HEADERS_TO_USE})
    set(abs_src_path "${SIM_ROOT_DIR}/${rel_path}")
    set(abs_dest_path "${GENERATED_SOURCES_DIR}/src/${rel_path}")
    list(APPEND GENERATED_H_FILES ${abs_dest_path})

    get_filename_component(dest_dir ${abs_dest_path} DIRECTORY)

    add_custom_command(
        OUTPUT ${abs_dest_path}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${dest_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${abs_src_path}" "${abs_dest_path}"
        DEPENDS ${abs_src_path}
        COMMENT "Copying FAKE header ${rel_path}"
    )
endforeach()

# 3. Create a target to encapsulate all the copy operations
add_custom_target(prepare_sources ALL
    DEPENDS ${GENERATED_CPP_FILES} ${GENERATED_H_FILES}
)



# --- Configure the Main Executable ---

# Add the generated .cpp files to the simulator target
target_sources(rrf_simulator PRIVATE ${GENERATED_CPP_FILES})

# CRITICAL: The executable target MUST depend on the preparation target
add_dependencies(rrf_simulator prepare_sources)

# CRITICAL: Set the include directory to ONLY be our sandboxed directory
target_include_directories(rrf_simulator PUBLIC
    # The root of the sandbox `src` dir
    "${GENERATED_SOURCES_DIR}/src"
    # Also include the original RRFLibraries and CANlib if needed
    ${RRFLibraries_SOURCE_DIR}/src
    ${CANlib_SOURCE_DIR}/src
)

# --- Final Touches ---
set_target_properties(rrf_simulator PROPERTIES OUTPUT_NAME "rrf_simulator")

message(STATUS "Sandbox for sources will be at: ${GENERATED_SOURCES_DIR}")
