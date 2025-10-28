cmake_minimum_required(VERSION 3.16)
project(RepRapFirmwareSimulator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Project Paths (Relative to this CMakeLists.txt) ---
# The root of our simulation project (the 'host' directory)
set(SIM_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/host)
# The location of the real ReprapFirmware source
set(RRF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ReprapFirmware)
set(RRFLibraries_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/RRFLibraries)
set(CANlib_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CANlib)

set(CMAKE_COLOR_DIAGNOSTICS ON)

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
    # Add .cpp files that implement fakes here
)


# --- List the real ReprapFirmware Source Files ---
# This is the list of .cpp files we want to compile the real version of.
set(RRF_SOURCES
    # Tier 1: Foundational Utilities
    ${RRF_SOURCE_DIR}/src/Platform/ArrayHandle.cpp
    ${RRF_SOURCE_DIR}/src/Platform/StringHandle.cpp
    ${RRF_SOURCE_DIR}/src/Platform/OutputMemory.cpp
    ${RRF_SOURCE_DIR}/src/ObjectModel/Variable.cpp
    ${RRF_SOURCE_DIR}/src/ObjectModel/ObjectModel.cpp
    ${RRF_SOURCE_DIR}/src/ObjectModel/GlobalVariables.cpp

    # Tier 2: Core Machine Components (Stateful but not Active)
    ${RRF_SOURCE_DIR}/src/Tools/Filament.cpp
    ${RRF_SOURCE_DIR}/src/Tools/Tool.cpp
    ${RRF_SOURCE_DIR}/src/Movement/Kinematics/Kinematics.cpp
    ${RRF_SOURCE_DIR}/src/Movement/Kinematics/CoreKinematics.cpp
    ${RRF_SOURCE_DIR}/src/Movement/Kinematics/HangprinterKinematics.cpp

    # Tier 3: The Motion Planning Engine (Bottom-Up)
    ${RRF_SOURCE_DIR}/src/Movement/DriveMovement.cpp
    ${RRF_SOURCE_DIR}/src/Movement/DDA.cpp
    ${RRF_SOURCE_DIR}/src/Movement/DDARing.cpp
    ${RRF_SOURCE_DIR}/src/Movement/Move.cpp

    # Tier 4: G-Code Interpretation and High-Level Logic
    #${RRF_SOURCE_DIR}/src/GCodes/GCodeBuffer/GCodeBuffer.cpp
    #${RRF_SOURCE_DIR}/src/ExpressionParser.cpp
    #${RRF_SOURCE_DIR}/src/StringParser.cpp
    #${RRF_SOURCE_DIR}/src/GCodes/GCodeMachineState.cpp
    #${RRF_SOURCE_DIR}/src/GCodes/GCodeInput.cpp
    #${RRF_SOURCE_DIR}/src/GCodes/GCodeQueue.cpp
    #${RRF_SOURCE_DIR}/src/GCodes/GCodes.cpp
    #${RRF_SOURCE_DIR}/src/GCodes/GCodes2.cpp
    #${RRF_SOURCE_DIR}/src/GCodes/GCodes3.cpp

    # Tier 5: Top-Level Application Objects
    #${RRF_SOURCE_DIR}/src/PrintMonitor/PrintMonitor.cpp
    #${RRF_SOURCE_DIR}/src/Platform/RepRap.cpp
)

# --- Patching #include "Somefile.h" into #include <SomeDir/SomeFile.h>
# This gives us control. We can shadow bracketed includes, but not quoted includes ---
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(IncludePatcher)

# Declare mappings, (SomeFile.h|SomeDir/SomeFile.h)
set(INCLUDE_PATCH_MAP
  "Platform.h|Platform/Platform.h"
  "RepRap.h|Platform/RepRap.h"
  "ArrayHandle.h|Platform/ArrayHandle.h"
  "StringHandle.h|Platform/StringHandle.h"
  "OutputMemory.h|Platform/OutputMemory.h"
  "StepperDrivers/SmartDrivers.h|Movement/StepperDrivers/SmartDrivers.h"
)
include_patcher_configure(PATCH_SCRIPT ${INCLUDE_PATCH_MAP})
include_patcher_add_patched_sources(rrf_simulator "${PATCH_SCRIPT}" ${RRF_SOURCES})

# --- Manage Include Directories ---
# Order matters: fakes first, then real headers as fallback.

# Root of fake include directory so <Platform/Platform.h> etc. resolve.
target_include_directories(rrf_simulator PUBLIC
    "${SIM_ROOT_DIR}"
    "${SIM_ROOT_DIR}/include"
)

# Real RRF headers last (fallback).
target_include_directories(rrf_simulator PRIVATE
    ${RRF_SOURCE_DIR}/src
    #${RRF_SOURCE_DIR}/src/Hardware/SAME70
    ${RRFLibraries_SOURCE_DIR}/src
    ${CANlib_SOURCE_DIR}/src
)


# --- Final Touches ---
set_target_properties(rrf_simulator PROPERTIES OUTPUT_NAME "rrf_simulator")

# Diagnostics
message(STATUS "Simulator Root: ${SIM_ROOT_DIR}")
message(STATUS "RepRapFirmware Source: ${RRF_SOURCE_DIR}")
message(STATUS "Fake header subdirectories: ${FAKE_HEADER_DIRS}")
