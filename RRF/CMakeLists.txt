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
    #${RRF_SOURCE_DIR}/src/ObjectModel/Variable.cpp
    #${RRF_SOURCE_DIR}/src/ObjectModel/ObjectModel.cpp
    #${RRF_SOURCE_DIR}/src/ObjectModel/GlobalVariables.cpp

    # Tier 2: Core Machine Components (Stateful but not Active)
    #${RRF_SOURCE_DIR}/src/Tools/Filament.cpp
    #${RRF_SOURCE_DIR}/src/Tools/Tool.cpp
    #${RRF_SOURCE_DIR}/src/Movement/Kinematics/Kinematics.cpp
    #${RRF_SOURCE_DIR}/src/Movement/Kinematics/CoreKinematics.cpp
    #${RRF_SOURCE_DIR}/src/Movement/Kinematics/HangprinterKinematics.cpp

    # Tier 3: The Motion Planning Engine (Bottom-Up)
    #${RRF_SOURCE_DIR}/src/Movement/DriveMovement.cpp
    #${RRF_SOURCE_DIR}/src/Movement/DDA.cpp
    #${RRF_SOURCE_DIR}/src/Movement/DDARing.cpp
    #${RRF_SOURCE_DIR}/src/Movement/Move.cpp

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

# --- Helper function for patching #include "Somefile.h" into #include <SomeDir/SomeFile.h>, so we can include-shadow it ---
# This avoids any shell/sed usage and works cross-platform.
set(PATCH_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/patch_includes.cmake")
file(WRITE "${PATCH_SCRIPT}" "
# patch_includes.cmake
if(NOT DEFINED INPUT OR NOT DEFINED OUTPUT)
  message(FATAL_ERROR \"patch_includes.cmake requires -DINPUT and -DOUTPUT\")
endif()

# Strip accidental surrounding quotes (if any)
if(INPUT MATCHES \"^\\\".*\\\"$\")
  string(REGEX REPLACE \"^\\\"(.*)\\\"$\" \"\\\\1\" INPUT \"\${INPUT}\")
endif()
if(OUTPUT MATCHES \"^\\\".*\\\"$\")
  string(REGEX REPLACE \"^\\\"(.*)\\\"$\" \"\\\\1\" OUTPUT \"\${OUTPUT}\")
endif()

# Absolutize relative paths against the build dir
get_filename_component(INPUT  \"\${INPUT}\"  ABSOLUTE BASE_DIR \"${CMAKE_CURRENT_BINARY_DIR}\")
get_filename_component(OUTPUT \"\${OUTPUT}\" ABSOLUTE BASE_DIR \"${CMAKE_CURRENT_BINARY_DIR}\")

file(READ \"\${INPUT}\" _c)

# Normalize line endings
string(REPLACE \"\\r\\n\" \"\\n\" _c \"\${_c}\")
string(REPLACE \"\\r\"   \"\\n\" _c \"\${_c}\")

# Nuke the quoted includes without changing the upstream ReprapFirmware repo...
# Replace: #include \"Header.h\"  ->  #include <SomeDir/Header.h>
string(REGEX REPLACE \"#[ \\t]*include[ \\t]*\\\"Platform\\\\.h\\\"\" \"#include <Platform/Platform.h>\" _c \"\${_c}\")
string(REGEX REPLACE \"#[ \\t]*include[ \\t]*\\\"RepRap\\\\.h\\\"\"   \"#include <Platform/RepRap.h>\"   _c \"\${_c}\")
string(REGEX REPLACE \"#[ \\t]*include[ \\t]*\\\"ArrayHandle\\\\.h\\\"\"   \"#include <Platform/ArrayHandle.h>\"   _c \"\${_c}\")
string(REGEX REPLACE \"#[ \\t]*include[ \\t]*\\\"StringHandle\\\\.h\\\"\"   \"#include <Platform/StringHandle.h>\"   _c \"\${_c}\")
string(REGEX REPLACE \"#[ \\t]*include[ \\t]*\\\"OutputMemory\\\\.h\\\"\"   \"#include <Platform/OutputMemory.h>\"   _c \"\${_c}\")

get_filename_component(_outdir \"\${OUTPUT}\" DIRECTORY)
file(MAKE_DIRECTORY \"\${_outdir}\")
file(WRITE \"\${OUTPUT}\" \"\${_c}\")
")

# --- Helper function: patch headers in pure CMake and add resulting files ---
function(patch_and_add_sources TARGET_NAME)
  foreach(ABS_SOURCE_FILE IN LISTS ARGN)
    file(RELATIVE_PATH REL_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}" "${ABS_SOURCE_FILE}")
    set(PATCHED_SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/patched_sources/${REL_SOURCE_FILE}")
    get_filename_component(PATCHED_SOURCE_DIR "${PATCHED_SOURCE_FILE}" DIRECTORY)

    add_custom_command(
      OUTPUT "${PATCHED_SOURCE_FILE}"
      COMMAND "${CMAKE_COMMAND}" -DINPUT:FILEPATH="${ABS_SOURCE_FILE}"
                                  -DOUTPUT:FILEPATH="${PATCHED_SOURCE_FILE}"
                                  -P "${PATCH_SCRIPT}"
      DEPENDS "${ABS_SOURCE_FILE}" "${PATCH_SCRIPT}"
      VERBATIM
      COMMENT "Patching ${REL_SOURCE_FILE}"
    )

    target_sources(${TARGET_NAME} PRIVATE "${PATCHED_SOURCE_FILE}")
  endforeach()
endfunction()

patch_and_add_sources(rrf_simulator ${RRF_SOURCES})


# --- Manage Include Directories ---
# Order matters: fakes first, then real headers as fallback.

# 1) Root of fake include directory so <Platform/Platform.h> etc. resolve.
target_include_directories(rrf_simulator PUBLIC
    "${SIM_ROOT_DIR}"
    "${SIM_ROOT_DIR}/include"
)

# 2) Add all subdirectories under host/include (only directories).
file(GLOB ALL_FAKE_INCLUDE_ITEMS LIST_DIRECTORIES true "${SIM_ROOT_DIR}/include/*")
set(FAKE_HEADER_DIRS "")
foreach(item ${ALL_FAKE_INCLUDE_ITEMS})
    if(IS_DIRECTORY ${item})
        list(APPEND FAKE_HEADER_DIRS ${item})
    endif()
endforeach()
target_include_directories(rrf_simulator PUBLIC
    ${FAKE_HEADER_DIRS}
)

# 3) Real RRF headers last (fallback).
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
