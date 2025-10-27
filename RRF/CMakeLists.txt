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
function(patch_and_add_sources TARGET_NAME)
  foreach(ABS_SOURCE_FILE IN LISTS ARGN)
    file(RELATIVE_PATH REL_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}" "${ABS_SOURCE_FILE}")
    set(PATCHED_SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/patched_sources/${REL_SOURCE_FILE}")
    get_filename_component(PATCHED_SOURCE_DIR "${PATCHED_SOURCE_FILE}" DIRECTORY)

    add_custom_command(
      OUTPUT "${PATCHED_SOURCE_FILE}"
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${PATCHED_SOURCE_DIR}"
      # Use a shell so '>' redirection works; quote everything; use '@' as sed delimiter.
      COMMAND "${CMAKE_COMMAND}" -E sh -c
        "sed -e 's@#include \\\"Platform.h\\\"@#include <Platform/Platform.h>@' \
              -e 's@#include \\\"RepRap.h\\\"@#include <Platform/RepRap.h>@' \
              \"$<SHELL_PATH:${ABS_SOURCE_FILE}>\" > \"$<SHELL_PATH:${PATCHED_SOURCE_FILE}>\""
      DEPENDS "${ABS_SOURCE_FILE}"
      VERBATIM
      COMMENT "Patching ${REL_SOURCE_FILE}"
    )

    target_sources(${TARGET_NAME} PRIVATE "${PATCHED_SOURCE_FILE}")
  endforeach()
endfunction()

patch_and_add_sources(rrf_simulator ${RRF_SOURCES})


# --- Manage Include Directories ---
# This section solves "sibling include" and "double include" problems.
# The order is important: fakes first

# 1. Add the root of the fake include directory.
#    This makes includes like <Platform/Platform.h> work.
target_include_directories(rrf_simulator PUBLIC
    "${SIM_ROOT_DIR}"
    "${SIM_ROOT_DIR}/include"
)

# 2. Automatically find and add all subdirectories of the fake include dir.
#    First, get everything matching the pattern.
file(GLOB ALL_FAKE_INCLUDE_ITEMS LIST_DIRECTORIES true "${SIM_ROOT_DIR}/include/*")
#    Then, create a new list and fill it ONLY with the items that are directories.
set(FAKE_HEADER_DIRS "")
foreach(item ${ALL_FAKE_INCLUDE_ITEMS})
    if(IS_DIRECTORY ${item})
        list(APPEND FAKE_HEADER_DIRS ${item})
    endif()
endforeach()
#    Finally, add the clean list of directories to the include path.
target_include_directories(rrf_simulator PUBLIC
    ${FAKE_HEADER_DIRS}
)

# 3. Add the ROOT of the ReprapFirmware 'src' directory last.
#    This is the fallback for all real headers we are not faking.
target_include_directories(rrf_simulator PRIVATE
    ${RRF_SOURCE_DIR}/src
    #${RRF_SOURCE_DIR}/src/Hardware/SAME70
    ${RRFLibraries_SOURCE_DIR}/src
    ${CANlib_SOURCE_DIR}/src
)


# --- Final Touches ---
set_target_properties(rrf_simulator PROPERTIES OUTPUT_NAME "rrf_simulator")

# Add a message to confirm paths are set correctly
message(STATUS "Simulator Root: ${SIM_ROOT_DIR}")
message(STATUS "RepRapFirmware Source: ${RRF_SOURCE_DIR}")
message(STATUS "Fake header subdirectories: ${FAKE_HEADER_DIRS}")
