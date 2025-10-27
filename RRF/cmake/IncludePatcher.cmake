# cmake/IncludePatcher.cmake
# Data-driven include patching for third-party sources.
# Usage:
#   list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#   include(IncludePatcher)
#   set(MAP "Platform.h|Platform/Platform.h" "RepRap.h|Platform/RepRap.h")
#   include_patcher_configure(PATCH_SCRIPT ${MAP})
#   include_patcher_add_patched_sources(rrf_simulator "${PATCH_SCRIPT}" ${RRF_SOURCES})

include_guard(GLOBAL)

# Generate a CMake script that rewrites:  #include "Header.h"  ->  #include <mapped/path.h>
# OUT_SCRIPT: var name that will receive the generated script path
# ARGN: list of "Header.h|Platform/Header.h" pairs
function(include_patcher_configure OUT_SCRIPT)
  if(NOT ARGN)
    message(FATAL_ERROR "include_patcher_configure: need at least one mapping (\"from|to\").")
  endif()

  # Stable name per mapping set
  string(SHA256 _hash "${ARGN}")
  string(SUBSTRING "${_hash}" 0 12 _short)
  set(_script "${CMAKE_CURRENT_BINARY_DIR}/patch_includes_${_short}.cmake")

  # Always (re)write so changes take effect
  file(WRITE "${_script}" [=[
# Auto-generated include patcher (no backrefs, no sed)
if(NOT DEFINED INPUT OR NOT DEFINED OUTPUT)
  message(FATAL_ERROR "patch_includes.cmake requires -DINPUT and -DOUTPUT")
endif()

# Strip accidental surrounding quotes
if(INPUT MATCHES "^\".*\"$")
  string(REGEX REPLACE "^\"(.*)\"$" "\\1" INPUT "${INPUT}")
endif()
if(OUTPUT MATCHES "^\".*\"$")
  string(REGEX REPLACE "^\"(.*)\"$" "\\1" OUTPUT "${OUTPUT}")
endif()

# Absolutize
get_filename_component(INPUT  "${INPUT}"  ABSOLUTE)
get_filename_component(OUTPUT "${OUTPUT}" ABSOLUTE)

file(READ "${INPUT}" _c)

# Normalize line endings
string(REPLACE "\r\n" "\n" _c "${_c}")
string(REPLACE "\r"   "\n" _c "${_c}")

# Replace a single include using one regex (no \\1 backrefs)
# We only need to escape '.' inside header names we match (e.g., Platform.h).
function(patch_one header repl)
  set(_h "${header}")
  # Escape '.' in the header for the regex
  string(REPLACE "." "\\." _h "${_h}")

  # Build regex:   #[spaces]include[spaces]"Header.h"
  string(CONCAT _pattern "#[ \t]*include[ \t]*\\\"" "${_h}" "\\\"")
  string(CONCAT _replacement "#include <" "${repl}" ">")

  string(REGEX REPLACE "${_pattern}" "${_replacement}" _c "${_c}")
  set(_c "${_c}" PARENT_SCOPE)
endfunction()
]=])

  foreach(_map IN LISTS ARGN)
    string(REPLACE "|" ";" _parts "${_map}")
    list(LENGTH _parts _n)
    if(NOT _n EQUAL 2)
      message(FATAL_ERROR "Bad mapping '${_map}'. Expected 'from|to'.")
    endif()
    list(GET _parts 0 _from)
    list(GET _parts 1 _to)
    file(APPEND "${_script}" "patch_one(\"${_from}\" \"${_to}\")\n")
  endforeach()

  file(APPEND "${_script}" [=[
get_filename_component(_outdir "${OUTPUT}" DIRECTORY)
file(MAKE_DIRECTORY "${_outdir}")
file(WRITE "${OUTPUT}" "${_c}")
]=])

  set(${OUT_SCRIPT} "${_script}" PARENT_SCOPE)
endfunction()

# Add patched copies of sources to TARGET.
# The copies live under ${binary_dir}/patched_sources/...
# Also add each source's original directory so sibling quoted includes still resolve.
function(include_patcher_add_patched_sources TARGET SCRIPT)
  if(NOT TARGET ${TARGET})
    message(FATAL_ERROR "include_patcher_add_patched_sources: unknown TARGET '${TARGET}'")
  endif()
  if(NOT EXISTS "${SCRIPT}")
    message(FATAL_ERROR "include_patcher_add_patched_sources: SCRIPT '${SCRIPT}' not found")
  endif()
  set(_sources "${ARGN}")
  if(NOT _sources)
    message(FATAL_ERROR "include_patcher_add_patched_sources: no sources provided")
  endif()

  foreach(ABS_SOURCE_FILE IN LISTS _sources)
    file(RELATIVE_PATH REL_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}" "${ABS_SOURCE_FILE}")
    set(PATCHED_SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/patched_sources/${REL_SOURCE_FILE}")

    add_custom_command(
      OUTPUT "${PATCHED_SOURCE_FILE}"
      COMMAND "${CMAKE_COMMAND}"
              -DINPUT:FILEPATH=${ABS_SOURCE_FILE}
              -DOUTPUT:FILEPATH=${PATCHED_SOURCE_FILE}
              -P ${SCRIPT}
      DEPENDS "${ABS_SOURCE_FILE}" "${SCRIPT}"
      VERBATIM
      COMMENT "Patching ${REL_SOURCE_FILE}"
    )

    get_filename_component(_SRC_DIR "${ABS_SOURCE_FILE}" DIRECTORY)
    target_include_directories(${TARGET} PRIVATE "${_SRC_DIR}")
    target_sources(${TARGET} PRIVATE "${PATCHED_SOURCE_FILE}")
  endforeach()
endfunction()
