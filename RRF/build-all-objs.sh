#!/usr/bin/env bash
# Build all object (.o) targets listed by `make help` from an external directory.
# You can run this script from anywhere; just point it at the CMake build dir.
#
# Usage:
#   ./build-all-objs.sh -C /absolute/or/relative/builddir
#   ./build-all-objs.sh -C RRF/build -j 12
#   BUILD_DIR=RRF/build ./build-all-objs.sh
#   ./build-all-objs.sh -C RRF/build -n          # dry-run: list targets only
#   ./build-all-objs.sh -C RRF/build --each      # one make per target (parallelized)
#
# Notes:
# - Requires GNU make build (CMake generator "Unix Makefiles").
# - Defaults to JOBS = nproc (or hw.ncpu); override with -j N or env JOBS=N.

set -euo pipefail

BUILD_DIR="${BUILD_DIR:-}"
JOBS="${JOBS:-$(command -v nproc >/dev/null 2>&1 && nproc || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"
DRY=0
EACH=0

usage() {
  sed -n '2,35p' "$0" | sed 's/^# \{0,1\}//'
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -C) BUILD_DIR="$2"; shift 2 ;;
    -j) JOBS="$2"; shift 2 ;;
    -n) DRY=1; shift ;;
    --each) EACH=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BUILD_DIR}" ]]; then
  echo "Error: You must specify the CMake build directory with -C <dir> or BUILD_DIR env." >&2
  exit 2
fi

if [[ ! -d "${BUILD_DIR}" ]]; then
  echo "Error: BUILD_DIR does not exist or is not a directory: ${BUILD_DIR}" >&2
  exit 2
fi

if [[ ! -f "${BUILD_DIR}/Makefile" ]]; then
  echo "Error: No Makefile found in BUILD_DIR: ${BUILD_DIR}" >&2
  exit 2
fi

# Extract .o targets from `make help` in the specified build dir
mapfile -t TARGETS < <(make -C "${BUILD_DIR}" help 2>/dev/null \
  | awk '/^\.\.\. /{print $2}' \
  | grep -E '\.o$' \
  | sort -u)

if (( ${#TARGETS[@]} == 0 )); then
  echo "No .o targets found in 'make help' for ${BUILD_DIR}." >&2
  exit 1
fi

if (( DRY )); then
  printf '%s\n' "${TARGETS[@]}"
  exit 0
fi

echo "Doing \`make prepare_sources\` in '${BUILD_DIR}'"
make -C "${BUILD_DIR}" prepare_sources

echo "Building ${#TARGETS[@]} object targets in '${BUILD_DIR}' with -j${JOBS}..."

if (( EACH )); then
  # One `make` per target; parallelize via xargs
  printf '%s\0' "${TARGETS[@]}" \
    | xargs -0 -n1 -P "${JOBS}" -I {} bash -c 'echo "==> make -C "$0" -k {}"; make -C "$0" -k "{}"' "${BUILD_DIR}"
else
  # One `make` call (fastest)
  make -C "${BUILD_DIR}" -k -j"${JOBS}" "${TARGETS[@]}"
fi
