#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PYTHON="${KLIPPY_PYTHON:-${ROOT_DIR}/.venv/bin/python}"
CONFIG_PATH="${KLIPPY_CONFIG_PATH:-${ROOT_DIR}/public/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg}"
SOCKET_PATH="${KLIPPY_SOCKET_PATH:-/tmp/klippy_uds}"
LOG_PATH="${KLIPPY_LOG_PATH:-/tmp/klipper.log}"
HOST_MCU_BIN="${KLIPPY_HOST_MCU_BIN:-${ROOT_DIR}/public/klipper/linux_mcu/klipper.elf}"
HOST_MCU_SERIAL="${KLIPPY_HOST_MCU_SERIAL:-/tmp/klipper_host_mcu}"
HOST_MCU_REALTIME="${KLIPPY_HOST_MCU_REALTIME:-0}"

HOST_MCU_PID=""
KLIPPY_PID=""

cleanup() {
  local exit_code="${1:-0}"
  trap - EXIT INT TERM HUP
  if [[ -n "${KLIPPY_PID}" ]]; then
    kill "${KLIPPY_PID}" 2>/dev/null || true
  fi
  if [[ -n "${HOST_MCU_PID}" ]]; then
    kill "${HOST_MCU_PID}" 2>/dev/null || true
  fi
  wait "${KLIPPY_PID}" 2>/dev/null || true
  wait "${HOST_MCU_PID}" 2>/dev/null || true
  exit "${exit_code}"
}

wait_for_path() {
  local target_path="$1"
  local timeout_seconds="${2:-5}"
  local attempts=$(( timeout_seconds * 20 ))
  for ((i = 0; i < attempts; i += 1)); do
    if [[ -e "${target_path}" || -L "${target_path}" ]]; then
      return 0
    fi
    sleep 0.05
  done
  echo "Timed out waiting for ${target_path}" >&2
  return 1
}

trap 'cleanup 143' TERM
trap 'cleanup 130' INT
trap 'cleanup 129' HUP
trap 'cleanup $?' EXIT

if [[ ! -x "${HOST_MCU_BIN}" ]]; then
  echo "Linux MCU binary is not executable: ${HOST_MCU_BIN}" >&2
  exit 1
fi

rm -f "${HOST_MCU_SERIAL}"

HOST_MCU_ARGS=(-I "${HOST_MCU_SERIAL}")
if [[ "${HOST_MCU_REALTIME}" == "1" ]]; then
  HOST_MCU_ARGS=(-r "${HOST_MCU_ARGS[@]}")
fi

"${HOST_MCU_BIN}" "${HOST_MCU_ARGS[@]}" &
HOST_MCU_PID=$!
wait_for_path "${HOST_MCU_SERIAL}"

"$PYTHON" \
  "$ROOT_DIR/klipper/klippy/klippy.py" \
  "$CONFIG_PATH" \
  -a "$SOCKET_PATH" \
  -l "$LOG_PATH" &
KLIPPY_PID=$!

wait "${KLIPPY_PID}"
cleanup $?
