#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PYTHON="${KLIPPY_PYTHON:-${ROOT_DIR}/.venv/bin/python}"
CONFIG_PATH="${KLIPPY_CONFIG_PATH:-${ROOT_DIR}/examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg}"
SOCKET_PATH="${KLIPPY_SOCKET_PATH:-/tmp/klippy_uds}"
LOG_PATH="${KLIPPY_LOG_PATH:-/tmp/klipper.log}"

exec "$PYTHON" \
  "$ROOT_DIR/klipper/klippy/klippy.py" \
  "$CONFIG_PATH" \
  -a "$SOCKET_PATH" \
  -l "$LOG_PATH"
