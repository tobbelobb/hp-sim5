#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON="${KLIPPY_PYTHON:-${ROOT_DIR}/.venv/bin/python}"

"$PYTHON" "$ROOT_DIR/klipper/klippy/klippy.py" "$ROOT_DIR/public/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg" -i "$ROOT_DIR/public/gcode/Hangprinter_logo6.gcode" -v -l "$ROOT_DIR/attic/klipper.log"
