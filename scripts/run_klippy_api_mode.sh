#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PYTHON="${KLIPPY_PYTHON:-${ROOT_DIR}/.venv/bin/python}"

"$PYTHON" \
  "$ROOT_DIR/klipper/klippy/klippy.py" \
  "$ROOT_DIR/examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg" \
  -a /tmp/klippy_uds \
  -l "/tmp/klipper.log"
