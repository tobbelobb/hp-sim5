#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <gcode-file>"
  exit 1
fi

GCODE_FILE="$1"
BASENAME="$(basename "$GCODE_FILE" .gcode)"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PYTHON="${KLIPPY_PYTHON:-${ROOT_DIR}/.venv/bin/python}"

"$PYTHON" "$ROOT_DIR/klipper/klippy/klippy.py" \
  "$ROOT_DIR/public/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg" \
  -i $GCODE_FILE \
  -o "$ROOT_DIR/public/mcu_commands/${BASENAME}.serial" \
  -v -d "$ROOT_DIR/public/klipper/linux_mcu/klipper.dict" \

"$PYTHON" "$ROOT_DIR/klipper/klippy/parsedump.py" \
  "$ROOT_DIR/public/klipper/linux_mcu/klipper.dict" \
  "$ROOT_DIR/public/mcu_commands/${BASENAME}.serial" \
  > "$ROOT_DIR/public/mcu_commands/${BASENAME}.txt"
