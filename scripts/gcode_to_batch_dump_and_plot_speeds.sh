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
  "$ROOT_DIR/public/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg" \
  -i "$GCODE_FILE" \
  -o "${BASENAME}.serial" \
  -v -d "$ROOT_DIR/public/klipper/linux_mcu/klipper.dict"

"$PYTHON" "$ROOT_DIR/klipper/klippy/parsedump.py" \
  "$ROOT_DIR/public/klipper/linux_mcu/klipper.dict" \
  "${BASENAME}.serial" \
  > "$ROOT_DIR/public/mcu_commands/${BASENAME}.txt"

gio trash "${BASENAME}.serial"

head "$ROOT_DIR/public/mcu_commands/${BASENAME}.txt" -n 6000 > "$ROOT_DIR/public/mcu_commands/${BASENAME}.shorter.txt"

"$PYTHON" plot_queue_step_speeds.py \
  --src "$ROOT_DIR/public/mcu_commands/${BASENAME}.shorter.txt" \
  --out-csv queue_step_speeds_per_step_${BASENAME}.csv \
  --clock-hz 50000000 \
  --plot \
  --out-dir plots_${BASENAME}
