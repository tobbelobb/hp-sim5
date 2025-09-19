#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <gcode-file>"
  exit 1
fi

GCODE_FILE="$1"
BASENAME="$(basename "$GCODE_FILE" .gcode)"

~/klippy-env/bin/python ~/repos/klipper/klippy/klippy.py \
  ~/repos/hp-sim5/examples/klipper/slideprinter/printer-slideprinter-linux-mcu.simple.cfg.incl_extruder_and_heatbed \
  -i $GCODE_FILE \
  -o "${BASENAME}.serial" \
  -v -d ~/repos/hp-sim5/examples/klipper/linux_mcu/klipper.dict \

~/klippy-env/bin/python ~/repos/klipper/klippy/parsedump.py \
  ~/repos/hp-sim5/examples/klipper/linux_mcu/klipper.dict \
  "${BASENAME}.serial" \
  > ~/repos/hp-sim5/public/examples/mcu_commands/${BASENAME}.txt

gio trash "${BASENAME}.serial"
