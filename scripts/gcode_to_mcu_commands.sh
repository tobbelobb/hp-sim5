#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <gcode-file>"
  exit 1
fi

GCODE_FILE="$1"
BASENAME="$(basename "$GCODE_FILE" .gcode)"

~/klippy-env/bin/python ~/repos/hp-sim5/klipper/klippy/klippy.py \
  ~/repos/hp-sim5/examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg \
  -i $GCODE_FILE \
  -o ~/repos/hp-sim5/public/examples/mcu_commands/"${BASENAME}.serial" \
  -v -d ~/repos/hp-sim5/examples/klipper/linux_mcu/klipper.dict \

~/klippy-env/bin/python ~/repos/hp-sim5/klipper/klippy/parsedump.py \
  ~/repos/hp-sim5/examples/klipper/linux_mcu/klipper.dict \
  ~/repos/hp-sim5/public/examples/mcu_commands/"${BASENAME}.serial" \
  > ~/repos/hp-sim5/public/examples/mcu_commands/${BASENAME}.txt
