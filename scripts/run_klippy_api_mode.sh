#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

~/klippy-env/bin/python \
  "$SCRIPT_DIR/../klipper/klippy/klippy.py" \
  "$SCRIPT_DIR/../examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg" \
  -a /tmp/klippy_uds \
  -l "/tmp/klipper.log"
