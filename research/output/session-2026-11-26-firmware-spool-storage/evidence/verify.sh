#!/usr/bin/env bash
# Headless "Print Squares" (draw_squares_bigger, closed-loop motors) for old/new command streams.
set -uo pipefail
ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
S=${S:?Set S to a directory with old/MACHINE.{can,serial} and base/MACHINE.{can,serial}}
OUT=$S/verify; mkdir -p $OUT
cd $ROOT
run() { # machine firmware label commands
  local out=$OUT/$1_$2_$3.json
  [ -s $out ] || node tests/benchmark/logo_accuracy.mjs --machine $1 --gcode draw_squares_bigger --firmware $2 --commands $4 2>/dev/null | grep '^{' > $out
  node -e "const r=JSON.parse(require('fs').readFileSync('$out','utf8'));const m=r.metrics;console.log(['$1','$2','$3',m.score.toFixed(2),(m.shapeRms*1e3).toFixed(4),(m.shapeP95*1e3).toFixed(4),(m.contourMax*1e3).toFixed(4),r.motors.totalMissedSteps].join('\t'))"
}
for m in ${MACHINES:-hp3 hp4 hp5 hp5_pulley_reduced}; do
  run $m rrf old $S/old/$m.can
  run $m rrf new $ROOT/public/RRF_CAN_commands/draw_squares_bigger_${m}_no_buildup.can
  [ -f $S/base/$m.can ] && run $m rrf base $S/base/$m.can
  run $m klipper old $S/old/$m.serial
  run $m klipper new $ROOT/public/mcu_commands/$m/draw_squares_bigger_no_buildup.serial
  [ -f $S/base/$m.serial ] && run $m klipper base $S/base/$m.serial
done
