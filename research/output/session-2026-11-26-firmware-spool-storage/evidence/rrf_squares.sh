#!/usr/bin/env bash
# Usage: rrf_squares.sh MACHINE CONFIG_NAME EXTRA_M666 VARIANT OUT_PREFIX
set -euo pipefail
ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
machine=$1; config=$2; extra=$3; variant=$4; out=$5
work=$(mktemp -d)
cp -r $ROOT/RRF/run/vsd/sys $work/sys; mkdir $work/gcodes
cp $ROOT/public/gcode/draw_squares_bigger.gcode $work/gcodes/
[ -n "$extra" ] && echo "$extra" >> $work/sys/$config
cd $ROOT
HP_SPOOL_VARIANT=$variant RRF/build/rrf_simulator --vsd $work --gcode gcodes/draw_squares_bigger.gcode -c sys/$config --can-log $work/sq.csv > $work/rrf.log 2>&1
.venv/bin/python RRF/csv2can.py $work/sq.csv > /dev/null
cp $work/sq.can $out.can
node tests/benchmark/logo_accuracy.mjs --machine $machine --gcode draw_squares_bigger --firmware rrf --commands $out.can 2>/dev/null > $out.json
rm -rf $work
node -e "const r=JSON.parse(require('fs').readFileSync('$out.json','utf8').split('\n').filter(l=>l.startsWith('{')).pop());const m=r.metrics;console.log('$out'.split('/').pop(),'score',m.score.toFixed(2),'shapeRms',(m.shapeRms*1e3).toFixed(4),'shapeP95',(m.shapeP95*1e3).toFixed(4),'contourMax',(m.contourMax*1e3).toFixed(4),'missed',r.motors.totalMissedSteps)"
