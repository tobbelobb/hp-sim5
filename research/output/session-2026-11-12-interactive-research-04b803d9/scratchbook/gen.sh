#!/usr/bin/env bash
# usage: gen.sh <name> <gcode in vsd/gcodes> <config in vsd/sys>  (run from repo root)
# simulator paths must stay < 120 chars, so the vsd copy lives in the scratchpad
set -euo pipefail
S=$PWD/research/output/session-2026-11-12-interactive-research-04b803d9/scratchbook
./RRF/build/rrf_simulator --vsd /tmp/claude-1000/-home-torbjorn-repos-hp-sim5/08774432-9f2e-49c4-adc5-f6122a8fe5da/scratchpad/v --gcode gcodes/$2 --can-log $S/can/$1.csv -c sys/$3 > $S/can/$1.log 2>&1
.venv/bin/python RRF/csv2can.py $S/can/$1.csv >> $S/can/$1.log 2>&1
rm $S/can/$1.csv
echo "$1 $(grep 'Final machine' $S/can/$1.log)"
