#!/bin/bash
# Blind force-model refits, one at a time (JOBS=4 inside).  usage: blind_lane.sh OUTDIR "RUNS" NAME 'PERTURB_JSON'
out=$1 runs=$2 name=$3 spec=$4
S=$(dirname "$0")
mkdir -p $out
for r in $runs; do
  f=$out/$(basename $r).$name.json
  [ -s $f ] && continue
  PERTURB="$spec" JOBS=4 PYTHONPATH=$PWD .venv/bin/python $S/blind_force_fit.py $r $f.tmp > /dev/null && mv $f.tmp $f
done
