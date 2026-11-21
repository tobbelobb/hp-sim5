#!/bin/bash
# Robustness refits (perturb.py): one job at a time, JOBS=4 inside.  usage: robust_lane.sh OUTDIR "RUNS" NAME PERTURB_JSON [HARMONICS] [HOMESTRAIN]
out=$1 runs=$2 name=$3 spec=$4 harm=${5:-50} home=${6:-}
S=$(dirname "$0")
mkdir -p $out
for r in $runs; do
  f=$out/$(basename $r).$name.json
  [ -s $f ] && continue
  PERTURB="$spec" HOMESTRAIN=$home JOBS=4 PYTHONPATH=$PWD .venv/bin/python $S/joint_k_cog.py $r $harm > $f.tmp && mv $f.tmp $f
done
