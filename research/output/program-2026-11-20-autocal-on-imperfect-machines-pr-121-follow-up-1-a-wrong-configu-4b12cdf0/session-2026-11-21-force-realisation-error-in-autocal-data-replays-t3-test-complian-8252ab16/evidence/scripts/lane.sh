#!/bin/bash
# One lane of live imperfect-machine runs (random stiffness), probe on; two lanes at most.
# usage: lane.sh OUTROOT PREFIX MACHINE "SEEDS" [--ellipsoid-shape]
root=$1 prefix=$2 machine=$3 seeds=$4; shift 4
for seed in $seeds; do
  out=$root/$prefix-s$seed
  [ -e $out/result.json ] && continue
  [ -e $out ] && mv $out $out.interrupted.$(date +%s)
  HP_SIM5_COMPLIANCE_PROBE=2,3,1 .venv/bin/python -m autocal.tools.imperfect_machine $out --machine $machine --seed $seed "$@" > $out.stdout 2>&1
done
