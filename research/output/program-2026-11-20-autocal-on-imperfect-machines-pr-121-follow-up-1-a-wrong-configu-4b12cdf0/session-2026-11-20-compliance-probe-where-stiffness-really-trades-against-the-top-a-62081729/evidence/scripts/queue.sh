#!/bin/bash
# One lane of live probe runs; run two lanes at most.  usage: queue.sh OUTROOT PREFIX MACHINE [--ellipsoid-shape]
root=$1 prefix=$2 machine=$3; shift 3
for seed in 1 2 3; do for ea in 0.8 1.2; do
  out=$root/$prefix-s$seed-ea$ea
  [ -e $out/result.json ] && continue
  [ -e $out ] && mv $out $out.interrupted.$(date +%s)
  HP_SIM5_COMPLIANCE_PROBE=2,3,1 .venv/bin/python -m autocal.tools.imperfect_machine $out --machine $machine --seed $seed --stiffness-scale $ea "$@" > $out.stdout 2>&1
done; done
