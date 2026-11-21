#!/bin/bash
# Refit every finished live run that lacks (or has an older) refit.  usage: refit_all.sh RUNROOT EVIDENCE
for run in $1/*/; do run=${run%/}; name=$(basename $run); grep -q fitted_mm $run/result.json 2>/dev/null || continue
  out=$2/results/live/${name}_refit.json
  [ $out -nt $run/result.json ] && [ $out -nt $(dirname $0)/refit2.py ] && continue
  PYTHONPATH=$PWD .venv/bin/python $(dirname $0)/refit2.py $run > $out.tmp && mv $out.tmp $out
done
