#!/bin/bash
# Copy the small inputs/outputs of finished runs into evidence/runs.  usage: keep_runs.sh EVIDENCE RUN_DIR...
# Kept: imperfection, result, manifest, force tuning, and gzipped full_auto log, final and rough datasets.
ev=$1; shift
for run in "$@"; do [ -e $run/result.json ] || continue; dst=$ev/runs/$(basename $run); mkdir -p $dst
  cp $run/imperfection.json $run/result.json $dst/; cp $run/sweeps.headless/manifest.json $dst/manifest.json
  cp $run/sweeps.json.force-tuning.json $dst/force-tuning.json
  for f in sweeps.json sweeps.rough_bootstrap.json sweeps.full_auto.log; do
    [ -e $run/$f ] && gzip -9nc $run/$f > $dst/$f.gz; done; done
