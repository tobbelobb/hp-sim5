#!/bin/bash
# Copy the small inputs/outputs of finished runs into evidence/runs (sweeps.json gzipped).  usage: keep_runs.sh EVIDENCE RUN_DIR...
ev=$1; shift
for run in "$@"; do [ -e $run/result.json ] || continue; dst=$ev/runs/$(basename $run); mkdir -p $dst
  cp $run/imperfection.json $run/result.json $dst/; cp $run/sweeps.headless/manifest.json $dst/manifest.json
  gzip -9nc $run/sweeps.json > $dst/sweeps.json.gz; done
