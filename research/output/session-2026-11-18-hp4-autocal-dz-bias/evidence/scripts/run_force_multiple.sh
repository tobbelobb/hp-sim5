#!/bin/bash
# usage: run.sh MULT DIR
cd /home/torbjorn/repos/hp-sim5
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 HP_SIM5_AUTOCAL_JOB=1 HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=4500000 HP_SIM5_TMP_FORCE_MULTIPLE=$1 \
  timeout 5400 .venv/bin/python autocal/autocal.py --headless-sim --machine-type hangprinter_4 --dataset $2/sweeps.json --recording-mode none --patience 8 > $2/autocal.log 2>&1
echo exit $?
