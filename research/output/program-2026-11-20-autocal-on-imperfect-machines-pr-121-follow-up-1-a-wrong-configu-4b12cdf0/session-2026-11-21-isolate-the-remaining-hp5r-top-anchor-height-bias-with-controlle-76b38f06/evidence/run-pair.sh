#!/usr/bin/env bash
set -eu
seed=$1
shape=$2
shift 2
out=${HP5Z_RUN_DIR:-/tmp/hp5z-runs}/${seed}${shape}
machine=hangprinter_5_pulley_reduced
mkdir -p "$(dirname "$out")"
args=()
if [[ $shape == e ]]; then args=(--ellipsoid-shape); fi
.venv/bin/python -m autocal.tools.imperfect_machine "$out" --machine "$machine" --seed "$seed" --prepare-only "${args[@]}" "$@" > "$out.setup.log"
export HP_SIM5_AUTOCAL_SCENE="$out/input-scene.usda" HP_SIM5_AUTOCAL_JOB=1 HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=4500000
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 XLA_FLAGS='--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1'
.venv/bin/python autocal/autocal.py --headless-sim --machine-type "$machine" --dataset "$out/sweeps.json" --recording-mode events --patience 8 "${args[@]}" > "$out/autocal.log" 2>&1 || code=$?
.venv/bin/python -m autocal.tools.imperfect_machine "$out" --rescore > "$out/score.log"
exit "${code:-0}"
