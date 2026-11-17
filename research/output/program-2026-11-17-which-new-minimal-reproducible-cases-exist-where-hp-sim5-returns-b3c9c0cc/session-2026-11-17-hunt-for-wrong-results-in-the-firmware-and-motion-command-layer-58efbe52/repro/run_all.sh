#!/usr/bin/env bash
# Run every repro in this session. Each script exits nonzero (with a one-line
# reason) when its bug is present, and 0 when the firmware/emulator is exact.
#
#   bash repro/run_all.sh            # all
#   bash repro/run_all.sh findings   # only the bug repros
#   bash repro/run_all.sh negatives  # only the negative-result checks
#
# Requires: node (v24), .venv/bin/python, prebuilt RRF/build/rrf_simulator.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(cd ../../../.. && pwd)"
PY="$ROOT/.venv/bin/python"
mode="${1:-all}"
fail=0

run() {
  local expected="$1"; shift
  local name="$1"; shift
  echo "=== $name (expect $expected)"
  if "$@"; then rc=0; else rc=$?; fi
  if [[ "$expected" == "fail" && $rc -ne 0 ]]; then
    echo "--- reproduced (exit $rc)"
  elif [[ "$expected" == "pass" && $rc -eq 0 ]]; then
    echo "--- clean (exit 0)"
  else
    echo "!!! UNEXPECTED exit $rc" >&2
    fail=1
  fi
}

if [[ "$mode" == "all" || "$mode" == "findings" ]]; then
  run fail  F1_rrf_standstill_offbyone        "$PY" repro/rrf_standstill_offbyone.py
  run fail  F2_rrf_multidriver_csv            node repro/rrf_multidriver_csv.mjs
  run fail  F3_klipper_player_step_loss       node repro/js_mcu_player_step_loss_synthetic.mjs
  run fail  F3b_klipper_player_step_loss_real node repro/js_mcu_player_step_loss.mjs public/mcu_commands/slideprinter/draw_squares.txt
  run fail  F4_klipper_oid_axis_mapping       node repro/js_klipper_oid_axis_mapping.mjs
  run fail  F1b_f1_wait_drift                 "$PY" repro/f1_wait_drift.py
  run fail  F2b_f2_binary_path                node repro/f2_binary_path_probe.mjs
  run fail  F3c_f3_impact_sim                 node repro/f3_impact_sim.mjs
  run fail  F3d_f3_impact_kinematics          "$PY" repro/f3_impact_slideprinter.py
fi

if [[ "$mode" == "all" || "$mode" == "negatives" ]]; then
  run pass  N1_klipper_limits_vs_analytic     "$PY" repro/kl_limits_vs_analytic.py
  run pass  N2_rrf_queue_stress               "$PY" repro/rrf_queue_stress.py
  run pass  N3_bucket_profile_vs_analytic     node repro/js_bucket_profile_vs_analytic.mjs
  run pass  N4_rrf_midmotion_limits            "$PY" repro/rrf_midmotion_limits.py
fi

exit $fail
