#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "== JS unit tests (jest) =="
(cd "$ROOT" && npx jest)

echo ""
echo "== Python unit tests (autocal) =="
(cd "$ROOT" && python -m pytest autocal/tests)

echo ""
echo "== Python unit tests (cable_joints/slideprinter/flipper) =="
(cd "$ROOT" && python -m pytest tests/python)

if [[ "${RUN_E2E:-0}" == "1" ]]; then
  echo ""
  echo "== RRF HTTP E2E tests =="
  (cd "$ROOT" && cmake --build RRF/build --target rrf_simulator -j)
  (cd "$ROOT" && tests/run_all_e2e_tests.sh)
  (cd "$ROOT" && ./test_http_subtasks.sh)
fi

if [[ "${RUN_SIM_E2E:-0}" == "1" ]]; then
  echo ""
  echo "== hp-sim UI E2E tests (requires visible browser) =="
  (cd "$ROOT" && node scripts/e2e_test_collect_single_sweep.mjs --sim)
  (cd "$ROOT" && node scripts/e2e_test_find_minimum_moving_force.mjs --sim)
  (cd "$ROOT" && node scripts/e2e_test_find_edge_force.mjs --sim)
  (cd "$ROOT" && node scripts/e2e_test_wait_for_stable_encoders.mjs --sim)
  (cd "$ROOT" && node scripts/e2e_test_return_to_origin_one_at_a_time.mjs --sim)
  (cd "$ROOT" && node scripts/e2e_test_return_to_origin_all_at_once.mjs --sim)
  (cd "$ROOT" && node scripts/e2e_test_calibrate_encoder_noise.mjs --sim)
fi
