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
  (cd "$ROOT" && node autocal/control/tests/e2e/collect_single_sweep.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/find_minimum_moving_force.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/find_edge_force.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/wait_for_stable_encoders.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/return_to_origin_one_at_a_time.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/return_to_origin_all_at_once.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/calibrate_encoder_noise.e2e.test.mjs --sim)
fi
