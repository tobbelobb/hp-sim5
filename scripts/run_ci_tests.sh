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

run_with_retries() {
  local label="$1"
  local cmd="$2"
  local max_tries=3
  local attempt=1

  while true; do
    echo "$label (attempt $attempt/$max_tries)"
    if (cd "$ROOT" && eval "$cmd"); then
      return 0
    fi
    if [[ "$attempt" -ge "$max_tries" ]]; then
      return 1
    fi
    attempt=$((attempt + 1))
    echo "Retrying..."
  done
}

if [[ "${RUN_DETERMINISM_TESTS:-0}" == "1" ]]; then
  echo ""
  echo "== RRF determinism tests (retry up to 3 times) =="
  run_with_retries "draw_squares" "RRF/tests/run_draw_squares_determinism_test.sh"
  run_with_retries "logo" "RRF/tests/run_logo_determinism_test.sh"
  run_with_retries "logo_slideprinter" "RRF/tests/run_logo_slideprinter_determinism_test.sh"
fi

if [[ "${RUN_RRF_HTTP_ENDPOINT_TESTS:-1}" == "1" ]]; then
  echo ""
  echo "== RRF HTTP endpoint tests =="
  (cd "$ROOT" && tests/run_all_rrf_http_endpoint_tests.sh)
fi

if [[ "${RUN_SIM_E2E:-0}" == "1" ]]; then
  echo ""
  echo "== hp-sim UI E2E tests (requires visible browser and human expert inspection) =="
  (cd "$ROOT" && node autocal/control/tests/e2e/collect_single_sweep.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/find_minimum_moving_force.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/find_edge_force.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/wait_for_stable_encoders.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/return_to_origin_one_at_a_time.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/return_to_origin_all_at_once.e2e.test.mjs --sim)
  (cd "$ROOT" && node autocal/control/tests/e2e/calibrate_encoder_noise.e2e.test.mjs --sim)
fi
