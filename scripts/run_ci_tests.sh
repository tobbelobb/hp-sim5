#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "== Build shared Rust physics for native and Wasm =="
(cd "$ROOT" && npm run build:native -- --no-pgo && npm run build:wasm)

echo "== Native Rust smoke tests =="
(cd "$ROOT" && cargo test --manifest-path src/wasm/cable_solver/Cargo.toml --locked)

echo "== HP4 production logo and loaded-state stability (mandatory) =="
(cd "$ROOT" && node --test tests/benchmark/pinhole_gate.test.mjs tests/benchmark/hp4_logo_stability.test.mjs \
  tests/benchmark/hp4_stability.test.mjs)

echo "== Rust source contract, JS parity and host integration =="
(cd "$ROOT" && node --expose-gc --test tests/benchmark/native_physics_engine.test.mjs \
  tests/benchmark/wasm_cable_solver.test.mjs tests/benchmark/physics_engine_ui.test.mjs \
  tests/benchmark/profile_headless.test.mjs tests/benchmark/hp4_open_loop_substeps.test.mjs \
  tests/benchmark/slideprinter_mounted_motors.test.mjs tests/benchmark/mounted_rotor_dynamics.test.mjs)

echo "== JS unit tests (jest) =="
(cd "$ROOT" && npx jest)

echo ""
echo "== Python unit tests (autocal) =="
(cd "$ROOT" && .venv/bin/python -m pytest autocal/tests)

echo ""
echo "== Python unit tests (cable_joints/slideprinter/flipper) =="
(cd "$ROOT" && .venv/bin/python -m pytest tests/python)

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
