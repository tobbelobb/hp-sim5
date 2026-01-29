#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Building rrf_simulator..."
cmake --build "$ROOT/RRF/build" --target rrf_simulator -j

echo ""
echo "Running end-to-end tests..."
echo "=============================="
echo ""

TESTS=(
    "$ROOT/tests/rrf_http_endpoint/test_01_connectivity.sh"
    "$ROOT/tests/rrf_http_endpoint/test_02_set_force_mode.sh"
    "$ROOT/tests/rrf_http_endpoint/test_03_motion.sh"
    "$ROOT/tests/rrf_http_endpoint/test_05_force_mode_then_pos_mode.sh"
)
BASE_PORT="${RRF_HTTP_BASE_PORT:-8080}"

PASSED=0
FAILED=0

for INDEX in "${!TESTS[@]}"; do
    TEST="${TESTS[$INDEX]}"
    TEST_PORT=$((BASE_PORT + INDEX))
    echo ""
    if [ -f "$TEST" ]; then
        if PORT="$TEST_PORT" bash "$TEST"; then
            PASSED=$((PASSED + 1))
        else
            FAILED=$((FAILED + 1))
        fi
    else
        echo "SKIP: $TEST not found"
    fi
done

if command -v node &> /dev/null; then
    echo ""
    SERVER_BIN="$ROOT/RRF/build/rrf_simulator"
    SERVER_VSD="$ROOT/RRF/run/vsd"
    SERVER_CFG="$SERVER_VSD/sys/config_slideprinter.g"
    SERVER_PORT="${RRF_HTTP_JS_PORT:-$((BASE_PORT + ${#TESTS[@]}))}"

    LOG_FILE="${RRF_HTTP_LOG_FILE:-/tmp/rrf_http_js_integration.log}"
    if [[ "${RRF_HTTP_DEBUG:-}" == "1" ]]; then
        "$SERVER_BIN" --vsd "$SERVER_VSD" -c "$SERVER_CFG" --server -p "$SERVER_PORT" &
    else
        "$SERVER_BIN" --vsd "$SERVER_VSD" -c "$SERVER_CFG" --server -p "$SERVER_PORT" >"$LOG_FILE" 2>&1 &
    fi
    SERVER_PID=$!
    cleanup_js() {
        kill "$SERVER_PID" 2>/dev/null || true
    }
    trap cleanup_js EXIT
    sleep 3

    if RRF_SERVER_URL="http://localhost:${SERVER_PORT}" node --experimental-modules "$ROOT/tests/rrf_http_endpoint/test_04_js_integration.mjs"; then
        PASSED=$((PASSED + 1))
    else
        FAILED=$((FAILED + 1))
    fi

    cleanup_js
    trap - EXIT
else
    echo ""
    echo "SKIP: Node.js not available for JS integration test"
fi

echo ""
echo "=============================="
echo "Results: $PASSED passed, $FAILED failed"

if [ $FAILED -gt 0 ]; then
    exit 1
fi
