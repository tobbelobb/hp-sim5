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
    "$ROOT/tests/rrf_http_endpoint/test_02_torque_mode.sh"
    "$ROOT/tests/rrf_http_endpoint/test_03_motion.sh"
    "$ROOT/tests/rrf_http_endpoint/test_05_autocal_workflow.sh"
)

PASSED=0
FAILED=0

for TEST in "${TESTS[@]}"; do
    echo ""
    if [ -f "$TEST" ]; then
        if bash "$TEST"; then
            ((PASSED++))
        else
            ((FAILED++))
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
    SERVER_PORT="${PORT:-8080}"

    "$SERVER_BIN" --vsd "$SERVER_VSD" -c "$SERVER_CFG" --server -p "$SERVER_PORT" &
    SERVER_PID=$!
    cleanup_js() {
        kill "$SERVER_PID" 2>/dev/null || true
    }
    trap cleanup_js EXIT
    sleep 3

    if node --experimental-modules "$ROOT/tests/rrf_http_endpoint/test_04_js_integration.mjs"; then
        ((PASSED++))
    else
        ((FAILED++))
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
