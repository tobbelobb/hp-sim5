#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 1: Basic Connectivity ==="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
VSD="$ROOT/RRF/run/vsd"
CFG="$VSD/sys/config_slideprinter.g"
PORT="${PORT:-8080}"
LOG_FILE="${RRF_HTTP_LOG_FILE:-/tmp/rrf_http_$(basename "$0" .sh).log}"
ENDPOINT="http://localhost:${PORT}/machine/code"

fail() {
    echo "FAIL: $1"
    if [[ "${RRF_HTTP_DEBUG:-}" != "1" ]]; then
        echo "  (rrf_simulator log: $LOG_FILE)"
    fi
    exit 1
}

cmake --build "$ROOT/RRF/build" --target rrf_simulator -j

if [[ "${RRF_HTTP_DEBUG:-}" == "1" ]]; then
    "$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" &
else
    "$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" >"$LOG_FILE" 2>&1 &
fi
SERVER_PID=$!
cleanup() {
    kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3

RESPONSE=$(curl -s -w "\n%{http_code}" "$ENDPOINT" \
    -d "M115" -H "Content-Type: text/plain") || fail "curl failed for M115"
HTTP_CODE=$(echo "$RESPONSE" | tail -n1)
BODY=$(echo "$RESPONSE" | head -n -1)

if [ "$HTTP_CODE" != "200" ]; then
    echo "FAIL: /machine/code returned $HTTP_CODE"
    exit 1
fi

if [[ "$BODY" != *"FIRMWARE"* ]]; then
    echo "FAIL: M115 did not return firmware info"
    echo "Response: $BODY"
    exit 1
fi
echo "  /machine/code (M115): OK"

echo "PASS: Basic connectivity test"
