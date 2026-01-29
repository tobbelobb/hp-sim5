#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 2: M569.4 Force Mode ==="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
VSD="$ROOT/RRF/run/vsd"
CFG="$VSD/sys/config_slideprinter.g"
PORT="${PORT:-8080}"
LOG_FILE="${RRF_HTTP_LOG_FILE:-/tmp/rrf_http_$(basename "$0" .sh).log}"

fail() {
    echo "FAIL: $1"
    exit 1
}

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

RESPONSE=$(curl -s "http://localhost:${PORT}/machine/code" \
    -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain")
[[ "$RESPONSE" == *"-0.000030 Nm"* ]] || fail "Expected '-0.000030 Nm' in response (got: $RESPONSE)"
echo "  Set force 0.001 N: OK"

RESPONSE=$(curl -s "http://localhost:${PORT}/machine/code" \
    -d "M569.4 P40.0 T0" -H "Content-Type: text/plain")
[[ "$RESPONSE" == *"pos_mode"* ]] || fail "Expected 'pos_mode' in response (got: $RESPONSE)"
echo "  Set position mode: OK"

RESPONSE=$(curl -s "http://localhost:${PORT}/machine/code" \
    -d "M569.4 P40.0:41.0:42.0 T0.002" -H "Content-Type: text/plain")
COUNT=$(echo "$RESPONSE" | grep -o "\-0.000060 Nm" | wc -l | tr -d '[:space:]')
[[ "$COUNT" == "3" ]] || fail "Expected 3 force values (got: $COUNT) - $RESPONSE"
echo "  Multiple drivers: OK"

RESPONSE=$(curl -s "http://localhost:${PORT}/machine/code" \
    -d "M569.4 P40.0" -H "Content-Type: text/plain")
if [[ "$RESPONSE" != *"Error"* && "$RESPONSE" != *"error"* && "$RESPONSE" != *"missing"* ]]; then
    fail "Expected error for missing T parameter (got: $RESPONSE)"
fi
echo "  Error handling: OK"

echo "PASS: M569.4 force mode test"
