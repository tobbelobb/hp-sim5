#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 2: M569.4 Force Mode ==="

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

curl_post() {
    local data="$1"
    local response
    response=$(curl -s "$ENDPOINT" -d "$data" -H "Content-Type: text/plain") || fail "curl failed for '$data'"
    echo "$response"
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

RESPONSE=$(curl_post "M569.4 P40.0 T0.001")
[[ "$RESPONSE" == *"-0.000030 Nm"* ]] || fail "Expected '-0.000030 Nm' in response (got: $RESPONSE)"
echo "  Set force 0.001 N: OK"

RESPONSE=$(curl_post "M569.4 P40.0 T0")
[[ "$RESPONSE" == *"pos_mode"* ]] || fail "Expected 'pos_mode' in response (got: $RESPONSE)"
echo "  Set position mode: OK"

RESPONSE=$(curl_post "M569.4 P40.0:41.0:42.0 T0.002")
COUNT=$(echo "$RESPONSE" | { grep -o "\-0.000060 Nm" || true; } | wc -l | tr -d '[:space:]')
[[ "$COUNT" == "3" ]] || fail "Expected 3 force values (got: $COUNT) - $RESPONSE"
echo "  Multiple drivers: OK"

RESPONSE=$(curl_post "M569.4 P40.0")
if [[ "$RESPONSE" != *"Error"* && "$RESPONSE" != *"error"* && "$RESPONSE" != *"missing"* ]]; then
    fail "Expected error for missing T parameter (got: $RESPONSE)"
fi
echo "  Error handling: OK"

echo "PASS: M569.4 force mode test"
