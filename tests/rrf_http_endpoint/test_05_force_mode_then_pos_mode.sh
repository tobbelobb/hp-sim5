#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 5: Auto-Calibration Workflow ==="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
VSD="$ROOT/RRF/run/vsd"
CFG="$VSD/sys/config_slideprinter.g"
PORT="${PORT:-8080}"
ENDPOINT="http://localhost:${PORT}/machine/code"
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

echo "  Setting torque mode..."
for DRIVER in 40.0 41.0 42.0; do
    RESPONSE=$(curl -s "$ENDPOINT" -d "M569.4 P$DRIVER T0.001" -H "Content-Type: text/plain")
    [[ "$RESPONSE" == *"Nm"* ]] || fail "Could not set torque mode for $DRIVER (got: $RESPONSE)"
done
echo "  Torque mode set: OK"

echo "  Returning to position mode..."
RESPONSE=$(curl -s "$ENDPOINT" -d "M569.4 P40.0:41.0:42.0 T0" -H "Content-Type: text/plain")
COUNT=$(echo "$RESPONSE" | grep -o "pos_mode" | wc -l | tr -d '[:space:]')
if [ "$COUNT" != "3" ]; then
    fail "Expected 3 pos_mode responses (got: $COUNT)"
fi
echo "  Position mode: OK"

echo "  Getting position..."
RESPONSE=$(curl -s "$ENDPOINT" -d "M114" -H "Content-Type: text/plain")
echo "  Position: $RESPONSE"

echo "PASS: Auto-calibration workflow test"
