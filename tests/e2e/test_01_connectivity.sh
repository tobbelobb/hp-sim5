#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 1: Basic Connectivity ==="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
VSD="$ROOT/RRF/run/vsd"
CFG="$VSD/sys/config_slideprinter.g"
PORT="${PORT:-8080}"

cmake --build "$ROOT/RRF/build" --target rrf_simulator -j

"$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" &
SERVER_PID=$!
cleanup() {
    kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3

RESPONSE=$(curl -s -w "\n%{http_code}" "http://localhost:${PORT}/machine/code" \
    -d "M115" -H "Content-Type: text/plain")
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
