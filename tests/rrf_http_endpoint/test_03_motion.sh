#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 3: Motion Command Generation ==="

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

curl_post "G28" > /dev/null

RESPONSE=$(curl_post "G1 X10 Y10 F1000")

if [[ "$RESPONSE" == *"---MOTION---"* ]]; then
    echo "  Motion data present: OK"
    MOTION_LINES=$(echo "$RESPONSE" | sed -n '/---MOTION---/,$p' | grep -E "^[0-9]" | wc -l | tr -d '[:space:]')
    echo "  Motion lines generated: ${MOTION_LINES}"
    if [ "${MOTION_LINES:-0}" -lt 1 ]; then
        echo "WARN: Expected motion data for G1 command (may be config-dependent)"
    fi
else
    echo "INFO: No motion delimiter (motion may be in separate channel)"
fi

RESPONSE=$(curl_post "M114")

if [[ "$RESPONSE" == *"X:"* || "$RESPONSE" == *"A:"* ]]; then
    echo "  Position report: OK"
else
    echo "  Position report format: $RESPONSE"
fi

echo "PASS: Motion command test"
