#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
CFG="$ROOT/RRF/run/vsd/sys/config_slideprinter.g"
VSD="$ROOT/RRF/run/vsd"
PORT=8080

"$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" > /dev/null 2>&1 &
PID=$!
cleanup(){ kill $PID 2>/dev/null || true; }
trap cleanup EXIT
sleep 2

check() {
  local expected=$2 payload=$1
  resp=$(curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "$payload")
  echo "Sent: \"$payload\""
  echo "Got: $resp"
  if [[ "$resp" != *"$expected"* ]]; then
    echo "FAIL: $name"
    exit 1
  fi
  echo "Success"
}

check "M115" "FIRMWARE_NAME"
check "M114" "X:"
check "M999999" "M999999: Command is not supported"
check $'M115\nM114' "FIRMWARE_NAME"

echo "PASS: HTTP injector tests"
