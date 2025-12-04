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

fail() {
  echo "FAIL: $1"
  exit 1
}

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
check "M569.4 P40.0 T0.001" "0.001000 Nm"
check "M569.4 P40.0 T0" "pos_mode"
check "M569.4 P40.0:41.0 T0" "pos_mode, pos_mode"
check "M569.4 P40.0" "Error"

# Motion capture should include the full set of CAN frames across sequential moves.
curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "G92 X0 Y0 Z0" > /dev/null
resp_x=$(curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "G1 X100")
[[ "$resp_x" == *"---MOTION---"* ]] || fail "G1 X100 did not return motion data"
motion_x=$(echo "$resp_x" | sed -n '/---MOTION---/,$p' | tail -n +3 | tr -d '\r')
first_idx_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{print $1; exit}')
last_idx_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{val=$1} END{print val}')
count_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{c++} END{print c+0}')
[[ -n "${first_idx_x:-}" && -n "${last_idx_x:-}" && -n "${count_x:-}" ]] || fail "Could not parse CAN indices for G1 X100"
(( last_idx_x - first_idx_x + 1 == count_x )) || fail "Missing CAN packets inside G1 X100 motion capture"

resp_y=$(curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "G1 Y100")
[[ "$resp_y" == *"---MOTION---"* ]] || fail "G1 Y100 did not return motion data"
motion_y=$(echo "$resp_y" | sed -n '/---MOTION---/,$p' | tail -n +3 | tr -d '\r')
first_idx_y=$(echo "$motion_y" | awk -F',' '/^[0-9]+/{print $1; exit}')
[[ -n "${first_idx_y:-}" ]] || fail "Could not parse CAN indices for G1 Y100"
(( first_idx_y == last_idx_x + 1 )) || fail "Motion capture dropped CAN packets between sequential moves"

echo "PASS: HTTP injector tests"
