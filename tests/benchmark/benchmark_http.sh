#!/usr/bin/env bash
set -euo pipefail

echo "=== HTTP Endpoint Benchmark ==="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
VSD="$ROOT/RRF/run/vsd"
CFG="$VSD/sys/config_slideprinter.g"
PORT="${PORT:-8080}"

if [ ! -x "$BIN" ]; then
    echo "rrf_simulator binary not found at $BIN"
    echo "Build it with: cmake --build RRF/build --target rrf_simulator -j"
    exit 1
fi

"$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" &
SERVER_PID=$!
cleanup() {
    kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3

echo "Benchmarking M115 (100 requests)..."
START=$(date +%s.%N)
for _ in $(seq 1 100); do
    curl -s "http://localhost:${PORT}/machine/code" -d "M115" -H "Content-Type: text/plain" > /dev/null
done
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
RATE=$(echo "100 / $DURATION" | bc -l)
printf "  M115: %.2f requests/sec\n" "$RATE"

echo "Benchmarking M569.4 (100 requests)..."
START=$(date +%s.%N)
for _ in $(seq 1 100); do
    curl -s "http://localhost:${PORT}/machine/code" -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain" > /dev/null
done
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
RATE=$(echo "100 / $DURATION" | bc -l)
printf "  M569.4: %.2f requests/sec\n" "$RATE"

echo "Benchmarking concurrent requests (10 parallel)..."
START=$(date +%s.%N)
for _ in $(seq 1 10); do
    for _ in $(seq 1 10); do
        curl -s "http://localhost:${PORT}/machine/code" -d "M115" -H "Content-Type: text/plain" > /dev/null &
    done
    wait
done
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
RATE=$(echo "100 / $DURATION" | bc -l)
printf "  Concurrent: %.2f requests/sec\n" "$RATE"

echo "Benchmark complete"
