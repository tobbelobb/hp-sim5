#!/usr/bin/env bash
set -u
set -o pipefail

# Usage: ./run_draw_squares_tests.sh [iterations]
ITERATIONS="${1:-10}"

# Resolve repo root (directory where this script lives)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

BUILD_DIR="$ROOT_DIR/RRF/build"
VSD_DIR="$ROOT_DIR/RRF/run/vsd"
BASE_LOG="$VSD_DIR/logs/test_draw_squares.jsonl"
TEST_LOG="$VSD_DIR/logs/test_draw_squares2.jsonl"

echo "Building rrf_simulator once..."
cmake --build "$BUILD_DIR" --target rrf_simulator -j

correct=0
failed=0

for ((i=1; i<=ITERATIONS; i++)); do
    if ! "$BUILD_DIR/rrf_simulator" \
        --vsd RRF/run/vsd \
        --gcode gcodes/draw_squares.gcode \
        --can-log logs/test_draw_squares2.jsonl \
        -c sys/config_hangprinter.g \
        > /dev/null 2>&1
    then
        # Treat failed run as incorrect
        ((failed++))
        printf "❌"
        continue
    fi

    if [[ ! -f "$BASE_LOG" || ! -f "$TEST_LOG" ]]; then
        ((failed++))
        printf "❌"
        continue
    fi

    diff_count=$(diff "$BASE_LOG" "$TEST_LOG" | wc -l)

    if [[ "$diff_count" -eq 4 ]]; then
        ((correct++))
        printf "✅"
    else
        printf "❌"
    fi
done

incorrect=$((ITERATIONS - correct))

# Summary: single line, wrapped in leading+trailing newline
echo
echo "Total runs: $ITERATIONS, Correct (4 lines): $correct, Incorrect/Failed: $incorrect"
