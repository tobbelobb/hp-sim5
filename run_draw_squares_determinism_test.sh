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
BASE_LOG="$VSD_DIR/logs/test_draw_squares.csv"
TEST_LOG="$VSD_DIR/logs/test_draw_squares2.csv"

echo "Building rrf_simulator once..."
cmake --build "$BUILD_DIR" --target rrf_simulator -j

incorrect=0
correct=0
failed=0
tries=0

for ((i=1; i<=ITERATIONS; i++)); do
    if ! "$BUILD_DIR/rrf_simulator" \
        --vsd RRF/run/vsd \
        --gcode gcodes/draw_squares.gcode \
        --can-log logs/test_draw_squares2.csv \
        -c sys/config_hangprinter.g \
        > /dev/null 2>&1
    then
        # Treat failed run as incorrect
        ((failed++))
        ((tries++))
        printf "❌"
        continue
    fi

    if [[ ! -f "$BASE_LOG" || ! -f "$TEST_LOG" ]]; then
        ((failed++))
        ((tries++))
        printf "❌"
        continue
    fi

    diff_count=$(diff "$BASE_LOG" "$TEST_LOG" | wc -l)

    if [[ "$diff_count" -eq 4 ]]; then
        ((correct++))
        ((tries++))
        printf "✅"
    else
        ((incorrect++))
        ((tries++))
        printf "❌"
        break
    fi
done


# Summary: single line, wrapped in leading+trailing newline
echo
echo "Total runs: $tries, Correct: $correct, Incorrect: $incorrect, Failed: $failed"
