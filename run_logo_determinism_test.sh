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
BASE_LOG="$VSD_DIR/logs/Hangprinter_logo6.csv"
TEST_LOG="$VSD_DIR/logs/Hangprinter_logo6_2.csv"

echo "Building rrf_simulator once..."
cmake --build "$BUILD_DIR" --target rrf_simulator -j

incorrect=0
correct=0
failed=0
tries=0

for ((i=1; i<=ITERATIONS; i++)); do
    if ! "$BUILD_DIR/rrf_simulator" \
        --vsd RRF/run/vsd \
        --gcode gcodes/Hangprinter_logo6.gcode \
        --can-log logs/Hangprinter_logo6_2.csv \
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

# Check if failed > 0 (or incorrect > 0)
if [[ "$failed" -gt 0 || "$incorrect" -gt 0 ]]; then
    echo
    echo "--- Analyzing first failure/incorrect run (files ${TEST_LOG#$ROOT_DIR/} vs ${BASE_LOG#$ROOT_DIR/}) ---"

    # 1. Compare file lengths
    base_lines=$(wc -l < "$BASE_LOG")
    test_lines=$(wc -l < "$TEST_LOG")

    echo "Base log lines: $base_lines, Test log lines: $test_lines"

    if [[ "$test_lines" -lt "$base_lines" ]]; then
        echo "Verdict: $TEST_LOG is TOO SHORT compared to the base log."

        # Now, compare the last line of the short log to the corresponding line in the base log
        test_last_line=$(tail -n1 "$TEST_LOG")
        base_corresponding_line=$(sed -n "${test_lines}p" "$BASE_LOG")

        if [[ "$test_last_line" == "$base_corresponding_line" ]]; then
            echo "Last line comparison: The last line of the test log IS EQUAL to line $test_lines of the base log."
        else
            echo "Last line comparison: The last line of the test log IS NOT EQUAL to line $test_lines of the base log."
        fi

    elif [[ "$test_lines" -gt "$base_lines" ]]; then
        echo "Verdict: $TEST_LOG is TOO LONG compared to the base log."
    else
        echo "Verdict: File lengths are equal, but content still differs (diff count was not 4)."
    fi

    # 2. Compare last lines
    # This section remains for cases where file lengths are equal or the test log is too long
    if [[ "$test_lines" -ge "$base_lines" ]]; then
      base_last_line=$(tail -n1 "$BASE_LOG")
      test_last_line=$(tail -n1 "$TEST_LOG")

      echo "Base last line: '$base_last_line'"
      echo "Test last line: '$test_last_line'"

      if [[ "$test_last_line" == "$base_last_line" ]]; then
          echo "Last line comparison: Last lines ARE equal."
      else
          echo "Last line comparison: Last lines ARE NOT equal."
      fi
    fi
fi
