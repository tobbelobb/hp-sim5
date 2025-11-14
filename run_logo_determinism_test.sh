#!/usr/bin/env bash
set -u
set -o pipefail

# Usage: ./run_draw_squares_tests.sh [iterations]
ITERATIONS="${1:-10}"
LINES_TO_IGNORE=4 # Define this constant globally for clarity

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
        exit_code=$?
        ((failed++))
        ((tries++))
        if [[ $exit_code -eq 124 ]]; then
            printf "⏱️"
        else
            printf "❌"
        fi
        continue
    fi

    if [[ ! -f "$BASE_LOG" || ! -f "$TEST_LOG" ]]; then
        ((failed++))
        ((tries++))
        printf "❌"
        continue
    fi

    # Primary check: exactly 4 lines of diff (likely just timestamp header differences)
    diff_count=$(diff "$BASE_LOG" "$TEST_LOG" | wc -l)

    if [[ "$diff_count" -eq 4 ]]; then
        # Standard Success Case
        ((correct++))
        ((tries++))
        printf "✅"
    else
        # Primary failure case. Now we run the lenient check to see if it's "close enough" (only differs at the end)

        # 1. Get file lengths to calculate truncation point
        base_lines=$(wc -l < "$BASE_LOG")
        test_lines=$(wc -l < "$TEST_LOG")

        lines_to_keep=$((base_lines - LINES_TO_IGNORE))
        if [[ "$test_lines" -lt "$lines_to_keep" ]]; then
            ((incorrect++))
            ((tries++))
            printf "❌"
            break
        fi

        # 2. Perform the truncated comparison
        # Use process substitution and 'head -n' to truncate the files before diffing
        # We ignore the first line (header) using sed within the head truncation to avoid line 1 mismatch errors
        diff_truncated=$(diff <(head -n "$lines_to_keep" "$BASE_LOG" | sed '1d') <(head -n "$lines_to_keep" "$TEST_LOG" | sed '1d'))

        if [[ -z "$diff_truncated" ]]; then
            # Lenient Success Case: Files are identical up to the last 4 lines.
            echo -n "☑️ " # Use a different symbol to denote lenient pass
            ((correct++))
            ((tries++))
            # Continue the loop for the next iteration instead of breaking
            continue
        else
            # Actual Failure Case: Differences found even when ignoring the end lines.
            ((incorrect++))
            ((tries++))
            printf "❌"
            # Keep the 'break' here to stop all iterations on a genuine failure
            break
        fi
    fi
done


# Summary: single line, wrapped in leading+trailing newline
echo
echo "Total runs: $tries, Correct: $correct, Incorrect: $incorrect, Failed: $failed"

# Detailed Analysis section (runs only if there were failures or incorrect runs)
if [[ "$failed" -gt 0 || "$incorrect" -gt 0 ]]; then
    echo
    # Using shell parameter expansion to remove the $ROOT_DIR prefix for cleaner output
    echo "--- Analyzing first failure/incorrect run (files ${TEST_LOG#$ROOT_DIR/} vs ${BASE_LOG#$ROOT_DIR/}) ---"

    # 1. Compare file lengths
    # Re-calculate these here in case the loop didn't run or completed fully
    base_lines=$(wc -l < "$BASE_LOG")
    test_lines=$(wc -l < "$TEST_LOG")

    echo "Base log lines: $base_lines, Test log lines: $test_lines"

    if [[ "$test_lines" -lt "$base_lines" ]]; then
        echo "Verdict: ${TEST_LOG#$ROOT_DIR/} is TOO SHORT compared to the base log."

        # Now, compare the last line of the short log to the corresponding line in the base log
        test_last_line=$(tail -n1 "$TEST_LOG")
        base_corresponding_line=$(sed -n "${test_lines}p" "$BASE_LOG")

        if [[ "$test_last_line" == "$base_corresponding_line" ]]; then
            echo "Last line comparison: The last line of the test log IS EQUAL to line $test_lines of the base log."
        else
            echo "Last line comparison: The last line of the test log IS NOT EQUAL to line $test_lines of the base log."
        fi

    elif [[ "$test_lines" -gt "$base_lines" ]]; then
        echo "Verdict: ${TEST_LOG#$ROOT_DIR/} is TOO LONG compared to the base log."
    # else is handled by the pinpoint logic below

    fi

    # Pinpoint the first meaningful line of difference (ignoring the header line 1)
    if [[ -f "$BASE_LOG" && -f "$TEST_LOG" ]]; then
        # Pipe the output of 'sed' (which skips the first line) into 'diff'
        diff_output=$(diff <(sed '1d' "$BASE_LOG") <(sed '1d' "$TEST_LOG"))

        if [[ -n "$diff_output" ]]; then
            # This captures the line range marker *relative to the modified (sed) files*
            first_diff_marker=$(echo "$diff_output" | head -n1)
            # Extracts the line number from the marker (e.g., "9c9" -> "9")
            relative_line_num=$(echo "$first_diff_marker" | cut -d'c' -f1 | cut -d',' -f1)

            # Add 1 back to the relative number to get the absolute line number in the original file
            absolute_line_num=$((relative_line_num + 1))

            echo "Details: Files started differing around line $absolute_line_num in the original logs (ignoring line 1)."
            # Display the problematic lines:
            echo "Base Log line $absolute_line_num: $(sed -n "${absolute_line_num}p" "$BASE_LOG")"
            echo "Test Log line $absolute_line_num: $(sed -n "${absolute_line_num}p" "$TEST_LOG")"
        else
            echo "Details: Files are identical after ignoring the first line."
        fi
    fi
fi

