#!/bin/bash

set -e

SIMULATOR="./build/rrf_simulator"
VSD_DIR="run/vsd"
GCODE="gcodes/draw_squares.gcode"
CONFIG="sys/config_hangprinter.g"
LOG1="run/vsd/logs/test_determinism_run1.jsonl"
LOG2="run/vsd/logs/test_determinism_run2.jsonl"

echo "Testing determinism of RRF simulator..."
echo ""

# Clean up old logs
rm -f "$LOG1" "$LOG2"

# Run 1
echo "Running simulation #1..."
timeout 70 $SIMULATOR --vsd "$VSD_DIR" --gcode "$GCODE" --can-log "$LOG1" -c "$CONFIG" > /dev/null 2>&1
if [ ! -f "$LOG1" ]; then
    echo "ERROR: First run did not produce log file"
    exit 1
fi

# Run 2
echo "Running simulation #2..."
timeout 70 $SIMULATOR --vsd "$VSD_DIR" --gcode "$GCODE" --can-log "$LOG2" -c "$CONFIG" > /dev/null 2>&1
if [ ! -f "$LOG2" ]; then
    echo "ERROR: Second run did not produce log file"
    exit 1
fi

# Compare logs, skipping the first line (which contains timestamp)
echo "Comparing outputs (skipping timestamp line)..."
DIFF_OUTPUT=$(tail -n +2 "$LOG1" > /tmp/log1_no_timestamp.jsonl && \
              tail -n +2 "$LOG2" > /tmp/log2_no_timestamp.jsonl && \
              diff -u /tmp/log1_no_timestamp.jsonl /tmp/log2_no_timestamp.jsonl || true)

if [ -z "$DIFF_OUTPUT" ]; then
    echo ""
    echo "✓ SUCCESS: Outputs are identical!"
    echo "  Lines in each output: $(wc -l < /tmp/log1_no_timestamp.jsonl)"
    exit 0
else
    echo ""
    echo "✗ FAILURE: Outputs differ!"
    echo ""
    echo "First 50 lines of differences:"
    echo "$DIFF_OUTPUT" | head -50
    echo ""
    echo "Total differing lines: $(echo "$DIFF_OUTPUT" | grep -c '^[+-]' || echo 0)"
    exit 1
fi
