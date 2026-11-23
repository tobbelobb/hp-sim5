#!/bin/bash
# Run imperfect-machine jobs two at a time: each line of $1 is "NAME ARGS...".
# Waits for this worktree's earlier harness runs first (shared 20-core host).
cd "$(git rev-parse --show-toplevel)"
while pgrep -f "imperfect_machine output/rb/" >/dev/null; do sleep 20; done
run() { name=$1; shift; [ -e output/rb/$name ] && { echo "skip $name"; return; }
  .venv/bin/python -m autocal.tools.imperfect_machine output/rb/$name "$@" > output/rb/$name.out 2>&1
  echo "done $name exit=$? $(grep -o '"stop_reason": "[^"]*"' output/rb/$name.out | head -1) $(grep -o '"max_abs_frame_error_mm": [0-9.]*' output/rb/$name.out | head -1)"; }
export -f run
grep -v '^#' "$1" | xargs -P 2 -L 1 bash -c 'run "$@"' _
echo QUEUE-FINISHED
