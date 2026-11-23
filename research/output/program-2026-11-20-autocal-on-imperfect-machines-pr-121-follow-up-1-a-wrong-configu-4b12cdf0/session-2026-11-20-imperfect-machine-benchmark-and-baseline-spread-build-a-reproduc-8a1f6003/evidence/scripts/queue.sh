#!/usr/bin/env bash
# Run imperfect-machine cases two at a time. Each stdin line: NAME ARGS...
# usage: queue.sh OUTPUT_ROOT < cases.txt   (run from the repository root)
root=$1
mkdir -p "$root"
grep -v '^\s*\(#\|$\)' | xargs -P 2 -L 1 bash -c '
  name=$0; [ -e "'"$root"'/$name" ] && exit 0
  .venv/bin/python -m autocal.tools.imperfect_machine "'"$root"'/$name" "$@" > "'"$root"'/$name.out" 2>&1
  echo "$name exit $?"'
