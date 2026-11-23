#!/usr/bin/env bash
# Paired imperfect-machine runs, two at a time. Each stdin line: ARM NAME ARGS...
# ARM base runs from BASE_TREE (338ded23, the integration branch), ARM home from this worktree.
# usage: BASE_TREE=.base (a short path: RRF rejects a long config path) queue.sh OUTPUT_ROOT < cases.txt   (run from the repository root)
root=$(realpath -m "$1"); here=$PWD
mkdir -p "$root"
grep -v '^\s*\(#\|$\)' | xargs -P 2 -L 1 bash -c '
  arm=$0; name=$1; shift; out="'"$root"'/$arm/$name"; [ -e "$out" ] && exit 0
  mkdir -p "'"$root"'/$arm"; tree="'"$here"'"; [ "$arm" = base ] && tree="'"$BASE_TREE"'"
  cd "$tree" && .venv/bin/python -m autocal.tools.imperfect_machine "$out" "$@" > "$out.out" 2>&1
  echo "$arm $name exit $?"'
