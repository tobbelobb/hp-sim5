#!/bin/bash
# Run live HP3 cases two at a time. Each line on stdin: NAME SHIFT...  (e.g. "A_m10 A=-10")
cd "$(dirname "$0")/../../../../.." || exit 1
S=research/output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-uneven-low-anchor-heights-the-default-point-fit-assumes-all-low-8b2f2abb
xargs -P2 -L1 sh -c 'PYTHONPATH=. .venv/bin/python '"$S"'/scripts/e4_live.py "$0" "$@" > '"$S"'/runs/$0.out 2>&1'
