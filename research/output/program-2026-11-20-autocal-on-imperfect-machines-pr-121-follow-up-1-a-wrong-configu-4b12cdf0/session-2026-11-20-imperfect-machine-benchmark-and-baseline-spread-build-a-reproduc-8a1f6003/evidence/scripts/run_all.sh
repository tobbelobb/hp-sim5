#!/usr/bin/env bash
root=/home/torbjorn/.t3/worktrees/hp-sim5/t3-1736081a-8b681353f15e7d46
Q=research/output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-imperfect-machine-benchmark-and-baseline-spread-build-a-reproduc-8a1f6003/evidence/scripts/queue.sh
cd "$root" && $Q output/imperfect < ${Q%queue.sh}validate_cases.txt > output/validate.log 2>&1
cd "$root/output/wt-newfit" && $Q output/imperfect < newfit_allmachines_cases.txt > ../newfit_all.log 2>&1
echo done >> ../newfit_all.log
