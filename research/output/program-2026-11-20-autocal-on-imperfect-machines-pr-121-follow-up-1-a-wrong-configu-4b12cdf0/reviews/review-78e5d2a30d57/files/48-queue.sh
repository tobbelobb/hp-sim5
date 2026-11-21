#!/bin/bash
# usage: queue.sh name args...   (runs one imperfect-machine HP3 autocal with the compliance probe)
cd /home/torbjorn/.t3/worktrees/hp-sim5/t3-cc036ed6-1b973479bb12c26c
name=$1; shift
HP_SIM5_COMPLIANCE_PROBE=2,3,1 .venv/bin/python -m autocal.tools.imperfect_machine output/compliance/live/$name --machine hangprinter_3 "$@" > output/compliance/live/$name.out 2>&1
echo "finished $name exit $?"
