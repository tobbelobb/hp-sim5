# Replay inputs and commands

Run from the repository root at `c5449c61` or later. The local interpreter was Python 3.12.3 with NumPy 2.4.3 and SciPy 1.17.1 on x86_64. Headless runs used Node 24.14.1 and the firmware/native physics artifacts for `7a57be78`. Replays use one BLAS thread.

[inputs.tsv](inputs.tsv) lists the file SHA-256, size in bytes, run name and source. `git` rows use `revision:path`. Extract them into `RAW/inputs/<run>/<filename>` with `git show`. Extract both the input and its companion imperfection file. The refs `t3/autocal-force-realisation` at `0ee10c68` and `t3/default-home-stretch-fit` at `2b086a3c` contain the exact compressed inputs. The older references are tracked on the production base.

New raw runs are under `output/research/force-fitting-b5e5ea67` in the main local hp-sim5 checkout. These files require access to that host. The git-sourced recordings and new raw simulator data were kept out of this change. The fresh HP3 prefix used the first five profiled sweeps in collection order.

```bash
RAW=/home/torbjorn/repos/hp-sim5/output/research/force-fitting-b5e5ea67
SCRIPT=research/output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-production-force-realisation-fitting-reuse-blind-period-scan-and-b5e5ea67/evidence/scripts
export OPENBLAS_NUM_THREADS=1 PYTHONPATH=.

.venv/bin/python -m autocal.tools.imperfect_machine "$RAW/live-hp3-2x-f137-s42" --seed 42 --cogging-scale 2 --cogging-freq 137
.venv/bin/python -m autocal.tools.imperfect_machine "$RAW/live-hp5r-1p5x-f137-s43" --machine hangprinter_5_pulley_reduced --seed 43 --cogging-scale 1.5 --cogging-freq 137
.venv/bin/python -m autocal.tools.imperfect_machine "$RAW/live-hp3-0x-s42" --seed 42 --cogging-scale 0

.venv/bin/python "$SCRIPT/production.py" "$RAW"/inputs/hp3-s? "$RAW/inputs/hp3-nominal"
.venv/bin/python "$SCRIPT/production.py" --prefix 5 "$RAW/live-hp3-2x-f137-s42"
.venv/bin/python "$SCRIPT/production.py" "$RAW/snapshots/live-hp3-2x-f137-s42-prefix" "$RAW/live-hp3-2x-f137-s42"
.venv/bin/python "$SCRIPT/production.py" "$RAW"/inputs/hp5r-s[57] "$RAW/inputs/hp5r-f137-s5"
for mode in double no-ripple offset second-harmonic; do
    .venv/bin/python "$SCRIPT/production.py" --mode "$mode" "$RAW"/inputs/hp3-s[1479]
done
.venv/bin/python "$SCRIPT/production.py" --mode missing-angle "$RAW/inputs/hp3-s1"
.venv/bin/python "$SCRIPT/production.py" --prefix 4 "$RAW/inputs/hp3-s1"
.venv/bin/python "$SCRIPT/production.py" autocal/data/references/{benchmark,noisy_0p05mm_benchmark,noisy_0p2mm_benchmark,noisy_0p5mm_benchmark,disturbed_benchmark}_923c2baa_hangprinter_3.json

for variant in full differential fixed; do
    .venv/bin/python "$SCRIPT/replay.py" --variant "$variant" "$RAW"/inputs/hp3-s? "$RAW/inputs/hp3-nominal"
done
.venv/bin/python "$SCRIPT/replay.py" "$RAW"/inputs/hp5r-s[056789] "$RAW/inputs/hp5r-f137-s5" "$RAW"/inputs/hp5r-s[12]-ea1.2 "$RAW"/inputs/hp5r-s[34]-ea0.8
.venv/bin/python "$SCRIPT/replay.py" --variant fixed "$RAW"/inputs/hp5r-s[056789] "$RAW/inputs/hp5r-f137-s5"
.venv/bin/python "$SCRIPT/replay.py" "$RAW"/inputs/hp5r-cog2-s[56]
.venv/bin/python "$SCRIPT/replay.py" "$RAW/snapshots/live-hp5r-1p5x-f137-s43-rough" "$RAW/live-hp5r-1p5x-f137-s43"
.venv/bin/python "$SCRIPT/replay.py" --mode extra "$RAW"/inputs/hp5r-s[05679] "$RAW/inputs/hp5r-f137-s5"
for mode in offset off200 no-ripple; do
    .venv/bin/python "$SCRIPT/replay.py" --mode "$mode" "$RAW"/inputs/hp5r-s[057] "$RAW/inputs/hp5r-f137-s5"
done
```

The first three commands create new run directories and should use a new raw root if these runs already exist. The archived simulator measurements are independent of the replay choices. The synthetic labels preserve the original motion and encoder phases. The added 200/turn HP5r tone uses seed 540 for phase per motor; the HP3 extra tone uses fixed phase 1 rad. No synthetic geometry was recorded as a simulator run.
