# Two-worker scheduling: measured cost and crash observations

October 7, 2026. Four complete regression batteries ran from approximately
15:56 to 16:09 CEST, using the current checkout and original dependencies.

**Two workers took 2.65 times as long as eleven workers. Both settings were
crash-free in this sample. These results do not establish a reliability benefit
from limiting concurrency. No production scheduling default was changed.**

## Results

Each battery attempted all 11 datasets, using fresh fixture copies. All 44
calibration subprocesses exited zero and generated complete logs. None timed
out, segfaulted or aborted.

| Run order | Concurrent dataset workers | Total elapsed time | Completed datasets | Native crashes |
| --- | ---: | ---: | ---: | ---: |
| 1 | 11 | 105.332 s | 11/11 | 0 |
| 2 | 2 | 281.409 s | 11/11 | 0 |
| 3 | 2 | 281.387 s | 11/11 | 0 |
| 4 | 11 | 106.986 s | 11/11 | 0 |

The second pair reversed the first pair's order. Matched-pair slowdowns were
2.67x and 2.63x. Mean completion times were:

- **Eleven workers: 106.159 seconds**, approximately 1 minute 46 seconds.
- **Two workers: 281.398 seconds**, approximately 4 minutes 41 seconds.
- **Cost: 175.239 extra seconds per battery**, 165% longer elapsed time and
  approximately 62% less completed-battery throughput.

Measured process CPU time divided by elapsed time averaged **14.45 CPU cores**
with eleven workers and **4.03 CPU cores** with two. This machine exposed 20
logical CPUs. Two workers did not use the available compute as fully in these
runs. Total CPU time averaged 1,534.0 versus 1,134.4 seconds per battery,
respectively: reduced contention used less CPU time, but completion was slower.

Sampling every 0.5 seconds observed peaks of **990 native threads** with eleven
workers and **180** with two. The maximum sum of descendant process RSS was
approximately **8.65 GiB** versus **2.02 GiB**. These are sampled process-RSS
sums, which can count shared pages multiple times, not unique physical memory.

## Numerical results and runner exit status

All four runs produced identical recorded final anchors, radii and fit score,
and identical per-iteration anchors, radii, fit score, rank score and history
rank score for every dataset. The two-worker scheduling change did not alter
these numerical outputs.

Each aggregate regression command returned **1**, because three dataset
summaries matched the historical references exactly and eight exceeded the
reference tolerance. The comparison pattern was the same in every run.
These reference mismatches are separate from child failures: every calibration
subprocess returned **0**. The batteries must not be described as passing all
reference comparisons.

## Method and scope

The unchanged `autocal/tools/regress_calibration_logs.py` ran with:

```text
--no-fail-score-mismatch --keep-going --color never
```

A temporary benchmark wrapper loaded the runner with `runpy`, capped only its
outer `ThreadPoolExecutor`, and observed dataset starts, finishes and exits.
It verified peak active calibration counts of exactly 11 and 2. No solver
concurrency was changed. Calibration children used `.venv/bin/python`, normal
GC and the runner's usual BLAS/OpenMP thread defaults. The wrapper removed
`PYTHONPATH`, `LD_PRELOAD` and `PYTHONMALLOC`; no GC workaround, custom signal
hook, XLA flag experiment or allocator variation was introduced.

Wall times include child startup and normal regression comparison work.
CPU time comes from child-process resource usage, including waited-for
descendants. Process/thread/RSS peaks were sampled rather than continuously
observed. Background machine load was not controlled, though the reversed
order and closely repeated timings support the measured difference.

Source revision: `054362b79f13d429399aff75e3bebba08efa9d8f`.
Python 3.12.3; JAX/jaxlib 0.9.2; NumPy 2.4.3; SciPy 1.17.1.
The production runner and original fixtures were verified unchanged by hash
after the experiment. No dependencies were installed or changed.

## Interpretation

Two-worker scheduling completed **22/22 dataset attempts without native
failure**, across two full batteries. Eleven-worker scheduling also completed
**22/22 without native failure**. The intermittent bug was not reproduced in
either arm, so this experiment cannot show that two workers prevent it or
reduce its incidence.

The slowdown is substantial and repeatable in this sample. On this evidence,
there is no demonstrated reliability gain to justify replacing the production
default with two workers. The underlying crash remains unresolved; this
experiment measures a possible mitigation without establishing a fix.

## Preserved evidence

Small evidence files and complete aggregate runner outputs are copied into
[evidence/scheduling-comparison/](evidence/scheduling-comparison/), so they
survive loss of the ignored working directory:

- [summary.json](evidence/scheduling-comparison/summary.json): all four timings,
  CPU measurements, sampled peaks and failure counts.
- [analysis.json](evidence/scheduling-comparison/analysis.json): effective
  concurrency, fixture verification and numerical comparisons.
- [provenance.json](evidence/scheduling-comparison/provenance.json): source,
  interpreter, versions, environment and fixture hashes.
- Each run directory contains `datasets.json`, `dataset-events.jsonl`,
  `summary.json` and `runner.txt`, with per-dataset durations, exit codes and
  scratch artifact paths.
- `benchmark_schedule.py` and `analyze_results.py` are snapshots of the temporary
  diagnostic scripts. Their original location was
  `output/research/autocal-crash-fix-20261007/scheduling-comparison/`; their
  relative root discovery assumes that directory depth.
- [manifest.json](evidence/scheduling-comparison/manifest.json) records SHA-256
  hashes of the copied evidence.

Full child console outputs and original generated calibration logs remain
under the ignored working directory and the scratch paths recorded in the
dataset evidence. The preserved aggregate outputs and parsed comparisons
record the conclusions above; they do not replace every original child log.
