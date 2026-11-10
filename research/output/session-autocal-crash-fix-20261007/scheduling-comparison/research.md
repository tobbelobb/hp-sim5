# Two-worker scheduling measurement

Objective: measure slowdown and native crash incidence of two-worker outer
scheduling on the full production regression battery. Overall crash-fix goal
remains open; this experiment does not redefine a workaround as a root fix.

Accepted user steering: run two-worker scheduling and measure completion time
and whether it remains crash-free. Normal GC and original dependencies.
No solver edits, dependency changes, collection, or robot motion.

Hypothesis: fewer concurrent JAX interpreters may reduce native crash incidence
while preserving all numerical results; its throughput cost is unknown.

Budget: four complete 11-dataset batteries, sequential order 11, 2, 2, 11.
Two matched pairs reverse order to reduce warmup/order bias. Each battery has
a 1,800-second limit; no retries, so failures remain visible. Stop to reassess
if a battery times out. Sampling interval 0.5 seconds. Record per-dataset exit,
duration, original fixture hash and artifact directory; total wall and CPU
time; peak process/thread/RSS samples; reference comparison separately.

The wrapper loads the unchanged runner with runpy and intercepts only its
outer executor and run_autocal observations. Solver children use ordinary
environment/GC and .venv/bin/python. Current source/fixture provenance is saved
in provenance.json. No production scheduling default is changed.

First eleven-worker battery completed: 105.332 seconds, 11/11 child exits zero,
no native failures, CPU time 1532.402 seconds (14.548 average CPU cores),
sampled peak 990 native threads and 11 calibration processes. Runner exit 1
is reference comparison mismatch (3 exact summaries, 8 changed), not a child
crash. Current numerical outputs will be compared directly across scheduling
settings to avoid mistaking historical-reference differences for a scheduler
effect. First six completed two-worker outputs were numerically identical in
final summary and recorded per-iteration anchors/radii/fit/rank metrics.

First two-worker battery completed: 281.409 seconds, 11/11 child exits zero,
no native failures; CPU time 1135.343 seconds, 4.034 average CPU cores, sampled
peak 180 native threads and 2 calibration processes. All 11 numerical outputs
(final summary and per-iteration anchors/radii/fit/rank metrics) match the
eleven-worker run exactly. First paired slowdown approximately 2.67x (+167%).
Neither arm crashed, so this pair cannot establish improved crash incidence.
Second pair is running in reverse order (2 then 11) as declared.

Second two-worker battery completed: 281.387 seconds, 11/11 child exits zero,
no native failures. Both two-worker batteries combined: 22/22 datasets completed
crash-free. CPU time 1133.382 seconds, 4.028 average CPU cores, verified peak
2 active calibration jobs. All numerical outputs from both two-worker runs
match the first eleven-worker run exactly. Final eleven-worker control is
running; finish it before reporting averaged throughput or crash-rate contrast.

Completed all four declared batteries. Eleven workers: 105.332 and 106.986 s;
two workers: 281.409 and 281.387 s. Mean slowdown 2.6507x (+165.07% elapsed;
62.27% lower throughput). Zero native failures at both settings (22 dataset
attempts each); all 44 child exits zero. All four complete numerical output
sets identical. Effective concurrency and fixture hashes verified. Aggregate
runner exits 1 are historical-reference comparisons, not child crashes.
Evidence archived beside tracked two-worker-scheduling-results.md, with SHA256
manifest. No production scheduling change. Mitigation reliability benefit
remains unproven; broad crash-fix goal remains open. No experiment still live.
