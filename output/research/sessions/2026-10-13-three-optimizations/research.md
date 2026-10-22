# Cumulative validation of three headless physics optimizations

Objective: assess numerical equivalence and throughput for the cumulative main-branch changes: per-update ECS component-store references, reusable attachment-transform scratch destinations, and attachment/tangent preparation reuse.

Hypothesis: all eight frozen HP3/HP4 logo and autocal workload/settings cases preserve exact 1,000-step checkpoint hashes; cumulative unprofiled median physics time is lower than the original headless profiling baseline.

Constraints: fixed inputs from the original headless profiling session; `--compare` exact checkpoint equality; Node profiling runner defaults (5,000 warmup steps, three fresh measured repeats); no concurrent simulation/test jobs during timings. Benchmarks measure fixed prefixes and physics batch time only, not hardware or adaptive collector performance.

Experiment CUM-1 completed: all eight three-repeat workload comparisons passed exact checkpoints. Median reductions were 6.7–16.0%, with a 9.0% median across cases. The full report and raw matrix are in [report.md](report.md) and [cumulative-matrix.json](cumulative-matrix.json).

Next decision: retain the three optimizations; their cumulative numerical and throughput evidence supports inclusion for these workloads. Future timing changes should compare against this cumulative source state, not add isolated percentages.
