# Headless JavaScript physics profiling

Objective: add stable profiling and measure real HP3/HP4 physics before optimization.
Hypothesis P01: attachment update dominates sustained production JS physics.
Hypothesis P02: transient allocation and GC consume a material share of runtime.
Hypothesis P03: layering changes the dominant system and/or throughput.
Constraints: production 2 ms physics, original solver/motors, fixed commands; no physics optimization in this turn. Compare exact numerical checkpoints across repeats and profiler modes.
Budgets: 30,000-step logo prefixes (60 simulated seconds); complete frozen HP3/HP4 autocal traces; three uninstrumented repeats, one per-system, one CPU and one sampled-allocation run per case. Sequential runs only.
Evidence: inputs frozen under inputs/, per-run JSON, checkpoints, CPU and heap profiles; original source paths and SHA-256 in input manifest.
Status: harness and focused tests pass; matrix pending. Not hardware or fresh adaptive collector measurements.
Next decision: measure baseline, identify costs, then choose one isolated optimization in a later turn.

P01 early results: HP3 layered logo takes median 2.747 s for 30,000 updates. System shares: solver 37.8%, attachment 28.9%, over-correction 12.2%. Layering off takes median 2.191 s with the same motor commands and matching rebake. This contradicts an assumption that attachment alone is always the largest system.
P02 early results: layered HP3 logo sampled allocation estimates 9.74 GB / 324,632 bytes per update, while reported GC is roughly 1–2% of wall time. Native sampler includes objects collected during the run. Allocations are not retained heap and estimated bytes are not exact.
Autocal HP3: complete 488,271-step saved schedule, including 1,956 encoder reads, repeats exactly across three plain runs. Median physics time 39.219 s, 24.900x real time; maximum recorded encoder difference 1.14e-13 degrees. Observations consume 0.28% of replay wall time. Layered HP3 profiler comparisons also pass so far.
Status: remaining matrix runs active, all sequential. No physics optimization made.

P03: layering effect is workload-dependent. HP3 autocal is 39.219 s on versus 40.693 s off, unlike the logo speedup with layering off. Off produces a maximum 1,080-degree difference from the original layered trace. The off case is a repeatable fixed-command stress replay, not a faithful alternative calibration collection. No layering toggle is accepted as an optimization.
HP4 layered logo: 16.223 s median / 30,000 updates; solver 47.7%, attachment 25.4%, over-correction 10.4%. CPU self hotspots: applyConstraint 9.8%, getComponent 9.1%, length 7.7%. Sampled allocation estimates 51.676 GB over this trajectory, while reported GC remains around 1% of replay wall time. Position/orientation helpers and buildPlaneBasis are leading allocation sites.
HP4 layered autocal: 80,415 steps / 160.83 simulated seconds, median physics 43.541 s. Three plain repeats and system/CPU comparison pass. System proportions match the logo workload closely. Remaining allocation and layering-off autocal measurements are active.

Final status: complete. All eight cases, 48 measured trajectories and 7,544,232 measured physics updates pass exact numerical checkpoint comparison across repetitions and profiler modes. [Report](report.md) and [matrix](matrix.json) contain measurements. No physics optimization attempted. Next decision: select one isolated optimization and repeat baseline numerical/speed checks.

Final verification: 164 Jest tests / 28 suites and four native Node tests passed. JavaScript syntax and diff whitespace checks passed. Research catalog regenerated. Persistent summary: `research/headless-js-profiling.md`; reproducible recipe: `tests/benchmark/HEADLESS_JS_PROFILING.md`.
