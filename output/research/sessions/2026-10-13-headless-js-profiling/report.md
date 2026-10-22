# Headless JavaScript physics profiling

Status: complete.

## Abstract

The production headless JavaScript HP3/HP4 physics was measured on fixed firmware logo trajectories and complete recorded autocal movement/settling schedules, with layering enabled and disabled. Eight cases used three uninstrumented repetitions and separate system-timing, native V8 CPU and allocation-sampling runs: 48 measured trajectories and 7,544,232 physics updates. Every profiler mode matched its baseline numerical checkpoints exactly. The constraint solver, attachment updates and over-correction dominate runtime. Allocation is substantial, but reported GC contributes about 1–2% of replay time. Layering removal speeds the logo workload but gives little benefit on the autocal schedules and can change the physical trajectory substantially. The faithful HP3 autocal replay also reproduces 1,956 saved encoder reads to floating-point rounding. These results establish a reproducible baseline; no physics optimization was made. Measurements exclude firmware, transport, rendering and recording costs, and frozen settling schedules cannot establish adaptive collector or hardware performance.

## Baseline throughput

Host: 12th Gen Intel(R) Core(TM) i7-12700K; linux 6.8.0-146-generic x64; Node v24.14.1, V8 13.6.233.17-node.44. Runs were sequential. Each process warmed a separate world for 5,000 steps, then used fresh worlds for all measured runs. Plain-run physics time includes GC during update batches and excludes construction, command delivery and observations.

| Machine | Workload | Layering | Steps | Simulated seconds | Physics seconds (3 runs) | Median seconds | Real-time factor | Median GC % | Estimated allocation/update |
| --- | --- | --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| HP3 | logo | on | 30,000 | 60.000 | 2.747, 2.729, 2.789 | 2.747 | 21.84× | 1.62 | 317 KiB |
| HP3 | logo | off | 30,000 | 60.000 | 2.191, 2.157, 2.229 | 2.191 | 27.38× | 1.72 | 258 KiB |
| HP3 | autocal replay | on | 488,271 | 976.542 | 39.516, 38.967, 39.219 | 39.219 | 24.90× | 1.03 | 292 KiB |
| HP3 | autocal replay | off | 488,271 | 976.542 | 41.074, 40.670, 40.693 | 40.693 | 24.00× | 0.94 | 306 KiB |
| HP4 | logo | on | 30,000 | 60.000 | 16.223, 16.152, 16.619 | 16.223 | 3.70× | 0.82 | 1682 KiB |
| HP4 | logo | off | 30,000 | 60.000 | 12.699, 12.264, 12.280 | 12.280 | 4.89× | 1.05 | 1259 KiB |
| HP4 | autocal replay | on | 80,415 | 160.830 | 44.587, 43.414, 43.541 | 43.541 | 3.69× | 0.78 | 1652 KiB |
| HP4 | autocal replay | off | 80,415 | 160.830 | 43.215, 44.225, 43.765 | 43.765 | 3.67× | 0.77 | 1652 KiB |

Allocation totals are statistical estimates from a separate 32 KiB sampling run, retaining samples for objects collected by minor/major GC. They include replay observations, whose separately measured time is small. They describe bytes allocated over time, not retained heap. GC percentages use Node performance-hook event durations; concurrent collector CPU work is not fully represented. Raw JSON includes GC counts/kinds, process CPU time and before/after memory usage. Instrumented runs are for attribution, not speed claims.

## Per-system cost

Percentages below divide system totals by physics time in the separate system-timing run. The JSON contains every one of the 19 production systems, call counts, totals, averages and maxima.

| Case | Constraint solver | Attachment update | Over-correction | Friction | Rigid-body sync |
| --- | ---: | ---: | ---: | ---: | ---: |
| hp3-logo-on | 37.8% | 28.9% | 12.2% | 3.6% | 2.4% |
| hp3-logo-off | 31.9% | 33.3% | 9.3% | 4.4% | 2.9% |
| hp3-autocal-on | 35.9% | 30.9% | 10.9% | 4.0% | 2.6% |
| hp3-autocal-off | 38.6% | 28.6% | 11.7% | 3.9% | 2.5% |
| hp4-logo-on | 47.7% | 25.4% | 10.4% | 3.1% | 4.2% |
| hp4-logo-off | 37.5% | 33.1% | 7.5% | 4.1% | 5.7% |
| hp4-autocal-on | 48.1% | 25.4% | 9.9% | 3.1% | 4.2% |
| hp4-autocal-off | 48.8% | 24.6% | 9.6% | 3.2% | 4.2% |

The solver is the largest system in seven cases; unlayered HP3 logo has attachment updates slightly ahead. Friction and the scheduled rigid-body sync system are smaller. Rigid-body position/orientation helpers also execute inside other systems, so their total cost is larger than the dedicated sync-system row.

## V8 CPU and allocation evidence

CPU samples are filtered to measured replay intervals, excluding startup, construction and warmup. They include observations and command arrival handling. Inclusive stack time overlaps between callers; self time does not. Raw profiles retain all call trees.

| Case | Five hottest functions by CPU self time | Sampled allocation total |
| --- | --- | ---: |
| hp3-logo-on | `getComponent` 8.4%, `applyConstraint` 6.7%, `_updateAttachmentPoints` 5.8%, `calculateAttachmentPoints` 5.8%, `length` 5.4% | 9.74 GB |
| hp3-logo-off | `getComponent` 8.1%, `length` 7.1%, `_updateAttachmentPoints` 6.3%, `calculateAttachmentPoints` 6.1%, `applyConstraint` 5.2% | 7.92 GB |
| hp3-autocal-on | `getComponent` 7.0%, `length` 6.7%, `_updateAttachmentPoints` 6.6%, `applyConstraint` 6.3%, `calculateAttachmentPoints` 6.3% | 145.91 GB |
| hp3-autocal-off | `getComponent` 9.2%, `applyConstraint` 7.1%, `length` 7.0%, `calculateAttachmentPoints` 6.3%, `_updateAttachmentPoints` 6.1% | 152.99 GB |
| hp4-logo-on | `applyConstraint` 9.8%, `getComponent` 9.1%, `length` 7.7%, `getEntityWorldOrientation` 4.7%, `getEntityWorldPosition` 4.4% | 51.68 GB |
| hp4-logo-off | `getComponent` 8.5%, `length` 7.5%, `applyConstraint` 6.6%, `getEntityWorldOrientation` 5.7%, `getEntityWorldPosition` 5.5% | 38.69 GB |
| hp4-autocal-on | `applyConstraint` 10.2%, `getComponent` 8.8%, `length` 7.5%, `getEntityWorldPosition` 5.2%, `getEntityWorldOrientation` 4.4% | 136.00 GB |
| hp4-autocal-off | `applyConstraint` 10.5%, `getComponent` 9.1%, `length` 7.9%, `getEntityWorldPosition` 4.7%, `getEntityWorldOrientation` 4.6% | 136.02 GB |

`applyConstraint` executes inside `PBDCableConstraintSolver`. `calculateAttachmentPoints` is shared by attachment updates and over-correction. `getComponent`, vector length calculations and world-position/orientation transforms are frequent shared costs. The leading allocation sites include `getEntityWorldPosition`, `getEntityWorldOrientation`, ECS `query`, `calculateAttachmentPoints`, `buildPlaneBasis` and inertia transforms. The profile stacks identify their callers; allocation size alone does not establish the benefit of changing a function.

## Numerical behavior and workload limits

All three plain repeats and every separate profiler mode matched exactly within each case, at 1,000-step full numerical checkpoints and available recorded encoder observation times. Checkpoints include frame poses, motor targets/encoders and missed-step diagnostics, velocities, cable segments/lengths/forces and queue length. Profiles are accepted only after matching the same scene/input/settings/step/checkpoint/warmup contract. Comparisons across layering settings intentionally have different physical inputs.

The HP3 autocal trace contains 488,271 steps, 35,959 movement commands, 311 torque-mode changes and 593 position-mode changes. Its first movement command arrives at step 330,770; the preceding settling and torque activity are preserved. The layered replay reproduces all 1,956 saved encoder reads with maximum difference 1.14e-13 degrees. The unlayered replay has maximum difference 1,080 degrees from those reads. It is a repeatable fixed-command stress replay, not a faithful replacement collection. Its slightly slower timing cannot be treated as the overhead of layering alone.

The HP4 autocal trace contains 80,415 steps, 4,035 movement commands, 41 torque-mode changes and 91 position-mode changes. The historical event format has no explicit encoder-response records, so the harness validates repeat/profiler numerical equality but does not claim old-trace encoder parity for HP4. The saved movement and settling schedule is held fixed; a fresh collector could choose different waits under changed layering or physics.

Logo inputs are the first 30,000 scheduled steps of each machine's existing layered RRF logo CAN stream, produced by the production firmware player and matching motor-axis configuration. The same motor commands are used for the layering-on/off cost comparison. HP3 and HP4 are distinct trajectories and physical designs, not interchangeable workloads. Input origins and hashes are preserved.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Attachment is always the largest system | P01: eight system profiles | Rejected | Solver leads seven of eight cases |
| Temporary allocation is substantial | P02: eight native allocation profiles | Supported | Hundreds of KiB to more than 1 MiB allocated/update |
| GC is the dominant current bottleneck | P02: GC events and CPU profiles | Rejected | Reported GC approximately 1–2%; solver/attachment math dominates |
| Disabling layering consistently speeds this simulation | P03: all on/off pairs | Mixed | Logo improves; autocal changes little, with physical divergence |
| The profiling modes preserve numerical checkpoints | P04: 48 measured trajectories | Supported | Exact checkpoint hashes within every case |

## Reproduce and continue

See [the profiling recipe](../../../../tests/benchmark/HEADLESS_JS_PROFILING.md). The retained [matrix runner](run_matrix.py) invokes only native Node profilers and the production world harness. [Input origins](inputs/origins.json), frozen input files and [autocal workload counts](autocal-workloads.json) preserve the actual schedules.

Next: choose one isolated code optimization, repeat the same plain runs with `--compare`, inspect numerical checkpoint equality and timing distributions, then reprofile. No physics optimization was attempted in this investigation. Small timing differences need more repetitions before acceptance.

## References

- [All numerical/timing/profile summaries](matrix.json): eight cases and raw artifact paths.
- [Research notebook](research.md): hypotheses, constraints, progress and steering.
- [Frozen source and dependency evidence](source/): working physics/headless diff, harness, summary reader, dependency versions and lockfile.
- Baseline Git revision `854430c6c2731eb5c85eb34c305213c2419a28a1`; actual JS source SHA-256 `bd4847f81f25f5f7e1ff26d3f7d71f6950ab0576bf6dd59c03a3e9d1a2a02566`. All 32 result files share this source hash.
- Original HP3 autocal schedule: `output/research/autocal-comfortable-force-20261010/isolated.headless_1/events.jsonl`.
- Original HP4 autocal schedule: `output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/P03-final-numeric/events.jsonl`.
- [Node CPU profiler](https://nodejs.org/api/cli.html#--cpu-prof), [GC performance hooks](https://nodejs.org/api/perf_hooks.html) and [V8 sampled allocations](https://chromedevtools.github.io/devtools-protocol/tot/HeapProfiler/#method-startSampling): native attribution mechanisms.

## Verification

164 Jest tests across 28 production HP3/HP4 and solver/friction/over-correction suites passed; four native Node replay/profile tests passed. JavaScript syntax checks and `git diff --check` passed. [Jest log](verification-jest.log), [Node log](verification-node.log).
