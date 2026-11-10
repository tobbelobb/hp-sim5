# Autocal validation, collection throughput and compiled Python physics

Status: complete.

Saved autocal and throughput evidence plus compiled-solver follow-up, updated 2026-10-08.

## Abstract

Recorded and synthetic HP3 experiments tested candidate validation, bounded adjustments and informative movements. A guarded veto and tie-breaker rejected impossible candidates in 15/15 deliberate counterexamples without changing the eleven recorded selections; no recorded HP3 accuracy gain was established. Bounded updates failed under winding mismatch and poor recorded starts. A production HP4 collection took 1693.12 seconds for twelve observations. Headless JS subsequently improved matched short recorded workloads by 19.03× and retained frozen-replay encoder agreement. A double-precision Warp CPU solver passed 99 fixtures and accelerated complete HP4 updates by 1.96×. Longer controlled comparisons used 20 simulated seconds per trial, three repetitions per HP3 backend and one per HP4 backend. CUDA took 37.24/168.61 seconds on HP3/HP4, versus CPU 26.87/105.95 and JS 1.27/6.63 seconds. Maximum sampled CUDA-to-JS encoder difference was 0.002161°, below the 0.01° comparison bound. Profiling identifies costly serial CUDA execution/readbacks and remaining Python systems. CUDA was slower for these single-world workloads. Fresh complete collection throughput, independent-world GPU batching and hardware accuracy remain unestablished. CPU-only JAX remains a deliberate prior performance choice.

## Topics

autocalibration, HP3, HP4, candidate-owned spool models, winding mismatch, elasticity, bounded updates, D-optimal movement, sentinel rejection, headless JS, Python parity, Rerun throughput, Warp CPU, CUDA, sequential XPBD, compact state arrays

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A common unseen sweep fixes history ranking | E01 / E03 | Not consistently supported | [Corrected audit](E03-selection-audit-corrected.json) |
| Candidate-owned models and complete coverage reject false validation | E07 | Supported on 15 counterexamples; recorded accuracy unchanged | [Stress results](E07-ranking-stress.json), [regressions](final-regressions.json) |
| Bounded residual updates improve geometry under model mismatch | E02 / E04 | Mixed; winding and poor starts defeat validation | [Synthetic](E02-E04-synthetic-results.json), [recorded](E04-recorded-results.json) |
| D-optimal selection chooses the best physical movement | E06 | Mixed; best in 2/3 starts | [Movement results](E06-movement-results.json) |
| Native production collection yields ingestible measurements | E05 | Supported for 12 HP4 observations; accuracy unestablished | [Collection summary](E05-native-summary.json) |
| Headless JS reduces runtime while retaining encoder parity | P01 / P02 / P03 | Supported on controlled workloads and frozen replay; fresh collection unmeasured | [Matrix](P02-matrix.json), [recorded replay](P03-final-recorded/replay.json) |
| Compiling the ordered solver accelerates full native physics | C02 / C03 | Supported; HP4 solver 7.90×, whole world 1.96×, still below realtime | [Repeated timing](compiled-solver/C03-timing.json), [fixture checks](compiled-solver/C02-final-focused.txt) |
| CUDA matches compiled CPU or JS speed | L01 / L02 | Rejected on the tested 20-s workload: 38.6% / 59.1% slower than CPU | [Complete timing](cuda-long-run/results.json) |
| CUDA solver matches targeted reference fixtures | C04B | Supported on five short fixtures; longer runs measured separately below | [CUDA comparisons](compiled-solver/C04-cuda-parity.txt) |
| Longer sampled encoder agreement stays within 0.01° | L01 / L02 | Supported at all 201 sampled observations; hardware accuracy unestablished | [Comparisons](cuda-long-run/results.json), [coverage](cuda-long-run/validation.json) |
| Serial CUDA execution/readbacks add cost | L03 | Supported by instrumented profiles; individual timings overlap | [CPU](cuda-long-run/L03-native-warp.txt), [CUDA](cuda-long-run/L03-native-warp-cuda.txt), [GPU intervals](cuda-long-run/L03-kernel-events.json) |

## References

- [Guarded validation and performance follow-up with limitations](report.md)
- [H1–H4, E01–E07 and P01–P04 budgets and decisions](research.md)
- [Separate physical and prediction errors](prediction-metrics.json)
- [Benchmark method, timings and reproducible replay commands](../../../../research/collection-performance.md)

- [Compiled Python solver methods, API, numerical and GPU limits](../../../../research/compiled-python-solver.md)
- [Warp CPU implementation](../../../../src/python/cable_joints_3d/warp_solver.py): the typed double-precision core and ECS adapter.

- [GPU visibility diagnosis](compiled-solver/C04-device-visibility.json): host RTX 3070 Ti exists; isolated `/dev` prevents this process from accessing it.

- [Unsandboxed CUDA detection](compiled-solver/C04-unsandboxed-diagnostic.txt): Warp access restored; JAX still lacks its CUDA plugin packages.

- [Long-run benchmark](cuda-long-run/benchmark.py), [measurement manifest](cuda-long-run/results.json), [repeatability](cuda-long-run/repeatability.json): frozen inputs, software/device provenance, timing and sampled consistency.
- [CPU-only JAX setup](../../../../autocal/README.md): existing dependency choice; user supplied its prior profiling rationale, not re-benchmarked here.
