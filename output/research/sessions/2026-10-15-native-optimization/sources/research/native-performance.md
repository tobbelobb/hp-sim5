# Native versus Wasm performance, 2026-10-15

The default native build now uses profile-guided compilation, and both Rerun
recorders batch raw telemetry with `send_columns()`. Matched HP3/HP4 runs use the
fuller **Wasm runtime** as the baseline. Three sequential repetitions preserve
commands, fixed timesteps, observation cadence and recorded values.

With Rerun enabled, the final native pipeline measures:

| Workflow | HP3 speedup | HP4 speedup |
| --- | ---: | ---: |
| Continuing world, 10,000 steps | 2.34× | 2.33× |
| Fresh trial, 200 steps | 3.01× | 2.85× |
| Standalone autocal service, 10,000 steps | 3.83× | 2.74× |
| Real six-point RRF collection | 1.73× | 1.93× |

Timers include recording finalization. Fresh trials include startup; the other
rows exclude service construction. Real collections use the actual RRF planner,
bridge and six-point collector. HP4 collection falls from **19.27 to 10.00 s**;
HP3 falls from **7.96 to 4.59 s**. These are simulated collection runs, not full
optimizer/fitting or calibration-accuracy results.

Most of the recorded saving comes from logging, which also speeds up Wasm.
With the same optimized logger, Wasm standalone service medians are 0.881/2.017 s
(HP3/HP4), versus native 0.837/1.819 s. Native execution adds a smaller gain to
that shared improvement. No-record real collections improve 1.05×/1.14×.

Physics-only throughput improves **1.25–1.31×** across eight 30,000-step
logo/autocal prefixes with layering on/off. Full autocal replays improve
**1.39× HP3** (12.81 → 9.24 s, 488,271 steps) and **1.27× HP4**
(9.23 → 7.26 s, 80,415 steps). PGO adds 7–18% throughput over the previous native
build on these workloads. Rust physics equations and step counts are unchanged.

All final native checkpoint hashes match the prior native engine; 186,366
audited RRD component rows retain the same clocks and values. All 42 real
collections retain exact selected measurements and step counts. Existing long
native/Wasm floating-point differences remain; these results do not claim
bitwise equivalence between targets. Verification passes 113 Python and 121 Node
checks, three untrained build modes and the production browser build.

`npm run build:native` uses PGO for release/profiling builds and needs
`rustup component add llvm-tools-preview`. `--no-pgo` selects an untrained build;
explicit CPU flags are respected. The profiler still selects `--solver
native|wasm|js` and an exact `--native-addon`; it never rebuilds or substitutes
an engine. [Build and profiling controls](rust-native-engine.md).

- [Full report](../output/research/sessions/2026-10-15-native-optimization/report.md)
- [Paired timings and all repetitions](../output/research/sessions/2026-10-15-native-optimization/summary.json)
- [Numerical and recording audits](../output/research/sessions/2026-10-15-native-optimization/final-recording-audit.json)
- [Source/artifact manifest](../output/research/sessions/2026-10-15-native-optimization/source-manifest.json)
- [Previous study](../output/research/sessions/2026-10-14-native-throughput/report.md)
