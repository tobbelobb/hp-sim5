# Native PGO and columnar Rerun performance, 2026-10-15

Status: complete.

This study compares native headless simulation with the fuller Rust/Wasm runtime, using the same Rust physics code, commands, fixed timesteps and output cadence. Raw column buffering replaces per-observation Rerun object construction in both recorders, and optimized native builds now use deterministic profile-guided compilation with explicit opt-out and artifact selection. Three sequential repetitions per case measure HP3/HP4, recording on/off, fresh and continuing worlds, the standalone autocal service and real six-point RRF collections, including finalization. Recorded runs improve 1.73–3.83× over Wasm with the earlier logging; most of that gain is logging and also benefits Wasm. Native physics improves 1.25–1.31× on eight 30,000-step prefixes and 1.39×/1.27× on full HP3/HP4 autocal replays. Native checkpoint hashes and audited RRD values remain unchanged, and all 42 real collections retain exact selected measurements and step counts. These measurements exclude full optimizer/fitting runs and do not establish calibration accuracy.

## Implementation

`ColumnarRecording` accumulates detached raw component fields and constructs Rerun columns once per batch. Both `RerunSystem` and `FlightRecording` use it for poses, geometry and scalar telemetry; rare styles, provenance and clears keep row logging. Limits are 128 observations or 65,536 component instances, checked at sample boundaries, with a one-second active flush. Live flight viewers also flush partial batches while idle. Owners flush before resets, removals, file rotation and finalization. Direct RerunSystem callers must flush it before their RecordingStream. The physical timelines are explicit; column writes omit the SDK's automatic ingestion `log_time`/`log_tick`. Sample cadence, signal names and values are preserved. Native collection/fresh trials reuse the recorder snapshot for numerical telemetry instead of building it twice. Fresh simulation_wall_s includes deferred column construction.

Optimized native builds train for 40,000 deterministic HP3/HP4 updates over layering on/off, position/torque modes and closed-loop transitions, then recompile with PGO. Host CPU tuning is the default unless an explicit target-cpu flag is supplied. `--no-pgo` preserves the untrained recipe; dev defaults to untrained. LLVM tools are required for PGO, with an explicit install/opt-out error. There is no silent fallback. Metadata records artifact, flags, profile hash, trainer hash and baked training scenes; default freshness checks cover the trainer. Profiling still chooses native/Wasm/JS and an exact native artifact without rebuilding. No Rust physics equations, solver iterations or timesteps changed.

## Matched end-to-end results

Ratios are Wasm elapsed time divided by native elapsed time. Each cell is the median of three fresh worlds/processes. Both sides use the same prepared scene, commands and clock. Continuing/service/collection timers exclude startup and include owner finalization; fresh trials include startup, every-step numerical telemetry and finalization. Recording policy is 10 Hz plus encoder observations for continuing/collection, every step for fresh trials, and stride 10 / 50 Hz for the standalone service.

| Workflow | Rerun | HP3 Wasm → native seconds | HP3 speedup | HP4 Wasm → native seconds | HP4 speedup |
| --- | --- | ---: | ---: | ---: | ---: |
| Continuing world, 10,000 steps | Off | 0.519 → 0.447 | 1.16× | 1.732 → 1.396 | 1.24× |
| Continuing world, 10,000 steps | On | 1.622 → 0.693 | 2.34× | 4.469 → 1.918 | 2.33× |
| Fresh trial, 200 steps | Off | 0.309 → 0.276 | 1.12× | 0.684 → 0.610 | 1.12× |
| Fresh trial, 200 steps | On | 1.455 → 0.483 | 3.01× | 3.553 → 1.247 | 2.85× |
| Standalone autocal service, 10,000 steps | Off | 0.414 → 0.346 | 1.19× | 1.318 → 0.998 | 1.32× |
| Standalone autocal service, 10,000 steps | On | 3.208 → 0.837 | 3.83× | 4.984 → 1.819 | 2.74× |
| Real six-point RRF collection | Off | 4.093 → 3.902 | 1.05× | 8.588 → 7.534 | 1.14× |
| Real six-point RRF collection | On | 7.960 → 4.590 | 1.73× | 19.274 → 9.995 | 1.93× |

Real collections use the production RRF planner, bridge and collector, unlayered stock HP3/HP4, fixed anchors [2,3], drive 0, sensor 1, three points per direction and four noise samples, with default forces and preload. Each isolated supervisor passed its preflight; command-context socket probing passed separately. Timers include 100 ms status polling and recording rotation/finalization, but exclude service construction/preflight. All jobs finish within the 120-second / 100,000-step bound. This is one six-point collection configuration, not full calibration optimization.

## Separating logging from native execution

The pre-PGO native build with columnar logging already improved recorded continuing worlds 2.35×/2.08× and standalone service 3.91×/2.74× over Wasm with row logging. The same columnar implementation also improves Wasm. These matched recorded results show that most of the end-to-end saving comes from logging:

| Workflow | Scene | Wasm with row logging (s) | Wasm with columns (s) | Final native with columns (s) |
| --- | --- | ---: | ---: | ---: |
| Fresh trial, 200 steps | HP3 | 1.455 | 0.516 | 0.483 |
| Fresh trial, 200 steps | HP4 | 3.553 | 1.281 | 1.247 |
| Standalone autocal service, 10,000 steps | HP3 | 3.208 | 0.881 | 0.837 |
| Standalone autocal service, 10,000 steps | HP4 | 4.984 | 2.017 | 1.819 |
| Real six-point RRF collection | HP3 | 7.960 | 4.701 | 4.590 |
| Real six-point RRF collection | HP4 | 19.274 | 10.696 | 9.995 |

In the HP4 10,000-step diagnostic Python profile, the old recorder made 115,797 log calls and 99,296 Scalars constructions for 201 observations. The columnar version made 597 row calls plus 2,292 send_columns calls; RerunSystem.update cumulative profiled time fell from 5.18 to 2.00 seconds. Instrumented timings are attribution evidence, not the reported speed ratios. The count of flushes varies with wall-time thresholds and topology/removals. CPU sampling attributes most physics time to the native call; perf events were denied by perf_event_paranoid=4, so native instruction attribution used Callgrind.

## Physics-only results

Eight 30,000-step prefixes use 5,000 warmup updates in a separate world. Timers include the shared JS command/adapter/extrusion systems and native physics, excluding construction and checkpoint observations. No recording, firmware or transport is inside these timers. The final artifact is built by the reproducible canonical trainer, separate from the earlier manually trained candidate.

| Workload | Wasm (s) | Prior native (s) | Final native (s) | Final / Wasm throughput | Gain over prior native |
| --- | ---: | ---: | ---: | ---: | ---: |
| hp3-logo-on | 0.8895 | 0.7588 | 0.7045 | 1.263× | 1.077× |
| hp3-logo-off | 0.7792 | 0.6663 | 0.6215 | 1.254× | 1.072× |
| hp3-autocal-on | 0.7698 | 0.6495 | 0.5968 | 1.290× | 1.088× |
| hp3-autocal-off | 0.8166 | 0.6922 | 0.6238 | 1.309× | 1.110× |
| hp4-logo-on | 3.5570 | 3.2751 | 2.8494 | 1.248× | 1.149× |
| hp4-logo-off | 2.9648 | 2.5608 | 2.2839 | 1.298× | 1.121× |
| hp4-autocal-on | 3.4352 | 3.0420 | 2.7152 | 1.265× | 1.120× |
| hp4-autocal-off | 3.3525 | 2.9896 | 2.6400 | 1.270× | 1.132× |

Full autocal schedules, layering on, use the same warmup/checkpoint policy:

| Scene | Steps | Wasm (s) | Prior native (s) | Final native (s) | Final / Wasm throughput |
| --- | ---: | ---: | ---: | ---: | ---: |
| HP3 | 488,271 | 12.8140 | 10.8891 | 9.2357 | 1.387× |
| HP4 | 80,415 | 9.2300 | 8.3018 | 7.2580 | 1.272× |

Extra helper inlining, avoiding entity copies/reusing a denominator, and avoiding unused geometry history did not show useful gains on the initial HP4 logo experiment; those source changes were reverted. Host tuning alone improved that workload about 4%. The initial manually trained PGO candidate improved all eight prefixes 9–14% over prior native. The shipped canonical trainer improves the prefixes 7–15%, and the full replays 14–18%, with unchanged serialized native checkpoint hashes. These are compiler/layout gains on the same physics, not simpler dynamics.

## Numerical and lifecycle checks

Every final native prefix and both long autocal replays have the same serialized checkpoint hash as the prior native build. Continuing/fresh trials repeat deterministically. All eight fresh/controlled cases retain expected clocks, shapes and metadata; the longer controlled HP3 native/Wasm comparison still exceeds atol=rtol=1e-6 (up to 1.52 micrometres in length, 0.00665 N in force and 2.53e-5 rad in encoder angle), as did the unchanged native baseline. Cross-target floating-point differences from the previous investigation remain; no native/Wasm bitwise trajectory equivalence is claimed.

The final recording audit compares 186,366 temporal component rows over HP3/HP4 against the prior native recorder, including clocks, poses, scalar telemetry and geometry. They match exactly after normalizing equivalent radius/color splats; nondeterministic SDK row IDs and ingestion clocks are excluded. All 42 real collection jobs retain exact selected measurements (all point fields except timestamp_ms), fixed/drive/sensor roles and physics step counts: 22,712 HP3 and 28,301 HP4. All 42 standalone service runs retain the same rounded M569.3 replies for each machine. This establishes preservation on the tested runs, not calibration accuracy.

Verification passed 113 focused Python tests (11 slow tests deselected), 121 Node checks, untrained release/profiling/dev build-mode checks and the production Vite build. Tests cover ragged/sparse columns, every sample and clock, partial final batches, idle live delivery, topology changes, resets, paused scenes, encoder motion, cancellation and RRD finalization. No native artifacts or training data enter the browser output.

## Reproduction and evidence

Starting revision is 3a5e35e7. The isolated baseline checkout has only wasm-baseline.patch, which explicitly selects the existing Wasm artifact for benchmark processes; its other code and row logging are unchanged. The Wasm-columns checkout adds only the logging changes. Final native uses the default rebuilt PGO artifact. There is no Python-physics speed baseline in this study. Timings are sequential on the same Intel i7-12700K desktop, Node 24.14.1, Python 3.12.3, Rust 1.98.1 / LLVM 22.1.8, Rerun 0.38.1. Desktop applications were not stopped and no CPU affinity was imposed. Small startup/no-record differences deserve less weight than large repeated recording gains. Profiling and tests were kept outside reported timing runs; initial experimental build/setup failures are retained.

The session budget was 20 million simulated updates and 4 GiB of new artifacts. All runs are simulated and bounded; no physical hardware was moved. Full optimizer/fitting/acceptance workflows and simultaneous multi-world scaling were not timed. Raw data, all repeats, manifests, profiles, scripts and failures remain in this session.

- [Final paired summary](summary.json), [final consumers](final-consumer-matrix.json), [Wasm/prior-native/columns consumers](consumer-matrix.json), [service comparison](service-matrix.json), [fresh/live comparison](live-fresh-matrix.json).
- [Final physics prefixes](final-physics-matrix.json), [full replays](long-matrix.json), [initial PGO matrix](pgo-matrix.json).
- [Final RRD audit](final-recording-audit.json), [trajectory audit](final-trajectory-audit.json), [real collection audit](final-live-audit.json), [service encoder audit](final-encoder-audit.json).
- [Source and artifact manifest](source-manifest.json), [native build metadata](production.node.json), [implementation patch](implementation.patch), [explicit Wasm baseline patch](wasm-baseline.patch).
- [Python checks](tests-production.log), [Node checks](node-tests.log), [Vite build](vite-build.log), [build modes](build-mode-tests.log), [socket probe](socket-probe.json).
- [Research notebook](research.md), [previous investigation](../2026-10-14-native-throughput/report.md).
- [Rerun columnar logging documentation](https://rerun.io/docs/howto/logging-and-ingestion/send-columns): API semantics; signatures checked against installed 0.38.1 SDK.
- [Rust PGO documentation](https://doc.rust-lang.org/rustc/profile-guided-optimization.html): instrument/train/merge/recompile workflow.
