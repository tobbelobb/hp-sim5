# Native simulation and columnar Rerun performance

Status: complete. Objective: improve native headless throughput and recording costs, using the fuller Wasm runtime as the only speedup baseline. Starting revision: 3a5e35e7. Browser/driver repair is complete and outside this investigation.

Hypotheses: O01 live row logging overhead can be reduced by buffering raw values and calling send_columns; O02 native compiler host tuning and redundant geometry work can reduce the physics cost; O03 remaining transport and observation overhead can be reduced without changing observation cadence or commands.

Constraints: same Rust source for native and Wasm; same fixed steps, force/motor algorithms, logging entities, values and explicit timelines; bounded buffers and live delivery; preserve reset/topology/removal semantics. No comparison to the previous Python physics engine. No physical hardware moves.

Budget: 20 million simulated updates, 45 minutes compute, 4 GiB new recordings. Three or more sequential timing repeats, profiling separately. No long live collection without preflight. perf events are unavailable (perf_event_paranoid=4); use Python/Node profiles and bounded diagnostic Rust stage timing instead.

Evidence: previous session ../2026-10-14-native-throughput/report.md; isolated unchanged checkout /tmp/hp-sim5-performance-before-3a5e35e7; before.node is the original release artifact and its adjacent metadata. No new measurement result yet.

Next decision: profile recording and native physics, implement the largest measured reductions, validate finalized RRD data, then run matched Wasm/native end-to-end benchmarks.

O01 initial profile: 10,000 fixed HP4 updates / 201 observations under cProfile spent 5.18 s in RerunSystem.update, including 115,797 log calls and 99,296 Scalars constructions. Total profiled timed advance/finalization 7.31 s. This is diagnostic, not an uninstrumented benchmark. Failed first profiling attempt used a relative artifact path with an isolated cwd; retained its artifacts, reran with an absolute path.

Data mapping before implementation:

| Live source | Existing entities | Archetypes | Timelines | Static/temporal |
| --- | --- | --- | --- | --- |
| Poses and cable geometry | world/machines/... | Transform3D, LineStrips3D, Arrows3D, Points3D | scene_generation, sim_step, sim_time; flight also wall_time | temporal |
| Lengths, forces, motors, encoders, velocities, deposits | existing metric paths | Scalars | same explicit sample timelines | temporal |
| Shapes, coordinates, axes | existing world paths | Points3D, TransformAxes3D, ViewCoordinates | none for native static styles | preserve static behavior |
| Collector events and clock provenance | existing clocks/autocal paths | TextLog, AnyValues, Scalars | preserve existing event/sample timelines | temporal |

Native and flight samples are produced live, not read from a supported file format. Buffer raw component fields and create archetype columns once per bounded batch. Keep all explicit timeline values; send_columns does not inherit set_time. Flush buffered data before resets/removals, finalization and explicit live delivery. No resampling or signal removal.

O01 implemented: shared ColumnarRecording buffers detached component values, with 128-sample / 65,536-instance limits and one-second active flush. Native RerunSystem and FlightRecording use columns for poses, geometry, scalars and sampled clocks. Rare styles, provenance and clears retain row logging. Owners flush before clears/reset, recording rotation, finalization and explicit delivery. Flight live viewers also get an idle periodic flush. Native snapshots are reused for the event journal rather than computed twice.

O02 trials, all HP4 logo layering on, 30k fixed steps, 5k warmup, three sequential unprofiled repeats: original Wasm median 3.697856 s; original native 3.260950 s; target-cpu=native 3.136504 s; explicit small helper inlining 3.202114 s; borrowing entity records/reusing denominator 3.264597 s; avoiding unused post-solve geometry history 3.254227 s. All native candidates retained the original native checkpoint hash. None of the source changes showed a sufficient useful gain; all reverted. Host tuning measured ~4% on this one workload and is not a default change. It remains explicitly selectable through RUSTFLAGS and --native-addon. perf was denied; Callgrind successfully collected ~1.10 billion instructions over 1000 engine steps. Node sampling attributes most time to the native step and about a tenth to the shared adapter.

O01 diagnostic after profile: RerunSystem.update falls from 5.18 to 2.00 profiled seconds; row log calls fall from 115,797 to 597, with 2292 send_columns calls. Whole profiled runs are not used for speed ratios. The exact number of column batches depends on wall-time flushes and topology removals.

O01/O03 first unprofiled matched results: recorded 10k-step NativeSession takes 0.689/2.146 s (HP3/HP4), versus old Wasm 1.622/4.469 s and old native 1.532/4.290 s. Standalone recorded service takes 0.821/1.821 s versus old Wasm 3.208/4.984 s. Optimized Wasm also benefits: 0.881/2.017 s standalone, showing most of the gain is logging rather than the runtime. Finalization is inside these timers; construction is reported separately. Three repetitions each. Broader tests passed 112 with 11 slow tests deselected before adding the explicit idle-viewer regression test.

Baseline construction: /tmp/hp-sim5-performance-before-3a5e35e7 is revision 3a5e35e7 with only the recorded wasm-baseline.patch selector, enabled by HP_SIM5_BENCH_WASM=1. It uses the unchanged Wasm artifact and all prior logging. HP_SIM5_BENCH_WASM=0 retains the original native release artifact. /tmp/hp-sim5-performance-wasm-columns adds the optimized Python logging files and the same explicit Wasm selector. No Python physics comparison is included. Command-context socket probe passed before real RRF collection; each fresh RuntimeService performs its own bounded preflight before start_collection. Raw timing/profile outputs and failed trials are retained.

Next decision: finish real six-point RRF and every-step fresh-trial timings, validate physics/measurements and complete RRD component rows, then publish measured results and limitations.

O02 follow-up supported: PGO plus host CPU tuning improves all eight 30k-step prefixes while retaining exact serialized native checkpoint hashes. A reproducible production trainer now runs 40k deterministic updates over HP3/HP4, layering on/off and motor/closed-loop transitions. Release and profiling builds default to this training; --no-pgo explicitly selects the old untrained recipe, and dev defaults to untrained. Explicit target-cpu flags are respected. Matching llvm-tools is required; missing tools fail with install/opt-out instructions and never silently substitute a build. Training happens only during an explicit build, never in the profiler or runtime.

The first instrumented staticlib host failed with an undefined LLVM profiling symbol. Retained instrumented-unlinked.node and the failure log. The build now explicitly links profiler_builtins and its atexit writer during profile-generate; training produces a real .profraw and profile-use succeeds. Manual original frozen-input training is retained separately from the default canonical production trainer. The default artifact records its exact profile hash, trainer hash, flags and baked training worlds; default freshness checks also validate the trainer. Final production timings use that artifact, not the manually trained candidate.

Final production 30k-prefix physics measurements: 1.248–1.309x versus Wasm, 1.072–1.149x versus the prior native release, exact native checkpoints in all eight. The full HP3 488271-step replay improves Wasm 12.814 s / original native 10.889 s to 9.236 s (1.387x versus Wasm, 1.179x versus prior native), with the same native checkpoint hash. HP4 long replay and final build-mode checks pending.

Final production consumer timings (three runs each) retained in final-consumer-matrix.json. Recorded continuing worlds: HP3 0.693 s / HP4 1.918 s. Recorded standalone services: 0.837 / 1.819 s. Recorded real six-point RRF collections: 4.590 / 9.995 s, versus Wasm 7.960 / 19.274 s. No-record real collections: 3.902 / 7.534 s versus Wasm 4.093 / 8.588 s. All collection step counts remain exactly 22712 / 28301. These comparisons include owner finalization and use Wasm alone as the external baseline.

The fresh-trial simulation_wall_s timer was corrected to include deferred column construction, as earlier row logging did. Whole-call timers already include it. Fresh-trial rows will be repeated with this final timer placement; earlier rows remain in their original artifact directories.

Status: complete. The full HP4 80,415-step replay measured Wasm 9.230 s / prior native 8.302 s / final native 7.258 s, with unchanged native checkpoint hashes. Fresh trials were repeated after deferred-column timer accounting was corrected; whole-call medians are 0.483 s HP3 and 1.247 s HP4 with recording, versus Wasm 1.455 / 3.553 s. No Python-physics timing baseline is included.

Final validation: 113 focused Python tests pass (11 slow tests deselected); all 121 native/Wasm/profiler/browser-UI Node checks pass; Vite production build passes with no native/training assets in dist; explicit --no-pgo release/profiling/dev artifacts build and load. Idle live flushing, sparse/ragged data, pause/reset/topology and finalized tails are covered. Final audits pass for 186,366 temporal RRD component rows and all 42 real collection jobs (21 per machine, including initial and production PGO rounds); selected point fields excluding timestamp_ms and physics step counts are exact across variants. All 42 standalone service runs have exact rounded encoder replies by machine.

Compiler-only PGO changes preserve the native trajectories. Existing cross-target differences remain: the 10k-step controlled HP3 case exceeds atol=rtol=1e-6, with maximum encoder difference 2.53e-5 rad, cable force difference .00665 N and length difference 1.52 micrometres; final native equals prior native exactly on those observations. Full native/Wasm trajectory equivalence or calibration accuracy is not claimed.

The final implementation also initializes recorder cleanup state safely, includes the trainer in stale-source detection and records deferred column construction in fresh simulation_wall_s. Cleanup/source-fingerprint additions do not change commands or physics. Final public summary is research/native-performance.md; the report, all repetitions and limits are in report.md. No further experiment is pending. Untested follow-ups: whole fitting/optimizer speed and simultaneous multi-world scaling; remaining per-step Python mirror/snapshot costs may warrant a separate investigation.
