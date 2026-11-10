# Native Rust headless throughput, 2026-10-14

Native Rust commit ab833759 was benchmarked against the exact preceding Rust/Wasm headless default at 7905e64f, and against the former Python default for fresh research trials. Three sequential repetitions covered HP3/HP4, layering on/off, frozen logo and autocal schedules, continuing worlds, every-step fresh telemetry, real six-point firmware collections, and the standalone autocal service with and without recording. Native shared-pipeline throughput improved 1.145–1.235×, with an aggregate ratio of 1.181×. Recording reduced end-to-end gains; short real firmware collections showed only 1.02–1.03× without recording and approximately 0.99× with recording. Fresh numerical trials improved 3.11–7.17× over Python; recorded trials improved 1.46–2.12×. Repeats were deterministic, but five long physics comparisons exceeded the declared pointwise tolerance. These host measurements do not establish calibration accuracy or complete autocal optimization speedup.

## Compared revisions and method

Baseline is exact `7905e64f`, in `/tmp/hp-sim5-before-native-7905e`; native is `ab833759`. All scenes, commands, timesteps and dependencies are matched. Fresh trial inputs have identical canonical USD hashes after removing documentary provenance; their per-revision defaults are Python versus native Rust. All other performance comparisons are Wasm versus native Rust.

Intel i7-12700K, Node 24.14.1/V8 13.6, Python 3.12.3, release Rust 1.98.1, default x86_64 host target, no RUSTFLAGS override. Processes ran sequentially, with no tests or CPU/heap profiles overlapping. Three fresh-world repetitions per case. No CPU affinity, desktop-process shutdown or cross-host measurements; grouped baseline-then-native order is a limitation, especially for small differences. Timings are medians, and the linked JSON retains every repetition.

162 timed trajectories exercised 9,147,678 measured updates. Eight additional current-Wasm validation trajectories exercised 1,257,372 updates: total 10,405,050 measured updates, plus 120,000 warmup updates. Artifact storage was about 1.1 GiB. The experiment stayed within its 15-million-update, 30-minute benchmark and 4-GB artifact limits. No hardware ran.

## N01: shared headless pipeline

These timers include command dispatch, physics, view maintenance and extrusion observations. They exclude scene construction, sampled observations, firmware, transport and recording. A separate 5,000-step warmup precedes measured worlds. Logo inputs are 30,000 steps / 60 simulated seconds. HP3 autocal is 488,271 steps / 976.542 seconds; HP4 autocal is 80,415 steps / 160.830 seconds. Layering-off replay of a layering-on trace is a stress workload, not a substitute adaptive collection.

| Workload | Wasm seconds | Native seconds | Throughput ratio | Less physics time | Pointwise tolerance |
| --- | ---: | ---: | ---: | ---: | --- |
| hp3-logo-on | 0.924 | 0.765 | 1.208× | 17.2% | Pass |
| hp3-logo-off | 0.792 | 0.660 | 1.200× | 16.7% | Drift |
| hp3-autocal-on | 12.953 | 10.490 | 1.235× | 19.0% | Drift |
| hp3-autocal-off | 12.786 | 10.751 | 1.189× | 15.9% | Drift |
| hp4-logo-on | 3.590 | 3.123 | 1.150× | 13.0% | Pass |
| hp4-logo-off | 2.882 | 2.482 | 1.161× | 13.9% | Pass |
| hp4-autocal-on | 9.347 | 8.124 | 1.151× | 13.1% | Drift |
| hp4-autocal-off | 9.293 | 8.118 | 1.145× | 12.6% | Drift |

Native improves throughput by 14.5–23.5%, corresponding to 12.6–19.0% less time. The ratio of summed case medians is **1.181×**, or 15.3% less physics time. This is the additional gain over the fuller Wasm engine, distinct from the preceding commit’s 2.5× improvement over the older kernel.

## N02–N05: local consumers

Continuing controlled movement uses 1,000 steps / two simulated seconds: a 500-step A ramp, a B torque transition and 499 holds. Construction is excluded; command execution, observation mirroring and finalized Rerun output are included. Fresh trials use the actual `run_experiment` default, 200 matched steps / 0.4 seconds, every-step numerical telemetry and optional every-step Rerun. Their wall timer includes construction, source hashing and finalization. Fresh simulation-loop and construction timers are retained separately.

Frozen HP4 replay uses the actual saved command/observation schedule: the first 20,000 steps / 40 seconds for the matched recording pair, and the whole 80,415-step numerical trace. It replays motor records, not a fresh firmware planner. Continuing recording is 10 simulated Hz plus encoder observations. The standalone autocal service uses 10,000 unlayered command steps / 20 seconds through its actual HTTP/bridge/encoder endpoint and production extended-reference receiver; recording uses stride 10 / 50 Hz. Its timer includes command delivery, advance, encoder query, drain and owned-service finalization; construction is separate. It does not run collection or fitting.

All 24 actual collection trials used fresh isolated launcher services, passed `preflight` before movement, and completed the production collector. Unlayered stock HP3/HP4, one configuration (`fixed=[2,3], drive=0, sensor=1`), three points per physical direction, four noise samples, default forces and sensor preload. Each job had 120 wall seconds and 100,000 physics steps as limits. Startup/preflight are excluded from the collection timer; the outer call timer includes polling and finalization. These are six-point proofs, not full independent calibration datasets.

| Case | Prior default seconds | Native seconds | Throughput ratio |
| --- | ---: | ---: | ---: |
| controlled-hp3-record0-steps1000 | 0.079 | 0.062 | 1.271× |
| controlled-hp3-record1-steps1000 | 0.197 | 0.177 | 1.115× |
| controlled-hp4-record0-steps1000 | 0.202 | 0.174 | 1.160× |
| controlled-hp4-record1-steps1000 | 0.483 | 0.449 | 1.075× |
| fresh-hp3-record0-steps200 | 0.834 | 0.268 | 3.108× |
| fresh-hp3-record1-steps200 | 1.960 | 1.345 | 1.457× |
| fresh-hp4-record0-steps200 | 4.071 | 0.568 | 7.168× |
| fresh-hp4-record1-steps200 | 7.111 | 3.351 | 2.122× |
| frozen-hp4-record0-steps20000 | 3.585 | 3.290 | 1.090× |
| frozen-hp4-record1-steps20000 | 8.677 | 8.269 | 1.049× |
| frozen-hp4-record0-steps80415 | 15.131 | 14.066 | 1.076× |
| live-hp3-record0-steps0 | 3.879 | 3.793 | 1.023× |
| live-hp3-record1-steps0 | 7.386 | 7.465 | 0.989× |
| live-hp4-record0-steps0 | 8.009 | 7.789 | 1.028× |
| live-hp4-record1-steps0 | 17.974 | 18.119 | 0.992× |
| autocal-service-hp3-record0 | 0.425 | 0.361 | 1.179× |
| autocal-service-hp3-record1 | 3.233 | 3.044 | 1.062× |
| autocal-service-hp4-record0 | 1.329 | 1.109 | 1.198× |
| autocal-service-hp4-record1 | 4.967 | 4.882 | 1.017× |

Fresh numerical trials improve **3.11× HP3 / 7.17× HP4** over their former Python default. Every-step recording reduces that to **1.46× / 2.12×**. These are short default 200-step runs, including startup; they do not establish the same ratio for arbitrarily long trials.

For the standalone autocal service, no-recorder runs improve **1.18× / 1.20×**; with extended-reference recording, only **1.06× / 1.02×**. Its full fitting/planning/automatic-acceptance workflow was not benchmarked.

For the real six-point collections, no-recorder ratios are only **1.02× / 1.03×**. Recorded medians are approximately **0.99×**, about 1% slower. With three repetitions and desktop activity, these small differences do not establish a useful end-to-end improvement. All HP3 runs used exactly 22,712 steps; all HP4 runs used exactly 28,301 steps, across both revisions, recording modes and repeats.

### Where collection time goes

| Recorded six-point collection component | HP3 Wasm → native seconds | HP4 Wasm → native seconds |
| --- | ---: | ---: |
| Physics | 0.724 → 0.599 | 3.382 → 2.942 |
| Physics plus worker transport/mirroring | 0.996 → 0.874 | 4.402 → 3.959 |
| Numerical/Rerun observation logging | 3.004 → 3.006 | 9.020 → 8.974 |
| Whole collection, client through finalization | 7.386 → 7.465 | 17.974 → 18.119 |

Physics is faster, but firmware/collector work, observations, transport and recording dominate the whole job. Component medians overlap (worker includes physics) and must not be summed. Unaccounted wall time includes collector/firmware work and waits; it was not further profiled in this investigation.

## Numerical audit and limits

- Each N01 repetition retains identical checkpoint hashes within its backend. All eight current-Wasm validation runs match the exact `7905e` checkpoint hashes, so the shared-adapter refactor does not change these Wasm trajectories.
- Cross-target N01 tolerance was declared before timing: `atol=rtol=1e-6`, with finite values and matching structures/discrete metadata/clocks required. Report-only drift was selected explicitly and the failed flags remain false. Three cases pass; five fail. Layered HP3 autocal reaches 2.915° native-versus-Wasm difference at sampled encoder reads; unlayered stress replay reaches 3.059°. The unlayered HP3 logo differs by up to 3.218 mm in sampled geometry and 1.817 N in forces. Native and Wasm can amplify different arithmetic/trig rounding in long feedback runs.
- All eight controlled/fresh consumer comparisons pass the same mixed-unit tolerance; their repeats are exact. Frozen HP4 comparisons fail that strict tolerance, with up to 1.121 micrometres in cable lengths, 0.352 micrometres in effector positions, 0.000647 N in forces and 0.001345° in observed raw encoders. This trace contains observation records but no explicit original encoder-response records; it does not validate original endpoint parity.
- All 24 live collections completed schema/physical-response validation. Selected measurements (roles, raw angles, converted drive/sensor lengths, noise means/sigmas, counts and force) are identical across the six repeats/revisions of each case, and across recording modes for each machine. This does not establish calibration accuracy. Four noise samples are deliberately a short proof and may trigger low-sample/noise-floor warnings.
- All recorded standalone services finalized their RRDs, recorded 1,001 physics samples through step 10,000, and drained their queues. Their rounded firmware encoder replies match across revisions/repeats; this is only a 0.01° reply-level check, not full state parity.
- Browser rendering, interactive edits and connection management remain JavaScript; the browser’s engine remains Wasm. Offline optimizer/fitting acceleration and a complete full-autocal run were not measured. No native-versus-browser-rendering or hardware claim is made.

## Hypotheses and outcomes

| Hypothesis | Experiments | Outcome |
| --- | --- | --- |
| Native improves shared-pipeline throughput across the eight workloads. | N01 | Supported for fixed-schedule throughput; numerical agreement is mixed. |
| Transport and recording reduce the end-to-end gain. | N02, N05 | Supported. |
| Fresh trials improve over their former Python default. | N03 | Supported for the tested 200-step runs. |
| Short fresh firmware collections improve in wall time. | N04 | Mixed: small numerical-mode gains, no recorded gain; no useful universal improvement demonstrated. |

## Evidence and reproduction

- [Physics matrix](physics/matrix.json): every case median, repeat, checkpoint and error; per-run files retain source/build/input hashes.
- [Consumer matrix](consumer-matrix.json): controlled/fresh/frozen/live repeats and full service manifests.
- [Standalone autocal service matrix](autocal-service-matrix.json): production HTTP/bridge/receiver timings and finalization.
- [Consumer numerical audit](consumer-numerics.json), [live measurement audit](live-numerics.json), and [current-Wasm identity audit](current-wasm-validation/results.json): detached comparisons and repeatability.
- [Source manifest](source-manifest.json): exact revisions, core/host/artifact hashes, toolchain and release flags.
- [Frozen input origins](../2026-10-13-headless-js-profiling/inputs/origins.json): logo and actual autocal schedules.
- [Previous Wasm engine study](../2026-10-14-rust-engine/report.md): earlier optimization and trajectory limitations; its timings were not reused here.
- [Research notebook](research.md): objectives, budgets, decisions and completed hypotheses.

Run the retained scripts sequentially from the repository root, with a new output session path if repeating. `matrix_run.py` runs N01 against the baseline checkout; `consumer_run.py` runs N02–N04; `autocal_service_run.py` runs N05; `validate_current_wasm.py` and `compare_consumers.py` perform identity/numerical checks after timings. The baseline checkout is pinned to `7905e64f` with the current Node dependencies, `.venv` and unchanged RRF binary. These runners preserve outputs and refuse existing trial directories.
