# Autocal validation, collection throughput and compiled Python physics

Recorded and synthetic HP3 experiments tested candidate validation, bounded adjustments and informative movements. A guarded veto and tie-breaker rejected impossible candidates in 15/15 deliberate counterexamples without changing the eleven recorded selections; no recorded HP3 accuracy gain was established. Bounded updates failed under winding mismatch and poor recorded starts. A production HP4 collection took 1693.12 seconds for twelve observations. Headless JS subsequently improved matched short recorded workloads by 19.03× and retained frozen-replay encoder agreement. A double-precision Warp CPU solver passed 99 fixtures and accelerated complete HP4 updates by 1.96×. Longer controlled comparisons used 20 simulated seconds per trial, three repetitions per HP3 backend and one per HP4 backend. CUDA took 37.24/168.61 seconds on HP3/HP4, versus CPU 26.87/105.95 and JS 1.27/6.63 seconds. Maximum sampled CUDA-to-JS encoder difference was 0.002161°, below the 0.01° comparison bound. Profiling identifies costly serial CUDA execution/readbacks and remaining Python systems. CUDA was slower for these single-world workloads. Fresh complete collection throughput, independent-world GPU batching and hardware accuracy remain unestablished. CPU-only JAX remains a deliberate prior performance choice.

# Encoder autocal investigation — 7 October 2026

Implemented a guarded whole-sweep validation repair. Each candidate retains its immutable fitted spool parameters, including encoder reference angles. Validation transforms raw observations with that candidate's radii and winding parameters, uses its fitting options, and requires predictions for every observation. Invalid or incomplete predictions return infinity rather than the fitting objective's finite underconstrained penalty. If every available validation attempt fails, full-auto stops without applying calibration. Failed validation is logged.

The common-model anchor comparison remains the primary history ranking, with the candidate-owned prediction used as a veto and tie-breaker. These fields are now named separately in the log. This conservative choice follows an experiment where making candidate-owned prediction the primary score worsened an elasticity fixture. The fitter, D-optimal movement selector, firmware/browser collector, automatic patience and bootstrap behavior remain unchanged. There is no manual print-inspection step.

**Scope and measures.** HP3 was prioritized following user steering. Existing `hp3` aliases and recorded HP3 fixtures use the `hangprinter_4` autocal geometry; the continuing native collector is HP4, not an HP3 accuracy test. Offline collection consumes recorded sweeps, with zero live movements. Anchor error is RMS Euclidean error per anchor against declared simulation truth; radius error is RMS mm per spool. Selection scores, geometric residuals, sensor prediction, failure coverage, sweep count and wall time are separate quantities.

Full sweeps are held out. “Future” sweeps were absent from the selected candidate's fit but can have been used for history selection. Strict test sweeps occur beyond every fitted replay prefix and never enter fitting or selection; some fixtures run out of such sweeps. Sensor-only prediction uses the analytical circle model conditioned on observed drive length. Without effector pose there are two sensor branches; the nearest branch is reported as an explicit lower bound, along with invalid prediction counts. It is not a uniquely predicted sensor trajectory. Geometric residuals are Sampson approximations in mm and are not labeled sensor error. Metrics use printed parameters rounded by the production log, so sub-centimetre differences need caution.

**Reproduction.** E01 audits all 11 October 7 reference logs. Seventh HP3's score-selected iteration had 100.737 mm parameter regret; April HP3 had 163.828 mm. This legacy diagnostic sums anchor Euclidean errors plus 2π times radius absolute errors; it is distinct from the RMSE table below. Fresh baseline replay already differs from some reference selections, so final regressions compare against fresh baselines rather than refreshing tracked references.

The current fresh baselines still disagree with physical accuracy. On seventh HP3, iteration 4 has a smaller future geometric residual (~0.50 robust mm) than iteration 3 (~1.06 mm), while its anchor RMSE is worse (55.81 vs 40.58 mm). Candidates also have different future folds, which adds ranking uncertainty. A single common final-sweep experiment did not consistently fix ranking. Sparse data and invalid constrained circles can give the fitting objective a finite sentinel of 100: that is not successful prediction.

**HP3 measurements.** Final selected anchors/radii match baseline where completed. This repair shows no measured HP3 anchor-accuracy or speed gain.

| Dataset | Anchor RMSE mm | Radius RMSE mm | Future sensor lower-bound RMSE mm | Strict test sensor / encoder RMSE | Fit iterations / recorded sweeps consumed | Baseline / final seconds |
| --- | ---: | ---: | ---: | --- | ---: | ---: |
| Fifth HP3 | 18.47 | 0.1845 | 14.35 | Unavailable | 7 / 9 | 58.08 / 57.63 |
| Seventh HP3 | 55.81 | 0.1745 | 2.07 | 2.34 mm / 3.53° | 5 / 7 | 41.79 / 41.83 |
| Sixteenth HP3 | 14.34 | 0.1845 | 0.94 | Unavailable | 6 / 8 | 48.70 / 44.39 |
| Nineteenth HP3 | 42.13 | 0.1845 | 6.75 | 4.88 mm / 7.62° | 5 / 7 | 42.67 / 39.24 |
| April HP3 | 64.42 | 0.1845 | 38.73 | 62.86 mm / 99.10° | 6 / 8 | 48.96 / 47.26 |

Each HP3 sweep contains 20 points. The unchanged replay counts imply unchanged recorded measurement counts and no added collector movements. Wall times include JAX startup, fitting, diagnostics and planning; native physics runs concurrently. They are not controlled solver-throughput measurements. The strict April test remains poor, despite a reasonable history score: this limitation is not fixed by validation bookkeeping.

**Structured adjustment experiments.** E02/E04 use synthetic HP geometry, 12 complete configurations, three seeds and poor starts (60.0–105.3 mm anchor RMSE). Fit sweeps 1–6, validate on 7–9, and test on 10–12. Local least-squares proposals are bounded by ±50 mm per optimizer coordinate and, when enabled, ±0.5 mm radius per update. A fixed 1 mm robust residual scale avoids adapting the validation loss to a bad model. Updates are accepted only when separate validation sweeps improve; at most four updates are attempted. Reusing the validation set can bias acceptance, which is why test sweeps remain untouched.

With the correct winding model and known radius, final anchor RMSE is 0.05–0.09 mm with 0.02 mm encoder noise, 1.35–2.16 mm with 0.5 mm noise, and 0.80–0.88 mm with a 20 mm sensor outlier per sweep. These are conditional synthetic results, not a replacement solver demonstration. Known radius was an experimental control. Joint radius adjustment under 0.1% pose-dependent elastic strain leaves roughly 60 mm anchor error and ~0.21 mm radius bias despite improved held-out residuals.

A physically representable winding mismatch (actual 0.68, modeled 0.636619) defeats the acceptance criterion: seed 0's fixed-radius anchor RMSE worsens from 60.0 to 146.0 mm while validation geometric residual falls from 11.13 to 3.49 robust mm. The first winding trial used 1.2, exceeding the spool inverse's domain; its clipped data are invalid and retained as `*-v1`, explicitly excluded from findings. A domain check was added to the experiment generator and the smaller mismatch repeated.

E04's ten recorded HP3 local trials cannot reliably escape bad starts. First-fit seventh HP3 geometry starts around 2191 mm RMSE and worsens under accepted proposals. Four recorded trials have incomplete strict-test prediction coverage. Warm-starting the full loop is also rejected: fifth HP3's selected anchors worsen from about 18.47 to 106.90 mm RMSE, with no convincing runtime benefit. These failures are retained in `warmstart/` and the E04 result files.

**Movement choice.** E06 uses the existing D-optimal selector to choose one of six controlled synthetic sweep configurations, then performs one bounded update. Across three starts, chosen movements reduce anchor RMSE from 60.0/71.8/105.3 to 29.1/33.4/76.8 mm. The selector chooses the physically best tested movement in two of three cases; the third has 7.35 mm anchor-RMSE regret. Each acquired sweep has 30 observations; three final configurations remain test-only. This supports retaining informative movement selection but does not support changing its geometry-only information approximation or claiming a collection-count reduction.

**Measured robustness improvement.** E07 supplies deliberate candidate counterexamples, not fitter-generated candidates: identical anchors with correct/incorrect radii and winding, plus an impossible collapsed HP geometry. Across three seeds for clean, noise, bad-point, elastic-strain and winding conditions (15 cases), the old shared-model score selects the collapsed geometry because its sentinel is artificially cheap. The final validation veto and tie-breaker select the physically correct candidate in all 15 cases. Impossible selections fall from 15/15 to 0/15. The radius-only tie also has an end-to-end full-auto regression: the old anchor comparison cannot distinguish 35 mm from 40 mm radii, whereas the frozen-model tie-breaker selects 40 mm. These controlled robustness gains do not imply improvement on every candidate pool.

The rejected primary-prediction variant (`corrected_slide/`) selects 39.82 mm instead of 39.14 mm on `flexible_lines_2000` and substantially worsens anchors. It also improves some other fixtures, but that mixed result is insufficient to make it the default. Final selection retains the tested baseline behavior on this case.

**Native collection and Rerun.** E05 uses real RRF planning and the unchanged production collector: one HP4 configuration, six points per physical direction, 16 noise samples, 15 s simulation settling deadline. Native physics is in metres; collector lengths are millimetres and raw encoders are degrees. The authored scene has 0.03 m spools and distributed cable attachment geometry; its truth is not the recorded HP3 fixture or firmware M669 guess. No native anchor/radius fit is claimed from one combined sweep.

The first observed drive setpoint is −47.124 mm but its encoder-derived measurement is −32.034 mm, a 15.09 mm discrepancy. Later observations track much more closely. At simulation t=56.6 s the trace reports ~1.006 rad drive tracking error and eight peak missed steps, with ~0.3 mm lower cable stretch and ~1.2 mm top cable stretch. These are model-mismatch and collection-quality mechanisms, not calibration gains. Numerical events are the source for these values; Rerun inspection confirms the motion, encoder and cable-force timelines. The existing recorder supplied the RRD. Concurrent edits elsewhere in the workspace caused a runtime source-stale flag; this run remains tied to its frozen loaded-source hashes and no second world was substituted.

**Validation and evidence.** All 300 autocal unit/regression tests pass (`autocal-tests.txt`). Full production CLI replays, the existing calibration-log regression comparator, and final-policy results are retained. All 11 final-policy CLI replays pass the existing calibration-log regression comparator against fresh baselines. Selected anchors, radii, fit/UI summaries and replay counts are unchanged across all 11 datasets. Across 35 full CLI experiment replays there are zero subprocess failures. The final-policy runs have 6 underconstrained selected fit attempts out of 64 iterations; this rate is unchanged and distinct from process success. Experiments, timeouts/invalid generators, raw data, commands, logs and numerical results remain under this session. `research.md` records hypothesis changes and budgets; scripts provide reproduction with `.venv/bin/python`.

Useful artifacts: `E01-audit.json`, `E03-selection-audit-corrected.json`, `prediction-metrics.json`, `E02-E04-synthetic-results.json`, `E04-recorded-results.json`, `E06-movement-results.json`, `E07-ranking-stress.json`, `final-regressions.json`, and `ranking-and-model-mismatch.png`/`.pdf`. The initial E03 reconstruction used the final replay horizon despite an excluded underconstrained fit; it is retained but superseded by the corrected horizon audit.

**Remaining limits.** Physical scale/radius/winding and elasticity can compensate each other. Candidate-dependent future folds remain incomparable in difficulty. The latest fit can lack a validation movement, and initial bootstrap still permits the existing fallback when none exists. A finite, complete residual is evidence of model consistency, not accurate anchors. More independent force/configuration sweeps, a credible winding/elasticity model, and identifiability checks are needed before deploying the bounded local solver or a new primary score. The implemented repair does not establish faster convergence, fewer movements, native HP3 accuracy or hardware transfer.

**Completed native outcome.** The job completed in 1693.12 wall seconds, advancing 80,415 fixed steps (160.83 simulation seconds). All 12 measurements survive in the version-2 dataset and its canonical drive/sensor roles. The production ingestion validator evaluates all 12 residuals; the supplied firmware-guess residual RMS is 0.2459 mm. This low residual does not establish accurate physical anchors. Observed sensor travel spans 39.65 and 39.67 mm in the two physical directions. There are 12 measurement positions plus setup/return motion. Bridge G1 payload counts represent firmware schedule records rather than independent calibration movements. Sampled telemetry peaks are 1.276 rad tracking error, 10 reported missed steps and 9.386 mm cable stretch during transients. Snapshot values at t=56.6 s above describe a different, calmer part of the same run. The world is no longer collecting.

The completed manifest and numeric motion summary are `E05-native-summary.json` and `E05-motion-metrics.json`. The finalized recording is `native/491996aca88c4370b7f348d11777f532/2f047235396741f085988cd05f7c124a/recording.rrd`; the dataset is alongside it as `sweeps.json`. `E05-native-final.png` and `E05-viewer-state.json` preserve inspection at the completed 160.83 s timeline. Viewer warnings concern display transparency and a timeline that did not yet exist at startup; the completed Viewer state contains that timeline.


## Tooling improvement follow-up, 2026-10-07

The 28.2-minute collection exposed a Python physics bottleneck: world updates
accounted for about 98% of an instrumented profile. Continuing collection now
uses the existing production JavaScript physics pipeline headlessly in Node,
with Python mirroring detached observations for the existing numerical telemetry
and Rerun recorder. Fixed 2 ms steps, solver settings, equations, encoder units
and the production collector/firmware boundary are preserved. Python remains an
explicit differential backend. Matched HP3 scene and firmware selection is now
available through `--machine hp3`; `--no-record` retains numerical evidence while
omitting RRD recording. Cancellation clears both adapter and worker queues at a
completed fixed-step boundary. Backend, worker health, source hashes and separate
physics/transport/observation timings are exposed.

Three sequential repetitions of the same 1,000-step controlled HP4 workload
with recording take median 20.709 s in Python and 1.088 s in headless JS: **19.03×
faster**, from 0.097× to 1.84× realtime. HP3 JS controlled motion reaches 6.32×
with recording and 9.65× without recording. Construction is excluded from these
short benchmarks; HP3 Python throughput was not measured.

The full original firmware schedule, 80,415 steps / 160.83 simulated seconds,
replays in **78.428 s with Rerun** or **53.015 s without Rerun**, respectively
2.05× and 3.03× realtime. All 1,120 recorded encoder reads match the original
Python observations within 0.002242°, passing the existing 0.01° tolerance.
Maximum sampled effector position, cable length and segment-force differences
are 0.014464 mm, 0.041146 mm and 0.086125 N. These are parity measurements, not
hardware fidelity or anchor accuracy evidence. Numerical modes produce identical
encoder results. Rerun successfully opened the finalized replay recording and
its screenshot is retained as `P03-rerun.png`.

The actual browser simulator also advanced 10,000 HP4 steps / 20 simulated
seconds in 3.75 wall seconds, including its tool request, with recording off.
That confirms useful browser stepping; it is not a timed browser collection.
Recorded replay observation logging costs 27.104 s versus 1.562 s in numerical
mode, making recording a substantial remaining optimization opportunity.

Validation: 345 Python tests pass, including all 300 autocal tests; 19 production
JS tests pass. Eight opt-in slow Python tests and one stdio initialization test
were deselected. The stdio timeout reproduces on the unchanged HEAD entry point
in this environment. Following final worker lifecycle edits, 13 focused native
collector/terminal tests pass. Compilation and diff checks pass. Earlier failed
tests, asynchronous-IPC experiments and benchmark repetitions remain archived.

A fresh end-to-end collection with the new supervisor was **not measured**:
the terminal sandbox denies socket creation, and the attached supervisor retains
its loaded old code until the next launcher start. Frozen replay exercises the
real recorded schedule but cannot test new planner/settling decisions. Therefore
78.428 s is not a measured replacement timing for the original 1693.12 s complete
collection. Fresh `run_experiment` still uses Python and remains slow. The next
check is a fresh HP3/HP4 production collection after restarting the launcher.
No calibration accuracy gain or movement-count reduction is claimed here.

Use `./hp-sim5-research-agent --machine hp3`, or add `--viewer none --no-record`
for numerical-only collection. Reproduce frozen replays with
`.venv/bin/python scripts/replay_collection.py SCENE EVENTS --output NEW_DIR`.
The full methods, measurements and limitations are in
[collection-performance.md](/home/torbjorn/repos/hp-sim5/research/collection-performance.md).
Evidence: `P01-profile.pstats`, `P02-matrix.json`, `performance_matrix.py`,
`P03-final-recorded/replay.json`, `P03-final-numeric/replay.json`,
`tooling-tests-final.txt`, `tooling-lifecycle-final.txt`, `baseline-stdio.txt`.


## Compiled Python solver follow-up, 2026-10-08

The user requested compiling Python's hot solver with compact state arrays,
using existing Warp where possible and optionally CUDA. The existing 2D Warp
solver is not a 3D substitute: it uses scalar angular state, float32 and atomic
parallel corrections, without the production 3D member/spool/load semantics.
C01 rejects that substitution by code inspection; it does not measure its speed.

Implemented an opt-in 3D Warp solver. The complete sequential constraint loop
and its frame/quaternion/tensor/spin math compile together. Double-precision
buffers pack only participating endpoints and parent bodies once per update,
are reused while sizes match, and share CPU NumPy views. Coupling metadata is
refreshed for topology and motor changes. Alternating path/joint order and
sequential A/B corrections remain unchanged. The reference Python implementation
remains the default for direct construction; continuing collection retains JS
as default. No dependency was added. Native Warp imports are lazy.

C02 compares 99 existing fixtures under their established tolerances. All pass,
including loaded machine pipelines, tensor bodies, rigid members, spools,
pinholes, torque limits, damping, winding, topology edits and load maps.
The full relevant regression run passes 737 tests, including all 300 autocal
tests and independent Python/JS differential tests. Five CUDA tests are skipped;
14 tests are deselected (13 opt-in slow tests and the previously reproduced
stdio environment timeout). Final source cleanup is checked by a further
focused run: 104 pass, five CUDA skips, 52 deselected. Compilation and diff
checks pass. This tests compiled physics and tool integration, not hardware.

C03 performs three sequential repetitions per machine/backend: 500 fixed
2 ms steps, an A-axis ramp then B torque mode. Whole-world timing includes every
physics system and the complete pack/solve/unpack adapter, but excludes
construction, compilation, recording and observation logging. CPU timing medians:

| Machine | Reference world seconds | Warp world seconds | Whole-world speedup | Solver speedup | Warp realtime factor |
| --- | ---: | ---: | ---: | ---: | ---: |
| HP3 | 1.839 | 1.327 | 1.39× | 3.00× | 0.754× |
| HP4 | 9.964 | 5.092 | 1.96× | 7.90× | 0.196× |

Initial uncached compilation is separately measured at 1.450 s. The first port
packed all ECS entities; compact endpoint/parent packing improved adapter cost.
Both earlier matrices remain archived as v1/v2. The final measured solver source
is frozen as `compiled-solver/solver-source.py` and its hash matches C03-timing.json.
Within-backend repetitions are exact. Cross-backend final maximum position,
quaternion-component, encoder and cable-force differences are respectively
2.663e-8 m / 4.307e-7 / 8.630e-7 rad / 1.111e-4 N on HP3 and
7.558e-12 m / 1.691e-6 / 5.103e-6 rad / 1.434e-8 N on HP4.
The largest encoder difference is approximately 0.000292 degrees. These values
are separate from throughput; no anchor error or calibration gain is measured.

C04 verifies that requesting an unavailable CUDA device fails explicitly.
CUDA mode uploads/readbacks compact buffers and uses the same ordered kernel.
The agent process cannot access CUDA hardware, so GPU parity and performance remain
**not tested**; five hardware-dependent fixture tests are ready to run on such a
host. One coupled world uses one thread. GPU transfer/launch costs may dominate,
and independent-world batching is future work. A successful CPU port does not
establish that CUDA is faster or accurate.

Continuing collection uses `--physics-backend native-warp`; experimental GPU
mode uses `native-warp-cuda`. Standalone Python, fresh `run_experiment`, and
frozen replay can also explicitly select the compiled solver. The fresh manifest
names backend/device/Warp version, and comparisons report backend differences.
Compiler diagnostics go to stderr to protect stdio MCP. Session tests cover
encoder barriers, reset and cancellation; the existing recording pipeline is
retained. A fresh complete firmware collection with this backend was not timed.

Python remains below realtime and JS remains faster. The compiled profile
identifies attachment/frame updates, rigid-body synchronization, friction and
over-correction as remaining interpreted work. The next useful step would be
compiling those systems and retaining numerical state between them, with the
same topology/command boundary contracts. This follow-up delivers the bounded
CPU solver improvement, not a fully compiled simulator.

References and retained evidence:

- [Methods and usage](../../../../research/compiled-python-solver.md): API, numerical contract, timing and GPU limits.
- [Repeated measurements](compiled-solver/C03-timing.json): C03 timings, device/package provenance, source hash and final numerical differences.
- [Benchmark script](compiled-solver/benchmark.py): controlled movement and timing method.
- [Frozen measured solver](compiled-solver/solver-source.py): exact implementation used in the final matrix.
- [Full regressions](compiled-solver/C02-regressions.txt), [final focused tests](compiled-solver/C02-final-focused.txt): C02/C04 verification and skips.
- [Compiled profile](compiled-solver/C03-warp-profile.txt): remaining Python bottlenecks; profile timing is not an uninstrumented throughput result.
- [NVIDIA Warp compilation documentation](https://nvidia.github.io/warp/v1.15/deep_dive/codegen.html), [structured arrays](https://github.com/NVIDIA/warp/blob/main/docs/user_guide/runtime.rst): mechanism references; installed Warp 1.12.0 was inspected and tested locally.


GPU diagnosis correction, 2026-10-08: the host GPU is an NVIDIA GeForce RTX
3070 Ti, and driver 580.178.04 is loaded. The agent execution environment has a
minimal tmpfs mounted over `/dev`, without NVIDIA device nodes. `nvidia-smi`
fails in this process, Warp reports CUDA error 100, and venv JAX sees only CPU.
`CUDA_VISIBLE_DEVICES` is unset. Plain `python` uses `/usr/bin/python` without
Torch/JAX, while `.venv/bin/python` contains JAX and Warp. This corrects the
earlier inference of absent host hardware; GPU validation remains unmeasured
because device access is unavailable here. No driver or dependency changes
were made. [Raw requested diagnostic](compiled-solver/C04-user-diagnostic.txt),
[device/mount evidence](compiled-solver/C04-device-visibility.json).


**Unsandboxed CUDA follow-up (C04B).** After the user disabled the sandbox,
`nvidia-smi` succeeds and NVIDIA character devices are exposed. Warp 1.12.0
reports CPU and `cuda:0` on the RTX 3070 Ti. All five targeted CUDA reference
comparisons pass (3.69 s; 102 other tests deselected), covering bodies, spools,
pinholes, the HP3 pipeline and loaded HP4 torque pipeline. This is initial
numerical CUDA evidence, not GPU throughput or long-horizon validation. JAX
0.9.2/jaxlib 0.9.2 remain CPU-only: CUDA12/13 plugin/PJRT packages are absent.
No dependencies were installed. The earlier restricted-process diagnostics
remain valid historical records, but the Warp access blocker is now resolved.
[Unsandboxed diagnostic](compiled-solver/C04-unsandboxed-diagnostic.txt),
[CUDA comparisons](compiled-solver/C04-cuda-parity.txt).

## Longer CUDA comparison, 2026-10-08 (L01–L03)

CUDA did not match compiled CPU or JS on this controlled workload. Each trial
advanced 20 simulated seconds / 10,000 fixed 2 ms steps, using the same frozen
scene, 500-step A-axis ramp, B-axis torque transition and subsequent hold.
HP3 has three fresh repetitions per backend, with rotated backend order; HP4
has one repetition per backend. There were 12 completed trials and no physics
failures. A preliminary script import-path failure is retained separately.

The table includes state packing, host/device transfers, synchronization,
all other physics systems, 10 Hz numerical observations and finalization.
It excludes scene construction/reset and disposable compilation warmup.
Rerun recording was disabled equally for all engines. A newly reset JS worker
can still incur V8 warmup during the timed run. This is controlled simulation,
not a fresh firmware/planner collection or a calibration fit.

| Machine | Warp CPU wall seconds | Warp CUDA wall seconds | JS wall seconds |
| --- | ---: | ---: | ---: |
| HP3, median of three | 26.87 | 37.24 | 1.27 |
| HP4, one trial | 105.95 | 168.61 | 6.63 |

CUDA is 38.6% slower than compiled CPU on HP3 and 59.1% slower on HP4. JS is
29.3× / 25.4× faster than CUDA on these cases. Realtime factors for CPU / CUDA /
JS are 0.744× / 0.537× / 15.756× on HP3 and 0.189× / 0.119× / 3.019× on HP4.
The desktop and Rerun remained active on the RTX 3070 Ti; GPU status was sampled
before/after each trial. This was not an isolated GPU benchmark. HP3 timing
ranges are CPU 26.624–27.021 s, CUDA 37.231–37.740 s, JS 1.250–1.294 s.

Every comparison checks all 201 matching observation steps and all motor,
effector, cable and force identities, with explicit coverage checks. Each HP3
backend repeats all sampled observations exactly. Maximum sampled CUDA
comparison differences are:

| Machine / comparison | Encoder degrees | Effector mm | Cable length mm | Segment force N |
| --- | ---: | ---: | ---: | ---: |
| HP3 CUDA vs CPU | 0.001036 | 0.000581 | 0.000705 | 0.002992 |
| HP3 CUDA vs JS | 0.001452 | 0.000227 | 0.001525 | 0.006671 |
| HP4 CUDA vs CPU | 0.0000102 | 0.00000284 | 0.00000837 | 0.00000502 |
| HP4 CUDA vs JS | 0.002161 | 0.000603 | 0.001772 | 0.001062 |

All encoder comparisons pass the prior 0.01° endpoint bound. Pose, cable and
force differences are separate measured quantities, not implied accuracy
thresholds. No anchor/radius error or hardware fidelity was established.
Sampling at 10 Hz can miss intermediate transients.

The complete solver adapter takes median 5.419 s on CPU versus 15.329 s on
CUDA for HP3; HP4 takes 14.389 s versus 77.105 s. Most of the added whole-world
cost is therefore in the CUDA adapter. Other Python physics remains a large
cost: HP4 spends about 91.5 s outside the solver, already far more than JS's
6.63 s complete run.

L03 profiles a separate 500-step HP3 startup ramp. CPU launch calls total
27.8 ms including synchronous execution; CUDA host launch calls total 22.9 ms.
The CUDA profile records 4,000 copy calls (five uploads and three readbacks per
step), totalling 577.8 ms. A separate instrumented GPU-event run measures
539.7 ms across 500 kernel intervals (1.079 ms average). Those values come from
separate instrumented runs: they are not independent costs to add together.
Readback timing includes waits for the kernel. This points to both expensive
serial GPU execution and per-step transfers; it does not justify attributing
the entire loss to transfer bandwidth. The single-thread ordered kernel is not
using independent-world parallelism. Kernel-event timing also includes enqueue
interval and scheduling effects; it is not an isolated instruction benchmark.

Keep JS as the collection default and compiled CPU as the faster Python option
for the tested workloads. CUDA remains useful for numerical experimentation;
it is not currently a performance option for one world. A stronger GPU design
would keep several physics systems resident and batch independent worlds while
preserving each world's solver order. Neither change was implemented in this
comparison.

The user clarified that CPU-only JAX is deliberate: earlier profiling found
GPU JAX slower for this workload, with better dependency and sandbox access as
additional benefits. This run did not benchmark JAX or change its dependencies.

Evidence: [complete measurements](cuda-long-run/results.json),
[coverage checks](cuda-long-run/validation.json), [repeatability](cuda-long-run/repeatability.json),
`benchmark.py`, immutable scene/event/command files in each trial directory,
`solver-source.py`, `L03-native-warp.pstats`, `L03-native-warp-cuda.pstats`,
`L03-kernel-events.json` and `profile_cuda.py`. The recorded revision is
62c086eb; exact solver and command hashes are in results.json. All earlier
failed setup and device-access evidence remains retained.

[Benchmark script](cuda-long-run/benchmark.py), [CPU profile](cuda-long-run/L03-native-warp.txt), [CUDA profile](cuda-long-run/L03-native-warp-cuda.txt), [GPU event intervals](cuda-long-run/L03-kernel-events.json).
