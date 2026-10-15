# Autocal recording costs, 2026-10-15

Use `recording_mode="events"` for routine full-autocal research and `"none"` for
large numerical sweeps. Choose `"sampled"` when motion/force plots are useful;
reserve `"full"` for discoveries that need detailed validation. On the matched
HP3 replay below, event recording cost about 5% more wall time than no recording;
full recording cost about three times as much. These are recording-workload
measurements, not full-autocal job speedups or calibration accuracy results.

## Measured comparison

Four sequential trials per mode, rotated mode order, on an Intel i7-12700K,
Node 24.14.1, Python 3.12.3 and the current native Rust engine. Each trial replays
the same **200,000 fixed 2 ms steps (400 simulated seconds), 30,758 commands and
1,060 nonempty encoder reads** from a saved production HP3 autocal trace.
Physics and source settings are identical across modes: HP3, layering disabled,
production command FIFO and closed-loop setting changes at their saved steps.

| Mode | Replay + drain + finalization, median (range), s | Complete trial median, s | RRD, MB | Physics snapshots |
| --- | ---: | ---: | ---: | ---: |
| `none` | 4.82 (4.75–4.87) | 5.15 | 0 | 0 |
| `events` | 5.05 (5.02–5.10) | 5.77 | 3.44 | 0 |
| `sampled` | 5.49 (5.44–5.52) | 6.20 | 10.96 | 2,001 |
| `full` | 15.23 (14.99–15.93) | 15.91 | 54.23 | 20,001 |

MB means 1,000,000 bytes. Every trial also writes the same **6.58 MB command and
encoder journal**; add that to RRD bytes when budgeting these two artifacts.
Verification checkpoints, input copies and process logs are separate benchmark
artifacts. This table is not a total full-autocal artifact-size estimate.

Replay timing includes command dispatch, physics, equal numerical verification
work, journal writes, event transport/backpressure, pending-message drain and
recorder exit/file finalization. Complete trial timing additionally includes
recorder startup, Node imports, world construction, a separate 5,000-step warmup
world and checkpoint/result serialization. RRD readback is outside both timers.
The real recorder receiver and `FlightRecorder` run with existing event batching,
stride 100/10 and compact geometry. No Viewer is started.

Within this replay, `events` used **67% less wall time and 94% fewer RRD bytes**
than `full`; `sampled` used **64% less wall time and 80% fewer RRD bytes**.
Those percentages apply only to the replay + drain + finalization column and
RRD bytes. Fitting-heavy jobs can have much smaller total wall-time savings.

## Why the comparisons are fair

We froze a prefix of the accepted October 10 HP3 full-autocal job
`2ec0b06774624962a18bdbd2818eb134`, retaining its original scene, command arrival
steps, FIFO order, idle intervals, setting changes and encoder boundaries.
The collector and optimizer did not choose new workloads for different modes.
Each fresh Node process used the same warmup and a fresh measured world.
Four repetitions rotate each mode through all four positions; no benchmark
trials or tests ran concurrently with the matrix.

All 16 measured trials passed these gates before their timings were summarized:

- Exact hashes of 202 full-state checkpoints and every one of the 1,060 encoder
  reads, including positions/orientations, velocities, motor targets/encoders,
  torque/missed-step state, cable geometry/lengths/forces and command queue length.
- Equal input schedule, scene/baked-scene hashes, step and command counts,
  physics engine and loaded native artifact. Artifact SHA-256:
  `ff44a93e57d9594024b2905dd84a88332fa9cd4d7db7c6c3471bf80ac869ba61`.
- Exactly **34,017 headless events** in each recorded trial, all acknowledged,
  finalized and read back from disk. Event sequence and contents matched exactly
  after removing wall/monotonic timestamps and random session IDs. No events
  were dropped by the cheaper modes. The numeric-only case has no RRD events.

The shared trajectory hash is
`47d58d4ece8068b4e4e0cba0a8e88e4b15babec1c1cd0830f686460c2c339c1a`.
The shared normalized event hash is
`1d08eb0d5580a450c3ec22932f6ff4782a61691e8f7aee3ccab13376f0775289`.
These are observed equality checks for this workload and engine, not a guarantee
that later simulations, different backends or full-autocal runs will repeat.

## Limits that matter for experiment design

This is a **frozen collection/recording workload**, not four full-autocal runs.
It excludes fresh firmware planning, HTTP bridging, adaptive settling decisions,
Python fitting/selection and Python/collector event logs or stage-artifact copies.
It includes actual production headless command/reply and encoder events. Extra
logs and large stage artifacts can make `events` recordings larger in real jobs;
event density and trajectory complexity also affect the ratios.

The current replay differs substantially from the older source measurements:
maximum original-encoder disagreement is **943.55 degrees**. That is disclosed
as drift, not accepted as source-engine parity or calibration accuracy. The
comparison is fair because every current mode produced the exact same new
trajectory and observations. None of these timings should be compared directly
with the older source job's 776-second complete wall time.

The encoder-boundary audit checked that all 1,060 nonempty source requests and
responses in this prefix occur at the same step. The final harness also defers
encoder evaluation until the saved response boundary for traces where the live
service drained queued motion after request arrival. Its verification run is
retained separately from the four-repeat timing matrix: all four additional
200,000-step trials reproduced the main trajectory, schedule and event hashes.

When changing backend, solver, scene or collection policy, repeat the numerical
checks and report any changed workload before claiming recording speedups.
For full-autocal comparisons, also check actual steps, commands, encoder reads,
collected points/datasets, stages/iterations and terminal decisions. Matching
CLI options alone does not establish equivalent work.

## Evidence and reproduction

- [Raw matrix and equality gates](../output/research/sessions/2026-10-15-autocal-recording-modes/matrix/results.json): per-trial timings, bytes, hashes, event counts, host and native artifact metadata.
- [Final harness verification](../output/research/sessions/2026-10-15-autocal-recording-modes/revalidation/results.json): matching hashes for all four modes; separate from the headline timing sample.
- [Frozen inputs and source contract](../output/research/sessions/2026-10-15-autocal-recording-modes/matrix/input/contract.json): prefix and original trace SHA-256, original scene and source manifest beside it.
- [Research report](../output/research/sessions/2026-10-15-autocal-recording-modes/report.md), [notebook](../output/research/sessions/2026-10-15-autocal-recording-modes/research.md) and [abstract](../output/research/sessions/2026-10-15-autocal-recording-modes/abstract.md): hypotheses, method audit and limitations.
- [Original accepted job](../output/research/sessions/2026-10-10-supervised-full-autocal/native/autocal/2ec0b06774624962a18bdbd2818eb134/sweeps.headless/manifest.json): source workload only, not a performance baseline.
- [Python orchestrator](../tests/benchmark/autocal_recording_modes.py) and [Node replay](../tests/benchmark/autocal_recording_modes.mjs): reproducible harness. The measured versions are archived in the session's `harness-v1/`; `matrix/working.diff` and `results.json` identify the dirty checkout and hashes at measurement time, based on Git `aba370774108cd856e88b524e4b65658021b4d7c`.

From the repository root, with localhost sockets available:

```bash
.venv/bin/python tests/benchmark/autocal_recording_modes.py \
  --source-headless output/research/sessions/2026-10-10-supervised-full-autocal/native/autocal/2ec0b06774624962a18bdbd2818eb134/sweeps.headless \
  --output output/research/recording-mode-comparison-new \
  --steps 200000 --repeats 4
```

Output directories must be new. Equality failures abort the comparison and
preserve individual trial/checkpoint/recording evidence. To measure another
workload, adapt its scene/layering and generation contract explicitly; this
harness rejects resets and layering changes rather than silently changing them.
