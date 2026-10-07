# Collection throughput, 2026-10-07

Continuing research collection now defaults to `headless-js`, using the production
JavaScript physics pipeline in Node. The Python engine remains available for
differential research. This changes execution backend, not the physics timestep,
solver iterations, force settings, winding, encoder semantics or collector.

## Repeated controlled movement

Measured on an Intel i7-12700K with Python 3.12.3. Each case uses a fresh world,
1,000 fixed 2 ms steps, an A-axis ramp followed by a B-axis torque transition,
and three sequential repetitions. Wall time includes numerical observations and
recording finalization; construction is excluded. Encoders repeat exactly within
each backend. Recording uses the existing 10 Hz plus encoder-observation policy.

| Scene / backend | Rerun | Median wall seconds | Simulation / wall time |
| --- | --- | ---: | ---: |
| HP4 / Python | Yes | 20.709 | 0.097× |
| HP4 / headless JS | Yes | 1.088 | 1.84× |
| HP4 / headless JS | No | 0.773 | 2.59× |
| HP3 / headless JS | Yes | 0.317 | 6.32× |
| HP3 / headless JS | No | 0.207 | 9.65× |

The matched HP4 recording comparison improves throughput **19.03×**. These are
short controlled-motion benchmarks, not timings of fresh collector runs. A
separate 20-second HP3 uncommanded settling smoke test reached 20.93× without
Rerun; it is a different workload and is not the sustained collection claim.

## Frozen real-collection replay

The earlier Python production collection used 80,415 steps / 160.83 simulated
seconds and took 1,693.12 wall seconds. Its actual firmware schedule and encoder
observation times were replayed through the new adapter:

| Replay mode | Wall seconds | Simulation / wall time |
| --- | ---: | ---: |
| Headless JS with Rerun | 78.428 | 2.05× |
| Headless JS, numerical evidence only | 53.015 | 3.03× |

Both modes evaluated all **1,120 recorded encoder reads**. Maximum encoder
difference from Python was **0.002242°**, passing the existing 0.01° endpoint
tolerance. At matching recorded observation times, maximum effector position,
cable length and segment-force differences were 0.014464 mm, 0.041146 mm and
0.086125 N. Those additional metrics are reported separately; the pass flag
specifically denotes encoder parity. No hardware accuracy claim follows.

For the recorded replay, physics itself took 46.265 seconds, physics plus worker
transport 48.772 seconds, and numerical/Rerun observation logging 27.104 seconds.
For the numerical replay, observation logging took 1.562 seconds. Thus Rerun is
a significant remaining cost after the engine change. The original Python
profile attributed about 98% of measured time to world updates.

These are frozen-schedule replays: they exercise real physics, commands, encoder
reads, observation mirroring and recording, but do not run a fresh firmware
planner or let the collector choose a new settling schedule. Do not describe
78.428 seconds as a measured replacement for the original complete collection.
This session's terminal sandbox denies socket creation, so a replacement HTTP
supervisor and its slow end-to-end tests could not be launched. The existing
supervisor retains its already-loaded Python code until the next launcher start.

The browser tool separately advanced 10,000 HP4 steps / 20 simulated seconds in
3.75 wall seconds including the tool request, with recording disabled. That
tests actual browser stepping; it is not a timed browser collection.

## Reproduce and use

```bash
./hp-sim5-research-agent --machine hp3
./hp-sim5-research-agent --machine hp3 --viewer none --no-record
./hp-sim5-research-agent --physics-backend native-python

.venv/bin/python scripts/replay_collection.py SCENE_USDA EVENTS_JSONL \
  --output output/research/replay-recorded
.venv/bin/python scripts/replay_collection.py SCENE_USDA EVENTS_JSONL \
  --output output/research/replay-numeric --no-record
```

Output directories must be new. Failed replays preserve evidence. The numerical
mode retains command/sensor JSON and manifests; its RRD artifact is absent.
Status and collection manifests name backend, machine design, fixed-step clock,
physics time, worker time, observation time, total time and realtime factor.
JS source hashes and worker health are reported. HP3 selects its scene and
1:1 mechanical-advantage RRF configuration together; HP4 retains its own config.

The JS worker yields between batches of at most 50 unchanged physics steps.
Cancellation reports the completed fixed-step boundary and prevents subsequent
execution. Encoder reads drain both pending adapter and worker commands. Python
mirrors only detached fields needed by existing telemetry; it never advances
the mirror. Numerical and recording modes produce identical encoder results.

Fresh `run_experiment` still uses Python and remains slow. The new backend does
not repair Python's hot solver loops. Further work should measure fresh HP3/HP4
collections after restarting the supervisor, then profile recording and expose
fast fresh trials with an explicit observation-rate contract. Calibration
identifiability, hardware fidelity and model mismatch remain separate issues.

Evidence is retained in
`output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/`: `P01-profile.pstats`,
`P02-matrix.json`, `performance_matrix.py`, `P03-final-recorded/replay.json`,
`P03-final-numeric/replay.json`, `P03-rerun.png` and `tooling-tests.txt`.

## Verification and pending checks

The final Python run passed 345 tests, including all 300 autocal tests; nine
were deselected (eight opt-in slow tests and one MCP stdio test). After the last
worker-lifecycle change, the focused native collector and terminal tests passed
13 tests, with five slow tests deselected. Production JS collector, RRF bridge,
flight recorder and browser research-control suites passed 19 tests across four
suites. Compilation and `git diff --check` passed.

The stdio initialization test times out in this terminal environment. Repeating
initialization with the unchanged HEAD MCP entry point reproduces the timeout;
this is retained in `baseline-stdio.txt`, rather than reported as a passing test.
Two dummy-service unit tests originally failed because they allocated real ports
in the socket-restricted sandbox; they now mock port allocation, matching their
existing service mocks. Fresh HTTP/firmware collection lifecycle and throughput
must still be verified outside that restriction. `tooling-tests-final.txt`,
`tooling-lifecycle-final.txt` and the retained earlier failures record the scope.
