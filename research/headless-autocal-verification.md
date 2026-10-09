# Headless autocal verification

The implementation uses the production cable baker, machine scene pipeline,
simulation system registration and command/encoder controller in Node. The
ordinary collector CLI uses a remote simulation clock; the full-auto fitting,
selection, recovery and patience logic is unchanged. No browser or independent
Python physics engine runs during headless calibration.

## Verification contract

`autocal/tests/test_headless_sim.py` includes two extended tests:

- A fresh HP3 dataset with global radius fitting and the example bounds,
  default collection and fitting settings, no shortened sweep count, manual
  stop or acceptance threshold. It must collect beyond bootstrap, reach normal
  automatic acceptance, apply/read back M669 and M666, and stop owned services.
  This same test runs independent matched collection through Chromium and Node:
  three paired sweeps, ten points per direction, sixty measurements per backend.
- SIGTERM during a real bootstrap after a completed measurement. The journal,
  interrupted manifest and final simulation time must survive, and all owned
  services must exit.

For matched collection, both fresh worlds use identical firmware config,
forces, 300 mm span, spool overrides, role configurations, default sampling and
fixed-step waits. Forces come from the full-auto run. Holding these inputs fixed
separates backend parity from independent adaptive tuning/span decisions.
The browser uses the production ECS/system pipeline and actual external command
WebSocket controller in Chromium, with bounded stepping controlled by the
collector clock rather than animation frames. This verifies physics/collector
measurements, not UI rendering, browser workers or foreground-tab scheduling.

RRF formats encoders to hundredths of degrees. The measurement contract allows
0.03 degrees for raw angles and noise means/deviations, 0.01 mm for lengths and
setpoints, and one 2 ms physics step for sample durations. Field shapes, sweep
roles, point counts and sample metadata must match. Simulation timestamps must
be finite and monotonic. Absolute timestamp offsets are diagnostics: small
differences can make an independent settling trial cross a quiet-window
boundary later. No wall-time criterion decides physical settling.

```bash
.venv/bin/python -m pytest autocal/tests/test_headless_sim.py -q -m slow
npx jest --runInBand autocal/control/tests/primitives \
  autocal/control/tests/behaviors autocal/control/tests/cli
```

## Local results, 2026-10-08

The final fresh HP3 global-radius run completed normally after nine iterations
and eleven sweeps (220 points), applying a global radius of 39.02 mm and
Q=0.636619. It used 6487.832 simulated seconds and 435.65 wall seconds, including
fitting. Both parameter commands were read back from firmware. The solver
reported an ideal fit score of 1.039. The fitted D height, 1852.29 mm, differs
from the preset's nominal 1900 mm; a low fit score does not by itself establish
parameter accuracy or hardware validity. Earlier trials, before routing the
size-tuning ramp wait through the simulation clock, stopped after six iterations
and eight sweeps with radius 39.16 mm and a concerning score of 25.01. Their
evidence is retained rather than substituted for the final run.

Initial independent adaptive Chromium/Node runs selected different spans and
failed a strict parity comparison (up to 1.60745 mm length difference). Matched
forces/span reduced observed differences to 0.02 degrees raw angles, 0.005236 mm
lengths and 0.01 degrees noise means. A 1000 ms timestamp offset followed an
independent settling boundary. The first assertion of identical timestamps
failed; the documented contract separates these offsets from measurement error.
The final extended suite passed both tests in 573.81 seconds, including the
clock-corrected fresh full-auto run, matched-input parity and interruption
cleanup. Its [parity result](../output/headless-verification/final-clock/test_fresh_hp3_full_auto_globa0/browser/parity.json)
and [full-auto manifest](../output/headless-verification/final-clock/test_fresh_hp3_full_auto_globa0/hp3/sweeps.headless/manifest.json)
record these bounds and the source hashes.

## Extended reference recording, 2026-10-09

`autocal.py --headless-sim --extended-reference` now uses the production flight
recorder protocol in Node. Autocal owns the disk-only recorder by default;
`--extended-reference-ws` selects an external recorder. Physics, headless command
boundaries, collector requests/replies and measurements, Python text writes and
stage artifacts share one RRD across resets and collector processes.

The fresh HP3 global-radius check reached normal automatic acceptance in
1525.41 wall seconds and 5354.192 simulated seconds. It collected eight sweeps
and 160 points across six collector processes, applied/read back M669 and M666,
and finalized one 1,320,323,251-byte RRD with 267,711 sampled physics observations
and 616,025 events. The default stride was ten with compact geometry. No messages
were rejected. The RRD retains all 160 measurement events, paired outcomes for
13,212 collector G-code requests, seven numerical stage artifacts, exact final
dataset/text/JSONL files and the exact concatenated timestamped text-log writes.
Source clocks were checked separately from wall time. See the
[RRD verification](../output/headless-extended-verification-no-ping/test_fresh_hp3_full_auto_globa0/hp3/rrd-verification.json)
and [run manifest](../output/headless-extended-verification-no-ping/test_fresh_hp3_full_auto_globa0/hp3/sweeps.headless/manifest.json).

The solver reported score 7.245, global radius 39.01 mm and D height 1835.96 mm.
This verifies recording and execution, not calibration accuracy. Later source
label/provenance additions have separate live lifecycle checks; each trial's
hashes/provenance describe the code actually loaded for that trial.

Two earlier full runs exposed a drain wait that required the whole event stream
to become empty and a backlog-sensitive transport disconnect. The physics wait
now resumes when sample capacity is available, with a progress-based
acknowledgement watchdog. Extended transport uses that watchdog instead of
WebSocket keepalive. Partial RRDs and failed trials remain in the
[validation notebook](../output/research/headless-extended-autocal-validation/research.md).
A real SIGTERM after the first bootstrap measurement passed in 53.44 seconds,
retaining partial points and a finalized incomplete RRD while stopping services.
Live checks also cover an external recorder, every-step/full geometry, explicit
25× clock differences, reset generations and recorder disconnection failure.

The combined slow suite's archive/automatic-completion assertions passed, but
its subsequent browser comparison failed at one point: 0.16 degrees and
0.0837758 mm, against unchanged 0.03-degree/0.01-mm tolerances. A focused rerun
and a diagnostic using the original `HEAD:scripts/autocal_headless.mjs` reproduced
exactly the same differences with the recorded run's force inputs. Other encoder
differences were at most 0.01 degrees. This is a pre-existing comparison issue
for those inputs; recording verification does not establish universal backend
parity. See the [original-service baseline](../output/headless-extended-parity-baseline/parity.json),
[baseline source identity](../output/headless-extended-parity-baseline/baseline-source.json)
and [current-service repeat](../output/headless-extended-parity-repeat/parity.json).
The final fast Python regression run passed 78 tests; the targeted JavaScript
regressions passed 104 tests. The slow suite remains one failure (parity) and
one pass (interruption); no tolerance was relaxed.

The live interruption test passed and retained partial points while stopping
owned services. Fast Python checks passed 75 tests; collector regressions passed
55 tests across 13 suites, including motion, settling and noise sampling with
the simulation clock. The two extended checks passed too.

Artifacts are retained in [headless verification output](../output/headless-verification/).
The [notebook](../output/research/headless-autocal-validation/research.md) records
failed trials, tolerance decisions and completed work. Per-run manifests contain
the actual scene/config/source hashes and clock bounds; earlier failed runs
remain in numbered directories.
