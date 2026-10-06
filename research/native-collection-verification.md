# Native collector verification, 2026-10-06

The first milestone passes: the existing collector runs real RRF planning against
one continuing native HP4 world, returns responsive version-2 measurements, and
loads them through autocal. The launcher supervises the runtime, firmware,
WebSocket bridge, optional Vite and Rerun outside the researcher's command
sandbox. Calibration accuracy remains unproven by this short preflight.

## Measurement experiment

Hypothesis: the production collector can obtain actual native encoder feedback
with browser-equivalent semantics, and increased preload can overcome the
stationary sensor observed on the authored HP4 scene.

Both runs start from the authored scene at 0.002 s timesteps, with configuration
`fixed=[2,3], drive=0, sensor=1`, three points per direction, four noise samples,
fixed deltas `0,0` mm and low/mid/max forces `0.01/0.05/0.5` N. The physical trial
changes sensor preload from 0.25 N to 1 N. A separate parser fix between runs
preserves all three coordinates in RRF's comma-separated M669 report; baseline
residual evaluation uses coordinates captured in its reply. Source hashes differ,
so these are not identical software builds.

| Observation | Baseline: 0.25 N | Trial: 1 N |
| --- | ---: | ---: |
| Run | `7171d8122bdd4f27b92f351b5a297475` | `bde3f48929e942dfa6587d60c6eafb3e` |
| Measurement points | 6 | 6 |
| Physics steps | 29,322 | 42,425 |
| Simulation seconds | 58.644 | 84.850 |
| Collection wall seconds | 517.530 | 832.783 |
| Physical sensor span, forward / reverse (mm) | Forward sensor stationary | 33.178 / 28.950 |
| Firmware-model residual RMS (mm) | 12.912 | 0.13405 |
| Maximum browser/native encoder error (degrees) | 0.006168 | 0.002246 |

The baseline's old manifest says complete, but the physical-response check added
subsequently rejects it. Its B encoder remained at 128.63 degrees while A swept.
JSON validity alone was insufficient. The selected implementation defaults to
1 N and requires measurable physical sensor response in each direction.

Evidence: [baseline manifest](../output/research/native-probe2/db3d704e2b42457f8e1a9c50f1e98059/7171d8122bdd4f27b92f351b5a297475/manifest.json),
[baseline replay](../output/research/native-probe2/browser-replay.json),
[trial manifest](../output/research/native-preload-trial/e3c99789fe704e7b9bf9a154a869d85e/bde3f48929e942dfa6587d60c6eafb3e/manifest.json),
[trial replay](../output/research/native-preload-trial/browser-replay.json).
The trial is an immutable copy from pytest's temporary directory; original
manifest paths are retained. All raw artifacts are local and ignored by Git.

## Doctor and independent replay

Actual `./hp-sim5-research-agent --doctor` exited successfully with collection
`9c91f1b45e3a45f392f833ccc89cb376`, session
`ab4d67c4e69d49aa88c35c87f33ca75f`. It repeats the responsive six measurements,
checks schema, canonical roles, reference-relative raw angles, millimetre
conversion, noise statistics and physical travel, and evaluates every point
through autocal. It then resets firmware and physics and cleans up services.
Collection took 820.171 wall seconds for 84.850 simulation seconds, including
telemetry and recording; this is not a physics benchmark.

Production JavaScript scene loading, physics and the browser encoder endpoint
replayed the doctor's frozen scene and actual firmware schedule for 42,425 steps.
All 434 encoder reads passed a 0.01-degree tolerance; maximum error was
0.0022457568 degrees. This checks numerical endpoint parity under the same
schedule, not independent browser collection or browser rendering.

The returned 175,311,618-byte RRD is finalized: `RrdReader.store()` and blueprint
loading succeed. Recording rotation preserves the world and leaves previously
returned collection evidence immutable.

Evidence: [doctor result](../output/research/native-doctor.json),
[collection manifest](../output/research/preflight/f6ed2a82ea6648ea849cafb13acff2aa/ab4d67c4e69d49aa88c35c87f33ca75f/9c91f1b45e3a45f392f833ccc89cb376/manifest.json),
[collector JSON](../output/research/preflight/f6ed2a82ea6648ea849cafb13acff2aa/ab4d67c4e69d49aa88c35c87f33ca75f/9c91f1b45e3a45f392f833ccc89cb376/sweeps.json),
[browser replay](../output/research/preflight/f6ed2a82ea6648ea849cafb13acff2aa/ab4d67c4e69d49aa88c35c87f33ca75f/9c91f1b45e3a45f392f833ccc89cb376/browser-replay.json).
Manifest hashes identify the loaded code; subsequent changes add explicit live
Viewer flushing and clearer telemetry/version documentation.

## Fitting and ground truth limits

A bounded call to the current calibration solver, initialized with firmware
anchors, used one restart and two iterations. It returned optimizer success but
cost 100 and zero valid sweeps. No geometry was identified. Evaluating the
unchanged firmware model separately on whole forward and reverse directions
gives RMS residuals of 0.14250 and 0.12503 mm. Reverse points are excluded from
the forward diagnostic, but this is not holdout validation of a trained model.
More independent sweeps and movements are needed before reporting fitted
anchor/radius error or generalization.

Authored USD spool radii are 30 mm. Reconstructing radii from the recorded
mm/degree values, including mechanical advantages `2:2:2:4` and 1:1 gears,
matches within floating-point roundoff. This proves unit conversion, not radius
estimation. Firmware M669 coordinates are guesses; distributed anchor/pulley
geometry has no unique equivalent point-anchor ground truth.

Evidence: [fit and direction audit](../output/research/preflight/f6ed2a82ea6648ea849cafb13acff2aa/ab4d67c4e69d49aa88c35c87f33ca75f/9c91f1b45e3a45f392f833ccc89cb376/fit-audit.json),
[bounded solver result](../output/research/preflight/f6ed2a82ea6648ea849cafb13acff2aa/ab4d67c4e69d49aa88c35c87f33ca75f/9c91f1b45e3a45f392f833ccc89cb376/bounded-fit.json).

## Live view and regression checks

Rerun Viewer MCP exposes `sim_step` and `sim_time`, the 3D world and five numeric
views. Seeking to step 100 and saving a screenshot succeeded. Visual inspection
shows the HP4 cables and frame, numeric traces and selected cursor. There are no
per-view warnings or errors; logs include a transient initial missing-timeline
warning before data arrives. Long collector advances explicitly flush live data
periodically; interactive steps flush before returning.

Evidence: [live screenshot](../output/research/native-viewer-proof/native-live.png),
[Viewer state](../output/research/native-viewer-proof/rerun_get_viewer_state.json),
[time selection](../output/research/native-viewer-proof/rerun_set_time_cursor.json).

Checks passed:

- Python researcher/session suite: 19 tests; three slow tests excluded by default.
- Real collection, autocal ingestion, persistent readback, reset and cleanup:
  one slow test passed in 837.05 seconds.
- Real stdio MCP, G-code, Vite readiness, coordinated reset and cleanup, plus
  live delivery during collector-style advances and Viewer time selection:
  two slow tests passed in 11.53 seconds.
- JavaScript collector/clock and RRF bridge checks: 54 tests across 15 suites.
- Production JavaScript replay: all 434 encoder reads passed.
- Python compilation and `git diff --check` passed.

See [native collection usage](native-collection.md) for tool contracts and
reproduction commands. Klipper streamed motion and a human control UI remain
future work.
