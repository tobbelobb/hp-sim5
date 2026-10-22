# Fixed-motor slipping corrupts upper autocal sweeps

Trial-004 contained two upper sweeps with visibly irregular paths. Recorded commands and encoders, followed by isolated production JS physics and RRF collector experiments, identified loss of fixed B position: its encoder spanned 193.04 and 200 degrees. Simulator branch tensions exceeded B's modeled holding torque, while the collector accepted stationary encoders after slipping and repeatedly corrected B between points. A reproduced baseline matched the recorded first upper sweep to 0.01 degree. Lower default commands and a fixed-target encoder guard reduce B spans to 0.50 and 0.18 degrees in the two configurations; all six bootstrap directions also complete. A deliberate overload is rejected at 8.01 degrees before saving a dataset, with position-hold commands issued. No runtime force sensing is introduced: tensions are diagnostic simulator evidence only. Eighty-nine collector tests pass. These results establish collection behavior in the tested simulator geometry, not hardware safety or improved numerical calibration accuracy; a fresh visual capture and optimization remain untested.

## What happened

The affected configurations are sweep_004, collected after full-auto iteration 1, and sweep_007, collected after iteration 4. Both hold B and D while A and C sweep, with fixed targets B=0 and D=-489.075 or -464.621 nominal millimeters. The B encoder spans 193.04 and 200 degrees. Final recorded B lengths are 60.319 and 37.694 mm despite a zero target. D stays within approximately half a degree. Original bootstrap fixed encoders span only .15–.21 degrees.

The collector uses the geometry-specific edge force of 12.8849 N both for symmetric positioning and drive pullout, and at least half that force (6.44246 N) for sensor measurement. In the replay at simulation time 2796 s, held B has drifted to 253.37 degrees versus a .225-degree target. Its branch tensions are 7.58 and 5.85 N: at .03918 m effective radius their sum implies approximately .526 Nm against .5 Nm modeled holding torque. B has crossed 35 magnetic detent cycles in this interval. This is genuine modeled slipping, not a trace rendering artifact. Accumulated missed-step counters outside held intervals are not reliable evidence because force-mode transitions retain stale targets.

The settling check asks whether encoders stop moving, so it accepts a motor that has slipped into a new stationary position. Fixed-anchor enforcement then issues repeated corrective B moves, roughly 30–132 mm between points, producing the irregular path and violating the constant-length assumption of a sweep. These points should not be treated as valid reference sweeps.

## Runtime changes and force-data boundary

The shared preload resolver bounds measurement at five times the tuned start command, and positioning/pullout at twenty times that command, each capped by the tuned maximum. In this recording that changes measurement from 6.44246 to 1.13636 N, and pullout from 12.8849 to 4.54543 N. Explicit sensor preload overrides are retained. Metadata now reports the actual measurement and pullout commands; the old `sensor_force_n` recorded .22727 N despite commanding 6.44246 N.

These values are commanded forces. The start/edge tuning procedure uses encoder motion, not measured cable tension. No simulated tension, modeled holding torque, or new hardware sensor is consulted by the runtime changes.

Every valid settling poll in a sweep checks the fixed encoders against their requested targets, converted using the collector's existing nominal mm-per-degree mapping. A deviation above 1.5 degrees fails collection immediately, including a quiet encoder after a slip. Points are checked again before acceptance. The existing CLI error handler requests position mode for all motors on failure. This is a position-integrity check, not a real-time force limit: it detects failure after displacement and cannot promise that a command is safe before displacement occurs.

## Experiments

Tests launch their own RRF simulator and production JS physics world with the recorded HP3 scene, layering and friction-radius settings. They use production command translation, encoder conversion and collector CLI on separate ports. No user browser or existing services are changed. The supplied auto-tuned force values are replayed rather than repeating tuning, and bootstrap travel is set to the recorded 489.075 mm. Each collector subprocess has a 300-second wall limit. Both directions contain ten accepted points.

| Experiment | Accepted points | Fixed B span | Outcome |
| --- | ---: | ---: | --- |
| Original collector, first upper target | 20 | 193.03 degrees | Reproduces bad recorded sweep |
| Final patch, first upper target | 20 | .50 degrees | Fixed D span .32 degrees |
| Final patch, second upper target | 20 | .18 degrees | Fixed D span .22 degrees |
| Final patch, three bootstrap configurations | 60 | At most .18 degrees across held axes | All six directions complete |
| Final patch, explicit old 6.44246 N sensor command | No completed dataset | Guard trips at 8.010 degrees | Four position-hold commands acknowledged |

The bootstrap takes 2066.976 simulated seconds and 121.39 wall seconds in the controlled test service. A pullout command of five or ten times start force also kept the upper geometry intact but failed the bootstrap's 120-second travel deadline; those failed trials remain preserved. The twenty-times-start default balances successful travel against held-motor load for the tested geometry. It is not a universal hardware force bound. Runtime timing continues to use the observed clock and independent wall deadlines, without assuming a requested speedup was achieved.

![First upper sweep trajectory and fixed B encoder comparison](upper-sweep-comparison.png)

The plot shows the first direction during collection; initial positioning and later return-to-origin are omitted. The two experiments have different travel extents under their different applied forces. The patched path preserves fixed B; collection tests do not prove optimal coverage of the workspace or calibration accuracy.

Collector regression tests: `npm test -- --runInBand autocal/control/tests tests/js/sweep_data_collection.test.js`: 18 suites, 89 tests pass, including quiet slipped-encoder rejection, free-axis movement, force overrides/caps, measured-clock waits and CLI handling. The high-force physics test uses simulator tensions only for diagnosis; rejection and cleanup depend solely on the fixed encoder deviation.

## Fit comparison

Using `parse_log_file`, `error_to_true`, `SIM_3D_TRUE_ANCHORS` and `SIM_3D_TRUE_RADII` from the existing regression tool:

| Fit | Logged quality score | Sum of anchor errors (mm) | Regression total error |
| --- | ---: | ---: | ---: |
| 1 | 4.659 | 315.772 | 320.409 |
| 2 | 26.29 | 6694.399 | 6699.036 |
| 3 | 29.80 | 3862.772 | 3883.268 |
| 4 | 25.21 | 101.739 | 107.155 |
| 5 | 24.37 | 73.911 | 74.804 |

The user is correct: fits 4 and 5 are closer to the known simulator geometry, although the current fit-quality score selects fit 1. Total error uses the regression helper's sum of anchor and radius errors, not an RMS. The score-ranking behavior is unchanged here. No new full optimizer run was performed; clean collection must be verified separately before assessing resulting calibration accuracy.

## Tested hypotheses and references

See [abstract.md](abstract.md) for outcomes E1–E6 and source links, and [research.md](research.md) for experiment decisions. [sweep-metrics.json](sweep-metrics.json) contains exact fixed spans and overload cleanup payloads. [analyze.py](analyze.py) reproduces those metrics and the plot; [collect-probe.py](collect-probe.py) and [physics-service.mjs](physics-service.mjs) describe the isolated experiments. [replay.mjs](replay.mjs), [commands.json](commands.json), [scene.usda](scene.usda) and [replay-results.json](replay-results.json) preserve the recorded-command replay. Baseline source revision is `adb7cbbb6f755b21b11b69e69c0041449d6b9696`; [working.diff](working.diff) and [fixed_anchor_guard.test.mjs](fixed_anchor_guard.test.mjs) preserve the tested source changes.
