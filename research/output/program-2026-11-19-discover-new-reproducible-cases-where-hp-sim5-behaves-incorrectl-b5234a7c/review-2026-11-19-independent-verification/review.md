# Independent verification review

Reviewer: fresh adversarial review, no part in producing the work. Checkout
`/home/torbjorn/repos/hp-sim5` at `56a66f77d3bfa58e52621d06f6273d5a1f3501f5`
(HEAD; matches both sessions' revision claim). Node v26.13.0. Every reproducer
was re-run here; nothing in the product tree or the researchers' session
directories was modified. Scratch scripts: `verify_b1_sweep.mjs`,
`verify_b3_reachability.mjs`.

## How the claims were checked

1. Re-ran all 11 reproducers and both control suites; all exit 1 (bug present)
   / 0 (controls) as claimed, with the same numbers.
2. Re-derived every headline number by hand (polyline lengths, step counts,
   sample shares) and, where the number depends on product code, re-ran the
   shipped module through a second entry point (Cluster A: read the worker's
   own command stream; Cluster B: `QualityMonitor` public API only).
3. Read the shipped code at every cited line, plus the RRF (`RRF/`) and Klipper
   (`klipper/`) trees that are vendored in this repo.

## Cluster A — G-code Upload path

The Upload path is real and reachable: `commandJobController.handleUpload` →
`detectFileFormat` (`.gcode`/`.gc` → `FileFormat.GCODE`,
`integrations/shared/fileFormatUtils.js`) → confirm dialog →
`workers.ensureMoveWorker()` → `worker.postMessage({type:'filename_upload',
filename:file})` → `moveCommander.js:438-443` → `commander.run(file.stream())`.
The harness calls the same `run()` on the same class with a `ReadableStream`;
the `globalThis.self`/`postMessage` stubs only bypass the message dispatcher
and do not touch the parsing/motion code. `dt = 0.002 s` matches
`public/usd_scenes/hp4.usda` (`timeCodesPerSecond = 500`) and the app's
`set_dt` message. **One emitted command = one physics step is correct**:
`remoteSpoolSystem.update()` dequeues at most one command (`:440-449`) and
`runner.js:228` calls `world.update(dt)` once per 2 ms tick. The time model
(`steps × dt`) is therefore the app's simulated duration, not an invention.
The reconstruction of positions from motor angles uses the worker's own
`pos_to_motor_pos_samples_deg` with the same config, so it recovers the
commanded trajectory exactly; a physical run could add tracking lag but cannot
recover 10 mm of motion that is never commanded.

**F1 `G91` ignored — CONFIRMED.** travel 10.000 mm vs 20.000 mm, end X
10.000 vs 20.000, 501 vs 1000 steps, time 1.002 vs 2.000 s; overlay 20.000 mm
(2 segments) from the app's own `parseGcodeText`. The relative-mode square
check is also exact: 52.426 vs 40.000 mm, end (0, −10), 10.000 mm max path
deviation. The second 1 mm of E really is emitted as a single stationary
`Move` (`num_steps === 0` branch, `moveCommander.js:308-328`).

**F2 `G0` dropped — CONFIRMED.** 28.284 vs 40.000 mm, 5.000 mm max deviation
from the commanded polyline, time 2.832 vs 2.1333 s; final position coincides
only by construction. F2b (F word on a `G0` line) confirmed: 0.600 vs 0.0667 s
(9.0×). `G0` is handled identically to `G1` in RRF (`GCodes2.cpp:228-229`) and
Klipper (`gcode_move.py:37`), so the expected values are firmware-grounded.
One wording nit: the overlay does not *draw* the rapid legs (it only emits
segments with `extrusionDelta > 0`), it only tracks position through them —
so its travel is 20 mm, not 40 mm. The table's 20 mm overlay figure and the
contradiction claim are still right.

**F3 `G01`/`G00` dropped — CONFIRMED WITH CORRECTION.** 0.000 vs 20.000 mm,
0.000 mm E, 0 steps; overlay 20.000 mm; `G00` sub-case 14.142 vs 20.000 mm,
5.000 mm off path. The headline survives the Klipper objection: RRF parses the
number numerically (`StringParser.cpp:988-1029`) and executes `G01` as `G1`;
Klipper's string-keyed table (`gcode.py:196-212`) answers `Unknown
command:"G01"` and executes nothing. So *both* firmwares produce zero motion,
but Klipper **tells the user** while hp-sim silently finishes the print. The
correct strength is therefore "hp-sim silently succeeds where RRF moves and
Klipper errors", not "hp-sim loses motion the machine would have made" for a
Klipper-configured Hangprinter. The report says this in its Limitations but the
table row ("`G01` numeric spelling … wrong motion") reads stronger than the
evidence for Klipper.

**F4 `N…*` framing dropped — CONFIRMED (latent).** 0.000 vs 20.000 mm, overlay
0 segments. Valid RS-274; both firmwares strip `N` and `*`. But no slicer emits
line numbers, and both app parsers agree with each other here (no in-product
contradiction) — it is a silent 100 % loss on an input almost nobody uploads.

**F5 `G20` inches — CONFIRMED WITH CORRECTION.** Travel/position claim exact:
1.500 vs 38.100 mm, overlay 1.500 mm (so no in-product contradiction).
**The extrusion sub-claim is wrong.** RRF converts *E* from inches as well
(`GCodes.cpp:1985`, `gb.ConvertDistance(eMovement[0])`), so `G1 X1 E1` in inch
mode commands 25.4 mm of filament, not 1 mm: commanded E is 50.8 mm, and the
commanded density is 50.8/38.1 = 1.333 mm/mm — *the same* density hp-sim
produces (2/1.5). The printed part is 25.4× too small in every dimension, not
"25.4× denser". The harness `oracle()` repeats the same mistake (it does not
scale `eDelta` by `st.unit`), which is why the reproducer prints "extrusion 0 %".

**F6 `G2`/`G3` dropped — CONFIRMED.** 10.000 vs 25.708 mm (25.7017 with the
oracle's 0.5 mm chords), E 1.0 vs 2.0 mm; mid-file `G3` case 28.284 vs
51.413 mm with 9.993 mm max deviation and the accumulated E dumped on the
diagonal. Overlay agrees with the simulation, so the report is right that there
is no in-product contradiction here.

**F7 `G4` dwell — CONFIRMED (timing only, latent).** 2.000 vs 3.000 s. The
1000 ms dwell is entirely absent; a `G4` emits no command, so no step is
consumed.

**F8 default feedrate 1000 mm/min — CONFIRMED (timing only, latent).**
1.200 s vs 0.400 s (RRF `DefaultFeedRate = 3000`, `Configuration.h:220`) and
0.800 s (Klipper `self.speed = 25.` mm/s, `gcode_move.py:48`). The value is
undocumented in `hp-sim-3d/README.md`; every shipped preset and every slicer
file sets `F`, so the divergence needs a hand-written file with no `F`.

Citation errors in Cluster A (do not affect any number):
* `klipper/klippy/gcode.py:46` for `register_command('G0', self.cmd_G1)` —
  it is `klipper/klippy/extras/gcode_move.py:37`.
* `gcode_move.py:55` for the 25 mm/s default — it is `:48`.
* `measurements.json → sources.sha256` is a list of *paths*, not hashes; the
  report's "sources pinned by SHA-256" is not true of the artifact as shipped.
* F3's RRF citation `StringParser.cpp:985-1032` is a loose range (the numeric
  parse is `988-1029`); harmless.

## Cluster B — quality report

Same reachability chain applies: `QualityMonitor` is fed by
`RemoteSpoolSystem`'s extrusion events and finalised by
`commands.finishJob()` → `runFinalQualityChecks()` when the worker drains
(`appBootstrap.js:197`, `commandJobController.js:123-137`,
`qualityController.js:234-237`). The stub HUD only receives rendered HTML.

**B1 corner overshoot reports the rigid offset — CONFIRMED.** Reproduced
exactly: `cornerOvershootAvg/Max = 1.00/1.00 mm`, `settleDistanceMax = 0.10 mm`,
`Mean offset X = 1.00 mm`, `Quality 90`; closed form after removing the mean
offset is 0/0. The module's own contract is explicit (`quality-monitor.js:128-131`
"after removing the mean offset", `:681` "not the offset"). Code path verified:
`_updateOffsetEstimate()` is called at `:646` before the sample's normal
equations are accumulated at `:659-667`; the ridge keeps the unconstrained
direction at zero, so the first sample of the new direction is scored with
`_shapeDistance ≈` the full offset (`:773`). My independent sweep reproduces
the researchers' 9/15 trigger pattern and the perpendicular control (0.000 mm,
score 97.156). Score difference 7.358 points reproduced.
*Fragility (they disclose it, I confirm it):* the trigger is a float knife-edge
in the step count. At a realistic HP4 density (2 ms steps, 600 mm/min →
2500 steps/leg) the value does **not** fire (0.0025 mm); 2499 or 2501 steps
does (1.000 mm). So the defect is real and deterministic per input, but roughly
half of step counts show ~one sample step instead of the offset.

**B2 "Path within 0.50 mm" is a time share — CONFIRMED.** 52.6 % vs 10.0 %
path-length share; fine tolerance also 52.6 %. The value is
`withinToleranceCount / sampleCount` (`:822`), samples are one per 5 ms-ish
physics step (one per consumed command), and the HUD label is `Path within …`
(`:934`). Arithmetic checked independently: 400/760 = 52.63 %, 10/100 = 10 %.
*Corrections to the presentation:* (a) the table's "expected 0.95 mm" for the
RMS is a path-length-weighted RMS; the per-sample RMS the code reports,
0.688 mm = sqrt(360/760), is the correct value for the metric it actually
computes, and the verdict correctly does not count it as an error — the table
should not juxtapose them as expected/reported. (b) "Steps are 5 ms of sim
time (`moveCommander.js:33`)" is wrong for the shipped app: `:33` is only the
fallback, and `set_dt` overrides it from the scene (HP4 = 2 ms). The finding
and all percentages are invariant to that (the density ratio is the speed
ratio), so nothing downstream changes.

**B3 Quality 100 on a third of a print — CONFIRMED WITH CORRECTION.** The
displayed result reproduces: score 100.0, "Score deductions: none",
`Progress 33.3 %`, 0 corners. `cornerError`/`settleDistance` are hard-wired to
0 when `cornerCount == 0` (`:859-860`) and `SCORE_TERMS` has no progress term,
so this is a structural gap, not an arithmetic error. I also checked
reachability, which the report leaves implicit: an aborted print does **not**
produce this card (`handleUserReset` resets the monitors without
`runFinalQualityChecks`), but a *reference/execution mismatch* does. I drove
the shipped chain end to end — `parseGcodeText` on a 3-leg file whose last two
legs are `G0 … E…` lines, the real `MoveCommander`, the real `QualityMonitor` —
and got reference 150 mm / 15 mm E, simulation 50 mm / 5 mm E, `progress 33.3 %`,
`cornerCount 0`, `score 99.9999999999`. So the scenario is reachable through
the shipped Upload path, but only via the Cluster-A class of bug (or enabling
quality mid-print); it is not what a normal completed print produces. The
`Progress` field does disclose the truncation. "Misleading" is the right
strength; "Quality 100 on a third of a print" overstates how often a user gets
there.

## Genuinely latent / low-reachability claims

* F4 (`N`/checksums) — valid G-code, but no slicer emits it; both app parsers
  agree, so nothing contradicts the user.
* F5 (`G20`) — rare for FFF; Klipper rejects it loudly. Core number holds.
* F7 (`G4`) — timing only; slicer output rarely contains dwells.
* F8 (default feed rate) — needs a file with no `F`; all presets set `F`.
* B3 — requires executed E < reference E; disclosed by `Progress`.
* B1 — input-dependent (offset must have a component along the first move,
  and the step count must land the corner sample on the "after" side).

## Reproducers that are fragile or stub-dependent

* B1's reproducer is inherently fragile: the headline number flips between the
  full offset and ≈one sample step with the step count (2500 vs 2501
  steps/leg). It is not stub-dependent (HUD only), but the report should state
  that a "typical" HP4 step density may not trigger it.
* The Cluster-A harness's `globalThis.self`/`postMessage` stubs are faithful:
  `run()` is the same entry point the `filename_upload` message calls, and
  `setAsapMode(true)` affects pacing only, not the emitted command stream.
  No finding depends on a stub.
* Cluster A's "simulated = what the machine actually does" is the *commanded*
  trajectory (exact for motor angles), not a physics solve with cable
  compliance; the report calls a physics run "only solver noise", which
  overstates it, but no headline number depends on that idealisation.
* Cluster B's "5 ms steps" premise is wrong (app dt is 2 ms for HP4); every
  reported number is dt-invariant, so this is a documentation error only.

## Overall

All eleven headline findings reproduce on this checkout. Two require
correction in their supporting claims (F5 extrusion/density; F3's Klipper
framing), one headline is confirmed but its reachability needed an end-to-end
demonstration (B3), and three are latent for ordinary slicer uploads (F4, F7,
F8). No finding was refuted.
