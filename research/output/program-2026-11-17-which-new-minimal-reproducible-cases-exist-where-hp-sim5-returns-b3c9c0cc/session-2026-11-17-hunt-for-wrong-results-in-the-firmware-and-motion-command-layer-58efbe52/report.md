# Wrong results in the firmware and motion-command layer

**Program:** 2026-11-17 "which new minimal reproducible cases exist where hp-sim5 returns wrong results"
**Session:** hunt-for-wrong-results-in-the-firmware-and-motion-command-layer-58efbe52
**Date:** 2026-11-17

## Question

Where does the firmware / motion-command layer of hp-sim5 produce results that are
provably wrong — not merely coarse? Concretely: does the Klipper/RRF
emulation, G-code handling, move planning, limit handling and step/angle
bookkeeping agree with exact closed-form kinematics and with the firmware's own
step stream, for single-axis and two-segment moves and for boundary inputs
(zero/negative feedrate, zero-length moves, huge feedrate, duplicate moves,
sub-step distances, G20/G21, M204/M203/M566 changes, commands arriving while
moving, queue overflow)?

## Verdict in one paragraph

Four genuine defects were found and reduced to one-command repros, all with
deterministic, quantitative errors. Three are in hp-sim5's own emulation code
(`integrations/rrf/`, `integrations/klipper/`) and one is in the fork-only host
code of the RRF firmware. The largest error by magnitude is a
silent **100 % loss of commanded motion** when the RRF CAN capture comes from a
CAN board with more than one motor (10 mm + 5 mm of commanded travel becomes
8.34 mm of phantom extrusion and no motion) — legal configuration, though none
of the bundled machines use it. The most far-reaching is a
**one-microstep loss per stepper at the start of most Klipper MCU streams**
played by the default upload player, reproduced on a checked-in preset.
Separately, the RRF host build's standstill bookkeeping is off by one
microstep, which perturbs subsequent commanded moves, and one non-default
Klipper input path applies extruder steps to a spool axis. On the positive
side, RRF/Klipper velocity, acceleration, feedrate-limit and step-quantization
behaviour matched the closed form to the step clock in every check I could
construct, and the emulator's 2 ms bucketed antialiasing reproduces the exact
trapezoid to < 1 µm after the ramp.

## Scope, revisions, environment

| item | value |
| --- | --- |
| hp-sim5 | `c2450975` (branch `main`) |
| Klipper fork | `klipper/` @ `9ad05951e` |
| RRF fork | `RRF/ReprapFirmware` @ `82f356b45` |
| emulator binary | `RRF/build/rrf_simulator` (prebuilt 2026-11-17, not rebuilt) |
| Python | `.venv/bin/python` |
| Node | v24.14.1 |
| run time | full repro suite ≈ 2 s (`bash repro/run_all.sh`) |

No source file was modified. Everything below runs from the repository root.

## Method

Two harnesses, each driving the real firmware code rather than a model of it:

* `repro/rrf_harness.py` builds a scratch VSD tree, runs
  `RRF/build/rrf_simulator --vsd … --gcode … --can-log … -c sys/<config>.g` in
  batch mode, and parses the CAN motion capture
  (`seq, canAddress, whenToExecute, accelClocks, steadyClocks, decelClocks,
  <one steps column per drive>, acceleration, deceleration`;
  `RRF/host/can/CanCapture.cpp`). Step clock = 750 kHz. G-code replies
  (`M114` → `Count`, the firmware's live motor counter) are parsed from stdout.
* `repro/klipper_harness.py` runs `klipper/klippy/klippy.py <cfg> -i <gcode>
  -o <serial> -d public/klipper/linux_mcu/klipper.dict` (Klipper batch mode =
  the same G-code parser, move planner and step compressor that the simulator
  streams) and decodes `queue_step`/`set_next_step_dir` into signed step counts.

Two configs are used: the shipped `RRF/run/vsd/sys/config_slideprinter.g` (the
actual hp-sim5 machine) and a minimal cartesian config with exactly 80 whole
steps/mm (`repro/rrf_harness.py:CARTESIAN_CONFIG`,
`repro/klipper_cartesian.cfg`) so that "1 step = 0.0125 mm" and every expected
value is exact. `repro/run_all.sh` runs the whole suite; a bug repro exits
nonzero with a one-line reason and prints its measurements; a negative-result
check exits zero only when the layer is exact.

## Findings

### F1 — RRF host build: standstill bookkeeping is off by one microstep, and the next move inherits it

*Repro:* `repro/rrf_standstill_offbyone.py` (`$ .venv/bin/python repro/rrf_standstill_offbyone.py`)

**Observed**, from the real firmware:

| case | G-code | expected | actual |
| --- | --- | --- | --- |
| A, slideprinter, exact return with a wait | `G1 X10 F600` / `M400` / `G1 X0 F600` / `M400` | net 0/0 steps on motors b/c | **−1 / +1 steps** (1 step = 0.0589 mm of line) |
| B, cartesian, 1 mm after a wait | `G1 X10` / `G1 X0` / `M400` / `G1 X1` | 80 steps, X = 1.0000 mm | **79 steps, X = 0.9875 mm** |
| B control, same without `M400` | `G1 X10` / `G1 X0` / `G1 X1` | 80 steps, X = 1.0000 mm | 80 steps, X = 1.0000 mm (correct) |
| C, single 0.05 mm move | `G1 X0.05 F600` / `M400` / `M114` | capture 4 steps = `Count` 4 | capture **4** steps, `Count` **3** |

*Error magnitude / tolerance:* exactly one microstep — 0.0125 mm on the
cartesian config, 0.0589 mm of line per motor on the slideprinter. Tolerance
here is zero: an `M400` should be a pure wait, and a return to the same
coordinate must end at the same motor position. Deterministic (the repro runs
case A twice and compares).

*Why this is a bug and not a simplification:* inserting a wait changes the
**physical step stream** of a later move (80 → 79 steps). The firmware's own
step capture and its own live position counter disagree (case C), and the
difference lands exactly in the movement layer.

*Strongest explanation (code-level):* fork-only host code. Commit `7eb7a97bf`
("Host build: keep motor positions at standstill") added
`Move::FreeOldSegments` (`RRF/ReprapFirmware/src/Movement/Move.cpp:2276`,
guarded by `RRF_HOST_BUILD`). Its not-yet-executing branch advances the live
motor position with a C cast,

```cpp
const int32_t netSteps = (int32_t)(length + dm.distanceCarriedForwards);   // truncates
```

while the real per-segment ISR path it is imitating,
`DriveMovement::TakeStepsAndCalcStepTimeRarely` (same file,
`DriveMovement.cpp:750`), uses `std::lrint(...)`. Truncation of each segment's
length (with the fractional carry kept in `distanceCarriedForwards`) leaves the
live position up to one step off. `GCodes::LockMovementSystemAndWaitForStandstill`
(M400, G4, tool change, probing) then calls
`ms.UpdateOwnedDriveEndpointsFromMotors()`, folding that offset into the
*commanded* user position, so the next absolute move is computed from a
position that is one step away from where the motors actually are.

*Caveat I could not fully settle:* the sign and size depend on the per-segment
fractional residues, so not every move shows it (10 mm at 80 steps/mm in one
move is exact; 0.05 mm is not). I did not single-step the ISR to observe the
interleaving of `FreeOldSegments` and the `ending` branch, so the exact bucket
of cases that trip it is characterised empirically, not derived. The main
competing explanation — a thread race between the host build's Main task and
its Move task at standstill — is weakened by the fact that the offset is
identical across repeated runs and across 100-repetition sequences, whereas a
race would vary; the commit `7eb7a97bf` also already added
`TaskCriticalSectionLocker` around the specific standstill windows it
identified.

*Physical size:* one microstep of line length on two of three motors. `M114`
reports `X 0.102, Z −8.641` after case A, but that Cartesian amplification is a
property of the near-singular 3-coplanar-anchor geometry, not of the step
error; the physical defect is the microstep.

### F2 — RRF CAN capture with more than one motor per CAN board is misparsed: all motion lost, phantom extrusion instead

*Repro:* `repro/rrf_multidriver_csv.mjs` (fixture regenerated by
`repro/make_rrf_multidriver_fixture.py`)

Firmware config: `M584 X40.0 Y40.1 Z40.2 E40.3` (one Duet-style CAN board at
address 40 with four drivers), G-code `G92 X0 Y0 Z0` / `G1 X10 Y5 F600`.
The firmware does the right thing (final position X=10, Y=5) and emits

```
0,40,0,2500,836026,2500,800,400,4.77027862e-10,4.77027862e-10
```

i.e. one row with **two** steps columns (800 = X, 400 = Y) before
acceleration/deceleration.

**Expected vs actual** through `integrations/rrf/rrfCanPlayer.js` +
`rrfFirmwareModel.parseRrfMotorAxisMapFromConfigText`:

| quantity | expected | actual |
| --- | --- | --- |
| CAN address 40 → axis | A (X) / B (Y), per driver | **E** (last axis mentioned on the board wins) |
| X travel | 800 steps = 10.000 mm | **0 steps** |
| Y travel | 400 steps = 5.000 mm | **0 steps** (never applied) |
| extrusion | 0 mm | **8.3401 mm** of phantom extrusion |
| acceleration | `4.77e-10` (normalised, per unit distance / tick²) | **`400`**, i.e. **8.4e11×** too large |

*Error magnitude / tolerance:* 100 % of commanded travel (15 mm) replaced by
8.34 mm of wrong-axis extrusion; 8.4e11× acceleration error. No tolerance can
absorb this.

*Why this is a bug and not a simplification:* the input is legal RRF
configuration (`board.driver` addressing), the firmware emitted the motion, and
the emulator silently maps it to a different axis. Nothing warns.

*Root cause:* two independent assumptions that one CAN address = one motor.
`rrfFirmwareModel.js:74` does `result.set(descriptor.canAddress, spoolAxis)`
and drops `descriptor.driver`; `rrfCanPlayer.js:461-481` (`_parseCsvRow`) reads
a fixed 9-column layout (`parts[6]` = steps, `parts[7]` = acceleration,
`parts[8]` = deceleration) although `CanCapture.cpp` writes one steps column
per drive in the message. The binary `.can` path
(`_parseBinaryStream`) is *also* address-keyed: a body record carries the CAN
destination address (40) and a single value, with no local driver index, so it
cannot separate two motors on one board either (see the follow-up section).

*Strongest alternative explanation:* one could argue hp-sim5 only supports the
bundled configs, all of which use one driver per CAN address. That is true of
the shipped presets, but the parser explicitly accepts the `.driver` suffix
(`parseRrfDriveSpecifier` returns `{canAddress, driver}`), so the format is
advertised, and the failure is silent. I found no documentation that limits a
CAN address to one motor.

### F3 — Klipper MCU player drops the first queue_step batch of every stepper except the first

*Repro:* `repro/js_mcu_player_step_loss_synthetic.mjs` (minimal),
`repro/js_mcu_player_step_loss.mjs public/mcu_commands/slideprinter/draw_squares.txt` (shipped preset)

This is the default path: `.txt`/`.serial`/`.log` uploads and fetched presets go
through `ensureKlipperMcuCommandPlayerWorker` →
`KlipperMcuCommandPlayer` (`KLIPPER_UPLOAD_PIPELINE = 'player'`).

Minimal synthetic stream, two steppers, one step each, both at tick 1 000 000
(2 ms buckets → 100 000 ticks):

| axis | expected | actual (incremental flush, as shipped) | actual with one deferred flush |
| --- | --- | --- | --- |
| A (first parsed) | 1.000 steps | 1.000 | 1.000 |
| B (second parsed) | 1.000 steps | **0.000** | 1.000 |

Real data (`public/mcu_commands/slideprinter/draw_squares.txt`, the checked-in
preset; stream contains exactly 5 steps on each of the three anchor motors):

| axis | stream | player output | loss |
| --- | --- | --- | --- |
| A | −5 | −5.000 | 0 |
| B | −5 | **−4.0521** | **0.948 microsteps = 0.0558 mm of line** |
| C | −5 | **−4.2080** | **0.792 microsteps = 0.0467 mm of line** |

The pattern repeats on the other checked-in presets (same script, other file):
`hp4/draw_squares_bigger_no_buildup.txt` loses 0.973 / 0.741 / 0.600
microsteps on B / C / D; `cubecorners/draw_squares_bigger_no_buildup.txt`
loses 0.958 / 0.768 / 0.640 / 0.549 / 0.405 / 0.320 on B / C / D / I / J / L.
In every preset the first-parsed stepper is exact and every later one is
0.3–1.0 microsteps short.

*Why this is a bug:* the player's own contract is to replay the firmware's
steps. `_flushReadyBuckets` advances a single global bucket cursor
(`nextBucketToEmit`) after every parsed line, while each axis's first
`queue_step` in a batch dump spans the idle time from MCU clock 0 to that
axis's first step and is therefore smeared over the buckets from 0. When the
second and later steppers are parsed, those early buckets were already emitted
and deleted (`_emitBucket` deletes the map entry and `nextBucketToEmit` never
goes back), so the writes are lost. Instrumenting the player confirms it: 127
late writes into already-emitted buckets for axis C totalling 0.9958 steps,
and zero loss when flushing is deferred to the end of the stream.

*Error magnitude / tolerance:* ≤ 1 microstep per axis, once per stream (a
constant position bias, not accumulating). 0.0589 mm of line per step on the
slideprinter; relative error 19 % on the 5-step `draw_squares` move.

*Strongest alternative explanation:* it could be argued that the first
`queue_step`'s huge interval is an artifact of Klipper batch mode and that
live streams start at the first step. Even so, any stream whose first
queue_step has `count > 1` loses those steps, and the synthetic case shows the
bookkeeping is wrong independently of the size of the interval. For the API
path (`api_motion_batch` → `consumeStepperBatch`) the same loss does not occur
because batches are not smeared from tick 0. Note that Klipper emits
`reset_step_clock` for idle steppers; the player ignores those lines, so a
stepper that first moves late in a *live* stream would be scheduled in the
past — I did not find a shipped stream that exercises it.

### F4 — Klipper parsed-line path maps extruder steps to a spool axis (extrusion silently lost)

*Repro:* `repro/js_klipper_oid_axis_mapping.mjs`

Fixture: `G90` / `G1 X10 E5 F600` on the shipped slideprinter config →
firmware emits 478 extruder steps on oid 3 (`step_pin gpiochip1/gpio24`).

| path | spool D | extrusion E |
| --- | --- | --- |
| `KlipperMcuCommandPlayer` (pin map, default upload path) | 0 steps | 4.9936 mm |
| `klipperPacerWorker` → `applyBucketedParsedLine` | **478 steps = 28.16 mm of line** | **0.0000 mm** |
| expected | 0 | 5.0041 mm |

*Error:* the whole 5.004 mm of commanded extrusion is lost and replaced by
28.16 mm of spool rotation; the *shape* is otherwise right (B/C = −147/+147).

*Root cause:* `klipperMotionCore.js:42` (`mapOidToAxis`) maps
`queue_step oid` → `DEFAULT_AXIS_ORDER[oid]`, and
`applyBucketedParsedLine` (line 588) ignores `config_stepper`'s `step_pin`.
The extruder is the fourth stepper (oid 3) in every shipped Hangprinter config,
so index 3 = "D". The other parser for the same stream
(`klipperMcuCommandPlayer.js`) maps by pin and gets it right, so the two
shipped parsers disagree on identical input.

*Scope:* the pacer worker's parsed-line path is reached by raw serial/text
input (`feedSerialChunk` → `processSerialLines`), i.e. the opt-in
`KLIPPER_UPLOAD_PIPELINE = 'raw'` upload, `playKlipperRawFile` and
`connectKlipperRaw`. With the shipped default (`'player'`) preset uploads do
**not** take this path. I classify it as genuine but non-default.

*Strongest alternative explanation:* "the pacer worker only ever sees streams
whose oids happen to match the axis order". That is false for every shipped
machine (the extruder is always oid 3 or later), so any use of that path
misplaces extrusion.

## Negative results (checked, correct)

* **Klipper feedrate/acceleration limits and step counts** —
  `repro/kl_limits_vs_analytic.py`: 80 mm moves with F600, F60000 (capped at
  `max_velocity` 300 mm/s), F30 (0.5 mm/s), `M204 P300`, `M203 X6000` all gave
  exactly 6400 steps and a first-step→last-step span matching
  `d/v + v/a` within 4–13 ms (the residual is the half-step at each end, as
  expected; tolerance stated in the script).
* **Klipper rejects invalid feedrate** — `G1 X10 F0` and `G1 X10 F-600` abort
  with `Invalid speed in 'G1 X10 F0'` (exit 255). Negative feedrate is not
  silently accepted. `G20` is reported as unsupported and aborts the batch.
* **Klipper relative sub-step moves accumulate correctly**:
  `G91` + 1000 × `G1 X0.001` → 15 steps on each of b/c, exactly the
  quantised line-length change for 1 mm, i.e. no lost sub-step distance.
* **Klipper duplicate / zero-length moves** produce no extra steps.
* **RRF velocity/acceleration limits** are exact to the step clock: 10 mm at
  F600 → 1.003333 s, `M204 P300` → 1.033333 s, `M203 X6000` + F6000 →
  0.133333 s (all equal to the closed form to the last printed digit).
* **RRF `M400`/queue overflow** — `repro/rrf_queue_stress.py`: 20 000 absolute
  one-microstep moves produce exactly 20 000 steps and X = 250.000 mm, no
  drops or duplicates.
* **RRF G20** — `G20` + `G1 X1` moves 2032 steps = 25.4 mm exactly.
* **RRF mid-queue limit change** — `repro/rrf_midmotion_limits.py`: with
  `G1 X10 F600` / `G1 X20 F600` / `M204 P300 T300` / `G1 X30 F600` the three
  segment durations are 1.0016667 / 1.0000000 / 1.0166667 s, exactly the
  blended-trapezoid values for ramping up at 3000, cruising, and ramping down
  at the new 300 mm/s². So a limit change applies to the following move and
  does not rewrite the queued plan.
* **RRF duplicate and zero-length moves** emit nothing.
* **RRF sub-step distances** — repeated absolute moves smaller than one step
  are coalesced (correct); relative sub-step moves accumulate.
* **RRF F0 / F−600** are accepted and silently clamped to
  `DefaultMinFeedrate = 0.5 mm/s` (`Config/Configuration.h:51`) — 10 mm takes
  20.0 s. This is upstream RRF policy, documented in source, so it is a
  limitation, not a defect.
* **Bucketed antialiasing vs the exact trapezoid** —
  `repro/js_bucket_profile_vs_analytic.mjs`: a real 6400-step Klipper stream
  replayed through `KlipperBucketedMotionCore` gives exactly 6400 steps, a
  peak rate of 1.600000 steps/2 ms bucket (analytic 1.600), the same move
  duration within 0.7 ms and a worst position error of **0.0008 mm** after the
  ramp. So the 2 ms bucket smearing is not the source of any finding above.

## Limitations and what I could not settle

* All evidence is **simulation/firmware-emulation evidence**. F1 is explicitly
  host-only (`RRF_HOST_BUILD`) and does not exist on real Duet hardware; F2–F4
  are hp-sim5 emulation defects, not Klipper/RRF firmware defects. Nothing here
  says anything about physical printers.
* F1's exact trigger set is empirical. I did not instrument the step ISR, so I
  cannot enumerate which segments trip the truncation; I only demonstrate
  reproducible cases and the one-line code asymmetry that explains them.
* F2 needs a config with more than one driver on a CAN address. None of the
  bundled machines do this, so no shipped preset is affected today; a user
  running a real Duet 3 board (multi-driver) would be.
* F3's size is bounded by the first queue_step batch, so ≈1 microstep for
  batch-mode dumps. I did not test a live Klipper session where an axis starts
  moving mid-print after a `reset_step_clock`; the player ignores
  `reset_step_clock`, which I suspect is a further (untested) defect.
* F4 is on a non-default pipeline; the default player path is correct here.
* I did not test commands arriving mid-motion through the interactive HTTP
  server (only through the batch queue, where 20 000 queued moves were
  correct). I also did not test input shaping, pressure advance, or arcs.
* The `Z` values printed by `M114` for the slideprinter after a small line
  error are amplified by the near-singular 3-coplanar-anchor geometry; do not
  read them as the size of the physical error.

## Evidence index

```
repro/
  run_all.sh                              # runs everything (≈2 s)
  rrf_harness.py                          # RRF batch-mode runner + CAN CSV parser
  klipper_harness.py                      # klippy batch-mode runner + queue_step decoder
  klipper_cartesian.cfg                   # 80 whole steps/mm, max_velocity 300, max_accel 3000
  rrf_standstill_offbyone.py              # F1
  rrf_multidriver_csv.mjs                 # F2
  make_rrf_multidriver_fixture.py         # F2 fixture generator
  js_mcu_player_step_loss_synthetic.mjs   # F3 minimal
  js_mcu_player_step_loss.mjs             # F3 on public/mcu_commands/slideprinter/draw_squares.txt
  js_klipper_oid_axis_mapping.mjs         # F4
  kl_limits_vs_analytic.py                # negative result
  rrf_queue_stress.py                     # negative result
  rrf_midmotion_limits.py                 # negative result
  js_bucket_profile_vs_analytic.mjs       # negative result
  f1_wait_drift.py                        # follow-up 2: F1 over 100 waits
  f2_binary_path_probe.mjs                # follow-up 3: F2 through the .can path
  f3_impact_sim.mjs                       # follow-up 1: F3 physical impact (physics)
  f3_impact_slideprinter.py               # follow-up 1: F3 kinematics cross-check
  f3_player_totals.mjs                    # helper: exact vs player step totals
  fixtures/
    rrf_two_drivers_one_board.{csv,g,gcode}
    klipper_cartesian_x80_f600.txt        # klippy parsedump of G1 X80 F600
    klipper_slideprinter_x10_e5.txt       # klippy parsedump of G1 X10 E5 F600
results.json                              # machine-readable measurements
```


## Follow-up: reachability and impact

### 1. Is F3 reachable from the shipped UI, and what does it do to the printed part?

**Tracing the app modules.** A preset button does **not** use the MCU player.
`hp-sim-3d/app/commandPresetResolver.js:resolvePresetCommand` returns
`machine.compiled_commands[preset.gcode].rrf` with
`format: FileFormat.RRF_CAN_BINARY`; "Print Squares" (`straightMoves`) resolves
to `public/RRF_CAN_commands/draw_squares_bigger_<machine>_no_buildup.can`, and
`commandJobController.runPreset` dispatches `isRrfFormat(format)` to
`workers.ensureRrfWorker()` (`integrations/rrf/rrfCanPlayer.js`). The Klipper
compilations that exist for hp3 / layered slideprinter / hp5 are not used by
the preset buttons.

An **upload** does. `hp-sim-3d/index.html` accepts
`.gcode,.txt,.serial,.usda,.csv,.can`; `handleUpload` (commandJobController.js:368)
→ `queueCommandFile` → `isKlipperFormat(format)` →
`shouldUseRawKlipperUploadPipeline(KLIPPER_UPLOAD_PIPELINE)`, and
`KLIPPER_UPLOAD_PIPELINE = 'player'` (workerController.js:36), so it takes
`workers.ensureKlipperMcuCommandPlayerWorker()` and posts `filename_upload` to
`KlipperMcuCommandPlayer`. So uploading any Klipper MCU dump — for example the
checked-in `public/mcu_commands/slideprinter/draw_squares_bigger_no_buildup.txt`,
or your own `klippy.py -o out.serial` + `parsedump.py` output as documented in
`ai_docs/Klipper_Debugging_tools.md` — is the default path that loses the first
`queue_step` batch. The live API bridge is a different path
(`api_motion_batch` → `consumeStepperBatch`) and does not lose steps.

**Physical consequence (production physics path).**
`repro/f3_impact_sim.mjs` feeds both step streams through the same conversion
the app uses (`angle = steps * 2π/3200`, posted as `{type:'Move', A, B, C}` to
`RemoteSpoolSystem`) and runs the checked-in Wasm cable solver on
`public/usd_scenes/slideprinter.usda` (line layering off, zero gravity, the
planar machine), settling 4000 steps and averaging 400:

| | effector x (m) | effector y (m) | effector z (m) |
| --- | --- | --- | --- |
| exact firmware steps (A,B,C = −5,−5,−5) | −0.000033 | +0.000058 | 0 |
| player-delivered steps (−5, −4.052, −4.208) | −0.000035 | +0.000017 | 0 |
| difference | **−0.0014 mm** | **−0.0409 mm** | 0 |

So the end-to-end effect on the shipped slideprinter is a **0.041 mm effector
offset, almost entirely in −Y** (towards anchor A at y = −2.1 m). That is the
expected direction: the two short-changed motors B and C are the symmetric pair
at +Y, so their cables end 0.056 / 0.047 mm longer than commanded and the
effector sits closer to A. Cross-check with the algebraic rigid-line
linearisation (`repro/f3_impact_slideprinter.py`): dx = −0.0053 mm,
dy = −0.0342 mm, same direction and magnitude. (The firmware's flex
`winch_forward_solve` does not converge at this pose because the home point
lies in the plane of the three anchors, where z is not first-order observable;
a rigid second-order solution would put |dz| ≈ 11.4 mm, but the sim's
slideprinter is planar, z ≡ 0, so that singular branch is not exercised. The
planar physics number is the meaningful one.) One caveat: the catalog scene's
anchors sit at 2.10 m while the Klipper config's are at 1.90 m, so the exact
number for the config geometry would differ by roughly 10 %; the spool radius,
and hence mm per step, is identical (30 mm ⇒ 188.496 mm/rev ⇒ 0.058905 mm/step).

**Constant or growing?** Constant. `repro/f3_player_totals.mjs <stream> 0.25`
replays the first quarter of the same stream: the per-motor deficits are
identical at 25 % and at 100 % of the stream (B = −0.9479, C = −0.7920,
A = 0 in both). Only a stepper that appears for the *first time* in a stream can
lose a batch, so the loss is a one-time offset per file per stepper, not a
drift.

### 2. Does F1 accumulate over many waits?

`repro/f1_wait_drift.py`, cartesian config (80 whole steps/mm):

| sequence | expected | measured |
| --- | --- | --- |
| 100 × alternating X0 / X1, `M400` after each | 0 steps, X = 0 | 0 steps, X = 0.0000 (per-move exactly ∓80) |
| 100 × monotone 1 mm staircase, `M400` after each | 8000 steps, X = 100 | 8000 steps, X = 100.0000 |
| 100 × monotone 1 mm staircase, `G4 P1` after each | 8000 steps, X = 100 | 8000 steps, X = 100.0000 |
| 100 × 10 mm round trip, `M400` after each move | 0 steps, X = 0 | 0 steps, X = 0.0000 (per-move exactly ±800) |
| the failing unit `G1 X10 / G1 X0 / M400 / G1 X1 / M400`, n = 1, 2, 5, 20, 100 | X = 1.0000 | **X = 0.9875 for every n** |
| slideprinter `G1 X10 / M400 / G1 X0 / M400`, n = 1, 2, 5, 20, 100 | net 0/0 steps | net −1/+1 (n=1), 0/+1 (n=2), +1/+1 (n≥5); X 0.068–0.102 mm |

So F1 is **bounded at one microstep and does not accumulate**. Every
well-behaved sequence above is exact even with 100 waits; the failure needs a
specific segment-residue combination. In the failing unit the per-move steps
are `[800, −800, 79, 721, −800, 79, 721, …]`: the 1 mm move from the
standstill-snapped origin loses one step (79), and the next 9 mm move absorbs
it again (721 instead of 720), so the error self-cancels to a fixed 1-step
offset at X = 1 mm. The slideprinter nets oscillate between −1, 0 and +1 step on
motors b/c but never grow with n. Classification unchanged (genuine, host-only) but
its severity is a bounded 12.5 µm (cartesian) / 0.059 mm-of-line
(slideprinter) position error, not a drift.

### 3. Does the `.can` binary path handle a two-driver board?

**No — neither path does.** `repro/f2_binary_path_probe.mjs`:

* The shipped converter `RRF/csv2can.py` reads the same fixed columns as the
  CSV player (`row[6]` = value, `row[7]` = acceleration). Converting the
  two-driver capture yields a `.can` whose only numeric payload is the 800 X
  steps and whose acceleration field is 400. Played through the binary path it
  produces **8.3401 mm of phantom extrusion and 0 steps of motion** — identical
  to the CSV path.
* Even a hand-built, *correct* `.can` (two body records in one context block,
  one per drive, values 800 and 400) merges the two drives: the record's 6-bit
  id is the CAN **destination address** (40), not the local driver index, so
  the decoder sees 1200 steps on one axis and produces **12.5102 mm of
  extrusion**.
* The axis map is address-keyed too
  (`parseRrfMotorAxisMapFromConfigText` stores `canAddress → axis`), so no
  layer of the pipeline can separate two motors on one board.

Consequently F2 is **not "CSV-only"**: multi-driver CAN boards are
unrepresentable end to end (CSV, `.can`, converter, axis map). The user action
that feeds it is uploading a `.csv` or `.can` capture produced by
`RRF/build/rrf_simulator --can-log out.csv` (or its `csv2can.py` /
`scripts/compile_machine_commands.py` derivative) from a config that maps two
motors to one CAN address, e.g. `M584 X40.0 Y40.1 …`; the file input accepts
both extensions.

**Bundled machines never produce it.** Every shipped RRF config maps exactly one
motor per CAN address (`M584 X40.0 Y41.0 Z42.0 …`, see
`RRF/run/vsd/sys/config_*.g`), so no bundled preset or compiled `.can` can
contain a multi-driver row. F2 stays **latent for the shipped catalog** and is
genuine for user configs / real multi-driver Duet hardware.
