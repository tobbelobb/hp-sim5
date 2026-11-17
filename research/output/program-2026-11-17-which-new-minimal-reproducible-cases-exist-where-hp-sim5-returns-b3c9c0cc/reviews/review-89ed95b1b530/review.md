# Independent review — reproducible correctness failures in hp-sim5

Reviewer: fresh independent subagent. Date: 2026-11-17.
Program: `program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc`
Candidate revision: `7d2b0e162b4501137b6a1ab78eb0649450018eea` (tree `4a87c758675a175229fe8b791d1a397b91ab69cb`).
Claimed base revision: `c2450975d7a3dcf1e6bd90d60afc1e7c3acc51f6`.
Review directory: `research/output/program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc/reviews/review-89ed95b1b530`.

## Bottom line

All seven numerical claims reproduce. I ran every one of the seven reproducers except the HP5
cable-span sub-measurement, and I ran the three cheapest decisive checks first as requested:
USD op-order (1414.2 mm), `metersPerUnit` (1000×, 2,233,831.9 mm), and RRF standstill
(79 vs 80 steps; −1/+1 steps after an exact return). The numbers match the claim.

The disagreement is not about the measurements; it is about **classification, severity, and
reachability**. The submission is a mix of:

* one asset-authoring / interoperability defect (the USD wheel transforms),
* one parser defect that is latent for the shipped catalog (multi-driver CAN),
* one host-only firmware-emulation defect (RRF standstill),
* one low-severity default-upload-path defect (Klipper player),
* one edited-scene robustness defect (missing spool attribute),
* and three invalid-input / opt-in / API-ordering hazards (`NaN` poisoning, `metersPerUnit`,
  ambient `enableLayering`).

The summary and limitations say most of this, but the claim sentence ("hp-sim5 … has a set of
new, reproducible incorrect behaviors") reads as seven user-visible wrong results. That
framing is overbroad. Scientific value is high; code readiness is low-to-moderate.

No claim part failed to reproduce numerically. I would **downgrade** parts 1, 2, 5 and 6 in
wording/classification, and I would **reject the headline framing** that these are seven
incorrect behaviours of hp-sim5 in normal use.

## Verification and provenance

Command:

```bash
./hp-sim5-research-program --resume program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc \
  --verify research/output/program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc/reviews/review-89ed95b1b530
```

Result: `verified: true`, candidate revision/tree as above, `scientific_validity:
not_assessed` (the tool checks hashes, not science).

Provenance discrepancy: the manifest says `base_revision = c2450975`, but the candidate
commit's actual parent is `089a7ad3` ("Measure the redundant HP5 line during profiled autocal
sweeps"). The delta between `c2450975` and `089a7ad3` is 11 files, all under `autocal/`;
`src/`, `hp-sim-3d/`, `public/`, `scripts/` and `RRF/` are unchanged. None of the seven
findings touches `autocal/`, so the results are valid for both revisions, but the revision
label is wrong by one commit and should be corrected. The candidate commit itself adds only
`research/` files (`git diff --name-only 089a7ad3 7d2b0e16 | grep -v '^research/'` returns
nothing), so "no source file was modified" is true of the research commit.

I also note the working tree had uncommitted edits (`summary.md`, `index.md`, `agenda.md`) that
are not part of the candidate; I did not use them. I reviewed the committed/bundled content. I
did accidentally rewrite one committed fixture (the multi-driver CSV) while regenerating it
from the firmware; I restored it with `git checkout --` and its committed hash is
`3446aa98…e391` again. Nothing else in the repository was modified by me.

## What I ran vs. what I only read

### Ran (reproduction)

All commands below were run from the repository at the candidate commit. "files/" means the
sealed review bundle directory; session directories are in the candidate commit.

| # | Check | Command (cwd abbreviated) | Result |
|---|---|---|---|
| 1 | USD minimal op-order | `node 02-repro_xform_order_minimal.mjs` (files/) | loader `(1,0,0)`, USD `(0,0,-1)`, **1414.2 mm**, exit 1 |
| 2 | `metersPerUnit` | `node 03-repro_meters_per_unit.mjs` (files/) | all four prims ratio **1000.0**; anchor error **2,233,831.9 mm**; exit 1 |
| 3 | RRF standstill | `.venv/bin/python repro/rrf_standstill_offbyone.py` (firmware session) | A net b/c **−1/+1**, M114 `Count 0 -1 2`; B `[800,−800,79]`, X **0.9875**; control `[800,−800,80]`, X 1.0; C capture **4**, M114 `Count` **3**; exit 1 |
| 4 | Shipped USD audit | `.venv/bin/python audit_transforms.py` (geometry session) | 312 tagged bodies, **81 disagree**, worst position **3090.240 mm**, SpoolA orientation **20.00°**; exit 1 |
| 5 | Shipped wheel list | `.venv/bin/python repro_xform_order.py` (geometry session) | **80 prims** disagree; hp4 `WheelAL_top` 507.2 mm; max 3090.2 mm; exit 1 |
| 6 | Missing spool attribute | `node repro_missing_attr.mjs` (geometry session) | SpoolC `undefined`; orphan ends at **2.5380 / 2.4839 m**; first step throws `Missing cable endpoint for joint 29`; exit 1 |
| 7 | Multi-driver CAN | `node repro/rrf_multidriver_csv.mjs` (firmware session) | address 40 → **E**; A/B travel **0 steps**; **8.3401 mm** phantom extrusion; accel **400 vs 4.77e−10**; exit 1 |
| 8 | Multi-driver fixture regeneration | `.venv/bin/python repro/make_rrf_multidriver_fixture.py` | firmware final position **X=10, Y=5**; CSV identical except `generated_at` timestamp; exit 0 |
| 9 | Klipper player, slideprinter | `node repro/js_mcu_player_step_loss.mjs …/draw_squares.txt` | A **−5**, B **−4.0521**, C **−4.2080**; 12,690 moves; exit 1 |
| 10 | Klipper player, synthetic | `node repro/js_mcu_player_step_loss_synthetic.mjs` | incremental A 1 / B **0**; deferred A 1 / B 1; exit 1 |
| 11 | Klipper one-time bias | `node repro/f3_player_totals.mjs … 0.25` and `… 1` | 25% and 100% deficits both B **−0.9479**, C **−0.7920**; full stream exact A/B/C = −5; exit 0 |
| 12 | Klipper physical impact | `node repro/f3_impact_sim.mjs …/draw_squares.txt` | dx **−0.0014 mm**, dy **−0.0409 mm**, dz 0; norm **0.0409 mm**; exit 1 |
| 13 | Klipper, hp4 / cubecorners | `node repro/js_mcu_player_step_loss.mjs …` | hp4 loses **0.973/0.741/0.600** on B/C/D; cubecorners loses **0.958/0.768/0.640/0.549/0.405/0.320** (plus **0.265** on O, omitted from the report) |
| 14 | NaN poisoning | `node repro_nan_poisoning.mjs` (physics session) | 2/3 bodies non-finite; rests `[NaN,NaN]`, stored `[0,NaN,0]`; lambda/tension **0**; after repair 3/3 bodies; exit 1 |
| 15 | Split NaN | `node repro_split_nan_radius.mjs` (physics session) | split off finite, split on `n=2`, 3 stored + 2 rest NaN, 2/3 bodies non-finite; exit 1 |
| 16 | Ambient flag | `node repro-plan-ambient-flag.mjs` (lifecycle session) | knot difference **1.8037799645 rad**, body position **7.199e−2 m** after 500 steps; exit 1 |
| 17 | My asset-order swap control | `node /tmp/opencode/asset_swap_probe.mjs` | 24 hp4 wheels, **max position delta 0** after swapping the order to `["translate","rotateXYZ"]`; exit 0 |

### Only read / inspected

* The sealed `manifest.json`, `summary.md` and the four session reports.
* Code: `src/js/usd/cable_scene_baker.js:290-370`; `hp-sim-3d/app/scene/usdValueReaders.js:85-135`;
  `hp-sim-3d/app/scene/machineEntityBuilders.js:40-215`;
  `integrations/klipper/klipperMcuCommandPlayer.js:60-466`;
  `integrations/rrf/rrfFirmwareModel.js:55-95`; `integrations/rrf/rrfCanPlayer.js:440-500`;
  `hp-sim-3d/app/workerController.js:36`; `hp-sim-3d/app/commandJobController.js:260-345`;
  `hp-sim-3d/app/commandPresetResolver.js:30-60`;
  `hp-sim-3d/app/featureFlagsController.js:65-90`;
  `src/js/cable_joints_3d/cable_joints_core.js:1199`;
  `hp-sim-3d/app/scene/cableJointBuilder.js:25-31`;
  `src/js/cable_joints_3d/rust_physics_engine.js:151`;
  `RRF/ReprapFirmware/src/Movement/Move.cpp:2276-2330`;
  `RRF/ReprapFirmware/src/Movement/DriveMovement.cpp:750`.
* Assets: `public/usd_scenes/hp4.usda`, `hp4_worn_bearings.usda`,
  `hp5_pulley_reduced.usda`, `cubecorners.usda`; every shipped scene declares
  `metersPerUnit = 1`.
* Not run: native-backend variant of the missing-attribute case; rebuilding
  `RRF/build/rrf_simulator` from source; full Jest/Python suites; the browser UI; the
  HP5 351.4 mm cable-span number; the forward-noise, determinism and parity sessions
  (not part of the seven-part claim).

The summary lists 13 findings; the claim selects seven. I did not audit the four extras
(raw Klipper OID→axis mapping, forward-transform noise units, zero-axis `rotateAroundAxis`,
`normalize` range), which the summary itself labels opt-in, harness-only or dead-code. They do
not affect the seven-part verdict.

## Verdict per claim part

| Part | Verdict | One-line reason |
|---|---|---|
| 1. USD `xformOpOrder` + `metersPerUnit` | **partially confirmed** | Measurements confirmed; the 80-wheel "loader is wrong" framing is disputed — the simulator matches author intent and an asset-order swap changes no simulator position. `metersPerUnit` is a genuine latent unit bug. |
| 2. Missing spool attribute | **partially confirmed** | Deletion, 2.54 m dangling ends and first-step throw reproduced; "silently" is inaccurate (console warning), and the attribute may be schema-required, but the `undefined` vs `null` dangling-joint bug is unambiguous. |
| 3. RRF standstill off-by-one | **confirmed** | Exact numbers, deterministic, within-firmware control without `M400` is exact; host-only and bounded. |
| 4. Multi-driver CAN | **confirmed** | Legal config, real firmware capture, 100% of linear motion lost, 8.3401 mm phantom extrusion; latent for shipped machines. |
| 5. Klipper player step loss | **partially confirmed** | Loss is real and deterministic, one-time, 0.0409 mm; "drops the first queue_step batch" overstates it (≤1 microstep per non-first stepper), and the "shipped preset" is an upload file, not a UI preset button. |
| 6. NaN poisoning | **partially confirmed** | Behaviour reproduced; it is invalid-input robustness/GIGO, not a wrong result for valid scenes, and the first step leaves 2/3 bodies NaN (3/3 only after repair). |
| 7. Ambient `enableLayering` | **confirmed** | 1.80378 rad / 7.199 cm reproduced; latent API-ordering hazard, not UI-reachable. |

"Partially confirmed" here means the measurement is confirmed but the claim's
classification, severity, or wording is not fully supported. No part is "not reproduced".

## Part 1 — the USD authority question

This is the part I scrutinised most, because the task asks whether the asset or the loader is
authoritative and whether calling the loader behaviour "incorrect" is defensible.

**What is measured.** The minimal one-prim scene is valid USD: `translate=(1,0,0)`,
`rotateXYZ=(0,90,0)`, order `[rotateXYZ, translate]`. pxr composes `R·T`, so the origin maps
to `(0,0,−1)`. The loader reads `xformOp:translate` directly as the position and applies
`rotateXYZ` only to the orientation, so it places the prim at `(1,0,0)`. 1414.2 mm. Confirmed.
The shipped audit is also confirmed: 80 wheel prims across hp4, hp4_worn_bearings and
hp5_pulley_reduced; 81 disagreements including `cubecorners` `SpoolA`; worst 3090.2 mm.

**Is the asset or the loader authoritative?** For the shipped scenes, the asset's *intent* is
clear and the loader matches it:

* the numeric comments next to the wheel translations are explicit
  (`# 0.1-0.0065-0.00075` for `z = 0.09275`), i.e. the raw translation is the intended
  position;
* the neighbouring pinholes are at the raw-translation locations, so the intended wheel stack
  touches the corner clamp; under `R·T` the wheel moves to `z = 0.34641` and the bearing stack
  is detached;
* swapping the declared order on the 24 hp4 wheel prims to
  `["xformOp:translate","xformOp:rotateXYZ"]` leaves **every simulator wheel position
  unchanged** (my control: 24/24, max delta 0). Under pxr, the swapped order puts the wheel at
  the raw translation. So the simulator does not "read the order and get it wrong"; it ignores
  the order for translation and implements a fixed `T·R` convention.

That makes the 80-wheel case an **asset authoring / export-interoperability defect** at least as
much as a loader bug. The assets parse as USD, though pxr warns about several prims that list a
non-existent `xformOp:orient` (including `cubecorners` `SpoolA`), and their declared order
contradicts the author's intended positions. The report's phrase "the assets themselves are not
valid USD" is imprecise: they are valid USD that encode an unintended transform. The correct
statement is "the assets do not mean what the author intended to a USD consumer".

**Where the loader is unambiguously at fault.** `cubecorners` `SpoolA` lists
`["xformOp:translate","xformOp:orient"]` but has no `xformOp:orient`; it has an unlisted
`rotateXYZ=(0,0,20)`. The loader's fallback applies the unlisted `rotateXYZ`, while pxr skips
the missing listed op and applies no rotation. That is a genuine loader non-compliance (it
applies an op not in the order), though physically harmless here because the rotation is about
the spool axis. It shows the reader is not honouring the order list, not merely choosing a
private translation convention.

**`metersPerUnit`.** Confirmed: a valid millimetre-authored scene (`metersPerUnit = 0.001`,
linear lengths ×1000) loads 1000× too large. The loader has no consumer of `metersPerUnit`
(`grep` finds none), all 11 shipped scenes declare metres, and the app accepts uploaded
`.usda`. This is a genuine latent unit bug on valid user input. Caveat: the synthetic scene
scales translations, radii and distance limits but not inertia/mass, so it is a position
equivalence test, not a fully equivalent dynamics scene; that does not affect the 1000×
position claim. The fixture itself has no explicit `metersPerUnit` (default 1), so the
report's "in metres (`metersPerUnit = 1`)" is implicit rather than declared.

**Verdict on part 1.** Measurements confirmed. The 80-wheel "loader is incorrect" claim is
**disputed** as a statement about simulation correctness; it is a real interoperability /
missing-validation defect, and the recommended asset fix is the right one. The SpoolA
unlisted-rotation case and the `metersPerUnit` case are genuine loader bugs, the latter latent.
Hence **partially confirmed**.

## Part 2 — missing spool attribute

Reproduced exactly: deleting `float3 physics:angularVelocity` from `SpoolC` prints
`Skipping Spool prim SpoolC due to missing attributes.`, leaves `SpoolC` undefined, leaves two
`CablePathCL` joints with `entityA=undefined`, places their ends 2.5380 / 2.4839 m from the
spool centre, and the first `world.update(dt)` throws
`Missing cable endpoint for joint 29`. `cableJointBuilder.js:31` checks `=== null` while
`Map.get` returns `undefined`; `rust_physics_engine.js:151` then throws. That part is
unambiguous and well reduced.

Two wording issues. (a) "Silently deletes a motor" is too strong: there is a console warning.
The *subsequent* dangling state is silent until the delayed throw. (b) Whether
`physics:angularVelocity` is "optional" is assumed, not proven: no schema/doc in the repo
enforces or documents it, and the loader's `applyPinhole` treats it as optional while
`applySpool`/`applyWheel` require it. Even if the attribute is required for a dynamic body, the
correct behaviour is a load-time rejection; the half-built world and `undefined`-vs-`null`
dangling joints are the bug. Reachability: edited/authored scene only. **Partially confirmed**
(behaviour confirmed; "silent"/"optional" wording overstated).

## Part 3 — RRF standstill off-by-one

Reproduced exactly, including the within-firmware control:

* case A: net CAN steps b/c = **−1/+1** (M114 `Count 0 -1 2`), repeated run identical;
* case B: segment steps `[800, −800, 79]`, final X **0.9875**; the same sequence without
  `M400` gives `[800, −800, 80]`, X 1.0;
* case C: 0.05 mm move = 4 captured steps, M114 `Count` 3.

The code asymmetry is present: `Move::FreeOldSegments` uses
`(int32_t)(length + distanceCarriedForwards)` (Move.cpp:2318) while the ISR path uses
`std::lrint` (DriveMovement.cpp:750). Host-only (`RRF_HOST_BUILD`), bounded at one microstep,
does not accumulate over 100 cycles per the report's follow-up. This is a genuine emulation
defect, but it is in the research/host build, not on real hardware. The claim is accurate.
**Confirmed.** Severity: 0.0125 mm (cartesian) / 0.0589 mm of line (slideprinter), one-time.

Minor note: the claim's "−1/+1 steps after an exact return" is the CAN step-stream net; the
firmware's own M114 counter shows `0 −1 2` (C is +2, not +1). The report uses the CAN net,
which is the right external step evidence, but the internal counter/stream disagreement is
larger than the claim's single number suggests.

## Part 4 — multi-driver CAN

Reproduced exactly and independently regenerated from the real firmware:

* fixture config `M584 X40.0 Y40.1 Z40.2 E40.3` is accepted by
  `RRF/build/rrf_simulator`; firmware final position is **X=10, Y=5**;
* capture row has 10 columns: `0,40,0,2500,836026,2500,800,400,4.77e-10,4.77e-10`;
* hp-sim5's `parseRrfMotorAxisMapFromConfigText` keys only by CAN address and drops the
  `.driver` suffix, so address 40 → **E** (last axis wins);
* `_parseCsvRow` assumes 9 columns, so `parts[7] = 400` is read as acceleration and the 400 Y
  steps are dropped;
* result: A/B travel **0**, **8.3401 mm** of phantom extrusion, acceleration **400** instead
  of **4.77e−10** (8.39e11× too large).

The config is legal RRF, the firmware emitted the motion, and the parser silently mis-maps.
The strongest alternative — "one motor per CAN address is an undocumented project limit" — is
weakened by the parser explicitly accepting `parseRrfDriveSpecifier`'s `.driver` suffix and by
the absence of any doc stating the limit. Latent for the shipped catalog (every bundled config
uses one motor per address). **Confirmed.**

## Part 5 — Klipper MCU player

Reproduced exactly on the shipped MCU dump:

* `public/mcu_commands/slideprinter/draw_squares.txt`: stream net A/B/C = −5/−5/−5; player
  output A −5, B **−4.0521**, C **−4.2080**; loss 0.948 / 0.792 microsteps;
* synthetic two-stepper case: A 1, B **0**; a single deferred flush gives A 1 / B 1;
* one-time bias: deficits are identical at 25% and 100% of the stream (B −0.9479, C −0.7920);
* physical impact on the production Wasm physics: **dx −0.0014 mm, dy −0.0409 mm**, norm
  0.0409 mm;
* same pattern on hp4 (0.973/0.741/0.600 on B/C/D) and cubecorners
  (0.958/0.768/0.640/0.549/0.405/0.320 on B/C/D/I/J/L; O also loses 0.265, omitted).

The mechanism is confirmed by inspection: `_flushReadyBuckets` advances one global
`nextBucketToEmit`; each axis's first `queue_step` is smeared from `state.lastTick = 0`; for
later-parsed steppers, the early buckets are already emitted and deleted, so those writes are
lost. The deferred-flush control proves it is a cursor/bookkeeping bug, not an inherent
consequence of 2 ms bucketing.

Wording issues. (a) "Drops the first queue_step batch" overstates: it loses the portion of the
first batch that falls into already-emitted buckets, ≤1 microstep per non-first stepper; the
whole batch is lost only in the synthetic count=1 case. (b) "Shipped preset affected" is
ambiguous: the checked-in `draw_squares.txt` is a shipped MCU dump, but it is not a
`machines/catalog.json` preset, and the UI preset buttons resolve to `RRF_CAN_BINARY` and do
**not** use this player (`commandPresetResolver.js:45-46`;
`commandJobController.js:270-330`). The report says this explicitly; the summary table does
not. The reachable default is **file upload** (`.txt/.serial/.log`, `KLIPPER_UPLOAD_PIPELINE =
'player'`), which is a real user action, but not the preset button. Severity: one-time
0.041 mm endpoint offset, 19% of the net 5-step move but ~0.002% of total travel.
**Partially confirmed** (behaviour confirmed; mechanism/severity wording overstated).

## Part 6 — NaN poisoning

Reproduced: one `NaN` in a mass position → 2/3 bodies non-finite (including the fixed roller's
quaternion/ω and the other mass's path fields), both path rests NaN, stored partially NaN,
`constraintLambda` / `constraintForceMagnitude` **0 / 0 N**; repairing the input and stepping
again leaves 3/3 bodies and both rests NaN. The core's own `f64::min`/NaN behaviour explains
the 0 N while `correct()` propagates NaN. This is a real state-poisoning robustness defect.

Classification: the input is invalid, so this is GIGO rather than a wrong physics result for a
valid scene. The core does try to contain non-finite values elsewhere (load torque,
attachments, servo/encoders/diagnostics), and the spread to unrelated bodies plus the
plausible 0 N report are the strongest reasons to count it as a defect. "Whole cable network"
is close (both path segments are poisoned) but the first step leaves 2/3 bodies, not 3/3;
3/3 only after repair. Reachability: invalid input only. **Partially confirmed.**

## Part 7 — ambient `enableLayering`

Reproduced exactly: same layered plan, same final `enableLayering=true`, only the apply/flag
order differs → `HybridKnotAngleComponent.pathAngles.35` differs by **1.8037799645 rad**, body
position by **7.199e−2 m** after 500 steps. `cable_joints_core.js:1199` reads the ambient
`layeringEnabled(world)` at apply time; the plan's baked half-width is not consulted. The UI
`setLineLayeringEnabledState` sets the flags first and then calls `onLineLayeringChanged` →
reset/rebuild, so the shipped UI is safe. This is a genuine API-ordering invariant violation,
not a user-visible regression. **Confirmed** (behaviour), latent/reachability caveat.

## Strongest alternative explanation

The strongest overall alternative is that this is **not seven cases of hp-sim5 returning wrong
results in normal use**, but a well-executed audit that found real defects mostly at the edges:

* **USD:** the assets are authoritative for intent and the loader is a private-format reader
  that deliberately treats `xformOp:translate` as position. The simulator is self-consistent
  and matches the author's numeric comments; the "0.20–3.09 m" is what a *different* consumer
  sees. The asset-swap test (24 hp4 wheels unchanged) supports this reading. On this reading
  the defect is asset authoring plus missing validation, not wrong simulation.
* **Missing attribute:** the scene schema may require `physics:angularVelocity`; the user
  edited a required field, so the loader's skip is defensible. Only the dangling joints and
  delayed throw are unambiguously wrong.
* **RRF:** the host build's truncation is a research-only performance shortcut; the error is
  bounded at one microstep and absent on hardware.
* **Multi-driver CAN:** the project may only support one motor per CAN address; the input is
  legal RRF but not a bundled topology.
* **Klipper:** the 2 ms bucket smearing is an intended approximation; a sub-microstep one-time
  endpoint offset is within the scale the 2 ms bucketing already accepts (though the bucket
  profile's worst position error is 0.0008 mm, so 0.041 mm is above it).
* **NaN / ambient flag:** invalid input / unsupported API usage, not user-visible.
* **`metersPerUnit`:** all shipped scenes are metres; the app could be documented as
  metres-only, though it does not reject other units.

The submission's own limitations section concedes most of this. The residual disagreement is
about the headline and the word "incorrect", not about the numbers.

## Gaps and limitations

1. **Sealed bundle is not self-contained.** 6 of the 14 bundled files cannot run from
   `files/`: `06` imports `rrf_harness.py`; `07` needs `fixtures/`; `08` computes `ROOT` five
   levels up and points outside the repo when copied to `files/`; `10`, `11` and `13` import
   `lib.mjs` (not bundled). `02`, `03` and `04` hardcode `/home/torbjorn/repos/hp-sim5`. I ran
   the reproducers from the committed session directories, not from the bundle. A reviewer
   with only the sealed bundle cannot reproduce 6/14 scripts. For a "fixed submission" this
   is a packaging gap; the candidate commit does contain the full session dirs.
2. **No code fixes or regression tests.** The submission is research evidence, not a patch.
   Root causes and suggested fixes are specific and actionable, but no test was added to CI,
   and the full Jest/Python suites were not run.
3. **Revision label.** Manifest says `c2450975`; actual parent is `089a7ad3`. The delta is
   autocal-only and does not affect the findings, but the provenance should be corrected.
4. **Prebuilt RRF binary and native addon are untracked.** `RRF/build/rrf_simulator` is
   gitignored (`.gitignore:24`) and
   `src/wasm/cable_solver/target/native/physics.node` is under the crate's ignored `target/`;
   neither is in the candidate commit. The reports record hashes (`1c137e6…` for the native
   addon), and the RRF source is present, but a fresh checkout must build both. Part 3 and the
   native-backend physics repros are therefore not reproducible from the commit alone without a
   build step; the manifest's "bundled native addon" wording is misleading. A rebuild at fork
   commit `82f356b45` would also close the binary/source provenance question.
5. **Synthetic `metersPerUnit` scene.** Positions/radii/distance limits are scaled; inertia
   and mass are not, so it is not a fully dynamically equivalent scene. The 1000× position
   claim is unaffected.
6. **Missing-attribute schema.** No repo schema/doc proves `physics:angularVelocity` is
   optional. The finding should be stated as "the loader requires it while the rest of the
   loader treats it as optional; a missing value should default or be rejected at load".
7. **Multi-driver topology.** Address 40 with four drivers is legal RRF and the firmware
   accepts it, but it may not represent a common Duet topology. No bundled machine exercises it.
8. **Klipper evidence is class-level, not UI-level.** The player class was driven directly
   with `setAsapMode(true)`; the actual worker/UI path and the production `set_dt` were not
   driven end-to-end. The 0.0409 mm uses the catalog scene's 2.10 m anchors while the Klipper
   config's anchors are 1.90 m; the report notes ~10% geometry difference. The preset-button
   non-reachability is correctly stated in the report but blurred in the summary table.
9. **NaN / split / ambient require invalid inputs, opt-in flags or API misuse.** They are
   robustness/latent findings, not user-visible wrong results.
10. **RRF case A internal inconsistency.** The claim's "−1/+1" is the CAN net; M114 `Count`
    is `0 −1 2` (C = +2). The report discusses the capture/counter disagreement in case C but
    not this one. The step-stream evidence is still valid.
11. **Cubecorners O loss omitted.** The report lists 0.958/0.768/0.640/0.549/0.405/0.320 on
    B/C/D/I/J/L but not O's 0.265; the claim "every stepper except the first" still holds.
12. **USD audit method.** The local pxr build lacks `UsdGeom.Circle`, so the audit reimplements
    the local transform composition (disclosed). For these flat scenes it is equivalent, but
    it is not a full `ComputeLocalToWorldTransform` / renderer test.
13. **No hardware, one host, Node V8 not a browser.** As stated in the submission.

## Additional test most likely to change my judgment

**Swap the declared order on all 80 affected wheel prims to
`["xformOp:translate","xformOp:rotateXYZ"]`, then (a) re-run the pxr audit and (b) load the
modified scenes through the full production pipeline and compare baked cable lengths,
joint attachment points and a short physics run against the originals.** If the simulator's
geometry is unchanged and pxr now agrees, part 1 collapses to asset authoring + missing
validation and the "loader is incorrect" claim should be downgraded. If the simulator's cable
lengths or dynamics change, the loader/baker has a deeper order dependence and the
loader-fault claim strengthens.

I already ran the position half on hp4: 24/24 wheel positions unchanged, max delta 0. The
remaining halves are hp4_worn_bearings, hp5_pulley_reduced, the cable-length comparison, and
the SpoolA orientation case. This is the single test most likely to move my verdict on the
main disputed part.

Secondary: drive the shipped `draw_squares.txt` through the actual
`ensureKlipperMcuCommandPlayerWorker`/UI upload path at the production `set_dt`, and measure
the effector offset end-to-end. If it is 0.0409 mm, part 5's reachability is fully confirmed;
if it vanishes, the class-level repro is a harness artifact.

## Scientific value vs code readiness

**Scientific value: high.** The program is well-structured, deterministic, quantitative and
unusually honest about controls and limitations. The negative results are valuable and should
be preserved: closed-form cable physics (Atwood, hanging statics, spool payout, rigid
residuals), world isolation/reset/determinism, Wasm/native per-step parity with a
perturbation-scaling explanation of the HP4 bursts, exact Klipper/RRF planning, forward
round-trip accuracy, and catalog/scene/firmware consistency. The root causes are specific:
`(int32_t)` vs `std::lrint`; `nextBucketToEmit` cursor vs per-axis first batch; address-keyed
CAN map dropping `.driver`; `entityA === null` vs `undefined`; ambient `layeringEnabled` at
apply time. The USD audit is a useful interoperability check with a clear recommended asset
fix. These are genuinely useful findings.

**Code readiness: low-to-moderate.** The deliverable is research scripts, not a code change:
no fixes, no regression tests, no CI integration, hardcoded absolute paths, a non-self-contained
sealed bundle, a prebuilt firmware binary, no UI/browser end-to-end test, and a one-commit
revision-label error. The findings are ready to be turned into fixes, but the submission
itself is not code-ready.

**Claim readiness: partially confirmed.** All seven measurements reproduce; the claim's
uniform "incorrect behaviors" framing does not. I would retitle it as "seven reproducible
defects and latent hazards, with reachability and severity", and put the reachability column
in the claim itself rather than only in the summary/limitations.

## Claim parts I would downgrade or reject

* **Downgrade part 1** (80-wheel sub-claim) from "the loader is incorrect" to "the assets
  declare an order that contradicts their intended positions; the loader silently implements
  a private `T·R` convention and has no validation, so USD consumers disagree by 0.20–3.09 m".
  Keep the measurement, the `SpoolA` unlisted-rotation loader bug (20°, physically harmless),
  and the `metersPerUnit` latent unit bug.
* **Downgrade part 2** from "silently deletes a motor" to "warns and skips the spool, leaving
  two dangling joints and a delayed first-step throw"; treat the optionality of
  `physics:angularVelocity` as an open schema question, not a proven fact.
* **Downgrade part 5** from "drops the first queue_step batch" to "loses ≤1 microstep per
  non-first stepper, one-time; 0.95/0.79 steps and 0.0409 mm on the shipped MCU dump; default
  upload path, not the RRF preset buttons."
* **Downgrade part 6** from "NaNs the whole cable network" to "after one invalid coordinate,
  2/3 bodies and 3 path fields are NaN and the reported force is 0 N; after repair, 3/3
  bodies; invalid-input robustness, not a valid-scene wrong result."
* **Keep parts 3, 4 and 7 confirmed**, with the reachability caveats: 3 is host-only and
  bounded; 4 is latent for the bundled catalog; 7 is latent API misuse and not UI-reachable.
* **Reject the headline** that these are seven incorrect behaviours of hp-sim5 in normal use.
  The correct summary is: two genuine loader/parser defects that are latent or
  interoperability-only (USD metersPerUnit/order; multi-driver CAN), one host-only firmware
  emulation defect (RRF), one low-severity default-upload defect (Klipper), one edited-scene
  robustness defect (missing attribute), and three invalid-input/opt-in/API-ordering hazards.

## Evidence links

* Sealed bundle: `reviews/review-89ed95b1b530/manifest.json`, `files/00-summary.md`,
  `files/01-report.md`, `files/05-report.md`, `files/09-report.md`, `files/12-report.md`.
* USD: `files/02-repro_xform_order_minimal.mjs`, `files/03-repro_meters_per_unit.mjs`;
  session `session-2026-11-17-hunt-for-wrong-results-in-machine-geometry-scene-config-loading-f6d4333b/`
  (`audit_transforms.py`, `repro_xform_order.py`, `repro_missing_attr.mjs`).
* Firmware/motion: `files/06-rrf_standstill_offbyone.py`, `files/07-rrf_multidriver_csv.mjs`,
  `files/08-js_mcu_player_step_loss.mjs`; session
  `session-2026-11-17-hunt-for-wrong-results-in-the-firmware-and-motion-command-layer-58efbe52/`
  (`repro/`, `repro/fixtures/`, `repro/f3_impact_sim.mjs`, `repro/f3_player_totals.mjs`).
* Physics: `files/10-repro_nan_poisoning.mjs`, `files/11-repro_split_nan_radius.mjs`; session
  `session-2026-11-17-probe-the-rust-cable-physics-core-for-wrong-results-degenerate-c-79c933d4/`.
* Lifecycle: `files/12-report.md`, `files/13-repro-plan-ambient-flag.mjs`; session
  `session-2026-11-17-check-cross-backend-and-state-lifecycle-correctness-wasm-vs-nati-8f0a1539/`.
* My asset-order control: `/tmp/opencode/asset_swap_probe.mjs` (24 hp4 wheels, max position
  delta 0 after the order swap).
