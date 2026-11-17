# Reproducible correctness failures found in hp-sim5

Program: `program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc`
Code under test: `c2450975` (main), with the parallel program's autocal-only `089a7ad3`
also present; evidence commit `7d2b0e16`. Date: 2026-11-17. Environment: Linux x64,
Node 24.14.1, Python 3.12, native addon
`src/wasm/cable_solver/target/native/physics.node`, checked-in
`public/wasm/cable_solver.wasm`.

**Review outcome.** A fresh independent reviewer verified the sealed bundle and
reproduced all seven numerical claims. It rejected the headline framing that these are
seven user-visible wrong results: the evidence is a mix of one asset/interoperability
defect, two latent parser/unit defects, one host-only firmware-emulation defect, one
low-severity upload-path defect, one edited-scene robustness defect, and three
invalid-input/API-order hazards. Parts 3, 4 and 7 (RRF standstill, multi-driver CAN,
ambient flag) were confirmed as stated; parts 1, 2, 5 and 6 were downgraded in
wording/classification. The verdicts below incorporate that review. Full assessment:
[review.md](reviews/review-89ed95b1b530/review.md).

Every case below has a one-command reproducer in its session directory. Bug repros
exit nonzero with an assertion message. No source file was modified. This summary is
walked through in `agenda.md`; detailed methods, alternative explanations and
limitations are in each session's `report.md`.

## Severity and reachability

| # | Wrong behavior | Reachability | Measured error | Repro |
|---|---|---|---|---|
| 1 | USD loader ignores `xformOpOrder`; shipped wheel assets depend on that | Shipped HP4/HP4-worn/HP5-reduced scenes; simulator matches author intent, standard USD consumers disagree | 0.20–3.09 m per wheel; minimal case 1414.2 mm | `repro_xform_order_minimal.mjs` |
| 2 | `metersPerUnit` ignored | Any valid non-metre USD scene; no shipped scene uses it | 1000x, 2,233,831.9 mm anchor error | `repro_meters_per_unit.mjs` |
| 3 | Missing spool attribute warns, skips the motor, leaves dangling joints | Edited/authored scene; schema optionality unproven | 2.54 m detached cable ends, first step throws | `repro_missing_attr.mjs` |
| 4 | RRF host standstill bookkeeping off by one microstep | Shipped RRF build, any `M400`/`G4` window | 79 vs 80 steps (0.0125 mm on 80 steps/mm; 0.0589 mm line); bounded | `repro/rrf_standstill_offbyone.py` |
| 5 | Multi-driver CAN boards unrepresentable end to end | Legal `M584 X40.0 Y40.1 ...`; no bundled machine | 0 motion, 8.34 mm phantom extrusion, 8.4e11x accel | `repro/rrf_multidriver_csv.mjs` |
| 6 | Klipper player loses ≤1 microstep on every stepper except the first | Default Klipper `.txt/.serial/.log` upload path, not preset buttons | 0.95/0.79 steps, 0.041 mm effector offset, one-time | `repro/js_mcu_player_step_loss.mjs` |
| 7 | Raw Klipper path maps extruder steps to spool axis D | Opt-in `raw` upload pipeline | E = 0.0000 mm, D = 28.16 mm of line | `repro/js_klipper_oid_axis_mapping.mjs` |
| 8 | Forward-transform dataset noise/bias labels too weak | Benchmark harness only | 25.5x (A–D), 51x (I) | `repro_noise_units.py` |
| 9 | One invalid (non-finite) input coordinate propagates NaN and reports 0 N | Invalid input; robustness, not a valid-scene result | 2→3 of 3 bodies + 3 path fields NaN | `repro_nan_poisoning.mjs` |
| 10 | Layering `split()` writes NaN for a rolling link without radius | Opt-in feature; input not produced by the scene baker | 2/3 bodies + path records NaN | `repro_split_nan_radius.mjs` |
| 11 | Applying a scene plan depends on the ambient `enableLayering` flag | Latent API ordering; UI always sets flag first | 1.804 rad knot, 7.2 cm body position | `repro-plan-ambient-flag.mjs` |
| 12 | `Vector3.rotateAroundAxis` with a zero-length axis is not a rotation | Dead code (`cable_joints_core.js:485`) | returns `v·cosθ`; 100 % error at θ=π/2 | `repro_rotate_zero_axis.mjs` |
| 13 | `normalize()` escapes its zero guard outside the double range | Latent; no path reaches \|q\|≥1e154 | `(0,0,0,0)` or `(Inf,NaN,NaN)` | `repro_normalize_range.mjs` |
| 14 | Layered presets scale moves by 0.47–0.85 % | Shipped layered presets; scene radius vs RRF `M666 R` | 20 mm command → 20.1498 mm (HP4 X) | `repro_layered_radius_scale.mjs` |
| 15 | CubeCorners tip does not follow the commanded path | Shipped CubeCorners presets; 3-DOF firmware model vs 6-DOF scene | 5 mm → 4.9059 mm (−1.88 %), 4.7348 mm layered (−5.30 %) | `repro_cubecorners_tip_rotation.mjs` |

**Review coverage.** The sealed review covered rows 1–7 and 11 (seven claim parts).
Verdicts: part 3 (RRF), part 4 (multi-driver CAN) and part 7/ambient flag were
**confirmed**; parts 1 (USD), 2 (missing attribute), 5 (Klipper player) and 6 (NaN)
were **partially confirmed** and downgraded in wording/classification. Rows 8–10 and
12–15 were produced after sealing and are **not independently reviewed**; each repro was
run by its producer and exits nonzero as documented.

## The four strongest cases

### 1. USD `xformOpOrder` is ignored (shipped scenes)

`machineEntityBuilders.js` / `cable_scene_baker.js` read `xformOp:translate` as the
position and never compose it through the declared rotation, and they apply a
`rotateXYZ` even when `xformOpOrder` omits it.

Minimal: one prim with `translate=(1,0,0)`, `rotateXYZ=(0,90,0)`, order
`[rotateXYZ, translate]`. USD composes `R·T` and puts it at `(0,0,-1)`; hp-sim5 puts
it at `(1,0,0)` — **1414.2 mm** apart. Across shipped assets, 80 wheel prims disagree
by 0.20–3.09 m (HP4 max 3090.2 mm, HP5-reduced max 3073.4 mm); one HP5 cable span is
21.27 mm in the simulator vs 372.71 mm in USD (351.4 mm error). The secondary case:
`cubecorners.usda` `SpoolA` has an unlisted `rotateXYZ=(0,0,20)` which hp-sim5 applies
and USD ignores.

Reviewer's downgrade, adopted here: for the 80 shipped wheels the simulator matches the
asset author's evident intent, and swapping the declared order to
`[translate, rotateXYZ]` leaves all 24 HP4 wheel positions unchanged (max delta 0, a
control the reviewer ran). So this is an **asset-authoring/interoperability defect plus a
missing validation**, not "the simulator returns the wrong geometry in simulation". The
unambiguous loader defects are (a) it never honors the declared order, so any external
or corrected USD asset is misread, and (b) `cubecorners.usda` `SpoolA` applies a
`rotateXYZ` that `xformOpOrder` explicitly omits (20°; physically harmless here).

### 2. `metersPerUnit` is ignored (valid USD, latent for catalog)

A valid millimetre-authored scene loads 1000x too large: anchor `(2,1,0) m` becomes
`(2000,1000,0) m` (2,233,831.9 mm error); spool and corner prims show the same 1000x
ratio. Every shipped scene declares metres, so this is latent — but it is a plain unit
bug on valid input.

### 3. RRF standstill loses one microstep (shipped build)

On the shipped slideprinter config, `G1 X10 / M400 / G1 X0 / M400` ends at **−1/+1**
steps on motors b/c instead of 0/0 (1 step = 0.0589 mm of line). On a clean 80-whole-
steps/mm cartesian config, the 1 mm move after a wait emits **79 steps (0.9875 mm)**
instead of 80; the same sequence without `M400` is exact. A 0.05 mm move produces 4
steps in the CAN capture but 3 in the firmware's own `M114` counter — the two disagree
inside the same firmware.

Cause: fork-only `Move::FreeOldSegments` (`RRF/ReprapFirmware/src/Movement/Move.cpp:2276`)
truncates with `(int32_t)` while the ISR path it imitates uses `std::lrint`. The offset
is bounded at one microstep, sign sequence-dependent, and does not accumulate over 100
repeated cycles (constant `X = 0.9875 mm`).

### 4. Klipper MCU player loses ≤1 microstep on every stepper except the first (upload path)

On the checked-in `public/mcu_commands/slideprinter/draw_squares.txt`, the player
delivers B = −4.052 and C = −4.208 steps for a stream containing −5 on each axis
(0.948 / 0.792 microsteps, 0.0558 / 0.0467 mm of line). In the production Wasm physics
on `slideprinter.usda` the resulting effector offset is **0.0409 mm**, almost entirely
−Y (dx = −0.0014, dy = −0.0409, dz = 0). The same pattern appears on the hp4 and
cubecorners presets (0.3–1.0 step on every stepper except the first parsed). The loss is
one-time per stream, not accumulating.

Cause: `_flushReadyBuckets` advances one global bucket cursor after each parsed line and
deletes emitted buckets; the first `queue_step` of each later stepper spans idle time
from clock 0, so its early buckets are already gone. Reachability: Klipper file uploads
(`.txt/.serial/.log`) use this player; the preset buttons use the RRF binary path and
are unaffected.

## Preset-consistency findings (interpretation matters)

These two are real measured disagreements in shipped presets, but they are model/
config mismatches rather than solver arithmetic errors; they are listed separately so
they are not overstated.

- **Layered presets scale every move by 0.47–0.85 %.** The layer model's effective
  radius at the scene's initial wound length is 39.000 mm (HP3/slideprinter/CubeCorners)
  or 47.000 mm (HP4), while the paired RRF config's `M666 R` is 39.1845 / 46.6045 mm,
  which the firmware uses for every step. A 20 mm HP4 +X command measures 20.1498 mm
  (X), 20.2063 mm (Y), 20.0990 mm (Z); the identical sweep without layering is
  20.0008 / 19.9979 / 19.9965 mm. All 9 `_w_line_layers` presets fail a 0.25 %
  tolerance; it is 3.2 microsteps at 20 mm, above quantisation. Alternative reading:
  the winding model is approximate and/or `M666 R` was calibrated at a different wound
  length; no documented layer-accuracy budget settles it. Repro:
  `node repro_layered_radius_scale.mjs` (exit 1).
- **CubeCorners tool tip does not move by the commanded amount.** A 5 mm +X command
  moves the tip 4.9059 mm (−1.88 %) without layering and 4.7348 mm (−5.30 %) with
  layering, while the body centre moves 5.046 / 5.144 mm and the body rotates
  0.189° / 0.078°. Exact 6-DOF geometry of the authored scene predicts the measured tip
  to 0.0014 / 0.0002 mm, and the same geometry with continuous line lengths is exactly
  5.000 mm. So the solver is faithful; the catalog pairs a 3-DOF RRF model with a
  6-DOF scene. Whether the effector's rotational freedom is intended is undocumented.
  Repro: `node repro_cubecorners_tip_rotation.mjs` (exit 1).

## Negative results worth keeping

- **Physics closed forms pass.** Atwood hairpin `a=g/3` to 2.4e-10 and `T` to 4e-10;
  hanging stretch within 0.49 %; spool payout `rΔθ` within 0.05 %; rigid residual 0;
  degenerate anchors/zero rest/tiny radius finite and correct; 5 production scenes ×
  layering on/off × 300 steps have 0 non-finite values and 0 negative tension.
- **State lifecycle is clean.** Live worlds bit-independent (1388/1388 fields), reset
  and rebuild exact in production order (3 cycles, both backends), bit-deterministic
  in-process and across process restarts.
- **Wasm/native is not qualitatively inconsistent.** Per-step difference ~1e-12
  relative; HP4 bursts to 0.14 rad / 1.2 mm are reproduced by native twins seeded at
  1e-15..1e-9, with matching growth rate (λ≈0.073–0.077 per 2 ms step).
- **Firmware planning is exact.** Klipper/RRF velocity, acceleration, feedrate limits
  and step counts match closed form to the step clock; a 6400-step trapezoid is
  reproduced to 0.0008 mm; 20,000 queued moves lose no steps.
- **Forward transform is good.** 1200 randomized round trips max 3.398e-4 mm (tolerance
  0.02 mm); all 351 non-converged probes are outside the reachable workspace.
- **Catalog consistency.** 11 scenes, 72 anchor offsets, 46 spool radii agree with
  their firmware configs; every scene loads complete cable paths.
- **Per-machine direction and sign are correct.** All 20 catalog machines were driven
  through the production RRF -> CAN -> spool -> headless-world path: no mirrored,
  swapped or reversed axis; cross-axis motion ≤ 0.069 mm on a 5 mm move; line length vs
  motor angle is exactly linear (max deviation 3.8e-16 m) and monotone; CAN step totals
  agree with the config's own closed form to ≤ 2 microsteps. 17 of 20 machines are
  within 1.5 % on every supported axis at 5 mm (the exceptions are the two CubeCorners
  presets and a 2-microstep HP3 Z outlier that vanishes at 20 mm).

## Limitations

All evidence is simulation; no hardware validation is claimed. Shipped scenes are the
HP3/HP4/HP5/slideprinter/cubecorners presets at this revision. Wasm ran inside Node's
V8, not a browser. The per-machine end-to-end kinematics probe is reported separately
in `session-2026-11-17-verify-end-to-end-kinematic-correctness-for-every-catalog-machin-4745f431/`.
The USD finding is an asset/loader contract defect: simulator geometry matches author
intent, while a compliant USD consumer sees different geometry. Latent cases are labeled
as such and are not evidence of a shipped user-visible regression.

Reproduction caveats from the reviewer: the sealed bundle is not self-contained for all
scripts (some need sibling `lib.mjs`, `rrf_harness.py` or fixtures); the prebuilt
`RRF/build/rrf_simulator` and native `physics.node` are gitignored/untracked, so a fresh
clone cannot run the firmware/native repros without building them; and the manifest's
`base_revision` label is one commit old (`c2450975` vs actual parent `089a7ad3`, an
autocal-only delta that does not touch the findings). Scripts are research repros, not
CI-ready tests.
