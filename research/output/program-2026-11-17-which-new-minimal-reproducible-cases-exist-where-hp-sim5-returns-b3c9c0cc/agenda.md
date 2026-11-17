# Which new, minimal, reproducible cases exist where hp-sim5 returns wrong results or violates its own stated invariants?

Goal: find *new*, *reproducible* incorrect behavior. For each genuine case deliver the
smallest possible reproducer (one script, no manual steps) and quantitative evidence
(expected vs actual, size of error, tolerance, code revision). Negative results are
valuable and must be preserved. Do not fix anything; no writes outside this program's
`research/output/` directory.

## Scope exclusions

- Encoder autocalibration is owned by a separate parallel program
  (`program-2026-11-17-compare-single-sensor-...`). Do not investigate, touch, or
  interfere with it.
- No source edits anywhere. Researchers write only inside their own session directory.
- Existing build artifacts only; no rebuilds unless unavoidable.

## Current understanding (2026-11-17)

All findings reproduce on `c2450975` with one-command scripts. "Genuine" below means
the script fails on valid or documented input; "latent" means it needs invalid input
or a non-default API ordering.

### Loader / geometry / units (geometry probe) — strongest cluster

1. **`xformOpOrder` is ignored for translation.** `machineEntityBuilders.js` /
   `cable_scene_baker.js` read `xformOp:translate` as position and never compose it
   through the declared rotation. A one-prim USD scene with order
   `[rotateXYZ, translate]` lands 1414.2 mm from where USD puts it. 80 shipped wheel
   prims in `hp4.usda`, `hp4_worn_bearings.usda`, `hp5_pulley_reduced.usda` are
   0.20–3.09 m off USD; one HP5 cable span is 21.27 mm in the simulator vs 372.71 mm
   in USD. Also applies a rotation that `xformOpOrder` omits (`cubecorners.usda`
   `SpoolA`, 20°). Genuine USD-loader defect on shipped scenes; the simulator's own
   geometry is the author's intent, which is why it went unnoticed.
   Repro: `node repro_xform_order_minimal.mjs` (exit 1), `.venv/bin/python repro_xform_order.py`.
2. **`metersPerUnit` is ignored.** A valid millimetre-authored scene loads 1000x too
   large (anchor `(2,1,0) m` becomes `(2000,1000,0) m`; 2,233,831.9 mm error).
   Latent only because every shipped scene declares metres.
   Repro: `node repro_meters_per_unit.mjs` (exit 1).
3. **Missing non-geometric spool attribute deletes a motor and leaves dangling
   joints.** Removing `physics:angularVelocity` from HP5 `SpoolC` yields one warning,
   no `SpoolC` entity, and two joints with `entityA === undefined`; first physics step
   throws `Missing cable endpoint for joint 29`. Repro: `node repro_missing_attr.mjs`.
4. **Forward-transform dataset noise/bias labels are 25.5x / 51x too weak** because
   `MM_TO_DEG` ignores the dataset's 12.75 gearing and MA 2/4. Benchmark-harness bug
   (the solver itself passes 1200 randomized round trips with max error 3.4e-4 mm).
   Repro: `.venv/bin/python repro_noise_units.py`.

### Firmware / motion layer (firmware probe)

5. **RRF host build standstill bookkeeping is off by one microstep.** `G1 X10 / M400 /
   G1 X0 / M400` leaves −1/+1 steps on slideprinter motors b/c; a 1 mm move after a
   wait emits 79 instead of 80 steps (0.9875 mm). `Move::FreeOldSegments` truncates
   with `(int32_t)` where the ISR uses `std::lrint`. Bounded, non-accumulating.
   Repro: `.venv/bin/python repro/rrf_standstill_offbyone.py` (exit 1).
6. **Multi-driver CAN boards are unrepresentable end to end.** With
   `M584 X40.0 Y40.1 Z40.2 E40.3`, 10+5 mm of commanded travel becomes 0 motion and
   8.34 mm phantom extrusion; acceleration read as 400 instead of 4.77e-10
   (8.4e11x). CSV, `.can`, converter and axis map all key by CAN address. Legal config,
   not produced by bundled machines. Repro: `node repro/rrf_multidriver_csv.mjs`.
7. **Klipper MCU player drops the first queue_step batch of every stepper except the
   first** on the default upload path. Shipped slideprinter `draw_squares.txt`:
   B −4.052 vs −5, C −4.208 vs −5 steps; 0.041 mm effector offset (mostly −Y),
   one-time constant, not growing. Preset buttons use RRF binary and are unaffected;
   Klipper `.txt/.serial/.log` uploads are affected. Repro:
   `node repro/js_mcu_player_step_loss.mjs`.
8. **Raw Klipper parsed-line path sends extruder steps to spool axis D.** `G1 X10 E5`
   -> E = 0.0000 mm, D = 478 steps = 28.16 mm of line. Opt-in `raw` pipeline only.
   Repro: `node repro/js_klipper_oid_axis_mapping.mjs`.

### Cable/physics core (physics probe)

9. **One non-finite input coordinate poisons the whole cable network silently.**
   2→3 of 3 bodies and all 3 path fields become NaN after one step while
   `constraintForceMagnitude` reports 0 N; unrecoverable after repairing the input.
   Invalid input, but silent and unbounded. Repro: `node repro_nan_poisoning.mjs`.
10. **Opt-in layering `split()` writes NaN** into persistent path records for a rolling
    link without a finite radius (`engine.rs:1283`), although `geometry()` tolerates
    the same configuration; 2/3 bodies NaN thereafter. Opt-in feature, input not
    producible by the scene baker. Repro: `node repro_split_nan_radius.mjs`.

### Lifecycle / parity (parity probe)

11. **Applying a baked scene plan is not self-contained.** `cable_joints_core.js:1199`
    derives the hybrid-knot angle from ambient `enableLayering`, not the plan, so the
    same plan yields a 1.804 rad knot difference and 7.2 cm body-position difference.
    UI always sets the flag first, so latent. Repro: `node repro-plan-ambient-flag.mjs`.

### Numeric helpers (numeric probe)

11b. `Vector3.rotateAroundAxis` is not a rotation for a zero-length axis (returns
    `v·cosθ`; norm 2.3e-16 instead of 3.742 at θ=π/2). Dead code at
    `cable_joints_core.js:485`. Repro: `repro_rotate_zero_axis.mjs`.
12. `normalize()` escapes its zero guard outside the double range
    (`Quaternion(1e200,0,0,1)` -> `(0,0,0,0)`; `Vector3(1e-310,0,0)` ->
    `(Inf,NaN,NaN)`). Latent. Repro: `repro_normalize_range.mjs`.
    51 other fuzz checks pass (quaternion identities, rotation matrices, tangent
    branches, inertia tensors, catenary, all six rotate-ABC orders).

### Per-machine kinematics (kinematics probe) — preset consistency, not solver bugs

13. Layered presets scale moves by 0.47–0.85 %: layer-model radius 39.000/47.000 mm vs
    RRF `M666 R` 39.1845/46.6045 mm. 20 mm HP4 +X -> 20.1498 mm (non-layered control
    20.0008 mm). All 9 layered presets. Repro: `repro_layered_radius_scale.mjs`.
14. CubeCorners tip moves 4.9059 mm (−1.88 %) / 4.7348 mm (−5.30 % layered) for a 5 mm
    command while the body centre moves 5.046/5.144 mm and rotates 0.189°/0.078°. Exact
    6-DOF scene geometry predicts the measured tip to 0.0014/0.0002 mm; solver faithful,
    catalog pairs a 3-DOF RRF model with a 6-DOF scene. Repro:
    `repro_cubecorners_tip_rotation.mjs`.
    Direction/sign is correct on every axis of all 20 machines; CAN totals match the
    config closed form to ≤2 microsteps.

### Negative results (strong, keep)

- Live worlds are bit-independent; production-order reset/rebuild is exact; runs are
  bit-deterministic in-process and across process restarts (both backends).
- Wasm/native: ~1e-12 relative per-step difference; HP4 transients amplify it to
  finite-time 0.14 rad / 1.2 mm bursts that re-converge. Native twins perturbed by
  1e-15..1e-9 reproduce the same bursts and growth rate -> chaos-amplified rounding,
  not a backend defect.
- Physics core matches closed forms: Atwood `a=g/3` to 2.4e-10, hanging stretch 0.49 %,
  spool payout 0.05 %, rigid residual 0; 5 production scenes x layering x 300 steps
  have 0 non-finite values and 0 negative tension.
- Firmware velocity/acceleration/feedrate/queue behavior matches closed form to the
  step clock; 6400-step trapezoid reproduced to 0.0008 mm; 20,000 queued moves exact.
- 1200 randomized forward-transform round trips max 3.398e-4 mm; catalog/scene/firmware
  anchor and spool-radius cross-checks agree.

## Open questions and alternatives

- USD finding #1: is the loader or the asset authoritative? Assets are invalid USD but
  match author intent; a compliant viewer renders a broken machine. Fix belongs in
  assets plus a loader warning, but the *behavior* (silently ignoring a declared op
  order) is still wrong. State both readings in the summary.
- F2 needs a per-drive CAN record identity (address alone is insufficient). Is any
  real Duet CAN capture single-driver-only? Bundled machines: yes.
- Numeric-helper fuzz probe pending: quaternion/vector3/geometry3/planar_math/
  inertia_tensor and JS spool wrap math.

## Review outcome (2026-11-17)

Sealed bundle `reviews/review-89ed95b1b530` verified; a fresh reviewer reproduced all
seven numerical claims. Confirmed as stated: RRF standstill, multi-driver CAN, ambient
layering flag. Partially confirmed and downgraded: USD op-order (asset/interop defect,
not simulation geometry), missing spool attribute (warns, does not silently delete),
Klipper player (≤1 microstep per non-first stepper, one-time; upload path only), NaN
poisoning (invalid-input robustness). Reviewer rejected the headline "seven incorrect
behaviours in normal use" and rated scientific value high, code readiness low-to-moderate.
Full assessment: `reviews/review-89ed95b1b530/review.md`.

Base-revision label is one commit old: code under test is `c2450975` plus autocal-only
`089a7ad3`; evidence commit `7d2b0e16`. Reviewer confirmed no finding touches `autocal/`.

## Next decision

Program is complete for this question. Preserve reports, repros and the review; do not
fix, push, or change autocal work. If continued, the highest-value follow-ups would be
(a) an end-to-end UI/worker test for the Klipper upload path, (b) a loader-level
validation test for `xformOpOrder`/`metersPerUnit`, and (c) a decision on whether
`M666 R` or the scene layer radius is authoritative for layered presets. These are
proposals only; no code changes were made.
