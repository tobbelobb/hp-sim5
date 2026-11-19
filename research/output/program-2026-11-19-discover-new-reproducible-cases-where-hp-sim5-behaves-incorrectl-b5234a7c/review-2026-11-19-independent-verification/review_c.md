# Independent verification — session C (`...-4a9e9720`): G6 net-zero, G92 target jump

Reviewer: fresh adversarial check, no part in producing the work. Checkout
`56a66f77d3bfa58e52621d06f6273d5a1f3501f5` (HEAD), Node v26.13.0.
Ran: `repro_g6_net_zero.mjs` (exit 1), `probe_world_g6.mjs`,
`probe_g92.mjs`, `probe_world.mjs`, `repro_preset_vs_upload_hp4.mjs`
(PASS/exit 0), plus my own scripts `verify_g6_g92.mjs`,
`verify_g6_physical.mjs`, `verify_g92_equiv.mjs`, `verify_g92_models.mjs`.
Longest run 0.9 s, RSS 100 MB, one process at a time; `free -h` unchanged
(swap 1.5 GiB before and after). Input hashes in `measurements.json → inputs`
match the working tree (`tighten.gcode` b4394b80…, `config_hp4.g` 322b94c9…).

## Engine contract (verified, not assumed)

* `RemoteSpoolSystem._processCommand`: `Move` → `commandedAngle = v`;
  `Add to reference` → `deltaAngle += v` (`remoteSpoolSystem.js:271-288`).
* `rust_physics_engine.js:115` packs `[commandedAngle, deltaAngle, …]` at field
  107, and `:185` binds the same order back; `engine.rs:1568` (and the identical
  diagnostics copy at `:1782`) computes `let target = e[107] - e[108]`, i.e.
  **target = commandedAngle − deltaAngle**; `flightRecorderSnapshot.js:103`
  repeats the same formula. So “Add to reference adds to deltaAngle and is
  subtracted from the target” is exact.
* Consequence: a pure coordinate-frame shift (G92) is correctly represented by
  a Move to the new commanded angle **plus** an Add of the same delta; a
  physical move needs a Move **only**. The CAN/Klipper players emit
  Add-then-Move (`rrfCanPlayer.js:255-267`); their `bucketAddToReference` is
  never populated at all (no `.set(` anywhere in `rrfCanPlayer.js`), so the
  preset path can never exercise either pattern — as the report says.

## C1 — G6 `A-42 B-42 C-42` nets 0 — **CONFIRMED**

My run of the shipped stream (dt as the repro leaves it, 5 ms): **175 Move +
1 `Add to reference`**; the last Move and the Add are bit-identical
`A=B=C=−2.800000000 rad`. Applying the world fold:

| | A/B/C net | A/B/C peak | line payout |
|---|---|---|---|
| full stream | **0.0000°** | −160.4282° | **0.0000 mm** |
| same stream minus the final Add | **−160.4282°** | −160.4282° | **−42.0000 mm** |

Headless HP4 world (`RemoteSpoolSystem`, one command per 2 ms step): net
target 0.00000°, peak −160.428° on A/B/C; with `dt = 0.002` the mover is
dragged **8.35 mm** and returns to within **0.002 mm** of its start (46/46/49
detent slips en route). So the physical line tightening is undone, not merely
the commanded number.

Expected value: `G6` is only defined by hp-sim (`_parse_g6_command`, line
deltas in mm, `F`-timed ramp, file named `tighten.gcode`); neither RRF nor
Klipper in this repo implements `G6` (`GCodes2.cpp` has no `case 6`; Klipper
has none), so “Hangprinter G6” in the report is an internal product contract,
not a firmware one. On HP4 `Q=0, R=30, U=2:2:2:4, H=L=20` the worker's own
formula gives −42/30·2 = **−2.8 rad = −160.428°**, and RRF's line→motor map
(`HangprinterKinematics.cpp:466-477`, `k0=2·stepsPerUnitTimesR/k2`) reduces to
exactly the same value at Q=0. Arithmetic confirmed.

Which stream is wrong: the pair is self-cancelling under the engine semantics,
so it cannot represent either intent. A frame shift needs Move+Add (what the
block emits) but needs no 0.87 s `F`-timed ramp; a physical tightening needs
the Move only. The ramp, the `F` word, the variable `line_deltas_mm` and the
file's name all say physical move ⇒ the trailing `Add to reference`
(`moveCommander.js:409-411`) is the wrong part. Counter-evidence the report
should mention: `tests/js/hp-sim-3d/moveCommander.test.js:87-97` **asserts**
this exact command pattern (Moves then Add), so it is a deliberate, tested
design contradiction, not a stray line; a fix must change that test. The
behaviour is inherited from the removed Python app (`move_commander.py` at
`817a7773^`), which had the same Move+Add. None of this changes the net-zero
result.

Reachability: `grep` finds `tighten.gcode` only in `public/gcode/` — no
button, catalog entry, preset mapping (`uploadPresetConfig.js`) or test uses
it; the README never mentions G6. G6 is reachable only by uploading a `.gcode`
file containing it. Agree with the report's “uploads, not presets”.

Caveat on the reproducer: it leaves `commander.dt` at the 5 ms fallback, while
the app sends `set_dt` from the scene (HP4 = 2 ms), so its command count (176)
is 2.5× smaller than the app's (438). Net/peak and the conclusion are
dt-invariant; the count in the report table is not the app's.

## C2 — `G92 X5` target jump — **CONFIRMED WITH CORRECTION**

Command stream and target jump are exactly as claimed: **0 Move + 1 Add**
(`A:+0.000434, B:−0.285475, C:+0.285706, D:+0.000877 rad`), world target jumps
`B = +0.285475 rad = +4.282 mm` of B line for `G92 X5`, and `12.839 mm` when
issued after `G1 X10` — my probe reproduces both to the shown digits. Since
G92 must not move the machine (RRF `SetPositions`, Klipper `cmd_G92` set the
frame only), a nonzero target is wrong.

Corrections:

1. **The effector part was not measured.** The author's `probe_world.mjs`
   prints `eff null` (its rigid-body lookup finds no member with a
   `PositionComponent`), so “the effector is driven 5 mm in the −X direction”
   is an inference from the target, not a world measurement. I measured it
   properly through the shipped `ExtruderComponent.tipPos`.
2. **The visible outcome depends on the motor mode**, and the default HP4 mode
   is open loop (`StepperMotorComponent.closedLoop = false`; no `closedLoop` in
   `hp4.usda`). With worker and world both at dt = 2 ms:
   * `G92 X5` then `G1 X10` (the ordinary file flow): closed loop ends at
     **+4.983 mm** vs **+4.998 mm** for the equivalent plain `G1 X5` (0.015 mm
     error, transient only); **open loop (shipped default) ends at
     +0.538 mm** — a persistent **4.46 mm pose error**, with 7/5 missed steps
     on B/C caused by the one-step target kick.
   * terminal `G92 X5` (no following Move): closed loop settles at
     **−5.000 mm**, open loop at **−0.606 mm** after 6 s.
   So the report's “next G1 re-establishes the frame” holds only in closed-loop
   mode; the description should say “one-step target jump; the pose error is
   mode-dependent (0 mm closed loop, ~4.5 mm open loop after a following move,
   ~5 mm if the bad target is held)”. The target numbers quoted in the report
   are right.
3. Sign slip in the prose: the probe's own target jump is **+0.285475 rad**,
   i.e. B line **+4.28 mm**, C **−4.29 mm** (consistent with the effector
   going −5 mm in X). The report text says “B −4.28 mm, C +4.29 mm”.

Independent check scripts: `verify_g6_g92.mjs` (stream + world fold + world),
`verify_g6_physical.mjs` (mover out-and-back), `verify_g92_equiv.mjs`
(G92 spelling vs equivalent plain file), `verify_g92_models.mjs`
(open vs closed loop, matched dt).

## Other claims checked incidentally

* The control (`repro_preset_vs_upload_hp4.mjs`) passes: 0.0232 % max line
  payout, 0.4705 mm max preset deviation, 0 upload deviation, 0.4806 mm
  4-line residual — the flex attribution is plausible on these numbers.
* Report line references `moveCommander.js:353-411`, `:409-411`,
  `:332-351`, `remoteSpoolSystem.js:267-289`, `engine.rs:1568` are accurate
  (the same formula also appears at `:1782`; `rust_physics_engine.js:115`
  confirms the field order).
* Secondary G6 linearisation (0.628° = 0.255 mm on the layered machine): my
  independent evaluation of `angle = 2(H/L)U·len/(√(R²+len·k2)+R)` gives
  −102.639° vs the worker's −103.270°, matching the report; it is invisible
  while C1 zeroes the move, as stated.

## Verdicts

* **C1 CONFIRMED** (net 0.0000° vs −160.428°, 0.0000 vs −42.0000 mm of line;
  mover returns to 0.002 mm). Add caveats: the trailing Add is asserted by a
  shipped test; G6 is an hp-sim-only command, so the expected value is the
  app's own intent, not a firmware contract.
* **C2 CONFIRMED WITH CORRECTION**: target jumps +4.282 / +12.839 mm exactly as
  claimed; the “effector driven 5 mm” part is unmeasured in the report and is
  true only for closed-loop/held-target; in the shipped open-loop default the
  ordinary `G92` + next-move flow leaves a 4.46 mm persistent pose error
  instead. Per-line signs in the prose are inverted.

Latent / fragile: both findings need an uploaded file; no shipped button
reaches G6, and G92-with-nonzero-X is unusual in slicer output. The C2 effect
is motor-model dependent (open loop = missed steps, closed loop = harmless
transient), so any severity claim should name the mode.
