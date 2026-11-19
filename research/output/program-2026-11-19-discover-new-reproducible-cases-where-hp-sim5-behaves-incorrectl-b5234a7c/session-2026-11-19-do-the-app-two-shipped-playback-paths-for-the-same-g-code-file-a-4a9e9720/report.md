# Do the app's two shipped playback paths for the same G-code file agree?

Investigation: `session-2026-11-19-do-the-app-two-shipped-playback-paths-for-the-same-g-code-file-a-4a9e9720`
Program: *Discover new reproducible cases where hp-sim5 behaves incorrectly* (2026-11-19)
Revision under test: `56a66f77` (`git status`: only this artifact directory is untracked).
Environment: Linux x64, Node 26.13.0, 2 cores / ~1.6 GiB free, **no build, no browser, no Vite, no
product-code edits**. Both playback paths are pure JS; every number below comes from running the shipped
classes themselves. Inputs pinned by SHA-256 in `measurements.json → inputs`.

**Verdict.**

1. **The two shipped paths agree on HP4.** Same file (`public/gcode/draw_squares_bigger.gcode`), same
   machine config, same start state: line-payout travel differs by **≤ 0.023 %**, toolpath by
   **0.47 mm max / 0.22 mm rms** over a 17.7 m, 600 mm-square path, and the preset plan's toolpath is
   **never** beyond 0.078 % of the commanded geometry. The whole residual is a **line-length offset field
   of 0 … −1.14 mm** in the compiled plan that no effector position can explain (4-line consistency
   residual 0.48 mm rms) — the signature of RRF's *flex compensation*, which the shipped G-code switches on
   (`M666 F1`) and which the upload path hardcodes off. That is a documented model difference, not a wrong
   result. HP4 layered agrees even better (0.25 mm max).
2. **G6 is a real wrong result in the Upload path** (`hp-sim-3d/app/workers/moveCommander.js:353-411`): the
   commanded line payout is applied and then **undone** by the trailing `Add to reference`, so
   `public/gcode/tighten.gcode` delivers **0.0 mm instead of −42 mm per line (100 % of the move lost)**,
   after a 42 mm out-and-back excursion. Confirmed end-to-end in the shipped physics world.
3. G6 has **no second path**: RepRapFirmware implements no `G6`, so a G6 file cannot be compiled into a
   `.can`, and no preset button can play `tighten.gcode`. The question "do the two paths agree on G6" is
   undefined — there is only one path, and it is wrong.
4. Secondary: the G6 angle conversion is exact for `Q = 0` but linearised for `Q ≠ 0`, i.e. **0.61 % /
   0.255 mm too much line** for the layered machine (masked by finding 2). The same `Add to reference`
   mechanism makes `G92 X<nonzero>` move the tool: **−5.000 mm** (closed loop) / **−0.606 mm** (open loop,
   HP4's shipped mode) for a terminal `G92 X5` that must not move at all — measured, adjacent, and
   possibly part of today's upload-gap batch.

---

## Method

* **Preset path** — `integrations/rrf/rrfCanPlayer.js`, class `RrfCanPlayer`, instantiated exactly as the
  worker does (driver→axis map parsed from the machine config with
  `parseRrfMotorAxisMapFromConfigText`), fed the shipped `.can` as `FileFormat.RRF_CAN_BINARY` (which is what
  `commandPresetResolver.js:46` declares for every preset). `sendCommand` captured in-process; `asapMode`
  + `_targetWaitMs()=0` remove wall-clock pacing only (geometry/timing of the plan untouched).
* **Upload path** — `hp-sim-3d/app/workers/moveCommander.js`, class `MoveCommander`,
  `configure(configText)` with the **same** config file the app fetches for the active machine
  (`commandJobController.js:212-245` posts `machine.rrf_config`), then `run(ReadableStream)`; `sendCommand`
  captured the same way.
* **Angles → line lengths** — `lib/rrfModel.mjs` re-implements RRF's own map from
  `HangprinterKinematics.cpp` (`Recalc`, `CartesianToMotorSteps`, `MotorPosToLinePos`). It reproduces the
  app's forward model `pos_to_motor_pos_samples_deg` to **9e-13 deg** on the hp4 geometry (probe_model.mjs).
* **Toolpath** — the 4 line lengths are inverted to an effector position by Gauss–Newton on the *same*
  anchors (`positionFromLines`). Validation: the upload path's own stream reconstructs the commanded G-code
  polyline to **6e-12 mm**, so the inverse is exact and the two paths are measured with one ruler.
  The preset plan has 4 lines for 3 unknowns; its least-squares residual is reported and is itself evidence
  (see below).
* **Start state** — identical by construction and verified: `G28` maps to `homeall.g` = `G92 X0 Y0 Z0`
  (no motion) in RRF, and the worker ignores `G28` while starting at (0,0,0) with zero angles. First preset
  sample is 1.3e-4 rad; first upload Move is at t = 1/num_steps. Extrusion is not compared (out of scope).

Reproduce everything: `node measure.mjs` (writes `measurements.json`), `node repro_preset_vs_upload_hp4.mjs`,
`node repro_g6_net_zero.mjs`, `node repro_g92_moves_tool.mjs`, `node probe_world_g6.mjs`.
Each run is < 5 s and < 200 MB RSS; the two world-based scripts need the shipped wasm solver (~100 MB).

## Generated plans used

| case | preset file (`RrfCanPlayer`) | G-code (`MoveCommander`) | config |
|---|---|---|---|
| HP4 control | `draw_squares_bigger_hp4_no_buildup.can` | `public/gcode/draw_squares_bigger.gcode` | `config_hp4.g` (Q=0, R=30, U=2:2:2:4, O=1, H=L=20) |
| HP4 layered | `draw_squares_bigger_hp4_w_line_layers.can` | same | `config_hp4_w_line_layers.g` (Q=0.6366197723675814, R=46.6045) |

Both `.can` files were produced from that G-code with that config by
`scripts/compile_machine_commands.py` (real RRF → CSV → `csv2can.py`), so the comparison is like-for-like:
**the known 2026-11-17 catalog-vs-RRF radius difference cannot appear here** — both paths read the same
config text (the catalog radius only affects the rendered scene, not the uploaded command stream).

---

## 1. NEGATIVE (the informative one): the control agrees

`repro_preset_vs_upload_hp4.mjs` → `PASS`, exit 0.

| quantity | preset `.can` | upload | difference |
|---|---|---|---|
| Move commands | 45 083 | 17 700 | segmentation only (M669 S130 T0.5) |
| Σ\|Δline\| A/B/C/D (mm) | 9983.97 / 12315.28 / 11486.52 / 2165.03 | 9982.34 / 12312.42 / 11485.49 / 2165.51 | **+0.016 / +0.023 / +0.009 / −0.022 %** |
| Σ\|Δangle\| A/B/C/D (deg) | 38135.94 / 47040.90 / 43875.27 / 16539.60 | 38129.72 / 47029.98 / 43871.34 / 16543.27 | ≤ 0.023 % |
| effector path travel (mm) | 17693.534 | 17696.057 (G-code: 17697.056) | −0.0143 % (each within 0.02 % of the G-code length) |
| deviation from commanded polyline | **max 0.4705 mm, rms 0.2163 mm** | max 6.3e-12 mm | — |
| net line at end (should be 0) | −0.5596 / −0.5596 / −0.5596 / −0.0884 mm | 0 / 0 / 0 / 0 | flex offset at the origin pose |

Tolerance statement: 0.47 mm on a 600 mm square is 0.078 % of the square side and 0.0027 % of the total
travel; 0.023 % of line payout is 2.3 mm on 10 m of line. There is **no** gross geometry error, no axis
permutation, no sign error, no scale error: a radius/scale mismatch would be zero at the line origin and
proportional to line length, but the preset's offsets are non-zero (−0.56 mm) *at the origin* and its
per-axis ratios are not a single scale factor.

### What the residual is

`measurements.json → cases.hp4_no_buildup.lineErrorVsIdealMm` and `.flexSignature`:

* every sample of the preset plan has line positions **shortened** relative to ideal geometry:
  max 0.000, min −1.104 / −1.143 / −1.143 / −0.149 mm (rms 0.559 / 0.574 / 0.565 / 0.086);
  the upload path is exactly 0 by construction (it *is* the ideal map);
* the offset varies smoothly with pose (0.4-0.6 mm on the 400 mm squares → up to 1.14 mm on the 1200 mm
  squares) and
  is **not a rigid transform**: inverting the 4 line lengths leaves a residual of **0.71 mm max /
  0.48 mm rms** (upload: 3.8e-12). A wrong anchor, a wrong spool radius or a coordinate offset would be
  consistent; a per-line compensation is not;
* the per-axis weighting matches RRF's flexible-line model
  `flex[i] = −F[i]/(SpringK·U²·N)` (`HangprinterKinematics.cpp:505-519`): measured mean
  offset(A) : offset(B) : offset(C) : offset(D) = **6.26 : 6.80 : 6.64 : 1**, predicted
  `1/(U²N)` = 6 : 6 : 6 : 1 (U=2:2:2:4, N=2:2:2:3). The same shape appears in the layered run
  (4.80 : 5.06 : 4.99 : 1);
* it only exists when the G-code enables it: both shipped files contain `M666 F1`
  (`draw_squares_bigger.gcode:3`), which sets `flexEnabled = true`
  (`HangprinterKinematics.cpp:294-306`);
* the upload path never models it: `moveCommander.js:39` (`this.use_flex = false`) and
  `configure()` (`:59-83`) parses only `Q R U L H O` — `F W S I X Y` are ignored.

Classification: **explained model difference** (flex compensation is a firmware feature that compensates
real line stretch; the sim's uploaded-line model is rigid). Not a defect in either path.

## 2. HP4 layered agrees better than the control

| quantity | preset `.can` | upload | difference |
|---|---|---|---|
| Σ\|Δline\| A/B/C/D (mm) | 9983.12 / 12313.98 / 11485.77 / 2165.19 | 9982.34 / 12312.42 / 11485.49 / 2165.51 | +0.008 / +0.013 / +0.003 / −0.015 % |
| path travel (mm) | 17697.044 | 17696.057 | +0.0056 % |
| deviation from polyline | max 0.2545 mm, rms 0.0638 mm | 6.3e-12 mm | — |
| line offset vs ideal (rms) | 0.199 / 0.184 / 0.182 / 0.035 mm (all ≤ 0) | 0 | flex again |
| 4-line residual | 0.593 mm max / 0.161 mm rms | 3.8e-12 | — |

So layering neither breaks the agreement nor interacts with the radius question; the flex offsets in the
regenerated layered plan are simply smaller. The Q/R values are identical on both sides, which is why the
0.47–0.85 % catalog/radius discrepancy from 2026-11-17 is invisible here (it is a scene/rendering
parameter, not a command-stream parameter).

### Per-G-code-move table (control, abridged)

`measurements.json → cases.hp4_no_buildup.perGcodeSegment` has all 26 segments; every segment behaves the
same way — the preset path tracks the commanded corner-to-corner line with a smooth negative line offset.

| segment (move) | length mm | preset max dev mm | preset rms mm | line offset at end A/B/C/D mm |
|---|---|---|---|---|
| 0 (200,200)→(200,−200) | 282.8 | 0.168 | 0.156 | −0.47 / −0.63 / −0.53 / −0.08 |
| 6 first 600 mm side | 600.0 | 0.194 | 0.174 | −0.63 / −0.55 / −0.37 / −0.07 |
| 11 first 800 mm side | 800.0 | 0.214 | 0.184 | −0.73 / −0.62 / −0.31 / −0.09 |
| 16 first 1000 mm side | 1000.0 | 0.308 | 0.215 | −1.03 / −0.85 / −0.31 / −0.11 |
| 22 worst 1200 mm side | 1200.0 | **0.470** | 0.305 | −0.78 / −0.19 / −1.14 / −0.05 |
| 25 (600,600)→(0,0) | 848.5 | 0.000 | 0.000 | — |

The 0.01125° step quantum of the `.can` (1/16 microstep at 200 steps/rev) is 0.0099 mm of line — far below
the offsets above, so step quantisation cannot be what we are seeing.

---

## 3. FINDING — G6 in the upload path is applied and then undone (net 0)

*Reproducer*: `node repro_g6_net_zero.mjs` → exit **1**.

Input: the shipped `public/gcode/tighten.gcode` = `G6 A-42 B-42 C-42 F5000`, machine `config_hp4.g`.

| | value |
|---|---|
| expected (Hangprinter G6 = line delta; also `moveCommander`'s own formula) | A/B/C = **−2.8 rad = −160.428°** (= −42 mm of line, R=30, U=2, H/L=1) |
| emitted stream | **175 `Move` + 1 `Add to reference`** (`Add`: A=B=C=−2.8 rad) |
| net target change (`commandedAngle − deltaAngle`) | A/B/C = **0.000000°**; D = 0 |
| peak excursion during the move | **−160.428°** (the 42 mm really is paid out, then pulled back) |
| line payout actually delivered | **0.0000 mm of −42 mm (0.0 %)** |
| machine used for the comparison | HP4 control; layered gives the same 0.0 % (peak −103.27°) |
| command count vs `dt` | 175 `Move` at the 5 ms fallback, **437 `Move` (438 commands) at HP4's 2 ms tick**; `net` and `peak` are identical in both (`measurements.json → g6.*.dtVariants`) |

Confirmed **inside the shipped world** (`probe_world_g6.mjs`: headless HP4, real `RemoteSpoolSystem`,
one command per physics step, 176 steps + 200 settle steps):
`net 0.00000° / peak −160.428°` on A, B and C.

Code path: `moveCommander.js:353-411`; the Move ramp is emitted at `:367-405`, `current_angles_rad` is
advanced at `:407`, and `:409-411` then emits `Add to reference` carrying **the same** `deltas_rad`.

World semantics: `hp-sim-3d/app/remoteSpoolSystem.js:267-289` (`Move` sets `commandedAngle`, `Add to
reference` shifts `deltaAngle`) and `src/wasm/cable_solver/src/engine.rs:1568`
(`let target = e[107] - e[108]`, i.e. `commandedAngle − deltaAngle`). The two contributions cancel exactly.

**This is a tested-in contradiction, not a stray line.** `tests/js/hp-sim-3d/moveCommander.test.js:82-97`
runs a one-line `G6 D1 I1` file and asserts at `:94-96` that the **last** command is `Add to reference`
with `D = U/R` — i.e. the test pins exactly the command that cancels the move, while never checking the
net motion. The Move ramp is clearly the intended motion: it is sized by the `F` word
(175 = ceil(72.75 mm ÷ (5000/60 mm/s) ÷ 5 ms)), `tighten.gcode` exists to shorten the lines, and
there is no other G6 semantics anywhere in the repo (no `G6` in RRF, Klipper, `src/python` or the docs),
so `G6` + `Add to reference` cannot both be meant as the same displacement.

Why this is not a documented model difference: `Add to reference` means "the user coordinate frame moved"
— that is what `G92` needs; a physical move must not touch it. With `G6` the Moves alone would deliver
exactly −42 mm. (If the stream is preceded by a bridge `Rebase reference`, the residual is +0.92° instead of
0.000° — still 0.6 % of the intended move, `measurements.json → g6.*.perAxis`.)

*Severity/likelihood*: any uploaded Hangprinter G6 file (the mechanism G6 exists for: manually tensioning
lines) silently does nothing net, after yanking the lines 42 mm and back. `tighten.gcode` itself is shipped
in `public/gcode/` but is not referenced by any button, catalog entry or test, so the blast radius is
"uploads", not "presets".

## 4. FINDING (smaller, same block) — G6 line→angle is linearised for Q ≠ 0

Same file, layered machine (Q = 0.6366197723675814, R = 46.6045). RRF's own map
(`HangprinterKinematics.cpp:466-477`) is
`angle = 2·(H/L)·U·len / (√(R² + len·k2) + R)`, `k2 = −U·O·Q`; `moveCommander.js:355-357` uses
`angle = len/R·U·H/L`, which is exact only for `Q = 0`.

| axis | exact angle for −42 mm | worker | error | in mm of line |
|---|---|---|---|---|
| A/B/C | −102.642° | −103.270° | **−0.628°** | **−0.255 mm (0.61 %)** |
| D | 0 | 0 | 0 | 0 |

`O` (lines per spool) and the buildup term are simply absent from the linear form; with `O = 1` and
`Q = 0` (both HP4 configs use `O = 1`) there is no factor error at all — the HP4 control is exact to
1e-13 deg. So this is a 0.6 % geometry error on the layered machine, not a factor-of-N error, and it is
currently invisible because finding 3 zeroes the whole move.

## 5. FINDING (adjacent) — `G92 X<nonzero>` moves the tool

*Reproducer*: `node repro_g92_moves_tool.mjs` → exit **1** (shipped headless HP4 world, open loop as
shipped).

`G92` must redefine the coordinate without moving the machine. The Upload path emits **only** the reference
shift and no matching `Move` (`moveCommander.js:332-351`), so
`target = commandedAngle − deltaAngle` jumps by `−delta` at the `G92` line.

Command level (`probe_g92.mjs`, HP4, `G92 X5` from the origin):

| axis | `Add to reference` (rad) | target jump = −Add (rad) | implied line change |
|---|---|---|---|
| A | +0.000434 | −0.000434 | −0.0065 mm |
| B | −0.285475 | **+0.285475** | **+4.282 mm** |
| C | +0.285706 | **−0.285706** | **−4.286 mm** |
| D | +0.000877 | −0.000877 | −0.0066 mm |

B's line is paid out and C's reeled in (B's anchor is at +x), i.e. the tool is pushed towards −x; the target
jump is 16.37°, not 0.

Measured end-to-end on the same headless HP4 world — mine, `probe_world.mjs` / `repro_g92_moves_tool.mjs`,
open loop as shipped:

| case | target jump | motor actually turned (encoder) |
|---|---|---|
| terminal `G92 X5` | B +16.3565°, C −16.3697° | B **+9.1565° = +2.40 mm of line**, C −1.9697° = −0.52 mm |
| `G92 X5` then `G1 X10` | B +16.3565°, C −16.3697° | B −1.9432°, C −5.2171° (includes the legitimate move) |

Independent review (same world, tool position, both motor modes):

| case | correct | closed loop | open loop (HP4 default) |
|---|---|---|---|
| terminal `G92 X5` | 0 mm | **−5.000 mm** | **−0.606 mm** |
| `G92 X5` then `G1 X10` | +4.998 mm | +4.983 mm | **+0.538 mm (4.46 mm error, 7/5 missed steps)** |

So the realised error spans nearly the whole 5 mm in closed loop and shrinks to 0.5–0.6 mm in open loop:
**the size is motor-mode dependent, and HP4 ships open loop.** This is why the defect can hide behind
closed-loop servoing while still leaving a persistent multi-millimetre offset in the shipped configuration.
`G92`-coordinate handling is not on my exclusion list (only `G92 E` is), but it is close to the batch
"G0/G91/arcs/units/dwell gaps" confirmed today, so it is reported as adjacent rather than as the headline.

The preset path cannot show findings 3-5: `RrfCanPlayer` never emits `Add to reference` (the
`bucketAddToReference` machinery at `rrfCanPlayer.js:255-270` / `:796-810` is never populated), RRF has no
`G6` at all, and RRF's `G92` is a pure coordinate change with no motion, so a compiled plan simply contains
no command for it.

---

## Strongest negatives (with tolerances)

* **N1 — preset vs upload, HP4 control: agree.** 0.023 % line payout, 0.47 mm max toolpath, 0.0198 % path
  length. Bounded by, and entirely attributable to, the flex offset field.
* **N2 — preset vs upload, HP4 layered: agree, tighter.** 0.015 % line payout, 0.25 mm max toolpath.
  The known 0.47–0.85 % catalog/radius difference does **not** appear, by construction: both paths consume
  the same `config_hp4_w_line_layers.g`.
* **N3 — segmentation and ramping do not move geometry.** 45 083 vs 17 700 commands (RRF plans at
  130 segments/s, ≥ 0.5 mm), yet the preset toolpath never leaves the ±flex envelope of the commanded
  polyline. Acceleration ramping is timing-only here; the geometry is the endpoint interpolation.
* **N4 — step quantisation is negligible.** 0.01125°/step = 0.0099 mm of line at R=30.
* **N5 — G6 has no preset counterpart.** RRF has no `G6` handler (`GCodes2.cpp` dispatch has no `case 6`;
  the only "G6" strings in the firmware are `G60`/`G68`), there is no `.can` for `tighten.gcode`, and the
  catalog has no `compiled_commands` entry for it. A G6 file can only ever take the upload path. This is a
  capability gap, not a disagreement.
* **N6 — no axis-mapping/units/scale bug found in either path.** Driver→axis maps agree (40/41/42/43 →
  A/B/C/D from `M584`), and the preset plan's line positions match ideal geometry up to a sign-definite
  additive offset (a scale error would vanish at the line origin; this offset is −0.56 mm there).

## Limitations / what would change the conclusion

* **Flex attribution is by signature, not by re-running the firmware solver.** I did not re-run RRF with
  `M666 F0` to produce a flex-free control: the prebuilt `RRF/build/rrf_simulator` cannot open files under a
  path longer than `MaxFilenameLength = 120` (`Configuration.h:143`) and this artifact path is 230 chars;
  using a short temp directory or building a fresh binary is outside the host constraints. If a flex-free
  compile of the same G-code shows the same 0.47 mm deviation, finding 1's classification becomes a
  *second* wrong result (a geometric defect in the compiled plan or in the decoder); if it shows ~0, the
  classification is confirmed. The per-axis 6:6:6:1 weighting, the sign, the pose dependence, the
  non-rigid residual and the magnitude (bounded by the configured `I`/`X` force limits over `S`) all point
  to flex.
* Only `draw_squares_bigger` was compared over its full length. `Hangprinter_logo6` (the "Print Logo"
  preset) was not compared end-to-end: its `.can` is 6 MB ≈ 1 M moves and the two passes exceeded the
  runtime budget (the 45 k-move case already takes 1.5 s of CAN decode plus 17.7 k inverse solves). The
  geometric claim is per-file independent, but the logo is untested.
* The G6/G92 defects are established at the command-stream level and confirmed in the shipped world with a
  few hundred to a few thousand physics steps (`probe_world_g6.mjs`, `repro_g92_moves_tool.mjs`), not with a
  full print or a browser. No hardware validity is claimed: on real hardware the same G6 stream would
  produce a 42 mm tension-relief excursion and back — whether the lines would then stay slack depends on
  the firmware's standstill behaviour, which is out of scope. For G92 the tool error is motor-mode
  dependent (5.000 mm closed loop vs 0.606 mm open loop; HP4 ships open loop), so the *sign and mechanism*
  are robust but the *size* is not a single number.
* Only HP4 was examined (plus the layered variant). HP5, CubeCorners, Skycam and Slideprinter presets exist
  with their own compiled plans and are untested here.

## Evidence index

| file | what it is |
|---|---|
| `lib/players.mjs` | harness: instantiate both shipped classes, capture the world-facing commands |
| `lib/rrfModel.mjs` | exact RRF line↔motor map (validated to 9e-13 deg against `kinematics.js`), 4-line inverse |
| `lib/paths.mjs` | streaming accumulators, toolpath statistics, line-error field |
| `measure.mjs` → `measurements.json` | all quantitative results, per-segment table, G6 table, input hashes |
| `repro_preset_vs_upload_hp4.mjs` | control: exits 1 if the two paths ever disagree beyond the flex envelope |
| `repro_g6_net_zero.mjs` | **finding 3**: exits 1 (net G6 payout is 0 of −42 mm) |
| `repro_g92_moves_tool.mjs` | **finding 5**: exits 1 (target jump and real motor rotation at `G92 X5`) |
| `probe_g6.mjs`, `probe_g92.mjs` | per-axis G6 / G92 tables from the worker's own stream |
| `probe_world_g6.mjs`, `probe_world.mjs` | same defects confirmed in the shipped headless world + physics |
| `probe_model.mjs` | kinematics cross-validation and round-trip |
