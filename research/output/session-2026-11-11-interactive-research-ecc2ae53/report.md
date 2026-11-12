# HP4 Print Squares corner bumps: closed-loop servos saturate under firmware flex pretension

Status: complete for the stated scope (simulation only; no hardware data).

## Question

In the browser, **Print Squares** on HP4 with line layering off, closed-loop motors on and live QC shows bumps near the corners, mostly in the +x/−y quadrant. What causes them? Are they a simulation artifact or plausible physics?

## Answer (short)

The bumps are **handovers between saturated closed-loop servos**. Every A/B/C motor in the HP4 scene has a 0.5 Nm limit (`stepper:holdingTorque`). The bundled CAN stream asks for more line pretension than that, so one of A/B/C is always current-limited. It lags its target by 0.15–0.40 rad, which is 4.5–12 mm of line at r = 30 mm, and acts as a constant-torque tensioner. HP4 has four motors for three translational DOF, so the system is redundant: the other three motors hold position and the limited motor yields. When the effector crosses the region where another motor's demand becomes the largest, the yielding role jumps to that motor and the tip steps by ~2 mm. Near the +x/−y corners A and B hand over (e.g. 500 mm square, ~200 mm before the corner, t ≈ 44.6 s). After the corner, A stays deep in saturation, and the side sags ~2.8 mm inward and then recovers.

The excess pretension comes from the G-code, not the physics engine. `draw_squares_bigger.gcode` line 3 is `M666 F1`, which **enables RRF flex compensation** with `I10`/`X40` planned forces. RRF's `FlexDistances` computes `flex = −F/(k·MA)` with `k = S/(d·MA + Y)` for **one line per spool**. The parameter `O` (lines per spool) enters only the buildup term. The HP4 scene has **two lines on each A/B/C spool** (AL/AR, …) and **three D lines**, so the simulated spool loads are ≈2.3× what the firmware plans (table below). The sim delivers what the commanded line lengths imply.

**Verdict:** the bump mechanism is plausible physics. A torque-limited position servo in an over-constrained CDPR saturates and hands over exactly like this, and real drivers with current limits would too. What is *not* physically intended is the size of the demand. That comes from the firmware flex model ignoring the line count, combined with a 0.5 Nm motor. Real hardware with the same line count, firmware and motors would show the same thing. Whether the real HP4 matches the scene's 2/3-line routing and 0.5 Nm motors was not verified here. No engine defect was found.

## What was tested

All runs use the production headless pipeline (`createHeadlessWorld`, Rust **native** engine, dt = 2 ms, 1 substep, HP4 `hp4_rigid_body.usda`, unlayered bake, `config_hp4.g` axis map). The CAN stream is decoded by `RrfCanPlayer` in ASAP mode, exactly like the browser's unlayered preset. Revision `ba4a8101`; native artifact sha256 `b12282dd…`; scene sha256 `153b3dd6…`; bundled CAN sha256 `6b3d9f4a…`. The metric is signed **contour error**: the Chebyshev radius of the deposited tip (extrusion events) minus the commanded square half-size; + means outside.

| Run | Motors | CAN | Contour RMS / p99 / max (mm) | ABC servo saturated | Slips |
|---|---|---|---|---|---|
| E1 `closed_unlayered` | closed, 0.5 Nm | bundled | 1.38 / 2.72 / 2.96 | always one motor | 0 |
| E2 `open_unlayered` | open, 0.5 Nm | bundled | 5.79 / 14.5 / 15.4 | n/a | B 3, C 3 (at ~30 ms) |
| E3 `closed_layered` | closed, layered scene + layered CAN | bundled layered | 1.72 / 3.46 / 3.82 | always one motor | 0 |
| E4 `open_layered` | open, layered | bundled layered | 5.76 / 13.8 / 31.9 | n/a | A1 B2 C1 |
| E5 `closed_unlayered_tau2` | closed, **2 Nm** (scene edit) | bundled | 0.79 / 1.74 / 2.32 | never (after start-up) | 0 |
| E6 `cl_regendraw_squares_bigger` | closed, 0.5 Nm | regenerated, `M666 F1` | 1.38 / 2.72 / 2.96 (= E1) | 100 % of time | 0 |
| E7 `cl_regen_rs_noflex` | closed, 0.5 Nm | regenerated, `M666 F0` | 1.31 / 7.13 / 17.3 | 0 % | 0 |
| E8 `ol_regen_rs_noflex` | open, 0.5 Nm | regenerated, `M666 F0` | 1.37 / 7.50 / 17.9 | n/a | **0** |
| E9 `cl_regen_rs_s40k` | closed, 0.5 Nm | regenerated, `M666 S40000` + `F1` | **0.37 / 0.90 / 1.15** | 16.5 % (C only, ≤0.11 rad) | 0 |

"Saturated" means the ABC servo error is above 0.05 rad. The servo saturates at an error of τ_max/(τ_max·pole pairs) = 0.02 rad (`servo()` in `src/wasm/cable_solver/src/engine.rs`).

### Observations

1. **E1:** the A/B/C encoder−commanded error alternates between ≈0 and 0.15–0.4 rad. Each switch of the lagging motor coincides with a ~1–2 mm step in contour error ([timeline](scratchbook/timeline.png), [corner zoom](scratchbook/corner_500.png)). D never saturates. The whole print also sits ~1.5 mm inside the commanded squares.
2. **E5:** with 4× torque, the servos track (error ≈ 0) and the steps disappear. The motors then need 0.78–0.85 Nm median and up to 1.38 Nm, which is above the stock 0.5 Nm 80 % of the time. The path still shows a smooth ~1 mm rigid offset. There is also a 0.45 s start-up chatter limit cycle (±0.2 rad) caused by the stiffer servo (Kp = 100 Nm/rad) at 2 ms ([start](scratchbook/start_tau2.png)). That chatter is a property of the test variant only.
3. **Start-up:** the first 16 commanded steps (32 ms) **shorten all four lines at once** (A −0.078, B −0.249, C −0.124, D −0.139 rad). No real motion does that. It is the flex pretension being applied right after `G28`. Line tension jumps from ~6 N to 9 N (0.5 Nm) or 14.5 N (2 Nm) and stays there; B saturates immediately ([start](scratchbook/start_default.png)). In open loop the same ramp causes the "instant" B/C slips at ~30 ms (E2). Without flex (E8) open loop has **zero** slips.
4. **Planned vs simulated loads** (E5, servos tracking; firmware forces approximated by min-norm tensions in [10, 40] N under gravity, [script](scratchbook/planned_vs_sim.py)):

   | pos (mm) | planned F_ABCD (N) | planned spool load F/MA | sim spool load (Σ lines) | ratio |
   |---|---|---|---|---|
   | (200,−200) | 18 / 17 / 12 / 10 | 9.0 / 8.4 / 5.8 / 2.5 | 29.7 / 28.1 / 20.3 / 6.1 | 3.3 / 3.3 / 3.5 / 2.4 |
   | (500,−500) | 40 / 37 / 10 / 23 | 20 / 19 / 5 / 5.6 | 45.6 / 43.0 / 8.6 / 8.6 | 2.3 / 2.3 / 1.7 / 1.5 |
   | (−500,500) | 10 / 19 / 27 / 14 | 5 / 9.6 / 13.5 / 3.6 | 13.4 / 23.5 / 31.6 / 6.4 | 2.7 / 2.5 / 2.3 / 1.8 |

   The ratio is ~2 or more. Two lines per A/B/C spool explain a factor of 2, and the E5 start-up chatter adds some ratcheted pretension. My QP is an approximation of RRF's `StaticForcesQp`, not a port, so I treat the ratios as indicative only.
5. **E7/E8 (no flex):** no saturation and no slips. However, the outer −y corners show slack-line wobble and excursions of up to 17 mm. With only gravity (0.081 kg) for pretension, lines go slack in parts of the workspace. This is also plausible physics: the machine needs *some* pretension.
6. **E9 (pretension halved via S = 40000, which emulates two lines per A/B/C spool in the flex model):** the bumps disappear and contour RMS drops from 1.38 to 0.37 mm with max 1.15 mm. Only C saturates briefly, with a small error.

## Failed or partial trials

- The first `M666 S40000` attempt was placed after `G28`. A geometry change un-homes RRF, so the run failed with `G1: insufficient axes homed`. The fix was to set `S` before `G28` ([gcode](scratchbook/can/_rs_s40k.gcode)).
- My regenerated baseline CAN is not byte-identical to the bundled one: D differs by ≤0.006 rad and the file is 276160 vs 276290 bytes. Replay results are identical to 3 decimals (E1 vs E6).
- Rerun visual inspection was skipped. Evidence is numeric telemetry plus matplotlib plots of the deposited tip path.

## Reproduce

```
cd research/output/session-2026-11-11-interactive-research-ecc2ae53/scratchbook
node squares_replay.mjs <label> 400 <closed 1|0> <layered 1|0> 1 [can path]     # writes <label>.json (stored here gzipped)
SCENE=<scene.usda> node squares_replay.mjs ...                                 # e.g. hp4_torque2.usda (holdingTorque 2.0)
../../../../.venv/bin/python contour.py <run>.json ; timeline.py <run>.json t0 t1 out.png ; corner.py 500 <runs>.json
./RRF/build/rrf_simulator --vsd RRF/run/vsd --gcode gcodes/<g>.gcode --can-log <out>.csv -c sys/config_hp4.g ; .venv/bin/python RRF/csv2can.py <out>.csv
```

Gunzip the `*.json.gz` files before running the analysis scripts. The variant G-code and CAN files are in [scratchbook/can](scratchbook/can).

## Useful next steps (proposed, not done)

1. **Firmware:** make `FlexDistances` divide by `linesPerSpool` (or document that `S` must be EA × lines). Alternatively set `O2:2:2:3` in the HP4 configs and regenerate the bundled HP4 CAN files. Then check whether planned and simulated spool loads agree within ~10 %.
2. **Check motor limits against planned forces:** at max planned force 40 N, MA 2 and r = 30 mm, one line per spool needs 0.6 Nm. That alone exceeds the 0.5 Nm scene motors, so `X40` (and `X170` in the layered config) is inconsistent with the motor. Either lower `M666 X` to what the motor can hold, or give the scene the real HP4 motor torque. Hardware data are needed to decide which.
3. **QC:** expose servo saturation (|error| > 1/pole pairs) as a QC/HUD flag. It explains bumps directly and is invisible today because slips stay at 0.
4. The ~1 mm rigid offset and ~1.5 mm shrink that remain at 2 Nm were not investigated. A likely cause is the point-anchor kinematic model vs. the scene's separated L/R line routing.

## Follow-up: `M666 N` / `winch_lines_per_axis` and regenerated command streams

New parameter (not yet committed; RRF and Klipper submodules plus hp-sim5 configs are in the working tree):

- **RRF** `HangprinterKinematics`: `M666 N<n>:<n>:…` gives the number of parallel lines per axis, default 1 for all 8 anchors. `FlexDistances` now uses `flex = −F/(k·MA·N)`. It is also echoed by `M666` and written by `M500`. `O` (lines per spool) still only affects buildup.
- **Klipper** `kin_winch.c` and `winch.py`: new option `winch_lines_per_axis`, default 1, with docs in `Config_Reference.md`. The new test `test_parallel_lines_share_flex` passes along with the other 8 winch tests.
- Configs: `N2:2:2:3` / `winch_lines_per_axis: 2, 2, 2, 3` for HP3 and HP4 (two lines each on A/B/C, three D lines in the scenes), and `2:2:2:2` for skycam. Cubecorners and slideprinter keep the default 1.
- Regenerated all 20 `public/RRF_CAN_commands/*.can` and the 4 hp3 `public/mcu_commands` pairs. Slideprinter Klipper outputs are unaffected by `N` and were not regenerated.
- Verification: compared with the old `S20000` HP4 Squares stream, the flex part of the new stream is 0.49–0.51× for A/B/C and 0.33–0.34× for D at five sampled steps ([flex_ratio.mjs](scratchbook/flex_ratio.mjs)).
- **Failed trial:** the first `DefaultLinesPerAxis` copied the 5-element initializer pattern, so anchors 6–8 were 0. Flex became a divide-by-zero, and cubecorners J/L/O got no moves (CAN files ~16 % smaller). Fixed with an explicit 8-element default; the regenerated cubecorners squares files are then byte-identical to the old ones.

Results with the new bundled HP4 Squares streams (headless, native, 0.5 Nm):

| Run | Contour RMS / p99 / max (mm) | ABC servo saturated | Slips |
|---|---|---|---|
| closed, unlayered | **0.36 / 0.85 / 1.11** (was 1.38 / 2.72 / 2.96) | 13.8 % of time, ≤0.105 rad | 0 |
| closed, layered | 0.68 / 2.81 / 4.30 (was 1.72 / 3.46 / 3.82) | 21.7 %, A up to 0.35 rad | 0 |
| open, unlayered | 7.06 / 19.0 / 33.8 (was 5.79 / 14.5 / 15.4) | n/a | B 3 at 0.72 s, C 2 at 73 s |
| open, layered | 7.83 / 29.8 / 49.8 (was 5.76 / 13.8 / 31.9) | n/a | B, A, C |

**Repo physics gates (open-loop layered HP4 logo):** `hp4_logo_stability` (native and Wasm) and `hp4_open_loop_substeps` (2/4/16) fail with the new logo stream: rocking 0.102 rad/s at step 1964 and an A slip at 3.9 s. All 15 gates pass with the old files. The cause is *not* `N`. The bundled layered HP4 CAN files were generated on 2026-04-29 with `M666 I3`. Commit `11044351` raised the config to `I10` without regenerating them. The regenerated stream therefore carries more startup pretension than before (−0.041 vs −0.020 rad on A at step 25, [first_n.mjs](scratchbook/first_n.mjs)). With `I3` + `N` the 20 s open-loop logo prefix has 0 slips. The old file can be reproduced with `git show ba4a8101:public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can`.

Open question for the owner: keep `I10` for layered HP4 (closed loop benefits, open-loop gates must be revisited) or return to `I3` (gates pass).

**Decision (owner, 2026-11-11):** layered HP4 goes back to `I3` (`config_hp4_w_line_layers.g`, `printer-hp4-linux-mcu-with-buildup.cfg`). The unlayered HP4 config keeps `I10`. After regenerating both layered HP4 CAN files, all 15 physics gates pass (pinhole gate, HP4 logo stability, HP4 stability, open-loop substeps, motor drag) and so do the preset-selection Jest tests. Closed-loop layered Squares: contour RMS **0.24 mm**, p99 0.67, max 1.10, 0 slips (was 1.72 / 3.46 / 3.82).
