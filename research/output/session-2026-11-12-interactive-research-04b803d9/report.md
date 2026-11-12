# HP4 logo path error: two firmware flex errors plus an unconverged HP4 solver

Status: complete for the stated scope (simulation only, no hardware data). Repo `8e2df52b`, RRF submodule `7eb7a97bf` (unmodified; instrumentation lived in a private build copy).

## Question

In the browser, HP3 (no layering, closed loop) prints the logo with path deviation max 0.27 mm. HP4 under the same settings gives RMS / 95% / max **0.39 / 0.62 / 0.69 mm**, mean offset (0.16, −0.42, 0.14) mm. Is there a theoretical error in the firmware?

## Answer

Yes, two. A third, larger part of the XY error is a simulator artifact. Together they explain the whole gap between HP4 and HP3.

1. **Firmware: flex compensation over-reels by a factor MA.** `HangprinterKinematics::FlexDistances` sets `flex = −F/(k·MA·N)`, `k = S/(MA·d + Y)`. That `flex` is in effector-distance units, and `stepsPerMm` already contains MA, so the spool reels `MA·flex = F·(MA·d+Y)/(S·N)` of extra line. Each line segment carries `T = F/(MA·N)`, which stretches the line by `T·(MA·d+Y)/S`. The correct value is therefore **`flex = −F/(k·MA²·N)`**. For HP3 (MA 1) the two are identical. For HP4 the firmware applies 2× the needed pretension on A/B/C and 4× on D. Klipper's `kin_winch.c:617` has the same expression (not tested).
2. **Firmware: `M666 F1` after `G28` shifts the believed position.** `M666` does not rehome. At the next standstill, RRF runs the flex-aware forward kinematics on motor steps that are still (0,0,0,0), i.e. untensioned lines. No pose fits, and the least-squares FK returns (0.0005, 0.0005, **0.163**) mm with cost 1.96. The logo G-code never commands Z, so the whole print runs at Z = 0.163 mm. A `G1 X0 Y0 Z0` right after `M666 F1` does not help, because RRF drops zero-length moves and the tension is never sent. This is the normal RRF move pipeline, not the host shim, so it should happen on hardware too.
3. **Simulator: the HP4 scene's `cablePath:solverIterations = 2` does not converge** under pretension at 1 substep. Its static equilibrium is biased: realized tension is only 0.64× the commanded stretch implies, and the response is asymmetric (Gauss-Seidel order). With uniform flex at home this shifts the effector (0.08, −0.21) mm, i.e. it creates the XY offset that is proportional to flex. With 10–20 iterations or 4–8 substeps the sim matches an independent ideal elastic model of the firmware's own assumptions.

With both firmware errors removed and a converged solver, the full logo reaches **0.07 / 0.13 / 0.22 mm**, mean offset (0.02, 0.00, 0.02) mm. That is as good as flex-off and on par with HP3 (0.10 / 0.15 / 0.23).

## Method

- **Logo replay**: [`scratchbook/logo_qc.mjs`](scratchbook/logo_qc.mjs) replays a CAN file through the production headless world. It uses the Rust native engine, dt 2 ms, 1 substep, closed loop, unlayered bake and the `config_hp4.g` axis map, and feeds every extrusion event into the production `QualityMonitor` with the logo G-code as reference, exactly like the browser. The bundled HP4 replay reproduces the screenshot to the digit (row 1 below). It takes 81 s of wall time per full 2076 s logo.
- **CAN variants**: [`scratchbook/gen.sh`](scratchbook/gen.sh) runs the shared `RRF/build/rrf_simulator` on a private VSD copy ([`scratchbook/vsd`](scratchbook/vsd)) and then `csv2can.py`. All variant G-code and configs are in that VSD:
  - `fix`: `config_hp4_fix.g` sets `M666 N4:4:4:12`. `linesPerAxis` appears only in the flex divisor, so N·MA emulates `MA²·N` exactly without a firmware rebuild.
  - `noflex`: the logo with `M666 F0`.
  - `z0`: the logo with `G1 Z1` and `G1 Z0` after `M666 F1`. A non-null move applies the tension before the standstill FK.
- **Firmware instrumentation**: a private build in the scratchpad with an env-guarded print in `CartesianToMotorSteps` and `MotorStepsToCartesian` ([patch](scratchbook/hpdbg_instrumentation.patch)). Outputs: [base 60-line prefix](scratchbook/hpdbg_base_prefix.txt.gz) and [fix + z0 prefix](scratchbook/hpdbg_fix_z1z0_prefix.txt.gz).
- **Static flex test**: [`scratchbook/static_flex.mjs`](scratchbook/static_flex.mjs) holds the HP4 at home, ramps given firmware `linePos` values onto the spools (θ = lp·MA/r), holds 4 s and reports the displacement and tensions. [`scratchbook/ideal_static.py`](scratchbook/ideal_static.py) solves the same problem for the firmware's own model: point effector, N·MA parallel segments, elastic length MA·d+Y, EA = 20000, unilateral lines, gravity.

## Results

### Full logo (closed loop, unlayered, 1 substep, native)

| # | Scene | CAN stream | Path RMS / 95% / max (mm) | h / v RMS | Mean offset X/Y/Z (mm) | within 0.25 mm |
|---|---|---|---|---|---|---|
| 1 | hp4 (default, 2 iter) | bundled | 0.39 / 0.62 / 0.69 | 0.36 / 0.15 | 0.16, −0.42, 0.14 | 30 % |
| 2 | hp4 | regenerated base | identical to 1 | | | |
| 3 | hp4 | fix (MA²) | 0.20 / 0.36 / 0.43 | 0.19 / 0.07 | 0.09, −0.21, 0.06 | 77 % |
| 4 | hp4 | base + z0 | 0.36 / 0.60 / 0.67 | 0.36 / 0.05 | 0.16, −0.42, −0.01 | 39 % |
| 5 | hp4 | fix + z0 | 0.22 / 0.37 / 0.43 | 0.20 / 0.09 | 0.09, −0.22, −0.09 | 73 % |
| 6 | hp4 | noflex | 0.08 / 0.16 / 0.26 | 0.08 / 0.02 | 0.03, 0.00, −0.02 | 100 % |
| 7 | **hp4, 20 iter** | bundled | 0.35 / 0.38 / 0.41 | 0.07 / 0.34 | 0.02, 0.00, **0.34** | 0 % |
| 8 | hp4, 20 iter | fix | 0.17 / 0.21 / 0.27 | 0.07 / 0.16 | 0.02, 0.00, 0.16 | 100 % |
| 9 | hp4, 20 iter | base + z0 | 0.20 / 0.24 / 0.28 | 0.07 / 0.19 | 0.02, 0.00, 0.19 | 100 % |
| 10 | **hp4, 20 iter** | **fix + z0** | **0.07 / 0.13 / 0.22** | 0.07 / 0.02 | 0.02, 0.00, 0.02 | 100 % |
| 11 | hp4, 20 iter | noflex | 0.08 / 0.16 / 0.24 | 0.08 / 0.01 | 0.03, 0.00, −0.01 | 100 % |
| — | hp3 (default) | bundled | 0.10 / 0.15 / 0.23 | 0.09 / 0.05 | −0.05, 0.10, 0.04 | 100 % |

With a converged solver (rows 7–10), both firmware errors appear purely as Z offset. The FK belief contributes +0.157 to +0.163 mm (row 8). MA over-compensation contributes +0.19 mm (row 9); the ideal model predicts +0.179 mm. The two add up to row 7's +0.34 mm. Logs with full QC metrics are in [`scratchbook/runs/*.log`](scratchbook/runs). The subsampled pose, servo, commanded-angle and tension traces (`runs/*.json`, 118 MB) and the CAN variants (13 MB) are not committed. They are regenerated by the Reproduce commands below.

### Static flex at home (closed loop, 4 s hold)

| Applied `linePos` (mm) | Sim, 2 iter (default) | Sim, 20 iter | Ideal firmware model |
|---|---|---|---|
| A −1 | 0.80 mm @ −86.7°, T_A 1.9 N | 0.674 mm @ −90.0°, 2.88 N | 0.674 mm @ −90.0°, 3.00 N |
| B −1 | 0.66 mm @ 18.7° | 0.674 mm @ 30.0° | 0.674 mm @ 30.0° |
| fix flex (−0.566 ABC, −0.083 D) | (0.079, −0.214, −0.074) mm, T 3.3 / 0.55 N | (0, 0, 0.010) mm, T 5.00 / 0.795 N | (0, 0, 0.008) mm, T 5.19 / 0.823 N |
| base flex (−1.132, −0.333) | (0.160, −0.436, 0.014) mm | — | (0, 0, 0.179) mm, T 10.6 / 1.61 N |

The planned forces are F_ABC = 20.7 N and F_D = 10 N from the instrumented firmware, i.e. 5.18 / 0.83 N per line. With the MA² flex, the ideal model returns exactly those tensions with 0.008 mm displacement, which confirms the derivation. The default sim reproduces the logo's XY offsets statically (row 1 vs base flex, row 5 vs fix flex), so they are not dynamic.

Residual XY offset for the fix flex vs convergence: 0.228 mm at 2 iterations, then 0.117 / 0.080 / 0.029 / 0.004 mm at 3 / 4 / 6 / 10 iterations. At 2 / 4 / 8 substeps it is 0.045 / 0.006 / 0.001 mm. Full-logo wall time at 20 iterations was 212 s vs 81 s.

## Failed or discarded trials

- **Wheel friction** (scene with `WheelMaterial` μ = 0): results are bit-identical to the default. Discarded.
- **Effector tilt and servo lag**: tilt shifts the tip by about 0.08 mm and p99 servo error is about 0.04 rad, the same with and without flex. Not the flex-proportional cause.
- **Planned F recovered from CAN differences** ([`planned_vs_real.py`](scratchbook/planned_vs_real.py)): wrong, because the CAN difference also contains the FK Z shift. It gave F_D 15 N vs 28 N, depending on `N`. The instrumented firmware replaced it.
- **"First CAN message" as a flex probe**: it is only the start of a move. Discarded in favor of end-of-stream totals and the instrumented build.
- `G1 X0 Y0 Z0` as a tensioning move: dropped by RRF as zero-length, so the Z belief still came out at 0.157 mm.
- The RRF host simulator silently fails to open files whose virtual path is longer than about 120 characters (`MaxFilenameLength`). The VSD copy therefore lives in the scratchpad with short file names.
- Regenerated base CAN is not byte-identical to the bundled file, but its replay QC is identical.

## Proposed next steps (not done)

1. **Firmware (RRF and Klipper)**: change the divisor to `MA²·N`: `flex[i] = −F[i] / (springKs[i] · MA · MA · linesPerAxis[i])` in `HangprinterKinematics.cpp:471` and `kin_winch.c:617`. After that the `N4:4:4:12` emulation is no longer needed. Then regenerate the HP4 (and cubecorners/skycam, if MA > 1) CAN files and recheck the open-loop physics gates. Pretension halves on A/B/C and quarters on D, which also relieves the servo saturation seen on 2026-11-11.
2. **Firmware**: make enabling or changing flex not corrupt the position belief. Options: have `M666 F` emit a real tensioning move to the current position, or make the standstill FK skip flex until lines are tensioned. The interim G-code workaround is `G1 Z1` + `G1 Z0` after `M666 F1`. A smaller related point: the standstill FK tolerance (1e-3) leaves about 0.02–0.03 mm belief error after each standstill (seen at the end of the prefix run).
3. **Simulator**: raise HP4 `cablePath:solverIterations` to about 6–10, or use 4 substeps. Check cost and the stability gates (`npm run test:physics`) first. Until then, the default HP4 sim under-reports pretension by about 36 % and adds a spurious XY offset of about 0.2 mm per 0.5 mm of flex.
4. **QC**: report the Z offset separately in the HUD. Rows 7–9 have a perfect XY shape but 0 % within 0.25 mm, purely due to Z.

## Reproduce

```bash
cd research/output/session-2026-11-12-interactive-research-04b803d9/scratchbook
# gen.sh expects the VSD copy at a short path (RRF's 120-char file name limit): cp -r vsd <short dir>/v, rename
# gcodes/Hangprinter_logo6.gcode to logo.gcode and logo_z0.gcode to logoz.gcode, and point gen.sh's --vsd at it.
(cd ../../../.. && ./research/output/session-2026-11-12-interactive-research-04b803d9/scratchbook/gen.sh hp4_fix_z0 logoz.gcode config_hp4_fix.g)
SCENE=$PWD/scenes/hp4_iter20.usda CAN=research/output/session-2026-11-12-interactive-research-04b803d9/scratchbook/can/hp4_fix_z0.can node logo_qc.mjs it20_fix_z0 hp4 1
node static_flex.mjs -0.56587 -0.56587 -0.56587 -0.08332     # SCENE=..., SUBSTEPS=..., GEO=1 optional
../../../../.venv/bin/python ideal_static.py
../../../../.venv/bin/python summ.py runs/*.log
```
