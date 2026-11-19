# Discover new, reproducible cases where hp-sim5 behaves incorrectly (2026-11-19)

Goal: find **new** cases where hp-sim5 returns a wrong result. For each genuine case:
the smallest one-command reproducer and quantitative evidence (expected vs actual,
error size, tolerance, revision). Negative results are preserved. Nothing is fixed.

## Scope exclusions

- Do not fix anything. No source edits; researchers write only inside their session dir.
- Encoder autocalibration is owned by another program; do not investigate it.
- Do not re-report prior findings (see "Already found").
- No rebuilds, no full simulations, no browser, no Vite, no full test suite.
- **Host is small: 3.8 GiB RAM, 2 GiB swap (already partly used), 2 cores.**
  Read files first. Micro-experiments only: target < 30 s, < 300 MB RSS, one process at
  a time, always wrapped in `timeout`. Check `free -h` before/after. Prefer pure JS
  with stubbed globals over physics worlds.

## Already found (do not re-report; a contradiction of a prior claim is fair game)

- 2026-11-16: hp5/hp3 layered bake slack; stale "Rebase reference" surviving Reset;
  layering toggle during Finish ASAP skipping rebake; native/Wasm friction divergence;
  unbounded encoder phase unwrap.
- 2026-11-17: `xformOpOrder` ignored; `metersPerUnit` ignored; missing spool attribute
  leaves dangling joints; forward-transform noise units; RRF standstill off-by-one;
  multi-driver CAN unrepresentable; Klipper MCU player drops first queue_step batch;
  raw Klipper E->D axis mapping; NaN poisoning; layering `split()` NaN radius; scene-plan
  ambient layering flag; `rotateAroundAxis` zero axis; `normalize()` overflow; layered
  preset radius scale (0.47-0.85 %); CubeCorners tip rotation. Strong negatives:
  physics closed forms, lifecycle/determinism, Wasm/native parity, firmware planning,
  catalog direction/sign.
- 2026-11-18: Python/native length telemetry aliases `commanded` to `actual`; G-code
  upload **extrusion** accounting (relative-E default, ignores `M83`, `G92 E`, keeps
  `last_e` across jobs); native recordings pose every member at its parent body;
  in-place Reset is not a new replay generation; quality history entry taken before the
  command queue drains; `fastModeActive` outlives its worker; stopped flight recorder
  disables the bridge; any non-motion payload un-pauses; two-client encoder stickiness;
  passive-wheel authored inertia ignored; no gyroscopic term. Negatives: recorder
  round-trip determinism, preset/RRF extrusion conservation, substep semantics,
  pause semantics, integration order, solver-iteration invariance.

## Current understanding (2026-11-19)

Manager confirmed one lead before commissioning (pure JS, stubbed `postMessage`):
`moveCommander.js` parses only `G1`, `G6`, `G92` and `M82/M83`. It never implements
`G0` or positioning modes. Two-command probe `M82 / G91 / G1 X10 E1 / G1 X10 E2`:
the app's own reference parser (`parseGcodeText`) travels **20.0 mm** and ends at
X=20; the simulation emits moves totalling **10.0 mm** and ends at X=10 (relative
moves treated as absolute). `G0 X10 E1 / G1 X20 E2` is silently merged into one
20 mm move. Both firmwares and the app's reference parser treat these as valid.
This is being investigated as Investigation A.

Investigation B tests whether the advertised quality report / reference overlay
computes correct, closed-form-checkable numbers on synthetic paths.

Investigation C (started after A finished) compares the two shipped playback paths
for the same file: the compiled RRF `.can` plans behind the preset buttons vs the
Upload path, HP4 non-layered as the control, HP4 layered and the shipped `G6`
`tighten.gcode`, attributing any disagreement to a code path.

### Confirmed 2026-11-19, Investigation A (manager re-ran all 8 repros, all exit 1)

The Upload path pattern-matches one literal token per line (`G1/G6/G92/M82/M83`,
`moveCommander.js:209-220`); everything else is silently `continue`d. Eight divergences
on valid G-code, six wrong motion, two wrong time; three contradict the app's own
reference overlay:

1. **`G91` ignored** (worst): travel 20 → 10 mm (−50 %), end X 20 → 10, time 2.00 →
   1.00 s; second 1 mm of filament deposited as a stationary blob. Overlay draws 20 mm.
2. **`G0` dropped**: two-leg print travel 40 → 28.28 mm (−29.3 %), path off 5.0 mm,
   time +32.7 %; `F` on a `G0` line lost → 9× slow. Overlay draws the rapids.
3. **`G01`/`G00` dropped**: travel 20 → 0 mm, extrusion 2 → 0 mm. Overlay draws 20 mm.
4. **`N`/`*` framing dropped**: 20 → 0 mm (valid RRF line-numbered file).
5. **`G20` inches**: travel 38.1 → 1.5 mm (−96.1 %), extrusion density 25.4× high.
6. **`G2/G3` arcs**: travel 25.71 → 10 mm (−61.1 %), extrusion −50 %; mid-file arc
   9.99 mm path error. Overlay agrees with the sim (both wrong).
7. `G4` dwell ignored (timing only, 100 % of dwell missing).
8. Default feed rate is 1000 mm/min (undocumented); 3.0× RRF / 1.5× Klipper timing.

Shipped presets contain none of these features; the exposure is the **Upload File**
feature on ordinary slicer output (G0/G91 are ubiquitous). Negatives (11/11): CRLF,
lowercase, tabs, both comment styles, `F`-only lines, checksum without `N`, explicit
`G21`/`G90`.

### Confirmed 2026-11-19, Investigation B (manager re-ran all 3 repros, all exit 1)

1. **Corner overshoot/settling report the print's global offset as a corner defect.**
   Perfect L (50+50 mm, 90° corner) shifted rigidly by 1.000 mm along the first move:
   `cornerOvershootAvg/Max = 1.00/1.00 mm` and `settleDistanceMax = 0.10 mm`, while the
   same card prints `Mean offset 1.00 mm` and the module's own comment says corners are
   judged "after removing the mean offset" (`quality-monitor.js:128-131`). A
   perpendicular 1 mm offset on the same print reports 0.00/0.00 and scores 97 vs 90
   (7.36 pts for the same shape error). Cause: `_updateOffsetEstimate()` (:646) runs
   before the current sample's normal equations (:659-667), so the first sample of a new
   direction is scored un-offset; overshoot is latched at :772-773. The value flips
   0 ↔ full offset with float rounding of E (9 of 15 step counts fire). The shipped
   preset's first corner does not fire (0.0016 mm), which is why the existing test misses
   it.
2. **"Path within 0.50 mm" is a share of samples (≈ printing time), not of path length.**
   10 mm at 5 mm/s on-path + 90 mm at 50 mm/s 1 mm off: HUD 52.6 %, path share 10.0 %
   (42.6 pp). Label at :934, value `withinToleranceCount/sampleCount` (:674-679), one
   sample per 5 ms step.
3. **Quality 100 / "no deductions" on a print that completed 1 of 3 legs and 0 of 3
   corners** (progress 33.3 %): `cornerError = 0` when `cornerCount == 0` (:859-860) and
   no progress term. Misleading rather than closed-form wrong; ranked third.

Strong negatives: rms and max bit-exact; p95 = upper edge of the 2 % log bin, worst
+1.973 %, never low; zero-distance samples → 0, no NaN; 0.2 mm bevel corner deviation
exact (0.141421356 mm); 45° threshold exact; pure lag stays at contour 0.0001 mm;
end-to-end `parseGcodeText` → monitor reports a 0.2 mm offset as 0.200000 mm and the
score recomputed from the reported metrics matches exactly.

### Confirmed 2026-11-19, Investigation C (manager verified the mechanism and repro)

1. **`G6` moves the spools and then cancels itself.** `tighten.gcode`
   (`G6 A-42 B-42 C-42 F5000`) on `config_hp4.g`: 175 `Move` commands ramp A/B/C to
   −160.428°, then one `Add to reference` carries the same −2.8 rad
   (`moveCommander.js:409-411`). The world target is `commandedAngle − deltaAngle`
   (`rust_physics_engine.js:115` → `engine.rs:1568`), so the net spool rotation is
   **0.000000° instead of −160.428°** — 0 of 42 mm of tightening (100 % lost) after a
   full 42 mm out-and-back excursion. Confirmed in the shipped headless HP4 world
   (net 0.00000°, peak −160.428°). Layered machine: net 0, peak −103.27°. Upload-only:
   no button/catalog entry references `tighten.gcode`.
2. **`G6` line→angle conversion ignores buildup (`Q`) and `O`.** For the layered config
   it over-rotates by 0.628° = 0.255 mm of line (0.61 %). Masked by finding 1.
3. Secondary/adjacent: `G92 X5` emits only `Add to reference`; the world target jumps
   (B line 4.282 mm for `G92 X5`, 12.839 mm from X=10) where G92 must not move. The
   frame is correct again after the next `Move`. Shipped files only use `G92 E0`
   (E-only, no motion). Related to, but distinct from, the 2026-11-16 stale-rebase leak.

Strong negatives: HP4 (Q=0, R=30) preset vs upload agrees — line payout within 0.023 %,
path travel −0.0143 %, toolpath max 0.470 mm / rms 0.216 mm over a 17.7 m path. The
residual is RRF flex compensation (`M666 F1`, `U=2:2:2:4`, `N=2:2:2:3`; ratio
6.26:6.80:6.64:1 vs predicted 6:6:6:1), a pose-smooth non-rigid shortening absent from
the upload path because `moveCommander.js:39` hardcodes `use_flex=false`; it is not a
defect. Layered pair agrees better (0.015 % payout, 0.255 mm max). RRF has no `G6`, so
no second path exists to compare.

## Open questions and alternatives

1. (A) How far does the upload path's command-semantics gap extend: `G90/G91`, `G0`,
   `G20/G21` units, `G2/G3` arcs, `G4` dwell, modal `F` lost on ignored lines, line
   numbers/checksums, multiple commands per line? Which produce silently wrong motion
   on *valid* G-code, and how large is the error?
2. (B) Does the quality report agree with ground truth: histogram quantile/bin edges,
   NaN/zero-distance samples, corner thresholds, extrusion weighting, score formula?
3. Unexplored if A/B dry up: cross-path consistency (preset RRF/klipper playback vs
   upload path for the same file); spool/line bookkeeping (`hangprinter_spools.js`,
   `Q` buildup in `kinematics.js` vs the Rust spool model); `qualityController` job
   plumbing; UI inspection tools.

## Next decision

Program complete for this question. Three investigations (A: Upload-path G-code
semantics, B: quality report, C: cross-path playback) found 14 reproducible
divergences, all re-run by the manager from a clean shell; a fresh reviewer confirmed
every headline and its corrections are applied. Summary: `summary.md`. Evidence is
uncommitted under this program directory (delivery is the human's call).

If continued, the highest-value follow-ups are (a) an end-to-end browser run of
`G91`/`G0`/`G6` uploads to confirm the user-visible symptom, (b) the same-file
preset-vs-upload comparison on the remaining catalogued machines, and (c) deciding
whether `G6`'s trailing `Add to reference` or its unit test is the intended contract.
These are proposals only; no code changes were made.
