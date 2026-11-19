# New reproducible incorrect-behaviour cases in hp-sim5 (2026-11-19)

Revision under test: `56a66f77` (working tree otherwise as found; no source edits, no
commits). Node 26.13.0, Linux x64. All evidence is simulator/code-level; nothing here
is hardware-validated. Nothing was fixed.

Program: `research/output/program-2026-11-19-discover-new-reproducible-cases-where-hp-sim5-behaves-incorrectl-b5234a7c/`
Agenda and full per-investigation reports are in the same directory.

## How to reproduce

Run from any directory (each script resolves the repo root itself):

```bash
node <session-dir>/repro_g91.mjs        # exits 1 while the bug is present
```

Every `repro_*.mjs` below exits nonzero on the current checkout and prints the exact
expected and actual numbers. The one control (`repro_preset_vs_upload_hp4.mjs`) exits 0.
All runs are pure JS or a few seconds of headless world; none needs a build.

## Findings

Ranked by reachability and size of the wrong result. "Shipped" means reachable from the
app's normal UI (`Upload File`, `Print Logo`, the quality card), not a private API.

| # | Finding | Expected → actual | Class |
|---|---------|-------------------|-------|
| 1 | Upload path ignores `G91` relative positioning | travel 20.000 → **10.000 mm** (−50 %), end X 20 → 10, 1000 → 501 steps; second 1 mm of E deposited as a stationary blob | shipped |
| 2 | Upload path drops `G0` rapids | travel 40.000 → **28.284 mm** (−29.3 %), path off **5.000 mm**, 2.133 → 2.832 s; `F` on a `G0` line lost → 0.0667 → 0.600 s (9×) | shipped |
| 3 | `G92` with coordinates drives the tool instead of only shifting the frame | `G92 X5`: target jump 16.37°, B line **+4.282 mm**, must be 0; terminal `G92 X5` ends **−5.000 mm** (closed loop) / −0.606 mm (open loop); `G92 X5`+`G1 X10` ends +0.538 mm open loop (**4.46 mm persistent error**, 7/5 missed steps) vs +4.983 closed / +4.998 plain | shipped |
| 4 | `G6` line move cancels itself | `tighten.gcode` (`G6 A-42 B-42 C-42 F5000`): **0.000° net instead of −160.428°** = 0 of 42 mm tightened (100 % lost), after a full out-and-back excursion of 160.4° | upload-only (shipped command) |
| 5 | Upload path drops `G2/G3` arcs | travel 25.706 → **10.000 mm** (−61.1 %), E 2.000 → 1.000 mm; mid-file arc 51.413 → 28.284 mm, **9.993 mm** path deviation | shipped |
| 6 | Upload path ignores `G20` inches | uniform 25.4× shrink: travel 38.100 → **1.500 mm**, E 50.800 → 2.000 mm (density unchanged) | shipped, rare |
| 7 | Upload path silently accepts and drops `G01`/`G00` | travel 20.000 → **0.000 mm**, E 2.000 → 0.000 mm; RRF executes it, Klipper errors, hp-sim reports success | shipped, rare |
| 8 | Upload path drops `N`-numbered / checksummed lines | travel 20.000 → **0.000 mm** (valid RRF file) | shipped, rare |
| 9 | Quality card reports a rigid print offset as corner overshoot | perfect L shifted 1.000 mm: `cornerOvershootAvg/Max` **1.00/1.00 mm** and settling **0.10 mm**, must be 0 per the module's own "after removing the mean offset"; score 89.798 vs 97.156 for the perpendicular control | shipped |
| 10 | "Path within 0.50 mm" is a sample/time share, not a path share | 10 mm on-path at 5 mm/s + 90 mm 1 mm off at 50 mm/s: card **52.6 %**, path share **10.0 %** (42.6 pp) | shipped |
| 11 | Quality score 100 when the reference path overruns the executed path | reference 150 mm / 15 mm E vs executed 50 mm / 5 mm E → progress 33.3 %, 0/3 corners, **score 100** ("no deductions") | shipped, misleading |
| 12 | `G4` dwell not simulated | 3.000 → **2.000 s** (100 % of dwell missing) | latent (timing) |
| 13 | Undocumented 1000 mm/min default feed rate | 0.400 s (RRF 3000) / 0.800 s (Klipper 1500) → **1.200 s** | latent (timing) |
| 14 | `G6` line→angle conversion ignores `Q` buildup and `O` | layered HP4: over-rotation **0.628° = 0.255 mm** of line (**0.61 %**); exact only at Q=0 | latent (masked by #4) |

Three findings (1, 2, 5) also contradict the app's **own reference overlay** drawn from
the same uploaded file (`parseGcodeText`), so the blue reference path and the simulated
effector visibly disagree inside the product.

### Root causes in one line each

- #1/#2/#5/#6/#7/#8/#12/#13: `moveCommander.js:209-220` matches one literal token per
  line (`G1/G6/G92/M82/M83`); everything else falls through `if (!command) continue;`
  with no warning. Positioning is always absolute (`:229-234`), `F` is only read by the
  `G1` parser (`:116-130`), and the default feed is 1000 mm/min (`:32`).
- #3: `G92` emits an `Add to reference` with `FK(new) − FK(current)` (`:332-351`).
  The engine target is `commandedAngle − deltaAngle` (`rust_physics_engine.js:115` →
  `engine.rs:1568`), so the frame shift immediately moves the tool; the next `Move`
  restores the intended frame. Size is motor-mode dependent.
- #4: `G6` ramps the motors with `Move` commands and then emits the same delta as
  `Add to reference` (`:409-411`), which the engine subtracts, undoing the move.
  `tests/js/hp-sim-3d/moveCommander.test.js:82-97` asserts that trailing `Add` and never
  checks net motion, so the behaviour is tested-in.
- #9: `_updateOffsetEstimate()` (`quality-monitor.js:646`) runs before the current
  sample's normal equations (`:659-667`), so the first sample of a new direction is
  scored un-offset; the overshoot is latched at `:772-773`. The trigger is
  float-fragile: 2499 or 2501 steps/leg fire (1.000 mm), 2500 does not (0.002 mm).
- #10: the value is `withinToleranceCount/sampleCount` (`:674-679`), one sample per
  executed step; the label is at `:934`.
- #11: `cornerError = 0` when `cornerCount == 0` (`:859-860`) and there is no progress
  term; `Progress 33.3 %` is disclosed in the same card.
- #14: `moveCommander.js:353-357` uses `Δ/R · U · H/L` and omits the buildup term that
  `kinematics.js:54-70` implements.

## Strong negatives (kept on purpose)

- **Upload parser controls (8/8 pass):** CRLF / no trailing newline, lowercase, tabs,
  `;` and `(...)` comments, `F`-only `G1` lines, checksum without `N`, explicit
  `G21`/`G90`. "Two commands on one line" is invalid input for RRF, Klipper and both app
  parsers; not reported.
- **Quality statistics:** running RMS and max are bit-exact; p95 is the upper edge of a
  2 % log bin, worst +1.973 % and never low; zero-distance samples → 0 (no NaN); the
  0.2 mm bevel corner deviation is exact (0.141421356 mm); the 45° threshold is exact;
  pure lag stays at 0.0001 mm contour; end-to-end `parseGcodeText` → monitor reports a
  0.2 mm offset as 0.200000 mm and a score recomputed from the reported metrics matches.
- **Cross-path consistency (HP4 control, `repro_preset_vs_upload_hp4.mjs`, exit 0):**
  compiled RRF `.can` preset vs Upload for `draw_squares_bigger` agree to **0.0232 %**
  of line payout and **0.470 mm max / 0.216 mm rms** toolpath on a 17.7 m path. The
  residual is RRF flex compensation (`M666 F1`, `U=2:2:2:4`, `N=2:2:2:3`; measured
  per-axis ratio 6.26:6.80:6.64:1 vs predicted 6:6:6:1), not a defect; the upload path
  hardcodes `use_flex=false` (`moveCommander.js:39`). Layered pair agrees to 0.015 %
  payout / 0.255 mm max.
- Shipped presets (`Print Logo`, `Print Squares`) contain none of the affected G-code
  features; the exposure is the `Upload File` path on ordinary slicer output.

## Independent verification

A fresh reviewer re-derived the numbers, checked the harness stubs (one emitted command
is one physics step; no verdict depends on a stub), the cited `file:line`s, and the
expected values against firmware/app contracts. All headlines were confirmed. Its
corrections were applied and re-run:

- `G20` also scales `E` by inches (RRF `GCodes.cpp:1985`), so the error is a uniform
  25.4× shrink, not a density change.
- `G01/G00`: Klipper errors out; only RRF executes it. hp-sim is the only one that
  silently accepts and drops.
- Quality "5 ms steps" was wrong (HP4 tick is 2 ms); percentages are tick-invariant.
- `G92` evidence was re-based on measured end-to-end world results, and prose signs
  corrected (probe: B +4.282 mm, C −4.286 mm of line).
- `G6` is a tested-in contradiction (see above), and its command count is 437 Moves at
  HP4's 2 ms tick (175 at the 5 ms fallback); net 0 and peak −160.428° are dt-invariant.

## Limitations

- Simulation evidence only, one host, one revision, no hardware comparison.
- Upload-path motion is reconstructed from the worker's own emitted `Move` stream and
  inverted through the same worker kinematics; motor angle → physics step is 1:1
  (`remoteSpoolSystem.js:439-449`).
- No full print, no browser, no `.can` comparison for `tighten.gcode` (RRF has no `G6`,
  so no second path exists).
- Finding 4 is upload-only: `tighten.gcode` ships in `public/gcode/` but no button,
  catalog entry or preset map references it, and the README never mentions `G6`.
- Findings 12-14 are latent (timing-only or masked). Finding 11 is misleading output
  with progress disclosed, not a wrong physical simulation.
- The corner-overshoot trigger is float-fragile; it depends on the exact per-leg sample
  count (see #9).

## Artifacts

```
session-…-which-g-code-…-1596692d/   report.md, measurements.json, 8 repro_*.mjs, _controls, _negatives
session-…-does-the-app-quality-…-cf06cc78/  report.md, measurements.json, 3 repro_*.mjs
session-…-two-shipped-playback-…-4a9e9720/  report.md, measurements.json, repro_g6_net_zero.mjs,
                                           repro_g92_moves_tool.mjs, repro_preset_vs_upload_hp4.mjs,
                                           probe_world_g6.mjs, probe_g92.mjs
review-2026-11-19-independent-verification/  review.md, review_c.md
```

No product source was modified and no commit was created.
