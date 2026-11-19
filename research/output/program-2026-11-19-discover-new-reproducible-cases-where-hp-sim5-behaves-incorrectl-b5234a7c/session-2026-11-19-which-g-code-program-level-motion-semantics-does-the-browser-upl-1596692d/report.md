# Which G-code program-level motion semantics does the browser Upload path get wrong?

Investigation: `session-2026-11-19-which-g-code-program-level-motion-semantics-does-the-browser-upl-1596692d`
Program: *Discover new reproducible cases where hp-sim5 behaves incorrectly* (2026-11-19)
Revision under test: `56a66f77`. Environment: Linux x64, Node 26.13.0, no build, no browser, no physics runtime.
Sources pinned by real 64-hex SHA-256 digests in `measurements.json` (`sources.sha256`: one
`{path, sha256}` entry each for `moveCommander.js`, `inspectionToolsController.js`, `remoteSpoolSystem.js`).

**Verdict.** The `.gcode`/`.gc` Upload path does not interpret G-code; it pattern-matches one token per line
against the literal strings `G1`, `G6`, `G92`, `M82`, `M83`
(`hp-sim-3d/app/workers/moveCommander.js:209-220`). Eight program-level features of valid, documented G-code
therefore diverge from the machine, six of them with wrong motion and two with wrong time. All eight fail
**silently**: unrecognised lines fall through `if (!command) continue;` and the only user feedback is
`console.warn` for *file formats*, never for G-code content.

Three of them contradict the app's own reference overlay (`parseGcodeText`, which is drawn from the same
uploaded file), so the discrepancy is visible inside the product: the blue reference path and the simulated
effector disagree.

| # | Feature | Smallest reproducer | Commanded → simulated | Classification |
|---|---|---|---|---|
| F1 | `G91` relative positioning | `M82`/`G91`/`G1 X10 E1 F600`/`G1 X10 E2 F600` | travel 20.0 → **10.0 mm (−50 %)**, end X 20.0 → **10.0**, time 2.00 → **1.00 s** | (a) wrong motion, overlay contradicts sim |
| F2 | `G0` rapid | 4-line two-leg print | travel 40.0 → **28.28 mm (−29.3 %)**, path off by **5.0 mm**, time 2.133 → **2.832 s (+32.7 %)** | (a) wrong motion, overlay contradicts sim |
| F2b | `F` word on a `G0` line | `G0 F9000` + `G1 X10 E1` | time 0.0667 → **0.600 s (9.0×)** | (a) wrong time |
| F3 | `G01`/`G00` numeric spelling | `G01 X10 E1 F600`/`G01 X20 E2 F600` | travel 20.0 → **0.0 mm (−100 %)**, extrusion 2.0 → **0.0 mm** | (a) wrong motion vs RRF, overlay contradicts sim; Klipper also does not move but warns |
| F4 | `N` line number + `*` checksum | `N10 G1 X10 E1 F600*45`/`N20 …*46` | travel 20.0 → **0.0 mm (−100 %)** | (a) wrong motion — **rare** (slicers do not emit line numbers) |
| F5 | `G20` inches | `G20`/`G1 X1 E1`/`G1 X1.5 E2` | travel 38.1 → **1.5 mm (−96.1 %)**, end X 38.1 → **1.5** | (a) wrong motion |
| F6 | `G2`/`G3` arcs | `G1 X10 E1` + `G2 X20 Y10 I10 J0 E2` | travel 25.71 → **10.0 mm (−61.1 %)**, extrusion 2.0 → **1.0 mm (−50 %)** | (a) wrong motion |
| F7 | `G4` dwell | `G4 P1000` between two moves | time 3.00 → **2.00 s**; 100 % of the dwell missing | (b) timing only — **latent** |
| F8 | initial modal feed rate | two `G1` moves, no `F` anywhere | 0.40 s (RRF 3000 mm/min) / 0.80 s (Klipper 1500) → **1.20 s (3.0× / 1.5×)** | (b) timing only — **latent** (real files set `F` early) |

“Commanded” is RRF/Klipper semantics, computed by a small independent interpreter in `harness.mjs` and
cross-checked against the app's own reference parser where the reference parser is correct. “Simulated” is
reconstructed from the shipped worker's own command stream: every emitted command is one physics step
(`hp-sim-3d/app/remoteSpoolSystem.js:439-449`), and its motor angles are inverted through the worker's own
kinematics, so the trajectory, travel, final position and time are what the machine in the app actually does.

Every reported finding uses valid, documented input. The only invalid input tested is the negative control
“two commands on one line”, which is **not** reported as a finding (RRF, Klipper and both app parsers all
execute only the first token there).

---

## 1. F1 — `G91` relative positioning is silently ignored  *(worst case)*

Smallest reproducer (`repro_g91.mjs`):

```
M82
G91
G1 X10 Y0 E1 F600
G1 X10 Y0 E2 F600
```

| quantity | commanded (RRF/Klipper) | simulated | error |
|---|---|---|---|
| travel | 20.000 mm | 10.000 mm | **−50.0 %** |
| final effector | X = 20.000 | X = 10.000 | **−10.000 mm** |
| extrusion | 2.000 mm | 2.000 mm | 0 % (but see below) |
| duration | 2.000 s | 1.002 s | **−49.9 %** |
| physics steps at dt = 2 ms | 1000 | 501 | — |
| reference overlay | 2 segments, (0,0)→(10,0)→(20,0), 20.000 mm | — | overlay contradicts sim |

Because X words stay absolute, the second `G1 X10` targets the position the effector is already at:
distance 0, so the code takes the `num_steps === 0` branch
(`moveCommander.js:308-328`) and emits **one stationary `Move` carrying 1.0 mm of extrusion**. The second
half of the commanded line is therefore deposited as a blob at X = 10 instead of along a 10 mm line, and the
print stops 10 mm short while the blue reference path reaches X = 20.

Code path: `moveCommander.js:209-220` (no `G91` case) → `:229-234` (X/Y/Z always assigned absolutely) →
`:330` (belief advanced as if absolute). The reference parser *does* implement it:
`inspectionToolsController.js:96-103`, `:142-143`.

Firmware: `RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:573-576` (`case 91: axesRelative = true`),
`klipper/klippy/extras/gcode_move.py:174-176`. Both firmwares agree.

Scaling: a relative-mode square (`G1 X10`/`Y10`/`X-10`/`Y-10`) travels **52.43 mm instead of 40.00 mm
(+31.1 %)**, ends at **Y = −10 instead of (0,0)** — 10 mm off and outside the intended square — with a
**10.0 mm maximum path deviation** and 31 % too much simulated time. Relative mode is not “ignored”, it
silently rewrites the whole program.

Classification: **(a)** shipped, user-visible: the same upload produces a 20 mm reference path and a 10 mm
simulated move.

## 2. F2 — `G0` rapid lines are dropped (and the feed rate on them is lost)

Smallest realistic reproducer (`repro_g0.mjs`) — rapid out, print a leg, rapid over, print a leg:

```
M82
G0 X10 Y0 F9000
G1 X10 Y10 E1 F600
G0 X20 Y10 F9000
G1 X20 Y20 E2 F600
```

| quantity | commanded | simulated | error |
|---|---|---|---|
| travel | 40.000 mm | 28.284 mm | **−29.3 %** |
| duration | 2.1333 s | 2.8320 s | **+32.7 %** |
| max deviation from commanded path | 0 mm | **5.000 mm** | — |
| final effector | (20, 20) | (20, 20) | 0 (coincidence) |
| extrusion | 2.000 mm | 2.000 mm | 0 % total, **density −29.3 %** (0.0707 vs 0.1000 mm/mm) |
| reference overlay | two **vertical** legs (10,0)→(10,10), (20,10)→(20,20) | two **diagonals** (0,0)→(10,10)→(20,20) | overlay contradicts sim |

Each `G0` line is skipped, so the following `G1` starts from the pre-rapid point and cuts the diagonal.
The extrusion total happens to be conserved, but both 1 mm segments are laid along the diagonals instead of
the commanded legs: the simulated print is a rotated/inflated version of the reference path.

Code path: `moveCommander.js:214` (only `code === 'G1'` becomes a move) → `:222-224` (line dropped) → the
position is never advanced (`:330`). Reference parser: `inspectionToolsController.js:124`
(`codeNumber === 0 || codeNumber === 1`), so the overlay draws `G0` exactly like `G1`.

Firmware: `RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:230-233` (`case 0: // Rapid move` → `DoStraightMove`),
`klipper/klippy/extras/gcode_move.py:37` (`gcode.register_command('G0', self.cmd_G1)`).

**F2b — the modal feed rate on a `G0` line is lost too.** `M82 / G0 F9000 / G1 X10 Y0 E1` runs
**0.600 s instead of 0.0667 s (9.0× slower)**: `F` is only read inside `_parse_g1_command`
(`moveCommander.js:124-128`), which is never called for a `G0` line. RRF applies `F` to both move types
(`GCodes.cpp:1902-1932`), as does Klipper (`gcode_move.py:147-153` on the shared `cmd_G1` handler for `G0`).

Classification: **(a)** shipped, user-visible (overlay path ≠ simulated path), plus a timing error.

## 3. F3 — numeric spellings `G00`/`G01`/`G02`/`G03` are dropped

Smallest reproducer (`repro_g01.mjs`):

```
M82
G01 X10 Y0 E1 F600
G01 X20 Y0 E2 F600
```

Commanded 20.000 mm / 2.000 mm extrusion / 2.000 s; simulated **0.000 mm / 0.000 mm / 0.000 s**, effector
never leaves (0,0). The reference overlay for the same file shows both 10 mm segments. This is a 100 % loss of
motion and extrusion.

Code path: `moveCommander.js:209` uppercases the first token and `:214`/`:216`/`:218` compare it to the
*strings* `'G1'`/`'G6'`/`'G92'`. `G01` never matches. The reference parser parses the command number with
`Number.parseInt` (`inspectionToolsController.js:84-86`), so `G01 == 1` there.

Firmware: RRF parses the command number numerically
(`RRF/ReprapFirmware/src/GCodes/GCodeBuffer/StringParser.cpp:985-1032`), so `G01` is `G1` and **RRF executes
the move**; Marlin/RS-274 files routinely spell moves `G01`. Klipper's handler table is keyed on `G1`, so
Klipper also executes nothing — but it says so: `Unknown command:"G01"`
(`klipper/klippy/gcode.py:272-306`). So the finding is not "both firmwares move and hp-sim does not"; it is
that hp-sim **accepts the upload, drops the lines without any message, and reports the print as finished**,
which is the worst of the three behaviours and the only silent one. The lost-motion numbers are therefore
against RRF, with Klipper as a second non-executing reference.

Sub-case (`repro_g01.mjs`): `G00 X10 Y0 F9000` followed by `G1 X10 Y10 E1` gives 20.000 mm commanded (RRF),
**14.142 mm simulated (−29.3 %)**, 5 mm off the commanded path — same mechanism as F2.

Classification: **(a)** shipped, user-visible.

## 4. F4 — lines carrying `N<line number>` / `*<checksum>` are dropped wholesale

Smallest reproducer (`repro_line_numbers.mjs`; checksums are computed by the script, both lines are valid
RS-274):

```
M82
N10 G1 X10 Y0 E1 F600*45
N20 G1 X20 Y0 E2 F600*46
```

Commanded 20.000 mm / 2.000 mm / 2.000 s; simulated **0 / 0 / 0** and the effector stays at (0,0). The line
number alone is enough (the second case in the script shows the same 0 mm with no checksums), and the
reference overlay is empty too, so the app shows a blank reference for a file that both firmwares execute.

Code path: `moveCommander.js:209-224` treats `N10` as the command word. Reference parser does the same
(`inspectionToolsController.js:84-124`).

Firmware: `RRF/.../GCodeBuffer/StringParser.cpp:123-141` (state `parsingLineNumber` after `N`), and
`StringParser.cpp:186-192` / `272-285` (checksum after `*`); `klipper/klippy/gcode.py:199-205`
(`if ''.join(parts[:2]) == 'N': cmd = ''.join(parts[3:5])`).

Classification: **(a)** wrong motion (100 % loss), but low likelihood for uploaded *files*: slicers do not
emit line numbers, and both firmwares only need them for serial streaming. When it happens, the app is
completely silent.

## 5. F5 — `G20` inches are interpreted as millimetres

Smallest reproducer (`repro_g20.mjs`):

```
M82
G20
G1 X1 Y0 E1 F600
G1 X1.5 Y0 E2 F600
```

| quantity | commanded (RRF) | simulated | error |
|---|---|---|---|
| travel | 38.100 mm | 1.500 mm | **−96.1 %** |
| final effector | X = 38.100 | X = 1.500 | **−36.6 mm** |
| duration | 0.150 s | 0.150 s | 0 % (F is scaled by the unit in both, so the time is unit-invariant) |
| extrusion | 50.800 mm | 2.000 mm | **−96.1 %** (E is a distance: RRF `GCodes.cpp:1985` converts it with the same 25.4 factor) |

Code path: `moveCommander.js:222-224` (G20 skipped); coordinates are taken as mm (`:229-234`); the reference
parser has no `G20` case either (`inspectionToolsController.js:88-126`), so overlay and simulation agree with
each other and both are 25.4× small. Because E is a distance as well, the extrusion scales down by the same
factor, so the extrusion *density* is unchanged (50.8/38.1 = 2.0/1.5 = 1.333 mm/mm): this is a uniform 25.4×
shrink of the whole program, not a density error.

Firmware: `RRF/.../GCodes/GCodes2.cpp:376-380` (`case 20: // Inches` → `UseInches(true)`),
`GCodeBuffer.cpp:1013-1016` (× `InchToMm = 25.4`, `RepRapFirmware.h:632`), `ConvertSpeed` at
`GCodeBuffer.cpp:1025-1027`, and the E word at `GCodes.cpp:1985` (`gb.ConvertDistance(...)`). Klipper instead raises
`Machine does not support G20 (inches) command` (`klipper/klippy/extras/gcode_move.py:159-161`): the user is
told. For a Hangprinter this is a low-likelihood input, but it is silently wrong rather than refused.

Classification: **(a)** wrong motion, low likelihood.

## 6. F6 — `G2`/`G3` arc moves are dropped, losing travel and extrusion

Smallest reproducer (`repro_arcs.mjs`):

```
M82
G1 X10 Y0 E1 F600
G2 X20 Y10 I10 J0 E2 F600
```

| quantity | commanded | simulated | error |
|---|---|---|---|
| travel | 25.706 mm (10 + quarter arc r=10) | 10.000 mm | **−61.1 %** |
| extrusion | 2.000 mm | 1.000 mm | **−50.0 %** |
| final effector | (20, 10) | (10, 0) | 14.142 mm short |
| duration | 2.571 s | 1.000 s | −61.1 % |

The arc's `E2` never reaches `last_e` (`moveCommander.js:236-240`), so a trailing arc loses its extrusion
completely. With a *mid-file* arc (`G3 X20 Y0 I10 J0 E2` + `G1 X20 Y20 E3`) the accumulated 3 mm is dumped
on the following 28.28 mm diagonal instead of 2 mm along a 31.42 mm half circle plus 1 mm along a 20 mm line:
**51.413 mm → 28.284 mm (−45.0 %)** and **9.99 mm maximum path deviation**. Extrusion totals coincide there,
so only the travel/path metrics expose it.

Code path: `moveCommander.js:214` (only `G1` is a move). Reference parser: `inspectionToolsController.js:124`
skips arcs too — the overlay *agrees* with the simulation, so there is **no in-product contradiction** here;
both disagree with the machine.

Firmware: `RRF/.../GCodes/GCodes2.cpp:251-277` (`case 2/3` → `DoArcMove`, `I/J` or `R`),
`klipper/klippy/extras/gcode_arcs.py:36-37`.

Classification: **(a)** shipped and wrong, moderate likelihood: arc-fitting (`gcode_arcs`, PrusaSlicer/Cura
arc welder, CNC/laser files) produces exactly these commands, and nothing in the UI says they were skipped.

## 7. F7 — `G4` dwell contributes no simulated time

Reproducer (`repro_dwell.mjs`):

```
M82
G1 X10 Y0 E1 F600
G4 P1000
G1 X20 Y0 E2 F600
```

Motion and extrusion are exact (20.000 mm, 2.000 mm), but duration is **2.000 s instead of 3.000 s**: the
full 1000 ms dwell is missing. Time is one timestep per consumed command
(`remoteSpoolSystem.js:439-449`), and `G4` produces no command at all, so the print clock skips it.

Firmware: `RRF/.../GCodes/GCodes.cpp:3742-3790` (`P` ms, `S` seconds, advances `simulationTime`),
`klipper/klippy/toolhead.py:554-557`.

Classification: **(b)** silent unsupported-command gap with no wrong motion (timing only). It still changes
every simulated duration, e.g. layer-cooling dwells.

## 8. F8 — the initial modal feed rate is 1000 mm/min instead of the firmware default

Reproducer (`repro_feedrate_default.mjs`): two 10 mm `G1` moves with no `F` anywhere.

| | duration |
|---|---|
| RRF default 3000 mm/min (`Config/Configuration.h:220`, `GCodeMachineState.cpp:16`) | 0.400 s |
| Klipper default 25 mm/s = 1500 mm/min (`gcode_move.py:48`, `:147-153`) | 0.800 s |
| hp-sim (`moveCommander.js:32`, `last_speed_mm_per_min = 1000.0`) | **1.200 s** |

3.0× slower than RRF, 1.50× slower than Klipper. The reference overlay carries `feedRate: null` for these
segments, i.e. it makes no claim, so there is no in-product contradiction. Nothing in `hp-sim-3d/README.md`
documents 1000 mm/min as an intentional default.

Classification: **(b)** timing only.

---

## Re-check of the manager's two confirmed leads

1. **`M82 / G91 / G1 X10 E1 / G1 X10 E2` → 10 mm vs 20 mm: confirmed exactly**
   (travel 10.000 vs 20.000 mm, end X = 10 vs 20, 501 vs 1000 physics steps at dt = 0.002 s). Added detail: the
   second 1 mm of filament is deposited at a *stationary* point, and the reference overlay draws 20 mm, so the
   product contradicts itself.
2. **`G0 X10 E1 / G1 X20 E2` “silently merged into one 20 mm move”: the merge is real, the 20 mm is not a
   divergence.** Measured: commanded travel 20.000 mm, simulated travel 20.000 mm, extrusion 2.000 mm both,
   duration 2.000 s both — in that specific file the error is invisible in every aggregate. `G0` only becomes
   measurable when (i) the next move changes direction (F2: −29.3 % travel, 5 mm path error), or (ii) the `F`
   word rides on the `G0` line (F2b: 9× timing error). The “merged” claim is confirmed structurally
   (1 move instead of 2).

## Negative results (tested, no divergence)

All measured with the shipped worker at dt = 0.002 s; the script `_negatives.mjs` prints 11/11 matches:

- Explicit `G21` (mm) and explicit `G90`: no effect, correct (absolute mode is the only mode implemented).
- CRLF line endings, missing final newline, blank lines, comment-only lines: correct.
- Lowercase commands (`m82`, `g1 x10 y0 e1 f600`): correct.
- Tab-separated words: correct.
- A `*checksum` **without** a line number (`G1 X10 Y0 E1*45 F600`): correct — `parseFloat("1*45") = 1`
  and the leftover `*45` token is ignored. Only the leading `N` word breaks parsing.
- Semicolon comments and parenthesised inline comments (`G1 X10 (prime)`): correct.
- A feedrate-only `G1 F600` line: correct (no move, feed stored modally); modal feed rate is otherwise sticky
  across moves.
- **Multiple commands on one line** (`M82 G1 X10 Y0 E1 F600`): both app parsers keep only the first command
  and silently swallow the rest. RRF and Klipper do the same thing (parameter `G1` is ignored by `M82`), so
  this is **invalid input, not a finding**. It is only dangerous because the failure is silent.
- The shipped presets (`public/gcode/draw_squares.gcode`, `Hangprinter_logo6.gcode`) contain no `G0`, `G91`,
  `G01`, `G2/G3`, `G20` or `G4` (only `G1`/`G28`/`G21`/`G90`/`M82|M83`/`G92`/`M104`/`M107`/`M666`/`M84`/`T0`),
  so they are unaffected — the bug needs a user-uploaded file.
- No G-code validation exists anywhere in the upload path (`grep` for `G91`/`G20`/`unsupported` under
  `hp-sim-3d/app` finds only file-format warnings), so “silent” means there is no warning to find.
- Assertion liveness: for every finding, a firmware-equivalent file that the shipped parser *does* handle
  passes the same assertions (`_controls.mjs`, 8/8), so the failures are caused by the feature under test and
  not by unattainable expectations.

## Method

- **Simulation side (shipped code, read-only).** `harness.mjs` imports `MoveCommander` from
  `hp-sim-3d/app/workers/moveCommander.js` with a stubbed `globalThis.self`/`postMessage`, configures three
  non-degenerate anchors (`M669 A0:-2000:1000 B1689.39746:975:1000 C-1689.39746:975:1000`, `M666 R30:30:30`,
  no spool build-up), sets `dt = 0.002 s` (from `public/usd_scenes/hp4.usda` `timeCodesPerSecond = 500`) and
  `setAsapMode(true)` so wall-clock pacing is irrelevant. The uploaded text is fed through a
  `ReadableStream` in two chunks, exactly like `File.stream()`.
- **What “simulated” means.** The worker emits `{type:'Move', A,B,C, E}` commands; `RemoteSpoolSystem`
  consumes exactly one queued command per physics step, so emitted commands × dt is the simulated duration,
  and the E values are the deposited filament. For geometry, each command's motor angles are inverted through
  the worker's own `pos_to_motor_pos_samples_deg` (Newton with numerical Jacobian; residual < 1e-6 deg,
  reconstruction of the start position reproduces 0,0,0 to ≈2e-13 mm). Travel is the polyline length through
  those per-step positions; “max path deviation” is the largest distance from any simulated point to the
  commanded polyline.
- **Commanded side.** `harness.mjs` `oracle()` is a ~200-line independent interpreter for exactly the tested
  subset (`G0/G1/G2/G3/G4`, `G20/G21`, `G90/G91`, `G92`, `M82/M83`, `N`/`*`), with a constant-feed time model
  (no acceleration) so that timing comparisons isolate semantics rather than RRF's trapezoid planner. It is
  cross-checked against `parseGcodeText` wherever the reference parser is correct (F1, F2, F3, F7, F8 all
  agree exactly), and against hand arithmetic written into each reproducer's `expectClose` call.
- **Why no physics runtime.** A private physics run would only add solver noise: the worker's command stream
  fully determines the commanded motor angles, which fully determine the effector pose through the same
  kinematics the solver uses. This investigation therefore stays text-level (and cheap: every script is one
  Node process, < 1 s, < 150 MB RSS).

## Limitations

- **Simulation evidence only.** Nothing here was checked on hardware. Every “commanded” value is the
  RRF/Klipper *source* semantics quoted by file:line, not a measured print.
- **Klipper disagreement on `G01`/`G00` and `G20`.** Klipper answers `G01` with a visible
  `Unknown command:"G01"` reply and rejects `G20` with an error (`gcode_move.py:159-161`), while RRF accepts
  both, but it warns the user in each case. F3 and F5 therefore describe divergence from RRF behaviour, and
  hp-sim is the only implementation of the three that neither moves nor tells the user: it silently produces
  no motion (F3) or a 25.4× smaller print (F5) and still reports success.
- **Acceleration is out of scope by construction.** Both sides use distance/feed. The simulated durations
  reported here (steps × dt) are the app's real durations, but they are not compared against a trapezoidal
  motion planner, so absolute timings for short segments are optimistic on both sides; the *relative* errors
  (50 %, 29 %, 100 %, 33 %, 3×) are unaffected.
- **Extrusion distribution vs extrusion total.** The app deposits E uniformly over each emitted step. F2 and
  the mid-file F6 case conserve the total E but place it on the wrong path; the reported “density” figures
  quantify that (F5 loses E as well, by the same 25.4× factor as the travel). This is not the already-known
  M82/M83/G92-E/last_e accounting defect (see
  `research/output/program-2026-11-18-...-b4ebb599/report.md`), which I did not re-test.
- **Likelihood is judgement, not measurement.** F1/F2/F3 come from real slicer/hand-written styles
  (relative mode, Cura-Marlin `G0` travel, `G01` spellings); F4/F5 are rare in files for an FFF Hangprinter;
  F6 depends on arc fitting.
- The kinematics configuration is a synthetic three-anchor machine, not HP4. The errors reported are
  geometric/path errors in mm and ratios in %, which do not depend on the anchor set; only the exact motor
  angles do.

## Reproduce

```bash
cd research/output/program-2026-11-19-discover-new-reproducible-cases-where-hp-sim5-behaves-incorrectl-b5234a7c/session-2026-11-19-which-g-code-program-level-motion-semantics-does-the-browser-upl-1596692d
node repro_g91.mjs            # exit 1: travel 10.000 vs 20.000 mm
node repro_g0.mjs             # exit 1: travel 28.284 vs 40.000 mm, 5.0 mm path deviation
node repro_g01.mjs            # exit 1: travel 0.000 vs 20.000 mm
node repro_line_numbers.mjs   # exit 1: travel 0.000 vs 20.000 mm
node repro_g20.mjs            # exit 1: travel 1.500 vs 38.100 mm
node repro_arcs.mjs           # exit 1: travel 10.000 vs 25.706 mm, extrusion 1.0 vs 2.0 mm
node repro_dwell.mjs          # exit 1: duration 2.000 vs 3.000 s
node repro_feedrate_default.mjs  # exit 1: duration 1.200 vs <=0.808 s
node make_measurements.mjs    # rewrites measurements.json
node _controls.mjs            # 8/8 controls pass (assertion liveness)
node _negatives.mjs           # 11/11 negatives match
```

Each `repro_*.mjs` exits **1 when the divergence is present** and prints the commanded and simulated travel,
extrusion, duration, final effector position, reference-overlay totals and emitted-step count. No manual
steps, no browser, no build.
