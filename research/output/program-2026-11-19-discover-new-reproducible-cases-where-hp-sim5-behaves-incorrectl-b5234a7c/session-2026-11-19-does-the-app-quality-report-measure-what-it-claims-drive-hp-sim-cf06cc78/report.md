# Does the app quality report measure what it claims?

Investigation `session-2026-11-19-does-the-app-quality-report-measure-what-it-claims-drive-hp-sim-cf06cc78`
Program: *Discover new, reproducible cases where hp-sim5 behaves incorrectly (2026-11-19)*

Revision: `56a66f77d3bfa58e52621d06f6273d5a1f3501f5`, Node v26.13.0. No physics world,
browser, Vite or test runner was used: the shipped modules were imported directly and
driven through their public entry points. Numbers below are from
[`measure_all.mjs`](measure_all.mjs) → [`measurements.json`](measurements.json).

## Verdict

The core statistics of the 3-D quality report are **correct**: path deviation rms and
max, the log-histogram percentiles, the corner-deviation geometry, the corner-detection
threshold, the lag/position-error separation, the m/mm units and the score formula all
match closed-form or brute-force references to the stated precision (§Negatives). I
found **no arithmetic error in the rms, max, percentile, corner-deviation or score
computations**.

Three report *numbers* are nevertheless wrong or misleading on plausible input:

1. **`Corner overshoot` and `Corner settling` report the print's rigid offset as a corner
   defect** (up to the full offset; `1.00 mm` reported where the closed form is
   `0.00 mm`), and the displayed value flips between `0` and the full offset with
   floating-point rounding of the extrusion bookkeeping. The score for the same physical
   print changes by 6–7 points depending on the offset direction and on step rounding.
   *(Shipped, user-visible, exact expected value.)*
2. **`Path within 0.50 mm` is a share of extrusion *samples* (≈ printing *time*), not of
   path length**: 52.6 % reported where 10.0 % of the path is within tolerance.
   *(Shipped, user-visible; the quantity contradicts the label.)*
3. **`Quality 100` with "Score deductions: none" when the reference describes three times
   the path the machine executed** (150 mm / 15 mm E reference vs 50 mm / 5 mm E executed;
   `G0 ... E...` is accepted by the reference parser and skipped by the command layer):
   unassessed corners and unexecuted material never deduct, and the score has no progress
   term. The card does show `Progress 33.3 %`, so this is *misleading*, not a wrong
   number; it needs a reference/executed mismatch, not an ordinary abort. *(Shipped,
   user-visible.)*

All three are in the *report*, not in the executed motion: in Findings 1 and 2 the
samples lie exactly on the nominal path plus a known rigid shift; in Finding 3 the
executed samples lie exactly on their own reference segments.

## What was driven

| Module | Entry points used |
|---|---|
| `hp-sim-3d/app/quality-monitor.js` | `setReferenceSegments`, `recordExtrusion`, `runFinalCheck`, `getMetrics`, `refreshHud` (stub HUD element), `setMachineContext`, `setMotorDiagnosticsProvider` |
| `hp-sim-3d/app/inspectionToolsController.js` | `parseGcodeText` + the same `* 0.001` position conversion / unchanged mm extrusion that `convertParsedSegmentsToSimulation` applies |
| `hp-sim-3d/app/qualityController.js` | read only (creates one `QualityMonitor` per machine, forwards `setReferenceSegments`; no statistics of its own) |

Sampling convention: one extrusion sample per executed command, tip at the **start** of
the step whose E it carries. This is the app's convention, not an assumption:
`remoteSpoolSystem.update()` dequeues and applies one command *before* the move it
extrudes for (`hp-sim-3d/app/remoteSpoolSystem.js:440-449`), and the shipped test helper
says so in words (`tests/js/hp-sim-3d/qualityMonitor.test.js:56-58`: *"the tip at the
start of each 0.1 mm step together with that step's extrusion"*). The HP4 scene runs
500 time codes per second, i.e. a 2 ms step (`public/usd_scenes/hp4.usda:7`); the
worker's own pacing uses a separate 1/200 s fallback
(`hp-sim-3d/app/workers/moveCommander.js:33`). None of the percentages below depends on
the tick length — only on samples being emitted per step.

## Finding 1 — corner overshoot/ringing report the global offset as a corner defect

**Reproducer:** `timeout 30 node repro_corner_overshoot_reports_offset.mjs` (exits 1).

Input: a perfect L — 50 mm along +X then 50 mm along +Y, a single sharp 90° corner —
printed exactly on the reference path with a rigid **1.000 mm offset along +X**, sampled
every 0.1 mm.

| HUD line | expected (closed form) | reported |
|---|---|---|
| `Corner overshoot avg / max` | 0.00 / 0.00 mm | **1.00 / 1.00 mm** |
| `Corner settling avg / max` | 0.00 / 0.00 mm | 0.10 / 0.10 mm |
| `Corner deviation avg / max (1)` | 0.00 / 0.00 mm | 0.00 / 0.00 mm |
| `Mean offset X / Y / Z` | 1.00 / 0.00 / 0.00 mm | 1.00 / 0.00 / 0.00 mm |
| `Quality` | — | 90 |

Control: the *same* 1.000 mm offset perpendicular to the first move (+Y) reports
`0.00 / 0.00 mm` overshoot and `Quality 97`. Score difference for the same shape error:
**7.358 points**.

**Contract violated** (both quotes are in the module):

> "per sharp corner (**after removing the mean offset**): deviation of the attained path
> from the corner point, **the largest deviation after it**, and the distance travelled
> until the path stays within tolerance" — `quality-monitor.js:128-131`

> "Corners are judged on shape too: rounding and overshoot, **not the offset**." —
> `quality-monitor.js:681`

The mean offset *is* removed exactly (it is reported as 1.00 mm), so the expected corner
overshoot is 0 by the module's own definition. Instead the reported overshoot *is* the
global offset, printed twice in the same card (once as `Mean offset`, once as
`Corner overshoot`).

**Code path** (`hp-sim-3d/app/quality-monitor.js`):

1. `_recordNormalizedExtrusion` calls `this._updateOffsetEstimate()` at **:646**, *before*
   the current sample's normal equations are accumulated at **:659-667**. The offset
   used to build the "shape" position (`sx = x - offset[0]`, :647-649) therefore lacks
   every direction that the current sample is the first to constrain.
2. For the first sample of the first new direction after a corner, the offset component
   in that new direction is 0 (only the ridge acts), so its shape position is displaced
   by the offset projection on the first move direction: `contourDistance = |offset ·
   firstMoveDir|`.
3. That sample is inside the corner's `±cornerWindow` and has `arcLength >
   corner.arcLength`, so `state.overshoot = this._shapeDistance` at **:772-773**, and its
   shape positions enter `settleSamples` at **:775-777**; the artificial jump to the
   corrected positions makes `_finalizeCorner` (:809-816) report one step of "settling".
4. `cornerOvershootMax` is then latched for the whole print (:846-848) and rendered at
   **:940-941**; `cornerError = max(cornerDeviationAvg, cornerOvershootAvg)` feeds 10 of
   the 100 score points (:859, :141).

**Why the number is also unstable.** Whether the corner-crossing sample is scored at all
depends on `arcLength > corner.arcLength` (:770-772) — i.e. on whether the accumulated E
reaches the vertex as an exact value or a float epsilon past it. Sweeping the number of
steps per 50 mm leg (`measurements.json →
findings.cornerOvershootReportsOffset.triggerSweep`), the same print with the same
1.000 mm offset reports **1.000 mm in 9 of 15 step counts and ≈0.001 mm in the other 6**
(score 88.92 … 96.30). Power-of-two E steps (1024, 1600, 2000) do not fire; 500, 800,
1000, 1500 do. The shipped "Print Squares" preset (`draw_squares_bigger.gcode`, first
move along −Y, E step 0.03125 mm) does **not** fire at its first corner (0.0016 mm), so
the bug is input-dependent — which is exactly why it survived the test suite.

**Same mechanism, opposite sign.** A *genuine* overshoot is under-reported because the
transient bulge is partly absorbed into the running offset: an L with a 0.5 mm overshoot
bulge (exact max distance to the reference path 0.5 mm, final mean offset 0.0097 mm)
reports `cornerOvershootMax = 0.3750 mm` against 0.4903 mm after removing the mean offset
— 23.5 % low (`measurements.json → …genuineOvershootUnderReport`).

**Classification:** shipped, user-visible wrong number (`Corner overshoot`), driven by a
one-sample estimation-order issue; score effect 2–7 points on a 100 mm print, and up to ~37
points on a 2 mm path (54.62 vs 91.96) where the same contaminated samples also enter
`shapeP95` (1.00 mm reported against 0.00 mm exact).
Not a duplicate of any prior finding.

**Limitations:** trigger fragility is part of the evidence — with 2500 steps on the
50 mm leg the same print does *not* fire (0.002 mm), with 2499 or 2501 it reports
1.000 mm, because the only thing that decides it is whether the accumulated E lands
exactly on the vertex or one float epsilon past it. The artefact further needs a corner
whose incoming direction leaves one offset axis unconstrained — i.e. a path whose first
move is not perpendicular to the offset. For a square, only the first corner is affected (`cornerOvershootAvg = d/2`…`d`). The sign
matters: with the offset *opposite* to the first move the same estimate error appears in
`Corner deviation` instead, but bounded by one sample step (0.0996 mm at 0.1 mm sampling,
score 95.27) rather than by the full offset. With no offset, or an offset perpendicular to
the first move, the report is exact.

## Finding 2 — `Path within 0.50 mm` is a time share, not a path share

**Reproducer:** `timeout 30 node repro_percent_within_is_time_share.mjs` (exits 1).

Input: a 100 mm line printed in two halves — 10 mm at 5 mm/s exactly on the path
(1000 samples at one per 2 ms step) and 90 mm at 50 mm/s 1 mm off the path (900 samples).

| quantity | expected | reported |
|---|---|---|
| HUD `Path within 0.50 mm` | 10.0 % (path length share) | **52.6 %** |
| HUD `Path within 0.25 mm` | 10.0 % (path length share) | 52.6 % |
| HUD `Path deviation RMS / 95% / max` | 0.688 / 1.00 / 1.00 mm (sample metric) | 0.688 / 1.00 / 1.00 mm |

The RMS line is **not** part of this finding: 0.688 mm is the correct sample RMS for the
metric the code computes, and it matches the exact value
`sqrt(900/1900) · 1 mm`. A path-length-weighted RMS would be 0.949 mm, but that is a
different quantity and nothing in the HUD claims it. The wrong-metric issue is confined
to the `Path within` share.

**Contract:** the HUD label is `Path within ${formatLengthMm(m.tolerance)} mm`
(`:934`) — "path within 0.5 mm" — but the value is
`withinToleranceCount / sampleCount` (`:674-679`, `:822`). Extrusion samples are emitted
once per executed command / simulation step, so their density is proportional to *time*,
not to path length; slow moves are over-weighted. The source comment only says "within-tolerance
share" (`:186`), which is what the code computes, so this is a **label-vs-quantity**
defect rather than a violated arithmetic contract. Magnitude on ordinary slicer output:
perimeters at 5 mm/s vs infill at 50 mm/s differ by 10× in sample density, so the
reported percentage can be 40+ points away from the path share.

**Classification:** shipped, user-visible, misleading. Not a wrong rms/p95: those are
computed per sample but are *distribution* percentiles, so the weighting matters much
less (and the p95 here is right either way).

**Limitation:** if the intended semantics is "share of printing time within tolerance"
then only the label is wrong; nothing in the module documents that intent.

## Finding 3 — `Quality 100` when the reference overruns the executed path

**Reproducer:** `timeout 30 node repro_quality_100_on_reference_overrun.mjs` (exits 1).

Input: real G-code parsed by `parseGcodeText` — `G1 X50 Y0 E5` followed by two
`G0 X50 Y50 E10` / `G0 X0 Y50 E15` lines. The reference parser treats `G0` as a moving,
extruding line, so the reference is **150 mm path / 15 mm E**; the command layer ignores
`G0` (`workers/moveCommander.js:206-224`), so the machine executes **50 mm / 5 mm E**.

| HUD line | value |
|---|---|
| `Quality` | **100** |
| `Score deductions` | **none** |
| `Progress` | 33.3 % |
| `Corner deviation avg / max (0)` | `-- / -- mm` |
| `Path deviation RMS / 95% / max` | 0.00 / 0.00 / 0.00 mm |

`cornerCount = 0` because a corner is only assessed when samples exist on both sides
(`_finalizeCorner`, `:785-788`), and `_composeScore` hard-wires the corner and ringing
terms to 0 when `cornerCount == 0` (`:859-860`). Unexecuted material and unreached corners
therefore score as perfect, and the score has no progress term — the 0-100 headline looks
like a part-quality verdict even though two thirds of the nominal path was never printed.

**Reachability.** This needs the reference path to describe more extrusion than the
machine executed; an ordinary abort of a print whose reference and executed paths agree
does **not** produce this card. The demonstrated route is a reference/command-parser
mismatch (`G0 ... E...`); that `G0`/`G91` semantics gap is a separate, already-reported
issue and is used here only to supply the mismatch. No claim is made that the card is
common.

**Classification:** shipped, user-visible, **misleading** (not a wrong number): the card
itself discloses `Progress 33.3 %`, and no closed-form expected *score* is defined. It is
a limitation of the headline number, not an arithmetic error.

**Limitation:** with `cornerCount > 0` the corner term does fire, so a partial print that
reached at least one corner is penalised (and then the shape terms also see whatever
tracking error the stop left behind).

## Negatives — what is measurably right

All "exact" values below are independent: closed forms, brute-force polyline distance, or
`O(N log N)` order statistics written for this investigation.

| Check | Input | Worst observed error |
|---|---|---|
| `contourRms` | 800 samples, ±0.05 mm random 3-D noise on a square | 9.4e-4 relative (windowed vs global nearest point) |
| `contourMax` | same | **0** (bit-exact) |
| `contourP95` vs exact nearest-rank 95th order statistic | 60 random log-uniform sets, N = 5…400, distances 1e-9…5 m | **+1.973 %** (bin edge), never below exact |
| histogram rms | same 60 sets | **0** |
| histogram max | same 60 sets | **0** |
| zero distances | 100–500 samples at exactly 0 | `p95 = rms = max = 0`, score 100 — **no NaN** |
| single outlier | 1 sample at 0.5 mm, 99 at 0 | `max = 0.500000 mm` exact; `p95 = 1e-7 m` (0.0001 mm) vs exact 0 — below the HUD's 0.01 mm resolution |
| corner deviation, closed form | 0.2 mm bevel at a 90° corner, exact vertex→polyline distance 0.141421356 mm | reported 0.141421356 mm, error **0** |
| corner detection threshold | turns of 44.9°, 45°, 45.1° | 0, 1, 1 corners — exactly the documented 45° (`:191`) |
| perfect print | square, 800 samples | all deviations ~1e-17 m, score 100.000, `progress` 1.000 |
| rigid offset 0.3 mm | straight line | `contourRms = contourP95 = contourMax = 0.300000 mm`; `horizontal 0.30 / vertical 0.00 mm`; `Mean offset 0.30 mm`; `within 0.25 mm = 0 %`, `within 0.50 mm = 100 %` |
| `meanLag` / tracking vs path deviation | pure lag 2 / 4.9 / 5.5 / 10 mm on a straight line | `contourP95 = 0.0001 mm`, `meanLag = lag`, `trackingMax = lag`; the reported `contourMax = lag` comes only from samples before the path start (a real distance to the nominal path, not the ±5 mm window) |
| units / parser end-to-end | `parseGcodeText` on G21/G90/M82/G92, travel, retract, Z-only prime with E, corner after a travel → ×0.001 positions, mm extrusion | 4 segments with the hand-computed E (5, 5, 1, 4) and lengths (50, 50, 0.2, 40); bounds = extruding endpoints; a 0.2 mm offset reports 0.200000 mm max, 0.2 mm mean offset, 0 mm vertical, progress 1.000 |
| score formula | any input | recomputed from the reported metric values with `100 − Σ w(1−e^(−v/s))` matches the reported score to **0.0** |
| score weights | — | 45+20+10+5+10+10 = 100 (`:138-145`) |

Known, documented range limits (latent, not reachable by a real machine):

* Distances above the top bin edge (~10.18 m) are not resolved: 20 m samples give
  `contourMax = 20 m` (exact) but `contourP95 = 10.1569 m`. The claim covers 0.1 µm–10 m
  (`:58-59`), so this is outside the stated domain; it would silently saturate the
  `Shape 95%` score term if it ever happened.
* `cornerCount` is a count of *assessed* corners only; the path's corner total is never
  computed or shown, so corner coverage (C of K) cannot be read from the report. A
  closed square reports 3 corners (the seam is not assessable), which is a defensible
  convention but looks like a missing corner.
* `Path deviation` is distance to the nearest point of the reference **within ±5 mm of
  arc length** (`contourSearchWindow`, `:189`, `:576-585`). For a raster whose parallel
  lines are adjacent in the segment list, a deviation toward the neighbouring line is
  measured against that line; the ISO-9283 "distance to the nominal path" reading is then
  ambiguous.

## Method

1. Read `quality-monitor.js`, `qualityController.js`, `inspectionToolsController.js`,
   `remoteSpoolSystem.js` and `moveCommander.js`; confirmed the extrusion-event ordering
   (tip before the move it pays for) and the E units (mm, positions m) from code, not
   assumption.
2. Built every input from closed-form geometry: straight lines, L corners, bevels,
   overshoot bulges, squares, circles, and the shipped preset G-code parsed by the real
   parser. The E stream is always tied to the geometric arc length so the "intended
   point" is unambiguous.
3. Compared reported metrics against (a) closed forms, (b) brute-force `O(N·S)` point→
   polyline distance, (c) `O(N log N)` order statistics, and (d) the module's own score
   formula re-evaluated from the reported values.
4. Rendered the real HUD HTML into a stub element (same shape as the shipped test) to
   confirm what a user actually sees, including the displayed rounding.
5. Cross-checked the mechanism of Finding 1 with a step-count sweep and with the shipped
   preset, and confirmed the shipped test that asserts "a rigidly shifted but otherwise
   perfect print still scores high" passes at 92.26 while its `cornerOvershootMax` is
   1.000 mm — the assertion never looks at overshoot.

## Reproduce

```bash
cd research/output/program-2026-11-19-discover-new-reproducible-cases-where-hp-sim5-behaves-incorrectl-b5234a7c/session-2026-11-19-does-the-app-quality-report-measure-what-it-claims-drive-hp-sim-cf06cc78
timeout 30 node repro_corner_overshoot_reports_offset.mjs      # exit 1 (expected 0.00, reported 1.00 mm)
timeout 30 node repro_percent_within_is_time_share.mjs         # exit 1 (expected 10.0 %, reported 52.6 %)
timeout 30 node repro_quality_100_on_reference_overrun.mjs     # exit 1 (score 100 at 33 % progress)
timeout 60 node measure_all.mjs                                # regenerates measurements.json
```

All runs are one process, a few seconds, < 100 MB RSS. Probes `probe*.mjs` are the
exploration trail and are kept for audit; `harness.mjs` holds the shared generators and
the independent references. `probe21.mjs` records the "print only the first leg" variant
that the card is *not* reachable from; it is kept only to show the score arithmetic
behind Finding 3.

## What would change these conclusions

* Finding 1 disappears if `_updateOffsetEstimate` is called after the current sample is
  accumulated (or if corner metrics are recomputed at `runFinalCheck` from the final mean
  offset). Re-running the reproducer would then show 0.00 mm.
* Finding 2 becomes a label fix if "share of time within tolerance" is the intended
  meaning; then only the string "Path within" is wrong.
* Finding 3 requires a decision: count unassessed corners as failures, or gate/annotate
  the score with `progress`.
* Any of the negatives would be contradicted by a counter-example with a documented
  expected value; none was found in ~5 000 synthetic samples across the cases above.

## Limitations of the evidence

* **Simulation-level only.** Nothing here says how often the trigger condition of
  Finding 1 fires on a real Hangprinter print (it depends on the sim's step count per
  move and on the float accumulation of E). It is a report defect, reproducible
  deterministically, not a hardware measurement.
* No physics world was run (host: 4 GiB RAM, 2 cores, swap partly used). The sample
  convention is taken from the shipped tests and from `remoteSpoolSystem`'s command
  ordering, both quoted above; a real run would add physical tracking lag, which the
  module handles correctly in the negatives.
* `HP4`-style multi-machine behaviour (events filtered by `machineId`, `:363-368`) was
  exercised only as a stub; it matched the documented per-machine contract.
* Out of scope by program instruction and not touched here: the 2026-11-18 quality-history
  entry before the command queue drains, and the G-code upload extrusion-accounting bug.
