# Quality Monitor check-up, 2026-11-12

Baseline revision `b497fbab` (main), RRF submodule `c78636d5f`. Changes are
uncommitted in `hp-sim-3d/app/quality-monitor.js` and
`tests/js/hp-sim-3d/qualityMonitor.test.js`.

## 1. 1220 missed steps after a near-perfect HP4 logo

**Reproduced headlessly.** [`scratchbook/logo_missed_steps.mjs`](scratchbook/logo_missed_steps.mjs)
plays the whole bundled `Hangprinter_logo6_hp4_w_line_layers.can` (1,038,258
ticks, 2076 s simulated) through the production headless world: native engine,
open loop, layering on, one substep. Wall time was 82.6 s. It ends with exactly
**1220 missed steps**, matching the screenshot. Per-step log:
[`scratchbook/full_run.json`](scratchbook/full_run.json).

**When they happen.** All 1220 (305 slip cycles: A 168, B 58, C 52, D 27)
happen in the **last 33 ticks (66 ms)**. There are zero before that. In the
same 66 ms the effector moves ~21 mm back in Y and ~10 mm up.

**Why.** `Move` commands carry absolute motor angles. The last CAN message
(47,500 step-clock ticks = 63 ms, the same timing as the final `G1 E… F2400`
retract) also moves A −10277, B +4168, C +3322 and D −2250 steps. That takes
every spool motor back to step 0 (the home pose, about 441 mm away) at about
7 m/s. No motor can follow this, so the stalls are physically correct and the
Quality Monitor counts them correctly. The commanded program is wrong.
[`scratchbook/can_tail_scan.mjs`](scratchbook/can_tail_scan.mjs) shows that 8 of
the 10 bundled logo `.can` files end this way (all except `hp3_w_line_layers`
and `skycam_w_line_layers`). The `draw_squares_bigger` files are unaffected
because they end with `G1 X0 Y0 Z0`. The bad move was already in the HP4 file
before the 2026-11-11 regeneration (checked `50e826bf^`).

**Where it comes from: the RRF host simulator, not the G-code.** Repro files
are in [`scratchbook/rrf_repro/`](scratchbook/rrf_repro/): a 40-line logo
prefix plus a short tail, run with
`RRF/build/rrf_simulator -c sys/config_hp4_w_line_layers.g`.
- The bogus move appears with and without `G28 X0` / `M84`, and with `M666 F0`.
  Flex compensation is not involved.
- With `G4 P100` dwells between moves (`_qc_multi`), **every** move that ends at
  a standstill gets a final segment that sends the motors back to step 0. The
  cumulative step count is exactly 0 after each one.
- The simulator always reports `Final machine position: X≈0 Y≈0 Z≈0`.

### Firmware fix (follow-up, RRF fork commit `7eb7a97bf` on `3.7-x64-rebase-3`)

**Host-only, and it only shows in CAN output.** At every wait for standstill,
`GCodes::LockMovementSystemAndWaitForStandstill` calls
`UpdateOwnedDriveEndpointsFromMotors()`, which copies
`dms[drive].currentMotorPosition` into the last endpoints. On hardware, the step
ISR (`TakeStepsAndCalcStepTimeRarely` for remote-only drives) keeps those
positions current, so real Duets are not affected. The host build never runs
that ISR. Its `FreeOldSegments` hack discarded finished segments without
counting them, so the positions stayed at 0. The host's only output is the CAN
capture (`.csv`/`.can` and the HTTP motion stream), so that is where the bogus
moves showed up.

Fixing that alone was not enough. Repeated runs of the same file gave
different results, exposing two thread races in the host's FreeRTOS shim (the
Move task is a real `std::thread`). On hardware, task priorities prevent both:
1. The retire loop calls `cdda->Free()` before advancing `getPointer`, so the
   Main task can read `getPointer->GetState() == empty`, treat the ring as idle
   with ~200 DDAs still queued, and reset the endpoints mid-print. This was
   found with temporary logging of scheduled/completed counts at the reset.
2. `ReadMove` clears `segmentsLeft` before `AddStandardMove` puts the move in
   the ring.

The fix (+30 lines, all `#if RRF_HOST_BUILD` except the position accounting in
the host-only `FreeOldSegments`):
- `FreeOldSegments` advances the DM position for each freed segment, as the
  ISR's end-of-segment branch does (`netStepsThisSegment` for a started
  segment, otherwise `length + distanceCarriedForwards` truncated).
- `TaskCriticalSectionLocker` (the shim's global mutex) around the retire loop,
  around `ReadMove`+`AddStandardMove` for both rings, and in
  `WaitingForAllMovesFinished`.

Ablations (3 runs each of `_qc_multi`, `_qc_nodwell`, `_qc_tail_full`):
without the `WaitingForAllMovesFinished` lock, the retire-loop lock does
nothing (multi-thousand-step jumps, nondeterministic). Without the `ReadMove`
locks, the repros still pass. I kept them because that race window is real
but these tests don't reach it. An extra "free all remaining segments on the
last retire" turned out to be unnecessary and was dropped.

Result: every repro is deterministic over 3–4 runs with no message above 63
steps. With and without dwells, the final step totals agree within ±1 step
(A −3172/−3173, B 1179/1180, C 2286/2285, D 266/265) and the final position is
X=30, Y=−150 (it was 0). Interactive server mode: after `M400` and `G4 P100`,
`G1 X101` moves 1 mm and `M114` reports X=101. RRF HTTP endpoint tests: 6/6
pass. `npm run test:physics`: 7/7 pass.

**Regenerated all 20 `public/RRF_CAN_commands/*.can`** with
[`scratchbook/regen_can.sh`](scratchbook/regen_can.sh) (9 s in parallel).
Every logo file now ends with only the 2 mm retract (E −192 steps, no
spool-motor steps). `draw_squares_bigger_hp4_no_buildup.can` came out
byte-identical. Full HP4 layered logo in the native sim
([`scratchbook/full_run_fixed.json`](scratchbook/full_run_fixed.json)):
**0 missed steps** (was 1220), 82.7 s wall. A QC replay gives score 58 (no missed-step deduction, was 49), 80.3 % within 0.5 mm, 45.1 % within 0.25 mm, position-error max 2.6 mm.

## 2. Cost of the end-of-print quality check

Bench: [`scratchbook/qc_bench.mjs`](scratchbook/qc_bench.mjs) (`node qc_bench.mjs <full_run_extrusions.json> both`; the 93 MB sample file is written by `logo_missed_steps.mjs` and not kept) replays the
992,533 extrusion samples recorded in the full run into `QualityMonitor` with
the real logo reference (19,642 segments). Each number below repeated within
±5 % over 3 runs.

| path | before | after |
|---|---|---|
| live scoring, whole print (spread over ~2000 s) | 354 ms | 245–258 ms |
| `runFinalCheck` after a normal print | 0.2 ms | 0.1–0.2 ms |
| Live QC unchecked or Finish ASAP: queued samples drained at the end | **287 ms single stall** | **0.1 ms** (samples scored as they arrive, 194–203 ms spread over the print) |

Changes:
- Removed the pending-extrusion queue. Samples are always scored when they
  arrive. That costs ~0.25 µs per sample, against ~80 µs per native physics
  step. The "Live Quality Checks" toggle now controls only the HUD and the
  extrusion tint, so finishing a print never has a backlog to drain.
- Made the hot path allocation-free. Intended-point, contour, shape and
  offset results are written into fields instead of new objects; corners keep
  their last sample as scalars; active corners are compacted in place.
  Garbage collection was ~20 % of the profile.
- The path-deviation and shape nearest-point searches now share one pass over
  flat `Float64Array` copies of the reference. Before, there were two passes
  over entry objects, each with a closure.
- Corner deviation is computed only inside the corner window.

Metrics are bit-identical before and after removing the queue. The hot-path
rewrite changes nothing beyond the fix below.

### Position error max 891 mm (found during item 2)

The HUD's "Position error … max 891.4 mm" came from one sample (step 629648).
Firmware rounds E to whole steps, which left reference line 13869 0.015 mm of
filament longer than what was extruded. So the first prime sample after the
following travel was still assigned to that line, 891 mm away. That single
sample raised position-error RMS from 0.59 mm to 2.46 mm. Fix in
`_locateIntended`: if the next entry starts after a travel, the sample is
within `travelSyncExtrusion` (0.1 mm of filament) of it, and the sample is
closer to that entry's start than to the current line's end, then E is
resynchronised to the next entry.

Full-logo result: position-error max 891.4 → 2.6 mm, RMS 2.46 → 0.59 mm,
progress 0.9999997 → 0.99999999. Other metrics move by at most a few µm
(corner and lag statistics shift slightly from the resync at 241 travels).
Score without missed steps: 58.9 → 58.8. The new test "a line that received
slightly less filament than planned…" fails on the old code (0.70 m) and passes
on the new code.

## 3. "Path within 0.25 mm"

Added `fineTolerance = 0.25 mm` and `withinFineTolerance`. The HUD now lists
"Path within 0.25 mm" above "Path within 0.50 mm". Full HP4 logo: 45.0 %
within 0.25 mm, 80.4 % within 0.5 mm. Covered by a new unit test.

## Verification
- `npx jest`: 52 suites, 303 tests pass (before the last test was added);
  `tests/js/hp-sim-3d`: 100 pass.
- HUD layout not checked in a browser (headless only).

**Cost of the fix.** Converting the full HP4 layered logo takes 1.72 s before
and 1.79 s after (mean of 3 alternating runs each, about +4 %), with the same
~5.4 MB peak memory. Browser/native physics and `.can` playback are unchanged:
they only read the generated files.

**Regression gate.** `tests/js/rrf/bundledCanStepJumps.test.js` (jest, ~3 s)
parses every bundled `.can` and fails if any single message moves a spool
motor more than 1000 steps. The regenerated files peak at 232 steps
(≤ 11k steps/s); the bogus ones were 3801–14877 steps (≥ 60k steps/s). With
the old HP4 layered logo swapped in, the gate fails with 10277.

## Next steps (proposed, not done)
1. Klipper-generated `public/mcu_commands` were not checked for the same class
   of bug.
