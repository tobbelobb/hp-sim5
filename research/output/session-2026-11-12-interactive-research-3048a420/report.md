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

Likely mechanism (from reading the source, not patched or tested):
`GCodes::LockMovementAndWaitForStandstill` (GCodes.cpp:1851) calls
`MovementState::UpdateOwnedDriveEndpointsFromMotors()`. That copies
`dms[drive].currentMotorPosition` into the last endpoints. In the host build
those positions seem to stay 0 for the CAN-remote drivers, so the ring's
endpoints reset to home and the next CAN message carries the difference. In the
logo the only standstill is the very end, which is why everything happens
there. Any G-code with dwells, `M400` or heater waits mid-print would get the
same bogus moves.

**Not changed here.** The fix belongs in the RRF host shim: keep
remote-driver motor positions updated, or skip the reset. After that, the
`.can` files need regenerating. A gate could also scan every bundled `.can` for
a final message of more than ~100 steps.

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

## Next steps (proposed, not done)
1. Patch the RRF host simulator's standstill endpoint reset and regenerate the
   20 `.can` files. Then rerun `logo_missed_steps.mjs`: expect 0 missed steps.
2. Add a bundled-`.can` sanity gate on maximum per-message step jumps.
