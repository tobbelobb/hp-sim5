# Hangprinter validation — 2026-11-16

Reviewed e5593f44, RRF 55a80fd91 and Klipper 98bf60294. The square scenes,
anchors, four corner attachments, 2:2:2:2:4 strands and 2:2:2:2:4 reduced
mechanical advantages remain unchanged. Development submodule contents were
not edited; pointers advance to forward 797620c7 and flex 8e0800a1.

## Verified fixes

* Applied and built the dev audit's firmware patches: axis-major RRF flex
  derivative buffers, complete QP bound transitions, double RRF row-space
  arithmetic, stable spool conversions, and Tikhonov hard 0/Tmax clamps even
  with pretension disabled. Tikhonov Tmin remains a soft target. RRF M666 now
  preserves the 0.081 kg mover mass instead of reporting 0.08 kg.
* Actual RRF/Klipper QP kernels match independent bounded least squares within
  0.002 N on the supplied opposite-bound regression and 60 random layouts,
  including force-only and moment rows. Real firmware forward/flex tests pass.
* Equal-strain planning includes nominal loads from compliant unused lines and
  rejects an unsupported multi-dimensional active pretension null space.
  Opposing unused low axes remain compliant during acquisition and returns.
* Candidate information eliminates already-known poses analytically. Ranking
  3,509 candidates took 2.02 s versus 30.53 s with repeated pose solves;
  regression tests compare the resulting matrices with finite differences.
  Removed discarded ellipse calculations from redundant-machine range fits.
* Headless requests serialize behind an interrupted collector's outstanding
  advance. Both a disconnected-client regression and a real acquisition-time
  Ctrl-C applied the selected parameters successfully, without HTTP 409.
* Research collection uses production tuning for every machine; removed the
  HP5-only exception and inadequate HP3/HP4 fixed-force defaults. Physical test
  fixtures now request the required eight encoder samples.
* Fixed quality-monitor extrusion-boundary assignment across travel moves;
  Klipper logo outliers of roughly 600 mm disappear. Raw-move spool conversion
  now handles positive buildup without cancellation or hiding invalid lengths.

## Autocal

Fresh datasets, native production physics, closed-loop motors, 20 solver
iterations, unchanged two-consecutive-sweep 0.5 mm coordinate-step rule,
patience 20, six million physics steps, 1000 s external limit. Wall times include
bootstrap, remeasurement, fitting and acquisition; other validation ran
concurrently. Sweep counts are retained sweeps, excluding discarded rough data.
Errors are Euclidean fitted-anchor errors against firmware anchors, without
alignment. An interrupted fit is not a stable-stop pass.

| Machine | Stop | Max anchor error mm | Sweeps | Wall s |
|---|---|---:|---:|---:|
| HP3 | stable-estimate | 1.023 | 10 | 612.19 |
| HP4 | stable-estimate | 4.559 | 14 | 937.39 |
| HP5 | stable-estimate | 1.851 | 11 | 544.45 |
| HP5 reduced | Ctrl-C | 2.414 | 14 | 999.57 |

Evidence is under `/tmp/hp5-review/`: `patched-hangprinter_3`,
`patched-hangprinter_4`, `hangprinter_5-final`, and `patched-hp5r`, each with
`.json`, `.full_auto_log.jsonl` and `.headless/manifest.json` artifacts.

Reduced autocal remains open. Its last five fit steps were
3.254, 0.535, 1.288, 0.494 and 0.523 mm; it missed the second stable hit.
The selected fit's A/B/C/D/I errors are 1.958/2.414/1.137/1.052/0.857 mm.
Ctrl-C application succeeded; `normal_completion` correctly remains false.
The prior run's latest-fit error grew from about 2.35 to 4.03 mm as coverage
expanded. Increasing the pose solve from 25 to 100 iterations barely changed
those estimates; this is systematic range-model drift, not pose nonconvergence.
Two diagnostic sweeps using true-anchor force profiles still left 0.0369 mm
consistency RMS after common offsets/scale; a pivot-rotation correction only
reduced it to 0.0346 mm. Remaining routing/load/compliance mismatch is therefore
not resolved by a better initial anchor estimate alone. Its exact decomposition
is still open. Firmware flex is disabled (`F0`) during autocal, so the patched
force allocator is not its direct cause. HP4 and reduced both retained 14 sweeps: 937.39 versus 999.57 seconds,
so the remaining failure is not a fivefold slowdown relative to HP4. No timeouts
or stop thresholds were raised. HP5 candidate tests enforce clearance of at least 10% of each anchor's
home distance; the configured top anchor is far from the low-anchor plane.

## Printing and regression suites

Regenerated registered HP3/HP4/HP5 direct/layered command streams with the patched
firmware, plus the manually selected HP4 Klipper logo stream. Complete HP5
squares pass all eight firmware/native/Wasm combinations, with tracking RMS
below 0.4 mm, contour maxima below 0.5 mm, and zero missed steps.

| Machine | Firmware | Tracking RMS mm | Contour max mm | Wall s |
|---|---|---:|---:|---:|
| hp3 | rrf | 0.2196 | 0.1922 | 28.20 |
| hp3 | klipper | 0.2192 | 0.1320 | 27.83 |
| hp4 | rrf | 0.2177 | 0.1934 | 145.50 |
| hp4 | klipper | 0.2283 | 0.1842 | 122.51 |
| hp5 | rrf | 0.2148 | 0.1762 | 36.81 |
| hp5 | klipper | 0.2328 | 0.1840 | 34.20 |
| hp5_pulley_reduced | rrf | 0.2315 | 0.2192 | 174.12 |
| hp5_pulley_reduced | klipper | 0.2429 | 0.2276 | 162.35 |

All complete-logo rows had zero missed steps. Printing uses catalogue defaults:
open-loop motors, one physics substep, one solver iteration for HP3/direct HP5
and ten for HP4/reduced HP5. No Rust physics changes were needed.

Final checks: 679 fast Python tests passed; the complete non-heavy Python run
passed 753 with one viewer-dependent skip (the subsequently added HP5 clearance
test also passed). 397 Jest tests, 85 physics tests (one optional skip), 12 RRF
and 13 Klipper tests passed. The two actual-kernel checks are included in Python.
The heavy interruption test passed. One separate heavy layered HP3 global-radius
fit test fails before and after the firmware patches: its second adaptive sweep
reports fixed-anchor drift, then fails returning U to encoder origin. This is
not a passing full-suite result; that layered spool-fit/return path remains open.

## Reproduction

Use an unused dataset directory for a fresh bootstrap:

```bash
cmake --build RRF/build --target rrf_simulator -j 4
.venv/bin/python scripts/compile_machine_commands.py --machine hp3 --machine hp4 --machine hp3_w_line_layers --machine hp4_w_line_layers --machine hp5 --machine hp5_pulley_reduced
mkdir -p /tmp/hp5-validation
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 .venv/bin/python autocal/autocal.py --machine-type hangprinter_5_pulley_reduced --dataset /tmp/hp5-validation/reduced.json --headless-sim --recording-mode none --patience 20 --verbose
```

Substitute `hangprinter_3`, `hangprinter_4`, or `hangprinter_5` and fresh dataset
paths for the other autocal rows. Require manifest `normal_completion: true`
and `stop_reason: stable-estimate` for a stable-stop pass.

```bash
.venv/bin/python -m pytest -m 'not heavy' -q
.venv/bin/python -m pytest -m heavy -q
.venv/bin/python RRF/tests/test_flex_mode_position.py
.venv/bin/python -m pytest klipper/test/klippy/test_winch.py -q
npx jest --runInBand
node --test --test-concurrency=1 tests/benchmark/*.test.mjs
node tests/benchmark/logo_accuracy.mjs --machine hp5_pulley_reduced --firmware klipper --open-loop
```

The remaining heavy failure can be isolated with:

```bash
.venv/bin/python -m pytest autocal/tests/test_headless_sim.py::test_fresh_hp3_full_auto_global_radii_and_matched_browser_collection -m heavy -q
```

HP4's Klipper logo is manually selectable rather than catalogued:

```bash
.venv/bin/python klipper/klippy/klippy.py public/klipper/hp4/printer-hp4-linux-mcu.cfg -i public/gcode/Hangprinter_logo6.gcode -o /tmp/hp4-logo.serial -d public/klipper/linux_mcu/klipper.dict
node tests/benchmark/logo_accuracy.mjs --machine hp4 --firmware klipper --commands /tmp/hp4-logo.serial --open-loop
```
