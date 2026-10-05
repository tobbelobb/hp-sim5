# HP4 logo headless performance

Measured on 2026-10-05 with Python 3.12.3, Linux 6.8 and an Intel Core i7-12700K.
The baseline is commit `0535d9a775b9516b3e8eb387ff2dd2996b5bea63`.
Both firmware streams were generated from `public/gcode/Hangprinter_logo6.gcode`.
The [benchmark instructions](README.md) reproduce command generation and runs;
[raw measurements](hp4_logo_results.json) include all timings and command hashes.

Each measurement advances the default HP4 for **3,000 steps / six simulated
seconds**, including initial travel and deposition. Timings are medians of three
fresh-world runs, executed sequentially. Physics uses the authored 2 ms timestep,
line layering, open-loop motors and two solver iterations. Simulation timings
exclude command parsing, scene loading and Rerun recording.

| Commands | Original | Optimized | Speedup | Original steps/s | Optimized steps/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| RRF | 120.324 s | 60.022 s | 2.00× | 24.9 | 50.0 |
| Klipper | 119.162 s | 59.767 s | 1.99× | 25.2 | 50.2 |

RRF optimized repeats: 59.877, 60.022, 60.142 s.
Klipper optimized repeats: 59.968, 59.767, 59.581 s.

The complete scheduled streams contain 1,038,258 RRF and 1,025,675 Klipper
commands, representing about 34 minutes of motion. These results measure their
first six seconds; complete-print wall times were not measured. Scene loading
takes roughly 0.03–0.04 seconds and parsing the complete command JSON takes about
one second on this machine. Neither explains the simulation cost.

## What changed

The baseline profile of 1,000 RRF steps attributes roughly 55% of runtime to the
cable constraint solver, 20% to attachment updating and 13% to over-correction
resolution. NumPy's general cross-product dispatch alone accounts for about 21%
of total profile time, nested inside those systems.

The final 1,000-step profile spends about 56% in the cable solver, 18% in
attachment updating and 9% in over-correction resolution. These are instrumented
profiles, separate from the wall-time measurements above; the baseline profile
uses the existing browser CAN preset and the final profile uses the regenerated
RRF stream. The sequential cable solve remains the largest cost.

- Use direct three-vector cross products and a dot-product-based length helper.
  Lengths retain the original NumPy reduction order.
- Keep quaternion arithmetic in Python floats and clamp scalar rotation values
  directly, retaining raw quaternion transform semantics and numerical ordering.
- Calculate winding frames and angles only for endpoints and callers that use
  them. Over-correction geometry does not calculate unused winding telemetry.
- Cache cable-plane bases by their exact normal values, with a 512-entry bound.
  Public results are copied so callers retain ownership; live normal changes
  select a new basis.
- Remove discarded solver endpoint coordinate transforms, duplicate spool
  inertia calculations and repeated joint direction/length calculations.

The system sequence, timestep, solver iteration counts, correction order,
friction, motors, encoders and extrusion remain active with the same equations.
No optional acceleration dependency is required.

## Validation

Every version reproduces its own final snapshots exactly over three runs. For
each firmware, original and optimized final frame/cable snapshots also compare
exactly, including poses, attachment points, lengths and forces.

The complete native 3D suite passed **270 tests**, including slow tests, after
the final changes. The focused firmware-player suites passed **7 tests**.
They cover live JS differentials, strict sustained motion,
torque/position transitions, mutable topology, scene replacement/append and saved
Rerun recordings. Existing parity tolerances are unchanged.

```bash
.venv/bin/python -m pytest tests/python/cable_joints_3d -q -m ""
npm test -- --runInBand \
  tests/js/hp-sim/rrfCanPlayerBinaryCompatibility.test.js \
  tests/js/hp-sim/klipperMcuCommandPlayerSync.test.js
```
