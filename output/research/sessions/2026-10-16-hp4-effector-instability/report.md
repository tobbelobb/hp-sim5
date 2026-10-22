# HP4 effector instability investigation

The reproduced loaded-state rocking is fixed in offline tests. The original full-autocal command has not been rerun. All code changes remain local and uncommitted; autocal acceptance thresholds are unchanged.

## Original failure

Bootstrap force tuning completed. Workspace size tuning then timed out after 120000 simulation milliseconds waiting for stable encoders. The recording shows substantial slow horizontal travel during that interval, rather than proving that corner rocking caused this particular timeout. Consequently the physics regression below does not establish successful full-autocal collection.

## Causes and corrections

The production pipeline integrated stiff cables and light rotating members once per 2 ms firmware tick. A loaded HP4 checkpoint sustains rocking at that step size. Four internal 0.5 ms physics steps settle the same state. Command dispatch, extrusion and recording retain their external cadence. This increases physics work per tick.

Cable damping had two additional defects. Its multiplier could become compressive, allowing a rope to push; the multiplier is now clamped to its tensile sign. Its rotational displacement calculation used acos(w), losing small rotations when quaternion w rounded to one. Computing the angle with atan2(norm(xyz), w) preserves those rotations. These corrections are mirrored in JavaScript, Rust, Python and Warp.

The damping clamp alone did not settle the full loaded stress case. More solver iterations and other investigated alternatives did not replace the need for smaller integration steps. Temporary broad parity tolerance changes and experimental alternatives were removed.

## Final physical evidence

The final comparison uses identical restored numerical state and final damping math in each backend. During the last five seconds of a twenty-second continuation, one 2 ms native step produced approximately 16.046 mm corner travel and 9.563 rad/s maximum angular speed. Four 0.5 ms steps settled the state to numerical noise in JavaScript, native Rust and browser Wasm, with zero recorded angular velocity. See [measurements](stress-summary.json) and [plot](stress-stability.png).

An early JSON snapshot lost nonfinite constants. Every affected slot was audited against the initialized schema and restored before repeating the final comparison. The checked-in regression fixture encodes these constants explicitly. This replay is a stress reproduction, not exact trajectory parity with the original browser recording.

## Validation

- JavaScript cable/scene tests: 194 passed.
- Browser Wasm strict parity benchmark: 99 passed.
- Native engine benchmark: 6 passed, 1 GC-related skip.
- Warp strict tests: 104 passed, 5 GPU skips.
- Loaded-state physical regression: passed, requiring decay over 10000 external ticks.
- Python differential suite: 163 cases passed; the remaining existing force comparison passed its targeted rerun after a force-only 1e-9 N absolute tolerance adjustment. Pose tolerances were retained.
- Machine/lifecycle/scheduler checks: 17 passed before the final sustained-case adjustment. The sustained 1000-step repeatability case then passed using the fixture's existing per-quantity tolerances (1 passed, 12 deselected).
- git diff --check passed.

The sustained test and force comparison were rerun individually after their adjustments; the combined suites were not rerun again. The native and Wasm artifacts were rebuilt. Reloading the simulator and rerunning the original full-autocal command remain the end-to-end verification steps.

## Scope and stopping point

The user requested a prompt wrap-up after the investigation expanded. No further experiments are planned in this session. No commits or hardware actions were performed. Detailed exploratory history remains in [research.md](research.md).


Follow-up: default-four integration was rejected for performance and restored to one. The original browser command still fails at one step. See [single-step follow-up](../2026-10-16-hp4-one-step-autocal/report.md).
