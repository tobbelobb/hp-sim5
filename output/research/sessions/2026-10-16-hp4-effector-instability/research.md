# HP4 effector instability investigation

Objective: identify the physical/numerical cause of upward corner vibration and the failed no-buildup-reference-004 bootstrap.

Accepted steering: user requires stable physics, not acceptance of perpetual motion. Initial collector timeout workaround was removed in full. No collector change remains.

Hypotheses: force pull genuinely travels slowly; rotational/cable constraints may sustain numerical vibration. Neither cause is established yet.

Budget: offline recording inspection and bounded headless replay (initial 600,000 steps, no RRD); no hardware or live continuing-world movements. Extend bounded trials only as evidence requires. Recording performance guide read; JSON numerical probes avoid recorder overhead.

E01: inspected saved browser RRD events and extracted ABC encoders. Size tune begins at sim 1046.062, final sample at 1167.422. A increases from zero to 1376.17 degrees; B/C decrease, D remains zero. Initial progress is genuine travel, not transport or wall-clock timeout.

E02: replay saved command arrival boundaries into a fresh no-layering HP4 world with closed-loop motors, 2ms steps. In progress. Current-source replay is not presumed identical to recorded browser engine.

Next decision: measure body rotation/corner heights, compare replay to saved recording, isolate sources of sustained vibration.

E03/E04 completed: budget expanded to bounded 80k-step matched force trials across JS/native/Wasm and eight 20k-step branches from one displaced state. The old solver rocks indefinitely in this state; enforcing tension-only damping removes the cycle. No collector changes remain. Final full recorded-command replay uses 600,000 steps, no recorder, for further limits assessment.

Implementation: clamp cable lambda <= 0 in JS/Rust/Python/Warp, both finite and zero stiffness. Rebuilt native/Wasm. Added shortening fixture and invariant tests. Cross-language derived force roundoff after 200 steps required 1e-9 N force-only tolerance for one fixture; no pose tolerance change.

Validation: 183 JS tests, 97 Wasm parity tests, six native tests (one skipped), 162 Python differential tests, 107 Warp tests (one skipped) passed. Initial Warp compile failed on read-only cache and passed on authorized outside-sandbox rerun. Initial PGO native source list check failed on profile dependency metadata; ordinary no-PGO rebuild passed unchanged test.

Next decision: report physical stability correction and distinguish it from the original slowly moving size-tune endpoint. Full-auto retry is not validated.

E05 complete: fixed full-command replay still rocks after position-mode transition (roughly 8–10 mm body vertical range per final 5s window). This invalidates any claim that the clamp alone fixes every observed instability. User notified; continue isolating full loaded state. E06 now branches timestep/damping/iteration variants after full history, preserving a numeric checkpoint for cheaper follow-up. Budget extended to this 584k-step prefix plus eight 20k-step branches. No live world changed.

E06 completed: full-history stress checkpoint still rocks with damping disabled, more solver passes, or the unilateral clamp alone. Four 0.5 ms internal physics steps settle the same checkpoint in JS/native/Wasm. E07 implemented production substep scheduling in JS/Python ECS and 3D composition; remote commands, extrusion and recordings stay outside the substep group, and the external timestep is restored even after failure. Only rigid-body/cable worlds request four steps. The original 2 ms firmware/sample clock is retained.

E08 (not shipped): accumulated per-iteration XPBD multipliers did not settle the full loaded checkpoint. E09 (not shipped): refreshing member spin world orientation during correction also did not settle it. No changes from either experiment remain.

E10 cross-language audit: finer integration amplifies derived force/velocity roundoff and passive-wheel phase differences. Native/Wasm vs JS 200-command HP4 comparisons retain pose bounds (2 micrometres, quaternion 1e-5), while assigning separate 1 mN force, 0.1 mm/s linear velocity and .005 rad/s angular bounds. Raw low-level parity remains much tighter. Python HP4 fixture bounds are updated from measured numerical differences; no autocal settling/acceptance bounds are changed. The sustained-motion test now explicitly uses autocal's closed-loop motor mode. A separate open-loop numerical audit is preserved and is not represented as strict long-horizon parity.

Stress final 5s: body vertical range 8.733 mm and CornerAL range 16.300 mm under the old 2 ms scheduling (even with unilateral clamp). With four substeps all three engines settle below 1e-9 mm and omega becomes zero. Artifacts: stress-summary.json, stress-stability.png, full-checkpoint.json.


## Final wrap-up

User requested stopping further investigation and restoring a clear stopping point. Final quaternion damping uses atan2 to preserve small rotations; strict Wasm and Warp parity checks pass without temporary broad tolerance allowances. Final restored-checkpoint comparison shows approximately 16.046 mm corner oscillation at one 2 ms step, settling to numerical noise at four 0.5 ms steps. Sustained 1000-step machine repeatability check passed (131.56 s). Original full-autocal retry remains untested. Changes are local and uncommitted. No further experiments planned.
