# HP5 extra sensor: implemented with local coupling validation

The spare low line is now a sensor during profiled autocal measurements on fresh
HP5 and HP5 pulley-reduced runs. Production implementation is commit `089a7ad3`
on main. Three coding/end-to-end passes were used. Manual calibration outputs
and the other research program were not modified.

## Decision and reviewed evidence

The [direct review](session-2026-11-17-independently-review-the-completed-direct-hp5-sensor-policy-expe-b225ae5c/report.md)
reproduced persistent slack at 0.05 N and tautness at 0.3 N on its tested poses.
Its broad-pose synthetic benefit did not reproduce on the narrow physical sweep.
The [reduced review](session-2026-11-17-independently-review-reduced-hp5-matched-short-collection-eviden-748ceb19/report.md)
verified a frozen low-force encoder and a useful fifth range with measurement-only
pretension. Same-recording four/five-range error was 22.13 / 1.85 mm, with the
review's force-label, bias, covariance and validation-leakage corrections taking
precedence over the original report. Raising global idle force collapsed coverage.

The [final delegated investigation](session-2026-11-17-run-a-predeclared-paired-full-autocalibration-test-on-hp5-pulley-3e03f53e/report.md)
validated a fresh short prototype and baseline parity. Its full baseline was
interrupted at the user's deadline and treatment was not launched. It provides
no paired full-loop result. Production implementation proceeded on the combined
evidence and the user's explicit instruction to implement.

## Implementation

Pullout/idle retain the low force. After checked rough bootstrap, the spare line
is preloaded and held at five times the tuned start force, capped at the tuned
maximum (2.02 N direct / 2.83 N reduced in these trials). Primary force profiles
account for that known load. Each spare range must resolve reversible encoder
coupling under unchanged measurement loads; frozen, hysteretic and untestable
ranges are excluded. This is stricter and costs more movement than the prototype's
sweep-level gate. There is no claim that these probes establish absolute tautness
or accurate commanded-force tracking.

Every point records physical-axis extra lengths, means, commanded force and probe
evidence. Reversed sweeps retain the real primary sensor identity. The fitter
handles mixed four/five-range points, two commanded sensor loads, reeving-aware
stretch and home offsets on extra ranges. Rejected extra ranges still contribute
their commanded load. Historical profiles retain their collection policy.

The first full trials exposed a planner crash when the most informative sweep
had no force-feasible interval. The fix searches the complete information ranking
until a force-feasible profile is found, while preserving configuration errors.
The saved direct failure skips 29 infeasible candidates before selecting a usable
one ([reproduction](production-validation/force-plan-repro.json)). This was the
only adjacent production defect fixed because it blocked the feature.

## Production-path validation

Fresh first-pass trials completed rough bootstrap, profiled remeasurement and new
sweeps on both machines, then hit that planner defect. A second direct trial was
stopped once a complete-ranking search proved necessary. Final trials resumed
only these saved observations in identical stock scenes, collected new sweeps,
ran normal history selection and applied M669. Each finalized normally and all
owned processes stopped. This validates continuation through collection, fitting,
planning, stopping and firmware apply; it is not a fresh final-revision baseline/
treatment comparison. Continuation time below excludes the earlier bootstrap.

| Machine | Stop | Sweeps / points | Accepted extra ranges | Applied worst coordinate error, absolute / translation-aligned (mm) | Matched full-recording refit, four → five ranges, aligned (mm) |
|---|---|---:|---:|---:|---:|
| direct | stable-estimate | 12 / 240 | 216 | 1.07 / 0.73 | 3.11 → 0.70 |
| reduced | patience | 13 / 260 | 216 | 1.17 / 0.98 | 1.27 → 2.88 |

Errors use nominal stock M669 anchor coordinates. Translation alignment subtracts
the mean coordinate displacement. The applied model is selected by normal history
selection; it is distinct from a refit of all final observations. The matched
refit keeps all commanded forces fixed, starts both fits at the same 2% perturbed
nominal geometry, fits home offsets then shifted ranges, and changes only whether
the extra ranges enter the objective. It uses all points, not a held-out test.

The direct refit improves, but the reduced late-data refit worsens. This negative
result limits the conclusion: a moving, reversible encoder can still carry smooth
pose-dependent bias, and more ranges need not improve a mechanically misspecified
fit. The reduced selected/applied estimate is better than its final all-data
refit, so reporting only its final selected accuracy would conceal that limitation.
Earlier short reduced results and the final direct result support the feature's
utility; they do not establish universal accuracy or faster calibration. More
force/bias modelling may help, but was left outside the three-pass implementation.
No hardware validity is established.

Checks: 692 fast Python tests passed; 168 collector tests passed; exact frozen,
reversed-role, mixed-range, offset and force-feasibility regressions passed. An
old manual dataset's per-point corrected residuals changed by at most 9.1e-13 mm.
The new opt-in heavy test reproduces a fresh full calibration for each machine;
its verifier was also run on both completed continuations. Full fresh tests were
not repeated after the planner fix; successful continuation evidence and the exact
failed-stage replay are preserved instead.

[Compact evidence and hashes](production-validation/manifest.json) include final
datasets, manifests, iteration logs, matched-refit code and the implementation
patch. Raw simulator logs remain under `output/extra-sensor-production/`; they are
not needed to recompute the archived matched fits. Recompute both matched fits from the archived data with
`OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python research/output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/production-validation/score.py`.
