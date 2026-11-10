# HP4 autocal wobble: one XPBD damping displacement per step (per-visit re-evaluation is an energy source)

Status: wobble root cause fixed and reproduced in RRDs; full one-substep autocal
completion still blocked by a separate post-move drift.

## Abstract

HP4 autocal at the default `--physics-substeps 1` did not settle: the effector
limit-cycled, settling checks timed out and force selection moved. A
same-backend A/B Rerun recording of a loaded HP4 checkpoint reproduces the
wobble: with the legacy damping term the late-window effector Z range is
4.07 mm with 3.63 rad/s peak angular speed at about a 24 ms period and cable
force swinging 18.0-46.2 N; with the fix it is 7.2e-14 m at zero angular speed
with steady force. Root cause: the XPBD damping displacement
`grad(C)·(x - x_prev)` was re-evaluated on every Gauss-Seidel visit, so HP4's
second of two visits consumed its own correction; at dt = 2 ms with
`damping = 5` and `compliance = 1/20000` (`gamma = 0.125`, `alphaTilde = 12.5`)
the term adds rather than removes energy. Freezing the displacement at its
first visit per step, mirrored in the JavaScript, Rust/native/Wasm, Python and
Warp solvers, settles the state and matches PBD guidance to keep damping out of
the position projection. Autocal still fails at one and two substeps, but on a
monotone 14-15 deg/s post-move encoder drift with no G-code in flight. A
four-substep control completes the same bootstrap and fit (score 0.5295), while
eight solver iterations at one substep fail identically and run 2.5x slower.
All evidence is simulated; no calibration-accuracy claim is made.

## Topics

HP4, autocalibration, XPBD, damping displacement, Gauss-Seidel visit order,
energy injection, limit cycle, physics substeps, effector settling, PBD best
practice, Rerun RRD reproduction, headless simulation.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Per-visit re-evaluation of the XPBD damping displacement adds energy and sustains the HP4 wobble | E05, E06, E07, E08, E09, E13, E14, E15 | Supported | [E13 A/B RRD](experiments/E13-rrd-js-pair/), [E14 visit trace](experiments/E14-visit-trace/visit-trace-summary.json), [E07 matrix](experiments/E07-matrix/matrix-fixed-vs-legacy.json) |
| More physics substeps mask the wobble by shrinking dt | E01, E07, E12 | Supported as masking, not a fix | [E07 matrix](experiments/E07-matrix/matrix-fixed-vs-legacy.json), [E12 timing](experiments/E12-timing/timing.jsonl) |
| Gravity, slack cables or non-taut friction cause the wobble | E05, E06 | Rejected | E05 gravity-off and 51/51 taut |
| Earlier load-bookkeeping and compliance fixes were sufficient | previous sessions, E01 | Rejected | [One-step autocal](../2026-10-16-hp4-one-step-autocal/report.md) |
| Freezing the displacement per step makes a full one-substep autocal run complete | sub1-s1, sub2-s1, sub1-fixedforce | Rejected as stated; still fails | [sub1 fixed-force log](autocal-sub1-fixedforce/sweeps.full_auto.log) |
| The remaining failure is a monotone post-move drift, not the 36 Hz wobble | sub1-fixedforce, prior session | Supported | [encoder trace](autocal-sub1-fixedforce/sweeps.headless/events.jsonl) |
| The post-move drift is timestep-controlled | sub4-fixedforce | Supported: four substeps complete bootstrap, a fourth sweep and a fit (score 0.5295) where 1 and 2 substeps time out | [sub4 log](autocal-sub4-fixedforce/sweeps.full_auto.log) |
| More solver iterations substitute for substeps at one step | sub1-iter8 | Rejected: same timeout, 2.5x slower | [iter8 log](autocal-sub1-iter8/sweeps.full_auto.log) |
| Damping only on the first visit is sufficient | E15 | Rejected (under-damps) | [E15 decay](experiments/E15-damping-splitting/decay.json) |

## References

- [Full report](report.md): equation order, fix, measurements and limitations.
- [Research notebook](research.md): decisions, budgets, steering and next step.
- Macklin et al., *XPBD*, MIG 2016, DOI 10.1145/2994258.2994272, sec. 5.
- Macklin et al., *Small Steps in Physics Simulation*, SIGGRAPH 2019,
  DOI 10.1145/3345768.3355926, sec. 4.1, 6, 7.
- Müller et al., *Detailed Rigid Body Simulation with XPBD*, SCA 2020, sec. 3.6.
- Prior sessions: [effector rocking](../2026-10-16-hp4-effector-instability/report.md),
  [substep options](../2026-10-16-hp4-substep-options/report.md),
  [one-step autocal](../2026-10-16-hp4-one-step-autocal/report.md).
