# HP4 one-substep autocal wobble: XPBD damping feedback and acceptance status

Status: in progress (fix verified in replay; full autocal acceptance incomplete).

## Abstract

Loaded HP4 autocal wobbled with the default one 2 ms physics step, disturbing
settling and force selection. We froze a production checkpoint, replayed it in
the native Rust solver, and recorded the effector in Rerun. With two XPBD
Gauss-Seidel passes the effector entered a deterministic bounded limit cycle
(6.27 mm late span, 1.79 rad/s). The cause was order-dependent damping:
each joint's velocity-dependent displacement was recomputed from poses already
changed by earlier visits, so damping differentiated its own correction and
injected energy; velocity reconstruction then fed the biased pose into the next
step. Caching each joint's first-visit displacement per substep removed the
cycle at the default one 2 ms step and at explicit two substeps (late span
about 1e-10 to 1e-9 mm, zero angular speed). Exact requested substeps replaced
implicit 1 ms promotion. The full headless default-one-substep autocal still
has not completed: the clean run was externally interrupted at 210,000 steps
after entering sweep collection, and earlier runs hit a now-fixed deadline bug.
This is simulated evidence, not hardware validation.

## Topics

autocalibration, Hangprinter HP4, XPBD, PBD, Gauss-Seidel, cable damping,
effector wobble, substeps, force tuning, headless simulation, Rerun

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| One 2 ms XPBD step with two passes produces a bounded effector limit cycle under load | E01, E02 | Supported | `wobble-baseline.rrd`; `baseline-i1-i2/` |
| Recomputing damping displacement from partially solved poses feeds energy into the cycle | E04, E05 | Supported | `trace-i2.stderr`; `final-cache/` |
| A per-substep first-visit damping cache removes the default one-step wobble | E05 | Supported | `wobble-damping-cache.rrd`; `recheck/` |
| A global beginning-of-substep snapshot is a better damping source | E06 | Rejected | `snapshot/` |
| Implicit promotion to two 1 ms steps is required for stability | E05 | Rejected | `1 x 2 ms`, i2 settles with the cache |
| The full default one-substep autocal completes end-to-end | E08 | Not tested (incomplete) | Last clean run interrupted at 210,000 steps |
| Default-substep force-max selection completes without wobble or timeout | E08 | Not tested (incomplete) | Clean run reused force tuning; earlier failures predate deadline fix |
| The damping-cache idea is novel to this branch | none | Rejected (known upstream) | Upstream `bf9f5399` |

## References

- [Full report](report.md): analysis, measurements and limitations.
- [Research notebook](research.md): experiment decisions and accepted steering.
- [Baseline RRD](wobble-baseline.rrd), SHA-256 `ff27609f93d4c1dbfc23d7af1fc0a15c88a3feffbd82c8cf70f64da90462736c`.
- [Fixed RRD](wobble-damping-cache.rrd), SHA-256 `229663f079530af0fd020a95fbeb3c5f08b2f9548af10d0ccb6a165f7314af9a`.
- [Freeze fixture](../../../../tests/fixtures/hp4_loaded_stability.json), SHA-256 `3d0ce9e30b2eaf35ff315b1102f9a911965b4b96a1625e998399503c7e725341`.
- [Prior HP4 effector-instability session](../2026-10-16-hp4-effector-instability/report.md).
- [Prior HP4 substep-options session](../2026-10-16-hp4-substep-options/report.md).
- [Prior one-step-autocal session](../2026-10-16-hp4-one-step-autocal/report.md).
- [Prior load-bookkeeping session](../2026-10-16-hp4-load-bookkeeping/report.md).
- [Prior roller-propagation session](../2026-10-16-hp4-roller-propagation/report.md).
