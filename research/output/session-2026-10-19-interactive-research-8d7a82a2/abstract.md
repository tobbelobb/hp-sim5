# HP4 autocal failures: numerical slowing, missing travel budget and a repeatable four-substep workflow

Status: complete investigation; the underlying dynamics correction remains open.

## Abstract

We investigated recurring HP4 autocal failures using October 16–19 logs and Rerun recordings, native Rust force comparisons, an exact replay and bounded headless calibration trials. Both movers weigh 81 g. At equal configured effective force, one-substep HP3 reaches Y=423 mm at 30 seconds versus HP4 80 mm; four substeps raises HP4 to 335 mm. Motor stabilization introduces timestep-dependent slowing amplified by reeving. The reproduced failure at 515.42 seconds instead expires in a separate 30-second preparation wait while free motors still pull. We extended that transit budget and corrected headless speedup to preserve physical wait durations. A faster motor formula was rejected because calibration fits deteriorated. With original physics and four substeps, 60 bootstrap measurements exactly repeat prior evidence. A fresh-world continuation from 100 measured points collects 40 more and stops automatically, with applied-anchor RMSE 4.02 mm versus nominal geometry. This establishes a controlled continuation workflow, not universal reliability or a completed fresh single-invocation run. All evidence is simulated; hardware accuracy and a complete dynamics repair remain untested.

## Topics

HP4, HP3, Hangprinter, autocalibration, torque mode, numerical damping, reeving, implicit integration, physics substeps, headless speedup, settling, dataset continuation, Rerun

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| HP4 moves substantially less at equal configured effective force | E02, E06 | Supported for controlled pulls | [Motion comparison](motion-comparison.png), [matched forces](E06-matched-forces/) |
| The remaining default timeout is missed fixed targets or rapid wobble | E04, E07 | Rejected for this failure: held motors stationary, free motors pulling | [Exact replay](E07-coast/replay.json), [failure screenshot](failure-viewer.png) |
| Longer size/pullout budgets cover the failing preparation wait | E04, E07 | Rejected: a separate ordinary 30 s wait remains | [Failure log](E04-sub1-clock/sweeps.full_auto.log), [coast check](coast-settling.json) |
| Headless speedup preserved physical collection durations before the fix | E01, E04, E17 | Rejected before the fix; corrected durations supported at 1/10/25 | [Original source](autocal_headless-before.mjs), [clock invariance](E17-clock-invariance.json) |
| Correcting the local steady motor slowing term is sufficient for production | E08–E12, E15 | Rejected: better travel but poor data and degenerate fits | [Rejected patch](rejected-motor-correction.patch), [residual audit](nominal-model-residuals.json), [seeded diagnostic](E15-seeded-fit-diagnostic.json) |
| Extending preparation completes stock one-substep calibration | E16 | Inconclusive: 50 bootstrap points before wall budget; former failure crossed | [Trial log](E16-preparation-budget/sweeps.full_auto.log), [manifest](E16-preparation-budget/sweeps.headless/manifest.json) |
| Stock four-substep data and fits repeat with the prior manual forces | E14 and prior f3c115bf control | Supported for bootstrap and fit; fresh full automatic completion budget-censored | [Exact 60-point repeat](E14-bootstrap-repeat.json), [control log](E14-stock-sub4/sweeps.full_auto.log) |
| A fresh matching world can continue measured four-substep data to automatic acceptance | E19 | Supported for this controlled trial: 100 seed + 40 fresh points | [Acceptance audit](E19-acceptance-audit.json), [fresh-sweep comparison](E19-fresh-continuation-parity.json) |

## References

- [Full report](report.md): equations, measured results, operating recipe and limitations.
- [Research notebook](research.md): steering, compute budgets and chronological decisions.
- [Prior damping investigation, f3c115bf](../2026-10-19-interactive-research-f3c115bf/report.md): the preceding four-substep control and unresolved post-move drift.
- [Prior endpoint investigation, bf5d807a](../2026-10-19-interactive-research-bf5d807a/report.md): rejected local controller and interpretation of logs 008/009.
- [Prior motor load study](../2026-10-16-hp4-roller-propagation/report.md): older stiffness counterfactual and alternative mechanisms.
- [Original reference-005 log](../../../extended-autocal/no-buildup-reference-005/sweeps.full_auto_1.log) and [RRD audit](E03-rrd-summary.json): 120 measurements, four completed collections and final browser-clock stall.
- [Scene complexity](scene-complexity.json), [world configuration audit](world-configuration-audit.json): actual mass, solver work and HP4/Q0/R30/native settings.
- [E19 dataset](E19-clean-continuation/sweeps.json), [seed contract](E19-clean-continuation/seed-contract.json), [run manifest](E19-clean-continuation/sweeps.headless/manifest.json), [finalized event RRD](E19-clean-continuation/sweeps.headless/hangprinter-20261019T215304.204951Z-8626c63b.rrd): provenance and automatic completion evidence.
- [Code provenance](provenance.json), [retained patch](retained-changes.patch), [source snapshot](retained-source/): starting revision 3b3a7fec and the six retained changes.
