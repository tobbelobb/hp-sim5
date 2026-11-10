# HP3 autocalibration under asymmetric friction and sensor noise

Status: report-only research handoff; updated 2026-11-05T10:26:20.544791+00:00.

## Abstract

Nominal defaults and the frozen bounded-preload policy both accepted all four anchors below 5 mm. With verified added guide friction and modest encoder noise, the development default failed before its first point; both development policy versions failed after 20 points. A held-out default failed after 40 points, before fitting. A native diagnostic shows load torque transmitted to a spool with macroscopic local cable slack. The findings expose a boundary of the current guide-friction model and collection workflow; they do not establish a force policy that improves calibration robustness or a hardware failure rate. The posthoc matched zero-guide/noise-only control ended as `accepted_below20` after 140 points and 4278.630 simulated seconds. Its applied maximum anchor error was 4.3495 mm.

## Topics

HP3, autocalibration, capstan friction, encoder noise, bounded preload, native load transfer, simulation validity

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Nominal defaults reach every anchor below 5 mm | nominal-numeric | Supported for one run | [Measurements](evidence/measurements.json) |
| One bounded preload law preserves nominal quality | nominal-consistent-preload | Supported for one run; all anchors below 5 mm | [Frozen policy](policy-frozen.json) |
| Bounded preload recovers the development friction/noise workflow | dev1-active-default, dev1-bounded-preload, dev1-consistent-preload | Not demonstrated: all failed before fitting | [Measurements](evidence/measurements.json) |
| The collection failure is confined to the development world | eval0-default | Rejected within this model: held-out run also failed | [Failure windows](evidence/failure-windows.json) |
| Spool load torque respects local slack under active guide friction | torque-slack-active, torque-slack-zero | Counterexample in a constructed native state; full root cause unresolved | [Active arm](evidence/torque-slack-active.json) |
| Noise alone explains the held-out difference | eval0-noise-only-default | accepted_below20; posthoc diagnostic | [Measurements](evidence/measurements.json) |

## Decision handoff

Retain the evidence and do not advance the preload policy as a robustness improvement. Review frictional-guide transport and spool torque coupling before making broader simulated or hardware claims. No production physics edit, merge, push, or code candidate is delivered. Held-out policy arms and eval1 remain untested.

## References

- [Report](report.md)
- [Notebook](research.md)
- [Declared worlds](design.json)
- [Frozen policy](policy-frozen.json)
- [Review manifest](review-manifest.json)
