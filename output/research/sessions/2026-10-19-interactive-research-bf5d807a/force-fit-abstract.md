# Force-tuning curves: saturation fits and bounded observation time

Status: complete — numerical model comparison; production force selection unchanged.

## Abstract

We investigated whether a simple force/travel fit can choose max force during force tuning, excluding sweep and size behaviors. Rust Native headless HP3 and HP4 trials requested25x speedup. Independent origin-start pulls at six forces ran until a five-second encoder quiet window or180 physical seconds. Identical sequential force schedules were also compared at12 and24 seconds per level. A two-parameter saturating Hill function fits the longer-response curves reasonably: predicted80% travel forces are1.65N for HP3 and4.93N for HP4. However, these are fits to a mixture of settled and finite-time responses, not equilibrium measurements. On sequential HP3 data the same estimate changes from7.84N at12 seconds to4.39N at24 seconds; HP4 short sequences do not identify a plateau. Actual HP3 settling times at4/8/16N were110/58.25/33.25 seconds; five of six HP4 trials remained unsettled at180 seconds. Fit-based selection is promising when saturation is observed, but cannot justify extrapolated force escalation from an approximately linear transient. Long HP4 pulls also produced large line tensions, so travel fit alone cannot establish strain safety.

## Topics

force tuning, max force, HP3, HP4, settling time, force displacement, Hill curve, saturation, finite-time response, Rust Native, momentum.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Force-tuning travel grows exponentially in force | E21 one-fixed, E25, E26 | Rejected as a useful model over tested range; growing exponential has poor held-out prediction | [Model scores](force-curve-fits.json) |
| A simple saturating function fits long-response travel | E25 | Supported numerically, not as equilibrium law | [Fits](force-tuning-fits.png), [raw data](E25-origin-curves/) |
|12-second sequential data uniquely identify an80% plateau force | E21 one-fixed, E26 | Rejected for these trials; duration and model strongly change estimate | [Fits and scores](force-curve-fits.json) |
| Higher force necessarily takes longer to settle | E25 | Rejected: HP3 high-force settling becomes faster | [HP3 summary](E25-origin-curves/summary-hp3.json) |
| Fitted F80 is a safe production max force | None | Not tested; no validation at inferred forces or hardware strain limit | Proposed only |

## References

- [Full report](report.md) and [notebook](research.md): interpretation, accepted steering and budgets.
- [Fit analysis source](fit_force_curves.py) and [scores/parameters](force-curve-fits.json): all fitted models and leave-one-force-out errors.
- [E25 native trials](E25-origin-curves/): raw250ms observations, settings, scene hashes and native artifact identity; archived harness.
- [E26 sequential24s](E26-sequence-24s/): raw observations and archived harness.
- [E21 HP3 force tuning](E21-primitives/hp3-one-fixed.json), [E21 HP4 force tuning](E21-primitives/hp4-one-fixed.json): preceding12-second comparison; other E21 configurations excluded.
- [Previous continuous collector evaluation](continuous-evaluation-report.md): preserved preceding report, a separate question.
- Force-tuning-only harness revision `cce7eddb`; consolidated origin-mode smoke check completed six fresh short native worlds.
- Source base revision `d4a6fd75`; reusable capture and physical-clock revisions `35dd9e79`, `0e044aaa`.
