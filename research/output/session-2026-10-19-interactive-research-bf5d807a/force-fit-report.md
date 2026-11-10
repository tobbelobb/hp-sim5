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

## Simple fit and proposed selection rule

Let x be cable travel and F motor-command force. The simplest reasonable saturating fit in this comparison is

    x(F) = L F² / (K² + F²)
    F80 = 2K

Its inverse is F(x)=K sqrt(x/(L-x)), which diverges as x approaches L. This matches the desired qualitative finite-travel/increasing-force behavior without an exponential term. It is empirical: its small-force response is quadratic, not linear. A linear-at-small-force alternative x=L F/sqrt(K²+F²), rational saturation LF/(K+F), saturating exponential, shifted geometric function, freely fitted Hill exponent, linear, quadratic and growing exponential were also tested. The initial linear expectation belongs to a quasi-static local compliance curve; these finite-time, preload/friction-affected measurements need not have that shape. No tested function is established as a physical law.

| Protocol | HP3 L (mm) | HP3 F80 (N) | HP4 L (mm) | HP4 F80 (N) |
| --- | --- | --- | --- | --- |
| Sequential12s, Hill2 | 994.93 | 7.84 | 547.66 | 15.48 |
| Sequential24s, Hill2 | 979.76 | 4.39 | 836.67 | 13.37 |
| Origin-start up to180s, Hill2 | 957.96 | 1.65 | 1021.66 | 4.93 |

The short HP4 asymptotes are not reliable estimates: they change markedly with duration and model. The freely fitted Hill exponent gives short-window HP4 F80=34.65/25.79N, while linear fits have no plateau. A shifted geometric fit predicts78.24N at12 seconds despite excellent within-range held-out prediction. Good prediction inside the measured range does not constrain a distant asymptote.

For longer trials, a free-exponent Hill fit x=L F^p/(K^p+F^p) improves training RMSE from37.65 to6.64mm on HP3 and12.86 to5.70mm on HP4. Leave-one-force-out RMSE is26.72/11.24mm respectively. Its F80 values are1.33/4.51N. The two-parameter fit's leave-one-out errors are56.47/20.99mm. These are deterministic model-comparison diagnostics, not confidence intervals or repeated-trial variance.

A fixed12-second sequential protocol remains useful for limiting user waiting. A fit-based candidate would collect those fixed windows, accept F80 only when flattening is observed and fitted plateau/F80 remain stable under another point and reasonable model choices, and refuse extrapolation when they do not. Require F80 inside the measured force interval and a separate material/line-load ceiling. These proposed gates and F80 validation trials are NOT TESTED. The present HP4 short-window data fail the flattening/identifiability requirement. Therefore no fitted selector was installed in production in this turn.

The proposed “nonlinear term twice linear term” criterion is also model dependent. A nonnegative fit F=a z+b z³ with z=travel/max-observed-travel gives HP3 a=6.119,b=5.814 and a crossover at26.63N, beyond the12N data. HP4 fits b=0 and has no crossover. This is not a useful observed threshold here. Growing exponentials fail to capture saturation. A power law F^p is exponential when plotted against log(F); curvature on the original log-force axis does not establish super-exponential growth in F.

## Actual encoder settling measurements

Quiet means all motor encoders span less than0.05degrees over a continuous five-second window, sampled every0.25seconds. Times include that detection window. This is encoder settling, not Cartesian settling. Each E25 force starts in a fresh identical native world after a two-second captured hold; low preload0.01N, driveB, fixedD, layering disabled, closed-loop motors, fixed2ms steps. These are the force-tuning configuration only. Different force levels do not inherit prior travel in E25. E21/E26 sequences do inherit prior travel and captured holds.

| Force (N) | HP3 time (s) | HP4 time (s) | HP3 travel (mm) | HP4 travel (mm) |
| --- | --- | --- | --- | --- |
|0.5|>180|149|196.98|23.07|
|1|>180|>180|593.21|136.31|
|2|>180|>180|876.42|396.14|
|4|110|>180|917.52|757.85|
|8|58.25|>180|927.45|939.58|
|16|33.25|>180|931.57|984.03|

The censored entries cannot honestly be replaced with actual times. Longer observation established some real values but did not settle most HP4 trials. Settling time alone is not a reliable cross-machine knee indicator. HP3 force-times at4/8/16N are roughly inverse-force, but that is three measured points, not a validated universal relation.

At8/16N HP4 sampled peak line tensions were58.98/64.30N; sampled maximum whole-path extension ratios0.00923 /0.01070. Command force does not cap supporting-line load. Path extension is not a material strain certification. Body motion persisted after captured holds in these cases. No safety conclusion follows from a travel fit.

## Scope, provenance and limits

All new physics runs used Rust Native and requested25x; actual HP4 throughput was compute limited. No sweeps, size tuning, calibration compute, Python physics backend or RRD collection were run. E25 used at most180physical seconds perforce plus holds; E26 used24seconds perforce. Preflight remained source-stale, so no attached-world movement was used; command-context socket probe passed and all experimental worlds were isolated fresh processes. Initial E25 harness budget omitted the final hold and failed before save; its log is preserved as E25-budget-failure.log, HP4 counterpart interrupted, and the corrected186-second bound was used for completed trials.

No repeats, alternate force spacing, inferred-F80 verification, hardware tests or integration of fit-based selection were performed. Thus the data support the functional form and expose identifiability limits; they do not yet demonstrate a better production force selector. All live jobs are finished. Prior production implementation stays unchanged.
