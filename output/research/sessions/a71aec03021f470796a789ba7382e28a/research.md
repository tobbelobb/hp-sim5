# Research record

Objective: Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can
be slow, fragile, or choose worse anchors because its selection score disagrees
with physical accuracy. First reproduce and measure those cases.

Investigate structured trial-and-error inspired by manual calibration: predict
sensor observations from an approximate model, choose a controlled informative
movement, collect encoder observations, compare with the prediction, make a
bounded calibration adjustment, and validate it with another movement. Preserve
the one-click user experience and automatic stopping; no manual print inspection.

Read the existing fitting, active-learning and candidate-selection code before
brainstorming. Record the leading hypotheses and test the promising ones on
recorded and synthetic data, then use simulated collection when warranted.
Separate anchor/radius error, held-out sensor prediction error, score ranking,
failure rate, movement count and elapsed time. Hold out entire sweeps and test
noise, bad points, elasticity, winding buildup and poor initial estimates.

Use native physics and Rerun when they can test the hypothesis, while respecting
the existing browser/firmware collector boundary and metre/millimetre conversion.
Retain failed experiments. Implement the best supported change, run the relevant
autocal regressions and explain measured improvements and remaining limitations.
Do not call a lower objective value an improvement when the calibration is worse.


Leading hypotheses:
1. Full-auto's iteration and coverage adjustments can make its reported history selection score rank a physically worse fit above another observed fit. Historical reference-log audit already reproduces this on five slideprinter data families; maximum observed regret is 14.901 mm.
2. A training objective evaluated on all points can prefer a calibration that predicts a whole unseen sweep/configuration poorly. Leave-one-sweep-out prediction is a better selector than the training rank, but must be tested for noise, outliers, elasticity and buildup before integration.
3. Existing finite-difference D-optimal sweep selection assumes a local correct model and may choose movements informative only under its approximate anchors; a bounded residual update with independent validation may be more robust, at additional movement cost.

Constraints/budgets: small source diff; no hardware motion; preserve one-click collector and automatic stop; all length geometry in autocal mm (native geometry m); at most one native two-configuration collection and one slideprinter regression pass this turn. Native data is collected through the existing HP4/RRF collector. Research session owns artifacts.

Baseline evidence: historical logs from five slideprinter families, March 11/24 versions, have cases where minimum raw history rank differs from physical best. Repeated ten_points_bigger_deltas runs select iteration 3 at 18.21–18.33 mm total parameter error while physical best has 3.31–7.37 mm error (regret 10.96–14.90 mm). five_points_bigger_deltas regrets 5.58–5.69 mm; layered_manual_some_bad_points and ten_points_w_buildup2 show 1.28–1.55 mm. flexible_lines_2000 and ten_points_w_buildup have zero selection regret in those logs. These are ground-truth parameter metrics from stored simulated runs, not held-out sensor residuals.

Experiment IDs: native collection job `d2e98d4fa0f7491cbaaede24521eecfa` (two HP4/RRF sweep configurations, six points per direction, eight noise samples); regression baseline is running.
Accepted steering: user requested implementation based on evidence, no manual inspection, native physics/Rerun when useful, and complete regression/report.
Next decision: determine whether held-out sweep prediction is already present in logs/data and can safely correct selection without additional collector movement; retain failed trials and add a guard only if accuracy evidence improves.
