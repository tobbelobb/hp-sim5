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
1. The D-optimal candidate score measures local ellipse-parameter information, not
   downstream physical accuracy; whole-sweep held-out prediction may rank sweeps
   better when the current model is biased or weakly identified.
2. The anchor solver's in-sample robust ellipse cost can prefer a physically
   worse anchor/radius fit under spool buildup, elasticity, or bad points.
3. Bounded trial-and-error with a held-out movement gate can prevent regressions,
   but extra collection may erase any runtime/movement benefit.

Constraints/budgets: no hardware movement; preserve browser/firmware collector,
millimetre dataset units and metre native scene units; no new outer loop or
manual-print workflow. First audit recorded data and deterministic synthetic
data; at most two short native physics trials only if a concrete simulator
question remains. Keep failed trials. Make the smallest justified source change.

Experiment IDs:
- E0: source audit and native capabilities; confirmed current D-optimal
  measurement vector is ellipse geometry, with no physical held-out gate.
- E1: `.venv/bin/python autocal/tools/regress_calibration_logs.py --only
  hangprinter_4 --keep-going --color never`; failed overall. On
  `seventh_hp3_dataset`, selected iteration 2 has 257.599 mm anchor error while
  iteration 3 has 156.862 mm; selection regret 100.737 mm. At another adjacent
  pair, rank preferred iteration 3 while true error worsened 5.541 mm. Recorded
  data reproduces the ranking/accuracy disagreement; one dataset's generated
  run also had true-error delta +39.453 mm.
- E2a: deterministic Slideprinter sweep simulation, four training sweeps,
  0.1 mm Gaussian length noise and one +15 mm point outlier; six candidates
  evaluated after D-opt ranking. Top D-opt candidate had 0.0303 mm anchor RMSE,
  the best observed. A different candidate had lower fitted objective (3.030
  versus 3.115) but worse anchor RMSE (0.0343 versus 0.0303 mm); do not equate
  objective reduction with calibration improvement. No D-opt regret in this
  trial (failed to reproduce selector error). In a poorer/ill-conditioned
  trial the top score was 0.353, anchor RMSE 1190.19 mm and fit cost 226.72;
  the best candidate RMSE was still 1188.08 mm (2.11 mm regret), so the run is
  retained as a failure, not a useful calibration.
- E2b: held-out scoring test uses a full 40-point sweep with 0.2 mm Gaussian
  length noise and one +20 mm sensor outlier. Exact model robust prediction cost
  17.768; perturbed model cost 720.534 (40.55x). This validates the residual
  discriminator, not end-to-end anchor improvement.
- E3: one production HP4/RRF collector job (`4b4d1aeb4eed4f3fafe9c45393c02f52`)
  with fixed [2,3], drive 0, sensor 1, five points, 200 mm target, 3 encoder
  samples and return-to-origin. Cancelled at step 25,117 / 50.234 s simulation
  after 411.55 s wall time, before any sweep completed. At step 24,000 the A
  encoder lagged target by 1.13 rad and peak missed steps were 9; Rerun also
  showed cable-length error near -0.4 m. Manifest requires reset. Retained
  `events.jsonl`, `recording.rrd`, and manifest under
  `native/2fa2642a56d04ffebd6443d4227fa540/4b4d1aeb4eed4f3fafe9c45393c02f52/`.
  Reset archived it and restored a fresh zero-step world
  (`1f7f65587ae14612aa31867b951a28e9`).

Implementation: added `autocal/predictive_validation.py`; full-auto history
summary now prefers candidate anchors with lower robust ellipse prediction cost
on later whole sweeps. It records training sweep IDs, logs held-out prediction
cost/count, and falls back to the established rank score when there is no later
sweep. The gate adds no movement. It uses the existing dataset millimetres and
the latest spool-corrected dataset representation when enabled.

E1 rerun after the change also exited 1. Over five recorded datasets its
diagnostic total true-error delta was -45.912 (lower overall), but the fifth
dataset worsened by 0.473 mm and had one score/true direction mismatch; strict
iteration/reference checks also fail. This is stochastic full-auto solver
behavior, and static replay has no future sweep, so the new gate fell back to
the old ranking and did not cause or fix those results.

Validation: 181 relevant autocal tests passed in 3.66 s, including the new
noisy/outlier whole-sweep test, active learning, fit, score UI, spool model,
no-collect and regression-log tests. `compileall` and `git diff --check` passed.
An additional 82 flex, zero-buildup, spool-fit and spool-model tests passed in
1.68 s. The four spool-related tests overlap the first batch.
Accuracy improvement is not yet established: there is no complete independent
movement collection and no end-to-end repeated-noise/elasticity/winding/bad
initial-guess benchmark for the new gate.

Accepted steering: keep D-optimal movement selection; add a whole-sweep
predictive validation score for prior calibration candidates, using later
collected sweeps and a robust sensor-ellipse residual. Prefer validated
candidates at automatic stop, with existing ranking as fallback when no later
sweep exists. This adds no movement and leaves one-click collection and
automatic stopping intact. Treat spool-corrected data consistently and test
noise/outliers, elasticity/winding handling, initial-guess failure, and runtime.
Next decision: report the held-out gate as a measured prediction safeguard,
with no claim of anchor error reduction until a completed independent-sweep
collection can validate it. Diagnose the A-axis missed steps before another
native collection.
