# Covariance metrics for encoder based CDPR autocalibration

## Abstract

This study asks whether covariance metrics from Caverly et al.'s online CDPR self-calibration paper can improve hp-sim5's encoder-based autocalibration ranking. The full recorded regression matrix was replayed after fixing the regression tool's module invocation: all 11 outputs match their stored references exactly, while reproducing 20 adjacent score/physical-error disagreements across nine fixtures. Selected-history anchor-sum regret ranges from 0 to 163.828 mm. A heuristic that weighted the existing ellipse-feature information by point count and inverse encoder variance selected the physically best available history in only 2/11 fixtures and sometimes increased anchor RMSE substantially. Candidate planning itself took 0.039–0.819 s per fit in a 33/90-candidate audit, so it does not explain the tens-of-seconds replay runtime. The paper's PDOP/ODOP depend on joint pose/shared-bias covariance and cannot be transplanted as anchor covariance. No calibration selector change was retained; only the regression launcher was repaired. Evidence is simulated recorded data, not hardware.

## Objective and setup

Read `autocal/README.md`, `autocal/optimization_algorithm_overview`, `autocal/objective_functions_overview`, `autocal/README_elliptical_feature_calibration.md`, `research/README.md`, `research/AGENTS.md` and `research/investigation.md`. Existing code fits anchors from per-sweep ellipse geometry, optionally alternates spool parameters, ranks new sweep configurations using D-optimal gain, and selects final history using common-dataset anchor comparison plus candidate-owned held-out prediction and history rank. The regression fixture declares simulated true anchor/radius values; no hardware metrology is involved.

The paper’s PDOP/ODOP use a covariance for jointly estimated poses at each time step and shared measurement biases. Those metrics approximate translation/orientation error. hp-sim5's current information matrix is instead a finite-difference Jacobian of ellipse features with respect to anchor parameters. The target parameters, observations, latent variables and covariance interpretation differ. Replacing the planner's score with an unscaled pose metric would be unjustified.

## Experiments

### E01 — current-source replay and reproduction

The first requested runner invocation failed before fitting because the harness called `autocal/autocal.py` by path. Python then named the script `autocal`, shadowing the package and raising `ModuleNotFoundError: 'autocal' is not a package`. Changed its subprocess command to `-m autocal.autocal` and reran sequentially with `.venv/bin/python autocal/tools/regress_calibration_logs.py --jobs 1`.

All 11 generated summaries exactly match the stored reference logs at the 0.01 mm tolerance, including iteration counts and selected parameters. The runner reports 20 adjacent score/ground-truth direction mismatches over nine fixtures. Selected-history regret in its diagnostic total parameter metric ranges from 0 to 163.828 mm. Examples:

| Fixture | Score-selected iteration | Physical-best iteration | Selected error sum | Regret |
| --- | ---: | ---: | ---: | ---: |
| ten_points_bigger_deltas | 3 | 1 | 17.529 mm anchor + 0.108 mm radius | 14.901 mm |
| seventh_hp3_dataset | 2 | 3 | 257.599 mm anchor + 0.738 mm radius | 100.737 mm |
| 23_april_2026 | 3 | 6 | 256.983 mm anchor + 0.698 mm radius | 163.828 mm |

The summary confirms exact regression to stored outputs, not calibration correctness; the ground truth is simulated fixture data. The detailed per-run logs remain in ignored `autocal/data/.regress_parallel_runs/` under the current checkout.

### E02/E03 — covariance transfer and heuristic falsification

A frozen-history audit formed per-sweep ellipse information matrices at each recorded candidate, scaled each sweep by point count divided by squared median positive encoder sigma, normalized weights, and selected the candidate with smallest maximum diagonal standard deviation. This is a heuristic diagnostic, not the paper's model: it neither estimates latent poses nor projects nuisance bias parameters.

It chose the physically best candidate in only 2 of 11 fixtures. The selected candidate worsened anchor RMSE relative to the existing history choice on several fixtures: flexible lines 7.492→32.145 mm, ten points bigger deltas 5.930→8.000 mm, fifth HP3 18.382→106.896 mm, and nineteenth HP3 42.130→91.154 mm. This rejects simple reweighting as a replacement selector. It does not reject the paper's covariance-based method under its own joint estimator assumptions.

### E04 — candidate planning cost

Recomputed D-optimal rankings for 33 candidate sweeps on Slideprinter histories and 90 on HP3 histories. Per-fit wall time was 0.039–0.819 seconds. The replay matrix takes tens of seconds per dataset because it runs repeated anchor/spool fits and underconstrained recovery; the candidate sweep ranker is not the main slow component in these cases. The measured candidate timing is a small offline benchmark, not physics throughput.

## Changes and checks

The only retained source change is the module invocation fix in [regress_calibration_logs.py](../../../../autocal/tools/regress_calibration_logs.py). Focused active-learning tests pass: `.venv/bin/python -m pytest autocal/tests/test_active_learning.py -q` (5 passed). The attempted Cholesky log-determinant patch was reverted without evidence. No production calibration objective, selector, score or automatic stopping behavior changed.

## Limitations and next experiment

The current sweep-level feature model does not expose the paper's pointwise latent pose/shared bias covariance. A proper transfer would require a joint estimator or a defensible Schur-complement projection over nuisance variables, explicit gauge/scale treatment, and comparison of predicted physical uncertainty with held-out pose/anchor error. The simple heuristic tested here is insufficient. No simulated collection or hardware test was run because the question was offline selection and current source regression; no accuracy or speed claim for a new calibration algorithm is made.

## References

- Caverly, Ryan J.; Cheah, Sze Kwan; Bunker, Keegan R.; Patel, Samir; Sexton, Niko; Nguyen, Vinh L. (2024; Journal of Mechanisms and Robotics 17(1), 2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://doi.org/10.1115/1.4065236). Its joint pose/shared-bias covariance and PDOP/ODOP formulation motivates E02 and constrains transfer claims. [Local PDF](../../../../autocal/papers/Covariance_based_data_quality_assessment_metrics.pdf).
- [Earlier 11-fixture ranking and persistent-cache study](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): baseline error disagreements, recovery fragility, and a controlled speed improvement unrelated to ranking.
- [Earlier negative ranking/update study](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): held-out prediction and bounded-update counterexamples.
- [Regression harness](../../../../autocal/tools/regress_calibration_logs.py): fixture truth, parameter metrics and ranking audit semantics.
