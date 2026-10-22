# Encoder CDPR autocalibration investigation — 2026-10-05

No tested calibration or selection change met the acceptance criteria. The final source change adds automatic within-run ranking/ground-truth auditing to the regression tool and labels its aggregate score as diagnostic. The production solver, one-click collector, candidate selection and automatic stopping behavior are unchanged. Failed candidate patches, datasets, traces and measurements are retained in this session.

This is a negative research result, not a claim that a new calibration algorithm works. The investigation reproduced worse-anchor selection, tested residual validation and bounded updates, and found cases where they still make physically worse choices. Online collection was attempted but could not be validated in this sandbox.

## Inputs, provenance and measurement contract

Starting repository revision: `aaf25cf78e2e84e8c3291d8ac00673ecb7970506`. At report verification, the checkout was clean at `c5024c94c55a1b1daeed0716e72057d371af90f8`; the differences from the starting revision are exactly the three final diagnostic/doc/test files. All Python commands used `.venv/bin/python`. I read `research/AGENTS.md`, `research/README.md`, `research/investigation.md`, the autocal README, active-learning/planning code, point-role normalization, fitting/alternating refinement, spool models, candidate selection and regression tooling before choosing trials. The first hp-sim5 MCP call was `capabilities`.

[Experiment registry](experiment_registry.json) identifies the recorded replay experiments and their input, text log and JSONL paths. [Artifact hashes](artifact_hashes.json) freeze datasets, scripts and logs. [Final source diff](final_source.diff) contains the shipped change; [rejected age-bonus patch](no_age/candidate.diff) is retained separately. Native experiments have their own immutable manifests and source/input hashes.

Autocal lengths and errors below are millimetres; raw angles are degrees. Native scene/telemetry lengths are metres and motor commands are radians. Anchor error is the sum of Euclidean anchor distances to the regression fixture's ground truth. Radius error is the sum of absolute radius errors. The existing regression total is `anchor_sum + 2π * radius_sum`; that composite never replaces the two component measurements.

Recorded ground truth is the existing simulation fixture geometry/radius in `regress_calibration_logs.py`, not independent hardware metrology. Logged parameters are rounded, so derived errors are approximate. Entire suffix sweeps were held out; no points from those sweeps entered the candidate fit. The synthetic holdout uses three entirely different fixed-length configurations. Recorded suffix sweeps can share a configuration family with training sweeps; these are sweep holdouts, not a guarantee of new configuration families.

The sensor audit solves the predicted conic at the observed drive length, converts predicted material sensor length through the fitted spool model back to encoder degrees, and uses the fixed base-model mm/degree conversion for comparison. Candidate radius cannot improve that metric merely by shrinking its unit conversion. Two metrics are recorded: distance to the nearest predicted branch (an optimistic prediction-set residual), and an explicit forecast selecting the branch closest to the model's origin length without looking at the held-out sensor. In these recorded fixtures the two agree for the score-selected candidates. Both roots often exist; branch ambiguity is counted rather than hidden. Invalid predictions incur a 10,000 mm diagnostic penalty and are included in the failure-rate denominator. The conic residual is also recorded separately; it is an approximate geometric residual, not sensor prediction error.

Elapsed replay times are JSONL start-to-final-iteration wall time. Offline prototype times include their evaluation overhead. Parallel regression times include contention. Native MCP times include numeric sampling and Rerun. None of these is a physics-throughput benchmark.

## Hypotheses and decisions

1. **Ranking already disagrees with physical accuracy.** Audit recorded histories against known anchors/radii, then rerun the current implementation. Confirmed.
2. **Whole-sweep encoder validation can repair ranking.** Rescore frozen candidates on the same untouched final two sweeps. Rejected as a general selector: lower sensor residual sometimes selects much worse geometry.
3. **A fitting-objective guard can prevent rank-driven harmful updates.** Add only a non-deterioration guard to the existing rank comparator, with all other replay settings fixed. Rejected: the final result barely changed and the first fit became much worse.
4. **Removing the iteration-age bonus prevents false progress.** Change only the history age adjustment, then compare both a single dataset and the full matrix. Rejected: the small-case gain did not survive the full matrix and even that case traded worse radius/prediction error for better anchors.
5. **Joint encoder residual fitting with bounded adjustments and a fresh validation sweep is faster and more stable.** Explore a robust joint anchor/global-radius least-squares prototype. Separate update caps from validation acceptance in synthetic ablations. Rejected for production: some clean synthetic cases work, but noise, outliers, bad initial estimates and recorded 3D data expose failures.
6. **Informative movement selection repairs those failures.** Compare a fixed movement order with the existing D-optimal ranking while keeping the bounded fitting and validation policy fixed. Mixed results; rejected for production.
7. **Good motor tracking is sufficient to ignore radius uncertainty.** Native sensitivity experiment with only the radius-to-angle conversion changed. Rejected: final effector deviation exceeded the criterion while peak tracking/force metrics were unchanged.

Acceptance required evidence on selected anchor/radius accuracy, held-out prediction, failures, movement effort and elapsed time, followed by collector/regression evidence. A lower selection score, a successful process, or an attractive screenshot was insufficient. I did not keep a solver patch that failed these checks.

## Reproduction of the existing failures

The archived histories contain 22 adjacent score/physical-direction disagreements across 7 of 11 datasets. Six histories select a physically worse result than another available candidate. The audit below is from the actual archived candidate parameters and scores, not a fitted explanation of them.

| Dataset | Score-selected iteration / total error | Physical-best iteration / total error | Adjacent disagreements |
| --- | --- | --- | --- |
| five_points_bigger_deltas | 4 / 11.805 mm | 3 / 6.227 mm | 4 |
| flexible_lines_2000 | 3 / 23.393 mm | 3 / 23.393 mm | 0 |
| layered_manual_some_bad_points | 3 / 3.585 mm | 3 / 3.585 mm | 2 |
| ten_points_bigger_deltas | 3 / 18.207 mm | 1 / 3.306 mm | 4 |
| ten_points_w_buildup | 3 / 4.645 mm | 3 / 4.645 mm | 0 |
| ten_points_w_buildup2 | 6 / 5.411 mm | 3 / 4.130 mm | 1 |
| fifth_hp3_dataset | 3 / 70.314 mm | 3 / 70.314 mm | 0 |
| seventh_hp3_dataset | 5 / 222.783 mm | 7 / 124.349 mm | 5 |
| sixteenth_hp3_dataset_even_more_pressure | 3 / 52.008 mm | 5 / 41.919 mm | 2 |
| nineteenth_hp3_dataset_even_more_pressure | 2 / 169.642 mm | 2 / 169.642 mm | 0 |
| 23_april_2026 | 3 / 261.369 mm | 6 / 97.541 mm | 4 |

The clearest small reproduction is `ten_points_bigger_deltas`: history ranking chooses iteration 3 with 17.529 mm anchor error and 0.108 mm radius error, total 18.207 mm. Iteration 1 has 1.534 mm anchor error and 0.282 mm radius error, total 3.306 mm. This demonstrates both the ranking problem and the need to report anchor/radius tradeoffs separately. `23_april_2026` selects total error 261.369 mm despite a later candidate at 97.541 mm.

[Archived ranking measurements](recorded_ranking_audit.json), [whole-sweep prediction audit](heldout_audit.json), and [ranking/prediction plot](ranking_and_prediction.png) provide every candidate's parameters, held-out IDs and measurements. Frozen suffix validation selected worse physical results in 5 of 11 datasets, equal results in 4, and better results in 2. For example, the fifth HP3 dataset's validation selector prefers total error 381.810 mm over the score selector's 70.314 mm, because its held-out sensor residual falls from 12.393 to 4.170 base-encoder mm. This is a failed trial, not a success.

## Controlled replay trials

All five runs below replay the same original dataset, initially three sweeps, with the same collector boundary and default solver/filter settings unless named in the changed-variable column. These are offline replay runs: no physical movement occurs. The point count is the prefix of recorded encoder observations consumed, a lower bound on measurement movements an online strategy would need; it excludes positioning, force tuning, homing and return travel.

| Experiment ID / log | Changed variable | Anchor sum mm | Radius sum mm | Total mm | Iterations / encoder points | Wall s |
| --- | --- | --- | --- | --- | --- | --- |
| [ten_points_baseline](baseline/ten_points_bigger_deltas.full_auto.log) | None | 17.529 | 0.108 | 18.207 | 6 / 160 | 43.24 |
| [ten_points_objective_guard](objective_guard/ten_points_bigger_deltas.full_auto.log) | Require non-worsening total fitting objective | 17.575 | 0.108 | 18.254 | 6 / 160 | 43.24 |
| [ten_points_trf](trf/ten_points_bigger_deltas.full_auto.log) | Anchor optimizer: trf | 17.529 | 0.108 | 18.207 | 6 / 160 | 45.72 |
| [ten_points_reconstruction](reconstruction/ten_points_bigger_deltas.full_auto.log) | Anchor proposal objective: 2 | 18.352 | 0.138 | 19.219 | 6 / 160 | 111.18 |
| [ten_points_no_age](no_age/ten_points_bigger_deltas.full_auto.log) | History iteration-age adjustment: zero | 1.534 | 0.282 | 3.306 | 4 / 120 | 26.07 |

The objective guard's first fit deteriorated from 3.306 to 57.511 mm total error. Its final result also worsened slightly. `trf` produced the same selected geometry and took slightly longer. Objective 2 produced worse geometry and much more wall time; its position-reconstruction path requires at least four anchors, so the Slideprinter run is also evidence against treating it as a universal replacement.

The no-age trial stops after four rather than six iterations, but radius error increases from 0.108 to 0.282 mm, and untouched-sweep sensor residual increases from 1.044 to 1.135 base-encoder mm. The later full matrix makes the rejection decisive.

## Fresh baseline versus no-age regression matrix

Both phases ran the actual full-auto regression commands for all 11 datasets, with the same initial datasets and settings. This fresh paired comparison separates candidate effects from discrepancies already present against archived reference logs. Logs and replay inputs were copied into the session; IDs below are the actual regression worker directories.

| Dataset / baseline log / trial log | Baseline total mm | No-age total mm | Change mm | Iterations baseline → trial |
| --- | --- | --- | --- | --- |
| ten_points_bigger_deltas [ten_points_bigger_deltas_342beccb](regression_runs/baseline/ten_points_bigger_deltas/ten_points_bigger_deltas.full_auto.log) [ten_points_bigger_deltas_c4d74e0c](regression_runs/no_age/ten_points_bigger_deltas/ten_points_bigger_deltas.full_auto.log) | 18.207 | 3.306 | -14.901 | 6 → 4 |
| flexible_lines_2000 [flexible_lines_2000_e8d7c9ef](regression_runs/baseline/flexible_lines_2000/flexible_lines_2000.full_auto.log) [flexible_lines_2000_29ee6459](regression_runs/no_age/flexible_lines_2000/flexible_lines_2000.full_auto.log) | 23.393 | 23.393 | +0.000 | 4 → 4 |
| ten_points_w_buildup2 [ten_points_w_buildup2_a6045ae8](regression_runs/baseline/ten_points_w_buildup2/ten_points_w_buildup2.full_auto.log) [ten_points_w_buildup2_96a5bc07](regression_runs/no_age/ten_points_w_buildup2/ten_points_w_buildup2.full_auto.log) | 5.411 | 9.545 | +4.134 | 7 → 5 |
| ten_points_w_buildup [ten_points_w_buildup_1c42adc4](regression_runs/baseline/ten_points_w_buildup/ten_points_w_buildup.full_auto.log) [ten_points_w_buildup_eca39319](regression_runs/no_age/ten_points_w_buildup/ten_points_w_buildup.full_auto.log) | 4.645 | 4.645 | +0.000 | 5 → 5 |
| nineteenth_hp3_dataset_even_more_pressure [nineteenth_hp3_dataset_even_more_pressure_78a0677e](regression_runs/baseline/nineteenth_hp3_dataset_even_more_pressure/nineteenth_hp3_dataset_even_more_pressure.full_auto.log) [nineteenth_hp3_dataset_even_more_pressure_e36ac2a9](regression_runs/no_age/nineteenth_hp3_dataset_even_more_pressure/nineteenth_hp3_dataset_even_more_pressure.full_auto.log) | 169.642 | 169.642 | +0.000 | 5 → 5 |
| five_points_bigger_deltas [five_points_bigger_deltas_8f6e4a75](regression_runs/baseline/five_points_bigger_deltas/five_points_bigger_deltas.full_auto.log) [five_points_bigger_deltas_53b5f74a](regression_runs/no_age/five_points_bigger_deltas/five_points_bigger_deltas.full_auto.log) | 11.805 | 11.805 | +0.000 | 6 → 6 |
| seventh_hp3_dataset [seventh_hp3_dataset_87b058be](regression_runs/baseline/seventh_hp3_dataset/seventh_hp3_dataset.full_auto.log) [seventh_hp3_dataset_13d4cbaa](regression_runs/no_age/seventh_hp3_dataset/seventh_hp3_dataset.full_auto.log) | 262.236 | 262.236 | +0.000 | 5 → 5 |
| sixteenth_hp3_dataset_even_more_pressure [sixteenth_hp3_dataset_even_more_pressure_653d15a9](regression_runs/baseline/sixteenth_hp3_dataset_even_more_pressure/sixteenth_hp3_dataset_even_more_pressure.full_auto.log) [sixteenth_hp3_dataset_even_more_pressure_51ec595a](regression_runs/no_age/sixteenth_hp3_dataset_even_more_pressure/sixteenth_hp3_dataset_even_more_pressure.full_auto.log) | 52.008 | 52.008 | +0.000 | 6 → 6 |
| 23_april_2026 [23_april_2026_79a1572e](regression_runs/baseline/23_april_2026/23_april_2026.full_auto.log) [23_april_2026_0ece6d11](regression_runs/no_age/23_april_2026/23_april_2026.full_auto.log) | 261.369 | 261.369 | +0.000 | 6 → 6 |
| fifth_hp3_dataset [fifth_hp3_dataset_faa71a18](regression_runs/baseline/fifth_hp3_dataset/fifth_hp3_dataset.full_auto.log) [fifth_hp3_dataset_a65480e3](regression_runs/no_age/fifth_hp3_dataset/fifth_hp3_dataset.full_auto.log) | 70.314 | 275.508 | +205.194 | 7 → 6 |
| layered_manual_some_bad_points [layered_manual_some_bad_points_41b719fd](regression_runs/baseline/layered_manual_some_bad_points/layered_manual_some_bad_points.full_auto.log) [layered_manual_some_bad_points_09e70f42](regression_runs/no_age/layered_manual_some_bad_points/layered_manual_some_bad_points.full_auto.log) | 6.129 | 12.403 | +6.274 | 7 → 7 |

The trial improves one selected result, worsens three, and leaves seven unchanged; aggregate selected total error increases by 200.701 mm. Fifth HP3 alone worsens by 205.194 mm. This is why the age-bonus patch was reverted.

[Baseline regression output](baseline_regressions.txt) and [rejected trial output](regressions.txt) both exit with failure against the archived references. The restored baseline already differs on `seventh_hp3_dataset` and `layered_manual_some_bad_points`, and some intermediate 3D fits differ even when the final selection agrees. No reference logs were rewritten to hide this. The trial's aggregate `final_score` was negative while its selected parameter errors were worse; the shipped tool now explicitly labels that composite diagnostic, never an acceptance criterion. [Paired measurements](paired_regressions.json) include separate anchor/radius errors, elapsed time and consumed point counts.

## Synthetic bounded trial-and-error and active movement trials

[Generator and update pilot](synthetic_trials.py), [all 48 ablation results](synthetic_results.json), and the per-case `synthetic/<case>_seed_<seed>/` inputs/traces are retained. Two seeds (7 and 19) are exploratory, not a robustness proof. Each dataset has 12 sweeps: three initial zero-delta constraints, six available nonzero-delta sweeps, and three untouched -150 mm holdout configurations. Each sweep has 16 encoder observation movements. Each pilot consumes 144 calibration point movements; 48 additional points are held out for evaluation.

The matched-model baseline uses known nonzero winding buildup 0.636619, true radius 39.184 mm, and the repository's nominal 2D anchors. Noise is 0.02 mm-equivalent except the noise condition (0.4 mm). Bad-point trials inject 20 mm-equivalent sensor errors at two of 16 points per sweep. Elasticity uses S=2000 with known 5 N drive and 2 N sensor tensions. The poor-initial condition starts anchors at 65% scale and radius 30 mm. The zero-buildup condition removes the absolute-scale cue. The generator matches the current fit's origin/material-length convention; unknown origin pretension, fixed-cable stretch, hysteresis and incorrect elasticity parameters remain limitations.

The pilot uses joint anchor/global-radius `least_squares`, soft-L1 with fixed 0.5 base-encoder-mm scale, 50 evaluation budget per round, and three rounds. Parameter caps are 150 mm per anchor optimization coordinate and 3 mm radius per round. Separate modes test only caps, only validation, and both. Current-round validation uses an entire sweep excluded from that proposal fit; later rounds may incorporate previously observed validation sweeps. Final holdout sweeps never enter fitting or acceptance. The earlier, confounded combined pilot is preserved under [pilot_v1](pilot_v1/synthetic_results.json); it was revised to isolate the variables.

Mean measurements over the two seeds follow; each cell is anchor sum / radius sum / optimistic unseen sensor residual, all in mm.

| Condition | unbounded | bounded_only | validate_only | bounded_validate |
| --- | --- | --- | --- | --- |
| clean | 2.346 / 0.062 / 0.028 | 2.346 / 0.062 / 0.028 | 2.346 / 0.062 / 0.028 | 2.346 / 0.062 / 0.028 |
| noise | 48.926 / 1.293 / 0.557 | 48.926 / 1.293 / 0.557 | 48.926 / 1.293 / 0.557 | 64.009 / 1.801 / 0.618 |
| outliers | 32.789 / 0.690 / 5.394 | 32.789 / 0.690 / 5.394 | 342.000 / 3.552 / 5.442 | 342.000 / 3.552 / 5.442 |
| elasticity | 2.004 / 0.050 / 0.027 | 2.004 / 0.050 / 0.027 | 2.514 / 0.067 / 0.029 | 2.514 / 0.067 / 0.029 |
| zero_buildup | 335.185 / 6.914 / 0.026 | 300.182 / 6.192 / 0.026 | 330.607 / 6.818 / 0.028 | 299.206 / 6.170 / 0.028 |
| poor_initial | 2.346 / 0.062 / 0.028 | 696.335 / 20.763 / 0.970 | 2.346 / 0.062 / 0.028 | 696.335 / 20.763 / 0.970 |

Validation rejects useful updates in the outlier condition: anchor error rises from about 32.8 to 342 mm. Hard caps prevent recovery from poor initial estimates within the movement budget: about 696 mm anchor error remains, versus about 2.35 mm for the uncapped pilot. In zero-buildup trials, sensor residual stays around 0.03 mm while anchor error exceeds 300 mm. With zero buildup, geometry and radius can scale together without changing encoder constraints; another movement cannot recover missing absolute-scale information by itself.

Numeric/domain failures are zero in these synthetic runs, so process success is misleading. As an explicitly exploratory diagnostic, an anchor-sum budget of 10 mm and radius-sum budget of 0.2 mm gives physical failures in 6/12 uncapped runs and 8/12 capped runs. Those thresholds were used for reporting, not tuned into acceptance. Runtime and point counts are separate fields in the JSON.

[Explicit predict → choose → observe → bounded adjust → independent validate trial](active_trials.py) implements the requested sequence using the existing D-optimal planner. Observations from the chosen informative sweep enter the adjustment; the next entire sweep validates it. Candidate fixed lengths are transformed through the current spool model before information ranking. [Final active results](active_results.json) record informative sweep IDs, encoder residuals, validation IDs, acceptance decisions, physical error, movement count and runtime. The only variable between each pair is fixed versus D-optimal ordering. Earlier pilot versions are also retained.

| Condition / seed | Fixed: anchor / sensor mm | D-optimal: anchor / sensor mm | Movement points each |
| --- | --- | --- | --- |
| noise / 7 | 29.999 / 0.462 | 16.377 / 0.457 | 144 |
| noise / 19 | 20.845 / 0.661 | 21.506 / 0.674 | 144 |
| outliers / 7 | 342.000 / 5.438 | 342.000 / 5.438 | 144 |
| outliers / 19 | 342.000 / 5.446 | 342.000 / 5.446 | 144 |
| poor_initial / 7 | 671.087 / 1.161 | 704.522 / 1.665 | 144 |
| poor_initial / 19 | 671.382 / 1.149 | 704.972 / 1.650 | 144 |

D-optimal ordering helps one noise seed, slightly worsens the other, and worsens both poor-initial trials. The planner's information matrix models anchor ellipse geometry, not the joint anchor/radius/buildup uncertainty. These experiments do not support replacing the current algorithm with this bounded loop.

The additional [recorded joint-fitting prototype](recorded_joint.py) is an exploratory algorithm comparison, not a one-variable causal test of the production pipeline. It jointly optimizes anchors/global radius, uses raw angles and fixed robust scale, and fits prefixes with the same final two sweeps withheld. [Results](recorded_joint_results.json) include every fitted candidate and measurement. It is fast but materially worse on all 11 score-selected comparisons; several 3D anchor errors exceed metres. This candidate was also rejected. Synthetic matched-model convergence cannot establish recorded-data or collector success.

## Native physical sensitivity and Rerun inspection

Reference experiment [daedb17f23f24acbaa0948b78442da04](../../experiments/daedb17f23f24acbaa0948b78442da04/manifest.json) and trial [fb2bebb1aa38415a9aff3b067e5688d5](../../experiments/fb2bebb1aa38415a9aff3b067e5688d5/manifest.json) use the same fresh `hp4_rigid_body.usda` scene, 300 steps, dt=0.002 s. The first 100 records ramp absolute A targets; 200 remaining steps hold. Reference final A=0.010000 rad; a hypothetical radius estimate 35% too small gives trial A=0.015384615 rad. Geometry is unchanged. This is a radius-conversion sensitivity test, not an autocal sweep or a validation of one of the candidate fits.

The criterion was final effector deviation <0.05 mm from the reference and zero missed steps. `compare_experiments` discloses only `commands_sha256` as changed. Actual step-300 samples were read with `read_experiment`; [numeric comparison and samples](native_comparison.json) are retained. Final positions differ by **0.0852663 mm**, so the trial fails. Both runs have zero missed steps and identical peak force (0.0979632 N), peak motor tracking error (0.000243262 rad), and peak cable-length error (0.0000114333 m). Good tracking and equal peak metrics do not imply equal physical motion. Some lateral cables are slack in the final samples; this small native test is not a realistic tension-controlled calibration sweep.

Both RRDs were opened in the launcher's assigned Viewer, the reference was closed before opening the replacement, and both were inspected at `sim_step=300`. The blueprint contains the 3D machine and line-length/error/force, motor and encoder views. No Viewer warnings/errors were reported. [Viewer state/logs](viewer_inspection.json), [reference screenshot](native_reference_step300.png), and [trial screenshot](native_underestimate_step300.png) record that inspection. The displacement is too small to establish visually; the conclusion comes from telemetry. Native timing is not used for a physics performance claim.

## Final implementation and checks

The source change is deliberately confined to the regression tool, its tests and four README lines:

- `history_ranking_audit` reports adjacent score/ground-truth disagreements, score-selected and physically best iteration, selected anchor and radius errors, and selection regret. It works even when generated and reference logs are identical. Missing/non-finite measurements and mixed raw/history score scales are reported as unavailable; gaps are not called adjacent.
- The existing aggregate `final_score` is labelled diagnostic; acceptance still uses the existing parameter/mismatch checks. The within-run audit is diagnostic and does not silently change solver behavior or regression exit policy.
- Tests cover worse geometry hidden inside an unchanged run, missing measurements, and incompatible rank scales. No unvalidated solver, selection or UI change remains.

[Final Python suite](tests_final.txt): **290 passed**. [Focused audit checks](audit_tests_final.txt): **24 passed**. `git diff --check` passed. The production solver files exactly match the starting revision.

[Collector Jest checks](collector_jest_tests_final.txt): 46/47 tests passed; the remaining test needs a local HTTP server and failed with `listen EPERM ... 127.0.0.1`. This is not a passing end-to-end collector check. The actual existing command `.venv/bin/python autocal/autocal.py --sim --machine-type slideprinter --speedup 25 --max-steps 1 --dataset <session>/online/dataset.json` was attempted; [output](online_attempt.txt) records firmware readiness failure before collection. [Vite output](vite.log) records the explicit denial of bind to 127.0.0.1:5173. Playwright setup was attempted but no successful browser collection occurred. No hardware movement was requested or performed.

## Remaining limits and next evidence required

The requested general calibration improvement remains unproven. This session improves automatic diagnosis, not the robot's calibration accuracy. The existing bad selections remain reproducible. No candidate was promoted simply because it was fast, had a lower score, or fit a matched synthetic model.

A further solver trial should model joint anchor/radius/buildup identifiability and distinguish residual bias from noise. Absolute scale needs an independent constraint when radius and winding scale are both free or winding is absent. Adjustment bounds should depend on uncertainty and initialization distance; the fixed caps tested here are inadequate. Validation needs a noise/outlier-aware decision that cannot reject every useful update, plus a physically justified prediction branch.

Before changing the one-click default, obtain successful browser/firmware `--sim` collection, repeat across more seeds/configurations, hold out complete new sweep families, include incorrect elasticity/origin tension and winding estimates, and compare selected geometry, radius, prediction, false-success rate, collection movements and elapsed time separately. Preserve existing automatic termination and rollback. Native motor samples must not be relabelled as collector sweeps; that requires the missing adapter and parity evidence.
