# Investigation of the fifth_hp3_dataset reference

Investigated 2026-10-07 at HEAD `4daac40c77276c14d62c86db51abfc2a40292998`.
All experiments used `.venv/bin/python`, recorded data and `--no-collect`.
No production source, dataset or reference was changed.

The March code still returns the reference's final calibration when replayed
today. It also produces today's worse intermediate trajectory. The large
390.788 penalty does not identify an abandoned, superior calibration algorithm.

## What the regression row measures

The active reference is
`autocal/data/references/fifth_hp3_dataset.full_auto_reference_run_march_23_2026.log`.
The runner selects the latest dated reference for each dataset independently;
the battery is not a snapshot of one historical commit.

Ground-truth parameter error is the sum of four Euclidean anchor errors plus
`2*pi*sum(abs(radius - true_radius))`. It is a parameter metric, not an
end-effector positioning error. The true radius is 39.1845 mm.

In the reported row:

| Field | Meaning |
| --- | --- |
| 0.473 | Returned calibration error minus reference returned calibration error |
| 390.788 | Mean of paired intermediate-attempt error differences |
| 3641.275 | Population standard deviation of current intermediate-attempt errors |
| 7 | Number of current intermediate attempts with parseable parameters |

`compute_true_iter_mean_delta` checks for finite parameters, but does not exclude
underconstrained sentinel results. Thus a failed solve contributes its anchor
coordinates even though full-auto excludes that result from final selection.

Using the rounded totals in the question, fifth contributes about 391.261 to
the composite score. The other datasets contribute about -35.800. However,
their returned-calibration errors contribute +41.524: their composite score
improves while their summed selected-parameter error worsens. The final-score
label already calls this metric diagnostic only.

## Reference and current trajectories

Errors below are total parameter errors in mm, parsed with the regression
runner's own functions. An asterisk identifies a cost-100, rank-50 sentinel
attempt, excluded from final calibration candidates.

| Iteration / sweeps available | Saved March 23 reference | Current replay | Current minus reference |
| --- | ---: | ---: | ---: |
| 1 / 3 | 158.564 | 275.508 | +116.944 |
| 2 / 4 | 70.787 | 70.787 | 0.000 |
| 3 / 5 | 70.314 | 70.314 | 0.000 |
| 4 / 6 | 10504.746* | 10504.746* | 0.000 |
| 5 / 7 | 381.810 | 3790.717* | +3408.907 |
| 6 / 8 | 5906.690* | 371.980 | -5534.711 |
| 7 / 9 | 368.704 | 5113.083* | +4744.379 |
| Returned calibration | 70.314 | 70.787 | +0.473 |

The reference's intermediate mean was 2494.516 mm; today's is 2885.305 mm.
Their medians are 368.704 and 371.980 mm. The large mean difference is dominated
by a change in failed optimization basins. Failure count also increased from
two to three, so there is a real stability concern; the mean is simply a poor
standalone measure of the final calibration.

An additional historical example makes that distinction concrete:
`autocal/data/fifth_hp3_dataset.full_auto_94.log` returns 32.721 mm parameter
error, better than the reference, but its intermediate mean delta is +526.616.
It uses per-anchor radius fitting rather than the battery's global mode, so it
is evidence of a different configuration, not a controlled code comparison.

## Why the final reference estimate was good

The reference replay started with three sweeps and added six recorded sweeps.
It selected iteration 3, rather than the final nine-sweep solve. Iteration 3's
anchors and radii are still produced today, to the precision of the logs.

Iteration 2 adds `sweep_004`, which changes the fixed D-axis length by about
-241.6 mm; the earlier three sweeps held D near its origin. Its total parameter
error falls to 70.787 mm. At iteration 3, all 20 points in `sweep_005` are
rejected, leaving 80 of 100 observations. The result is essentially a refit of
the same four accepted sweeps, with error 70.314 mm.

The historical selection score favors iteration 3 (4.431) over iteration 2
(4.473). Their raw rank scores both round to 4.683; iteration and coverage
adjustments determine their ordering. This distinction remains in today's
code before the new predictive selector is applied.

The reference returns radius 39 mm for all four spools, at the imposed lower
bound of `--r0-bounds 39,40`. Radius error contributes 4.637 mm to the total;
anchor error contributes 65.677 mm. The D-anchor contributes about 30.66 mm
of that anchor error. This was a useful estimate, not a near-exact calibration.

Its logs also report `info rank: 10/12`, `chi2_red=96.86` and covariance-scaled
`max_std=9830mm`. These diagnostics do not establish strong confidence merely
because the UI labels its fit quality usable. Later accepted historical fits
show sweep biases spanning several mm; other fits lose enough valid sweeps to
hit the fixed cost-100 sentinel.

## Commit history and recorded intent

| Commit | Message / relevant change |
| --- | --- |
| `51716744`, March 19 | “Includes 3d autocal in regression test.” Introduces the older March 19 reference, using base radius 39. Its returned parameter error is 158.564 mm. |
| `d6fc78ab`, March 19 | “Restored the pre-df9eeb15 layered scoring behavior.” |
| `4617795c`, March 19 | “Uses normalized_sweep_bias penalty to improve 3d autocal.” Initially adds the signal to both ranking and the alternating loop. |
| `5b2e0c02`, March 19 | “Only use normalized_sweep_bias penalty in rank/score.” Removes its influence from the inner solve. |
| `57cc3b8f`, March 19 | “Increases _SCORE_UI_LAYERED_SWEEP_BIAS_WEIGHT from 0.5 to 1.5.” The 1.5 weight is still present today. |
| `f193251c`, March 20 | “The special base radii for fifth_hp3_dataset was just wrong.” Changes its regression argument from 39 to 30, matching the dataset's collection configuration. This is the base conversion radius; fitted radius remains constrained to 39–40. |
| `849eca71`, March 20 | “Retry after unconstrained sentinel.” Adds recovery and warm-start handling. |
| `0a1bbdd8`, March 20 | “Don't short circuit the whole full auto after unconstrained sentinel.” Continues the loop after recovery instead of immediately accepting it. |
| `df699275`, March 23 | “Adds two reference runs to the autocal regression test.” Also adds the fifth dataset's new reference. Its preserved JSONL records a start at 10:26:22 local time. |
| `84930240`, March 23 | “Automate reading the latest reference log. Don't hard code it.” Renames the fixture to its current name and introduces automatic reference discovery. |
| `a71e7c7d`, March 24 | “Refreshes regression logs.” Refreshes six Slideprinter references, leaving fifth's March 23 reference intact. |
| `1404ce4f`, March 26 | “Manages Python dependencies.” Introduces root requirements, optional CPU JAX requirements and a documented venv setup. This is a plausible environment-change boundary, not a demonstrated cause of the numerical difference. |
| `4cfcdc7d`, June 4 | Adds optional TRF. Message reports lower variance and about 25% faster initial regression runs, explicitly retaining L-BFGS-B as default. |
| `eb6df36e`, June 4 | Adds optional LM. Message says it struggles with 3D speed and correctness; L-BFGS-B remains default. |
| `37c46c09`, September 10 | Adds optional sparse recovery and changes recovery geometry checks. The user's command does not enable sparse recovery. |
| `4daac40c`, October 7 | “Luna's maybe improvement of autocal?” Changes final history selection using future-sweep prediction. It does not change these seven fitting results. |

I found no commit message saying that the March calibration was intentionally
abandoned. The default algorithm retains its core mechanisms. In particular,
`active_learning.py`, `alternating_refinement.py`, `planning_pass.py`,
`theoretical_ellipse.py`, `spool_model.py`, `calibrate.py`, `dataset_roles.py`
and the reference dataset are byte-identical between `84930240` and HEAD.

## Controlled replay

I exported isolated copies of `autocal/` at `df699275` and HEAD, then replayed
the same reference dataset with the recorded arguments. `df699275` introduces
fixtures and regression-runner changes, so its fitting code matches the tracked
state immediately before the fixture was committed.

Both processes used the same `.venv/bin/python`, with NumPy 2.4.3, SciPy 1.17.1,
JAX/JAXlib 0.9.2, `PYTHONHASHSEED=0` and numerical thread limits of 1. Both exited
successfully. The March replay's resolved settings exactly match the preserved
historical JSONL settings. Current settings differ only in the equivalent
optimizer spelling `lbfgsb` versus `L-BFGS-B`.

The two fresh replays produced identical logged intermediate anchors, radii
and full-precision JSONL primary costs in every iteration. Both reproduce
`true_iter_mean_delta=390.78844503693796` and
`true_gen_std=3641.2747606996` against the saved reference.

| Run | Selected iteration | Returned error | Mean delta against saved reference |
| --- | ---: | ---: | ---: |
| Saved reference | 3 | 70.314 | 0.000 |
| March code replayed today | 3 | 70.314 | +390.788 |
| Current code replayed today | 2 | 70.787 | +390.788 |

Current selection prefers iteration 2's held-out prediction score of 240.8
over iteration 3's 270.8. These use four and three held-out sweeps respectively.
The change accounts for the extra 0.473 mm in the returned result. The existing
log `fifth_hp3_dataset_d45c9a41`, using the earlier rank-based history selector,
also returns 70.314 mm with the same
+390.788 intermediate mean delta.

This rules out subsequent tracked algorithm changes as necessary to produce
the large trajectory penalty: the March source produces it too. Numerical
environment or execution differences are the leading explanation. The original
artifacts do not record dependency versions, an interpreter identity or a
source hash, so the exact library/runtime change cannot be established, and
unrecorded working-tree differences cannot be excluded. Reverting source alone
does not restore the saved trajectory.

The historical global-radius run `fifth_hp3_dataset.full_auto_87.log`, recorded
March 19 at 19:58, has the same seven parameter errors and final error as the
March 23 reference. The saved trajectory was therefore reproducible across
historical runs; it was not unique to the fixture selected on March 23.

## Implications

Use returned-calibration error to assess calibration accuracy, and report
intermediate failure count and trajectory statistics separately. Avoid treating
arbitrary coordinates from cost-100 sentinel fits as returned-calibration
quality. Keep them as evidence of solver instability.

Future reference runs should record the source hash, dataset hash, interpreter,
numerical dependency versions and resolved options. That would make the next
historical comparison attributable. A dependency-controlled reproduction of
the original runtime would be needed to identify the precise numerical cause.

Artifacts retained alongside this report: `results.json`, `reference.log`,
`reference.jsonl`, `reference-settings.json`, and complete isolated source/run
directories `march/` and `current/`.
