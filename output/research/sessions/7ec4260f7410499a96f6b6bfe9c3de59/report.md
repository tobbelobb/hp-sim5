# Encoder autocalibration research report

## Change

Full auto now scores each earlier calibration on complete sweeps collected
after that calibration. It ranks those candidates by robust held-out ellipse
prediction cost, then uses the existing score/rank ordering as a tie breaker.
If there is no later sweep, it uses the existing ranking. The gate adds no
movement, keeps the one-click collector and stopping behavior, and logs its
held-out cost and sweep count. The scoring helper is in
`autocal/predictive_validation.py`; history selection is wired in
`autocal/autocal.py`.

The validation error uses the observed drive and sensor lengths in the existing
millimetre dataset and the forward ellipse from candidate anchors. It uses the
latest spool-corrected dataset representation when spool fitting is enabled.
It is a sensor prediction check, not direct proof of anchor accuracy.

## Measurements

| Experiment | Evidence |
| --- | --- |
| Recorded regression, before change | `seventh_hp3_dataset`: rank-selected iteration 2 had 257.599 mm summed anchor error; physical best iteration 3 had 156.862 mm. Selection regret was 100.737 mm. A generated full-auto run showed +39.453 mm true-error delta on this dataset. |
| Synthetic D-opt sweep ranking | With four Slideprinter sweeps, 0.1 mm length noise, one +15 mm outlier and a near-correct initial model, the top D-opt candidate gave 0.0303 mm anchor RMSE, best among six candidates. A lower fitted objective (3.030 vs 3.115) had worse anchor RMSE (0.0343 vs 0.0303 mm). |
| Synthetic poor start | The top-ranked candidate fit had 1190.19 mm anchor RMSE; the best candidate was 1188.08 mm (2.11 mm ranking regret). All candidates were physically poor, so this is a failed calibration case. |
| Held-out scorer | On one whole 40-point sweep with 0.2 mm Gaussian length noise and a +20 mm sensor outlier, robust prediction cost was 17.768 for the exact geometry and 720.534 for a perturbed geometry (40.55x). This validates the discriminator only. |
| Native HP4/RRF collection | One config, five points requested, 200 mm target, 3 encoder samples. Cancelled before any complete sweep at step 25,117 / 50.234 s simulation. Wall time was 411.55 s. At step 24,000, A encoder lag was 1.13 rad with 9 peak missed steps; Rerun showed cable length error near -0.4 m. The failed recording and events are retained; the world was reset after archiving. |

The recorded regression was rerun after the change and still exited 1. Its
diagnostic true-error total improved by 45.912 mm across five datasets, but one
dataset worsened by 0.473 mm and had a score/true-direction mismatch. Those
static replays contain no future sweep, so they used the old rank fallback.
The regression is stochastic and does not establish a gain from the new gate.

## Verification

181 relevant autocal tests passed in 3.66 s, including predictive validation,
active learning, calibration, ellipse cost/solver, spool fitting, no-collect,
UI score and regression-log tests. `compileall` and `git diff --check` passed.
An additional 82 flex, zero-buildup, spool-fit and spool-model tests passed in
1.68 s (four spool tests overlap the first batch).

## Limits and next evidence

The code change is a bounded automatic validation step: the solver's existing
parameter bounds still constrain each calibration, and later sweep observations
decide which historical candidate is accepted. No additional movement is
introduced. The measured synthetic test supports whole-sweep prediction as a
useful discriminator, but anchor-error reduction from the integrated gate has
not been measured. The production collection failed before yielding a usable
sweep, so the firmware-to-dataset path could not validate that claim.

Next, diagnose why the HP4 movement missed steps, then compare rank-only and
held-out selection on repeated complete sweeps,
reporting anchor and radius error, held-out sensor prediction, score ranking,
failure rate, movement count and elapsed time separately. Include elasticity,
winding buildup, outliers and poor initial estimates before claiming physical
accuracy improvement.
