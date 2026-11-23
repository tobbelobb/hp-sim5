# Production force-realisation fitting on HP3 and HP5r

2026-11-21. Simulation and synthetic replay evidence. Production baseline `7a57be78` (parent PR #122), opt-in implementation `c5449c61`.

## Decision

Enable the HP3 differential force correction with `--fit-force-ripple`. Keep it opt-in while there is only simulator evidence. It improves the existing equal-strain point fit with home offsets using the configured stiffness. Leave HP5r on the commanded-force fit. The HP5r replays establish that ripple is recoverable, but a small joint fit can mistake other range-model errors for motor offsets or compliance.

## HP3 home-strain fit

The production comparison replays identical recorded points through `fit_ellipse_dataset` with the option off and on. Error is the largest absolute anchor-coordinate error in the print frame. All values below are millimetres.

| Data | Runs | Mean baseline → guarded fit | Fits accepted |
| --- | ---: | ---: | ---: |
| Archived nominal and imperfect machines, 50/turn | 11 | 0.454 → 0.146 | 11 |
| Fresh HP3, 2× ripple at 137/turn, first five profiled sweeps | 1 | 0.740 → 0.438 | 1 |
| Same fresh run, all 12 profiled sweeps | 1 | 0.542 → 0.450 | 1 |
| Synthetic ×2 single-tone ripple at fixed poses | 4 | 0.907 → 0.158 | 4 |
| Synthetic extra 200/turn tone, with recorded 50/turn ripple | 4 | 1.097 → 0.424 | 4 |
| Synthetic cancellation of the recorded ripple | 4 | 0.159 → 0.159 | 0 |
| Synthetic +0.5 N offset on B | 4 | 1.077 → 1.077 | 0 |
| Earlier simulator references, clean, noisy and disturbed | 5 | 2.683 → 2.683 | 0 |

Every accepted run in the table improved. The worst archived result went from 0.595 to 0.200 mm across the cohort. In the two-tone cases the fit selected the stronger tone; the remaining 50/turn error explains why those results were worse than the single-tone ones. Removing one point's encoder angles or reducing coverage to four sweeps reproduced the baseline exactly. The fresh zero-cogging run timed out during force auto-tune, so cancellation in the table is synthetic.

The scan reuses the previous study's 1–400 integer periods per encoder turn, grouped by whole sweep. The shared period and individual motor phases/amplitudes keep the model small. Geometry and the free top home offset are nuisance columns. Each fold reselects the period using its training sweeps, with residuals linearised at the full baseline geometry. The correction requires every fold to recover the same period, at least 75% less held-out squared residual, positive corrected sensor tensions, and a successful bounded production refit that halves RMS on the same points. The raw angles, force commands, and stiffness stay in the source dataset. The fit uses the stretch *difference* between commanded and fitted tension, preserving the HP3 home-strain term and avoiding duplicate correction.

A second geometry fit alone did not explain the gain. Refitting the differential model with zero force correction gave roughly the baseline error. In the full stretch model, freeing compliance gave a different answer: the ripple-plus-contrast fit worsened 10 of 11 HP3 recordings despite cutting the local held-out residual by about 99%. For example, seed 2 went from 0.462 to 0.94 mm while estimated compliance was 1.75 against 0.92 true. In equal-strain sweeps the commanded stretch largely behaves like a change of scale and origin. Small residuals in that geometry do not determine the common compliance or force scale.

The median single-thread CPU time for the 11 production fit-stage replays was 2.61 s baseline and 4.11 s with the option. The median paired cost was +1.56 s per accepted stage. The fit requests no extra collection. The fit-only CLI detected 50/turn on archived HP3 seed 1. The non-heavy autocal suite passed (566 tests). Archived results and fresh data were collected with the baseline fitter, so the paired comparison measures fitting at fixed coverage.

A separate [full-auto CLI replay](evidence/full-auto-137/) on the same fresh 2×, 137/turn recording used 7 sweeps with the flag and 12 without it. The selected fits have 0.459 and 0.542 mm maximum error, respectively. Both replays used the recorded acquisition order.

In full-auto HP3, the jackknife still refits commanded-force measurements. Its score describes sensitivity under that baseline model and does not estimate uncertainty jointly with the ripple correction.

## HP5r and force contrasts

The exploratory replay uses the point-stretch residual, a shared blind period, and a local compliance update, optionally with mean-centred per-motor offset and force-scale contrasts. The comparisons include the configured-stiffness production baseline and a commanded-force local joint-compliance arm. Means in the table are across the same runs except where a fit was invalid.

| Data | Runs | Configured baseline | Joint compliance | Joint + ripple | Joint + ripple + contrasts |
| --- | ---: | ---: | ---: | ---: | ---: |
| Archived usable nominal/137-per-turn recordings | 10 | 1.59 | 1.48 | 0.79 | 0.58 |
| Actual 2× cogging, profiled data retained from incomplete runs | 2 | 1.73 | 1.60 | 0.90 | 0.88 |
| Fresh 1.5× at 137/turn, five profiled sweeps | 1 | 1.92 | 1.18 | 0.77 | 0.78 |
| Synthetic ×2 single-tone force labels | 6 | 1.84 | 4 invalid | 0.81 | 0.61 |
| Synthetic +0.5 N on B | 4 | 1.61 | 1.31 | 0.97 | 0.63 |

The scan found 137/turn in every fold of the archived HP5r 137/turn recording. Its 1.05 mm baseline fell to 0.81 mm with ripple and 0.56 mm with contrasts. With +0.5 N on B, the same run went from 1.16 to 0.76 mm with contrasts, but held-out gain fell to 66%. Seed 5 at 50/turn recovered B's +0.375 N mean-centred offset to about +0.38 N. In clean seed 0, the model inferred opposing corrections around ±1 N even though the simulator generated only ripple. That correction lowered in-sample error but cut held-out squared residual by only 12%. In fixed-compliance seed 8, adding contrasts worsened 0.41 to 0.88 mm despite strong held-out residual gains.

The fresh HP5r run independently recovered 137/turn in all folds of both its four rough sweeps and five profiled sweeps. On its rough sweeps, the configured fit was 1.10 mm and the ripple arm was 0.41 mm. The fresh high-ripple runs were ended after collecting these datasets. Their results are fit comparisons at the recorded sweep coverage.

With an added 200/turn tone and +0.5 N on B, the shared-period HP5r model failed: contrasts worsened both finite fits, and the other two predicted negative sensor forces. A 75% residual threshold alone admitted one of those nonphysical fits. In the no-ripple force-label controls, the contrasts also regressed seed 5 from 0.37 to 0.47 mm. One archived low-EA run converged to a roughly 3 m baseline error, and none of its candidates recovered the geometry. This run is recorded separately from the ten usable nominal runs. The exploratory scan and refits took a median 6.77 s additional wall time on the ten usable runs, versus 9.06 s for the baseline.

The clean-data gain supports treating the periodic term and the contrasts as separate hypotheses. It does not establish that the nuisance terms measure motor calibration. Modelling more than one blind period, separating reeving error, and validating on measured hardware forces would change the decision about an HP5r default and about offset/scale terms. Common force scale remains confounded with compliance. Direction-dependent force terms were outside this comparison.

## Evidence

- [Production paired results](evidence/results/) files start with `production-`. [Production replay](evidence/scripts/production.py) uses the fit-stage entry point and reproduces the synthetic range perturbations at fixed poses.
- [Joint-fit replay](evidence/scripts/replay.py) changes recorded forces for the HP5r synthetic cases and refits the configured baseline on those same labels. It records all model variants, period selections, estimated compliance, and force plausibility measurements.
- [Input provenance and commands](evidence/inputs.md) records the earlier Git refs, simulator commands, and host-local raw storage. The source and input checksums are in [the manifest](evidence/inputs.tsv).
- Earlier context: [force-realisation study](../session-2026-11-21-force-realisation-error-in-autocal-data-replays-t3-test-complian-8252ab16/report.md), including the updated blind scan at `0ee10c68`, and [paired home-strain default](../session-2026-11-21-make-the-home-stretch-correction-the-default-autocal-fit-the-com-01a484d6/report.md) at `2b086a3c`.
