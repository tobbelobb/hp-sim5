# HP3/HP4 production autocal sanity battery

## Question and method

Exercise the current production firmware/collector/fitter path on independent headless simulated plants and compare calibration output with a reference geometry, including translation-aligned shape error. The test is designed to find failures as well as good scores. Each fresh run uses the catalogued 20 cable-solver iterations, stock autocal defaults, no layering and no recording. The soft HP3 scene sets all cable-path stiffness-per-unit-length values from 20000 to 10000; HP4 worn bearings uses the catalogued alternate scene. The helper retains the exact scene, command, manifest, partial data and logs. The firmware's initial M669 is the reference; on these catalogued scenes it represents the configured nominal homed geometry. Scene variants that alter geometry require an independently verified reference. Firmware reports round to 0.01 mm. Translation alignment subtracts the mean of the four anchor error vectors; it is a shape diagnostic, not an absolute calibration accuracy measurement.

The runs are simulation, not hardware. Each case is one trial; no confidence interval, measurement noise sweep, hardware transfer, or general reliability is established. A small fit residual can coexist with wrong geometry. Inspect the retained stages and failures before drawing conclusions.

## Same-data fitter counterexample

The [frozen HP3 bootstrap stage](../../autocal_research_with_Fable5p2/evidence/headless_runs/h1/sweeps.stages/fit-001-default.json) contains constant-force data from revision `1172c62a`. Replaying that identical embedded dataset at current HEAD with stock options versus `fit_home_offsets=true` produced:

| Fit | Normalized cost | Worst absolute coordinate error (mm) | Worst translation-aligned coordinate error (mm) |
| --- | ---: | ---: | ---: |
| Stock | 0.488405 | 8.466 | 8.412 |
| Offset | 0.488261 | 60.382 | 52.333 |

The offset fit placed D.z at 1960.38 mm versus a 1900 mm reference. This is a deliberately unfavorable ablation on earlier constant-force, single-height bootstrap data; it does not establish that offsets harm the newer equal-strain collection. Poor identifiability of the offset and D height is a plausible alternative to any optimizer defect. A multi-height, equal-strain same-data comparison and held-out sweeps would change the interpretation. [Exact replay inputs and results](evidence/fit-comparison/summary.json) retain both complete fit outputs and logs.

The selected seventh-HP3 and HP4 recorded-data full-loop regressions both matched stored outputs exactly; the HP3 history audit reports 100.737 mm selection regret. [Regression output](evidence/recorded-regression.log). Regression agreement checks stability, not ground-truth accuracy. These are simulated recorded datasets, not hardware recordings.

## Reproduction

Use `.venv/bin/python` from the repository root. The helper refuses to overwrite an output directory. For the four cases use:

```sh
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp3 --machine hangprinter_3
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp3-soft --machine hangprinter_3 --stiffness 10000
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp4 --machine hangprinter_4
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp4-worn --machine hangprinter_4 --scene public/usd_scenes/hp4_worn_bearings.usda
.venv/bin/python autocal/tools/regress_calibration_logs.py --dataset-name seventh_hp3_dataset --dataset-name hp4_no_buildup_Oct_16_2026_4x_substepping --jobs 2 --color never
```

For the ablation, use `python -m autocal.tools.compare_fit_replays STAGE OUTPUT default.json offset.json --reference-m669 hp3-reference.txt`; the exact inputs are in [fit-inputs](evidence/fit-inputs/). Outputs retain per-arm commands, option objects, full fit artifacts and logs. Scene overrides are set through `HP_SIM5_AUTOCAL_SCENE` by the helper and are copied and hashed by the headless manifest. Do not infer scene truth from firmware initialization for a modified-geometry scene.

## Full production-path runs

| Scene | Stop | Worst coordinate error, mm | Worst aligned coordinate error, mm | Quality score |
| --- | --- | ---: | ---: | ---: |
| [HP3 nominal](evidence/hp3/summary.json) | spread consensus | 1.370 | 1.225 | 0.7822 |
| [HP3, stiffness 10000](evidence/hp3-soft/summary.json) | patience | 9.980 | 9.588 | 1.36 |
| [HP4 nominal](evidence/hp4/summary.json) | spread consensus | 4.830 | 2.835 | 0.5427 |

The softer case kept all nine stiffness values at 10000 in the baked scene, so the scene change did reach the simulated plant. The fit stopped normally and the UI described its score as near-perfect, yet the geometry was almost 10 mm off even after translation alignment. This contrasts with the nominal run but is one deterministic trial per condition; changed sweep selections, mechanics and force tuning could also contribute. Check same-data ablations and further seeds before attributing the entire difference to elastic model bias. The nominal result exactly reproduces the earlier d2 rounded M669; it is not an independent stochastic replicate. Both cases start from configured nominal firmware geometry.

The worn-bearing case is still running at this initial commit. The three completed runs' event journals remain local and uncommitted; [inventory](evidence/event-journals.json) records their hashes and sizes. The native binary's recorded source hashes match this worktree. Reproduction requires the local Python environment, RRF build, Node dependencies and native physics build; paths in original manifests and input records are historical, not portable.
