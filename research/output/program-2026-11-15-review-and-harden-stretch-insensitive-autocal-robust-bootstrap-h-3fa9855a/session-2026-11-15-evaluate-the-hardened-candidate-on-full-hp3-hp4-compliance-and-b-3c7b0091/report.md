# Frozen candidate production and paired-fit evaluation

Candidate `921ced3c9523847c03753f5bf0e227d6c45519e6`. These are simulated production collector/firmware/fitter runs and recorded simulated-data replays, not hardware measurements. Each condition is one run; no robustness bound follows. Firmware M669, rounded to 0.01 mm, is the nominal scene reference; translation alignment subtracts the mean four-anchor error. The question is whether the hardened rough bootstrap and final selection eliminate earlier full-path accuracy failures. They do not in these examples.

## Full production runs

| Case | Stop | Sweeps / points | Worst coordinate mm, absolute / aligned | Earlier frozen baseline absolute / aligned mm |
| --- | --- | ---: | ---: | ---: |
| HP3 stiffness 10000 | patience | 8 / 160 | 4.60 / 2.16 | 9.98 / 9.59 |
| HP3 stiffness 12000 | patience | 8 / 160 | 2.21 / 1.22 | — |
| HP4 nominal | spread consensus | 8 / 160 | 4.82 / 2.83 | 4.83 / 2.835 |
| HP4 worn | spread consensus | 8 / 160 | 2.50 / 1.055 | 1.88 / 0.64 |

All four completed normally (inner exit 0) but the battery returned 1 because each exceeded its default 1 mm threshold. Nominal HP4 called the result “near-perfect” despite D.z 1895.18 versus 1900 mm. Previous nominal HP3 was 1.37/1.225 mm; a final candidate nominal run belongs to the parent and was not repeated here. Earlier [baseline report](../session-2026-11-15-build-and-exercise-reproducible-full-hp3-hp4-autocal-sanity-batt-fc7253a7/report.md) collected different observations and predates this candidate. Differences therefore do not isolate the hardening or establish repeatability.

The accepted rough bootstrap replay gave 24.77/24.77 mm (10000), 20.92/21.11 mm (12000), 5.87/5.62 mm (HP4), and 12.31/12.00 mm (worn HP4); its own full-rank/convergence criteria and assumed-noise uncertainty passed. The two HP3 cases needed two extra rough sweeps; both HP4 cases needed one. See [rough replay](evidence/rough-replay.json) and each run's checks and rough dataset. Final fits improved those errors but rough errors alone do not explain final outcomes. For 10000, iteration 3 selected 4.60 mm, while the fourth iteration's printed M669 has 2.44 mm worst coordinate error. The last candidate had no future sweep for held-out evaluation; this demonstrates selection regret on this run, not that using the last fit would generally improve accuracy. The applied iteration 3 failed the 5 mm consensus spread criterion (5.868 mm) and was used as best-ranked. [Full log](evidence/hp3-10000/sweeps.full_auto.log) preserves the candidates and selection.

## Same-observation fit replays

Each row replays one full dataset with four explicit option pairs: offsets off/on and pointwise plus sweep-wise filtering off/on. Both fitters use the same observations within a row; point-fit objective is independent but includes the ellipse result as an additional seed, and does not implement the ellipse filtering switches. Values are worst coordinate errors in mm, absolute/aligned, ellipse then point. The stage uses archived h1 template settings and a replaced embedded dataset, so it is a controlled offline fit, not a reproduction of every warm start or final history selection. Full options, anchors, costs and commands are in the linked summary files. All replay arms exited zero.

| Recorded dataset | offset / filtering | ellipse abs / aligned | point abs / aligned |
| --- | --- | ---: | ---: |
| [Previous nominal HP3, 140 points](evidence/compare-hp3/summary.json) | off / off | 12.433 / 12.001 | 14.246 / 13.547 |
| | off / on | 15.920 / 11.822 | 14.246 / 13.547 |
| | on / off | 2.295 / 1.030 | 2.526 / 1.218 |
| | on / on | 1.569 / 0.965 | 2.526 / 1.218 |
| [Previous soft HP3, 140 points](evidence/compare-hp3-soft/summary.json) | off / off | 23.008 / 22.797 | 33.371 / 33.134 |
| | off / on | 48.448 / 45.588 | 33.370 / 33.133 |
| | on / off | 7.919 / 7.824 | 8.375 / 7.975 |
| | on / on | 7.919 / 7.824 | 8.375 / 7.975 |
| [Candidate soft HP3, 160 points](evidence/compare-candidate-soft/summary.json) | off / off | 25.999 / 24.162 | 33.317 / 31.693 |
| | off / on | 31.784 / 30.457 | 33.317 / 31.692 |
| | on / off | 3.086 / 1.027 | 2.524 / 0.754 |
| | on / on | 2.445 / 1.911 | 2.525 / 0.754 |
| [Candidate HP4, 160 points](evidence/compare-candidate-hp4/summary.json) | off / off | 8.540 / 7.945 | 8.823 / 8.706 |
| | off / on | 9.519 / 9.204 | 8.823 / 8.706 |
| | on / off | 4.479 / 2.653 | 4.492 / 2.668 |
| | on / on | 2.515 / 1.986 | 4.492 / 2.668 |

On these datasets offsets improve these offline error measures substantially; they do not remove the earlier soft-data error. Filtering without offsets worsens absolute ellipse error in each example, although normalized cost falls. On candidate soft data, filtering with offsets reduces absolute ellipse error but worsens aligned error. Candidate HP4's final history result differs from the full-data offline fit, as expected from selection and warm starts. Neither objective nor point-fit RMS alone is a physical-accuracy guarantee.

[Seeded corruption](evidence/compare-hp3-soft-corrupt-v2/summary.json) of the previous soft recorded dataset adds independent Gaussian 0.03 mm to drive and sensor lengths, updating both raw and mean length fields, and +1 mm to three sensor points (zero-based indices 86, 22, 67), seed 9212026. It is synthetic corruption of recorded lengths, not simulated encoder noise. With offsets on, filtering off gives ellipse 8.454/7.926 and point 8.166/7.737 mm; filtering on gives ellipse 7.737/6.181 and point 8.166/7.737 mm. One corruption is not a robustness bound. An initial raw-only corruption left the used mean lengths unchanged and produced identical results; that invalid stress attempt is retained in its comparison summary and is not used as evidence of robustness.

## Reproduction and provenance

Run from repository root with `.venv/bin/python`; native physics and RRF binaries were built in this worktree, and Python and Node dependencies linked from `/home/torbjorn/repos/hp-sim5`. Production commands below used the default 1 mm threshold, no layering or recording, and at most two simultaneous battery or replay processes. Each `OUTPUT` must be new. Default stop failures are preserved in summaries and exit files.

```sh
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp3-10000 --machine hangprinter_3 --stiffness 10000
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp3-12000 --machine hangprinter_3 --stiffness 12000
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp4 --machine hangprinter_4
.venv/bin/python -m autocal.tools.sanity_battery OUTPUT/hp4-worn --machine hangprinter_4 --scene public/usd_scenes/hp4_worn_bearings.usda
```

The [comparison recipe](prepare_comparison.py) creates a stage with `read_stage_artifact`/`write_stage_artifact`, explicit option files, reference and hashes. For each dataset:

```sh
PYTHONPATH=. .venv/bin/python INVESTIGATION/prepare_comparison.py research/output/autocal_research_with_Fable5p2/evidence/headless_runs/h1/sweeps.stages/fit-001-default.json DATASET PREPARED
.venv/bin/python -m autocal.tools.compare_fit_replays PREPARED/stage.json OUTPUT PREPARED/offset-false-filter-false.json PREPARED/offset-false-filter-true.json PREPARED/offset-true-filter-false.json PREPARED/offset-true-filter-true.json --reference-m669 PREPARED/reference.txt
```

Add `--corrupt` to preparation for the seeded perturbation. [Input metadata](evidence/prepared-hp3-soft-corrupt-v2/input.json) records the seed, source and template hashes and point indices. [Evidence inventory](evidence/inventory.json) records input and result hashes and event-journal sizes/hashes. Journals remain local only and are excluded from Git; no portable off-host copy is asserted. Reproduction requires the physics and firmware toolchain. A controlled matched-collection test, independent truth/held-out data and multiple conditions could change the interpretation of the relative accuracy and selection effects.
