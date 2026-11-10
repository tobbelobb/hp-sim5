# Stable encoder averaging and numerical fragility in HP3 autocalibration

Status: complete.

## Abstract

This investigation asked how to improve encoder-only HP3 autocalibration with known radii, no line layering and automatic stopping. Profiling the supplied recording found settling dominant, and frozen candidates reproduced score/geometry disagreement. Sparse sphere-closure and initializer trials did not justify fewer-point defaults. A return shortcut saved time in bounded trials but worsened full-auto collection. Original noise samples exposed accumulation error in 413 of 640 encoder mean fields. Generic centered averaging reduced this to two fields and made successful/failed recorded 140-point inputs return the identical constrained 2.4937 mm anchor-RMSE fit. The retained eight-line averaging change finished automatically with 200 points, 5659.096 simulated seconds and 2.3843 mm anchor RMSE. Its unpaired total is 16.24% shorter and its anchor error 15.61% lower than the supplied reference; different adaptive stopping paths prevent attributing this to faster collection. Against the fresh current baseline it uses 25% more points and 12.93% more time for 5.79% less anchor error. The return shortcut was removed. Evidence is simulated or analytical on one stock HP3 geometry; general fewer-point reliability, score-based accuracy and hardware transfer remain unresolved.

## Topics

HP3, CDPR, encoder-only autocalibration, centered averaging, numerical sensitivity, underconstrained sentinel, return-to-origin, sparse points, candidate ranking, automatic stopping, patience, fixed-budget benchmarking, Rerun profiling, research harness

## Tested hypotheses

| Tested hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Fit scores reliably rank known geometry | E01, E02, E12 | Rejected on reference; fresh baseline final validation selects its best fit | [Reference timing/ranking](E01-profile-and-geometry.json), [grid/anchor audit](E12-geometry.json) |
| Lost fixed-encoder variation explains the reference failure | E01 | Rejected for this dataset: variation below 1e-6 mm | [Fixed-encoder audit](E01-fixed-encoder-audit.json) |
| Sparse sphere closure robustly replaces current fitting | E03, E05 | Rejected as a default | [48 recorded fits](E03-closure-pilot.json), [36 synthetic fits](E05-synthetic-closure.json) |
| Closure initialization enables three points per direction | E04 | Rejected: all four sparse prefixes return the sentinel | [Eight initializer trials](E04-seed-pilot.json) |
| Faster individual returns improve full-auto collection | E06, E09, E07b, E19, E24 | Rejected as default; positive local timings do not yield a net gain | [Bounded returns](E09-physical-returns.json), [return-only failure](E13-live-comparison.json), [mean-only control](E26-mean-only-comparison.json) |
| Near-identical encoder means can change recovery and collection | E15 | Supported on paired recorded inputs | [Six numerical controls](E15-numeric-pilot.json) |
| Centered averaging improves mean accuracy and fixes the paired sentinel failure | E16–E18 | Supported on captured samples and paired fits | [Original sample audit](E16-mean-audit.json), [paired generic averages](E17-stable-mean-pilot.json), [arithmetic tests](E18-mean-tests.txt) |
| Centered averaging improves every intermediate geometrical fit | E20 | Rejected; mixed prefix outcomes | [Five additional prefixes](E20-stable-prefixes.json) |
| Averaging alone improves the complete one-click workflow | E22, E24, E26 | Mixed: better geometry, but more points/time than fresh baseline | [Stored replay](E22-replays/stable.log), [fresh comparison](E26-mean-only-comparison.json) |
| Stored-data fitting remains compatible | E11 | Supported: all 11 regressions exactly equal | [Regression log](E11-regressions.txt) |
| Continuous acquisition, task-space precision or pointwise fixed geometry improves calibration | none | Not tested | [Initial hypotheses](research.md) |

## References

- [Full report](report.md): measurements, implementation and limitations.
- [Notebook](research.md): objective, hypotheses, budgets, steering, experiment decisions and failures.
- [Manifest](manifest.json), [retained diff](final-source.diff), [return-only diff](E07-return-only-source.diff), [combined diff](E19-combined-source.diff): baseline revision `45862559e95af66a5ed749540763d41d859f5b7e`, frozen code/input hashes and which changes survived.
- [Supplied reference](../../../extended-autocal/no-buildup-reference-003/), its [RRD](../../../extended-autocal/no-buildup-reference-003/hangprinter-20261010T182010.757112Z-814ac4d9.rrd), [dataset](../../../extended-autocal/no-buildup-reference-003/sweeps.json) and [stage snapshots](../../../extended-autocal/no-buildup-reference-003/sweeps.stages/): E01/E12 recorded evidence at revision `1e026534483e2175f4c101b645b1f54f65f9faba`.
- [Fresh baseline manifest](native/output/research/sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E02-baseline-live/sweeps.headless/manifest.json), [return-only manifest](E07b-candidate-live/sweeps.headless/manifest.json), [combined manifest](native/E19-stable-mean-live/sweeps.headless/manifest.json), [retained averaging-only manifest](E24-mean-only-live/sweeps.headless/manifest.json): used HP3 configuration, effective layering, geometry, firmware, timings and applied M669.
- [Independent RRD verification](E14-rrd-verification.json): measurement counts, encoder means, frozen scene/config hashes, physical stop and finalized recordings; tool/recording integrity does not establish calibration accuracy.
- [Prior settling study](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md): optimized point waits and endpoint recovery already in the current baseline.
- [Prior thinning/ranking study](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md), [covariance study](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md), [bounded-update study](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): preceding negative results narrowed this study; their experiments are not counted as new results.
- Yuan, Han; You, Xianghui; Zhang, Yongqing; Zhang, Wenjing; Xu, Wenfu (2019), “A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation,” DOI [10.3390/app9112182](https://www.mdpi.com/2076-3417/9/11/2182): redundant encoder-length least squares motivates the closure pilot; ideal simulated accuracy does not establish noisy HP3 performance.
- Caverly, Ryan J.; Cheah, Sze Kwan; Bunker, Keegan R.; Patel, Samir; Sexton, Niko; Nguyen, Vinh L. (2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://experts.umn.edu/en/publications/online-self-calibration-of-cable-driven-parallel-robots-using-cov/): institutional abstract motivates covariance-guided data retention and stopping; PDOP/ODOP are distinct from the current anchor covariance.
- Alexandre dit Sandretto, Julien; Daney, David; Gouttefarde, Marc (2012), “Calibration of a Fully-Constrained Parallel Cable-Driven Robot,” ROMANSY, [author PDF](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf): redundancy-based joint geometry/pose self-calibration motivates latent-pose elimination; its embodiment differs from HP3.
- [Current algorithm](../../../../autocal/optimization_algorithm_overview), [objectives](../../../../autocal/objective_functions_overview), [ellipse guide](../../../../autocal/README_elliptical_feature_calibration.md), [extended recording guide](../../../../autocal/data/references/extended-reference.md): solver, collector, selection and recording contracts actually inspected.


[Benchmarking and harness review](tooling-review.md) responds to the user's stopping-policy correction. Full-run totals are descriptive policy outcomes, not isolated collection-speed estimates. Proposed fixed-work, common-budget and tool improvements are not tested. No new experiments were run.
