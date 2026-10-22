# Trust-region anchor solving makes HP3 encoder autocalibration more accurate, cheaper to compute and less selection-sensitive

Status: complete in the research worktree; its default change was not imported
into main after review against the newer main commits.

## Abstract

Encoder-based CDPR autocalibration in hp-sim5 searches for anchor geometry with a
multi-start local solver, and earlier sessions showed its selection score often
disagrees with true geometric accuracy. This session profiled a full headless HP3
run (5180.8 simulated seconds; 21% per-sweep pull-outs, 24% force tuning, 24%
per-point settling, 23% travel), re-tested force tuning, pull-out bounds, point
counts and solver settings, and measured what the algorithm can actually achieve.
Fit-only replay showed the dominant error term is not selection but the common error
of the fitted pool: the applied consensus (2.706 mm) is only 0.32 mm worse than the
best candidate in the pool (2.385 mm). The largest reproducible gain is the anchor
solver: switching the default from `lbfgsb` to bounded `trf` cut mean applied RMSE
over 10 recorded datasets from 4.348 to 3.590 mm (-17%, worst case 9.963 -> 9.206 mm),
took a full headless run from 2.706 to 2.410 mm with byte-identical collected data,
identical 5180.8 simulated seconds and ~4x less solver CPU, and reproduced on an
independent clean 22-pool benchmark of staged fits (target group 4.158 -> 3.485 mm,
selection regret 1.295 -> 0.267 mm). The worktree default was verified end-to-end.
The gain does not transfer to the repository's 11 legacy fixtures (Slideprinter/HP4
recordings with line buildup and fitted spool radii, outside the scoped regime):
there `trf` raises mean applied error from 16.2 to 27.9 mm, because its more
self-consistent early fits change which candidates the tightest-pair consensus picks
(the mechanism was isolated and no alternative selection rule tested, including a
pool of both solvers, repairs it). The proposed default change was verified in the
research worktree but was not imported into main. Fewer-data-point ideas were measured and rejected:
even-point decimation per sweep blows up unpredictably (individual datasets 19-456 mm
at N = 10-14), while planned 6-point sweeps are only a mild accuracy/time trade
(2.514 mm at 4868 s, or 2.031 mm at 4773 s with a lower 0.91 N sensor force).
Limitations: all evidence is simulated. A new paired matrix across 10 HP3 worlds,
varying mover mass (0.80-1.25x) and gravity (8.5-11.0 m/s²), modestly favors `trf`:
mean applied RMSE 2.874 vs 2.967 mm and per-world iteration-mean delta -0.064 mm.
Neither solver had an iteration above 20 mm in that matrix, so it does not test
extreme-failure avoidance. A current-main replay of the three worst legacy fixtures
still favors `lbfgsb` (mean 32.2 vs 76.8 mm), so the default change was not imported.

E36 reviewed the user's subsequent per-restart fallback regression. On the seventh
dataset, fallback removed an extreme first fit but created a tight biased early
pair: summed applied anchor error rose 197.410 -> 257.120 mm while paired
iteration-mean error improved by 1501.281 mm. Seventeen of nineteen retries were
iteration-limit exits. Per-restart fallback is now opt-in; solver status and better
intermediate fits do not guarantee a better applied consensus.

## Topics

autocalibration, Hangprinter 3, CDPR, anchor solver, trust-region least squares,
L-BFGS-B, local optima, multi-start restarts, consensus anchors, candidate
selection, simulator harness validity, fewer data points, sensor collection force,
simulated seconds, HP3

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Raising sweep feed rate is free accuracy-wise | E02 | rejected | B2/B3 cost table |
| Lowering the sensor collection force monotonically improves anchors | E21 | rejected | force sweep 0.68/0.91/1.36/2.2 N |
| Fewer points per sub-sweep is an equal-accuracy speedup | E02/E25 | rejected (`lbfgsb`), mixed (`trf`) | T3 vs T1 |
| The pull-out settle can be bounded below its physical endpoint | E20 | rejected | 435 mm endpoint loss at a 5 s cut |
| The selection score disagrees with true geometric accuracy | E01-lab, E23 | supported, but only 0.32 mm here | oracle 2.385 vs applied 2.706 |
| Solver choice, not data, explains most of the pool error | E23-E26, E32, E35 | supported for tested no-buildup HP3 | 10-dataset battery, `V1-verify-default`, clean 22-pool benchmark, 10 varied simulated worlds |
| Consensus spread is a usable early-stop signal | E27 | supported as a signal, insufficient as a rule | saves a sweep on 2 of 11 datasets |
| `trf` also improves the legacy buildup/radii fixtures | E28, E29 | **rejected** | legacy mean 16.2 -> 27.9 mm applied RMSE |
| Per-restart TRF fallback safely improves applied results | E36 | **rejected as a default policy** | seventh-dataset applied anchor error +59.710 mm despite paired iteration mean -1501.281 mm; preserved on/off logs |
| `trf` modestly improves no-buildup HP3 across varied simulated mass/gravity | E35 | supported, small effect | 10 paired worlds; applied mean 2.967 -> 2.874 mm; no >20 mm iteration failures for either solver |
| A different pool-selection rule repairs the weak solver cases | E30, E33 | **rejected** | current rule best in 3 of 4 groups; solver-union no fix |
| Fewer points per sweep by decimation stays accurate | E34 | **rejected** | N = 10-14 blow-ups of 19-456 mm on individual datasets |
| Divide-and-conquer sub-experiments can isolate parameters | none | not tested | proposed |
| Many plausible worlds are needed to judge robustness | none | not tested | proposed (cost) |

## References

- [Full report](report.md): measurements, rejected hypotheses, the legacy-fixture
  caveat and limitations.
- [Research notebook](research.md): experiment decisions and accepted steering.
- [Reference run](/home/torbjorn/repos/hp-sim5/output/extended-autocal/no-buildup-reference-003/)
  (2.828 mm applied RMSE), the run this work had to beat.
- Clean pool benchmark: `runs_refset/pool_evidence_clean.txt`,
  `runs_refset/pool_evidence.json`, `runs_refset/union_rule_clean.txt`,
  `runs_refset/refset_solver_compare.txt`, `runs_refset/thin_study_boundary.json`.
- Varied-world experiment: `environment-matrix-20261015-retry1/matrix_summary.json`
  and the compact per-world artifacts; collection and metric scripts are under
  `tools/`.
- Earlier sessions: [quiet-window reuse](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md),
  [ranking vs physical accuracy](../autocal-held-out-history-ranking-7ec4260f/abstract.md).
- Code revision: `1404c5ac` (repo HEAD) plus the one-line solver default change.
