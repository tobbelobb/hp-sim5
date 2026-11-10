# History-seeded fit variants for robust encoder autocalibration (HP3, no buildup)

Status: complete.

## Abstract

Encoder autocalibration repeatedly fits the growing sweep dataset, and a cold
multi-start fit sometimes lands on the cost-cap sentinel or stalls in a bad local
optimum. We reproduced that offline on the supplied 9-sweep HP3 reference
dataset and in live HP3 runs, then implemented a small change: each iteration
also fits the present dataset seeded from the best anchors found so far, and the
two candidates compete through the existing validity and ranking selection. The
change adds no data collection and leaves the one-click flow and automatic
stopping untouched. In a matched live A/B on the same unlayered HP3 world with
the identical 9-sweep/180-point layout, the applied anchor RMSE improved from
2.706 to 2.440 mm (reference 2.828 mm) at the same seven iterations and the same
patience stop; on the identical reference dataset replayed offline it improved
from 2.623 to 2.552 mm applied and 2.528 to 2.426 mm best candidate. Cold fits
are bit-identical between variants, so the change is purely additive, and it
costs roughly 2-8% of a live run's wall time in extra fitting. All evidence is
simulated/synthetic; no hardware measurements were taken, and host load made
wall-clock comparisons unusable.

## Topics

autocalibration, HP3, CDPR, encoder-only, local optima, cost-cap sentinel,
multi-start, warm start, candidate ranking, patience stopping, simulation

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Selection score disagrees with geometric accuracy in the reference run | E01 | Supported | `offline/audit.py` on `offline/stages/history-007.json` |
| Fewer points per sweep preserve accuracy | E02 | Rejected | `offline/thin.py`, `offline/thinruns/*.log` |
| Unmodelled cable elasticity dominates the anchor error | E03 | Rejected | `offline/joint_fit.py`-adjacent `--flex` replay |
| Fixed-line encoder deltas drift inside sweeps | E03 | Rejected (no drift) | `offline/trilat.py` |
| Nuisance offsets or unknown home pose improve anchors | E03 | Rejected | `offline/joint_fit.py` |
| Broadening the final consensus beyond the closest pair helps | E04 | Inconclusive / not supported | `offline/audit.py` candidate ensembles |
| Refitting each iteration from the best-so-far anchors (competing warm start) improves robustness and accuracy | E04, E05, E05b, E05c | Supported in simulation | `offline/loop_emul.py`; `offline/base-replay.log` vs `offline/warm-replay.log`; live `native/autocal/c75c027e...` (divergence rescue) and the matched pair `native/autocal/d7a7507f...` vs `native/autocal/aa8435df...` |
| Cutting collected sweeps (early stop) preserves accuracy | E05 | Rejected | `offline/loop_emul.py` cumulative-sweep ablation |
| Force-selection sensitivity explains most cross-world accuracy spread | none | Not tested | no matched-world fixture available in the task regime |

## References

- [Full report](report.md): implementation, measurements, limitations.
- [Research notebook](research.md): budgets, experiment decisions, steering.
- [Reference run to beat](../../../extended-autocal/no-buildup-reference-003/): supplied 9-sweep HP3 baseline, applied anchor RMSE 2.828 mm.
- [Controlled replay](offline/base-replay.log) and [warm replay](offline/warm-replay.log): identical-data offline A/B.
- [Matched live A/B](native/autocal/d7a7507f7b60486da3c0d626be819afd/sweeps.full_auto.log) (base@10) vs [H6@10](native/autocal/aa8435dfa54c4c28a8f040c43fa2ebce/sweeps.full_auto.log): same 9 sweeps/180 points, 2.706 vs 2.440 mm.
- [Prior session, settling and score-vs-geometry](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md): earlier HP3 collection-timing and ranking findings reused here.
- [Recording-cost budget](../../../../research/autocal-recording-performance.md): recording modes are cheap relative to collection.
- [Truth anchors](../../../../autocal/tools/regress_calibration_logs.py): `SIM_3D_TRUE_ANCHORS` used for all RMSE numbers.
