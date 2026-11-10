# Autocal ranking, bounded updates and identifiability

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

Seven hypotheses were investigated using recorded histories, controlled replay, synthetic bounded updates and native radius sensitivity. Ranking disagreed with physical accuracy in 22 adjacent transitions across seven of eleven datasets. Whole-sweep prediction selected worse physical results on five datasets. Removing the iteration-age bonus improved one case but increased aggregate selected parameter error by 200.701 mm over the full matrix. Bounded residual updates and informative movement selection worked in some synthetic conditions and failed under others. No tested solver or selection patch met the acceptance criteria. The retained change added ranking/ground-truth diagnostics. This session has a detailed report and experiment registry despite lacking a research.md notebook; online collection accuracy remained unvalidated.

## Topics

autocalibration, HP3, Slideprinter, history age bonus, held-out prediction, bounded updates, D-optimal movements, radius uncertainty, negative results

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Ranking agrees with physical accuracy | Recorded ranking audit | Rejected | [Audit](recorded_ranking_audit.json) |
| Whole-sweep prediction repairs ranking generally | Held-out audit | Rejected; worse in 5/11 fixtures | [Predictions](heldout_audit.json) |
| Objective guard, TRF or reconstruction objective improves replay | ten_points_objective_guard / ten_points_trf / ten_points_reconstruction | Rejected | [Controlled trials](report.md#controlled-replay-trials) |
| Removing history age bonus improves the full matrix | ten_points_no_age; paired regression runs | Rejected; aggregate error +200.701 mm | [Paired results](paired_regressions.json) |
| Bounded joint fitting and D-opt movements repair poor starts | Synthetic and recorded joint trials | Mixed; rejected for production | [Synthetic results](synthetic_results.json), [recorded results](recorded_joint_results.json), [movement results](active_results_v2.json) |
| Good motor tracking makes radius uncertainty negligible | Native sensitivity comparison | Rejected | [Comparison](native_comparison.json) |

## References

- [Seven hypotheses, accuracy criteria and negative findings](report.md)
- [Experiment inputs and log paths](experiment_registry.json)
- [Frozen source, dataset and evidence hashes](artifact_hashes.json)
