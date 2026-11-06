# Finite-prediction consensus: broader replay rejects a blanket restriction

Status: complete.

## Abstract

Should final calibration consensus exclude fits without a finite held-out
prediction? An earlier three-history screen improved two cases. This frozen
comparison used eleven older simulated histories, including eight dataset names
beyond that screen. Among the eight additional datasets, one improved, two
worsened and five were unchanged; summed per-anchor error changed from 401.100
to 400.797 mm. Excluding the newest fit increased error by 0.503 mm on flexible
lines and 0.094 mm on one buildup dataset. All eleven chosen single-fit
prediction scores matched saved values under current code, and every applied
model diagnostic was finite. However, the three changed comparisons had no
common sweep unseen by both applied models, so predictive improvement remains
unestablished. The probe used 22 selections and 49 frozen prediction evaluations,
with no fits, physics or collection. Keep production defaults and retire the
blanket restriction as a default-change proposal. These are saved simulation
results, not hardware accuracy or changed stopping trajectories.

## CM-E1: broader accuracy comparison

The [initial screen](../2026-10-23-program-task-sanity/report.md) improved two of
three frozen histories by 3.905 and 2.118 mm summed anchor error. Those histories
motivated the rule. Their results were reused, not rerun or counted as independent
confirmation.

We selected one final history for each of the eleven dataset names in the
[October 15 corpus](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e02---spread-gate-replay-over-the-11-stored-datasets-supported).
Eight names are additional to the initial screen. Three overlap and serve only
as sensitivity checks across producer revisions. The [input audit](audit.json)
finds eleven distinct sweep-content hashes, rather than repeated cache or timing
replays. This is variation across datasets, not eleven independent physical worlds:
the Slideprinter fixtures share nominal geometry, as do the HP3 fixtures.

The baseline calls current `select_history_plan` on saved evaluated candidates.
The variant changes only the set sent to `consensus_anchor_estimate`, retaining
finite stored own-model prediction scores. Ranking, chosen single fit and its
spool model, 0.01 mm merge distance and 5 mm spread limit are fixed. An absent
consensus keeps the chosen single fit. Truth is never consulted by the rule.
Errors below are sums of per-anchor Euclidean distances in millimetres, using
the fixture truth in [inputs.json](inputs.json); they are not coordinate RMSE.

| Additional dataset | Baseline members | Finite-only members | Baseline error, mm | Finite-only error, mm | Delta, mm |
| --- | --- | --- | ---: | ---: | ---: |
| 23_april_2026 | 4,5 | 4,5 | 99.579733 | 99.579733 | 0 |
| fifth_hp3_dataset | 2,3 | 2,3 | 65.908288 | 65.908288 | 0 |
| nineteenth_hp3_dataset_even_more_pressure | single fit 2 | single fit 2 | 165.005839 | 165.005839 | 0 |
| sixteenth_hp3_dataset_even_more_pressure | 4,5 | 4,5 | 37.934862 | 37.934862 | 0 |
| flexible_lines_2000 | 3,4 | 2,3 | 23.533389 | 24.036387 | +0.502998 |
| ten_points_bigger_deltas | 1,4 | 1,4 | 2.513340 | 2.513340 | 0 |
| ten_points_w_buildup | 3,5 | 3,4 | 2.380698 | 2.474855 | +0.094156 |
| ten_points_w_buildup2 | 4,7 | 3,6 | 4.244127 | 3.343729 | -0.900399 |
| Sum | | | 401.100276 | 400.797031 | -0.303245 |

The predeclared nonworsening condition fails on two additional datasets. The
newest unvalidated fit can help the applied geometry; absence of a future sweep
is not equivalent to a failed prediction. The one improvement does not justify
selectively tuning membership to these fixtures. All three changes occur among
Slideprinter datasets, so even a machine-level restriction would retain both
signs of effect.

The three overlapping dataset names are unchanged in these older histories:
five-points is 10.893581 mm with members 4,5; layered is 5.279793 mm with members
4,6; seventh HP3 is 198.592312 mm using single fit 4 after the spread guard.
These are different histories from the initial screen, not contradictory
recomputations of its inputs. Across all eleven, baseline error is 615.865962 mm,
consistent with the preceding report's rounded 615.866 mm; finite-only is
615.562717 mm. We do not aggregate repeated datasets across the two studies as
independent support.

## Prediction guardrails and source compatibility

The full applied plan uses the averaged anchors with the best-ranked fit's
unchanged spool parameters. We scored that combination for both arms with the
saved collector angle preference, noise settings, residual mode and sigma source.
Checking a member's individual score alone would miss the actual applied model.

All eleven version-1 envelopes decode with current `read_stage_artifact`.
Saved evaluated candidate payloads exactly match their raw candidate entries;
no candidate changes during the probe. The archived source hash maps and current
hashes are in [results.json](results.json). Relevant source has changed since
producer revision `72523b9d`, including removal of the flexibility arguments.
Every input had that correction disabled with multiplier 1.0. Recomputing the
chosen single fit's future-fold prediction matched all eleven saved scores
within 1e-8 absolute/1e-10 relative tolerance. This establishes compatibility
for those frozen diagnostic evaluations, not equivalence of current fresh fits
or a recomputation of every candidate's ordering.

All 49 prediction evaluations were finite: eleven chosen single fits, twenty-two
applied plans on each chosen fit's future fold, and sixteen applied plans on
common unseen folds. The saved candidate predictions contain no failed scores.
There are no all-failed or all-unvalidated histories in this corpus; those
production fallbacks were inspected, not newly exercised by this experiment.

The chosen fit's future fold may already have trained a consensus member. The
probe records overlap IDs rather than calling such a score held-out validation
of the average. On the three changed selections these diagnostics were:

| Dataset | Baseline diagnostic | Finite-only diagnostic | Common unseen sweeps |
| --- | ---: | ---: | ---: |
| flexible_lines_2000 | 21.723213 | 21.977461 | 0 |
| ten_points_w_buildup | 22.188339 | 22.534494 | 0 |
| ten_points_w_buildup2 | 21.495201 | 21.845175 | 0 |

These costs cannot establish predictive harm or benefit: both applied models
overlap that diagnostic fold, and no common unseen sweep remains after excluding
training data of all applied members and the chosen spool fit. Eight other
histories have common unseen folds and finite scores, but the two policies apply
identical geometry there. H2 is inconclusive for changed selections. The small
anchor improvement on the second buildup dataset also does not improve its
overlapping prediction diagnostic, illustrating the need to evaluate both.

## Decision and next useful observation

**Retire the blanket finite-prediction membership restriction as a default-change
proposal. Keep current all-candidate consensus.** H1 fails its nonworsening
condition, and there is no predictive evidence supporting a narrower deployable
rule. The earlier two gains remain valid within their frozen-history scope.
No thresholds were retuned, reference outputs rewritten or production behavior
changed. The candidate remains an analytical comparison, not an opt-in feature.

Current `autocal.py` uses `select_history_plan` for final summary/acceptance;
`stopping.py` separately uses raw-history consensus. A final-selection-only patch
could be evaluated without asserting a changed stopping or fitting trajectory.
A change to the shared consensus helper would need the corresponding replay.
This investigation provides neither trajectory nor hardware evidence.

No further numerical work is needed to reject unconditional adoption. Reopening
membership needs a truth-free discriminator and a common unseen whole-sweep
comparison of actual anchor-plus-spool models on the regression cases. Search
compatible saved later sweeps first; if unavailable, explicitly budget a
fit-on-prefix/terminal-holdout replay. That narrower rule is **not tested** here.
Task 006's durable handoff remains next in queue order; task 005 can use these
exceptions when choosing the next accuracy investigation.

## Reproduction and verification

From the repository root:

```bash
.venv/bin/python output/research/sessions/2026-10-23-consensus-membership/probe.py
.venv/bin/python output/research/sessions/2026-10-23-consensus-membership/audit.py
```

The [probe](probe.py) reads hashed, gzip-compressed original envelopes from
[inputs/](inputs/) using the [manifest](inputs.json); temporary decompression
uses the existing typed artifact decoder. It needs no original worktree paths.
The [audit](audit.py) verifies input/script identity and dataset uniqueness
without repeating numerical selection or prediction. [audit.json](audit.json)
also preserves individual candidate errors and training-sweep coverage.

The producer ran the probe once: 22 selections, 49 prediction evaluations,
2.259 seconds in the timed section excluding import startup; this is not a
fitting or full-autocal speed benchmark. The nonnumerical audit passed once.
There were zero fresh fits, physics runs/steps or collections. No physics,
motor, cable transport or scene dynamics code changed, so `npm run test:physics`
is outside this research-only result's scope.

## References

- [Initial screen](../2026-10-23-program-task-sanity/report.md): motivating measurements reused without duplicate execution.
- [October 15 report](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md) and [verification script](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/scripts/verify_gate.py): corpus origin and original spread-gate scope.
- [Lane 04 integration](../../../../research/autocal-ranking-lane04-integration.md): separation of single-fit diagnostics and applied geometry.
- [Selection source](../../../../autocal/history_selection.py), [prediction source](../../../../autocal/predictive_validation.py), [final consumer](../../../../autocal/autocal.py) and [stopping consumer](../../../../autocal/stopping.py), evaluated at `5400cefc`: implementation and consumer boundaries; exact file hashes in results.
- [Fixture truth definitions](../../../../autocal/tools/regress_calibration_logs.py): nominal simulated anchors, captured in the input manifest.
- [Notebook](research.md) and [abstract](abstract.md): budget, hypotheses and decision handoff.
