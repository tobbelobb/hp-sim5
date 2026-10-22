# Lane 04 integration: dead CLI removal and reproducible history selection

The unused `--plot-residual-histogram` option is removed. The standalone histogram
script remains supported and documented. Final history selection is extracted
from the large full-auto loop into `history_selection.select_history_plan`, shared
with offline rank replay. This changes no fitting, scoring, consensus or stopping
policy. Offline rank artifacts now record applied anchors and summary metadata,
so the chosen single fit can be distinguished from the consensus mean.

## Evidence and decisions

Lane 04's three-fixture screen found mixed effects from shotgun and consensus,
and one improvement from fit-structure penalty 2. That does not justify replacing
defaults or deleting rank penalties, which also affect per-iteration variant
selection and the spool fitting path. The held-out sort ordering alone cannot
establish that those penalties are dead.

The current parser already uses `BooleanOptionalAction` for robust diagnostics,
following lane 02's cleanup. Keep both enable/disable forms: identical numerical
results establish that they control logging, not that users should lose that
control. This integration preserves their default and spelling.

A static consumer search confirmed the histogram option had no reader. The plot
module is a documented standalone CLI; lack of imports does not make that module
dead. Update the collector's sweep-point help, which also advertised a histogram
consumer that did not exist.

The extracted function preserves the existing failed-prediction veto, fallback
when no held-out observations exist, best-ranked metadata and all-candidate
consensus. In particular, it still permits unvalidated newest fits in consensus.
The lane's mixed results do not support changing this policy here. The guide now
explains that the reported score belongs to the selected single fit, not the
applied average, and that agreement does not validate the averaged geometry.

## Small follow-up investigation

All 14 candidate plans in lane 04's three baseline history artifacts contain 83
noise-metric fields under `plan.calibration.details.noise_metrics`. The report's
claim that frozen histories lack these metrics is incorrect for these artifacts.
Researchers can therefore inspect/recompute scoring from saved metrics before
scheduling additional fits. This only supports scoring-only counterfactuals;
penalty changes that alter the fitting trajectory still need whole-run replay.

Replaying the extracted selection on all three saved histories reproduced the
previous inline implementation's anchor arrays, history order, metadata and plan
keys exactly. This is refactor equivalence on recorded simulated fits, not a new
accuracy measurement. No new fits, movement or hardware experiments were run.

## Validation

88 focused tests passed across history selection, CLI parsing, no-collection loop
and stage replay. New behavioral checks cover failed-prediction veto, unvalidated
fallback, distinction between single-fit scores and applied consensus, candidate
immutability and offline export of the actual consensus anchors. `git diff
--check` passed. No physics, motor, transport or scene dynamics changed.

## References

- [Lane 04 report](/home/torbjorn/repos/hp-sim5-worktrees/research-878b9086/output/research/sessions/2026-10-22-interactive-research-30b803ad/report.md): prior screen and limitations, source 4f175079.
- [Lane 04 proposals](/home/torbjorn/repos/hp-sim5-worktrees/research-878b9086/output/research/sessions/2026-10-22-autocal-flags-04-ranking-cli/proposed-changes.md): integration candidates.
- [Checks and artifact hashes](../output/research/sessions/2026-10-22-autocal-lane04-cleanup/checks.json): three exact selection comparisons and the noise-metric presence audit, integration baseline a194e540.
- [Reproduction script](../output/research/sessions/2026-10-22-autocal-lane04-cleanup/check_histories.py): no fitting or collection; reads the source worktree artifacts directly.
