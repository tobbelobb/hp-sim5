# Queryable autocal evidence with scoped withdrawal consequences

Status: complete tool implementation and saved-evidence inspection.

## Abstract

Can program memory reconstruct a research decision and the consequences of
withdrawing a necessary premise? Task 008 extends the existing SQLite program
store with append-only scoped claims, typed discussion/decision relations, AND
premise derivations, alternative support, reported assessments and dated review
events. A curated program contains 16 unassessed findings from nine inspected
reports/summaries and structured sources. Eleven CM-E1 compressed/raw history
hash pairs were checked without numerical replay. Twelve fresh-process CLI calls
reconstruct blanket-membership retirement, unresolved predictive/hardware scope,
and counterfactual withdrawal. The CM-E1 retirement path loses support while an
alternative conservative rationale survives; the attribution correction stays
unaffected. Seven focused graph checks and ten existing store checks pass.
Historical source/config scope is preserved when new applicability is marked
unestablished. Missing ranking/attribution artifacts remain explicit. These are
tool checks and inspected saved simulation reports, not independent scientific
confirmation, reviewed admission, hardware validation or full manager readiness.

## Delivered behavior

`ProgramStore.graph()` shares the existing records, transactions and optimistic
program revisions. Claims, artifacts, derivations, relations, reported assessments
and review events are append-only. Later claim versions preserve earlier scope
and append supersession/review records. Required-premise edges are derived from
frozen AND-input lists; alternative sufficient derivations are separate records.
Grounding requires available evidence, so cyclic discussion cannot bootstrap
support. Required-premise cycles are rejected on mutation and snapshot import.

The production CLI adds `claim`, `derive`, `relate`, `assess`, `review-event` and
`query`. Search, support/challenges, upstream/downstream, route rationale and open
question results retain citation closure, source/config/hash details and reported
assessment states. Shared byte/input identities warn about correlated evidence.
Unknown lineage does not imply independence. Imported claims remain unassessed;
all reported assessments remain unreviewed and cannot admit knowledge.

Withdrawal appends a dated review event, preserves the subject and reevaluates
grounded support. A surviving alternative can retain support, with review still
required. New applicability contexts remain unestablished while old measurements
retain their original scope/support. Later dependencies also inherit review
requirements. The store does not automatically change strategic route states.

## Real saved-data query results

The [12-call transcript](../../../../research/program-graph/queries.json) runs
the production CLI in fresh processes after importing the curated snapshot.
The [inspection ledger](../../../../research/program-graph/inspection.json)
lists nine inspected sources, their hashes, missing files and eleven checked
compressed/raw envelope hashes. The [curation script](../../../../research/program-graph/curate.py)
rebuilds the exercise; the [guide](../../../../research/program-graph/README.md)
documents ordinary commands and API semantics.

| Query/exercise | Result |
| --- | --- |
| Why retire blanket finite-prediction membership? | CM-E1 worsens two additional datasets, with no common unseen fold for changed applied models. Earlier scoped gains remain recorded. |
| What remains unresolved? | Comparable predictive safety, live HP4/hardware accuracy and completion of the unfinished historical selector investigation. All imported claims also await admission. |
| Does CM-E1 cover HP4? | Five frozen histories carry the manifest label `hangprinter_4`; six are `slideprinter`. The three changed selections are Slideprinter. Historical `hp3` filenames do not override configuration identity. No live HP4 or hardware validation was established. |
| Withdraw necessary `cm-regressions` premise | `retire-via-cm` and `keep-via-cm` lose structural support; conservative `keep-via-history` survives. Affected conclusions require review. |
| Unrelated attribution correction | Remains supported and unaffected by that withdrawal. |
| Future source/config context | Historical compatibility remains supported; the new context is unestablished. |

Earlier mixed studies report different revisions, selector policies and metrics;
their shared fixture families prevent treating their reports as independent
confirmation. CM-E1 reports earlier gains whose original session is unavailable
here; these remain secondary reported evidence. Old paired-regression JSON,
refreshed comparison JSON and Lane 02 summary-comparison JSON are absent. The
unfinished notebook has no final selector result. No missing or proposed run was
converted into a negative research finding.

## Verification and limits

[checks.txt](checks.txt) records seven focused graph tests, ten existing store
tests and the curation run. Tests exercise required-premise cycle rejection,
discussion cycles, grounded alternatives, withdrawal propagation, later dependent
claims, source/config changes, scope/version preservation, shared evidence,
malformed inputs, stale revisions, immutable history, fresh CLI processes and
lossless graph export/import with rejection of unreviewed admission attempts.

Python compilation and `git diff --check` pass. The session catalog was regenerated
and this session added while preserving historical entries absent in this
checkout. No numerical or physics replay occurred. The result establishes local
graph behavior and cited inspection, not calibration accuracy. Runtime dispatch,
review admission and the complete manager release remain tasks 009, 010 and 016.

## References

- [Notebook](research.md): objective, zero numerical budget and accepted task steering.
- [Seed](../../../../research/program-graph/autocal-seed.json): 16 source-linked unassessed findings and scoped route decisions.
- [CM-E1 report](../2026-10-23-consensus-membership/report.md), [results](../2026-10-23-consensus-membership/results.json), [inputs](../2026-10-23-consensus-membership/inputs.json): retirement, holdout limits and exact configuration labels; analysis `5400cefc`, producer `72523b9d`.
- [Earlier negative ranking report](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): scoped mixed selector outcomes, source `aaf25cf7`; raw paired artifacts absent.
- [Refreshed references report](../../reference-refresh-october-7-2026/report.md): mixed aggregate comparison `b60856d9` to `4daac40c`; comparison JSON absent.
- [Lane 02 attribution](../../../../research/autocal-filtering-lane02-integration.md): individual-fit/final-consensus correction at `87093cdd`; underlying summary JSON absent.
- [Unfinished abstract](../a71aec03021f470796a789ba7382e28a/abstract.md), [notebook](../a71aec03021f470796a789ba7382e28a/research.md): incomplete result boundary.
- [Agenda](../../../../research/autocal-agenda.md): current conservative routes and withdrawal rationale.
