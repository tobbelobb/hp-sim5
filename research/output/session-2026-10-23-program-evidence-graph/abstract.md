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

## Topics

research manager, evidence graph, program store, autocalibration, CM-E1,
consensus membership, provenance, required premises, withdrawal, HP4 scope

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Typed queries reconstruct the scoped decision and open questions with sources | G008-CLI | Supported tool check | [CLI transcript](../../../../research/program-graph/queries.json) |
| Withdrawal invalidates necessary AND premises while alternative support survives | G008-WITHDRAW | Supported tool check | [Focused checks](checks.txt), [transcript](../../../../research/program-graph/queries.json) |
| Configuration changes erase historical measurements | G008-APPLICABILITY | Rejected tool behavior | [Transcript](../../../../research/program-graph/queries.json) |
| Imported reports establish hardware accuracy or accepted manager knowledge | None | Not tested; no admission or hardware work | [Inspection limits](../../../../research/program-graph/inspection.json) |

## Decision handoff

- Scope: offline program memory and inspected saved simulation evidence; no numerical reproduction.
- Review: task-008 curator inspected named sources; old paired/comparison/summary artifacts are absent. All claims remain unassessed.
- Decision: task 008 is complete; keep scientific scope and reported outcomes distinct from structural support and admission.
- Next: task 009 dispatches the existing worker. Task 010 owns reviewed admission; task 016 remains the manager release gate.

## References

- [Report](report.md): implementation, query results and limitations.
- [Notebook](research.md): objective, budget and next decision.
- [Graph guide and seed](../../../../research/program-graph/README.md): commands, sources, hashes and real-data exercise.
- [CM-E1 report](../2026-10-23-consensus-membership/report.md): scoped retirement evidence, inspected without replay.
