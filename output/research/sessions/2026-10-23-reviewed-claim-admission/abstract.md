# Fixed submissions and controller-owned evidence admission

Status: complete (task 010 lifecycle capability; manager release not complete).

## Abstract

This session tested whether a controller can retain exact claim/evidence/candidate
versions, commission a separate saved-evidence reviewer and admit only that
assessment. Ninety-six focused tests passed, including twenty-one new review
fixtures, store/graph/worker regressions and affected Codex/Claude launcher checks.
A scoped report-only negative finding yields keep, a lower-fit-cost overclaim
yields revise, and missing evidence or a candidate lacking its required physics
stability receipt yields block. Tests also cover byte changes, later unfavorable
reviews, withdrawal, trusted restore and the cooperative same-UID write boundary.
A separate exercise sealed three existing CM-E1 files and constructed both real
supported-client commands without starting a client or admitting a claim. These
are lifecycle/inspection checks, not numerical reproduction, live-model review or
manager-release proof; task 016 retains the live gate.

## Topics

research manager, fixed submissions, evidence review, claim admission, provenance,
negative findings, candidate stability gate, Codex, Claude, same-UID boundary

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Fixed claim/evidence/candidate versions prevent a stale favorable review from admitting changed targets through the controller API | QA-010-1 | Supported within the cooperative API boundary | [Test transcript](pytest-final.txt), [checks](checks.json) |
| Saved evidence can prepare separate supported-client review commands without physics startup | QA-010-2 | Supported for capture and command planning; live dispatch not tested | [Exercise](exercise.json), [command plans](command-plans.json), [sealed manifest](sealed-example/manifest.json) |
| The complete manager passes its genuine live release gate | None | Not tested | Task 016 remains outstanding |

## Decision handoff

- Scope: standard-library controller/store operations, synthetic local child reviewers and copied CM-E1 evidence; no fresh numerical work.
- Review: implementation author inspected tests, metadata and saved source evidence; no independent numerical reproduction or live coding-client verdict occurred.
- Decision: keep task 010 capability and retain all release limits; same-UID access is not adversarial isolation.
- Next: task 011 chat entry can use this API. Task 016 must exercise actual independent review through the existing coding client.

## References

- [Full report](report.md): behavior and limitations.
- [Notebook](research.md): budget and decisions.
- [Review API guide](../../../../research/program-review.md): exact controller contract and actual write boundary.
- [Fixture judgements](../../../../tests/fixtures/program-review/cases.json): negative/overclaim/missing/gate cases.
- [Preceding dispatch implementation](../2026-10-23-program-worker-dispatch/report.md): existing-worker/auth integration.
- [CM-E1 producer report](../2026-10-23-consensus-membership/report.md): source of copied saved evidence, not a new admitted result.
