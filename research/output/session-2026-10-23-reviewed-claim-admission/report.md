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

## Implemented behavior

`ProgramStore.reviews(root)` seals explicit selected bytes and exact append-only
claim records, source/base/candidate commit identities and actual git diffs. A
separate Codex/Claude context returns strict target-bound JSON with finding,
execution-independent practical value, code readiness, actual checks, limitations
and discrepancy. Shared command/auth helpers preserve the existing worker
launcher. Review status saves a receipt but cannot admit knowledge. Controller
`apply` rechecks integrity and writes one scoped admission; missing evidence,
withdrawal, unresolved discrepancies and absent matching inspected physics gates
block favorable admission. Report-only negatives need no code patch.

Graph queries retain pending submissions, completed reviews, admissions,
withdrawal/invalidation and superseded verdict history. Imported bare claims and
producer assessments remain unassessed. Trusted controller backup/restore
preserves existing reviewed history; sealed files are required for a subsequent
integrity check/admission, not for reading historical graph memory.

## Verification record

QA-010-1: the final affected integration passed **96 tests**, **18 deselected**,
**34 subtests**, in 7.00 seconds. This includes 21 review cases and all previously
passing store/graph/worker regressions plus affected launcher command/auth,
worktree, cleanup and client-format checks. [checks.json](checks.json) stores the
exact command and [pytest-final.txt](pytest-final.txt) its complete output.
Earlier investigation ran 14 initial review tests successfully. An old store test
then exposed changed Markdown wording for a program with no admissions; the
conditional no-result message was restored before the final passing integration.
This was an output regression, not a scientific admission.

QA-010-2: [exercise.py](exercise.py) used the actual repository source and a
zero-work producer fixture to copy/hash the existing report, abstract and results
from CM-E1. It verified the sealed submission and generated both supported review
commands through the shared helpers. [pending-query.json](pending-query.json)
retains the pending state and exact evidence citations; there are zero actual
client processes, reviews or admissions. The saved command target paths were
temporary; the script reproduces the exercise with fresh IDs, and the selected
bytes/manifest are retained in [sealed-example](sealed-example/).

Compilation, documentation/queue/source-hash checks and `git diff --check` passed.
The research catalog was regenerated while retaining historical entries missing
from this checkout. No dynamics code changed, so `npm run test:physics` was not a
delivery gate for this task. The physics-gate fixture is synthetic, including its
positive control; it does not claim a real gate ran.

## Limits and next decision

Same-UID workers/reviewer shells can access this store or bypass its public API.
Read-only reviewer tools, worktrees, hashes and append-only triggers are not
protected evaluation. Import is trusted controller restore, not worker ingestion.
Committed candidate patches are supported; report-only submissions propose no
code for delivery. Changed-path gate detection is conservative and reviewers
must flag dynamics changes in additional paths. Reviewer deadlines depend on
polling the owning controller; autonomous recovery/stop is task 013.

Actual nested Codex startup remains blocked by writable-state initialization in
this environment; Claude is unavailable. Existing physics prerequisites remain
incomplete. No new API, key, credential or network installation was introduced.
The parent's separate MCP SDK recovery does not establish a live reviewer or
production physics. Task 016 retains all genuine live-client/production/replanning
release requirements. Proceed to 011 only in a new task invocation.
