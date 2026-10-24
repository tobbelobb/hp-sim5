# Fixed submissions and separate review

Task 010 adds `store.reviews(repository_root)`, a controller API on the existing
program store. It seals a selected submission, starts a separate Codex or Claude
review conversation, records its scoped assessment, and explicitly applies the
assessment to graph memory. It starts no physics world, supervisor or Viewer.
Task 011 owns chat/CLI/MCP entry; task 016 retains the genuine live-review gate.

```python
with ProgramStore('/path/to/manager-store') as store:
    controller = store.reviews('/path/to/hp-sim5')
    submission = controller.submit(
        program_id, ended_attempt_id, claim_id,
        base_revision=base_commit, candidate_revision=None,
        config={'machine': 'HP3', 'comparison': 'frozen final consensus'},
        input_identities=['sha256:EXACT_INPUT_HASH'],
        selected=[
            {'id': 'report', 'role': 'report', 'path': '/session/report.md'},
            {'id': 'abstract', 'role': 'abstract', 'path': '/session/abstract.md'},
            {'id': 'measurements', 'role': 'evidence', 'path': '/session/metrics.json',
             'artifact_id': metrics_artifact_id},
        ], producer_checks=[])
    plan = controller.plan(program_id, submission['id'], backend='codex')
    run = controller.start(program_id, submission['id'], backend='codex', wall_minutes=5)
    # Keep this controller alive and poll status until state != 'running'.
    run = controller.status(program_id, run['id'])
    if run['details']['state'] == 'completed':
        decision = controller.apply(program_id, run['details']['review_id'])
```

`submit` accepts an ended attempt, including a failed or cancelled investigation
with useful partial evidence. It does not infer a scientific result from that
attempt's exit code or accept producer admission fields. The claim is the exact
append-only graph claim version, including its text and scope. The manifest
captures source/base/candidate commit IDs, source-dirty and execution observations,
configuration and input identities, selected report/abstract/evidence/check files,
producer checks, and the actual binary base-to-candidate diff and changed paths.
A candidate proposed for code delivery must already be committed and resolvable
in this repository; omitted `candidate_revision` means a report-only submission,
with no code candidate to mark ready or deliver. The source-dirty observation
remains visible for review; a commit ID alone does not describe uncommitted work.

Explicit selection copies at most 32 files / 32 MiB to the manager store's
`submissions/SUBMISSION_ID/`. Each file has an ID, role and SHA-256. Graph artifact
references must match the selected bytes. Missing selected files remain explicit
gaps, allowing a useful blocking assessment instead of losing the partial record.
There must be report, abstract and evidence bytes to admit knowledge. No automatic
recursive environment/recording capture occurs; known credential/control descriptors
such as `.env`, `auth.json` and `connection.json` are rejected. This is a narrow
selection guard, not a secret scanner. Choose files from an existing
[handoff export](handoffs.md) when the producer session may disappear; do not copy
credentials or every RRD merely to make the submission portable.

The canonical JSON manifest has its own hash. `verify` checks that manifest,
selected bytes, exact claim record and candidate diff before review dispatch,
before saving the assessment and inside the admission transaction. Changed sealed
bytes fail integrity checks; they cannot reuse a favorable review. Changing the
original producer file does not alter a frozen submission. A changed claim,
configuration, evidence set or candidate needs a new submission. `invalidate`
appends a reason and prevents further review/admission for the old submission.
A favorable assessment of one candidate cannot be applied to a different candidate:
`apply` takes only the saved review ID, with no target override.

`plan` returns a genuine supported-client argv and bounded prompt without creating
a run or launching anything. `start` creates a new reviewer context ID, never
resumes the worker conversation, and reuses the same coding-client command/auth
helpers as `hp-sim5-research-agent`. Codex uses `exec`, its existing sign-in/provider
environment and a read-only tool sandbox. The hp-sim5 and Rerun MCP servers are
disabled. Claude uses its existing authentication, an empty strict MCP attachment,
and saved-evidence reading/shell tools. There is no new model API or credential
provisioning. Provider/model metadata and argv are retained, never environment
values. `research/prompts/reviewer.txt` supplies the evidence-first instructions;
the dispatched prompt hash and contract `hp-sim5-evidence-review-v2` are recorded.
Historical version-1 submissions retain their original contract and semantics.

The reviewer returns a strict JSON assessment naming the submission ID/hash. It
records finding (`supported`, `rejected`, `mixed`, `inconclusive`, `not_tested`),
`keep`/`revise`/`block` verdict and reason, practical value, code readiness,
limitations, unresolved discrepancy and actual checks with selected asset IDs.
Checks distinguish `inspected` producer evidence from `recomputed`, `reproduced`
or `test` observations. Reading a measurement never creates an independent
numerical reproduction. Model identity here is the selected client/model setting;
an omitted model stays unspecified rather than inventing a provider-reported model.

A zero reviewer exit only permits contract parsing and a saved review receipt.
It never writes an admission. Nonzero exits, invalid JSON, mismatched target,
corrupt bytes or wall timeout yield no review receipt. Status is nonblocking;
the deadline is enforced when polled by its owning controller. A new controller
can inspect historical runs but does not adopt saved PIDs. Keep polling the
owning controller; durable autonomous recovery/stop orchestration is task 013.

`apply` rechecks integrity and writes one append-only controller decision. Missing
selected evidence, withdrawn evidence, or an unresolved discrepancy cannot become
a favorable scientific admission. A physics/motor/cable/dynamics candidate requires a passing
`npm run test:physics` producer check for the exact candidate plus the selected
check output explicitly inspected by the reviewer. Changed-path detection is
conservative (physics/motor/cable/scene paths); the reviewer must identify relevant
dynamics changes in other paths too. A check record contains exactly `command`,
`outcome`, `candidate_revision` and `asset_id`; no startup/parity/higher-substep
substitution qualifies. Producer check records are inspected receipts, not new
controller reproductions. The fixtures use synthetic receipts and do not claim
that the actual physics gate ran. In version 2, a missing producer gate records
blocked code readiness and engineering reasons while preserving an otherwise
accepted scoped finding. Version 1 retains its historical admission block.
Reviewers can add `required_checks` as explicit argv arrays. The
[integration controller](program-integration.md) retains these requirements and
unresolved reviewer code blockers through checks on the final combined revision.

The finding, execution state, practical value and code readiness remain separate.
A sound report-only negative result can receive `keep` and close an investigation
without a patch. A lower fit-cost measurement can survive while a physical-accuracy
overclaim receives `revise`. Missing scientific evidence yields `block`, retaining
the original reviewer response and controller guard reasons. Missing delivery
checks block code readiness separately under version 2.
Worker assignment records retain their execution/unreviewed observations; graph
admission history is the authoritative knowledge view.

Graph `support`, `search` and other existing queries include submission, reviewer
and admission citations. Submission states show a pending review, completed review
and pending admission, reviewed/withheld decision, invalidation or withdrawal;
the exact candidate and submission hash accompany them. Claim and artifact states
list admitted findings, pending submission IDs and assessment history. An earlier
favorable submission does not admit a new pending candidate. Later assessments
of the same submission supersede earlier effective decisions without deleting
history. Withdrawal removes current reviewed support but preserves the measurement,
review and original decision. Applicability to a new configuration remains
unestablished; historical scope is retained.

## Actual write boundary

The controller owns submission capture, review-run/result records and admission
writes. Workers provide claims and selected evidence. Reviewers provide an
assessment of the fixed target; neither role's output fields or process exit
directly admit graph knowledge. Append-only database triggers prevent accidental
updates/deletes to submission, review and admission history. Import/export is a
**trusted controller backup/restore operation**, preserving historical reviewed
state after validating references and target hashes. It is not an ingestion path
for untrusted worker JSON. JSON exports reference sealed files rather than embedding
them; preserve/copy `submissions/` with the store when future verification is needed.
Historical graph queries can still work when those files are unavailable, but a
new admission fails closed without them.

All of this runs under the same operating-system UID and shared worktrees. A
worker or shell process with that UID can access the manager files, call controller
methods or bypass them through direct SQLite writes. The Claude shell allowance
also relies on the cooperative reviewer contract. Worktrees, read-only reviewer
tools, file hashes and SQLite triggers are accident/integrity checks, not protected
evaluation or adversarial isolation. No stronger deployment boundary is claimed
or required to use this local implementation.

Verification uses real SQLite, git commits/diffs, local child processes and supported
client command construction, with fixture reviewer judgements. Run:

```bash
.venv/bin/python -m pytest -q tests/python/test_research_program_review.py
```

The fixture cases cover a scoped negative finding, lower-fit-cost overclaim,
missing evidence and an actual git physics candidate missing its stability gate.
They also cover changed bytes, fixed candidate scope, later unfavorable reviews,
withdrawal/invalidation, trusted restore, both supported client formats and the
same-UID write boundary. They are lifecycle evidence, not live-model review,
scientific reproduction or `MANAGER_RELEASE_COMPLETE`.
