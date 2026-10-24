# Prepare and check reviewed patches

Scientific acceptance and patch readiness have separate records. `review_apply`
admits a scoped finding after separate review. Integration outcomes never edit
that admission. A conflict, stale base, changed candidate or failing combination
can block code while its useful finding remains available to graph and strategy
queries. A report-only negative finding needs no patch integration.

Use the program-bound chat CLI or the same operations through MCP:

```bash
./hp-sim5-research-program tool CONNECTION integration_prepare --params '{"submission_ids":["SEALED_SUBMISSION_A","SEALED_SUBMISSION_B"],"target_base":"EXPLICIT_COMMIT","config":{"machine":"hp4"},"input_identities":["EXACT_INPUT_ID"]}'
./hp-sim5-research-program tool CONNECTION integration_check --params '{"integration_id":"RETURNED_JOB_ID","config":{"machine":"hp4"},"input_identities":["EXACT_INPUT_ID"],"checks":[{"name":"physics delivery gate","argv":["npm","run","test:physics"]}],"wall_minutes":5}'
./hp-sim5-research-program tool CONNECTION integration_status --params '{"integration_id":"RETURNED_JOB_ID"}'
```

Choose the checks and finite wall allocation for the actual final change. The
physics example requires the repository's normal native/Wasm build dependencies.
Missing dependencies produce a failed check, never a substituted physics result.
Additional relevant tests can appear in the same ordered `checks` list. Commands
are argv arrays, without an implicit shell. Checks return immediately and the
retained host polls their child process, within the CLI's ten-second request
deadline. `integration_status` and `overview` return receipts and artifact paths.
Graph queries link the same receipts from the reviewed submissions.

Preparation takes ordered, sealed submission IDs, never mutable worker branches.
It reverifies the selected evidence and binary diffs and records their exact
submission, review and admission IDs. Each selected patch must have a currently
accepted finding. The explicit target resolves once to a commit; every sealed
patch must use that base and descend from it. A differing base yields `stale_base`.
There is no silent rebase. Submit and review a new candidate when the base changes.

The controller creates a new `research-integration/…` branch and worktree under
the private store. It applies frozen patches in order and commits the combined
candidate with repository hooks disabled. It does not check out, reset or stash
the human checkout. It does not merge or push. Failed worktrees, patches and apply
logs remain available. Preparation makes no shared output, dependency or build
symlinks. Install or build prerequisites in the candidate's own paths when needed.
The check child imports Python source from that candidate's `src/python` first.

Preparation receipts record base, combined commit/tree/diff, source configurations
and input IDs, final configuration/input hashes and required checks. Producer
passing checks do not establish combined readiness. Before and after evaluation,
the controller requires the same commit/tree/index and a clean Git status,
including untracked files. Git-ignored build output is permitted. The check plan
and receipt identify the evaluator contract, its revision and controller/runner
source hashes. A changed evaluator invalidates that run. Receipts are append-only
in SQLite, with hashed JSON files; old results retain their original contracts.

Physics, motor, cable transport and dynamics changes require the exact real
`npm run test:physics` on the combined revision. Path detection and prior sealed
requirements can only strengthen this requirement. Version-2 reviewers may also
name explicit `required_checks` argv arrays, including physics requirements for
changes outside filename detection. Every required argv must occur in the final
plan and pass. Startup, parity, synthetic producer receipts and extra substeps
cannot satisfy the gate. The task-015 fixtures exercise missing-gate rejection;
they do not claim an actual physics run.

An unresolved reviewer `blocked`, `needs_changes` or `report_only` readiness for a
patch yields `engineering_blocked`, even if unrelated checks pass. Correct the
candidate and obtain a new fixed review. A sound candidate awaiting delivery
checks can instead receive reviewer `ready` with explicit `required_checks`.
Version-2 admission preserves scientific acceptance when a producer physics gate
is missing, while recording blocked engineering readiness. Historical version-1
admission keeps its original stricter semantics. No old admission is upgraded.

The store reserves exclusive worker/reviewer capacity before preparation starts.
The lease stays held while prepared, during Popen startup, through checks, and
after unknown ownership or cleanup. Other programs sharing the store are also
blocked. This extends task 014's shared-runtime reservation boundary. Pausing
blocks new preparation/checks; running checks continue. Stop and shutdown cancel
owned checks and wait for observed cleanup. A check runner has a finite deadline
and writes its birth identity before launching commands. Cleanup observes its
whole Linux process group, including descendants after leader loss. A process
exit alone does not release capacity.

Discard a prepared candidate with `integration_release`, supplying
`integration_id` and `reason`. After host loss, inspect `integration_status` and
call `integration_recover`. Recovery never adopts or signals historical PIDs.
It retains the lease while any historical group member lives, or startup/cleanup
identity is unknown. Once whole-group exit is observed, a recovery `reason` records
an explicit release without accepting historical checks. Unknown preparation
startup remains held for inspection. A failed or released candidate needs a new
preparation for further checks, preserving all earlier receipts.

These are cooperative same-UID controls. Other stores, direct shell processes,
escaped process sessions and malicious file edits are outside their isolation
boundary. Hashes and process-group observation do not create protected evaluation.
Supported checks keep their descendants in the runner's process group and finish
external services before exiting. The plan, receipts and status state this
boundary. A Linux subreaper detects ordinary detached orphan children on clean
runner completion and retains the lease as unsupported work. Recovery requires
their observed exit too. Abrupt runner loss can prevent that observation; this
does not prove that detached services or external worlds have stopped.
Configuration and input IDs are supplied declarations frozen into the plan.
Commit/tree/index/status, executed argv, exit codes and recorded output hashes
are observed identities. Checks must inspect the actual relevant configuration
and inputs to substantiate broader runtime claims.
Task 016 still owns the real coding-client and production-tool release pilot.
