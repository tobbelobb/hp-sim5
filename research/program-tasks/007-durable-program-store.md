# 007: Create and inspect a durable research program

Outcome: a chat agent can create a real program from one question, record routes
and bounded assignments, and reconstruct its state in a fresh process. Implement
the first vertical slice of [the manager contract](MANAGER.md), not just schemas.

Start from archive.py, existing session artifacts and the launcher contract.
Introduce a small program module and `hp-sim5-research-program` command with
structured create/open, status/overview, question/approach and assignment commands.
Exact spelling is an implementation choice; document the implemented commands.
Keep the execution/review/knowledge states separate. Include IDs, revisions,
timestamps, program constraints and append-only decisions/steering.

Use SQLite transactions or demonstrate equivalent atomic behavior. Include a
schema version, safe initialization/reopen, readable JSON/Markdown export and a
minimal import/version error path. Keep large scientific artifacts external.
Authoritative storage must have a clear location/owner and must not silently
follow worker-writable links. No physics world should start to inspect a program.

Done when:

- The question alone creates a program; optional constraints are preserved.
  Missing numerical caps allow a manager to record a targeted allocation rather
  than becoming implicit zeros. No worker runs without a recorded finite allocation.
- Another process sees the same overview, IDs, live alternatives and assignments.
  Invalid transitions and duplicate identities fail without partial state.
- Concurrent updates do not lose records; interrupted writes leave a recoverable
  program. Export/reopen preserves decisions and evidence references.
- Focused temporary-store/CLI checks demonstrate the above, including a pending
  program honestly reporting no scientific result. The next task can use actual
  APIs instead of inventing another store.

Use fixtures, no model/physics runs for storage correctness. Make one task-owned
commit with queue/progress updates. Graph semantics, worker launching and manager
chat reasoning belong to the later dependent cards.

## Implemented outcome (2026-10-23)

Implemented `hp-sim5-research-program`, `scripts/research_program.py` and the
stdlib `hp_sim5_research.program.ProgramStore` API. The
[command/store guide](../program-store.md) documents actual commands, state
transitions, revision checks, finite assignment allocations and storage ownership.
SQLite schema/version 1 lives in manager-owned `.research-programs/`, outside
worker-shared output. JSON export/import preserves IDs, revisions, decisions,
steering and external evidence references; Markdown renders the complete view.

Verification: `.venv/bin/python tests/python/test_research_program.py -v` passed
10 offline tests. Fresh CLI processes reconstruct the same program, two routes
and a bounded assignment; completed execution remains unreviewed/unassessed.
Four processes initialize concurrently; four writers retain 60 distinct decisions
and all revision increments. Killing a writer during a transaction leaves no
partial note/revision and reopens with SQLite integrity `ok`. Duplicate IDs,
stale revisions, invalid transitions/allocations, import/version errors and linked
or unsafe store paths fail without partial program state.

No numerical/model/physics work was needed. Recorded allocations are not yet
runtime enforcement; artifact references do not verify bytes. Same-user workers
are not adversarially isolated. Graph, dispatch and reviewed admission remain
the dependent cards, and task 016 remains the manager release gate.
