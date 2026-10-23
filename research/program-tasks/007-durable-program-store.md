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
