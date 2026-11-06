# Research Program feature

A thin layer around the independent researcher: choose a question, delegate it,
review the result, check the useful change and squash it onto main.

| Path | Purpose |
| --- | --- |
| `role-instructions/MANAGER.md` | Research direction and communication with the human |
| `role-instructions/RESEARCHER.md` | Independent hypotheses, experiments, debugging and improvements |
| `role-instructions/REVIEWER.md` | Separate review and help delivering useful changes |
| `learned-wisdom/for-managers/`, `for-researchers/`, `for-reviewers/` | Optional proven hints, roughly 4–12 words per item |
| `program/` | Launcher, private store, workers, review and integration |
| `scripts/` | CLI entry and child-process runners |
| `tests/` | Fixture clients and real Git/process checks; no paid model calls |
| `reference/` | Implementation details, read when needed |
| `archive/` | Historical designs and audits; never runtime instructions |

Results live in `research/sessions/SESSION/`. The manager writes only a short
`research/programs/PROGRAM_ID/summary.html` linking to them. Controller state and
launch transcripts stay in ignored `.research-programs/` storage.

[Launch and output guide](../human_docs/research-programs.md).
[Exactly what enters each role's context](reference/context.md).
[Tool reference](reference/tools.md).

Run checks with `.venv/bin/python -m pytest -q research-program-feature/tests`.
