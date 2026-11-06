# Research Program feature

This directory owns the program manager, reviewer and orchestration code.
Program investigations go to `research/sessions/` and manager notes to
`research/programs/PROGRAM_ID/manager/`.
Hangprinter research, mission prompts and saved findings belong in
[`research/`](../research/). New session notes belong in that session's
`scratchbook/`, beside its report and evidence.

| Path | Contents |
| --- | --- |
| `role-instructions/MANAGER.md` | The manager's complete role instructions |
| `role-instructions/REVIEWER.md` | The reviewer's complete role instructions |
| `program/` | Python orchestration package: entry, store, host, workers, review, strategy, graph, controller, pool and integration |
| `scripts/` | CLI entry and child-process runners |
| `tests/` | Program tests; no live research or paid model calls |
| `reference/` | Tool protocols and implementation details |
| `archive/` | Historical design and implementation audits; never runtime instructions |
| `early-core-ideas.md` | A short reading of the original design |

[Launch and use a program](../human_docs/research-programs.md).
[Cleanup findings](cleanup-notes.md) explain the old layout and remaining machinery.
The root `hp-sim5-research-program` is a thin executable entry point.
The existing research agent and shared simulation tools remain in
`src/python/hp_sim5_research/` and `scripts/research_agent.py`.
Python package identifiers use underscores; the project name is hp-sim5.
The program package here is simply `program`.

Run the program checks from the repository root:

```bash
.venv/bin/python -m pytest -q research-program-feature/tests
```
