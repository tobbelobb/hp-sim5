# Research programs

A manager chooses useful questions, launches researchers, arranges separate
review and delivers one checked squash on main. Researchers own their hypotheses,
experiments and fixes, and can also work independently.

```bash
./hp-sim5-research-program --codex --question 'Your question'
# Or use --claude.
./hp-sim5-research-program --codex --program PROGRAM_ID
```

Keep the returned program ID. Reopening restores tool state; continue an existing
model conversation in its coding client. Add `--grill-me` to discuss goals and
authority before dispatch, or `--ask-questions` for consequential choices during
work. Both default to off.

Omit `--codex` and `--claude` to manage from an existing chat. The response supplies
manager instructions and `attachment.connection`. Use that path as CONNECTION:

```bash
./hp-sim5-research-program tool CONNECTION overview
./hp-sim5-research-program tool CONNECTION capabilities
./hp-sim5-research-program tool CONNECTION commission --params '{"objective":"Investigate the timeout and fix its cause","settings":{"machine":"hp3"}}'
./hp-sim5-research-program tool CONNECTION worker_start --params '{"assignment_id":"ASSIGNMENT_ID"}'
```

Calls return while work continues. Inspect status before retrying a timed-out
change. Commissioned researchers use a separate worktree by default.
A question and launcher settings suffice; there is no route-planning or
budget-accounting step. Human constraints remain part of the task.

## Where the output goes

```text
research/output/
  index.md                         # One catalog: title, result sentence, report link
  session-SESSION/                 # A standalone launcher session
    report.md                      # Canonical researcher explanation and evidence links
    abstract.md                    # Title and one result sentence for search/cataloging
    scratchbook/                    # Optional notes and small helpers
    ...                            # Original data, plots and runtime evidence
  program-PROGRAM_ID/
    summary.html                   # Short human result, plots if useful, links to sessions
    session-ATTEMPT/               # A researcher session dispatched by the program

.research-programs/                 # Ignored tool state, launches, reviews and checks
```

Read the program summary first, then follow its links to the original reports.
There is no second program report or narrative ledger. Regenerate the catalog
with `.venv/bin/python scripts/research_index.py`; it updates
`research/output/index.md`. All research output lives under `research/output/`
so it can be browsed, searched and ignored as one tree.

The reviewer checks the researcher's existing report, evidence and committed
candidate. The manager and reviewer resolve gaps, include the useful evidence
and summary in the final candidate, and run the relevant checks on it.
`integration_finish` delivers that checked tree as one squash commit on main and
marks the program completed. Main must still match the reviewed base and its
worktree must be clean. If either changed, prepare and check the updated candidate.
The integration tools do not push; delivery follows the human's stated authority.
A negative result can be delivered as a reviewed report and evidence change.

Pause prevents new dispatch and leaves current jobs running. Stop requests
cancellation; inspect status until cleanup finishes. Closing the manager chat
alone leaves the controller running. Launching a program starts no physics world.

The independent launcher uses the same researcher role:

```bash
./hp-sim5-research-agent --codex --machine hp3 --prompt 'Your research question'
```

Each role can invent small helpers as needed and leave proven, roughly 4–12 word
hints in its `research-program-feature/learned-wisdom/for-ROLE/` directory.

[Role context trace](../research-program-feature/reference/context.md) ·
[Tool reference](../research-program-feature/reference/tools.md)
