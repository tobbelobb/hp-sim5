# Manual acceptance runs

Run from the repository root on the pushed `main`. Use a fresh terminal or chat
for each test. Replace capitalized IDs and paths below with the returned values.
Use explicit investigation IDs: bare `--resume` follows a host-wide convenience
pointer that other researchers can change. Keep useful evidence; these tests do
not authorize committing or pushing their outputs.

## 1. Opening and reopening without empty artifacts

```bash
./hp-sim5-research-agent --prompt 'Manual check: opening without experiments'
./hp-sim5-research-agent --resume INVESTIGATION_ID
```

Compare the two JSON responses: `id` and `session_dir` must match. The returned
artifact directory must not exist yet. Private metadata under `.research-state/`
is expected. No coding client or physics process should start.

Then try the actual client lifecycle:

```bash
./hp-sim5-research-agent --codex --prompt 'Manual opening test. Do not write files or configure a world. Wait for me to exit.'
./hp-sim5-research-agent --codex --resume INVESTIGATION_ID
```

Use the new ID printed by this client launch. Exit the first conversation without
doing research, reopen, then exit again. There should still be no artifact folder.
Interactive Codex can show its conversation picker; choose the original chat.
Repeat with `--claude` if desired. The launcher preserves investigation identity
independently of the client's conversation identity.

## 2. Worktree and report continuity

```bash
./hp-sim5-research-agent --codex --worktree --prompt 'Manual storage test. Write report.md in the assigned artifact directory containing "First visit: storage smoke test, no scientific finding." Do not create a physics world, modify code, commit, or push. Then stop.'
```

Use `--claude` instead if preferred. Record the investigation ID and worktree path
printed by the launcher. After exiting, inspect that worktree:

```bash
git -C WORKTREE_PATH status --short
./hp-sim5-research-agent --codex --resume INVESTIGATION_ID --prompt 'Append "Second visit: resumed the same investigation." to the existing report.md. Do not overwrite its first line, create another investigation, configure physics, commit, or push. Then stop.'
git -C WORKTREE_PATH status --short
```

Expected: tracked research remains present, with no mass of deleted reports.
There is one new investigation folder in that worktree, not a symlink to the main
checkout's research output. Its report contains both visits. Resume returns to the
same worktree without allocating a second one. Keep the worktree until any useful
evidence has been preserved; deleting it also deletes uncommitted research there.

## 3. Two independent, optional physics worlds

This requires the simulation dependencies, but no model calls. Start two prepared
investigations and note their distinct IDs:

```bash
./hp-sim5-research-agent --prompt 'Manual world A'
./hp-sim5-research-agent --prompt 'Manual world B'
./hp-sim5-research-agent --resume ID_A --start-runtime --no-record
./hp-sim5-research-agent --resume ID_B --start-runtime --no-record
```

Copy each returned `tool_connection` path. Before configuring either world:

```bash
.venv/bin/python scripts/research_tools.py call CONNECTION_A runtime_status
.venv/bin/python scripts/research_tools.py call CONNECTION_B runtime_status
```

Both should report no world (`world_id` is null), and neither investigation should
have an artifact directory. Configure A, then confirm B is still unconfigured:

```bash
.venv/bin/python scripts/research_tools.py call CONNECTION_A configure_session --params '{"configuration":{"machine":"hp3"}}'
.venv/bin/python scripts/research_tools.py call CONNECTION_B runtime_status
.venv/bin/python scripts/research_tools.py call CONNECTION_B configure_session --params '{"configuration":{"machine":"hp4"}}'
.venv/bin/python scripts/research_tools.py call CONNECTION_A runtime_status
.venv/bin/python scripts/research_tools.py call CONNECTION_B runtime_status
```

Inspect the returned settings, distinct world IDs, and separate evidence paths.
World A must retain its ID after B is configured. Look through each world's
`manifest.json` under its investigation's `native/` directory.

Always stop both owned runtimes, including after a failed step:

```bash
./hp-sim5-research-agent --resume ID_A --stop-runtime
./hp-sim5-research-agent --resume ID_B --stop-runtime
./hp-sim5-research-agent --resume ID_A --runtime-status
./hp-sim5-research-agent --resume ID_B --runtime-status
```

Both should report `running: false`. Restart A with the same start command: the
investigation path stays fixed, previous evidence remains, and the new runtime has
no world. Stop it again when finished. `--no-record` intentionally skips Rerun;
use `--viewer headless` without `--no-record` for a separate recording inspection.

## 4. Real platform delegation and corpus judgment

Paste this into a new T3 Code conversation. Try the same prompt in Codex Cloud
after selecting this repository and the pushed revision:

> Use the research-program and research-corpus skills. Start a small program to
> identify one important unresolved uncertainty in our existing calibration
> research. Use saved evidence only: no new simulation, fitting, or collection.
> Delegate exactly two independent researchers through this platform: one audits
> the strongest existing improvement claim; the other looks for contradictory or
> negative evidence and a simpler explanation. Give each a separate investigation
> and a fresh context. Use isolated platform workspaces when available. If this
> platform cannot delegate, report that limitation instead of launching another
> coding CLI. Have a fresh reviewer inspect the resulting evidence and conclusions.
> Preserve the evidence already present. Update the agenda with the most useful
> next question. Draft a topic synthesis update only if the evidence changes our
> understanding. Do not commit or push. Return the program ID, actual artifact
> paths, relevant code/evidence revisions, and links to reports and the agenda.

Look through the actual platform child conversations, not just the manager's
summary. Each researcher should have one stable artifact path and specific source
references. The reviewer should inspect evidence and explain what it did and did
not reproduce. The final agenda should make a decision about research direction.
Check for contradictions being preserved, appropriate limits on simulation claims,
and topic notes linking original reports instead of duplicating them.

Reopen the same platform conversation and ask:

> Resume program PROGRAM_ID. Reopen its existing researchers as needed. Explain
> what we learned and which observation would most change the next decision.
> Do not commission new investigations or run experiments.

Expected: the same agenda, membership, and researcher artifact paths. Existing
evidence is retained. Platform conversation resume happens in the platform UI;
the CLI does not reconstruct a lost platform conversation.

## 5. Independent review of a committed candidate

After you have inspected and committed a useful small report, follow the selection
JSON example in [README.md](README.md#fixed-evidence-for-review). Select that exact
commit and report, then run:

```bash
./hp-sim5-research-program --resume PROGRAM_ID --review SELECTION_JSON
./hp-sim5-research-program --resume PROGRAM_ID --verify REVIEW_DIRECTORY
```

Look through `manifest.json`, `candidate.diff`, and the selected `files/`. They must
match the committed candidate even if the checkout has later edits. Verification
checks bytes and reports scientific validity as unassessed. Give the returned
handoff to a fresh platform reviewer; its written assessment supplies the scientific
review. Uncommitted reports cannot be included in this committed snapshot.

## Automated checks to rerun locally

```bash
.venv/bin/python -m pytest -q research-program/tests tests/python
.venv/bin/python -m pytest -q research-program/tests/test_runtime.py -m slow
```

The second command creates real independent worlds in temporary fixture checkouts
and checks cleanup. It makes no model calls. Anthropic managed deployment still
needs a separate infrastructure trial; these local checks do not establish that
deployment works.
