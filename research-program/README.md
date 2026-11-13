# Research programs

A program is a research question, a living agenda, and investigations with evidence.
Your coding platform runs the conversations and subagents. This directory contains
no model loop, database, worker scheduler, or automatic merge machinery.

| File | What it does |
| --- | --- |
| `cli.py` | Opens programs, prepares researcher handoffs, and selects review evidence |
| `roles/` | The complete, short instructions injected for each role; edit these directly |
| `evidence.py` | Seals selected evidence and a committed code revision for independent review |
| `platforms.md` | T3 Code, Codex Cloud, and Anthropic setup and agent lifecycle |
| `manual-tests.md` | Manual acceptance runs, expected results, and a small platform research trial |
| `cloud/` | Optional Linux runtime installation; no model credentials |
| `tests/` | CLI, Git worktree, resume, evidence, and runtime lifecycle checks |
| `learned-wisdom/` | Earlier short implementation lessons, retained for relevant investigations |
| `archive/` | Historical designs, assessments, and evidence; never runtime instructions |

Research method and publishing guidance live in the portable
[research-program](../.agents/skills/research-program/SKILL.md) and
[research-corpus](../.agents/skills/research-corpus/SKILL.md) skills. Claude's skill
directory links to the same files. Role instructions remain short and are intended
for human revision. Only the selected role and bound investigation paths are injected;
platform guidance, historical documents, and the corpus are read when relevant.

## Start and resume

In T3 Code or a hosted chat, use the current authenticated conversation:

```bash
./hp-sim5-research-program --question 'Improve calibration reliability'
./hp-sim5-research-program --resume PROGRAM_ID
./hp-sim5-research-program --resume                 # latest on this checkout's host
```

The JSON response supplies `instructions`, `task`, the program directory, and its
agenda. Read these in the current conversation. `--cloud` explicitly selects this
same existing-chat route. No coding CLI or physics dependencies are needed.

For an ordinary terminal, `--codex` or `--claude` opens a manager conversation:

```bash
./hp-sim5-research-program --codex --question 'Improve calibration reliability'
./hp-sim5-research-program --codex --resume PROGRAM_ID
```

Program resume restores the question, agenda, and membership. Backend conversation
resume is separate: Codex offers its conversation picker when no ID is bound.
Continue T3 and hosted conversations in their own UI. Program records do not pretend
to restore a platform conversation or a stopped physics world.

## Parallel researchers

```bash
./hp-sim5-research-program --resume PROGRAM_ID --agent 'Investigate observability'
./hp-sim5-research-program --resume PROGRAM_ID --agent 'Test a simpler baseline'
./hp-sim5-research-program --resume PROGRAM_ID --agent-id INVESTIGATION_ID
```

Each call returns one portable handoff: `instructions`, `task`, `id`, and
`session_dir`. Give the instructions and task to a fresh platform agent. T3 handles
provider choice, isolated workspaces, steering, waits, and cancellation; Codex Cloud
and Anthropic use their authenticated agents. These independent investigations can
run concurrently. They do not need physics unless their question calls for it.
Follow [platforms.md](platforms.md) for workspace binding and tools.

`agenda.md` records what matters, current understanding, alternatives, and the next
decision. Reconsider it after informative results, persistent surprises, diminishing
returns, or changed priorities. A useful negative result can finish an investigation.
The CLI updates membership atomically; agents should coordinate edits to the shared
agenda and topic syntheses.

## Independent researcher and tools

Both original entry points remain. Starting a researcher without a backend prepares
a handoff. With a backend it opens an ordinary conversation and a private supervisor:

```bash
./hp-sim5-research-agent --prompt 'Investigate the observed failure'
./hp-sim5-research-agent --codex --prompt 'Investigate the observed failure'
./hp-sim5-research-agent --claude --worktree --prompt 'Investigate the observed failure'
./hp-sim5-research-agent --resume INVESTIGATION_ID
```

`--resume` accepts an ID or existing artifact directory. With no value it selects the
last opened investigation on this host. Use explicit IDs when researchers run in
parallel: commissioning and status commands can change this convenience pointer. `--session-dir PATH` selects a new artifact path.
Opening creates private metadata, not an empty corpus directory or placeholder
abstract. Artifact directories appear when research writes a report or an experiment.
Reopening never overwrites reports. Runtime logs and connection files stay under
ignored `.research-state/`; substantive evidence stays in the assigned investigation.

Old forwarded client resume flags no longer silently allocate another directory.
For an old conversation, identify both identities once:

```bash
./hp-sim5-research-agent --codex --resume research/output/OLD_SESSION --conversation CLIENT_ID
```

Subsequent `--resume` reuses the artifact path and known backend ID. Batch Codex
captures IDs from its event stream; Claude assigns a session ID before launch.
Interactive Codex without a bound ID offers its native resume picker. The launcher's
resume restores investigation storage even if a new backend conversation is needed.

A worktree retains tracked research and writes new evidence in its own checkout.
Only ignored runtime dependencies are linked. A platform-created worktree must bind
a prepared handoff with `--resume ID --workspace` before starting its runtime. Resuming a launcher-created worktree
returns to that worktree; it does not allocate another one. Physics worlds are fresh
unless the existing supervisor is still running.

## Fixed evidence for review

Write a selection JSON with repository-relative files and a committed candidate:

```json
{
  "claim": "The baseline explains the apparent improvement",
  "scope": "Recorded HP3 measurements; no hardware validation",
  "base_revision": "COMMIT_BEFORE_CHANGE",
  "candidate_revision": "COMMIT_TO_REVIEW",
  "files": ["research/output/PROGRAM/session-ID/report.md", "research/output/PROGRAM/session-ID/results.json"],
  "limitations": ["One configuration"]
}
```

```bash
./hp-sim5-research-program --resume PROGRAM_ID --review selection.json
./hp-sim5-research-program --resume PROGRAM_ID --verify REVIEW_DIRECTORY
```

Give the returned handoff to a fresh reviewer. The bundle contains exact selected
bytes, SHA-256 checksums, the candidate commit/tree, and its diff. At most 64 files
and 32 MiB are copied; reference large recordings by durable URI and checksum.
Every selected file must be committed in the candidate; the bundle reads its Git blob,
even when that commit came from another researcher’s worktree. Uncommitted edits are excluded. Checksums establish integrity,
not scientific validity. This is cooperative same-user tooling, not an adversarial
security boundary. Reviewers write their actual assessment and checks to review.md.

Delivery uses ordinary Git/PR operations with the human's authority. Run relevant
checks on the actual final candidate after combining changes; physics changes need
`npm run test:physics`. A review does not authorize a push or automatic merge.

## Storage and portability

```text
research/output/program-ID/
  program.json                 # question and researcher membership, relative paths
  agenda.md                    # current direction and evidence links
  session-ID/                  # created when its researcher writes evidence
    report.md
    abstract.md
    native/                    # independent worlds and experiment artifacts
  reviews/review-ID/            # selected fixed evidence and independent assessment
.research-state/                # ignored sessions, locks, launch logs, live connections
```

Commit the useful reports, agenda, manifests, and topic syntheses. Keep large raw
recordings on durable storage with checksummed references. On another host, resume
the program by directory and use `--agent-id` to reconstruct a researcher's local
metadata. The corpus skill explains synthesis and preservation.

The viewer now defaults to `none`; request `--viewer headless` or `--viewer window`
when needed. Sidebar grouping is opt-in through `--sidebar-section UUID`.

Legacy program.json records with nested `program.id` and `program.goal` can be
reopened with an empty new membership list. Archives without program.json remain
readable evidence; carry useful questions into a new program.

The former `research-program-feature` controller and cloud model queues have been
removed. Existing reports and archives are preserved. Old ignored SQLite state is
not imported or deleted; retain it if needed, and carry useful questions/evidence
into a new plain program rather than restarting historical PIDs or queued actors.

Run `.venv/bin/python -m pytest -q research-program/tests` for this layer. No paid
model calls are part of the suite.
