# Research programs

A research program is a continuing question managed in a coding chat. The manager
chooses useful investigations, delegates them to the existing research agent,
asks a separate reviewer to inspect the evidence, and explains what to try next.
The human supplies the question and steers the conversation.

## Start locally

```bash
./hp-sim5-research-program --codex --question 'Your research question'
# Or use the installed Claude client:
./hp-sim5-research-program --claude --question 'Your research question'
```

The manager uses ordinary CLI calls. It starts no physics world and needs no
manager MCP server. Workers still use the existing launcher and physics tools.
The original direct interface remains available:

```bash
./hp-sim5-research-agent --machine hp4 --codex --name 'One investigation' --viewer none
```

## Codex Cloud adapter

In a cloud chat, say “Launch a Research Program on this question: …”. The
[cloud skill](../.agents/skills/cloud-research-client/SKILL.md) owns its entry,
dispatch, setup scripts and adapter. Opening a cloud program returns the same
shared manager instructions that the local launcher injects. The optional
[boundary map](../.agents/skills/cloud-research-client/references/boundary.md)
documents the core attachment points.

## Continue and steer

Say "continue", "what do we know?", "why this approach?", "try this idea",
"pause new work", "stop that worker" or "stop the program". Pause holds new
dispatch; running work continues to its deadline. Stop requests cancellation,
and the manager checks actual cleanup before declaring the work stopped.

Keep the returned program ID. In the same checkout, open it again with:

```bash
./hp-sim5-research-program --codex --program PROGRAM_ID
# From an existing cloud manager chat:
./hp-sim5-research-program --cloud --program PROGRAM_ID
```

Default live memory lives in `.research-programs/programs.sqlite3`. Worker notes
and evidence live under `output/research/`. Reopening memory restores neither a
model conversation nor a physics world. Moving a partly finished program between
machines is not a primary workflow.

## Save research in Git

At a completed iteration or before ending the program, finish owned cleanup and
update research.md, report.md and abstract.md. Then save the small results:

```bash
./hp-sim5-research-program save PROGRAM_ID --commit
# Include additional relevant analysis scripts, inputs or configuration:
./hp-sim5-research-program save PROGRAM_ID --file path/to/analysis.py --commit
# Inside a manager chat, use its bound connection directly:
./hp-sim5-research-program tool CONNECTION save --params '{"commit":true,"files":["path/to/analysis.py"]}'
```

This writes `research/programs/PROGRAM_ID/`: program memory, a readable overview,
worker notes, referenced small evidence and fixed review submissions. It commits
only that directory, preserving unrelated staged work. It never pushes. Code
changes belong in their own explicitly selected commits. Both command forms ask
the controller to perform the Git operation in its own execution context.

Runtime stores matching `.research-programs*/` and generated `research/programs/`
archives are ignored by default. An explicit save --commit stages just that
program's archive with git add --force. This keeps abandoned trial archives out
of broad commits while allowing deliberate research-result commits. Saving without
--commit leaves the archive on disk and ignored.

The default copying limits are 1 MiB per file and 8 MiB total evidence. Missing,
large and private/runtime files are listed in files.json instead of copied.
Inspect those omissions: deliberately retain any larger evidence needed to
support the findings separately. Explicit --file selections fail if unsafe,
missing or too large. Credentials, live connection files and dependency trees
are not research results. These Git records preserve findings and provenance;
they are not a portable running environment.

## Inspect without starting agents

```bash
./hp-sim5-research-program overview PROGRAM_ID
./hp-sim5-research-program query PROGRAM_ID open_questions
./hp-sim5-research-program query PROGRAM_ID support CLAIM_ID
./hp-sim5-research-program export PROGRAM_ID --format markdown
```

Use `--store DIRECTORY` before an operation for a separate trial store.
The [tool guide](program-manager.md) documents lower-level operations.
