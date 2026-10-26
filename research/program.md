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
./hp-sim5-research-agent --codex --name 'One investigation' --viewer none
```

To try the complete management cycle cheaply, ask for one saved-evidence
investigation, a separate review, and a decision, with no new numerical work.
The worker currently starts its normal supervisor even for saved-evidence work.
A later bounded production experiment can test physics-tool integration.

## Use the current cloud chat

Ask the authenticated cloud chat:

```text
Read research/prompts/research-manager.txt and scripts/cloud/RESEARCH.md.
Manage this question: [question]. Use cloud subagents for workers and reviewers.
```

The chat runs:

```bash
./hp-sim5-research-program --cloud --question 'Your research question'
```

This does not open another coding client. The current conversation is the
manager. The controller starts the existing research launcher; its model request
goes directly to a queue that the parent dispatches through authenticated
subagent tools. No second login, API key, Codex-home write grant or PATH wrapper
is required. See the [cloud instructions](../scripts/cloud/RESEARCH.md).

Keep the chat working through each bounded cycle. Background services manage
owned processes and deadlines, but do not conduct research without the manager.
CLI model/provider flags do not configure cloud actors. Cloud mode refuses
nested interactive Codex/Claude manager launches. An explicit --cloud is useful
on platforms that do not expose a recognizable cloud runtime marker.

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
```

This writes `research/programs/PROGRAM_ID/`: program memory, a readable overview,
worker notes, referenced small evidence and fixed review submissions. It commits
only that directory, preserving unrelated staged work. It never pushes. Code
changes belong in their own explicitly selected commits.

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

A fixed (historically called sealed) submission is a copy of a claim's report,
abstract and selected evidence at review time. Its hashes establish exactly what
was reviewed, not that the claim is true. Editing paths inside a manifest changes
its identity too. Keep such records unchanged; resolve relocated files separately.
There is no "ignore integrity and keep reviewed status" import option.

## What has been demonstrated

The [October 24 cloud pilot](cloud-client-pilot/2026-10-24/README.md) used a
prototype transport: two investigations and separate reviews, one small native
physics operation, interruption and fresh-manager reconstruction. It did not
demonstrate scientific improvement. That prototype's PATH wrapper and independent
ten-minute timeout have been removed. The current transport and local manager
need live trials on their actual platforms; fixture tests alone cannot establish
that authenticated model sessions work.

Parallel pools and combined-patch checks remain available as advanced tools.
Start with one worker and a separate reviewer. The historical development cards
record earlier implementation work and failures; they do not expand this workflow
into an independent background research service.
