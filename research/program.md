# Research programs

A research program is a continuing question managed in a coding chat. The manager
chooses useful investigations, delegates them to the existing research agent,
asks a separate reviewer to inspect the evidence, and explains what to try next.
The human supplies the question and steers the conversation.

For actors, prompts, tools and a measured cloud timeline, open the
[visual report](reports/research-program-cloud.html). The
[adapter boundary map](../scripts/cloud/BOUNDARY.md) separates platform code
from the local core.

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

## Codex Cloud adapter

The optional [Codex Cloud adapter](../scripts/cloud/RESEARCH.md) uses the current
authenticated conversation as manager. Its prompts, dispatch instructions,
setup and permission workarounds live under `scripts/cloud/`; the
[boundary map](../scripts/cloud/BOUNDARY.md) inventories the few core attachment
points. Both routes use the shared `research/prompts/research-manager-workflow.txt`.

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

A fixed (historically called sealed) submission is a copy of a claim's report,
abstract and selected evidence at review time. Its hashes establish exactly what
was reviewed, not that the claim is true. Editing paths inside a manifest changes
its identity too. Keep such records unchanged; resolve relocated files separately.
There is no "ignore integrity and keep reviewed status" import option.

## What has been demonstrated

The [October 26–27 audit](programs/program-7aa80aa18f10499e85cb24ffeae712cb/selected/output/research/sessions/2026-10-26-cloud-platform-audit/report.md)
records actual workers, separate fixed-evidence reviews, native experiments,
collection, Rerun and browser inspection through the current cloud relay. Its
final repaired-source production regression was interrupted. Read its tested
scope and remaining gap before treating the whole workflow as validated.
The [HTML report](reports/research-program-cloud.html) presents that evidence as
a timeline. Fixture tests do not establish authenticated model access.

The [October 24 pilot](cloud-client-pilot/2026-10-24/README.md) and development
cards retain earlier experiments and superseded transport designs.

Parallel pools and combined-patch checks remain available as advanced tools.
Start with one worker and a separate reviewer. The historical development cards
record earlier implementation work and failures; they do not expand this workflow
into an independent background research service.
