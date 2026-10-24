# Start a research program from a question

The manager implementation is available above the existing
`hp-sim5-research-agent`. Tasks 007 through 015 implement durable graph memory,
approach selection, bounded workers, separate review, recovery and combined patch
checks. The full release is not yet verified: [task 016](program-tasks/016-live-manager-release.md)
is blocked on native build and coding-client prerequisites. See the
[actual pilot and limits](../output/research/sessions/2026-10-24-live-manager-release/report.md).

## Give the manager only a question

In a repository-connected coding chat, send:

```text
Read research/prompts/research-manager.txt and manage this question:
Can encoder-only HP3 calibration improve from the same measurements?
```

Replace the question with yours. Optional constraints can follow it. The agent
creates or resumes a program, inspects useful prior evidence, develops competing
approaches and chooses finite assignments. It delegates through the existing
worker and incorporates separately reviewed results into the graph. The agent
handles the tool calls; you do not need to prepare worker prompts or a work board.
Plain chat without repository/tool access cannot execute these operations.

Then say "continue", "what do we know?", "why this approach?", "what supports
that claim?", "try this idea using the same measurements", "pause", "stop that
worker" or "stop the program". Pause holds new dispatch while existing work keeps
running within its deadlines. Stop requests owned cleanup and reports its actual
boundary. To move chats, say:

```text
Read research/prompts/research-manager.txt. Resume program PROGRAM_ID.
Inspect the overview and reconcile ownership before continuing.
```

Keep the returned program ID. Reopening does not replay old assignments or restore
a physics checkpoint. An unknown old process requires an explicit bounded handoff.
The [manager tool guide](program-manager.md) explains each operation.

Dedicated manager chats use the existing installed client and authentication:

```bash
./hp-sim5-research-program --codex --question 'Your question'
./hp-sim5-research-program --claude --program PROGRAM_ID
./hp-sim5-research-program --serve --program PROGRAM_ID
```

An existing coding chat uses the same tools through CLI calls, with automatic
file-host startup and no manual terminal/server setup. `--serve` writes MCP
attachment configuration for a client supporting it. The manager itself starts
no physics world or Viewer.

## Memory, evidence and code

The default store is `.research-programs/programs.sqlite3`. It contains program
goals, questions, approaches, assignments, attempts, graph relations, dated route
decisions, steering and review/integration history. Large artifacts remain in
research session directories and are referenced by identity/hash. This is a typed
queryable graph in SQLite; no graph service or model API is required.

Ask the agent for scoped support, challenges, unknowns or the consequences of
withdrawing evidence. Alternative derivations and shared source identities remain
visible. A new simulator revision does not retroactively establish applicability.
The [graph guide](program-graph/README.md) describes these rules.

You or the agent can inspect readable exports without starting a world:

```bash
./hp-sim5-research-program overview PROGRAM_ID
./hp-sim5-research-program query PROGRAM_ID open_questions
./hp-sim5-research-program query PROGRAM_ID support CLAIM_ID
./hp-sim5-research-program export PROGRAM_ID --format markdown
./hp-sim5-research-program export PROGRAM_ID --format json
```

Place `--store PATH` before the operation for another store. JSON exports preserve
history; artifacts still need the [portable evidence handoff](handoffs.md).
Execution success does not admit a claim. Separate review fixes evidence versions
before admission, and [strategy](program-strategy.md) uses effective reviewed
results to revise routes. [Patch integration](program-integration.md) has separate
readiness records. Failed checks or conflicts preserve useful scientific findings.
Research patches are prepared in separate worktrees and are never auto-merged or
pushed by these tools.

Fresh fits, headless/native physics and simulated collection remain allowed.
The manager chooses targeted finite budgets from workload costs and uses supported
`--speedup 25` for simulated autocal CLI when appropriate. Read the
[recording-cost guidance](autocal-recording-performance.md). The
[controller](program-controller.md) enforces reservations, slots, review backlog
and owned wall deadlines. Tool limits apply to supported calls. Numerical estimates,
worker reports and measured observations stay distinct; arbitrary shell movement
and spend are not hard aggregate quotas. The pool defaults to one worker and
supports up to four with explicit store/allocation capacity.

## Prerequisites and current release block

Ordinary program CLI and graph operations need the local `.venv/bin/python` and
standard library. Dedicated chat needs an installed supported coding client with
working existing authentication and writable client state. MCP needs the repository's
supported SDK and working stdio/event-loop facilities. Workers need the normal
[research runtime setup](README.md#setup-and-interactive-use), including native
build tools and Node development headers. The current launcher starts a supervisor
even for a saved-data fitting assignment.

The 2026-10-24 actual pilot created a program, launched that worker in an isolated
worktree, queried the graph, exercised steering/lifecycle commands and reopened
durable state. Native build stopped at missing Node-API headers. Dedicated Codex
and separate review stopped at read-only app-server initialization; stock MCP
initialization timed out. No numerical investigation or scientific admission
completed. Two reviewed investigations, production-tool use, observed live stop
and fresh-chat continuation remain required for release. The report lists exact
unmet criteria and the configuration bug fixed during these attempts.

## Single-agent alternative

The original one-investigation workflow remains available. It uses ordinary
research notes and an agenda in the same agent conversation:

Give an agent at this repository's root your question and the saved entry prompt:

```text
Read research/prompts/research-program.txt and use that workflow to investigate:

[My research question]
```

That is enough to start. One agent finds relevant prior work, chooses one
economical investigation, carries it through within budget, checks its claim,
and saves the evidence and next decision. A broad question can need several
iterations. You receive what changed, what remains uncertain and which decision
deserves attention next.

Fresh fits, headless/native physics runs and simulated data collection are
allowed by default. The agent starts from useful prior evidence and uses
targeted experiments whenever they can resolve the question. If you supply no
numerical limits, it chooses and records a finite budget for the smallest useful
probe, then executes it. You can add your practical goal, constraints and limits:

```text
Read research/prompts/research-program.txt and investigate:
Can encoder-only HP3 calibration become more accurate from the same measurements?

Preserve one-click operation and automatic stopping. Reuse saved data when useful.
Run targeted fresh fits, physics or simulated collection when necessary.
Choose and record a finite experiment budget before numerical work.
Use --speedup 25 for supported simulated autocal CLI runs and prefer headless
native Rust; choose recording detail to suit the question.
Complete one useful iteration and update the agenda with the result.
```

Prefer quick, cheap probes over large unfocused sweeps. Use a small pilot to
estimate cost when necessary. Headless physics already runs as fast as possible;
speedup settings must preserve simulation-time waits/deadlines and the intended
timestep/substeps. Use the [recording-cost guidance](autocal-recording-performance.md)
to avoid unnecessary capture overhead. The agent preserves negative and
inconclusive results and stops at the recorded limits. Your explicit constraints
always take precedence.

## Choose how far to go

| What you want | Prompt |
| --- | --- |
| Question to one completed iteration | “Read research/prompts/research-program.txt and investigate: [question].” |
| Discuss a plan before experiments | “Use research/prompts/next-assignment.txt for [question]. Return the proposal only.” |
| Continue in the same conversation | “Continue this program for one more iteration within [budget]. Reconsider the next assignment using the latest evidence.” |
| Resume with a fresh agent | “Read research/prompts/research-program.txt. Resume [agenda path] using [session path]/abstract.md and its cited evidence. My latest steering is [constraints].” |
| Review a particular result | “Use research/prompts/reviewer.txt to review [report], [revision/diff] and [evidence/checks]. Inspect saved evidence first.” |
| Move to another checkout or machine | “Use research/handoffs.md to export [session] with the inputs needed for [claim], then verify it after relocation.” |

Replace bracketed values. The same conversation can do the research and a
labeled self-review. A separate reviewer is optional; its independence should
not be confused with an independently reproduced experiment.

## Where the program lives

The [autocal agenda](autocal-agenda.md) records current autocal priorities and
the evidence behind them. For a different topic the agent uses a relevant note
or creates one concise agenda. Each investigation keeps research.md (decisions
and budgets), report.md (evidence and limitations) and abstract.md (searchable
summary, decision and next observation) in its session directory. The
[catalog](../output/research/index.md) makes those summaries discoverable.

The [entry prompt](prompts/research-program.txt) connects the existing
[assignment](prompts/next-assignment.txt), [review](prompts/reviewer.txt),
[abstract](abstract-template.md) and [portable handoff](handoffs.md) conventions.
Tasks 001 through 006 completed these foundations. Tasks 007 through 015 implement
the manager described above; task 016 retains the live release gate. Use
`research/program-tasks/PROMPT.md` for development backlog work and the appropriate
question-first prompt for scientific research.

See the [worked question-to-decision examples](../output/research/sessions/2026-10-23-research-program-smoke/desk-cases.md)
and [smoke-test record](../output/research/sessions/2026-10-23-research-program-smoke/report.md)
for what was actually exercised. That historical smoke record used the original
zero-experiment defaults; the policy in this guide supersedes that restriction.

## When the question needs live simulation

Use the existing launcher on a machine with the repository's research runtime
installed, then give the same prompt in its conversation:

```bash
./hp-sim5-research-agent --codex --name "my research question" --viewer none
```

See [setup and execution instructions](README.md#setup-and-interactive-use) for
dependencies, machine settings and the existing supervisor tools. The program
layer organizes the research; live experiments need those tools and a recorded
targeted budget, which the agent can choose. Ordinary coding agents can inspect
code, analyze saved artifacts and maintain the program without starting a world.
A resumed launcher conversation starts a fresh world; a saved report or final
snapshot does not restore it.
