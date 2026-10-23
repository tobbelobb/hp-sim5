# Start a research program from a question

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
The development backlog under program-tasks/ is complete; start new research
from your question or agenda rather than asking the agent to find another
development task.

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
