# Research program smoke checks and a question-only starting prompt

Status: complete for offline workflow checks; live research runtime not exercised.

## Abstract

Can a user start this research workflow with only a question? Four standard-library
smoke tests made fourteen real CLI calls covering catalog generation/search,
pending/legacy sessions, relocated handoffs, damaged/missing evidence, invalid
export requests and saved job states. All passed. Two manual desk applications
then checked an already answered consensus question and an unresolved HP4 mass
question. The existing briefing prompt intentionally stopped at a proposal; a
new entry prompt connects it to one bounded execution, labeled self-review and
saved decision. A quickstart gives copyable question, continuation, resume and
review prompts. These checks establish offline behavior and demonstrate prompt
usage in this conversation. They do not evaluate independent agents, live
supervisors, physics stability, new calibration accuracy or hardware behavior.
No fresh fits, predictions, physics runs or collection were performed.

## Topics

research program, question-only prompt, smoke tests, catalog, evidence handoff,
historical job status, self-review, budgets, research quickstart

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The catalog finds decision handoffs and distinguishes pending or missing summaries | RP-S1 | Supported for CLI fixtures and real CM-E1 lookup | [CLI results](smoke-results.json), [commands](cli-invocations.json) |
| Selected evidence survives source removal and diagnoses damaged/missing files | RP-S2 | Supported for synthetic relocation and partial-export fixtures | [CLI results](smoke-results.json), [harness](smoke.py) |
| Saved job outcomes imply neither liveness nor scientific success | RP-S3 | Supported for six states and a corrupt record | [CLI commands/results](cli-invocations.json) |
| One question reaches a scoped answer or bounded missing-evidence decision | RP-D1 | Supported in two manual cases in this conversation | [Cases and self-reviews](desk-cases.md), [inspection](desk-inspection.json) |

## Decision handoff

- Scope: offline archive/catalog CLIs and two manual prompt applications, base 0606f031.
- Review: current agent ran four CLI smoke tests and inspected saved evidence;
  prompt review was a self-review, with no independent numerical reproduction.
- Decision: use the question-first entry prompt and guide; retain existing runtime ownership and experiment-budget rules.
- Next: give an agent a real research question and optional constraints/budget.
  Live experiments need the existing launcher/supervisor and applicable checks.

## References

- [Report](report.md), [notebook](research.md): scope, commands, limits and usability change.
- [Quickstart](../../../../research/program.md), [entry prompt](../../../../research/prompts/research-program.txt): user entry points.
- [Smoke harness](smoke.py), [results](smoke-results.json), [calls](cli-invocations.json), [documentation checks](documentation-checks.json): verification evidence.
- [Desk cases](desk-cases.md), [CM-E1 report](../2026-10-23-consensus-membership/report.md): saved scientific evidence reused without numerical reruns.
