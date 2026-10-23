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

## What actually ran

The [standard-library harness](smoke.py) ran once, with four passing tests and
fourteen CLI calls in approximately 0.44 seconds on this environment. This is a
smoke-check duration, not a research or simulation speed measurement. Each child
CLI has a ten-second timeout and uses temporary fixture directories.

| Check | Behavior actually exercised | Result |
| --- | --- | --- |
| RP-S1 | Build/search a temporary catalog, deduplicate an alias, preserve decision and relative evidence links, label pending and legacy sessions | Passed |
| RP-S2 | Export six selected files, relocate, delete every original source, recover and verify; corrupt an input afterward | Passed; corruption exits 1 with partial evidence, retaining the report |
| RP-S3 | Read accepted/failed/cancelled/budget_exhausted/running/cancelling records and malformed JSON | Passed; terminal and historical-nonterminal remain distinct; liveness unknown and scientific success not assessed |
| RP-S2 negative paths | Wrong expected hash, missing source, malformed selected job and path traversal | Passed; partial evidence remains recoverable and invalid requests exit 2 before creating a destination |

Full commands, exit codes and outputs are retained in [cli-invocations.json](cli-invocations.json).
Temporary source/current paths in that file deliberately no longer exist; these
are synthetic fixtures and their removed paths are part of the relocation test.
No attachment data or recording was automatically included. Direct status calls
also had process signalling/PID lookup patched to fail if attempted.

A separate real catalog search found CM-E1 and its decision handoff. Inspected
its saved summary and changed-case coverage, and checked the selection source
against the producer's analysis hash. [desk-inspection.json](desk-inspection.json)
records those observations. No selection or prediction evaluation was repeated.
Pytest is absent from this checkout; these are real CLI smoke tests run through
Python's unittest, not a claim that the full pytest suites were rerun.

## Prompt usability and review

[Two manual cases](desk-cases.md) exercise the user-facing flow. The first question
closes with a scoped evidence-based answer instead of manufacturing another
experiment. The second retains a different research topic, follows a superseding
HP4 report and returns an explicit untested experiment because the default
budget allows no fresh physics. Both include a self-review of the exact claim.
This is one agent applying the prompts; it is not an independent-agent evaluation
or a guarantee of future model behavior.

The initial inspection found that next-assignment.txt is deliberately
briefing-only. Added [research-program.txt](../../../../research/prompts/research-program.txt)
as a single entry that uses that preparation step, then carries one assignment
through execution, review and agenda/report/abstract updates within the human's
budget. The existing briefing-only route remains useful when requested.
[program.md](../../../../research/program.md) gives copyable prompts, defaults,
continuation/resume/review examples and the existing launcher route.

The entry defaults to a 30-minute ceiling for saved-evidence work and zero fresh
fits, physics or collection; an explicit user budget takes precedence. It does
not create an outer loop, mandatory agent hierarchy or new runtime service.
Current root delivery gates still apply to later implementation.

Self-review target: the four smoke-check outcomes, the two desk cases, the new
guide/entry prompt and their current diff against base 0606f031. Finding quality:
useful offline behavior is demonstrated; prompting evidence is limited to these
manual cases. Readiness: no runtime source changed; documentation links and
abstract/report consistency checked in [documentation-checks.json](documentation-checks.json).
Verdict: keep the guide/entry prompt and offline smoke evidence. Live simulation,
calibration accuracy and hardware remain outside this check, with their existing
requirements preserved. No duplicate science run or extra reviewer is needed
to substantiate this narrower claim.

## References

- [Notebook](research.md), [abstract](abstract.md): scope, budget, hypothesis outcomes and next decision.
- [Smoke results](smoke-results.json), [commands](cli-invocations.json), [harness](smoke.py): actual checks.
- [Manual cases](desk-cases.md), [inspection](desk-inspection.json): question-to-decision examples and evidence identities.
- [Question-first quickstart](../../../../research/program.md), [entry](../../../../research/prompts/research-program.txt), [assignment](../../../../research/prompts/next-assignment.txt), [review](../../../../research/prompts/reviewer.txt): user workflow.
- [Portable handoffs](../../../../research/handoffs.md), [archive source](../../../../src/python/hp_sim5_research/archive.py): exercised CLI implementation and scope.
