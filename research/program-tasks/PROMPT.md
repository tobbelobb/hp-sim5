Work from the hp-sim5 repository root. Follow the root and research AGENTS.md
instructions, then read research/program-tasks/README.md, tasks.json and the
recent entries of progress.txt. For tasks 007–016 also read MANAGER.md, which
defines the accepted manager release. Do not read the whole research corpus up front.

Do one coherent outcome this iteration: investigation, implementation, focused
verification and decision/handoff. Do not stop after a proposal when the selected
task authorizes implementation. Briefing/review prompts are tools to use where
helpful, not mandatory extra agent invocations. Real worker dispatch must use
hp-sim5-research-agent. Separate review reuses the supported coding-client/auth
path; it need not start physics merely to inspect saved evidence. Trust your
judgment about implementation details.

The user wants a manager between their chat and that existing worker, including
durable graph memory and approach selection. A static agenda, longer worker
prompt or passing mock alone does not satisfy those implementation cards.
Task 016 is the live release gate. Individual task completion is not a claim
that the entire manager is ready.

Choose and complete that outcome:

1. Inspect the working tree, HEAD/upstream and relevant code. Consult the latest
   shared queue when reachable and incorporate available completed work before
   choosing; preserve local/user changes. Do not redo work delivered upstream or
   in this conversation. If networking is unavailable, record the freshness
   limitation and use available evidence rather than blocking on an unnecessary
   permission request. Resume an appropriate unfinished `in_progress` item;
   otherwise choose the lowest-priority-number `ready` task whose dependencies
   are `done`. Open its task file and only needed references. Check whether the
   desired behavior already exists before implementing it.
2. If no task is ready, one `proposed` task may be promoted when its dependencies
   and activation condition have actual evidence. Otherwise stop with
   `NO_READY_TASK` and the concrete reason. Do not manufacture eligibility.
   `blocked` tasks need their blocker resolved first.
3. Set the selected task `in_progress`. Deliver its useful outcome through
   implementation and checks. Revise/combine obsolete work with a reason, while
   preserving required manager capability or providing an equivalent tested
   outcome. Do not park a core capability merely because a Markdown approximation
   exists. Avoid tiny proposal-only or prompt-edit-only implementation commits.
   There is no minimum runtime; do not pad a quick success.
4. Overlay the selected task's `budget` on queue defaults. These are development
   work limits, separate from runtime program allocations and enforced services.
   Write a smaller concrete budget before numerical work; zero is appropriate
   when fixtures suffice. Fresh fits, bounded native/headless physics and
   simulated collection are allowed to resolve named gaps. Use targeted probes,
   supported --speedup settings (normally --speedup 25 for simulated autocal CLI)
   and suitable recording. Headless physics already runs as fast as possible.
   Reuse useful evidence first. Required delivery checks are additional to
   exploratory run counts: plan for them and never waive them. If scope exceeds
   the cap, preserve useful work and state the remaining scope/budget.
   The program store/graph, controller, chat tools, worker pool and separate
   review path are authorized where their cards require them. Keep them compact;
   do not add unrequested framework/cloud infrastructure or extra hierarchies.
   Do not require duplicate science runs merely for reviewer independence.
5. Verify changed behavior once at the appropriate level. Repeat passing checks
   only for a relevant change or unresolved concern. Preserve useful failures
   and missing-evidence limitations. Distinguish inspecting producer evidence,
   fake-process tests, actual client interaction and live numerical reproduction.
6. Mark `done` only after acceptance criteria pass. Mark genuine unfinished
   blockers `blocked`, or record a bounded follow-up for an independently useful
   completed part. Append a concise dated progress.txt entry: task ID, files,
   checks run/reused, result/limitations and next decision. Never rewrite earlier
   entries. Update priorities/briefs when evidence changes them. A required
   manager capability cannot be parked simply to declare the release finished.
7. Make one local commit containing only task-owned changes and queue/progress
   updates after relevant checks. Preserve unrelated user work. Do not push
   unless the human authorized it in the current session. If a commit cannot
   be made, retain the work and explain why.

End with a short account of the change, verification and next useful decision.
Use `TASK_DONE`, `BLOCKED` or `NO_READY_TASK` as appropriate. An optional outer
driver must be finite and stop on BLOCKED/NO_READY_TASK; these are development
bookkeeping signals, not scientific acceptance. Do not start a second task.
After task 016 passes, additionally report `MANAGER_RELEASE_COMPLETE` with its
live evidence record; before then distinguish TASK_DONE from manager readiness.
