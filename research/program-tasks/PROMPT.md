Work from the hp-sim5 repository root. Follow the root and research AGENTS.md
instructions, then read research/program-tasks/README.md, tasks.json and the
recent entries of progress.txt. Do not read the whole research corpus up front.

Do one coherent outcome this iteration. A task can include investigation,
implementation, focused verification and the resulting decision/handoff. Do not
stop after writing a proposal when the selected task authorizes carrying it out.
The saved briefing/review prompts are tools to use where helpful, not mandatory
extra agent invocations. Trust your judgment about implementation details.

Choose and complete that outcome:

1. Inspect the working tree and relevant current code. Resume an unfinished
   `in_progress` item when appropriate; otherwise choose the lowest-priority-number
   `ready` task whose dependencies are `done`. Open its task file and only the
   references needed for that change. Check whether the desired behavior already
   exists before implementing it.
2. If no task is ready, you may promote one `proposed` task whose dependencies and
   activation condition are satisfied by actual evidence. Otherwise stop with
   `NO_READY_TASK` and the concrete reason. Do not build a speculative feature to
   manufacture eligibility. `blocked` tasks need their blocker resolved first.
3. Set the selected task to `in_progress`. Deliver the useful outcome, including
   the natural next steps needed to make it reviewable. Revise, combine or park
   work that has become trivial or obsolete, with a reason. The first three tasks
   took 1m31s, 3m28s and 8m10s: use this as evidence about agent capability, not a
   runtime target. Do not pad a quick success, but do not split one useful result
   into repeated prompt-edit, desk-check and proposal-only commits either.
4. Use the queue's default budget overlaid by the selected task's `budget`.
   These are work limits, not an enforced resource service or a spending quota.
   Write a lower, specific experiment budget before numerical work; often zero
   is sufficient. Fresh fits and bounded headless simulation are allowed when
   they resolve a named gap. Reuse existing evidence first. Required repository
   delivery checks are additional to exploratory run counts; plan time for them
   when applicable and never waive them. If the task needs more, retain the useful
   result and state the remaining scope/budget instead of quietly expanding it.
   Add no evaluator service, obligatory duplicate run, database, scheduler, cloud
   deployment or extra agent hierarchy merely to satisfy an architecture diagram.
5. Verify the changed behavior once at the appropriate level. Do not repeat
   passing checks without a relevant new change or unresolved concern. Preserve
   useful negative results and missing-evidence limitations. A reviewer can
   inspect saved evidence without claiming to have reproduced it independently.
6. Mark the item `done` only when its acceptance criteria are met. Mark genuine
   unfinished blockers `blocked`, or make a smaller follow-up task if the useful
   completed part stands on its own. Append a short dated entry to progress.txt:
   task ID; changed files; checks run and reused; actual result/limitation; next
   decision. Do not rewrite earlier entries. Adjust future priorities or park
   unneeded proposals when this result gives a reason. Keep this entry concise;
   experiment details belong in the linked research record, not repeated here.
7. For a completed implementation, make one local commit containing only the
   task-owned changes and its queue/progress updates after the relevant checks.
   Keep unrelated user changes intact. Do not push. If a commit cannot be made,
   retain the changes and report why rather than discarding work.

End with a short account of the change, its verification and the next useful
decision. Use `TASK_DONE` for a completed item, `BLOCKED` for unfinished work, or
`NO_READY_TASK` when nothing can be selected. An optional outer driver must be
finite and stop on BLOCKED/NO_READY_TASK; these signals are task bookkeeping,
not scientific acceptance. Do not start another task in this invocation.
