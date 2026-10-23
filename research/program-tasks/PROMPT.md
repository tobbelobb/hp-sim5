Work from the hp-sim5 repository root. Follow the root and research AGENTS.md
instructions, then read research/program-tasks/README.md, tasks.json and the
recent entries of progress.txt. Do not read the whole research corpus up front.

Do exactly one useful task this iteration:

1. Inspect the working tree and relevant current code. Resume an unfinished
   `in_progress` item when appropriate; otherwise choose the lowest-priority-number
   `ready` task whose dependencies are `done`. Open its task file and only the
   references needed for that change. Check whether the desired behavior already
   exists before implementing it.
2. If no task is ready, you may promote one `proposed` task whose dependencies and
   activation condition are satisfied by actual evidence. Otherwise stop with
   `NO_READY_TASK` and the concrete reason. Do not build a speculative feature to
   manufacture eligibility. `blocked` tasks need their blocker resolved first.
3. Set the selected task to `in_progress`. Make the smallest useful change that
   meets its acceptance criteria. Revise, split or park obsolete work with a
   reason. A revision to the task must preserve the user's intent; shrinking the
   claim honestly is preferable to weakening a required check.
4. Obey the zero-new-simulation/fit/collection budget. Use existing artifacts and
   focused checks. Do not create an evaluator service, mandatory second numerical
   run, database, scheduler, cloud deployment or additional agent hierarchy.
   If the task genuinely needs a different budget, record that need and stop it
   as blocked; do not quietly spend it. Required repository checks still apply
   to any actual change in their scope.
5. Verify the changed behavior once at the appropriate level. Do not repeat
   passing checks without a relevant new change or unresolved concern. Preserve
   useful negative results and missing-evidence limitations. A reviewer can
   inspect saved evidence without claiming to have reproduced it independently.
6. Mark the item `done` only when its acceptance criteria are met. Mark genuine
   unfinished blockers `blocked`, or make a smaller follow-up task if the useful
   completed part stands on its own. Append a short dated entry to progress.txt:
   task ID; changed files; checks run and reused; actual result/limitation; next
   decision. Do not rewrite earlier entries. Adjust future priorities or park
   unneeded proposals when this result gives a reason.
7. For a completed implementation, make one local commit containing only the
   task-owned changes and its queue/progress updates after the relevant checks.
   Keep unrelated user changes intact. Do not push. If a commit cannot be made,
   retain the changes and report why rather than discarding work.

End with a short account of the change, its verification and the next useful
decision. Use `TASK_DONE` for a completed item, `BLOCKED` for unfinished work, or
`NO_READY_TASK` when nothing can be selected. An optional outer driver must be
finite and stop on BLOCKED/NO_READY_TASK; these signals are task bookkeeping,
not scientific acceptance. Do not start another task in this invocation.
