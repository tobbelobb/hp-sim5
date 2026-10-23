# Task 009 implementation record

Objective: dispatch one bounded assignment through the existing research worker,
observe its actual paths/process/artifacts, and cancel through its cleanup owner.

Hypothesis: a small stdlib adapter on the existing ProgramStore and launcher can
provide this without another physics world or a new authentication path.

Development budget: at most the task's 90-minute ceiling; zero fresh fits,
numerical predictions, physics trials/steps, collections or movements. Required
verification is fixture/process, launcher regression and argument dry-run work.
Runtime assignment allocations are separate recorded worker contracts.

Source base: cd2755f37f3a7df0462ef313d6bd66f0fac30a4f, confirmed by parent as
canonical HEAD/origin/main after tasks 007/008. No shell fetch/push was attempted.
Task code is recorded in the task-owned commit and [source hashes](source-hashes.json).

Accepted steering: one implementation agent in the shared checkout; one local
commit; no task 010 or push; use actual ProgramStore/graph APIs; retain fake-process
versus real-launcher/client/live-science boundaries. Parent supplied released
pytest sources and prerequisite observations; no network installation was retried.

W009-FIXTURE: real fake-launcher children exercised dispatch, failures, partial
artifacts, ownership, cancellation and export/import; no science accepted.
W009-LAUNCHER: original launcher regressions with fake clients/services, plus
SIGTERM during runtime startup, Viewer readiness interruption and cleanup errors.
W009-DRY: four retained actual launcher argument dry-runs; no client/world started.

One initial assertion compared a program revision as if graph knowledge should
freeze it. Corrected the assertion and retained the failed-check explanation.
Startup ownership inspection justified two small launcher cleanup fixes.

Decision: complete task 009; preserve worker output as unassessed submissions.
Next decision: task 010's fixed submissions and separate review. Task 016 must
resolve live prerequisites and demonstrate production worker/review integration.
