# 006: Read saved outcomes of ended jobs without rerunning them

Activate when a current investigation needs an ended job's status, live tools
cannot return it, and saved job artifacts exist. Otherwise park this task.
Benefit: preserve the value of expensive completed or interrupted work.
Budget: zero simulations, fresh fits or collections. Use temporary files and
fake child processes for tests; no supervisor/firmware startup is needed.

Read the relevant part of [the tooling wish list](../prompts/researcher-tooling-wish-list.md),
[autocal_jobs.py](../../src/python/hp_sim5_research/autocal_jobs.py), and
[its tests](../../tests/python/test_autocal_jobs.py). Recheck the current gap:
jobs are tracked in memory, while `job.json` already stores terminal evidence.
Do not treat the entire historical wish list as an unimplemented specification.

Expose a small read-only path to the saved outcome in the existing research
index/status tooling, reusing the persisted schema. Choose the narrowest route
that solves the observed lookup. A fresh launcher uses a new session directory,
so an old artifact must be locatable explicitly; searching only the new session
would not solve the problem. Keep job/world/session identity and evidence paths.

Done when saved accepted/failed/cancelled/budget-exhausted outcomes can be read,
and a stale `running` record is labeled as an unresolved historical observation
rather than a live job or completed research. Never infer process ownership from
a reused PID, signal an old process, or mark calibration accurate from CLI status.
Missing/corrupt artifacts produce a short useful diagnostic.

No adoption, physics checkpoint/resume, new durable scheduler, broad CLI parity
work or changes to cancellation semantics. If the observed need is actually a
still-live service, use the existing attachment route and record that this task
was unnecessary. Run only the focused fixture tests for the code actually changed.
