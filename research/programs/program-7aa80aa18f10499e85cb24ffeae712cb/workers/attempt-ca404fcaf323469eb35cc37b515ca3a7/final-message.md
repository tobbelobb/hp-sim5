The baseline cloud audit found two reproducible defects after runtime readiness passed.

- Fresh experiments fail in shared-output worktrees because generated output containment is checked against the worktree input root. Both direct requests failed before physics, and the MCP integration test reproduced it.
- Isolated research-services jobs discard WARP_CACHE_PATH, so two Warp CPU tests try a read-only home cache.
- research-services finished with 172 passes and three failures. headless-autocal passed all 49 selected tests. HP3 preflight passed in 0.818 seconds, and I read its 67,028-byte RRD as six data chunks. The attached world stayed at step zero.

All 23 tool schemas were inventoried. Six tools were invoked, with actual MCP execution performed by the parent in its permitted command context using my authored scripts. My default socket probe returned EPERM, and an earlier additional-network request stalled with unknown outcome before the manager interrupted it. The report preserves that distinction.

I made no source edits, shared builds, collections or fits. No successful direct physics experiment, calibration result, Viewer inspection or hardware evidence is claimed. report.md, abstract.md, coverage.json, test-summary.json and raw failed evidence are complete in this session. Next, repair output containment and job caches, repeat the exact failures and run a fresh production trial.
