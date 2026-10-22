# Supervisor-owned full-autocal jobs

Objective: expose the production full-auto CLI through asynchronous launcher MCP jobs, even when client command sockets are denied.

Hypothesis: one isolated HP3 CLI job with known 30 mm radii, zero buildup and layering disabled can physically collect, fit, plan, select, automatically accept and finalize a readable RRD without changing an attached HP4 world.

Constraints: `.venv/bin/python`; production JS collector and physics; no offline replay, synthetic collector or acceptance via budget stop. One real acceptance job, 20 fitting iterations, 20 points per sub-sweep, 3600 s wall time, 5 million physics steps, one BLAS/OpenMP thread. No algorithm changes or outer agent loop.

Accepted steering: user reports the other Codex agent is stopped; preserve unrelated work.

Experiment IDs: full trial a792904dd547473ca0f6583f5ba547b9 failed during bootstrap pullout (120 s encoder stability timeout). It advanced 666,417 physical steps / 1,332.834 simulated seconds and finalized its partial RRD. The socket-denied MCP client returned its job ID in 0.003 s; HP4 remained at step zero. No fitting or automatic acceptance was reached. The first cancellation test waited for a measurement for 120 wall seconds and failed its test deadline during force tuning; its shutdown finalized a partial recording, but pytest removed its temporary directory before preservation (a recordkeeping limitation).

Focused lifecycle checks passed for cancellation (a06bbedcc9ce4c51a865afe99709f246), HTTP shutdown (64789c7483ea402a9b72a8e954c60907), forced owner SIGKILL (317a350a34e04df8a2acd5ea9b016611) and physics-step exhaustion (301dab92216b42c9ae5dd246bd664b6c). Copies of logs/manifests/RRDs are in lifecycle/. Partial physical RRDs were readable through RrdReader.store(). Unit checks also cover invalid settings/artifact paths, pre-spawn cancellation, orphaned children, concurrent cancel/shutdown and acceptance versus budget classification.

Second acceptance trial: preserve production fitting/planning/selection/stopping, but use the existing CLI manual-force path with 0.01/0.23/9.2 N, still known 30 mm radii, zero buildup and no layering. The first run established a real failure under production autotuning, so explicit bounded collector forces are now available through the structured API. This is an input change, not an acceptance threshold change. No algorithm changes are made.

Completed second trial 2ec0b06774624962a18bdbd2818eb134: genuine default patience acceptance at iteration 6 (three non-improving attempts), exit 0, eight sweeps / 320 points, 2,099,074 physics steps / 4198.148 simulated seconds, 780.154 wall seconds. MCP start latency was 0.002766 s. Attached HP4 session identity and step zero were unchanged. All registered child PIDs were absent after finalization. RrdReader.store() read the finalized footer; independent inspection found exactly 320 recorded measurements, six complete collections, the final physical stop event, seven embedded stage artifacts and exact final dataset/log content. Original/baked scene and firmware copies match their recorded hashes.

The selected parameters were concerning (CLI score 6018); automatic acceptance validates the workflow contract, not physical accuracy. The first default-autotuning failure is retained as a known integration limitation. No direct research MCP connector exists in this Codex chat, so the restricted SDK stdio client was used; direct launcher-attached Codex/desktop verification remains an integration check for a new session.

During the accepted run, the progress counter was corrected to sum subsequent per-sweep journals. The running supervisor retained the earlier implementation and reported 120 bootstrap points; final dataset, all journals and RRD independently contain 320. The final implementation's aggregate counter has a focused regression test. Source hashes in each job identify its loaded-era sources; restart the launcher to load later source edits.

Next decision: implementation complete. Investigate the default no-buildup pullout timeout and concerning physical calibration in separate research; neither is fixed or concealed by this tooling change. Fifth/seventh HP3 replay regressions remain exactly equal to references. Catalog refreshed.
