A lot of the pain was not physics — it was losing multi-hour runs to lifecycle gaps. My wish list, roughly in priority order:

1. Durable, reattachable jobs

- list_jobs() with state, owner, artifacts, start time, last error.
- job_status(id) that returns a tombstone after completion/failure instead of erroring like autocal_status(32335) did after the launcher restarted.
- attach_job(id) / adopt_job(id) so a resumed Codex conversation can reattach to a live supervisor job rather than start a fresh world.
- On supervisor shutdown, automatically finalize orphans as stop_reason=supervisor_restart, drain the recorder and mark artifacts partial. The clean run stayed status=running forever.

2. Supervisor-managed autocal with CLI parity
start_autocal was safer than a shell background job, but it could not accept a seed dataset, --physics-substeps, --max-steps, or reused force tuning. That forced me into exec_command background runs, which die with the session. I want:

start_autocal({
  dataset, seed_dataset, max_steps, physics_substeps,
  recording_mode, force_tuning, speedup, budgets
})

with the returned job surviving or being reattachable across launcher restarts.

3. Resumable checkpoints
The launcher explicitly says snapshots are observations, not checkpoints. A multi-hour full-auto run cannot resume from a partial sweep. I want:

- save_checkpoint(run_id, step) / resume_checkpoint(...) with a versioned native state blob (bodies, joints, multipliers, damping cache, RNG).
- At minimum, deterministic replay from the initial fixture plus the command/force timeline, so an interrupted acceptance can be reconstructed.

4. Stage-scoped acceptance and fast orchestration dry-runs
--max-steps 3 still means hours of physics. I want named profiles:

run_acceptance(profile="settling" | "force-probe" | "single-sweep" | "full")

and a mocked/fast physics clock for testing force tuning, deadlines, dataset merge and stop logic without simulating every cable. Most of the failed runs failed in orchestration, not physics.

5. A registered stability suite and cross-backend matrix
The wobble reproduction is a bespoke record_hp4_wobble.mjs plus a frozen JSON fixture. It should be a first-class suite:

stability_matrix({fixture, backends: ["rust","js","python","warp"],
                  iterations: [1,2,3,4], substeps: [1,2]})

returning one table and one RRD per cell, with pass/fail thresholds. start_test currently only exposes headless-autocal and research-services; the stability benchmark and full-auto smoke tests were outside it.

6. Solver introspection without a custom trace build
The per-iteration trace required a special non-production artifact. I want runtime-selectable tracing:

trace_solver({joints, fields: ["d","lambda","C","D","alpha_tilde","gamma"],
              passes: "all", steps: [7500, 7600]})

and an energy ledger that flags “damping is adding energy while no external work is applied.” That would have found this bug much faster than amplitude matrices.

7. RRD as a queryable dataset, not only a viewer
Rerun MCP drives the UI; it does not read numeric series. Every metric extraction became a Python script. I want:

- rrd_query(path, entity, component) returning timestamps and values.
- rrd_compare(a, b) with derived series: late-window span, deviation from rolling mean, angular speed, autocorrelation, spectral peaks.
- rrd_annotate and a saved blueprint template for effector/cable diagnosis.
- Automatic hashes and a manifest beside every recording.

8. Automatic provenance
The clean run’s provenance.json was hand-written. Every artifact should record:

- full command line and environment,
- git describe --dirty, dirty_diff_sha256, and a saved source patch/archive,
- world_id, session_id, job_id, parent_job_id,
- exact physics artifact hash and per-source-file hashes.

Without this, an uncommitted fix is not reproducible after the worktree changes.

9. Structured failure taxonomy and post-mortem
A multi-hour run surfaced as collector exit status 1 plus a wall of command text. I want a top-level failure: {kind: "deadline_exceeded" | "force_stall" | "physics_nan" | "recorder_error", trial, force, step, message} with the last relevant log lines, not just an exit code.

10. Dataset and acceptance-fixture tooling
The target dataset was absent. I want:

- dataset_lint(path): schema, sweep count, point coverage, force metadata, hashes.
- A checked-in or deterministically generated acceptance dataset registry, so agents do not depend on user-local paths.
- seed_dataset as a first-class autocal argument.

11. Parallel batch runner with quotas
Twenty cores were available, but I ran sequentially to avoid contamination. A supervisor-managed batch runner with isolated ports/worlds and per-job CPU/memory/disk budgets would let me run default-1-substep and explicit-2-substep acceptance in parallel. Disk was at 97%; recording_mode=events RRDs are manageable, but sampled/full need quota enforcement and cleanup that preserves manifests.

12. Prior-art search
I found upstream bf9f5399 late. A prior_art_search(query) across local sessions, metrics, git history and remotes would prevent duplicate work and make “has anyone tried this?” a one-call check.

If I could add only three: durable/reattachable jobs, supervisor-managed autocal with full CLI parity, and resumable checkpoints. Those three would have turned the clean 210,000-step run into evidence instead of a lost trial.
