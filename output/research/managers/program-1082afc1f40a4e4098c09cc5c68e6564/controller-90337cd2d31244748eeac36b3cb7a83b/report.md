# Hangprinter native prerequisites repaired; worker reaches client initialization

Status: native and supervised-runtime setup verified. Existing program active without allocated work. Scientific investigation remains blocked at coding-client initialization.

## Abstract

Can the existing Hangprinter research program get past its native-build prerequisite without replacing its store or history? All three captured rustc probes now use ignored stdin, piped stdout and inherited stderr. The installed Node 23.11.1, Rust 1.99.1 and Node-API headers were verified; the no-PGO native build and addon load passed. Actual worker retries exposed an empty local Python environment and a missing collector JS dependency. Declared Python requirements, full locked JS dependencies and the pinned RRF simulator were restored. A nonmoving launcher preflight then passed all six checks at world step zero. The final managed worker successfully started its native and supervised runtime, then failed at read-only Codex app-server initialization before model execution. Three new failed startup attempts are retained alongside the original failure and sealed review history. No saved-data scout, calibration measurement or scientific admission completed. The next necessary work concerns writable client runtime state, separately from the repaired native build.

## Topics

Hangprinter, native build, Node-API, Rust probes, runtime prerequisites, RRF, launcher preflight, research manager, existing login, client initialization

## Tested hypotheses

| Check or question | Experiment ID | Outcome | Evidence |
| --- | --- | --- | --- |
| Installed native prerequisites build and load the addon | NATIVE-PREREQ-01 | Passed engineering verification | [Build/load receipt](native-prerequisite-retry/verification.json) |
| Launcher runtime services and tools are ready | RUNTIME-PREFLIGHT-01 | Six checks passed at step zero | [Preflight result](native-prerequisite-retry/launcher-preflight-result.json) |
| Existing-login research worker reaches model execution | Fourth managed startup in original chain | Blocked at read-only app-server initialization | [Final worker status](native-prerequisite-retry/bootstrap-verified-worker-status.json) |
| Common unused whole sweeps exist for changed CM-E1 models | Three new scout assignments | Not tested; startup prevented inspection | [Startup chain](native-prerequisite-retry/startup-chain-summary.json) |

## Decision handoff

- Scope: native/runtime engineering verification at source base `8acde0c21d5b9db8893115e3e79d82d9e41f98ca`, with the three-line build-script change uncommitted. No physics equations or calibration defaults changed.
- Review: checks and startup logs inspected by manager. The original standalone reviewer failure and sealed submission remain unchanged and unreviewed.
- Decision: retain the native probe fix and working prerequisites. Stop rebuilding or repeating the unchanged client failure. Preserve scientific alternatives and all prior attempts.
- Next: establish supported writable runtime state for the existing-login coding client. Then separately review the retained startup submission before a new bounded scientific assignment under the restored default backlog limit.

## References

- [Full report](report.md), [notebook](research.md), [overview](overview.md), [operation transcript](manager-operations.jsonl).
- [Previous continuation](prior-continuation-report.md) records the earlier state before this prerequisite repair.
- [Original research assessment](../controller-6664a6157d2e451a8ec92f9b1efd67af/report.md) preserves the uncertainty map and experiment rationale.

## Source change and verification

`scripts/build_native_physics.mjs` has exactly three changed lines. Every captured `rustc` probe uses `{ encoding: 'utf8', stdio: ['ignore', 'pipe', 'inherit'] }`, covering the compiler identity and both target-library lookups. The edit leaves build flags and native physics source unchanged. `node --check`, `npm run build:native -- --no-pgo` and `loadNativePhysicsAddon()` passed with the requested explicit environment exports. [Raw verification log](native-prerequisite-retry/verification.log) and [machine-readable receipt](native-prerequisite-retry/verification.json) preserve commands and output. PGO training was not run.

The environment exports select `/workspace/.tools/cargo`, `/workspace/.tools/rustup` and the installed Node 23.11.1 headers/bin. Their presence was checked rather than assumed. The initial no-PGO build already worked, but this turn applies the newly unconditional stdio requirement to all three probes and rebuilds the addon with matching metadata.

## Additional actual prerequisites

The first new worker passed native loading but failed because `.venv` was the empty environment created during the original manager setup. Installed the repository's `requirements.txt`, verified its runtime imports and ran `pip check`. Installed CMake 4.4.4, fetched the three exact pinned RRF submodules and built `rrf_simulator` with four jobs. No full autocal fit or firmware motion experiment ran.

The first npm attempt failed because its default cache was read-only. Retained that failure and used `/workspace/.cache/npm`. Production-only installation passed, but the next actual runtime startup revealed that the collector imports `ws`, which is available through the full locked development dependency graph. Ran full `npm ci` with browser downloads disabled, preserving `package-lock.json` and avoiding a dependency-source change.

Evidence includes [Python installation](native-prerequisite-retry/python-dependencies.log), [runtime versions](native-prerequisite-retry/runtime-dependencies.json), [initial npm failure](native-prerequisite-retry/js-dependencies.log), [production npm install](native-prerequisite-retry/js-dependencies-writable-cache.log), [full npm install](native-prerequisite-retry/js-full-dependencies.log), [pinned submodule checkout](native-prerequisite-retry/rrf-submodules.log), and [RRF build receipt](native-prerequisite-retry/rrf-build.json) with [build log](native-prerequisite-retry/rrf-build.log).

## Nonmoving launcher verification

Ran the existing launcher's `--preflight` path with HP3, native headless physics, no Viewer and no continuing recording. This was a bounded engineering check, separate from the research-worker allocation. Its absolute launcher deadline was 90 seconds, with the normal cleanup path retained. [The result](native-prerequisite-retry/launcher-preflight-result.json) records six passes for capabilities, collector, firmware, recorder, services and settings. The world reported step zero with no active job. The recorder check creates isolated probe evidence; it does not imply a continuing scientific recording or calibration result.

## Managed retries and remaining boundary

Each startup retry used a distinct assignment and finite runtime reservation. The chain limit increased explicitly only after a material prerequisite change, ending at four total launches including the original attempt. The original worker, every failed retry and the old separate review remain in the same store. Temporary backlog capacity permitted one new obligation beside the retained historical review; it has been restored to one worker and one review obligation after cleanup.

| Attempt | Observed worker wall time | Actual stopping point |
| --- | ---: | --- |
| `attempt-fad71bf317c2489d9d3a58523e701efc` | 0.246 s | Missing NumPy after native loading |
| `attempt-02ca2a76af6a4866b86c8fe1feb55881` | 1.585 s | RRF booted; collector missing `ws` |
| `attempt-4c96256b324647d3baf8eaf0ac653b6e` | 2.062 s | Native/runtime startup completed; Codex app-server read-only initialization |

The final attempt wrote its real research launch metadata and reached the existing-login coding-client invocation. Its stderr reports `failed to initialize in-process app-server client: Read-only file system`. This confirms that the worker shares the previously observed client-state boundary; the problem is not limited to the standalone reviewer. The prior filesystem-only trace identified attempted writes to runtime SQLite state and `installation_id` under `/run/codex-environment/codex-home`. No credentials, client homes, filesystem mounts or authentication were changed to work around it.

All three new attempts returned through launcher cleanup with no reported cleanup errors. This records completion of the owned cleanup path, not restoration of a running physical world. No startup failure rejects a scientific hypothesis. The final state has zero active workers/reviewers, no unknown ownership, and the original single pending review obligation. No further worker or reviewer launch is currently allocated.

## Next allocation

The native prerequisite task is complete. A later tooling assignment should first establish a supported coding-client initialization path with writable runtime state while reusing the existing login. Keep that work separate from the original sealed startup review. Once the client works, fund a separate review of the retained submission and apply only a valid assessment. Then commission a fresh twelve-minute saved-data scout with the existing three-case, twelve-extra-file scope. Do not rebuild native physics or reinstall dependencies merely to repeat the same client-state failure.

The fair-validation route remains promising on inspected historical reports. Model sensitivity and dynamic fidelity remain credible alternatives. No new evidence about their scientific outcomes was produced this turn.
