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
