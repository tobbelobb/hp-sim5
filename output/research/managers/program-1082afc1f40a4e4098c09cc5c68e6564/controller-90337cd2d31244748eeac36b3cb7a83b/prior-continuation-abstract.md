# Hangprinter program resumed with a working native build

Status: existing program active; new scout ready without runtime allocation. Child-client workflow unresolved.

## Abstract

Can the existing Hangprinter uncertainty program continue after the cloud toolchain is selected correctly? The manager reopened program `program-1082afc1f40a4e4098c09cc5c68e6564` using its existing store and history, inspected the overview and reconciled ownership. No active or unknown children were found. With `scripts/cloud/env.sh` sourced, the requested native build without PGO succeeded under Node 23.11.1 and Rust 1.99.1; native addon freshness and loading also passed. No compiler-probe patch was needed. The idle controller was retired and reopened through supported lifecycle commands so children inherit the toolchain, then the program resumed. A bounded existing-login client probe still failed at read-only app-server initialization before model execution. The requested `scripts/cloud/RESEARCH.md` is absent. A distinct twelve-minute scout is ready, with no runtime allocation or new worker. Earlier failed attempts and the pending review obligation remain intact. No new scientific result is available.

## Topics

Hangprinter, research manager, program recovery, cloud toolchain, native Rust, existing ChatGPT login, app-server startup, common unused sweeps

## Tested hypotheses

| Hypothesis | Experiment ID | Outcome | Evidence |
| --- | --- | --- | --- |
| Sourced cloud environment can build the native addon without PGO | CLOUD-BUILD-01 | Supported for build and load only | [Build receipt](native-build-verification.json) |
| A workspace-write ephemeral child client can initialize with the current existing login | CLOUD-CLIENT-01 | Blocked at initialization; no model execution | [Probe](existing-login-probe/result.json), [stderr](existing-login-probe/stderr.log), [filesystem trace](existing-login-probe/file-access.strace) |
| Common unused calibration sweeps exist | assignment-holdout-scout-cloud | Not tested; no new worker dispatched | [Commission](next-scout-commission.json) |

## Decision handoff

- Scope: cloud build, process ownership and client startup at revision `8acde0c21d5b9db8893115e3e79d82d9e41f98ca`. No calibration, motion or hardware result.
- Review: manager-observed engineering evidence. The original standalone review failure remains separate and unreviewed.
- Decision: keep the program active and preserve every failed attempt. Retain the original scientific route order and the ready scout.
- Next: establish the prescribed existing-login workflow from the missing guide. Resolve or explicitly account for the retained review obligation through supported tools, then allocate the new twelve-minute scout and separate review. No runtime is currently reserved.

## References

- [Report](report.md), [notebook](research.md), [overview](overview.md), [operation transcript](manager-operations.jsonl).
- [Previous manager report](../controller-6664a6157d2e451a8ec92f9b1efd67af/report.md) preserves the uncertainty map and failed launches.
