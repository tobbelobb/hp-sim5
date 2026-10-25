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

## What changed

The old controller was alive with no active children. Recovery found no unknown cleanup or reviewer records. Its environment predated the cloud toolchain setup. `shutdown`, followed by reopening the same program from a shell that sourced `scripts/cloud/env.sh`, created controller `controller-90337cd2d31244748eeac36b3cb7a83b` without replacing the store, program identity or scientific history. A second recovery and the explicit resume operation completed before the new commission.

The native build finished its Cargo release phase in 3.50 seconds and produced `src/wasm/cable_solver/target/native/physics.node`. The saved build receipt contains the artifact hash and source metadata. A direct `loadNativePhysicsAddon()` call then passed freshness and loading on Node 23. Neither check advances a simulation or validates physical accuracy. Because compiler probes succeeded, the conditional rustc stdio edit was not applied.

## Remaining client blocker

`codex login status` confirmed the existing ChatGPT sign-in. No credentials were displayed, provisioned, copied or changed. The normal client helper constructed a workspace-write, ephemeral, no-tools probe with research MCP servers disabled. It exited 1 before producing any model output. A bounded filesystem-only trace observed read-only failures opening the runtime SQLite database and `installation_id` under `/run/codex-environment/codex-home`, plus the PATH-alias directory. This is a client runtime-state write problem, not a missing API key or native compiler failure.

The OpenAI repository's [read-only CODEX_HOME issue](https://github.com/openai/codex/issues/51293) contains a maintainer explanation that ephemeral execution still requires writable initialization state. That source supports the startup requirement; the local trace establishes this program's observed failure. The manager made no client-home, filesystem-mount, sandbox or authentication changes to work around it.

`scripts/cloud/RESEARCH.md` could not be read because it is absent in the main checkout and the retained worker worktree. The available `scripts/cloud/README.md` was inspected, but it does not specify the requested existing-login research worker/reviewer workflow. A concise question requesting that guide's location or command was sent while independent setup work continued. No answer was available at this handoff.

## Program state and next allocation

The existing program is active. The old failed worker and separate reviewer remain failed, the sealed startup submission remains unchanged, and one future review obligation is retained. There are zero active worker or reviewer reservations and no unknown ownership. `assignment-holdout-scout-cloud` is a distinct ready commission carrying the original rival explanations, outcomes and twelve-minute data-inspection scope. It has no runtime grant or attempt. Starting it still requires a working child-client workflow and an explicit review-backlog decision through supported tools.

The scientific priorities are unchanged: fair prediction of the actual applied model first, controlled measurement-model sensitivity as the rival, and motor timestep fidelity as another important route. A successful build has not supported or rejected any of those explanations. The eventual scout gets at most twelve extra files, zero fitting, scoring, collection and experiment physics steps. Five minutes of separate review and 190 cleanup seconds should be reserved separately when dispatch becomes possible.
