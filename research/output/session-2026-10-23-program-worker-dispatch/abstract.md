# Existing research worker dispatch with observed ownership and cleanup

Status: complete task 009 tool implementation; live manager release not established.

## Abstract

Can the manager dispatch a bounded assignment through the existing research
launcher while retaining ownership and evidence? A standard-library adapter on
ProgramStore now reserves one worker, writes an exact assignment/budget prompt,
launches the existing batch route, and records process birth identity, source,
returned session/worktree paths and partial artifacts. Start/status/cancel remain
responsive. Zero exits remain unreviewed submissions. Fake child fixtures cover
normal/failing/auth-blocked launches, cancellation, partial manifests and stale
PID refusal. Actual launcher regressions with fake clients/services verify
interrupted runtime/Viewer startup cleanup and cleanup-error reporting. The final
adapter/launcher check passed 57 tests; a later import-validation change passed
all 10 adapter tests. Seventeen earlier store/graph checks are reused. Four saved
real launcher dry-runs cover Codex/Claude batch and worktree arguments. They start
no client or physics world. Missing runtime dependencies and nested Codex writable
state prevent inferring live readiness. No scientific experiments or admission
occurred; live worker/review/graph proof remains task 016.

## Topics

research manager, worker dispatch, ProgramStore, process ownership, cancellation,
launcher, worktree, Codex, Claude, batch, partial artifacts

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| One bounded assignment can be dispatched and observed without accepting science | W009-FIXTURE | Supported tool check with fake launchers | [Checks](checks.json) |
| Cancellation reaches actual launcher cleanup even during startup | W009-LAUNCHER | Supported with fake services and a real signal | [Checks](checks.json) |
| Both supported client argument routes survive prompt/worktree handling | W009-DRY | Supported launcher dry-run only | [Four outputs](dry-run-records.json) |
| A live worker can complete production research and independent review here | None | Not tested; prerequisites unavailable | [Parent observations](prerequisites.json) |

## Decision handoff

- Scope: process/store/launcher behavior, fake clients/services and real argument dry-runs.
- Review: implementation worker inspected and ran focused tests; no independent scientific review or numerical reproduction.
- Decision: task 009 is complete; submitted worker output remains unreviewed and unassessed.
- Next: task 010 adds fixed submissions and separate review; task 016 must supply actual live proof.

## References

- [Report](report.md): implementation, observations and limitations.
- [Notebook](research.md): objective, budget, steering and next decision.
- [Adapter guide](../../../../research/program-workers.md): actual API and lifecycle boundary.
- [Test-tool provenance](test-tool-provenance.json): parent-staged released pytest sources used for regression execution.
