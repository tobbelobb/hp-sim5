# Small cloud runtime tests leave autonomous research unverified

Status: complete.

## Abstract

This saved-evidence review asks whether bounded cloud-native runtime tests establish calibration improvement or an unattended research release. No new experiments were run. The runtime summary records two 100-step rust-native calls of 0.2 simulated seconds each. A separate launcher-attached HP3 run records 100 steps, 101 telemetry samples and 101 RRD samples with a readable blueprint. This review checked the latter run's six saved artifact hashes against its validation record. These observations support bounded production-tool and recording feasibility. They contain no fitted candidate comparison, held-out prediction evaluation or hardware measurement. Calibration improvement remains inconclusive and was not tested. The separate run's report explicitly leaves platform dispatch relayed by the parent chat and excludes interruption/recovery and separate review. Full unattended release therefore remains unverified. Continue the existing-login cloud adapter route with a bounded lifecycle and independent-review gate before any scientific calibration study.

## Topics

cloud agents, existing login, HP3, rust-native, MCP, Rerun, calibration evidence, unattended release

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Bounded production calls and recording are feasible | 8fd9086bb97e40df815b9236ece792e5; a5a83009ee5a4079a1da693294ddd8aa; 8f977de942c64d7391f63c3198ba0c9d | Supported within saved test scope | [Runtime summary](../2026-10-24-cloud-research-runtime/production-validation.json), [production result](../attempt-d35a0ef8b1b54fc4a14bc9a1ba28aa9f/production-result.json) |
| Calibration improves | None | Not tested; inconclusive | No fit or held-out comparison in the cited tests |
| Full unattended research release works | None for full lifecycle | Not tested; inconclusive | [Prior report](../attempt-d35a0ef8b1b54fc4a14bc9a1ba28aa9f/report.md) identifies remaining dispatch and lifecycle limits |

## Decision handoff

- Scope: saved simulated runtime tests at recorded revision 8acde0c21d5b9db8893115e3e79d82d9e41f98ca. No hardware inference.
- Review: this cloud worker inspected saved records and checked artifact and available native source hashes. No numerical recomputation or new RRD decoding.
- Decision: keep the existing-login cloud adapter as a feasible bounded route. Block claims of calibration improvement or full release pending matching evidence.
- Next: a proposed, untested manager-to-launcher-to-cloud-worker-to-separate-reviewer lifecycle gate with durable completion, cancellation and recovery evidence. Then separately authorize a calibration comparison with independent held-out evaluation.

## References

- [Full report](report.md) explains inference limits.
- [Research notebook](research.md) records allocation and steering.
- [Validation](validation.json) records checked hashes and zero numerical work.
- [Previous cloud worker](../attempt-d35a0ef8b1b54fc4a14bc9a1ba28aa9f/report.md) supports transport feasibility and identifies untested release behavior.
