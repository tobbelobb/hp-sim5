# Autocal research allocation after mixed consensus evidence

Status: complete (desk exercise; next numerical assignment not tested).

## Abstract

What should receive the next research allocation after broader replay rejects
blanket finite-prediction consensus membership? This desk exercise compares saved
consensus, ranking, solver-attribution and model-adequacy evidence with current
selection code. CM-E1 reports one gain and two regressions across eight additional
datasets; all three changed comparisons lack common unseen sweeps. The agenda
therefore retains production defaults and proposes a bounded compatible-sweep
search, with at most six later frozen prediction evaluations if eligible data
exist. It compares four routes and demonstrates how withdrawing CM-E1 would
replace that allocation with evidence recovery while preserving other findings.
Direct claim links suffice; no graph infrastructure was added. Saved results
were inspected, not independently reproduced. Some historical raw artifacts are
unavailable here. No fits, selection/prediction experiments, physics, collection
or hardware measurements were performed; improved calibration accuracy is not
established by this workflow result.

## Topics

autocalibration, research agenda, common unseen sweeps, consensus, model adequacy,
evidence dependencies, negative results, withdrawal exercise, research allocation

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Direct evidence links suffice for a scoped allocation and next brief | AG-D1 | Supported for this four-route desk exercise | [Agenda](../../../../research/autocal-agenda.md), [checks](checks.json) |
| Removing a supporting result changes the allocation while preserving other scope | AG-D2 | Supported for a counterfactual CM-E1 withdrawal | [Exercise](../../../../research/autocal-agenda.md#withdrawal-exercise-what-if-cm-e1-is-unavailable) |
| The proposed common-fold probe improves calibration | None | Not tested | [Proposed assignment](../../../../research/autocal-agenda.md#next-assignment-common-unseen-sweeps-for-changed-applied-models) |

## Decision handoff

- Scope: task 005, evidence/source inspection at fd10fd94; no runtime changes.
- Review: this agent inspected saved reports, CM-E1 JSON and current consumers;
  hashes/links/queue/catalog checked. No independent numerical reproduction.
- Decision: keep defaults; use the linked agenda and defer graph tooling.
- Next: commission the compatible-unseen-sweep assignment; absent eligible data,
  scope model adequacy separately. The numerical assignment remains proposed.

## References

- [Full report](report.md), [notebook](research.md), [verification](checks.json): actual work and limitations.
- [Working agenda](../../../../research/autocal-agenda.md): route priorities, claim dependencies and bounded next assignment.
- [Producer CM-E1](../2026-10-23-consensus-membership/report.md), [saved measurements](../2026-10-23-consensus-membership/results.json): reused scientific finding.
- [Handoff procedure](../../../../research/handoffs.md): durable evidence recovery.
