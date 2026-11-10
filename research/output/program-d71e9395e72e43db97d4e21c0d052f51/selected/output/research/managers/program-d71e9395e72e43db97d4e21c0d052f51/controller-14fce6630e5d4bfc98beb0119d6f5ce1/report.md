# Common unseen-sweep validation: scoped plan, controller blocked

Status: blocked.

## Abstract

Can saved observations fairly compare the two applied calibration models in the three changed CM-E1 cases? The manager inspected the existing research agenda, CM-E1 report and input manifest, then scoped a small data-lineage investigation. CM-E1 reports zero common unseen sweeps for these cases; that is prior reported evidence, not a new result. No worker, reviewer or numerical experiment ran. The bound overview tool rejected an unverifiable controller; reopening failed while its file lock remained held and its PID was invisible in the command namespace. External controller health remains unknown. A proposed 12-minute worker contract, separate six-minute review capacity and cleanup allowance are preserved. Scientific findings remain unreviewed and the requested first iteration is incomplete.

## Topics

Autocalibration, consensus membership, held-out sweeps, data lineage, model adequacy, controller ownership.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Compatible saved sweeps exist outside both complete applied models' training lineage | None | Not tested | [Proposed contract](proposed-investigation.json) |
| Common-fold prediction distinguishes the two models | None | Not tested; outside this nonnumerical trial | [Prior CM-E1 report](../../../sessions/2026-10-23-consensus-membership/report.md) |

## Decision handoff

Keep current consensus defaults as a manager allocation decision based on the prior report. No scientific admission was made. Resolve controller visibility/ownership through supported platform tooling before commissioning the proposed lineage scout. If no compatible common fold is found and separately reviewed, prioritize scoping model adequacy. A feasible fold alone would not authorize changing defaults.

## References

- [Full report](report.md): inspected evidence and execution boundary.
- [Research notebook](research.md): objective, constraints, budgets and steering.
- [Runtime observations](runtime-observations.json): actual tool errors and ownership checks.
- [Proposed investigation](proposed-investigation.json): finite scope, competing explanations and predeclared decisions.

## Inspected prior evidence and rationale

The [October 23 agenda](../../../../../research/autocal-agenda.md) recommends a bounded common-fold search before additional membership or ranking rules. The [CM-E1 report](../../../sessions/2026-10-23-consensus-membership/report.md) reports one improvement, two regressions and five unchanged cases among eight additional dataset names. All three changed comparisons had zero common unseen sweeps for the full applied anchor-plus-spool models. Those reported results reject unconditional nonworsening within the saved comparison and leave predictive safety unresolved. No historical measurement was recomputed here.

The [input manifest](../../../sessions/2026-10-23-consensus-membership/inputs.json) and saved results are present. They identify the three cases as Slideprinter data; HP3-like filenames elsewhere in the corpus do not establish machine identity. The proposed worker must check observation contents, complete training lineage and settings before calling any data unseen. The competing model-adequacy route is retained because a selector failure does not falsify encoder-only calibration or establish the model's correctness.

## Operational boundary

The bound `tool ... overview` returned `controller_unavailable`. The supported `--cloud --program program-d71e9395e72e43db97d4e21c0d052f51` reopen returned `Controller startup failed; inspect controller.log`. The log was empty. A nonblocking advisory-lock check found the controller lock held, while the descriptor PID was absent from `/proc` in the command environment. Separate command calls exposed different PID namespace identities. This explains why local process verification cannot establish controller ownership; it does not establish that an external controller is dead. The descriptor records local transport. No transport switch or replacement controller succeeded.

The offline ordinary `overview PROGRAM_ID` command read durable memory successfully. It showed the saved first-trial goal and no attempts. Manager steering, alternatives and allocation rationale can be retained through offline CLI operations. Live capabilities, worker dispatch, fixed submission and separate review cannot be used from this environment at present. No pending bridge request was dispatched, no child output was fabricated, no historical PID was signalled and no permission change was attempted.

## Next bounded decision

The saved contract caps three base histories and twelve additional files, with zero fits, predictions, physics, collections, movements or recordings. Proposed capacity is 25 minutes including a 12-minute worker contract (three minutes of startup margin), six-minute separate review, three minutes of manager/dispatch margin and 190 seconds for cleanup. Capacity has not been allocated and the contract has not been commissioned. Actual startup failure is an operational finding, not a negative scientific result.

If a compatible unseen set is verified, retain a feasibility result and scope any later scoring separately. If none qualifies or lineage remains unknown, the claim stays inconclusive and model adequacy becomes the next useful route to scope. Restore a supported verified program connection before either outcome can be investigated and separately reviewed.

## Delivery and cleanup

No physics, motor, transport, dynamics or source code changed. No new worker or review attempt was started, so this conversation owns no such process to cancel. Existing external controller health remains unknown. Catalog regeneration and a research-only save will preserve these notes; Git commit status is recorded in delivery-status.json. The original requested completed, separately reviewed investigation remains unfinished.
