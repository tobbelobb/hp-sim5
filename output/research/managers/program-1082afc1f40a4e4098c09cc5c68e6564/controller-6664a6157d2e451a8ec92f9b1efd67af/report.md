# Hangprinter's unresolved questions and the cheapest useful test

Status: saved evidence assessed; delegated investigation and separate reviewer blocked at startup. No new scientific admission.

## Abstract

Which uncertainties about Hangprinter should receive the next small experiment? This manager inspected saved calibration, measurement-model and motor-dynamics reports, searched the research catalog and inspected the existing evidence graph. The most immediate calibration gap is a fair prediction comparison of the actual applied anchors and spool model. In the saved CM-E1 comparison, all three changed model pairs have zero common unused whole sweeps. A twelve-minute saved-data feasibility scout was commissioned through the existing research worker, with no fitting, scoring, collection or motion. Its launcher failed at missing Node-API headers before the coding client or investigation ran. The manager therefore recommends the scout and a conditional six-score comparison, while preserving model sensitivity and dynamic fidelity as competing routes. No new scientific result, hardware measurement or accuracy improvement is claimed. The priorities are practical judgments based on inspected reports that have not received a new independent scientific review.

## Topics

Hangprinter, research priorities, encoder-only calibration, identifiability, held-out validation, cable winding, motor dynamics, timestep convergence, hardware accuracy

## What matters and remains unresolved

The recurring uncertainty is whether internally consistent encoder behavior tells us where the nozzle really is. Better fit scores, quieter encoders and a stable simulation each answer narrower questions.

| Priority | Question and consequence | Inspected evidence | Cheapest observation that could change the decision |
| --- | --- | --- | --- |
| 1 | Does the calibration we actually apply predict new measurements? Without this, another ranking improvement may be an artifact of overlap or different test data. | [CM-E1](../../../sessions/2026-10-23-consensus-membership/report.md) reports one improvement, two regressions and five unchanged cases among eight additional dataset names. Every changed model pair has zero common unused sweeps. These are saved simulated histories. | Search compatible later saved sweeps, then compare the two frozen applied models on exactly the same unused observations. No new fit or motion. |
| 2 | Which geometry errors can encoder measurements distinguish from spool radius, winding and cable effects? A good prediction can still describe the wrong physical geometry. | The [fixed-encoder audit](../../../2026-10-09-slip-recovery-and-encoder-model/report.md) reports unchanged residuals after per-point fixed-angle perturbations. It also reports a failed run whose fixed angles varied by at most 0.01 degree. One omitted effect cannot explain every failure. | At known synthetic geometry, compare current residuals against predictions using measured per-point fixed lengths, with a zero-drift control and NumPy/JAX parity. Do not add unconstrained offsets or tune a selector. |
| 3 | Which simulated motor failures and damping behavior are physical? Incorrect integration can reverse a control or torque-margin decision. | The [HP4 Squares investigation](../../../../../research/hp4-squares-substeps.md) reports no losses at one substep but about two magnetic periods of B lag at finer steps. The retained inertia correction passes stability checks; a separate damping correction fails loaded settling. | A short matched native timestep refinement, followed by an independent motor/load measurement if simulation conclusions are to guide hardware. Repeating the existing 1 through 16 substep result adds little. |
| 4 | How much accuracy transfers to hardware, and how much depends on approach direction, tension and winding history? Simulation agreement cannot establish a real nozzle error budget. | The inspected calibration and dynamics sources explicitly limit their conclusions to simulation. The simulator's fixed-axis closed loop is an ideal constraint, rather than a torque-limited hardware controller. This inspection does not establish the absence of hardware evidence elsewhere. | On an already operating printer, measure repeated returns to surveyed points with an independent camera or position reference. Start with approach direction at fixed load, then change one physical condition in a separate contract. |
| 5 | Which movements buy independent information, and when is stopping justified? Low noise and more points can coexist with poor geometric rank. | The [research agenda](../../../../../research/autocal-agenda.md) preserves model adequacy and informative collection as alternatives. Historical candidate and movement studies have mixed outcomes, rather than a general collection rule. | Evaluate information and prediction change for one proposed movement on controlled geometry before paying for another full collection. Hardware accuracy still needs independent measurement. |

These priorities reflect the available repository evidence and setup costs. They are not a complete literature survey or a claim that calibration is always the highest-value problem for every Hangprinter use.

## Selected experiment and its feasibility gate

The cheapest useful next step is a saved-data search. The cheapest numerical experiment it can enable is a paired prediction test of the two already frozen CM-E1 models. Keep each arm's applied averaged anchors and its selected fit's spool parameters together. A member fit's score does not validate that combined model.

The funded scout inspects the three changed cases, `flexible_lines_2000`, `ten_points_w_buildup` and `ten_points_w_buildup2`, and at most twelve additional dataset or manifest files. It verifies content hashes, machine, units/frame, encoder origin, spool/layering, collection settings and the union of all training and selection use. Unknown lineage disqualifies a sweep as a verified holdout. It performs no scoring.

If that gate succeeds, separately commission at most six frozen prediction evaluations, two arms per eligible case, on a frozen common whole-sweep set. Use the existing residual-based score with identical noise and preprocessing settings. Report costs, point and sweep counts, invalid results, overlap and source compatibility. This is residual prediction, not a direct measurement of nozzle position or a fresh encoder forecast.

| Observation | Decision that changes |
| --- | --- |
| A compatible unused sweep exists | Fund the bounded paired scoring assignment. Finding data alone does not establish a prediction benefit. |
| Every bounded candidate is absent, incompatible or has unknown lineage | Park this particular saved-data search and prioritize controlled model sensitivity. Independent validation remains a credible broader idea. |
| One model is invalid on an eligible common fold | Reject that model for that fold and retain the failure. |
| Finite costs differ by more than `1e-8 * max(1, abs(cost_a), abs(cost_b))` | Record a numerical prediction difference only. A disagreement with physical anchor error raises the priority of model adequacy. |
| Costs tie within that threshold | The tested observations do not discriminate these arms. Do not tune another membership threshold on the same cases. |
| Launcher, client or reader fails | Record an engineering blocker. Change no scientific conclusion. |

The cost contract is twelve worker minutes for feasibility, zero fresh fits, zero prediction evaluations, zero collections and zero physics steps. Five separate review minutes and 190 cleanup seconds were reserved. The conditional six-score experiment has not been funded. These are allocation estimates and limits, not measured execution costs or dollar quotes. The actual failed startup used about 5.57 seconds of observed worker wall time.

## A small hardware experiment, when hardware is available

If the practical question is real print accuracy, the cheapest informative hardware test uses an independent position reference. On an already functioning printer, choose five accessible points in one surveyed plane. Return to each point from two approach directions, three times each, at fixed speed, load and winding setup. This gives thirty position observations. Record encoder positions and independent position measurements; establish camera or gauge uncertainty using stationary repeats and a known scale before interpreting the differences.

An approach-dependent displacement exceeding three pooled standard errors of the measurement difference is evidence of a repeatable path-history effect in that plane. Set a separate application accuracy tolerance before measurement. Consistent returns within measurement uncertainty bound repeatability only for those points and conditions. They do not prove absolute accuracy, full-workspace accuracy or vertical accuracy from one planar camera. A second physical condition needs its own bounded comparison to separate tension, stretch and winding effects.

No hardware access, camera, gauge, safe movement envelope or purchase price was established in this conversation. This is an unfunded proposal, with zero incremental equipment purchase only if a suitable reference is already available. The saved-data experiment remains cheaper to start here.

## Actual execution and evidence boundary

Program `program-1082afc1f40a4e4098c09cc5c68e6564` contains five open questions, three route plans, source hashes, outcome-dependent actions and one commissioned scout. The [operation transcript](manager-operations.jsonl) records the actual calls. [Overview](overview.md) and [brief](brief.md) are generated by the controller.

Worker `attempt-2a0ba36011304a0c844d06b70fd0b734` used the existing launcher in its own worktree. It exited 1 at `Node-API headers are missing`, before research-client execution. The controller observed `launcher_finally_completed`. This is a startup cleanup observation, not evidence about stopping a running physical world. See the [manager-authored startup report](../../../sessions/attempt-2a0ba36011304a0c844d06b70fd0b734/report.md) and its evidence. The original launcher notebook is retained with a manager addendum.

Fixed submission `submission-8dcfa72e3f48459abdf6b894c2932a5d` contains the manager-authored startup report, abstract, controller status and launcher stderr. Separate review `review-run-e2f622e5447b400b956df399f29965ea` exited 1 at `failed to initialize in-process app-server client: Read-only file system`. It produced no assessment or review ID, and `review_apply` was not called. The predeclared `failed` strategy outcome was recorded with no route actions. [Review status](review-status.json) and [reviewer stderr](reviewer-stderr.log) retain this independent engineering failure.

No production source changed, no physics delivery is proposed, and no numerical or hardware experiment completed. The previous [live-manager release blocker](../../../sessions/2026-10-24-live-manager-release/report.md) remains relevant. This run does not establish end-to-end manager release readiness.

## Tested hypotheses

| Hypothesis | Experiment ID | Outcome | Evidence |
| --- | --- | --- | --- |
| A common unused compatible sweep exists in the named search scope | `assignment-holdout-scout` | Not tested; launcher failed before inspection | [Startup report](../../../sessions/attempt-2a0ba36011304a0c844d06b70fd0b734/report.md) |
| Frozen applied models differ on a fair prediction fold | None | Proposed, not tested | Experiment contract above |
| Current per-point modeling or motor dynamics explain the remaining accuracy gap | None | Not tested in this program | Rival route plans in [overview](overview.md) |

## Decision handoff

- Scope: inspected saved simulation reports at checkout `8acde0c21d5b9db8893115e3e79d82d9e41f98ca`; one failed launcher startup. No hardware inference.
- Review: historical findings were inspected by this manager. The separate startup reviewer failed before assessment, so this program has no reviewed scientific admission.
- Decision: keep current calibration defaults and all three credible routes. The tool failure retires none of them.
- Next allocation: none currently running or queued. The unused grant is closed and new dispatch paused. One future review obligation remains for the sealed startup submission, with zero active workers or reviewers. After writable authenticated client state works, fund five minutes of separate review of that retained submission and apply only a valid assessment. After that obligation clears and normal native-build prerequisites work, create a distinct twelve-minute feasibility assignment with the same scientific scope. Fund paired scoring only after verified holdout discovery. Do not spend another launch on unchanged startup blockers.

## References

- [CM-E1 report](../../../sessions/2026-10-23-consensus-membership/report.md) and [manifest](../../../sessions/2026-10-23-consensus-membership/inputs.json) identify the applied-model gap and the three cases.
- [Autocal agenda](../../../../../research/autocal-agenda.md) supplies the bounded common-fold method and rival model explanation.
- [Fixed-encoder audit](../../../2026-10-09-slip-recovery-and-encoder-model/report.md) supplies the reported residual invariance, drift-independent failure and ideal-controller limitation.
- [HP4 Squares investigation](../../../../../research/hp4-squares-substeps.md) supplies the quantitative timestep and damping findings.
- [Existing graph seed](../../../../../research/program-graph/autocal-seed.json) preserves related unresolved questions and source limitations. Imported records have no automatic scientific admission.
- [Prior live-manager report](../../../sessions/2026-10-24-live-manager-release/report.md) documents the earlier runtime blockers. This run observed its own failure separately.
- [Operation transcript](manager-operations.jsonl) records source SHA-256 values, route rationale, budgets and actual program operations.
