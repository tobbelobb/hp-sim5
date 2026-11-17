# HP5 redundant-sensor comparison

## Constraints

Manual first-manual-autocal-of-hp5-pulley-reduced output and services are read-only.
Requested DeepSeek V4.1 Flash 1017/OpenCode workers use isolated headless worlds.
The latest user instruction authorizes production implementation on main, end-to-end
validation and at most three coding/test passes. The final delegated investigation
was capped and has finished; no more research agents are needed.

## Reviewed direct findings

Independent replay confirms persistent low-force take-up slack and taut redundant
lines at 0.3 N in the tested short sweeps. This is not a universal force threshold.
The broad-pose synthetic extra-range benefit was not observed on the narrow
physical sweep pose set. Constant bias is absorbed by home offsets; pose-dependent
slack remains dangerous. Original statistical mode labels and reproduction script
needed corrections; source submission and reviewed replay are both preserved.

## Reduced findings pending review

Matched short prototype collector recordings favor measurement-only pretension
plus five-range fitting over either discarded extra ranges or raising idle force.
The extra range tracks motion while the current low-force encoder is frozen.
Actual pose truth, elastic-deformation bounds, holdout design, sigma ratios and
cogging causality need the commissioned independent review's interpretation.

## Next decision

Await the reduced review via platform completion. Do not poll or start more runs.
Then publish a cross-machine synthesis with scope and limits, update the topic,
and retain compact evidence. Recommend only what the reviewed recordings support;
no evidence yet establishes faster full production calibration or hardware safety.

## Extension authorized after short-study reviews

The reduced saved-data review reproduces the same-recording five-range gain but
corrects control force labels, inferred-pose bias, covariance claims and holdout
offset leakage. The direct review finds no physical-sweep fitting gain yet.
Commission one isolated integrated-prototype investigation: preflight force/range
bookkeeping and observable validity handling, then at most one fresh baseline and
one treatment full autocal on reduced HP5. Stopping thresholds and physics budgets
stay fixed. Record absolute and aligned anchor errors, accepted/rejected sweeps,
steps, sim/wall time, stop reason and all inputs. No direct full run initially.
The user's other research program and manual dataset remain untouched.

## Implementation decision (supersedes the earlier waiting steps)

Both independent reviews have finished. They support measurement-only pretension
and explicit fifth-range fitting, while correcting several original numerical
claims. The final worker completed a fresh short prototype preflight but stopped
the baseline full run at its deadline; treatment full calibration was not run.
There is no paired full-loop result from that worker.

Production pass 1 now implements profile-planned extra force, per-point reversible
coupling checks, physical-axis range/force records, all-line static stretch and
home-offset handling. Rough bootstrap and historical profiles keep their previous
policy. Fresh direct and reduced production headless calibrations are running in
output/extra-sensor-production/pass1, with no contact with the manual run or the
other research program. Confirm normal completion/application and extra-range
acceptance, inspect anchor error and a matched four/five-range ablation, then
publish code and evidence. Additional coding passes are reserved for actual failures.

## Completion

Implementation committed as 089a7ad3. Both final continuations complete normally
and apply parameters. Three coding/test passes are exhausted. Direct same-recording
refit improves; reduced late-data refit worsens despite a good selected estimate.
Publish this limitation rather than claiming universal accuracy or speed. Further
force/bias work is separate; no more experiments are commissioned here.

## Integrated pipeline delivered

68260871 implements sensing from the start, reuse of force-aware bootstrap,
fifth-range planning, force-aware information, frozen validation/model selection
and early good-fit acceptance. Three passes used; two fresh simulations per
machine and final replay/apply completed. No further experiments commissioned.
