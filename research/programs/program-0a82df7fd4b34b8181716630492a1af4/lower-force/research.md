# HP3 lower measurement force

Status: in progress. No calibration result yet.

Six sequential current-code pairs, nominal then development then heldout evaluation. Candidate sensor-4mid-v1 is frozen before primary results. Source a68d6adf.

Main explanation: lower measurement force reduces equilibrium bias. Rival: motor drag/noise erases benefit or increases effort above1.20x. Cheapest falsifier: fixed paired applied readback results.

Manager agreed pre-result margin: evaluation applied maximum error improves by at least10% and0.5mm, development maximum also improves, disclose nominal regression, no new failure, each completed pair <=1.20x physics steps/sim time. The human endpoints remain all anchors<5mm good and any>20mm or incomplete failure. No hardware reliability claim.

Budgets:100worker minutes including10report reserve;7collections,80fresh fits,28Msteps,2000points;4Msteps/8000sim seconds per collection;6auto iterations.15review minutes controlled by manager. Runs sequential. Raw evidence pushed after every collection.

Read complete queued handoff/current continuation/AGENTS. Verified59 recovered setup files into session only. Source imports rebuilt from current upstream. No manual engine.prepare. Supervisor preflight and independent socket probe passed. Attached supervisor has stock layering enabled; isolated primary harness explicitly uses config_hp3.g, zero buildup, known30mm radii and layering off, verified in each primary manifest.

## n-default

accepted_below20; stop=spread-consensus; normal acceptance=True.

Applied per-anchor error mm: {'A': 2.759438348649989, 'B': 4.547215890556576, 'C': 1.164774359856155, 'D': 1.7423260314878015}. Maximum 4.547215890556576, sqrt(mean of four squared anchor distances) 2.8584992419409856. Every<5: True; anchors>20: 0.

140 physical points, 7 sweeps, 9 fits, 2156688 steps, 4313.376 simulated seconds, 739.4404828699999 wall seconds. Wall time depends on host load. Intermediate max tail 3508.881327566772 mm, invalid fits 0.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

## n-candidate

workflow_failure; stop=None; normal acceptance=False.

Applied per-anchor error mm: None. Maximum None, sqrt(mean of four squared anchor distances) None. Every<5: None; anchors>20: None.

50 physical points, 0 sweeps, 0 fits, 1347144 steps, 2694.288 simulated seconds, 406.4474101080002 wall seconds. Wall time depends on host load. Intermediate max tail None mm, invalid fits 0.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

## Manager steering after nominal transport failure

The nominal candidate failed after50points with M569.3 encoder reply timeout5000ms before fitting. Native error was null, physics exited0, both owned children were absent at17:58:08UTC. No OOM/kill event was observed. Shell reads also stalled, but cumulative throttling and host stalls do not establish cause. Preserve workflow failure and uncertainty. Manager agreed to finish development/evaluation four first, keep report reserve, change no timeout or policy. A seventh unchanged nominal confirmation may be funded separately if useful and cannot fit current launcher deadline. No silent budget extension.

## Reporting count clarification

After the nominal transport failure, the evaluator now counts completed physical sub-sweeps from final step_index in all partial point journals and separately keeps dataset direction entries and started headings. Nominal candidate has5completed of6started physical sub-sweeps despite no finalized dataset. Old raw archives stay immutable. This changes reporting only, with no measurement/force/solver/stopping change.

## Separate follow-up authorized before its execution

Manager decision decision-f6e3ee17d877459b915ada44a1b7944c authorizes at most5worker minutes after all6primary jobs and only with report reserve remaining. Hypothesis: the shared M569.3 bridge silently caps an explicitly longer configured encoder budget at5seconds, causing a recoverable valid delayed response to fail. Bounded discriminator is real-WebSocket stock-vs-fix delayed reply plus effective-default5s, shorter command deadline, missing/disconnected failure and numerical-payload regressions. No extra collection, no primary source changes, no failure removal or hardware-accuracy claim. Primary transport cause remains unestablished.

## d-default

accepted_below20; stop=spread-consensus; normal acceptance=True.

Applied per-anchor error mm: {'A': 2.499299901972569, 'B': 3.695888659558299, 'C': 3.36598989218654, 'D': 2.820638225650294}. Maximum 3.695888659558299, sqrt(mean of four squared anchor distances) 3.1301749527044542. Every<5: True; anchors>20: 0.

140 physical points, 7 sweeps, 9 fits, 2161548 steps, 4323.0960000000005 simulated seconds, 643.4076395029999 wall seconds. Wall time depends on host load. Intermediate max tail 1911.6000238647873 mm, invalid fits 0.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

## Seventh collection decision

After d-default, manager declined a seventh unchanged nominal candidate pass solely to replace the failed primary. Under the frozen no-new-failure gate it cannot erase the counterexample and cannot change adoption without isolation evidence. Finish3remaining primary jobs, authorized <=5min separate transport follow-up if reserve remains, reports/checks. Seventh stays unused unless a specific unresolved decision justifies its cost.

## Failure wall-clock reporting correction

The auxiliary runner timestamp difference needs a fallback when failure omits exit.json. Corrected n-candidate summary primary_process_wall_s to manifest406.4474s. Main reported wall_s was already correct. The old raw verified-result retains the original auxiliary-field error; its archive stays immutable. No physical metric or policy changed.

## Controlled continuation contingency

Manager instructed: after devpair checkpoint, start evaluation only if BOTH remaining primary jobs plus pushes and10min reporting reserve fit conservatively in live100min deadline. Otherwise seal four-primary controlled partial evidence with evaluation NOT TESTED. Manager will review partial and fund fresh bounded continuation for only unrun evaluation pair and focused transport work; do not repeat successful jobs or extend current deadline. Aggregate caps remain7collections/80fits/28Msteps/2000points.

## d-candidate

accepted_below20; stop=spread-consensus; normal acceptance=True.

Applied per-anchor error mm: {'A': 2.559882809817639, 'B': 2.8528975546135613, 'C': 2.222780471118976, 'D': 2.460508077613276}. Maximum 2.8528975546135613, sqrt(mean of four squared anchor distances) 2.5341111597514803. Every<5: True; anchors>20: 0.

140 physical points, 7 sweeps, 9 fits, 2300140 steps, 4600.28 simulated seconds, 913.0879748390003 wall seconds. Wall time depends on host load. Intermediate max tail 2200.8462674411935 mm, invalid fits 1.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

## Controlled partial boundary

At18:28:35UTC, d-candidate completed in913.09s. A conservative evaluation-pair estimate2*(15.22+2)min plus4minpushes and10minreportreserve is48.44min, exceeding roughly41minremaining. Per manager instruction neither evaluation arm starts. Four primary attempts are sealed. Evaluation pair and separate transport experiment are NOT TESTED. Only these unrun jobs move to a fresh finite continuation. The lower-force production default remains unadopted, and nominalfailure remains in the denominator.
