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
