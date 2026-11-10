# HP3 encoder-reference experiments

Status: completed bounded study through E17. Fresh centroid collection slightly improves accuracy but uses more data; prior broad retrospective interpretation corrected. See report.md, fresh-results.json and fresh-manifest.json.

Objective: reproduce score/geometry disagreement and test whether re-referencing saved encoder data near the estimated anchor center or an asymmetric reachable point improves robustness or reduces data requirements. Preserve one-click collection and automatic stopping.

Accepted steering: use HP3; known spool radii; zero buildup and disabled layering. Ground truth is evaluation-only. Start with existing datasets and offline transformations, then decide whether live collection is justified.

Constraints: small source changes, local `.venv/bin/python`, headless production backends for any collection. No physics/motor/scene changes planned. Preserve source datasets and failed trials. No external papers used yet.

Hypotheses:
- H1: existing HP3 no-buildup candidate scores disagree with geometry (baseline reproduction).
- H2: changing reference to an observed point near the estimated anchor centroid improves solve reliability without extra measurements.
- H3: an asymmetric edge reference improves reliability more than a central one.
- H4: successful reference selection transfers to smaller prefixes/subsets (not tested yet).

Budget: initial offline phase up to 60 bounded fits, 20 CPU minutes, no movement, no RRD (numerical evidence only). Expand only after reviewing pilot results. If live collection is justified: preflight plus command-context socket probe first, events recording, HP3/layering-off configuration checked.

Preceding evidence: read research/index.md and solver/warm-start studies 98170696 and 9d784051. Their thinning and score-replacement failures argue for paired tests and recording all intermediate candidates. Current main is 2c0b86ff189d27859b83543ed3fa837cfa658a3b; preexisting untracked research/prompts/autocal_move_the_origin_idea.txt is user material.

Design concern: a saved point supplies four length deltas but only three Cartesian coordinates. Treating all four deltas as an exact new zero can constrain noisy data differently. Restore the original reference without ground truth by trilateration and measure closure error. Compare rigid-invariant pairwise anchor error as well as original-frame error; never treat a guessed Cartesian shift as known.

Initial plan: E01 baseline audit; E02 transformation invariance; E03 paired observed-reference pilot. Completed experiments and expanded IDs are recorded below.

Next decision: implement and verify offline reference transformation; choose several independently collected no-buildup HP3 datasets with known provenance.

## Results and steering, offline pilot

E01 completed current-main full-auto replays on REF, W00 nominal, W08 heavy/high-gravity HP3 recordings. Applied RMSE 2.690, 2.508, 3.689 mm; raw score/geometry disagreement reproduced. Retained all stage artifacts and stdout. An initial analysis used the JSONL log as text and omitted selected warm candidates; corrected by reading history artifacts and parsing the console summary with fixed radii. No runs were discarded.

E03 (18 fits): first-three-sweep reference choices, prefixes 3/5, lbfgsb, same first-fit seed translated for each reference. Center shifts worsened all six fits. Extreme edges caused 191–2796 mm errors. E04 (18 fits): all available sweeps, lbfgsb/trf, same negative trend.

Found a confound: the extreme edge moves a low anchor outside the production solver's quadrant bounds. E03/E04 are retained as unrestricted-reference failure evidence, not a fair rejection of feasible edge references. E05 (54 fits) corrects this by selecting the farthest reference whose translated seed respects all original bounds; choices use only points present in each prefix. Prefixes 3,5,all; both solvers. Center still worsens all comparisons; feasible edges slightly improve the three-sweep lbfgsb fits but are worse at larger prefixes and can destabilize trf at three sweeps. No ground truth used in point choice, fitting or restoration.

E02 initial sensitivity script failed after REF because it selected an invalid cold W00 fit; failed stdout retained. E02-corrected uses the actually selected history plan. It compares exact fitted-geometry centroid/asymmetric translations, finite-difference residual Jacobians and restoration closure, without another solve. It is a local numerical diagnostic, not hardware evidence or proof of global identifiability.

Budget expansion justified by ~0.2–3 s observed fit calls: finish six exact requested CLI replay trials (first-three-sweep center/feasible-edge selection), then an idealized synthetic center check if needed. Continue numerical-only, zero new movement; total offline compute remains bounded to 20 minutes. No production collector or stopping changes justified by results so far.

Next decision: check the complete autocal.py path on rebased files, including automatic stopping and restoration; distinguish a nearest available point (still 344–549 mm from estimated centroid) from a truly centered reference.

## Final decision and completed evidence

E05: 33/36 feasible shifts worsened matched original RMSE; all 18 center shifts were worse. The only wins were three-sweep lbfgsb edge fits (mean 8.813 -> 8.345 mm), insufficient to match full-data accuracy and not transferable to trf.

E02-corrected/bootstrap: exact geometry translations preserve normalized residuals within 4.4e-11. Centroid-reference Jacobian conditioning improves modestly at three sweeps but worsens at five. These local, coordinate-dependent diagnostics do not establish removal of a physical ambiguity. A truly centered collected dataset remains not tested.

E06: six full CLI replays. Original/center/edge applied RMSE: REF 2.690/5.467/3.797 mm, W00 2.508/4.123/3.049, W08 3.689/7.006/13.323. Both W08 shifted files run out of recorded sweeps and terminate at --no-collect, rather than hitting automatic consensus stopping. This limitation is recorded explicitly; success exit codes are not evidence of completed live calibration. All source inputs verified unchanged.

E07: 18 restored-seed fits using the original data/objective. Three/five-sweep fits return the originals; full-data proposals worsen or fail. W00 center seed even improves original cost while worsening truth, so selecting by that scalar would still fail. No production seed/selection change accepted.

E08: CLI shift/restore checked against the saved fit; both JSON and console-summary restoration work, source hash/offset metadata retained, overwrite rejected. Targeted tests: 32 passed in 2.03 s. Numerical plot inspected visually. Retained source changes: experimental autocal/tools/rebase_dataset.py, its meaningful invariance/closure tests, and README usage. No physics/motor/transport/scene implementation change; no live or browser experiment. No new RRD, doctor or preflight needed because no collection was attempted.

Measured workload: E01 three complete original replays, E03 18 fit calls, E04 18, E05 54, E06 six complete shifted replays, E07 18 fits; sum of measured call wall times ~320 s. E07 overlapped part of E06, so no performance comparison is drawn. Numerical phase stayed within the expanded 20-minute compute bound and used zero movement.

Reproduction variants: experiments.py pilot --label NEW --prefixes 3 5 --reference-max-sweeps 3 --edge-policy unrestricted reproduces the E03 design; --prefixes 0 --solvers lbfgsb trf --edge-policy unrestricted reproduces E04; --prefixes 3 5 0 --solvers lbfgsb trf (default feasible) reproduces E05. Use fresh output locations. Analysis fixes did not rerun or delete failed fits.

Accepted outcome: reject observed-center/edge rebasing as a default policy on this evidence; preserve one-click collection and current automatic stopping. No safe fewer-point or end-to-end speed improvement claimed. Report and abstract contain hypothesis outcomes, source references and limits. Next decisions are proposed only: explicit reference-error covariance, per-point fixed lengths, shared candidate validation movements, or a bounded newly centered collection after proper live preflight.


## E09 follow-up: medians and effective reliability

Accepted steering: user asks for median scores and failed runs/iterations versus default settings. Re-analyze saved data; no new movements or optimizer calls. Separate production underconstrained flags, >100 mm restored geometric error, non-restorable geometries, process exits and autonomous completion. Default full-auto L-BFGS-B only in the primary comparison; matched-seed/TRF matrices remain separate.

Measured: final median fit score original/center/edge 0.1928/0.2704/0.2411; final median anchor RMSE 2.690/5.467/3.797 mm. Among 17 chosen iterations per reference, >100 mm failures 0/7/5. Cold variants have >100 mm counts 5/11/11, with three additional unrestorable edge cases; production underconstrained flags 4/2/8. Thus fewer center underconstrained flags do not establish better physical reliability. Default warm variants shield all its chosen iterations from >100 mm errors; shifted warm variants are less effective. No process exits failed, but shifted W08 jobs exhaust replay rather than hitting the automatic stop. Only three simulated datasets: no population failure-rate claim.

The first audit failed when trying to restore a cold candidate with nonpositive implied original cable lengths. Preserve E09-initial-failure.log; corrected E09-reliability.json counts these explicitly. Artifact costs verified against per-iteration log metrics. No source/solver change required. Added follow-up to report/abstract; next decision unchanged.

## E10–E12 planned causal audit (user steering)

User challenges the almost uniformly worse shifts and asks whether origin assumptions or ground truth leaked into production. Reopen the interpretation: prior results establish behavior of the implemented rebasing workflow, not that the physical original point is universally preferable.

Budget: at most 30 new offline stage fits, 5 minutes measured fitting; no collection, physics or browser changes. Audit production seed/bounds/warm-history code, perturb/remove stored M669 metadata as a truth-leak negative control, test re-zeroed encoder metadata versus length-only rebase, and compare noisy observed offsets with fitted-geometry-consistent offsets. Keep all earlier evidence. Next decision: distinguish broken coordinate invariance, changed likelihood and bad warm-history initialization before drawing a causal conclusion.

## Fresh centered collection steering (E13 onward)

User explicitly authorizes minor collector/USDA changes and fresh HP3 recordings, suggesting speedup 25 and fixed forces 0.0223/0.4455/6.5739 N. Accepted plan: compare fresh baseline and actual centroid-origin scene, then decide on repetitions/full automatic jobs. Avoid the noisy fixed-offset rebase. The core solver/collector is unchanged; per-trial scene/config inputs are frozen under the stable session directory. Shift the static anchor/spool assembly relative to the unchanged effector so its initial physical reference is genuinely different. At p=(0,0,265) mm the virtual four-anchor centroid is at the effector reference. Preserve native noise/settling and raw encoder sampling.

Budgets: initial two three-sweep bootstrap trials, <=2.5 million physics steps and <=600 s wall each; sampled recording for inspection. If valid, two full-auto trials with default stopping and <=5 million steps/600 s each, events recording; optional one repeat pair only after reviewing results. Known 30 mm radii, Q=0, layering disabled, default one physics substep, default per-direction point density, default lbfgsb/4 restarts/400 iterations. Ground truth is plant/configuration and evaluation only, never anchor-fitting input. Movement effort measured in points, steps and simulated seconds; no wall-time speedup claim across unmatched trajectories.

Preflight first found the launcher's fresh HP4 layered world; it was not used for HP3 experiments. Reconfigured the attached supervisor to stock HP3, layering=false, physical radii [30]*4, Q=0, then requested full expected-configuration preflight. Separate agent-command socket probe passed. These are tool checks, not accuracy results. npm run test:physics rebuilt native/Wasm and passed all 11 stability tests; physics-gate.log retained. New centered trials use isolated CLI-owned worlds and do not inherit the supervisor's configuration.

E13 pilots finished. With max-steps=1 the current loop fitted the initial three sweeps, then collected one adaptive sweep before reaching the cap: 80 points each, not the planned 60. Both manifests say stop_reason=max-steps and normal_completion=false; their printed/applied summaries are diagnostic, not successful automatic calibration. The default cold bootstrap failed in both (2356/2610 mm); recovery gave 15.17 mm baseline and 3062 mm centered. Measured quiet-sample sigma is nonzero (~0.0074 degrees), so this is fresh production encoder/noise evidence.

E14 full fresh jobs both reached spread-consensus automatic acceptance. Baseline: 7 sweeps/140 points, 1,050,815 steps, 4.588 mm applied RMSE. Center: 9/180, 1,131,349 steps, 4.277 mm. Thus the earlier blanket “all shifts worsen accuracy” inference does not survive fresh collection, but no fewer-data benefit is shown. These are two deterministic single-world conditions, not a reliability distribution.

E15 cold-fit audit of the fresh pilot datasets: at three sweeps TRF recovers 15.168/13.503 mm baseline/center versus default cold sentinel failures; at four sweeps both solvers give 3.579/4.228 mm. This isolates bootstrap fragility from usable later data; it does not justify reinstating a global TRF/fallback default given prior regressions.

Budget decision: use the remaining optional pair allowance for one new asymmetric reference rather than repeating deterministic identical jobs. E16 physical reference p=(500,200,265) mm, same centroid height, same stock plant translated, same forces/points/noise/solver/stopping; events recording, max 5 million steps and 600 s wall. Its canonicalized truth stays inside all original quadrant bounds. Evaluate yaw consistently. No production alias or solver change is needed.

E16 offset=(500,200,265) mm failed before measuring a sweep: production encoder stability timed out after 30 simulated seconds (15,004 steps). Keep finalized events RRD, stdout, config and manifest. Firmware initial M669 exactly matches the translated plant; no radius/layering/substep mismatch found. No anchor-fit accuracy conclusion is possible. One smaller asymmetric retry E17 at (250,100,265) mm is justified to distinguish a reference-workflow issue from the larger displacement; sampled recording this time to retain physical state on failure. Same 5-million-step/600-second full-auto cap, same quiet check, no physics or collector relaxation.

## Completed fresh-reference phase and final decision

E17 smaller asymmetric reference completed normal automatic patience stopping after 13 growth iterations: 15 sweeps/300 points, 2,071,672 native physics steps, 12.602 mm canonical-frame applied anchor RMSE. The larger E16 reference remains a preserved initial-settling failure, not a failed fitted calibration. E14 original and centroid references both completed spread-consensus stopping, with 4.588/4.277 mm applied error and 140/180 points. These single-condition outcomes do not establish failure probabilities.

The sampled RRD audit (local rerun-docs skill plus installed RrdReader source, no conversion or re-logging) found maximum rotation near measurement timestamps of 0.0132° baseline pilot, 0.0288° center pilot, and 2.1086° in E17. Median angles are 0.00411°/0.00431°/0.01048° and lookup gaps <=0.1 simulated seconds. Transient maxima are 0.378/1.085/2.337°. The asymmetry experiment therefore challenges the no-rotation fitting assumption at some points; its poor result cannot isolate numerical conditioning. Actual measured sigma is nonzero, no noise samples or settling checks removed. Baseline/center pilot and full-job first-three-sweep numeric hashes match exactly despite sampled versus events recording.

Fresh manifests, contracts, scenes, firmware inputs, source snapshot and numerical results are linked from fresh-manifest.json, fresh-results.json, sampled-rotation.json and fresh-bootstrap-repeatability.json. Temporary RRF configs removed on all exits. Stock scene/firmware and production physics/solver/collector/stopping source were unchanged. The earlier source/ and manifest.json remain intact; source-fresh/ records later helper/documentation versions. npm run test:physics passed all 11 checks; no failing stability evidence was dismissed.

Final interpretation: my earlier blanket rejection of moving the physical reference was too strong. The retrospective file transform changes exact-reference assumptions and has asymmetric seed/bounds effects; it is not a pure coordinate-invariant test. Fresh centering slightly improves final accuracy in this paired condition but increases points by 40 and physics work by 80,534 steps, so it has not delivered the user's fewer/faster-data objective. No true-anchor metadata leakage was found by direct code inspection or deletion/falsification controls. Explicit origin-dependent geometry prerequisites and a bad bootstrap/history chain explain some numerical fragility, with E12 isolating that effect. An appropriate next research direction is an origin-aware, truth-free bootstrap and statistically consistent reference/measurement model before accepting new stopping or point-count policies. These future changes remain untested, not promised gains.

Status: completed through E17. No active collection/autocal child jobs remain. Preserve one-click UX and automatic stopping: no production algorithm change accepted. The report abstract and catalog now include fresh positive/negative results and the corrected causal interpretation.
