# Research record

Objective: # Bounded research assignment

{
  "program_id": "program-0a82df7fd4b34b8181716630492a1af4",
  "assignment": {
    "id": "assignment-75a29344551b4d1e96b3acb643eced4d",
    "program_id": "program-0a82df7fd4b34b8181716630492a1af4",
    "kind": "assignment",
    "title": "Finish ONLY frozen evaluation e-default then e-candidate, preserving four prior attempts. Keep force4mid unadopted while no-new-failure gate fails. After six primary attempts, independently reproduce and fix the explicit encoder reply-budget bug with bounded realWebSocket evidence, preserving effective5s default and shorter deadlines. Use seventh slot only for stock5mid nominal full-autocal confirmation of supported transport fix with predeclared one-time6s delayed reply. Deliver minimal supported production transport patch, all raw checkpoints pushed after each collection, complete combined reports, tests and code-only commit for fixed review/integration.",
    "revision": 1,
    "created_at": "2026-11-05T21:21:34.982335+00:00",
    "updated_at": "2026-11-05T21:21:34.982335+00:00",
    "details": {
      "allocation": {
        "collections": 3,
        "fresh_fits": 53,
        "physical_points": 1530,
        "physics_steps": 20031477,
        "review_minutes": 15,
        "wall_minutes": 80
      },
      "approach_id": "approach-1ae49e51d47342ae9cc9f973962a14f9",
      "execution_state": "ready",
      "knowledge_state": "unassessed",
      "question_id": "question-0e6d4ed320fe44a793f3bc5eb35f9c34",
      "review_state": "unreviewed"
    }
  },
  "constraints": {},
  "attempt_id": "attempt-b46f63c3e41b42338c0e719a9837ce52",
  "source_revision": "131acef1f7d1ddee22b657d920231a32d60e6c6f",
  "session_dir": "/workspace/hp-sim5/output/research/sessions/attempt-b46f63c3e41b42338c0e719a9837ce52",
  "execution": {
    "backend": "codex",
    "physics_backend": "headless-js",
    "viewer": "none",
    "record": false,
    "worktree": true,
    "client_args": [],
    "dont_ask": true,
    "goal_mode": true,
    "shared_runtime_write": true,
    "machine": "hp3"
  },
  "shared_filesystem": "Workers own separate worktrees, worlds and session artifacts. Output, RRF/build, dependencies and the starting native addon are shared. Treat shared paths and datasets as immutable; use only your session directory for new artifacts. Shared builds, dependency changes and combined evaluation require an exclusive shared_runtime_write reservation. Same-UID cooperation is not adversarial isolation.",
  "shared_runtime_write": true
}

Respect the recorded allocation. Preserve unsuccessful and partial evidence. Execution success does not accept a scientific claim.

Workers own separate worktrees, worlds and session artifacts. Output, RRF/build, dependencies and the starting native addon are shared. Treat shared paths and datasets as immutable; use only your session directory for new artifacts. Shared builds, dependency changes and combined evaluation require an exclusive shared_runtime_write reservation. Same-UID cooperation is not adversarial isolation.
Your clarification contact is the research manager, never the human. Do not use human question tools. Resolve repository facts yourself. For a blocking manager decision, atomically write /workspace/hp-sim5/output/research/sessions/attempt-b46f63c3e41b42338c0e719a9837ce52/manager-question.json as {"id":"a unique question id","question":"question, context and recommended answer"}. The manager polls worker_status and atomically writes /workspace/hp-sim5/output/research/sessions/attempt-b46f63c3e41b42338c0e719a9837ce52/manager-answer.json as {"id":"the same question id","answer":"manager decision"}. Accept only an answer matching your current id. Ask one question at a time; keep unique ids so old answers cannot answer new questions. Continue independent work while waiting; poll at intervals of a few seconds, within the existing wall deadline. Never extend budgets or assume silence grants authority. If no answer arrives, retain the question and report blocked/partial evidence. Record answers in research.md.

Cloud worker setup (cloud only): in every shell, run `source .agents/skills/cloud-research-client/scripts/env.sh` before Python, Node, native, recorder or physics commands. Use its pinned Node 23.11.1 toolchain. Run simulation jobs one at a time; do not launch concurrent trials.


Recorded strategy contract:
{"human_inputs": [{"created_at": "2026-11-05T08:51:52.888014+00:00", "details": {"citation_ids": ["steering-a39bbe4dc7e14007b5220c51e586b667"], "intent": "instruction", "links": [], "next_assignment_id": null, "provenance": {"date": "2026-11-05", "source": "current conversation"}, "steering_id": "steering-a39bbe4dc7e14007b5220c51e586b667"}, "id": "human_input-91020536fd3c428ebf53314b4411ff8f", "kind": "human_input", "program_id": "program-0a82df7fd4b34b8181716630492a1af4", "revision": 1, "title": "Launch a Research Program around a question in research/prompts/autocal_parameter_variation_focus.txt. Check earlier research and update the prompt. Choose a remaining promising question. Concrete autocal results are required. Wide budget; use judgment. Grill the human to sharpen the program.", "updated_at": "2026-11-05T08:51:52.888014+00:00"}, {"created_at": "2026-11-05T08:58:17.241490+00:00", "details": {"citation_ids": ["steering-54b6ff121dc946fbb392880752b92881"], "intent": "instruction", "links": [], "next_assignment_id": null, "provenance": {"date": "2026-11-05", "source": "current conversation, explicit interview answers"}, "steering_id": "steering-54b6ff121dc946fbb392880752b92881"}, "id": "human_input-6bf594fbf9a54f51b174c31ef49dcf69", "kind": "human_input", "program_id": "program-0a82df7fd4b34b8181716630492a1af4", "revision": 1, "title": "Interview ended at human request. Priority: fewer failures across setups, better accuracy, lower runtime, then explanation. The claim to challenge is transfer to real machines: mildly fuzzed USDA, sensor noise and unpredictable friction causing unrecoverable bad calibration would threaten the entire project. Choose forces and measurements robust across machines. More than 20 mm Euclidean placement error on ANY anchor is a practical failure; less than 5 mm on EVERY anchor is good. Emphasize friction. A 20% collection-time increase is acceptable if it substantially reduces large errors. Wide budget; proceed autonomously within these priorities. Missing fifth UI question needs no follow-up.", "updated_at": "2026-11-05T08:58:17.241490+00:00"}, {"created_at": "2026-11-05T11:55:22.450290+00:00", "details": {"citation_ids": ["steering-b2ce90c86a5345ca820da9857319185b"], "intent": "instruction", "links": [], "next_assignment_id": null, "provenance": {"date": "2026-11-05", "source": "current conversation"}, "steering_id": "steering-b2ce90c86a5345ca820da9857319185b"}, "id": "human_input-91603d5151214d8ba35e9cf89b8a36b4", "kind": "human_input", "program_id": "program-0a82df7fd4b34b8181716630492a1af4", "revision": 1, "title": "Commit and push all recoverable work. Recover originals and checkpoint later results remotely after each experiment. The human wants actual autocal improvements with concrete before/after evidence, not only rejected prototypes. Priorities, thresholds and +20 percent effort allowance remain. Interview closed; proceed autonomously.", "updated_at": "2026-11-05T11:55:22.450290+00:00"}, {"created_at": "2026-11-05T17:29:29.608262+00:00", "details": {"citation_ids": ["steering-0d92d2780e22433394248c37471caf5b"], "intent": "instruction", "links": [], "next_assignment_id": null, "provenance": {"source": "2026-11-05 current user request"}, "steering_id": "steering-0d92d2780e22433394248c37471caf5b"}, "id": "human_input-446c12d0517c4b69bc85271932036d90", "kind": "human_input", "program_id": "program-0a82df7fd4b34b8181716630492a1af4", "revision": 1, "title": "Finish frozen lower-force experiment per CONTINUE.md on current main. Work autonomously. Commit and push after each completed or failed collection, with raw evidence. User authorizes supported production delivery, fixed-evidence review, integration, documentation and cleanup. No unsupported force adoption.", "updated_at": "2026-11-05T17:29:29.608262+00:00"}], "investigations": [{"created_at": "2026-11-05T21:21:34.982482+00:00", "details": {"assignment_id": "assignment-75a29344551b4d1e96b3acb643eced4d", "citation_ids": ["claim-a2c1c9c97fab423099085fa39b817cfd", "decision-f6e3ee17d877459b915ada44a1b7944c"], "execution": {"machine": "hp3", "recording_mode": "none", "simulated_autocal_speedup": 25}, "explanations": [{"approach_id": "approach-1ae49e51d47342ae9cc9f973962a14f9", "explanation": "Fixed lower force may reduce applied geometry bias in heldoutmotor/noise world, but original nominal failure remains a barrier to default adoption."}, {"approach_id": "approach-4efeaafb10e946d68b589da1124f50ad", "explanation": "Motor drag or noise erases development accuracy gain or effort exceedsallowance."}], "kind": "experiment", "outcomes": {"completed": {"actions": [], "finding": "mixed", "observation": "Both remaining primary attempts completed or failed honestly; original adoptioncriteria assessed; separate realWS/productionchecks support minimal bounded transport fix with fullauto confirmation if justified."}, "force_rejected": {"actions": [], "finding": "rejected", "observation": "Forcecandidate doesnotmeet frozenaccuracy/failure/effortgates. Preservecounterexamples and keep5mid while separately assessing transport fix."}, "inconclusive": {"actions": [], "finding": "inconclusive", "observation": "Namedruntimeproblem preventsremainingvalidpair or supportedproductionfix; retainpreciseblocker."}}, "route_plan_ids": ["route_plan-8fa009f70b534bd5baca577073f587cb", "route_plan-a14c39791a884a788f8ac33a7ec9d67d"]}, "id": "investigation-9c47eb67dea64e8bbf63ce30f7f1d4e3", "kind": "investigation", "program_id": "program-0a82df7fd4b34b8181716630492a1af4", "revision": 1, "title": "Finish ONLY frozen evaluation e-default then e-candidate, preserving four prior attempts. Keep force4mid unadopted while no-new-failure gate fails. After six primary attempts, independently reproduce and fix the explicit encoder reply-budget bug with bounded realWebSocket evidence, preserving effective5s default and shorter deadlines. Use seventh slot only for stock5mid nominal full-autocal confirmation of supported transport fix with predeclared one-time6s delayed reply. Deliver minimal supported production transport patch, all raw checkpoints pushed after each collection, complete combined reports, tests and code-only commit for fixed review/integration.", "updated_at": "2026-11-05T21:21:34.982482+00:00"}], "route_plan": {"created_at": "2026-11-05T21:21:34.346036+00:00", "details": {"approach_id": "approach-1ae49e51d47342ae9cc9f973962a14f9", "citation_ids": ["decision-570ae430f6c4401cba5c83ccaa261f21", "admission-22c5d2de62e841e9bd8b77c2b3ae4811"], "decision_value": "Adopt a small supported collector improvement with fresh paired geometry, or reject it explicitly and retain all counterexamples.", "dependency_ids": [], "estimated_cost": {"collections": 3, "fresh_fits": 53, "physical_points": 1530, "physics_steps": 20031477, "review_minutes": 15, "wall_minutes": 80}, "hypothesis": "Lower measurement force reduces equilibrium stretch/model bias across varied drag/noise while preserving completion and automatic stopping.", "novelty": "First frozen lower-sensor-force production candidate paired across motor Coulomb/stiction and noisy encoders. Distinct from failed guide preload experiment.", "objections": [], "observation": "Paired lower-force and default fresh completed calibrations improve applied per-anchor errors without new failure and with <=20 percent extra collection effort.", "priority": 1, "prompt": {"id": "autocal-parameter-variation", "path": "research/prompts/autocal_parameter_variation_focus.txt", "sha256": "1361ac15505cd6f58f45e61c93f5bdeaf3fb3836e4c3bdcb8e9fb4300749e47f", "title": "HP3 autocalibration across physical variation"}, "reason": "Continue only the two frozen, unrun evaluation jobs after observed host delays made the pair unsafe in first100min deadline. Fourattempts remain immutable; no repeat of successful jobs. A focused explicit-encoder-budget fix addresses actuallostcollection; bounded realWS contrast and optional seventh stock5mid confirmation have concrete delivery value. No force-default adoption while original no-new-failure gate fails.", "reopening_condition": "A different encoder-observable force rule, materially different measured drag, or independently corrected model.", "retry_key": "081d59eb5d1f8aa300b282528851baf62a4ae73be067a730485d901fe3a5f968", "retry_limit": 2, "scope": {"buildup": 0, "candidate": "sensor measurement force min(forceMax,4*forceMid); all pullout/preload/settling/stopping/solver rules unchanged", "effort_limit": "1.20 times paired completed baseline simulated time/steps", "failure": "any anchor >20 mm or no completed calibration", "good": "every anchor <5 mm", "guides_mu": 0, "layering": false, "machine": "hp3", "radii": "known 30 mm", "worlds": "nominal noise .03 seed4001; dev motor factors [.5,2,1.5,1]/[2,.5,1,1.5] sigma .03 seed4101; eval [.5,1.5,2,.75]/[1.5,2,.5,1] sigma .05 seed4201"}, "tradeoffs": ["At most twenty percent extra completed-pair simulated collection effort.", "No hardware reliability claim from three hypothetical simulated worlds."], "uncertainty": "The force-valley result used another solver/source revision and only one world; transfer is unproven. Motor factors are hypothetical, not hardware-characterized."}, "id": "route_plan-8fa009f70b534bd5baca577073f587cb", "kind": "route_plan", "program_id": "program-0a82df7fd4b34b8181716630492a1af4", "revision": 1, "title": "Continue only the two frozen, unrun evaluation jobs after observed host delays made the pair unsafe in first100min deadline. Fourattempts remain immutable; no repeat of successful jobs. A focused explicit-encoder-budget fix addresses actuallostcollection; bounded realWS contrast and optional seventh stock5mid confirmation have concrete delivery value. No force-default adoption while original no-new-failure gate fails.", "updated_at": "2026-11-05T21:21:34.346036+00:00"}}

Finish this program, not another proposal. The human interview is finished. Complete only the unrun work in this fresh bounded attempt. Read current AGENTS.md/research/AGENTS.md and COMPLETE research/programs/program-0a82df7fd4b34b8181716630492a1af4/CONTINUE.md plus research/programs/program-0a82df7fd4b34b8181716630492a1af4/lower-force/CONTINUE.md. Parent has already read all handoff/archives and separately reviewed first4attempts. Admission admission-410f1328caf0460aa2142d5339f7ccc2 supports exact partial report, report_only, with dev max3.6959->2.8529mm at1.064117effort but nominalcandidate failed50points on5sM569.3transport. Keep it a practicalfailure; no force-default adoption. Evaluation NOT TESTED. Do NOT repeat n-default,n-candidate,d-default,d-candidate or rejectedpreload work. Prior first4raw/checkpoints immutable and available in tracked lower-force directory and output/research/sessions/attempt-7b09fff3cc5b4bb89c33586f94cc4ebe. This is no worldrestore.

Budget:80worker minutes INCLUDINGlast10reports,15review reserved by manager,190scleanup. Original aggregate limits remain7collections,80freshfits,28Msteps,2000points. Alreadyused4collections,27fits,7,968,523steps incl3003probe,470points. YOUR REMAININGMAX3collections,53fits,20,031,477steps,1530points. Eachcollection4Msteps/8000simseconds,6autoiterations. Sequentialjobs on2CPU. Use .venv/bin/python and source cloud env.sh each Python/Node/physics shell, pinnedNode23.11.1,nativeRust,speedup25,recordingnone, full command/noise journals and passive snapshots. Supervisor research_preflight ANDseparate command-contextresearch_socket_probe before longcollection. Verify actual motorECS/native fields, layeringoff,30mmradii,zerobuildup/guidefriction after ordinaryfirstupdate, never manualprepare. Reuse old3003step intervention evidence only when source/native identical; count any newprobe steps.

REQUIRED PRIMARYJOBS: fresh e-default THEN fresh e-candidate. Frozen evaluation factors A/B/C/D Coulomb .5/1.5/2/.75, stiction1.5/2/.5/1, readGaussian.05deg,seed4201,guide0,known30mmradii,buildup0,layeringoff,otherwise stockHP3. Fixedcandidate measurementmin(forceMax,4*forceMid) versus5, explicitoverrides/pullout20/preload2/guards/settling/solver/starts/planning/stoppingunchanged. Preserve ORIGINAL transportsource/timeouts for both primaryruns, including5sresolvercap. Rebase saved currenta68d6adf harness to YOURsession/worktree before runs; all ECSimports must share samecurrentworktreeclassidentity. Never oldprepare.py. Freeze newpath/source/native hashes before anyevalrun. Do not select forces, thresholds, starts, measurements or answer from truth. Pre-result meaningfulmargin evaluation max>=10%AND>=0.5mm gain, devmaximproved,no newfailure, completed-pair efforts<=1.20x. Originalnominalfailure still blocks4middefault even ifevalaccuracyimproves; keep5midproduction. Do not use laterconfirmation to erasefailure.

DURABILITY: After EACH completed/failedcollection, save readable result/spec/hash/candidatediff and compact hash-verified rawarchive with ALLjournals/dataset/partialpoints/fitselection/snapshots/inputs, commit and PUSH YOURproducerbranch BEFORE next longjob. Exclude secrets/controlfiles, never stage apparentoutputdeletions. Send manager pushedhash and conciseoutcome. Update shared tracked lower-force report/abstract/notebook/metrics/budget/README to contain all6primaryresults; preserve prior4rawarchives and originalpartial review via Gitcommit57658240. Copy oldsmall evidence only as needed; complete first4rawarchives alreadydurable. Newcombinedcompactreviewbundle may omit duplicatehugeevents.jsonl with explicitfullarchivehashreferences, but nojournal omitted from durableperrunraw. Report actualrounded M669ANDfirmwarereadback A/B/C/Derror,max,RMSE=sqrt(mean4squaredEuclideandistances),every<5/any>20,normalstop/cap/abort,physicalpoints,persistedsweeppairs/completedandstartedphysicalsubsweeps,fits/intermediatetails/flags,steps,simtime,wallwithhostcaveat. No reduced>20failurerateclaim ifbaselinesnone.

SEPARATE SUPPORTED PRODUCTIONCHANGE after all6primaryattempts: fix actualexplicitencoderreplybudget bug in integrations/rrf/rrfSimulatorBridge.mjs. Finitecommandtimeout is hardcapped5000ms even when callerconfig encoderTimeoutMs120000. Originaltimeoutcauseunknown; validlateencoderresponse10812atunchangedstep saved. Do NOT changephysics,motor,forcemeasurementdefault,physicalsettlingguards or originalprimarysource. Withinabout5minfocuseddevelopment/testbudget, independently reproduce realWebSocket M569.3 stock5s failure for valid6s delayed reply vs respectingexplicitlongbudget success withexactsamepayload; preserve effectivecurrent default5s and shortercommanddeadlines, disconnected/missingreplyfailure and correctrequestID routing. Bound allwaits and cleanservices. Minimalcodepatch+meaningfulregressiontests. No broadinfrastructurework. Existingtests/js/rrf/rrfSimulatorBridge.test.js hasWSfixtures. Capture before/after commands/sourcehashes/timings/rawreply evidence. If morethan5min needed, askmanager with specificgap while retaining reportingreserve.

SEVENTH slot is authorized ONLY as justified stock5mid nominal full-autocal confirmation of supportedtransportfix. If fixing/tests succeed, freeze confirmationprotocol BEFORErun: nominal frozenmotors,noise.03deg seed4001,30mmradii,zero build/layer/guide; original5mid force; candidatebridgefix; one-time6s delay of firstnonempty encoderresponse aftertotalSteps>=1,300,000, based on observedfailurestage and noanchortruth. Instrument transport only AFTERcapturing encoderresponse values, do not advancephysics or changeforces duringdelay. This tests realworkflowwithfixedexplicit120sbudget and retainedphysicalguards. Distinguish controlledsoftwaredelay from originalunprovedhostcause. It is not a replay of failednominal4mid and cannot eraseit. No otherextra collection without managerjustification. Preserve all raw and PUSH before furtherlongwork. Report before/after WSfailure/success AND actualfullautocal anchor/effort outcome separately. If this cannot fit ornamedgapblocks, tellmanager beforestarting ratherthanoverrun.

PRODUCTION DELIVERY: keep force5mid untouched, ship only supportednarrowtransportfix. All relevantcollector/headless/autocalchecks, no unnecessarytestbroadening. Commit/pushsourceanddocs onproducerbranch, provide exact base/sourcecandidatecommits and changedfilehashes. Prefer separate code-onlycandidate commit directlydescending explicitbasea68d6adf to avoid archive binarydiff/falsephysicsgate; parentcan constructit fromsame sourceblobs if needed. Never fake exactcandidatecheckreceipts if testsrananothercommit; retain actualsourceidentity. Manager owns separatefixedreview/integration; do notmerge main. Update missionprompt/portfolio to retire actuallysettledquestions and catalog; no pending/proposalending if authorizedworkremains. Parent handles programcontroller history/admission exports and finalintegration.

Before finalresponse provide complete report/abstract/evidencebundle/manifest/checklogs/codecommit and actualusage. Write YOURreal finalresponse to final_output. No humanquestions; contactmanager for blockers. Exact selectedmissiontext follows:

Improve encoder-only HP3 autocalibration across physically different machines.
Updated 2026-11-05 after inspecting prior reports, the human interview and the completed guide-friction investigation.

The goal is accurate, reliable anchors with less collection effort, preserving
one-click operation and automatic stopping. Use known spool radii, zero buildup
and disabled simulator line layering. Use HP3, not HP4. Real calibration has no
ground-truth anchors. Truth may construct and evaluate a simulated plant, but
must not choose measurements, fit seeds, thresholds or the applied answer.

The historical reference is output/extended-autocal/no-buildup-reference-003.
Also use a fresh current-code baseline for any live comparison. Physics, motor,
collector and fitting changes since that reference can change its results.

Human priorities and success criteria

Prioritize fewer failures across physical setups, then accuracy, then runtime,
then explanation. More than 20 mm Euclidean placement error on ANY anchor is a
practical calibration failure. Good performance requires less than 5 mm on EVERY
anchor. Report all per-anchor errors and their maximum; RMSE alone is insufficient.
Emphasize unpredictable friction, modest scene perturbations and simulated sensor
noise. Up to 20% more collection time is worthwhile if it substantially reduces
large errors. A small simulation matrix cannot establish hardware reliability.

What earlier runs have already established

- Collection profiling is complete enough to guide a new experiment. Settling,
  pull-outs, force tuning and travel all matter. Native Rust, headless collection
  and cheaper recording modes are available. Do not start another general
  profiling or engine-rewrite project without identifying a new bottleneck.
  See research/autocal-recording-performance.md and
  output/research/sessions/2026-10-15-native-optimization/abstract.md.
- Earlier warm-start work improved simulated HP3 fits without new measurements.
  Blind point thinning, more restarts and simply collecting at half density did
  not reliably preserve accuracy. Offline thinning and fresh coarse collection
  are different interventions. See
  output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md.
- A distinct-fit consensus-spread stopping rule has been investigated and
  implemented. Agreement can still reflect two biased fits. A new stopping rule
  needs evidence beyond remeasuring convergence on the same nominal world. See
  output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/report.md.
- It is no longer true that every experiment used one physical setup. A paired
  ten-world study varied mover mass 0.80-1.25x and gravity 8.5-11.0 m/s^2, keeping
  anchor layout, friction and stiffness fixed. Mean applied anchor RMSE was
  2.967 mm for L-BFGS-B and 2.874 mm for TRF. Neither solver had a candidate
  above 20 mm. These mild load changes do not establish robustness to large
  failures or other machine properties. See section 10 of
  output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md.
- TRF is available, but a global solver switch and per-restart fallback were
  rejected after legacy-data regressions. Better intermediate fits can create a
  tight biased consensus and worsen the final answer. Keep final applied error,
  per-iteration errors and extreme failures separate. The same report's sections
  9-11 document the distinction. Disagreement-driven fresh collection remains
  proposed, not demonstrated.
- Force sensitivity is measured on the nominal world and is not monotonic.
  A lower sensor force helped some runs but cost time; no robust force policy
  across different line properties or layouts was established. See sections
  4.4 and 4.7 of that report. Do not claim that weaker forces always help.
- Moving the encoder reference toward the anchor centroid has also been tested.
  Fresh centered collection slightly improved final RMSE, 4.588 to 4.277 mm,
  while increasing points from 140 to 180. An asymmetric reference was worse.
  Retrospective rebasing changes reference/noise assumptions and is not an
  equivalent substitute for fresh collection. See
  output/research/sessions/2026-10-20-interactive-research-ae0174ce/report.md.
- Generic ranking and finite-prediction-only consensus membership have mixed
  results. Validation must use compatible whole sweeps unseen by every member
  of the actual applied anchor-plus-spool model. See research/autocal-agenda.md
  and output/research/sessions/2026-10-23-consensus-membership/report.md.

- The 2026-11-05 guide-friction study tested a bounded preload change in an
  isolated harness. It preserved nominal accuracy but did not recover collection.
  Three completed calibrations had all anchors below 5 mm; four active-guide
  trials failed before fitting. A matched zero-guide/noise-only control passed.
  A constructed probe found load torque with macroscopic local cable slack.
  The independent reviewer kept the scoped report-only result. No production
  change was adopted, and hardware robustness remains unresolved.
  The original workspace files were recovered after an execution-attachment
  failure. The full original evidence and raw journals are preserved on the
  research/autocal-originals-2026-11-05-0a82df7f and
  research/autocal-recovery-2026-11-05-0a82df7f branches.
  Do not repeat the same preload/guide matrix as an untested improvement.

- The fresh2026-11-05 lower-force continuation at a68d6adf has a controlled four-attempt partial. In development,4mid improved final applied maximum error3.6959→2.8529mm at1.0641times simulated effort; both completed arms had every anchor below5mm. A nominal4mid attempt failed before fitting with a5000ms M569.3 transport timeout and remains a practical workflow failure. No force default was adopted. Evaluation calibration is NOT TESTED; only its two frozen jobs remain, followed by a separately authorized transport-budget discriminator. Do not repeat the completed primary jobs or erase the failure with a later pass. See research/programs/program-0a82df7fd4b34b8181716630492a1af4/lower-force/report.md and CONTINUE.md.

These are inspected historical reports, not fresh replications at current HEAD.
Check that linked raw inputs and harnesses exist before budgeting a replay.
Several older reports refer to external worktrees or absent artifacts.

Remaining promising questions

1. Which force and measurement choices avoid large errors across machines?
   Start with existing motor Coulomb/stiction variation and noisy encoder reads,
   keeping guide friction zero until its transport/load model is validated.
   Extend to stiffness, layouts and payload when they resolve a specific gap.
   First verify each intervention changes the intended physical property in the
   baked native world. Distinguish unsupported parameters from negative results.
   Screen a small declared design, then reserve different worlds for evaluation.
   Compare current automatic force tuning with a bounded encoder-based policy.
   Do not let simulated truth set per-world forces or select the winning rule.

2. Can force-varied measurements resolve solver disagreement?
   Freeze L-BFGS-B and TRF candidates from the same encoder prefix. Use their
   predicted measurements and observable noise to choose a feasible extra sweep,
   potentially at a different sensor force. Compare both frozen candidates on
   exactly the same fresh whole sweep before refitting. Compare with the existing
   planner and a matched extra-sweep control. Measure whether the decision also
   improves applied geometry, rather than assuming prediction cost proves it.
   If solvers agree but are both biased, can a force change expose model error?

3. Which missing measurement would constrain the weak geometry directions?
   Known radii supply a metric length-change reference; joint unknown radii and
   geometry introduce additional scale ambiguity. Distinguish gauge freedoms,
   weak numerical conditioning and systematic model bias. Use local Jacobian
   checks, controlled synthetic cases and fresh held-out sweeps to find which
   feasible encoder-only experiment adds independent information. Do not repeat
   the failed retrospective origin shift or add unidentifiable nuisance terms.

4. Can smaller calibration experiments isolate a useful model error first?
   For example, test bidirectional hysteresis or force-dependent apparent length
   at repeated encoder configurations before fitting geometry. Treat commanded
   forces as commands, not calibrated tension measurements. Test whether a
   correction transfers to unseen sweeps/worlds and improves actual anchors.
   Pointwise fixed-encoder variation was ignored in an older objective audit,
   but was negligible on a later nominal reference. Establish its size in the
   target regime before implementing a larger objective model.

Choose one bounded discriminator first. Record the main explanation, a credible
rival, the cheapest falsifying observation and what result changes the next
decision. Expand only after a useful result. A wider budget is permission to
resolve uncertainty, not a reason to repeat settled studies.

Execution and reporting

- Read research/AGENTS.md, research/README.md and
  research/autocal-recording-performance.md. Use the local .venv/bin/python.
- Start with saved/synthetic evidence, then collect when the question requires
  fresh data. Use native Rust, supported --speedup 25, events for routine live
  work and none for fit-only matrices. Keep detailed recordings for motion
  questions. Run the supervisor research_preflight and a separate command-context
  scripts/research_socket_probe.py before long live collection.
- Declare finite worker/review time, fits, collections, points, physical steps,
  simulated time and cleanup budgets. Preserve unsuccessful and partial trials.
- Verify plant settings, encoder reference, coordinate frame, dataset lineage,
  source and native artifact hashes. Stock configure_session currently rejects
  custom anchor layouts; use an isolated documented research harness if needed.
- Report anchor RMSE with its precise definition, final applied error,
  per-iteration distribution/tail errors, invalid fits, automatic acceptance,
  points, sweeps, simulated seconds, steps and wall time. Separate selection-only,
  fit-only, synthetic, fresh simulated and hardware evidence. Do not pool repeated
  datasets as independent physical worlds.
- Freeze measurement/selection rules without test-world truth. State development
  versus evaluation worlds and every exception. No large-error reliability claim
  follows from a matrix containing no large errors.
- Implement only a supported change and run relevant autocal checks. Physics,
  motor or dynamics changes require npm run test:physics. Commit and push research checkpoints under the human's explicit authorization;
  adopt production changes only when supported. Preserve one-click operation and automatic stopping.
- Maintain research.md, abstract.md and report.md with experiment IDs, measured
  outcomes, references, limitations and next decision. Regenerate the catalog
  with .venv/bin/python scripts/research_index.py. Research Programs use the
  controller's fixed submission, separate review and result-saving workflow.

Durability: preserve and push small datasets, results, source snapshots and failed
trial records after each completed experiment. End-of-turn saving alone is not
sufficient protection against workspace interruption. Keep large journals in a
separate durable artifact store and record any omission explicitly.


Current hypothesis: pending.
Constraints and budgets: set before experiments.
Experiment IDs: none yet.
Accepted steering: none yet.
Next decision: establish a measurable baseline.


## Frozen continuation setup, 2026-11-05 21:33 UTC

Only e-default then e-candidate use original primary transport. Prior four runs and their report-only admission remain immutable. All upstream source/native hashes matched a68d6adf and the existing 3003-step torque probe. Harness changes only current worktree/session import paths. Remaining caps are 3 collections, 53 fits, 20,031,477 steps and 1530 points. 80-minute worker deadline starts 21:30 UTC; final ten minutes reserved for reporting. Separate transport fix is authorized after six attempts.

## Invalid path startup

A rebasing-order error retained the old session path in the intervention import. Motor component identity mismatch produced empty motor selection. Stopped with SIGINT before fits, preserved separately. This is not evaluation evidence. Corrected session segment and added a four-selected-motors startup assertion. Manager question invalid-path-startup-slot-20261105T2134 requests slot accounting.

## invalid-path-startup

invalid_setup_startup; stop=None; normal acceptance=False.

Applied per-anchor error mm: None. Maximum None, sqrt(mean of four squared anchor distances) None. Every<5: None; anchors>20: None.

0 physical points, 0 sweeps, 0 fits, 71980 steps, 143.96 simulated seconds, 19.782724542001233 wall seconds. Wall time depends on host load. Intermediate max tail None mm, invalid fits 0.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

Manager answer invalid-path-startup-slot-20261105T2134: Manager counts invalid startup as collection 5 and all numerical costs. Aggregate 7 remains hard. Remaining two slots are corrected e-default/e-candidate. No fullautocal-with-fix confirmation is permitted. RealWS and relevant noncollection checks may support transport fix.

## e-default

accepted_below20; stop=spread-consensus; normal acceptance=True.

Applied per-anchor error mm: {'A': 3.1261797773000164, 'B': 1.1408847843991956, 'C': 3.4115644622382355, 'D': 3.0715793982900084}. Maximum 3.4115644622382355, sqrt(mean of four squared anchor distances) 2.834959883811394. Every<5: True; anchors>20: 0.

140 physical points, 7 sweeps, 9 fits, 2226664 steps, 4453.328 simulated seconds, 707.4829356749997 wall seconds. Wall time depends on host load. Intermediate max tail 5264.460651398488 mm, invalid fits 0.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

## e-default checkpoint

Normal spread-consensus completion, 140 points, 7 persisted pairs, 14 completed physical sub-sweeps, 9 fits across 5 iterations, 2,226,664 steps and 4453.328 simulated seconds. Applied and firmware readback agree: A3.12618 B1.14088 C3.41156 D3.07158 mm, max3.41156, RMSE2.83496. All below5. Intermediate max5264.46065 mm retained. Full raw archive 74 members, 154069472 bytes uncompressed, 8690145 gzip; every member hash read back. Committed and pushed 0c0de5a9aa629fc7075a4e7615271f08625ee33b before e-candidate startup. Production transport and force files still equal primary base.

## e-candidate startup

Final allowed collection uses frozen sensor-4mid-v1 and original transport. All per-name selected ECS/native friction values verified, 30mm spool radii, zero guide native coefficients, no layering, sigma .05 seed4201. Import audit preserved; no truth-based changes.

## Controller mailbox decision

The manager answered invalid-path-startup-slot-20261105T2134 through worker_answer. Exact answer follows.

Count invalid startup as collection5 and all71980steps/143.96simseconds/0points/0fits. Preserve and push raw as invalidsetup, not valid e-default or friction-negative evidence. Remaining TWOcollectionslots are required corrected fresh e-default then e-candidate. Originalaggregate7 remains; REMOVEoptionalstock5mid fullauto transportconfirmation, nohidden8th or probe/integration relabeling. Supportedtransportfix can use realWSbefore/after and relevant noncollection headless/bridge checks; finalreport states no fullautocal-with-fix confirmation. Assert all51projectimports/scene paths and exact4motor ECS/native per-name values before longcollection, freeze correctedharnesshashes withoutpolicyretuning. Keep originalfailednominal incounts and force5middefault. Do not write manager controlanswers yourself; record collaborationreceipt innotebook.

Transport route approach-6c857194c0624dd9bcc532cfec2d2fe3, route_plan-3569e65fced2449cb8857eb29bb78ff0, withdraws full-autocal confirmation and permits real WebSocket contrast plus noncollection checks only.

## e-candidate

accepted_below20; stop=spread-consensus; normal acceptance=True.

Applied per-anchor error mm: {'A': 1.5484508387418094, 'B': 2.409835579837204, 'C': 2.7517842429905643, 'D': 1.7723148704448473}. Maximum 2.7517842429905643, sqrt(mean of four squared anchor distances) 2.1747657369140136. Every<5: True; anchors>20: 0.

140 physical points, 7 sweeps, 11 fits, 2384425 steps, 4768.85 simulated seconds, 627.2916642359996 wall seconds. Wall time depends on host load. Intermediate max tail 4534.578640158273 mm, invalid fits 0.

Full raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.

## Transport discriminator frozen after e-candidate raw push

Fresh Node process each arm, real WebSocket, local firmware HTTP stub returns ok, same M569.3 command/120000ms configured and command budgets, exact same finite payload, 6000ms controlled response delay after values captured, zero physics. Stock should time out at5000ms. Fixed should return payload at6000ms. No original host-cause claim; no full-autocal confirmation.
Started 2026-11-05T21:59:36.561432+00:00. Focused development/test budget300s. All six primary attempts retain original transport.

## Supported transport result

Real WebSocket M569.3 contrast used separate fresh Node processes, a local firmware HTTP stub returning ok, an explicit120000ms encoder budget and command timeout, and the exact same finite A/B/C/D response delayed6000ms after capture. Stock source300791fa failed after5009.345ms with the5000ms timeout; patched sourcebbc06bf8 returned the unchanged numeric payload after6018.729ms. No physics ran in this contrast. The historical nominal timeout cause remains unknown.

The patch replaces the fixed5000ms cap with the configured encoder budget, bounded by any shorter command deadline. An explicit5000ms command default preserves the old effective default; the direct encoder helper retains2000ms.45 relevant JavaScript checks and2 noncollection headless checks passed on exact code-only candidate903b00d47c579bda68425e6000ed16a7341b2813, whose direct parent is a68d6adf. They cover delayed numeric replies, default5s, shorter command/encoder budgets, missing replies, disconnected clients and concurrent request IDs. In-flight disconnects retain the existing behavior of failing at the configured pending-request deadline; no immediate-close rejection is claimed. The headless checks used30physics steps and no calibration fits or points. No full-autocal-with-fix confirmation was run because the invalid startup consumed the optional collection slot.

The initial wrong pending-disconnect test and two headless artifact setup failures are preserved in the evidence. The first after-contrast ran on c8aa142a; its bridge source blob is identical to final903b00d4. All final regression checks ran on903b00d4. Only test expectations changed between these candidates. The focused phase completed within262.5s, within the300s budget. No force, motor, physics or physical guard change is included.

Manager corrected the README description of disconnect behavior. Final candidate 95a0af8f63f4d0d257629ea7eaaa295c2638f276 changes documentation only after passing checks at903b00d4. No production logic changed and no redundant test rerun is claimed.

## Completed evaluation and final decision

The final e-candidate raw checkpoint was pushed at4d144da50fab7e6a866fde71524dc39c47cc9538 before any transport work. It stopped normally after140points,7pairs,14completed/14started physical sub-sweeps,11fits across5iterations,2384425steps and4768.85simulated seconds. Applied/readback maximum2.751784mm and RMSE2.174766mm. All anchors below5mm. Evaluation gain0.659780mm or19.339521percent, effort1.070851times. B worsened1.140885 to2.409836mm. Intermediate max4534.578640mm and the underconstrained-penalty fit remain preserved. No invalid fit flags were erased.

The evaluation margin passes. The original nominal failure still violates the no-new-failure gate, so the fixed4mid force adoption is rejected and stock5mid remains. All six primary attempts used original transport. The separate supported change honors explicit encoder reply budgets, with no physical policy changes. Final code-only candidate95a0af8f has direct basea68d6adf. Tests actually ran903b00d4;95a0af8f changes only the disconnect wording in README, with byte-identical code/test blobs. No exact-commit test result was fabricated.

Aggregate numerical use is7collections including the invalid startup,47freshfits,12651622steps including3003prior probe and30noncollection verification steps,750points. This worker used3collections,20freshfits,4683099steps and280points. Collection slots are exhausted. No full-autocal transport confirmation or eighth collection ran. Remaining numerical caps do not authorize another collection.

The combined compact view was independently recomputed from all raw archives, with every included member hash verified and exact full archive references for omitted duplicate event journals. All prior four raw archives remain byte-identical to commit57658240. Catalog was regenerated in shared output and a searchable entry snapshot is tracked. Temporary verification services are retired; the original idle supervisor remains launcher-owned. Manager owns fixed review and integration.
