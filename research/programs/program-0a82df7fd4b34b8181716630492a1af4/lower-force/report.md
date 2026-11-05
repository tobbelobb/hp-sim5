# HP3 lower sensor measurement force under motor drag and encoder noise

Status: in progress.

We compare the fixed4×midpoint sensor measurement force with the current5×midpoint rule in six fresh paired HP3 autocalibrations. The nominal and development worlds precede a heldout evaluation pair. All worlds use known30mm radii, zero buildup, no layering or guide friction, native Rust, and seeded Gaussian encoder-read noise. Motor Coulomb and smoothed-stiction factors vary by axis. 4 of six primary collections have completed or failed. No adoption decision yet. Complete the frozen matrix without retuning. A small hypothetical three-world simulation cannot establish hardware reliability or a lower failure rate. Applied anchor errors come from the rounded M669 command and a checked firmware readback. Every intermediate fit, failure and full command/noise journal is preserved separately from final applied geometry.

Topics: HP3, autocalibration, measurement force, motor friction, encoder noise, applied anchors.

## Frozen decision and method

Base implementation is a68d6adffec3aea86a55a17d6b4d65a27badd42a. The candidate sensor-4mid-v1 changes only the isolated helper default from min(forceMax,5*forceMid) to min(forceMax,4*forceMid). Explicit overrides, pullouts at explicit20*midpoint, direct2*midpoint preloads, force tuning, guards, settling, L-BFGS-B fitting, seed selection and automatic stopping remain unchanged. All three copied collector files come from current base. Current service source retains upstream nonfatal recording behavior. Imports were rebased to the current worktree to preserve ECS component identity. No old prepare.py or manual engine.prepare was used.

Before primary results, the manager agreed to operationalize meaningful improvement as evaluation applied maximum error reduced by at least10% and0.5mm, with development maximum also improved. All paired runs must complete with no new failure and each completed pair must use <=1.20×baseline physics steps and simulated time. Nominal regression is disclosed even when other gates pass. Human endpoints remain any anchor>20mm or incomplete calibration failure and every anchor<5mm good. The policy and hashes were frozen before the first primary collection. Truth only builds/evaluates the fixed plant; it does not choose measurements, forces, starts, selection or stopping.

| World | Coulomb A/B/C/D factors | Stiction A/B/C/D factors | Read sigma deg | Seed |
| --- | --- | --- | ---: | ---: |
| nominal | 1/1/1/1 | 1/1/1/1 | .03 | 4001 |
| development | .5/2/1.5/1 | 2/.5/1/1.5 | .03 | 4101 |
| evaluation | .5/1.5/2/.75 | 1.5/2/.5/1 | .05 | 4201 |

Fresh loaded effective defaults were0.001Nm Coulomb and0.0015Nm stiction. The1001step-per-world torque probe checked ECS and packed native fields after the ordinary first step. At +0.01Nm target for1000steps, A encoder response differed from nominal by0.9120rad in development and0.9196rad in evaluation. Guide native mu remained0. This establishes intervention activation, not hardware realism. Supervisor preflight and the separate command-context socket probe passed. The attached idle supervisor used stock layering; each independent primary harness uses config_hp3.g, zero buildup and layering off, checked in its manifest.

## Applied results

Errors are Euclidean anchor distances in millimetres. RMSE is sqrt(mean of the four squared Euclidean distances). The four-decimal presentation does not increase the precision of M669's0.01mm coordinate rounding. Firmware query and applied command agreement is checked to0.005mm per coordinate, and both geometries are preserved. Sweep counts distinguish persisted dataset pairs from completed bidirectional physical sub-sweeps, counted by final point index in all point journals. Started sub-sweeps are separate result fields.

| Run | Outcome | A mm | B mm | C mm | D mm | Max mm | RMSE mm | Every<5 | Anchors>20 | Points | Pair/sub sweeps | Fits | Steps | Sim s | Wall s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n-default | accepted_below20 | 2.7594 | 4.5472 | 1.1648 | 1.7423 | 4.5472 | 2.8585 | True | 0 | 140 | 7/14 | 9 | 2156688 | 4313.3760 | 739.4405 |
| n-candidate | workflow_failure | not comparable | not comparable | not comparable | not comparable | not comparable | not comparable | None | None | 50 | 0/5 | 0 | 1347144 | 2694.2880 | 406.4474 |
| d-default | accepted_below20 | 2.4993 | 3.6959 | 3.3660 | 2.8206 | 3.6959 | 3.1302 | True | 0 | 140 | 7/14 | 9 | 2161548 | 4323.0960 | 643.4076 |
| d-candidate | accepted_below20 | 2.5599 | 2.8529 | 2.2228 | 2.4605 | 2.8529 | 2.5341 | True | 0 | 140 | 7/14 | 9 | 2300140 | 4600.2800 | 913.0880 |

nominal: max improvement not comparablemm, not comparable%; candidate/default effort not comparable. Both normal completion=False; both readback checks=False.
development: max improvement 0.8430mm, 22.8089%; candidate/default effort 1.0641. Both normal completion=True; both readback checks=True.

Wall time includes fitting and host scheduling. Jobs run sequentially under a2CPU allocation. Use steps and simulated time for the effort criterion; do not interpret wall-time differences as policy speed effects. Matched seeds share a random-stream prefix, while adaptive workloads consume later noise values at different physical times.

## Intermediate tails and failures

Final accepted geometry and intermediate fit accuracy are separate endpoints. Production-valid or solver-success flags do not establish accuracy.

- n-default: 9 fits across 5 iterations, tail maximum 3508.8813mm, 2 fits with any anchor>20mm, 0 invalid-flagged fits, 2 underconstrained-penalty fits; stop=spread-consensus, normal acceptance=True.
- n-candidate: 0 fits across 0 iterations, tail maximum not comparablemm, 0 fits with any anchor>20mm, 0 invalid-flagged fits, 0 underconstrained-penalty fits; stop=None, normal acceptance=False.
- d-default: 9 fits across 5 iterations, tail maximum 1911.6000mm, 1 fits with any anchor>20mm, 0 invalid-flagged fits, 1 underconstrained-penalty fits; stop=spread-consensus, normal acceptance=True.
- d-candidate: 9 fits across 5 iterations, tail maximum 2200.8463mm, 1 fits with any anchor>20mm, 1 invalid-flagged fits, 0 underconstrained-penalty fits; stop=spread-consensus, normal acceptance=True.

## Evidence and limits

Each run directory contains a readable result, frozen policy, candidate diff and raw-artifact-manifest.json. The compact gzip raw archive retains dataset, every fit and selection state, full auto log, complete command/noise journals, passive initial/terminal/native snapshots, input/invocation and exact harness. Every file was verified by SHA256 after archive readback. Each completed or failed collection was committed and pushed before the next collection. No raw evidence is omitted. The compact review copy, when present, excludes duplicate events.jsonl only and points to each full raw archive and hash.

Only three hypothetical worlds and one seed per world were tested. No hardware data, guide-friction intervention, layout, stiffness, payload, spool-radius or buildup generalization is claimed. The motor model uses smooth friction and does not represent ideal static breakaway. A baseline with no final>20mm placement cannot support a claim of fewer such failures.

The nominal timeout trace preserves request10812 with valid finite A/B/C/D response followed by the5000ms timeout at the same simulator step1,347,144. The event journal lacks wall timestamps per response, so exact reply latency and original delay cause remain unestablished. Source inspection finds createGcodeBridge caps command encoder resolution at5000ms despite explicit120000ms configuration. A separate real-WebSocket stock-vs-fix check is authorized only after allsix primary jobs; it is NOT TESTED in a controlled four-attempt partial. The timeout remains in every denominator and cannot be erased by confirmation or source inspection.

Reporting errata are preserved in research.md. Completed physical sub-sweeps now come from final point indices rather than only finalized dataset metadata, so the aborted nominal candidate correctly has5completed/6started despite0finalized dataset pairs. An auxiliary process-wall field originally lacked a failure fallback; final summary uses the correct406.4474s manifest wall. Main reported wall and physical metrics were already correct. Original raw archives stay immutable.

References: the current CONTINUE.md and continuation-setup/policy-frozen.json define this continuation. The earlier workers/attempt-939c073f49004eb6949910600200be51/report.md documents the rejected guide-preload experiment. Native and source hashes, plant settings and all exceptions are in each run's policy and raw manifest. research/autocal-recording-performance.md supports recording none for this numerical matrix, not a calibration accuracy claim.
