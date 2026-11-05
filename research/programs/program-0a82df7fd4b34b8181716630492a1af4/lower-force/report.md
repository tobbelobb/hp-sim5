# HP3 lower sensor force and encoder reply budgets

Status: complete. Six frozen primary attempts compare 4mid with stock 5mid sensor measurement force under declared motor drag and Gaussian encoder noise. 6 primary attempts are recorded. Keep the production 5mid measurement-force default. The preserved nominal 4mid workflow failure violates the no-new-failure gate. No later pass can erase it. Development applied maximum error was 3.6959 to 2.8529 mm at 1.0641 times simulated effort. Evaluation applied maximum error was 3.4116 to 2.7518 mm at 1.0709 times simulated effort. Evaluation remains separate from development. The separately tested WebSocket transport patch honors explicit encoder budgets while preserving the effective 5-second command default and shorter deadlines. One invalid path-rebasing startup is preserved and counted in the hard seven-collection allocation. No full-autocal run with the transport patch was authorized after that startup consumed the optional slot. These hypothetical simulated worlds and software transport checks do not establish hardware reliability.

Topics: HP3, autocalibration, sensor force, motor friction, encoder noise, WebSocket, reply timeout.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Fixed 4mid improves accuracy across the frozen worlds without new failure or excess effort | n-default, n-candidate, d-default, d-candidate, e-default, e-candidate | Rejected for production; nominal candidate workflow failure remains | [Measurements](measurements.json) |
| Explicit encoder reply budgets can be honored while preserving bounded failures and routing | Fresh-process real WebSocket contrast and regression checks | Supported within software transport scope | [Transport evidence](evidence/transport-summary.json) |

## Decision handoff

- Scope: hypothetical HP3 motors and noisy encoders, known 30 mm radii, zero guide friction/buildup, layering off. Real WebSocket tests use a declared firmware HTTP stub.
- Review: producer hash/readback and numerical recomputation; manager owns independent fixed review and integration.
- Decision: retain 5mid. Deliver only the supported encoder reply-budget patch after software checks pass.
- Next: fixed review of the exact source candidate and evidence. No further primary or confirmation collections are authorized in this attempt.

## References

- [Research notebook](research.md) records the frozen protocol and manager decisions.
- [Frozen policy](policy-frozen.json) and [budget ledger](budget-ledger.json) preserve source/native identities and hard caps.
- The original independently reviewed four-attempt partial remains at Git commit 57658240.


## Frozen method

The primary production implementation and native addon bytes match base a68d6adffec3aea86a55a17d6b4d65a27badd42a. The continuation begins at repository revision 131acef1f7d1ddee22b657d920231a32d60e6c6f, with no production source differences in the primary path. All six primary attempts retain the original 5000 ms encoder resolver cap. The isolated sensor-4mid-v1 changes only min(forceMax,5*forceMid) to min(forceMax,4*forceMid). Explicit sensor overrides, 20mid pullouts, 2mid preloads, physical guards, settling, L-BFGS-B, starts, planning, selection and automatic stopping stay fixed. Truth is used only to construct and evaluate the plant, never to choose measurements or answers.

All worlds use HP3, known 30 mm radii, zero buildup, layering off, guide friction zero, native Rust, Node 23.11.1, speedup 25 and recording none with full journals and passive snapshots. No manual engine.prepare was used. Finite per-job caps are 4 million steps, 8000 simulated seconds and six automatic iterations. The native/ECS motor activation probe from the prior session is reused only after source/native equality was checked; its 3003 steps stay in the aggregate. Supervisor preflight and a separate command-context socket probe passed.

| World | Coulomb factors A/B/C/D | Stiction factors A/B/C/D | Read sigma deg | Seed |
| --- | --- | --- | ---: | ---: |
| Nominal | 1/1/1/1 | 1/1/1/1 | .03 | 4001 |
| Development | .5/2/1.5/1 | 2/.5/1/1.5 | .03 | 4101 |
| Evaluation | .5/1.5/2/.75 | 1.5/2/.5/1 | .05 | 4201 |

Effective motor defaults are .001 Nm Coulomb and .0015 Nm smoothed stiction. The corrected evaluation runs verify per-name native values .0005/.0015/.002/.00075 and .00225/.003/.00075/.0015 Nm respectively after an ordinary first update. All spool radii are .03 m, guide native coefficients are zero and layering is false. The startup correction rebases every absolute import into this session/worktree and asserts four selected motors before stepping.

## Applied geometry

Errors are Euclidean distances in millimetres. RMSE is sqrt(mean of the four squared Euclidean distances). Command-rounded M669 geometry and firmware query-rounded geometry are both reported. Agreement allows 0.005 mm per coordinate because the firmware query prints two decimals. Every anchor below 5 mm is good; any anchor above 20 mm or an incomplete workflow is a practical failure.

| Run | Geometry | A mm | B mm | C mm | D mm | Max mm | RMSE mm | Every <5 | Any >20 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| n-default | M669 command | 2.7594 | 4.5472 | 1.1648 | 1.7423 | 4.5472 | 2.8585 | True | False |
| n-default | Firmware readback | 2.7594 | 4.5472 | 1.1648 | 1.7423 | 4.5472 | 2.8585 | True | False |
| n-candidate | M669 command | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| n-candidate | Firmware readback | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| d-default | M669 command | 2.4993 | 3.6959 | 3.3660 | 2.8206 | 3.6959 | 3.1302 | True | False |
| d-default | Firmware readback | 2.4993 | 3.6959 | 3.3660 | 2.8206 | 3.6959 | 3.1302 | True | False |
| d-candidate | M669 command | 2.5599 | 2.8529 | 2.2228 | 2.4605 | 2.8529 | 2.5341 | True | False |
| d-candidate | Firmware readback | 2.5599 | 2.8529 | 2.2228 | 2.4605 | 2.8529 | 2.5341 | True | False |
| e-default | M669 command | 3.1262 | 1.1409 | 3.4116 | 3.0716 | 3.4116 | 2.8350 | True | False |
| e-default | Firmware readback | 3.1262 | 1.1409 | 3.4116 | 3.0716 | 3.4116 | 2.8350 | True | False |
| e-candidate | M669 command | 1.5485 | 2.4098 | 2.7518 | 1.7723 | 2.7518 | 2.1748 | True | False |
| e-candidate | Firmware readback | 1.5485 | 2.4098 | 2.7518 | 1.7723 | 2.7518 | 2.1748 | True | False |


## Completion and effort

Persisted pairs count dataset entries. Completed physical sub-sweeps count final point indices across all journals, including failures and retries; started sub-sweeps count collector headings. Steps and simulated seconds determine effort. Wall time includes fitting and host scheduling under a two-CPU allocation.

| Run | Outcome and stop | Points | Pairs | Completed/started sub-sweeps | Fits/iterations | Steps | Sim s | Wall s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n-default | accepted_below20; spread-consensus | 140 | 7 | 14/14 | 9/5 | 2156688 | 4313.3760 | 739.4405 |
| n-candidate | workflow_failure; None | 50 | 0 | 5/6 | 0/0 | 1347144 | 2694.2880 | 406.4474 |
| d-default | accepted_below20; spread-consensus | 140 | 7 | 14/14 | 9/5 | 2161548 | 4323.0960 | 643.4076 |
| d-candidate | accepted_below20; spread-consensus | 140 | 7 | 14/14 | 9/5 | 2300140 | 4600.2800 | 913.0880 |
| e-default | accepted_below20; spread-consensus | 140 | 7 | 14/14 | 9/5 | 2226664 | 4453.3280 | 707.4829 |
| e-candidate | accepted_below20; spread-consensus | 140 | 7 | 14/14 | 11/5 | 2384425 | 4768.8500 | 627.2917 |


Pre-result adoption requires evaluation max error improvement of at least 10 percent and 0.5 mm, development max improvement, no new failure, and no more than 1.20 times paired completed-baseline simulated effort.


nominal: max improvement n/a mm, n/a percent; effort ratio n/a; both normal automatic completion False; readback checks False.


development: max improvement 0.8430 mm, 22.8089 percent; effort ratio 1.0641; both normal automatic completion True; readback checks True.


evaluation: max improvement 0.6598 mm, 19.3395 percent; effort ratio 1.0709; both normal automatic completion True; readback checks True.


Keep the production 5mid measurement-force default. The preserved nominal 4mid workflow failure violates the no-new-failure gate. No later pass can erase it. Evaluation meaningful-margin gate passes. A baseline with no final placement above 20 mm cannot establish a reduced large-error failure rate.

## Intermediate fits

These fits are diagnostic evidence, separate from final applied geometry. Production-valid and solver-success flags do not establish truth accuracy.

| Run | Max-error min/median/p90/tail mm | Fits with any >20 | Invalid flags | Underconstrained penalty |
| --- | --- | ---: | ---: | ---: |
| n-default | 3.2830/5.4355/3508.8813/3508.8813 | 2 | 0 | 2 |
| n-candidate | n/a | 0 | 0 | 0 |
| d-default | 3.6132/6.1612/1911.6000/1911.6000 | 1 | 0 | 1 |
| d-candidate | 2.6076/2.8588/2200.8463/2200.8463 | 1 | 1 | 0 |
| e-default | 3.2649/3.4500/5264.4607/5264.4607 | 2 | 0 | 2 |
| e-candidate | 2.6782/2.8270/4483.8195/4534.5786 | 2 | 0 | 1 |


Every fit and per-iteration selection/validity/tail metric remains in measurements.json and the raw archives. The original nominal candidate failed after 50 points on M569.3 request 10812 at step 1,347,144, without an applied answer. Its valid finite same-step late reply does not prove the original host-delay cause. That failure remains in the primary matrix.

## Separate transport result

The real WebSocket M569.3 contrast used separate fresh Node processes, a local firmware HTTP stub returning ok, and explicit 120000 ms encoder and command budgets. Both arms received the exact same finite A/B/C/D response, delayed 6000 ms after its values were captured. Stock source 300791fa failed after 5009.345 ms with a 5000 ms timeout. Patched source bbc06bf8 returned the unchanged payload after 6018.729 ms. No physics ran. This controlled delay does not establish the historical nominal timeout's host cause.

The patch honors the configured encoder reply budget, bounded by any shorter command deadline. The command default remains effectively 5000 ms, and the direct helper retains its separate 2000 ms default. Tests cover delayed numeric replies, the five-second default, shorter command and encoder budgets, missing replies, disconnected clients and concurrent request IDs. A new query with no connected client fails immediately. An in-flight disconnect retains the existing behavior of failing at its configured pending-request deadline.

All 45 JavaScript checks and two noncollection headless checks passed on exact code-only commit 903b00d47c579bda68425e6000ed16a7341b2813, directly based on a68d6adf. The final candidate 95a0af8f63f4d0d257629ea7eaaa295c2638f276 changes only README wording after those checks. Its production and regression-test blobs are byte-identical to the checked commit. The first after-contrast ran on c8aa142a, whose production bridge blob is also identical. The receipts record the actual commits rather than attributing earlier tests to a later commit.

The first pending-disconnect test assumed immediate rejection and failed. Its corrected assertion preserves the existing configured-deadline behavior. Two headless setup errors, a missing artifact parent directory and missing native build metadata in the verification checkout, are preserved with their raw logs. No native rebuild occurred. Final headless checks used 30 physics steps and no calibration fits or points. The focused development and checks completed within 262.5 seconds of the 300-second budget. No full-autocal-with-fix confirmation ran because the invalid startup consumed the optional collection slot. No force, motor, physics or physical guard change is included.

## Invalid setup and budgets

The invalid-path-startup imported the prior session intervention module after a rebasing-order error. The ECS component identities did not match and motor selection was empty. It was interrupted before any fit or point, at 71,980 steps and 143.96 simulated seconds. It is excluded from physical-policy inference and counted as collection 5. The archive retains the original service/runner snapshots, actual imported old module, invocation hashes and invalid-setup record. Its harness-source copy is the later corrected setup, explicitly distinguished from the actual snapshots. The manager removed the optional full-autocal transport confirmation. No eighth collection or full-autocal-with-fix success is claimed.

Aggregate use is 7 collections, 47 fresh fits, 12651622 physics steps including the prior 3003-step probe and bounded headless verification steps, and 750 physical points. The exact ledger records the hard caps and invalid setup.

## Evidence and limits

Each durable per-run archive has all command/noise journals, datasets and partial points, fits and selection, source/native/input hashes, candidate diff and passive snapshots. SHA256 readback verifies every member. Each completed or failed collection was committed and pushed before the next. Prior four raw archives are immutable, and the independently reviewed partial report is preserved at Git commit 57658240. A compact review bundle omits only duplicate events.jsonl files and cites full raw archive hashes for every omission.

Three hypothetical worlds with one seed each do not establish hardware reliability. Motor friction factors are not hardware-characterized. No claims cover guide friction, changed layouts, stiffness, payload, unknown radii or buildup. Seeded random streams share prefixes, but adaptive workloads consume later values at different times. Controlled software reply delays do not establish the cause of the historical timeout.

References: [frozen policy](policy-frozen.json), [all measurements](measurements.json), [pair decisions](pairs.json), [budget ledger](budget-ledger.json), [research notebook](research.md), [continuation handoff](CONTINUE.md), and the per-run raw-artifact-manifest.json files. The native activation probe and call-site audit are included in evidence. research/autocal-recording-performance.md supports the recording choice only.
