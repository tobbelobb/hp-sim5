# HP3 friction/noise robustness screen

Updated 2026-11-05T10:26:20.544791+00:00. This is a report-only result.

Nominal defaults and the frozen bounded-preload policy both accepted all four anchors below 5 mm. With verified added guide friction and modest encoder noise, the development default failed before its first point; both development policy versions failed after 20 points. A held-out default failed after 40 points, before fitting. A native diagnostic shows load torque transmitted to a spool with macroscopic local cable slack. The findings expose a boundary of the current guide-friction model and collection workflow; they do not establish a force policy that improves calibration robustness or a hardware failure rate. The posthoc matched zero-guide/noise-only control ended as `accepted_below20` after 140 points and 4278.630 simulated seconds. Its applied maximum anchor error was 4.3495 mm.

## Design and separation of evidence

The interview was complete before this assignment. The prioritized endpoints were fewer workflow failures and large anchor errors, accuracy, effort, then explanation. Any anchor above 20 mm is a failure; every anchor below 5 mm is good. Error is Euclidean displacement of each anchor from its authored nominal value. Anchor RMSE is sqrt(sum of four squared anchor errors / 4), not coordinate-wise RMSE. Truth appears only in plant construction and the separate saved-evidence evaluator, never force tuning, fitting, selection, or stopping.

| World | Split | Added guide mu A/B/C/D | Encoder sigma ° | Seed |
| --- | --- | --- | --- | --- |
| dev0 | development nominal | 0/0/0/0 | 0 | 100 |
| dev1 | development | .003/.008/.006/.004 | .02 | 101 |
| eval0 | declared held-out | .008/.0035/.0045/.006 | .03 | 202 |
| eval1 | declared held-out; unrun | .004/.0065/.008/.003 | .05 | 303 |
| eval0-noise-only | posthoc model diagnostic | 0/0/0/0 | .03 | 202 |

These are hypothetical small disturbances, not measured hardware distributions. At 30 mm radius, the noise standard deviations correspond to about .0105-.0262 mm. The guide coefficients give only roughly 1-2% capstan tension-ratio bounds. No stiffness, layout, motor-friction, broad friction-range, layering, radius, or buildup generalization was tested.

The original development/evaluation split was retained. Incorrect future clock labels in the original design/notebook were preserved with original hashes and mtime and corrected before any held-out scoring; the correction metadata is embedded in [review-inputs.json](evidence/review-inputs.json). Exact v2 code was frozen at 2026-11-05T09:44:12Z in [policy-frozen.json](policy-frozen.json), before eval0 started. The later zero-guide control was declared as a posthoc model/noise discriminator before its own run; it is not a held-out policy-success result. The full planned policy matrix was stopped when physical snapshots exposed a model-validity concern.

## Plant and intervention verification

All reported numerical jobs use HP3, native Rust, known radii [30,30,30,30] mm, zero buildup, no layering, no Viewer. The historical backend label `headless-js` is accompanied by `physics_engine=rust-native` in each job manifest. Native artifact SHA256 is `66a8451bc1a42310e4dfae166288737e595f75bc1173c8f88e92163afb94a5d8`; source revision is `5bb8eb955fa680ddc631b6fb10f988ec3efb497a`. Node is pinned to v23.11.1 by the environment script. Standalone probes launched without that environment crashed in native buffer detach even on nominal geometry; they are execution failures, not friction sensitivity.

The authored `.005` pinhole material is bound to terminal attachments, whereas capstan equilibration uses the nine interior AnchorAL/AR/BL/BR/CL/CR/D0/D1/D2 contacts. Stock effective guide mu is zero. The USD loader reads staticFriction, and the anchor entity builder discards friction even if material bindings are changed. A session-local pre-preparation ECS hook injects CoefficientOfFrictionComponent only on named interior guides; production physics and scene files remain unchanged. After the ordinary first native step, `intervention-native.json` records exact guide identities, coefficients, motor defaults, and artifact identity per run. Binding-only controls retain mu0 and are excluded from active-friction contrasts. Zero-mu injection and unmodified nominal have identical complete first-step native state hash `a70e31cd5329f19b53ae53c3166db07cd3e9a9c045811dffa4b1a017457fa3a2`.

Noise is added independently to every encoder response before collector averaging. Journals preserve clean and noisy values, commands, actual simulator step/time, and seed. Clean values are used only in posthoc diagnostics. Matched seed means identical random-stream prefix; adaptive workloads may consume subsequent samples at different physical times. Empirical sigma is reported in measurements.json. The failed active dev1 default measured .02020° and eval0 default .03002°.

The model is capstan rest-length redistribution, not bearing axle loss. Motor Coulomb/stiction/windage settings are unchanged and distinct. Its smoothed motor friction vanishes exactly at zero angular speed, so it is not an ideal static breakaway model. [Source map](evidence/model-source-map.json) and native readback delimit these claims.

## Frozen policy and collection conditions

The policy commands `min(forceMax,5*forceMid)` instead of `2*forceMid` for the free sensor when encoder-observed pullout transitions to held-drive settling and at ordinary point preloading. Both force values come from unchanged production noisy-encoder autotuning. Measurement already used that bounded5mid law. Stability windows, tolerances, timeouts, fixed-anchor guards, force tuner, fitting, selection and stopping are unchanged. Missing-force fallback is retained; commanded preload never exceeds the autotuned maximum. Commands are force equivalents, not measured cable tension.

V1 changed only the pullout preload and failed after20 points when the next path still dropped to2mid. V2 applied the same law consistently to both paths and also failed after20 points at an unchanged600-s simulated settling guard. This negative development evidence is preserved. No further tuning followed the freeze.

| World/arm | forceMid N | forceMax N | default preload N | v2 preload N |
| --- | --- | --- | --- | --- |
| nominal, both arms | .24793256 | 2.61354401 | .49586512 | 1.23966281 |
| dev1, all arms | .20661047 | 3.04913467 | .41322094 | 1.03305234 |
| eval0 default | .22727151 | 3.35404814 | .45454303 | untested |
| eval0 noise-only default | .22727151 | 3.35404814 | .45454303 | untested |

The core runs allow normal automatic stopping up to12 iterations, 4 million physics steps/8000 simulated seconds,10 points per sub-sweep, dt .002s and timeScale25. The manager raised the original600-s wall cap to1200s after a healthy nominal run exceeded600s. The final noise-only diagnostic has a1080-s wall cap to preserve the10:29 reporting reserve. A wall cap would be reported as a bounded execution outcome, not an intrinsic calibration failure. Main collection jobs run sequentially on a two-core cgroup. Host loads differed historically; wall times are retained but not compared as policy speed effects.

The earlier supervised recorder-failed baseline had a6-iteration cap. The successful nominal-numeric run has an effective12 cap: its preserved argv contains the runner's initial6 followed by an overriding12, and actual execution logs show iterations1/12 through5/12. All later main trials use the same effective12 override. The duplicate argument is preserved, not rewritten as a different historical invocation.

## Workflow and applied geometry

| Run | Outcome | Physical points | Physics steps | Simulated s | Applied max error mm |
| --- | --- | --- | --- | --- | --- |
| nominal-numeric | normal acceptance | 140 | 2,091,604 | 4183.208 | 3.8407 |
| nominal-consistent-preload | normal acceptance | 140 | 2,076,452 | 4152.904 | 3.8732 |
| dev1-active-default | workflow_failure; no geometry | 0 | 749,017 | 1498.034 | — |
| dev1-bounded-preload | workflow_failure; no geometry | 20 | 1,020,587 | 2041.174 | — |
| dev1-consistent-preload | workflow_failure; no geometry | 20 | 1,283,333 | 2566.666 | — |
| eval0-default | workflow_failure; no geometry | 40 | 1,432,483 | 2864.966 | — |
| eval0-noise-only-default | normal acceptance | 140 | 2,139,315 | 4278.630 | 4.3495 |

| Accepted run | A error mm | B error mm | C error mm | D error mm | Anchor RMSE mm | Anchors >20 mm | Every anchor <5 mm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| nominal-numeric | 3.8407 | 3.5992 | 3.4637 | 1.8605 | 3.2850 | 0 | True |
| nominal-consistent-preload | 3.8732 | 3.7185 | 3.3663 | 1.9804 | 3.3198 | 0 | True |
| eval0-noise-only-default | 3.7091 | 4.3495 | 3.9875 | 1.4781 | 3.5623 | 0 | True |

Applied geometry is parsed from the exact rounded M669 command sent to firmware and its readback. Full-precision fitted candidates are preserved separately. No error is assigned to a pre-fit collection failure.

Across the listed main trials: accepted below20mm=3; accepted with an anchor above20mm=0; capped without normal stopping=0; workflow failure with no geometry=4; workflow failure with interim geometry=0. These are trial counts, not independent-world failure-rate estimates. Earlier infrastructure attempts and the canceled inactive-friction pilot are separated below.

The two nominal runs each stopped normally by spread consensus after5 iterations/9 fits and140 physical points. The completed nominal policy/default effort ratio is4152.904/4183.208 =0.99276 for both simulation time and steps, with equal points. This single healthy nominal pair meets the20% effort allowance; it establishes no recovery benefit in frictional worlds. A successful run is never divided by a failed arm's early timeout to claim a speed gain.

The nominal default's worst intermediate cold fit had2693.9716mm maximum anchor error despite solver success and production-valid marking. Its underconstrained penalty was active with rank8/cost100; a warm candidate and history/consensus prevented application of the bad cold geometry. The nominal v2 intermediate tail was11.4409mm. Preserve every typed fit and history artifact; favorable final RMSE must not hide bad iteration tails. No false automatic acceptance above20mm has been observed in the completed runs, but failed collections yield no geometry test.

The matched noise-only control also produced a rejected2628.2689mm cold-fit tail with all four anchors above20mm, rank0/cost100 and underconstrained_penalty=true while production valid=true. Three successful final runs therefore coexist with two severe intermediate failures. These are candidate failures, not final false acceptances. The control's final maximum4.3495mm and normal completion support guide-model activation, rather than noise alone, as the discriminator in this matched pair. It remains a posthoc model diagnostic, not held-out validation of the preload policy.

Earlier attempts are retained separately: the supervised events-recording baseline advanced992881steps/1985.762sim s and30points, then failed recorder acknowledgement/fetch; a noise-only inactive-friction pilot advanced255131steps and was canceled under early host contention. Two further startup attempts advanced no physics or collection and are excluded from the12-job numerical counter by explicit manager decision, while all wall costs and failures remain. These are infrastructure or bounded pilot outcomes, not active-guide calibration evidence.

## Why the physical-model boundary matters

In the dev1 v2 failure, clean C encoder range was18.13° in the last1s and early/late mean drift48.96° over5s, far above unchanged .3° quiet-range/1.5° vibration tolerances. Noise alone cannot account for that guard rejection. Yet the terminal snapshot shows effector speed4.34e-6m/s, C spool speed .26184rad/s, and about119.4/119.3mm slack in the two spool-to-guide segments with zero local constraint force. The distal segments carry1.37535/.50044N. Eval0 also has large clean motor drift while terminal effector speed is2.87e-6m/s. These are instantaneous post-cleanup observations; stop commands can alter motor state. They do not establish effector motion or a credible hardware failure.

The source unconditionally defers direct spool load to a neighboring pinhole constraint and maps the downstream constraint back to the spool, without checking guide mu or local slack. Native `load()` accumulates that mapped torque in entity slot59. The friction pass runs before constraints and skips rest redistribution under active friction when both positive extensions are zero, while zero mu always redistributes. These facts suggest a frictional-guide transport/load-coupling boundary; they do not prove a complete causal chain.

A paired native probe constructs a state from the v2 terminal geometry, motor commands, local rest lengths and path storage, then advances8s with active guides or zero guide mu. It is not exact replay: hidden knot/history fields retain fresh-scene values. After8s the active arm has53.49/53.35mm local slack and zero local tension, distal forces1.1063/.5839N, native cable-load torque+.050706Nm, spool speed+.2640rad/s and effector speed6.72e-6m/s. The zero-guide arm has only5.03/2.21micron slack and spool speed+.01997rad/s. The active native cable-load power is+.01339W while target motor power is-.00818W at that sample. These signed powers and rest/storage balances are recorded, but they are not a full energy audit: body work, gravity, frictional losses and all integration terms were not balanced. The result directly confirms torque can coexist with macroscopic local slack in this constructed native state. It supports reviewing the specific pinhole-transfer assumption; it is not a production fix or proof that encoder-only calibration is impossible.

## Scope and decision

No force-policy improvement is established. Held-out policy arms, eval1, broader mu stress and extra-data recovery were not run. Extra points cannot repair a settling failure that prevents the first fit; that alternative was not silently counted as tested. Native guide-friction behavior needs a separate reviewed physical-model proposal and required gates before changing production physics. The preserved default nominal quality is real simulation evidence; active-guide failures remain valid workflow observations under this model, with physical-transfer claims explicitly limited.

For hardware context only, Piao et al.2019, Sensors19(11):2520, DOI10.3390/s19112520, describe pulley loading/unloading hysteresis and distinguish bearing/sliding losses. Their method uses additional force sensors for training; it is neither encoder-only evidence nor a source for these mu values. No hardware result was collected here.

An unexecuted next discriminator is documented in [proposed-next-discriminator.md](proposed-next-discriminator.md), with a small reusable motor-field injection/readback module. It keeps guide mu0 and stresses the separate native motor Coulomb/stiction route. The exact parameter slots, effective defaults and zero-speed smoothing limitation are documented; no motor-friction collection was run.

## Reproduction and review

Use the assigned checkout and source `.agents/skills/cloud-research-client/scripts/env.sh` in each ordinary shell. The corrected scene/service generator is `harness/prepare.py`; [generator-reproduction-v2.json](evidence/generator-reproduction-v2.json) verifies generated byte equivalence. The old pilot generator/chronology evidence is retained and must not replace the corrected files. Saved per-run source snapshots and invocation.json identify the exact service and collector used. Policy hashes must match policy-frozen.json before any held-out work.

```sh
source .agents/skills/cloud-research-client/scripts/env.sh
S=/workspace/hp-sim5/output/research/sessions/attempt-939c073f49004eb6949910600200be51
.venv/bin/python "$S/harness/run.py" dev1 NEW_DEFAULT_RUN --max-steps 12
.venv/bin/python "$S/harness/run.py" dev1 NEW_POLICY_RUN --max-steps 12 --bounded-preload
node "$S/harness/torque_slack_probe.mjs" active
node "$S/harness/torque_slack_probe.mjs" zero
.venv/bin/python "$S/harness/analyze.py" NEW_DEFAULT_RUN NEW_POLICY_RUN
```

Fresh reproduction would consume a new approved budget; the submitted evidence can be reviewed without simulation using analyze.py and the preserved datasets, partial point journals, fit/history JSON, loaded-native settings and measurement JSON. [Review manifest](review-manifest.json) hashes the selected small evidence bundle; full command/noise journals remain in the session. [Budget ledger](budget-ledger.json) records all attempts and diagnostics. No production source files were changed, and no merge or push was performed.
