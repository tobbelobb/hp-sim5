# Independent review of local coupling and reeving-aware force profiles

Reviewed frozen candidate `8338be8f585f4552bca4af74155bd87132daa9a9`, relative to `ecf052eb`, with `6ade9822` as the initial workspace-planning context. The revised per-point protocol rejects all three previously reported reserve failures in deterministic encoder/command mocks. One P2 force-model finding remains: the recorded strain target can disagree with the commanded force because tables clamp small positive forces and collection accepts points beyond the table's support. A smaller early-validation gap permits a mixed invalid legacy dataset to reach the simulator startup boundary. No P1 or additional P2 defect was established in the coupling protocol.

This review started no services or physics, touched no running processes, and changed no production source or shared Git state. All calculations are synthetic, using the nominal geometry as a review fixture. The parent's stateful HP3/HP4 trials, concurrent corrections, and their physical accuracy are outside this report.

## Force-model finding

**P2 — Retain the claimed strain target in the actual force commands and measurement domain.** At candidate `autocal/equal_strain.py:95`, `np.maximum(intended_sensor_force, minimum_force_n)` changes the unique equilibrium force chosen to satisfy the common-strain condition. Positive finite tension alone does not justify substituting another force while retaining the original `strand_tension_n` metadata.

The [analytical helper](evidence/reeving-review.py) uses the repository's nominal tetrahedron, W = 0.081 kg, N = `[2,2,2,3]`, U = `[2,2,2,4]`, Y = `[528.3,528.3,528.3,398.9]` mm, and tuned middle force 0.23 N. The new default strand target is 0.2875 N. For fixed A = +390 mm and D = −1170 mm, 50 positive-tension predicted poses remain. Six rows in each direction's table are clamped. At one such pose, the intended sensor force is 0.0232228 N but the file commands 0.23 N. Re-solving static balance at that commanded force changes the achieved weighted target to **0.497076 N**, about 73% above the recorded target. The projected flex error is **0.0358085 mm** at EA = 20000 N. [Full inputs and outputs](evidence/reeving-review.json) include both directions and additional fixed targets.

That number is a projected first-order length inconsistency, not a measured calibration error or anchor-error prediction. The calculation assumes one static equilibrium branch, common EA, ideal force control, and the firmware's flex relation. Real endpoint dynamics might stop collection before any clamped row is visited. That is the strongest alternative explanation for limited practical impact; it does not restore the table's stated force invariant.

A related collection path requires enforcement even after removing the clamp. Candidate `sweep_data_collection.mjs:709` holds the nearest endpoint force beyond a table, and `performForceSweep` has no measurement-support check. In `outsideProfileSupport` of the [command-mock output](evidence/coupling-review.json), a force table supported only on drive `[−200,−100]` mm produces a successful five-point sweep at `[−500,−400,−300,−200,−100]` mm. The first three points use the held endpoint force. The first point has fresh local coupling evidence; later points have sufficient rewind. Neither condition establishes that endpoint holding obeys the equal-strain relation outside the modeled domain.

The correction should preserve intended force values, omit unsupported force/pose regions, and require every retained raw and mean measurement to lie within the appropriate direction's supported table interval. Endpoint holding can still serve transit. The parent accepted the clamp finding and described that correction plan; this frozen review does not certify the later implementation.

## Reeving algebra and continuation

The new algebra matches RRF's `HangprinterKinematics::FlexDistances` at source lines 494–506, with `SpringK` at 1442. Write `q_i = d_i + Y_i/U_i` and let `n` span the null space of the line-direction matrix. The firmware's flex magnitude is

`F_i * q_i / (EA * N_i * U_i)`.

Choosing the pretension so that `sum(n_i*q_i*F_i/(N_i*U_i)) = tau*sum(n_i*q_i)` makes the flex projection equal a common scale plus a constant per-line reference. Thus effective N·U and carriage-equivalent Y/U are the appropriate factors. RRF's force-to-torque conversion at lines 1303–1304 already interprets commanded force as carriage force and divides by U, so the static force-balance matrix should not receive another U factor.

At the same positive-tension test pose and common target 1 N, the corrected rule balances weight to below 1e−15 N and has projected strain residual below 2e−13 N·mm. The old U = 1 rule, evaluated under HP4's actual U, instead achieves weighted target 0.436407 N. This is an algebraic comparison with the same target, not a production old-versus-new accuracy trial. The default target scaling by `min(N*U)` also changes the chosen numerical target and must be distinguished from that fixed-target comparison.

The derivation requires a single pretension degree of freedom and common line compliance. Friction, unequal EA, routing errors, force tracking, slack, pulley losses, branch changes, and biased fitted anchors remain outside this invariant. Positive tensions alone do not certify physical reachability.

Generated tables carry `reeving-aware-v1` and their strand target, and collection copies them into sweep metadata. The [offline check](evidence/reeving-review.json) confirms that changing the recorded tuned middle force from 0.23 to 0.4 N preserves the new files' target at 0.2875 N. Inconsistent recorded targets are refused. The permissive dataset schema retains the new metadata and clock fields. These checks do not make clamped forces faithful to the preserved target.

For an entirely profiled legacy U ≠ 1 dataset, `main` rejects before a mocked `HeadlessSimulation` entrypoint; the stub starts no services. With `--no-collect`, the same dataset reaches the mocked offline dispatch, confirming that the new early policy leaves offline replay available. This checks the policy boundary, not the numerical success of replay.

**Smaller early-validation gap:** `profile_policy_error` returns immediately when `is_equal_strain` is false (`equal_strain.py:157`). A dataset containing one legacy profiled sweep and one constant-force sweep therefore returns no policy error and reaches the mocked simulator startup boundary. Existing `fit_stage.py:88` later rejects the model mixture, so this reproduction does not establish silent model contamination. It does show that the stated early legacy-data refusal does not cover mixed invalid input. Both exact input files and boundary results are retained in [the evidence](evidence/reeving-review.json).

## Coupling and the three previous edge cases

The [encoder helper](evidence/coupling-review.mjs) loads the candidate's collector and probe from Git and guards its other imported implementations against changes. Its assertions pass:

| Comparison | Frozen candidate result |
| --- | --- |
| Raw rewind 180.010°, subsequent mean crosses below 180° | Rejects; publishes zero points. Unchanged-encoder control retains three points. |
| 20 mm observed interval, sensor gain 10, strict initial reserve consumes the interval | Rejects with no remaining measurement interval; publishes zero points. |
| Reversible lower-force coupling but flat sensor response above 2.5 N | Skips the high-force point, retains only three of four attempted points, then rejects the direction for insufficient useful points. |
| Same 20 mm interval, later per-point protocol, gain 10 | Retains five distinct drive positions, each with fresh local coupling evidence. |
| Later frontier with completely flat sensor response | Retains zero points and rejects the direction. |

The first two failures no longer become usable sweeps. The third no longer reuses another point's force/interval approval. Accepted points are exposed through `onPoint` only after both raw and noise-mean validation. A direction that later fails its minimum-count check may already have exposed valid individual diagnostic points; it returns no usable sweep and the collector does not write a completed dataset from that direction.

Planner-owned `completed_sweeps` counts prior nonempty input sweeps; the collector enables probing at count ≥ 3. Bootstrap collection remains strict because it does not pass that count. The count grants permission to test local coupling, not approval by itself. The initial reserve uses `180*H/L` motor degrees, consistent with `computeMmPerDegree`. A 2:1 spindle-gear mock requires 360 motor degrees and checks both raw and mean readings.

Each later target is interior to the observed drive interval. The radius is bounded by both observed endpoints. The protocol holds one intended sensor force through start → away → start → toward → start and checks drive tracking, statistically resolved sensor response, opposite response on reversal, and encoder-state recovery. A flat/unresolved response returns to start and is skipped. Failed return and transport errors propagate. Fixed-anchor checks continue during settling and noise validation; the existing limited slip-recovery wrapper holds and restarts an entire direction, with no probe approval carried into the retry. This is code-path and mocked return evidence, not a physical recovery trial.

The force-dependent-offset mock confirms that retained points reuse the returned probe's settled raw observation and its 16-sample noise acquisition. All ten retained points have a raw timestamp matching an encoder read, no mode change between that raw read and noise completion, and a 640 ms noise window beginning at the raw simulation clock. Each unreserved point has a fresh force-matching proof. The exact timestamp assertions concern the deterministic simulation clock; hardware timestamp latency was not measured.

No encoder-only test identifies absolute spool occupancy, slack length, correct routing, true carriage pose, or actual force. Reversible response is useful local transmission evidence under the tested preload. It cannot establish the equal-strain constitutive model, safe global reach, or calibration accuracy.

## Workspace frontier and limits

The progressive workspace policy has no permanent ~390 mm fixed-target cap. Size tuning supplies a measured step, and completed fixed targets extend its bounds. The [offline frontier check](evidence/reeving-review.json) confirms that an empty failed target at D = −1560 mm does not enlarge the bounds; a nonempty completed target advances the next D bound to −1950 mm. The collector's new minimum-count check prevents the tested flat frontier and zero-arc cases from supplying such completion evidence.

A failed direction currently aborts collection. There is no recorded failed-frontier state that automatically redirects the planner to another candidate on restart. The 0.25 pretension-share ranking heuristic versus 0.05 profile-generation heuristic also remains unchanged. The [prior independent review](../session-2026-11-15-independently-review-progressive-workspace-planning-and-sensor-s-e57f8f1b/report.md) records the associated conservative geometric censorship. These are remaining exploration limits, not permission to weaken the sensor gates. Five distinct encoder points are a minimum collection criterion, not an observability or height-coverage certificate.

Actual wider volume, useful pose/force diversity, and accuracy require the parent's separately retained sampled trajectories and further physical validation. What would change this review's model conclusion is a force generator and collector that preserve the invariant over every retained point; what would establish a practical improvement is matched achieved-coverage and accuracy evidence, with force tracking and compliance assumptions checked.

## Reproduction

From repository root:

```bash
HP_SIM5_REVIEW_DIR=research/output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-review-the-revised-per-point-coupling-protocol-initial-reserve-c-010e9604
node "$HP_SIM5_REVIEW_DIR/evidence/coupling-review.mjs"
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python "$HP_SIM5_REVIEW_DIR/evidence/reeving-review.py"
```

Both helpers' assertions passed. Their JSON outputs retain source hashes, inputs, controls, negative results, and limits. They pin the reviewed changed entrypoints to Git and fail if a live imported dependency has changed. No broad suite or physics battery was repeated. No global corpus index, topic, or agenda was changed because this task authorized writes only inside the assigned review directory.
