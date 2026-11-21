# Rough-bootstrap robustness on imperfect machines

Investigation, 2026-11-21, branch `t3/rough-bootstrap-robustness`. The base is `t3/test-compliance-probe-stiffness` (the probe and free low z, a7a8d179) merged with `t3/imperfect-machine-benchmark` (the harness with HP4/HP5r support and force auto-tune fix 1ec95e5e). The fix is commit `9b2941ee`.

**Base of the measurements.** All 29 sweep runs, and so every failure rate below, were measured on `3aa8cacc` + fix `9b2941ee`. `3aa8cacc` is the merge of the probe and benchmark branches, with free low z at a7a8d179.
- The branch was then merged with `t3/uneven-low-anchor-heights` at e292f73e (merge `cb960d2d`). That merge changes only the jackknife spread in `autocal/anchor_uncertainty.py`, from classical to robust median.
- The rough bootstrap can't see this change, because force tuning, size tuning and `assess_rough_geometry` never call `anchor_uncertainty`. The spread does feed fit selection and patience ranking, though, so I re-ran HP5r s3 and HP5 s6 on the merged base.
- Both are bit-identical to their pre-merge runs: same anchors, error and simulated time ([merged-hp5r-s3](evidence/runs/merged-hp5r-s3/), [merged-hp5-s6](evidence/runs/merged-hp5-s6/)).
- So the bootstrap failure rates hold on the merged base by construction. The other 27 final errors were not re-measured there.

**Evidence type.** Simulation only: the headless native physics with the RRF host firmware, run through `autocal.tools.imperfect_machine`. There is no hardware data. Errors are fit minus scene truth in the print convention frame, in mm.

## Summary

1. **Root cause: force auto-tune, not the rough fit.** On HP5r seed 3, the edge-force ramp logged travel 1.05°, 139.35° and 138.95° at 1.03, 1.44 and 2.02 N.
   - The one-sample rule "travel stopped increasing" fired on the −0.4° gain, so force max became 1.44 N.
   - Size tuning pulls its pair at 0.75 × force max = 1.08 N. That is only 5% above the 1.03 N breakaway, so the effector moved **0.25 mm** (normal: 366–670 mm).
   - All three rough sweeps then ran with fixed deltas of −0.2 mm, so they all passed through home. Max anchor std was 66 km. `next_rough_sweep` found no reachable candidate, because the exploration grid is ±0.2 mm, and the bootstrap stopped with `unreliable-rough-bootstrap`.
   - The failure doesn't depend on EA (true stiffness) or on the probe, because force tuning happens before either matters.
2. **It is a creep plateau that HP5r always has.** On every one of 28 unique HP5r ramps, the line creeps 117–167° at 1.44 N and only +0 to +80° more at 2.02 N, then runs free at 2.83 N (≥ 1034°). Seed 3 is the only machine whose 2.02 N gain fell below the 0.5° threshold. HP3 seed 9 (fixed by 1ec95e5e) was the same single-sample rule firing on pre-breakaway stiction.
3. **The fix** (`autocal/control/behaviors/force_tuning.mjs`, −9/+3 lines) deletes the single-sample plateau stop and the stiction guard that 1ec95e5e added to it. A flat or reversing sample now counts as one decline under the existing rule that needs two declines in a row. On seed 3 the ramp continues to 2.83 N (1212°) and picks edge 3.96 N, the same edge as most HP5r machines. Size-tune travel is then 545 mm, and the run finishes `good-fit` at **1.17 mm** in 4014 s.
4. **No change elsewhere.**
   - **Bit-identical runs.** 16 of 16 fixed runs that have a pre-fix twin are bit-identical: same max error to 5e-5 mm and same simulated time. These are HP3 nominal/s1/s9, HP4 nominal/s0–s2, HP5 s0–s3 and HP5r nominal/s0–s2 ([identity_vs_prior.txt](evidence/results/identity_vs_prior.txt)).
   - **Replay of logged ramps.** I replayed all 105 unique edge ramps on this host (1175 tuning files) through the old and new `findEdgeForce`. The old rule changes the outcome on only 5: the 3 seed-3 ramps and 2 old HP4 backfill ramps ([edge_replay_archive.txt](evidence/results/edge_replay_archive.txt)). In those 2 HP4 ramps the travel really reverses at the workspace limit (2160 → 2055° and 2632 → 2558°). The new rule costs them one more trial at 1.4× force. None of today's HP4 runs (nominal, s0–s2) reaches that branch.
5. **Failure rate on random imperfect machines** (±30 mm low dz, ≤ 50 mm top offset, EA ×0.8–1.2):

   | Machine | Seeds | Before fix | After fix | Max error after (mean ± sd, worst) |
   | --- | --- | --- | --- | --- |
   | HP5r | 0–9 | **1/10** (s3) | **0/10** | 1.39 ± 0.49, worst 2.31 |
   | HP5 | 0–7 | 0/8 | 0/8 | 0.98 ± 0.27, worst 1.36 |
   | HP4 | 0–2 | 0/3 | 0/3 | 0.53 ± 0.04, worst 0.57 |
   | HP3 | 1, 9 | 0/2 | 0/2 | 0.82, 1.12 |
   | All random | 23 | 1/23 (95% CI 0.1–22%) | 0/23 (95% CI 0–15%) | |

   - **How "before" was obtained.** HP5r seed 3 is a real pre-fix run. For every other seed the replay shows the old rule picks the same edge, and a run with the same edge is bit-identical (point 4), so the pre-fix outcome is the fixed run's outcome.
   - **Nominal machines.** All four nominal machines pass after the fix: HP3 1.19, HP4 0.72, HP5 0.66, HP5r 1.10 mm.
   - **Bootstrap margin.** Every fixed run's rough bootstrap was ready after 1–3 extra sweeps (the limit is 9). The final max anchor std was 6.5–22.6 mm (the limit is 25).

6. **HP5r seeds 7 and 9 complete but have the worst errors (2.31 and 2.27 mm).** Both are top I.z, with EA ×0.80 and ×1.14. This is the known configured-stiffness/top-z trade on HP5r sweeps, which the compliance probe addresses (investigation 4: 2.6 mm mean at EA ×0.8/1.2). It is not a bootstrap issue, and the fix doesn't touch these runs.

## Method

- **Reproduction.** `imperfect_machine output/rb/hp5r-s3-base --machine hangprinter_5_pulley_reduced --seed 3` on the merged code fails identically to the earlier runs. Its size-tune travel is 0.249 mm, against 0.246 and 0.254 mm in the two earlier runs ([before-hp5r-s3](evidence/runs/before-hp5r-s3/)).
- **Offline diagnosis.** I re-ran `assess_rough_geometry` and `next_rough_sweep` on the archived rough dataset. The check gives rank 12 of 12 but a max anchor std of 66 km, and `exploration_grid` gives a ±0.2 mm grid. Every one of the 25 unseen candidates predicts zero reachable points. `config.max_travel_mm` = 0.203 mm points to size tuning.
- **Cross-run survey.** I surveyed every `sweeps.json.force-tuning.json` and full-auto log on this host ([size_tune_ratio_archive.txt](evidence/results/size_tune_ratio_archive.txt)).
  - In every run with a sound size tune, the size-tune pull is ≥ 2.06 × the breakaway force.
  - Only two runs fall below that: HP3 s9 before 1ec95e5e at 0.75×, and HP5r s3 at 1.05×. Both gave sub-mm size-tune travel.
- **Replay.** [`edge_replay.mjs`](evidence/scripts/edge_replay.mjs) feeds logged edge samples into the old (3aa8cacc) and new `findEdgeForce`. When the new rule needs a sample that was never logged, the replay reports a difference.
- **Live sweep.** [`queue.sh`](evidence/scripts/queue.sh) runs [`jobs_fixed.txt`](evidence/scripts/jobs_fixed.txt), at most 2 runs at a time. The per-run table is [runs_table.md](evidence/results/runs_table.md), built by [`table.py`](evidence/scripts/table.py). The kept run files (imperfection, result, manifest, force tuning, gzipped dataset, rough dataset and log) are in [evidence/runs](evidence/runs/).
- **Unit tests.** `find_edge_force.test.mjs` now has seed 3's logged ramp (it expects 2.83 N, as on nominal HP5r) and a two-flat-sample plateau. The reversal test needs a fifth trial. Full jest: 453/453 pass. No physics code changed, so `test:physics` does not apply.

## Merge note

Both harness branches changed `perturbed_scene`. I kept the benchmark branch's implementation, which moves every frame-side prim that isn't a member of the effector rigid body. It is general across HP4 and HP5r, and it has the stronger test, for reeved machines. I also kept `--ellipsoid-shape` and the `HP_SIM5_COMPLIANCE_PROBE` passthrough from the probe branch. The harness test passes (5/5). The HP5r and HP4 runs here reproduce the benchmark branch's results exactly, so the merged scene perturbation is the one that branch used.

## Alternative explanations and limitations

- **Two flat samples in a row would still fail.** The new rule would still pick a near-breakaway edge if two consecutive samples after breakaway were flat. That has not been observed in any of the 28 HP5r ramps: 2.83 N always runs free. But nothing guarantees it on hardware.
  - A cheap backstop would be to fail size tuning loudly, or to retry it at force max, when travel is under a few mm. The pre-fix bootstrap did fail loudly, about 1400 s in, but it never recovered.
  - I didn't add this backstop, to keep the change minimal and to avoid changing passing runs.
- **The creep is unexplained.** I haven't explained why HP5r creeps only ~140° at both 1.44 and 2.02 N while not stalled. Candidates are capstan friction on the pulley-reduced wheels and the swing under the position-held top line I. It is a property of the simulated HP5r, and real hardware may be different.
- **Wider sample.** Only 23 random machines were run, so a residual failure rate of up to 15% is not excluded statistically. The mechanistic evidence (105 replayed ramps, the creep margin) is stronger than the count.
- **Sample is uneven across machines.** HP3 had only 2 random seeds and HP4 had 3, so the earlier HP3 benchmark (10 seeds) is the broader HP3 evidence. All of those seeds had sound edge ramps under the old rule.
- **Earlier HP5 A+5 failure.** The old-fit HP5 A+5 mm `unreliable-rough-bootstrap` was a different mechanism: the shared-low-z gauge inside the rough fit, fixed by a7a8d179. No HP5 size tune on this host is below 490 mm.
- **What would change the conclusion.** It would change if an HP5r or other ramp were found with two flat samples after breakaway, or if a bootstrap failed with a normal size tune.

## Follow-up: robustness to larger motor force error (cogging ×2, ×3)

Requested after the force-realisation investigation (branch `t3/autocal-force-realisation` 1e2413f3) found HP5r collection failing 2/2 at 2× cogging and HP3e seed 9 failing collection at 1×. This section ended at the manager's wrap-up. It is **incomplete**: the fix below is committed but **not verified live**.

**Base.** All runs below are on `924094cd`: this branch plus `t3/uneven-low-anchor-heights` e292f73e and `t3/autocal-force-realisation` (for `--cogging-scale`). They were run before the tension-cap commit. Simulation only.

### Diagnosis

1. **HP3e seed 9 at 1× (their run): the edge-rule knife-edge already fixed here.** Its edge ramp logged 1.11° and 1.61° at 0.405 and 0.567 N. The single-sample plateau rule set edge = start = 0.405 N, so size tuning moved 0.32 mm and collection failed with "sweep too short". Their branch had neither 1ec95e5e nor 9b2941ee. In this branch that exact ramp is a unit test and picks 2.18 N. The live re-run (`cog1-hp3e-s9`) was queued but never started before wrap-up.
2. **Cogging inflates the measured breakaway, and the equal-strain plan scales every line tension with it.**
   - Force auto-tune measures the force needed to start moving. With 2×/3× cogging that is 2.02 / 2.83 N on HP5r, against 1.03 N at 1×. This is a true breakaway, not a misreading.
   - The equal-strain strand target is `5 × force_mid / min(N)`, so every planned line tension scales with it.
   - On HP5r s6's sweeps the plan peaks at 20–23 N at 1×, 36–42 N at 2× and 51–60 N at 3×. The peak is always on a held, position-mode line.
   - The slipped sweep in their 2× run (A held at −700 mm) planned 37 N on A, against 20 N at 1×. The simulated spool holds about 33 N (1 N·m holding torque on a 30 mm spool).
3. **Two collection failures follow from that load.**
   - **At 2×:** held-line slip (their branch, 2/2). On this base, HP5r s5 and s6 at 2× had **no slips**. Their harness did not yet move the anchor-side wheels, so the geometry differs.
   - **At 3×:** on this base, HP5r s5 and s6 both fail the rough bootstrap. The position-held drive of the spool-reserve probe misses its target by 0.087 mm, against a 0.079 mm tolerance. I believe this is load deflection from the tripled tensions, but it has not been verified.
4. **Separate from collection: the HP5 good-fit stop gate is never met.**
   - HP5/HP5r accept a fit only with point-fit rms ≤ 0.05 mm (`HP5_GOOD_RMS_MM`). On HP5, stable stop also requires that gate.
   - Force ripple raises the rms floor: HP5r 0.070–0.073 mm at 2×, against 0.027–0.032 at 1×; HP5 0.075 (2×) and 0.089 (3×).
   - The runs keep collecting with stable streaks of 5–13/2 until max-steps or the 55 min wall timeout. This is a stopping-policy issue, not collection. I did not change it; it needs a decision (see below).
5. **Slip recovery is a no-op under equal-strain profiles.** `collectWithSlipRecovery` lowers `forceMid`, `forceMax` and `sensorCollectionForce`. But with a sensor-force profile, both the sensor and extra forces come from the profile, so the retries repeat the same forces (their log: extra force 5.549 N after each "lowering forces"). The equal-strain target must stay constant within a dataset, so the profile can't simply be scaled per sweep. Not changed; noted.

### Cogging runs so far (before the tension cap)

| Run | Cogging | Stop | Max error, mm | Start / edge N | Slips | Last point-fit rms | Outcome |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [fix-hp5r-s5](evidence/runs/fix-hp5r-s5/) / [s6](evidence/runs/fix-hp5r-s6/) | 1× | good-fit | 1.17 / 1.00 | 1.03 / 3.96 | 0 | 0.03 | ok |
| [cog2-hp5r-s5](evidence/runs/cog2-hp5r-s5/) | 2× | wall timeout at 8376 s | — | 2.02 / 5.55 | 0 | 0.073 | **no stop** (stable 13/2, good-fit 0/2) |
| [cog2-hp5r-s6](evidence/runs/cog2-hp5r-s6/) | 2× | max-steps (8614 s) | 1.54 | 2.02 / 5.55 | 0 | 0.070 | **no stop**; result applied |
| [cog3-hp5r-s5](evidence/runs/cog3-hp5r-s5/) | 3× | unreliable-rough-bootstrap | — | 2.83 / 7.77 | 0 | — | **fails**: drive tracking 0.087 > 0.079 mm |
| [cog3-hp5r-s6](evidence/runs/cog3-hp5r-s6/) | 3× | unreliable-rough-bootstrap | — | 2.83 / 7.77 | 0 | — | **fails**: same |
| [fix-hp4-s0](evidence/runs/fix-hp4-s0/) | 1× | stable-estimate | 0.57 | 0.79 / 4.27 | 0 | — | ok |
| [cog2-hp4-s0](evidence/runs/cog2-hp4-s0/) | 2× | stable-estimate | 0.71 | 1.71 / 6.57 | 0 | 0.040 | ok |
| [cog3-hp4-s0](evidence/runs/cog3-hp4-s0/) | 3× | stable-estimate | 1.29 | 2.61 / 7.17 | 0 | 0.056 | ok |
| [fix-hp5-s0](evidence/runs/fix-hp5-s0/) | 1× | good-fit | 0.52 | 0.53 / 2.02 | 0 | — | ok |
| [cog2-hp5-s0](evidence/runs/cog2-hp5-s0/) | 2× | wall timeout | — | 1.03 / 2.83 | 0 | 0.075 | **no stop** (stable 5/2, good-fit 0/2) |
| [cog3-hp5-s0](evidence/runs/cog3-hp5-s0/) | 3× | stopped at wrap-up | — | 1.56 / 3.05 | 0 | 0.089 | unfinished |
| cog2-hp3-s1 | 2× | stopped at wrap-up | — | 0.87 / 2.40 | 0 | — | unfinished |
| cog3-hp3-s1, cog1-hp3e-s9 | 3×, 1× | not started | | | | | |

**Failure rates so far, on base 924094cd (n is tiny: 1–2 seeds):**

| Machine | 1× | 2× | 3× |
| --- | --- | --- | --- |
| HP5r | 0/10 (this report) | 0/2 collection failures, but 2/2 never stop (max-steps or timeout) | **2/2 bootstrap failures** |
| HP5 | 0/8 | 1/1 never stops | unfinished |
| HP4 | 0/3 | 0/1 | 0/1 (1.29 mm) |
| HP3 | 0/2 | unfinished | not run |

There are no "after" rates: the fix has not been run live.

### Fix committed, unverified live: cap planned line tension

`autocal/equal_strain.py`:
- **Pose cap.** Equal-strain profiles drop poses where any planned line tension exceeds 0.75 × M666 X. M666 X is the firmware's own "max planned force" safety limit: 40 N on HP4/HP5r, 170 on HP3/HP5. So the cap doesn't depend on the cogging pattern.
- **Target cap.** When a dataset has no recorded target yet (the bootstrap), the strand target is the largest value, at most the existing `5 × breakaway` rule, that leaves every rough sweep at least half its usable poses. The target then stays constant across the dataset, as the fit requires.

Evidence:
- **Unit tests.** The existing equal-strain, planning and harness tests pass (31). No new test covers the cap yet.
- **1× unchanged (offline).** Rewriting the bootstrap profiles from the recorded rough data of 18 1× runs (HP5r 11, HP4 4, HP5 2, HP3 1) gives byte-identical profiles ([profile_replay_1x.txt](evidence/results/profile_replay_1x.txt), [profile_replay.py](evidence/scripts/profile_replay.py)). At 1× the plan peaks at ≤ 24.8 N on HP5r and ≤ 17.5 N on HP4, below the 30 N cap. HP3/HP5 plan ≤ 6 N against a 127 N cap.
- **At 2× (offline).** On HP5r s6's rough data the cap lowers the target only 2.5% (5.06 → 4.93 N) and drops 19–24 of 52 poses per sweep.

Open: whether the cap is the right lever.
- **Not shown to help on this base.** It does not address the 2× stop-gate problem. Whether it removes the 3× drive-tracking failure is unknown, and so is its effect on the slip seen on their branch.
- **Next runs.** The decisive comparison is `cog3-hp5r-s5/s6` and `cog2-hp5r-s5/s6` re-run on the committed fix.

### Needs a decision

The HP5 stop gate is an absolute rms (0.05 mm) that any repeatable force error can exceed. Options:
- make the gate relative to the run's own early rms;
- let a long stable streak stop HP5 without good-fit;
- add the force-realisation investigation's joint force terms to the fit, so the rms floor returns.

Each one changes stopping on HP5/HP5r, so it is the manager's call.
