# Ellipsoid autocal (sphere meshes): collector, planners, and the misfit that decides accuracy

Interactive investigation, 2026-11-19/20, branch `autocal-point-fit-spools-sphere-meshes`.
Simulation evidence only: headless native Rust physics with RRF host firmware, one
deterministic run per machine and variant. No hardware data.

This report is written for the next agent. That agent's job is to make Hangprinter
autocal more robust by chasing the systematic misfit described in section 6. Sections
1–5 say what was tried and what is settled. Section 7 says what to do next.

**Summary.**
- **Winner.** Flat circle sweeps with the point fit remain the best collection on the user's score, which is error × time and must hold under disturbances.
- **Ellipsoid shape.** Sphere meshes ("ellipsoid shape") reach 0.3 mm on clean HP3/HP4 at best. That result does not survive small data changes, it costs up to 2× the time, and it loses under disturbances.
- **Error source.** The held anchor's height is exactly degenerate with the scale of the free lines' stretch correction: rms flat, ~8.5 mm per unit scale. That correction is 10–20% off, so a physical model error, not the collection plan, decides accuracy.
- **Two collector bugs fixed:**
  - an open-loop relative-move bug that made saturated motors "slip";
  - a held-length recording bug that double-counted line drift, present in every dataset.

## 1. Question

Can a collection that holds one line at a fixed length, so the effector moves on a sphere around that anchor
("ellipsoid shape", `--ellipsoid-shape`), beat the default flat circle sweeps
(`--ellipse-shape`, point fit)? The verdict uses one score, set by the user:

> Twice the time draws down the score as much as twice the accuracy draws it up.

So a variant's score per machine is **max anchor coordinate error × simulated time**
(lower is better). Variants are compared by the geometric mean over the four benchmark
machines (HP3, HP4, HP5, HP5 pulley-reduced). The winner must also hold up on disturbed
data, not just on clean runs. Two scores are therefore reported:

- **clean score**: the clean error × time;
- **disturbed score**: the geometric mean of the clean error and the mean error under each of four disturbance
  scenarios, × time.

## 2. Setup

- **Truth:** `BENCHMARK_MACHINES` (via `autocal/tests/test_disturbed_robustness.py::TRUTH`).
- **Live runs:**
  `autocal/autocal.py --headless-sim --machine-type M --ellipsoid-shape --dataset output/<variant>-M/sweeps.json`.
  Simulated time comes from `sweeps.headless/manifest.json` (`simulated_s`).
- **Sweep baseline:** the `benchmark_923c2baa_*` reference datasets and their logged
  live times (HP3 5516 s, HP4 4897 s, HP5 3762 s, HP5r 4672 s).
- **Disturbances:** produced offline by
  [`robust_compare.py`](evidence/scripts/robust_compare.py), using `autocal/tools/disturb_dataset.py`'s
  `scenario_events` and `apply_events` with seeds 0–2:
  - `mixed`: two 10 mm line pulls plus 3% outliers;
  - `noise0.05` and `noise0.2`: Gaussian encoder noise in mm;
  - `mixed+noise0.2`.

  Each fitter refits the disturbed copy:
  - sweeps use the sweep autocal's two-stage point fit (`test_disturbed_robustness._fit`);
  - ellipsoid datasets use `ellipsoid_autocal.fit(..., "ellipsoid")`.

  A replay cannot collect more data, so these errors overstate what a live run would show. They
  rank fits and datasets, not live behaviour.
- **Ranking:** [`rank.py`](evidence/scripts/rank.py) computes both scores.

## 3. Variants tried

All ellipsoid variants hold D (HP3/HP4) or the top line I (HP5). Each ellipsoid has one row per free line:
- the row also holds that line (the "level line");
- it is an ordinary force sweep of 5 points per direction;
- it is fitted jointly as one ellipsoid in squared-length space (`ellipse_fit.py`), with home offsets and constant-force stretch correction.

| Variant | Commit | Heights (held line) | Level lines | Stop |
| --- | --- | --- | --- | --- |
| fixed schedule | `140dcbf4` | 8 fixed fractions of measured travel, 0 to −1.5× | fixed fractions of travel from home | 2 consecutive steps ≤ 0.5 mm, min 3 |
| backfill | `39d78fc1` | 3–4 meshes, revisited and backfilled | from home | stable |
| backfill2 | `6411dd0d` | as backfill, held ≤ −1× travel | from home | stable |
| heights | `e4c6200d` | low, ½ highest, highest (10% clearance), then gap splits | idle (natural) | step + row jackknife |
| heights2 | (between) | same | idle, all free lines idle while settling | same |
| heights3 | (between) | same | shortened all the way to the sphere's shortest | same |
| heights4 | `e022c095` | same | shortened half way from idle to shortest | same |
| fixed schedule on closed-loop collector | this report | the fixed schedule, ported onto the `e022c095` collector | from home | as fixed schedule |

Per-visit errors for every live run are in
[`live_runs_per_visit.txt`](evidence/results/live_runs_per_visit.txt). The original
fixed-schedule datasets (2026-11-19) no longer exist; their numbers come from the
README: HP3 0.29 mm / 5177 s, HP4 0.31 / 6491, HP5 0.56 / 8326, HP5r 1.16 / 8347.

## 4. Results

The scores come from refits of the recorded datasets with the held-length recording bug of 5.4 repaired:
- [`ranking_repaired.txt`](evidence/results/ranking_repaired.txt);
- per-scenario errors in [`disturbed_repaired/`](evidence/results/disturbed_repaired/);
- as recorded: [`ranking_as_recorded.txt`](evidence/results/ranking_as_recorded.txt).

"orig" is commit `140dcbf4` rerun in a worktree; it reproduces its README numbers exactly. "sched" is the same schedule on the closed-loop collector.

| Variant | HP3 | HP4 | HP5 | HP5r | Clean score | Disturbed score |
| --- | --- | --- | --- | --- | --- | --- |
| **sweeps, point fit (default)** | 0.88 mm, 5516 s | 0.68, 4897 | 0.91, 3762 | 1.11, 4672 | 4112 | **7943** |
| orig fixed schedule | 0.29, 5177 | 0.31, 6491 | 0.56, 8326 | 1.16, 8347 | **3421** | 8910 |
| heights4 (planned heights) | 0.96, 6160 | 0.86, 5460 | 1.60, 7667 | 1.61, 7763 | 8079 | 11934 |
| sched (fixed schedule, closed-loop collector) | 0.79, 6635 | 0.57, 5909 | 2.37, 6901 | 1.23, 8549 | 7447 | 12892 |
| backfill2 | 1.43, 5024 | 0.60, 8622 | 1.10, 9054 | 1.10, 13731 | 8632 | 17746 |

**Final code**, with both collector fixes and both regressions fixed, live with four simulations at a time ([`live_final.txt`](evidence/results/live_final.txt), [`ranking_final.txt`](evidence/results/ranking_final.txt)). The held-length gap in this data is 0.000 mm:

| Variant | HP3 | HP4 | HP5 | HP5r | Clean score | Disturbed score |
| --- | --- | --- | --- | --- | --- | --- |
| **sweeps, point fit (default)** | 0.97 mm, 4587 s | 0.74, 4924 | 1.04, 3778 | 0.93, 4686 | 4077 | **7623** |
| ellipsoid shape, planned heights, ellipsoid fit | 2.60, 6068 | 0.46, 4622 | 1.27, 6094 | 2.90, 6495 | 8350 | 12105 |

The ellipsoid runs swing per machine again: HP4 improves from 0.86 to 0.46 mm, while HP3 and HP5r get worse by more than 1.5 mm. That swing is the knife-edge of 5.6.

As recorded, before the repair, the disturbed scores were:

| Variant | Disturbed score |
| --- | --- |
| sweeps | 7565 |
| heights4 | 12312 |
| heights2 | 14885 |
| backfill2 | 16480 |
| heights | 21664 |

**Verdict.** Sweeps with the point fit win once disturbances count. They remain the default (`--ellipse-shape`, `--point-fit`).
- **The original fixed schedule** wins on clean data only, and only with its own collector:
  - it is ~2× slower on HP5;
  - it scores worse than sweeps under pulls and outliers on HP3/HP4 and under heavy noise on HP4 and HP5r.
- **Ellipsoid shape:** it stays available as `--ellipsoid-shape` (`--ellipsoid-fit` default, `--point-fit` backup) with the planned-heights planner, which beats the closed-loop fixed schedule on the disturbed score.
- **Ellipse fit:** `--ellipse-fit` stays available for ellipse-shape data.

### Observations

- **Single runs are noisy.** Per-visit errors swing by 0.5–1 mm between consecutive
  ellipsoids. For example, HP3 `heights` reads 0.72 → 1.44 → 0.61 → 0.56 mm, and HP5r `backfill` reads 0.53 → 0.78 → 1.38 → … → 0.61 mm.
  - A variant's final error is one sample of that process.
  - Differences below ~0.5 mm between variants are not established by one run each.
  - Collection is deterministic, so a rerun repeats exactly; variance needs perturbed starts, e.g. a different home pose or force targets.
- **Encoder noise dominates the disturbed score.** At 0.2 mm encoder noise every variant reaches 2–8 mm.
  - Pulls and outliers (`mixed`) are rejected well by both fits, within ~0.3 mm of clean.
  - Real encoder noise is unknown. The noise columns matter more than any planner choice seen here.

## 5. Settled findings

### 5.1 "Slip" was an open-loop bug in our collector, not in the machine

The HP5 failure "Return on axis V did not reach its encoder origin (−15.19°)" was
reproduced in isolation
([`hp5_saturation_row.cfg`](evidence/scripts/hp5_saturation_row.cfg),
[`collect_once.py`](evidence/scripts/collect_once.py),
[`hp5_saturation_repro.txt`](evidence/results/hp5_saturation_repro.txt)).

How it happened:
1. Positioning held two lines in position mode at once (top −620 mm, A −310 mm).
2. Together they squeezed the effector. The top motor saturated and stopped 14.7° short; the encoder showed this correctly.
3. RRF `G1 H2` moves are relative to the firmware's *commanded* position, not to the encoder. `M569.4` mode switches do not reset that command.
4. The relative return therefore went exactly where it was commanded, and the stale command carried the 14.7° error into the return.

What the collector does now:
- `settleOnTargets` (`autocal/control/primitives/uncalibrated_actions.mjs`):
  1. reads the encoders after every positioning move and sends one correction for any miss;
  2. if the motor is still short, it is saturated: back it off past the reached length (3× the miss, at most ¼ of the move) and hold there;
  3. record the held length instead of the target.
- `returnMotorsToOriginOneAtATime` corrects any stale command from the encoders.

The same pose now collects (`axis V saturated at -617.763 mm … holding -555.999 mm instead`).

Lesson for future collectors: never chain relative moves without reconciling the
command with the encoder. A closed-loop machine only behaves closed-loop if the host
treats the encoder as the truth.

### 5.2 Level lines

A level line's useful length depends on how high the sphere is. Raising the effector
lengthens every low line, so targets relative to home ask for impossible poses at height. Three choices were tried:

| Level line | Result |
| --- | --- |
| Left idle where it settles (`fixed=n`) | nearly slack; HP5 biased 3.2 mm (heights2) |
| Shortened by the full span (heights3) | the row's circle shrinks to a point; rows rejected ("Insufficient coupled measurement points"); HP3 failed |
| Shortened half way from idle to the sphere's shortest (heights4, `fixed=n-X`) | all rows measurable, no lost runs |

The fixed schedule sidesteps this by using lengths from home that fit its
moderate heights.

### 5.3 The fit choice does not matter on this data; the data and stretch handling do

On the same ellipsoid datasets the point fit (`--point-fit`) and the ellipsoid fit agree
within ~0.1 mm. This holds clean, with pulls and outliers, and with 0.05–0.5 mm noise (both reach ~4 mm at 0.2 mm
noise, as the sweep benchmark does). No reason to prefer either for accuracy.

Removing the constant-force stretch correction costs 10.8 mm on HP4
([`height_hangprinter_4.txt`](evidence/results/height_hangprinter_4.txt)). Stretch handling is
first-order on these machines.

### 5.4 Planning to the reachable volume did not help

heights4 spans the most volume: held lengths to −1710 mm at 10% clearance. It is
nevertheless less accurate than the fixed schedule and the sweeps. Leave-one-height-out and
height-subset refits of heights4 all land at 0.8–2 mm
([`height_hangprinter_4.txt`](evidence/results/height_hangprinter_4.txt),
[`height_hangprinter_5.txt`](evidence/results/height_hangprinter_5.txt)). No single
height is "the bad one". The error is a property of the fit model meeting this data,
not of where the data is.

### 5.5 Held lengths were recorded inconsistently (fixed)

Since at least `dfc0e3e3`, a sweep's `fixed_lengths` was an encoder read taken *after* the sweep, while each point's `fixed_error_angles_deg` is measured from the positioning *target*. `pose_fit.dataset_line_lengths` adds the two, so each point's held length was off by the line's drift between target and end of sweep ([`held_length_gap.txt`](evidence/results/held_length_gap.txt)):

| Data | Double-counted drift |
| --- | --- |
| sweep benchmark (`benchmark_923c2baa_*`) | up to 0.046 mm |
| original fixed schedule | 0.009 mm (its collector met targets within 0.01°) |
| heights4 HP5 | 0.17 mm |
| sched HP5 | 0.35 mm |

The closed-loop collector settles on the reached length, and lines drift more afterwards.

**Fix:** the collector now records the target (`sweep_data_collection.mjs`, test in `force_tuning_checkpoint.test.mjs`).

Older datasets can be repaired by setting `fixed_lengths` from `fixed_target_angles_deg × mm_per_degree` ([`repair.py`](evidence/scripts/repair.py)). The loader does not do this yet: whether to repair historical references automatically is a decision for the next agent, because it shifts every stored reference result by ±0.1–0.4 mm.

### 5.5b Two regressions from the closed-loop collector, found by live default runs

`e4c6200d` was only live-tested with `--ellipsoid-shape`. The default sweep autocal then failed on all four machines.

1. **Force profile lookup.**
   - **Cause:** positioning returned encoder reads as the held targets. Equal-strain force profiles are matched by the planned target within 0.001 mm, so a 0.005 mm read difference left "Sensor force profile needs exactly one entry … found 0".
   - **Fix:** a line within tolerance of its target now keeps the planned target. Only saturated or `n±X` lines take the length they hold.
2. **Return-to-origin stability.**
   - **Cause:** the encoder correction waited for all encoders to settle while the other motors were still in force mode. HP5 hit "Timed out waiting for encoder stability" at 3809 s, deterministically.
   - **Fix:** the axis is now checked only with every motor held in position. A correction runs under the return's modes and is checked again.

A third failure was not a code bug: an HP3 G-code timeout during force tuning. Eight simulations at once oversubscribe 20 cores, since `rrf_simulator` alone uses ~1.6 cores. Run at most four at a time.

### 5.6 Single-run accuracy is a knife-edge

The original and the closed-loop fixed-schedule HP3 datasets are nearly identical. Swapping in three held lengths that differ by ≤0.12 mm takes the original from 0.29 to 1.11 mm; swapping two rows whose drive starts differ by 0.6 mm gives 0.56 mm ([`hp3_data_sensitivity.txt`](evidence/results/hp3_data_sensitivity.txt)).

The original's 0.29 mm is therefore not evidence that its schedule is better. Section 6 explains the mechanism.

## 6. The misfit: what is known

The error sits almost entirely in the held anchor's z (HP5 top z +1.73 mm, HP4 D.z
−0.81 mm in heights4), traded against the held line's home offset.

1. **The residual is small.**
   - The data fits the true anchors to 0.02–0.03 mm rms.
   - A free length-scale factor stays at 1.
   - The fitted rms at the wrong anchors is about the same as at truth.
2. **The posterior std of that coordinate is 0.3–0.5 mm on every dataset and plan.** That geometry amplifies a systematic length misfit of a few
   hundredths of a millimetre by about 15× into the held anchor's z. The amplification is
   the ill-conditioning of "anchor z vs home offset of the same line", which only height
   diversity of the *other* lines breaks.
3. **The residual at the true anchors grows with height**
   ([`rowres_hangprinter_4.txt`](evidence/results/rowres_hangprinter_4.txt),
   [`rowres_hangprinter_5.txt`](evidence/results/rowres_hangprinter_5.txt)). Per-row rms at truth:

   | Held length | HP4 | HP5 |
   | --- | --- | --- |
   | 0 | 0.010 mm | 0.017–0.020 mm |
   | −425 / −435 mm | 0.016–0.022 mm | 0.018–0.036 mm |
   | −850 / −870 mm | 0.016–0.019 mm | 0.029 mm |
   | −1280 / −1292 mm | 0.018–0.023 mm | 0.029–0.060 mm |
   | −1710 mm | 0.027 mm | 0.013–0.019 mm |

   This is not white noise: noise would not depend on the pose. Something in the length model is pose dependent:
   - the stretch correction's tensions;
   - effector rotation;
   - pulley/exit-point geometry;
   - the held motor's compliance;
   - or sag.

   HP5's highest sphere (−1713 mm) is clean again. There, all non-held lines are near-vertical and lightly loaded, which points at the *load-dependent* candidates.
4. **The stretch-scale probe** ([`stretch_scale.py`](evidence/scripts/stretch_scale.py)) rescales the stretch correction, on all lines or
   only the held line, and refits.

   **This is the clearest lead.**
   - The held anchor's z moves linearly with the scale, about 8.5 mm per unit (HP4 −8.5, HP5 +9.1).
   - The fit rms does not change at all (HP4 0.0354 ± 0.0002 mm, HP5 0.0600 ± 0.0002 mm), so the data cannot tell the stretch scale from the anchor height.
   - The error vanishes when only the **free** lines' correction is rescaled ([`stretch_scale_hangprinter_4.txt`](evidence/results/stretch_scale_hangprinter_4.txt), [`stretch_scale_hangprinter_5.txt`](evidence/results/stretch_scale_hangprinter_5.txt)). Rescaling the held line alone barely matters:

   | heights4 | ×1.0 | free lines ×0.9 | free lines ×1.1 | held line ×0.9 / ×1.1 |
   | --- | --- | --- | --- | --- |
   | HP4 max error (D.z) | 0.81 (−0.81) | 0.47 (−0.03) | 1.58 | 0.73 / 0.89 |
   | HP5 max error (I.z) | 1.73 (+1.73) | 2.41 | 1.04 (needs ~×1.19 for 0) | 1.95 / 1.51 |

   - The same holds on the *sweep* benchmark with the sweep autocal's point fit ([`stretch_scale_sweeps_hp5.txt`](evidence/results/stretch_scale_sweeps_hp5.txt)): HP5 pulley-reduced goes 1.30 → 0.72 mm with free lines ×1.1, while the top line ×1.1 changes nothing.
     HP3/HP4 sweeps get no stretch correction at all (`dataset_stretch` returns None for non-redundant sweep data); their known biases are covered by the stretch topic note.

   **Conclusion.** The "systematic misfit" is a ~10–20% error in the *free lines'* stretch correction:
   - too large on HP4, too small on HP5;
   - invisible in the residual, because it is exactly degenerate with the held anchor's height;
   - so the stop rule, the jackknife, the posterior std and the fit rms cannot see it.

   No planner can fix this; the stretch correction itself must be right. Each line's correction is its tension (from static equilibrium) × its compliance (from the firmware flex model, `autocal/flex.py`: M666 `S`, `U`, `N`, guy lengths `Y`). So one of three is off, per machine:
   - the tension estimate;
   - the compliance;
   - the guy-wire lengths.

### Candidate sources, in suggested order

0. **Flex model parameters vs the simulator.** Before anything else, compare per line:
   - `autocal/flex.py`'s compliance `(d·U + Y) / (S·U²·N)`, as implemented there;
   - the simulator's actual line EA, strand count, reeving and spool-to-anchor path length.

   The memory note on HP4 solver convergence records that RRF flex needs MA². A wrong MA power, strand count or guy length on the free lines would give exactly a per-machine scale error of the right size. It is cheap to check from the scenes (`public/usd_scenes/*.usda`) and the M666 in each dataset's `config`.
1. **Tension estimate in the stretch correction.**
   - `pose_fit.stretch_corrected_lengths` gets unmeasured line tensions from static equilibrium. It uses the commanded sensor force, `force_low` for idle lines and the effector weight.
   - Motor cogging (known: ~0.33/0.67 N sinusoid, see
     `research/topics/autocal-stretch.md`), pulley friction and reeving efficiency make the true tensions differ.
   - The held line carries the largest and most pose-dependent load.
   - Test: replay with true tensions (section 7.1) and refit with them.
2. **Held-line constancy.**
   - The held motor is in position mode, so the held line's *spool* angle is fixed, but its length is not: line stretch between spool and anchor varies with tension along the sphere.
   - The ellipsoid model assumes a constant held length plus the stretch model; any spool-side compliance (spool/flex not in the firmware model, M666 `I`/`J`/`L`/`H`) is unmodelled.
   - Test: record true held-line length per point in the replay.
3. **Effector rotation.**
   - HP4 rolls ~0.2°/m off-centre (chiral D reeving), slackening one of each ABC parallel pair; see the HP4 D.z investigation.
   - HP5 pulley-reduced has U=2 reeving too. Both machines show the largest errors.
   - The equal-strain per-strand fix (`/min(N)`) applies to sweeps; check whether the ellipsoid collection's constant-force rows keep pairs taut.
4. **Home reference.**
   - The home offset of the held line absorbs part of the error. Fitting offsets is needed (no offsets → 10+ mm), but the prior σ=5 mm and the "offset vs anchor z" trade are exactly the ill-conditioned direction.
   - A physical home reference under known tension would remove it; see the home-reference investigation in the topic note.

## 7. What to do next

### 7.1 Attribute the misfit exactly (highest value)

`research/output/session-2026-11-18-hp4-autocal-dz-bias/evidence/scripts/replay_poses.mjs`
replays a headless run's `sweeps.headless/events.jsonl` through the production world. It
records the true effector pose and every cable joint's force at each encoder read.

1. Run it on `output/heights4-hangprinter_5/` and the HP4 run, using the scene from that run's commit. The 2026-11-18 report explains the scene-version pitfall.
2. Swap in the true quantities one at a time, with the fitter unchanged and only `dataset_line_lengths` / the stretch tensions replaced, as in that report's `attrib*.py`:
   - true geometric lengths;
   - true tensions;
   - true held length.

The quantity whose substitution removes the held-anchor z error is the misfit.

Start with the free lines' tensions. Section 6 item 4 shows the error is the free lines' stretch scale: compare the equilibrium tensions `stretch_corrected_lengths` computes with the replay's joint forces, strand by strand. That includes HP5's parallel ABCD pairs (N=2) and HP4's U=2 reeving.

### 7.2 Measure variance before ranking planners

Ranking needs at least 3 perturbed runs per variant and machine. Perturb the home position by a few cm, or the force targets by ±5%. Until then, treat differences < 0.5 mm as unresolved.

### 7.3 Noise robustness

At 0.2 mm encoder noise every collection shape gives 2–8 mm. The fits reject pulls and outliers but have nothing against white noise except more points. Options:
- more points per row at the same time cost (rows are cheap relative to positioning, see 7.4);
- longer noise averaging per point;
- weighting by the recorded per-point sigma.

The real encoder noise level should be measured on hardware before optimising this.

### 7.4 Time

An ellipsoid costs 1000–1400 simulated s. The time goes into positioning, settling and force-mode waits, not into fitting. The new log timestamps (`autocal.py`, upstream `723b0646`) show it per step. Each ellipsoid's rows re-position the held and level lines from origin. Positioning once per ellipsoid and sweeping the rows from there would cut most of the per-row overhead.

### 7.5 Do not repeat

- Pure open-loop relative positioning: see 5.1.
- Level lines relative to home at large heights, or fully shortened: see 5.2.
- Switching to the point fit for robustness: it is equal, see 5.3.
- Ranking sweeps by info/second, or stopping on cost or covariance: negative results in the memory/topic notes.

## 8. Limitations

- One run per variant and machine, deterministic; see 4.
- The disturbed scores are offline refits of fixed datasets.
- The sweep baseline's datasets come from a different commit (`923c2baa`) than the ellipsoid runs. The fits are the current ones, but its collection predates the closed-loop collector.
- The original fixed schedule was rerun from `140dcbf4` in a git worktree placed under `output/`. RRF's host build cannot open its config from a long path: a worktree in the session scratchpad failed with "no configuration file found" and empty mm/deg.
- The repaired scores are refits. The live runs' stopping decisions were made on unrepaired data, so their times are as collected.
- The final live runs are in `output/fixsweeps-*` and `output/fixellipsoid-*`, which are not committed.
- Simulation only. Real encoder noise, real stretch and real friction are unknown.

## 9. CI confirmation and where its data lives

GitHub Actions run 44794416329 on `f54ef580` ran the default sweep benchmark
(`autocal/tests/test_autocal_benchmark.py`, one job per machine). It reproduced
the final local sweep runs to within 3 simulated seconds:

| Machine | Error | Simulated time |
| --- | --- | --- |
| HP3 | 0.97 mm | 4584 s |
| HP4 | 0.74 mm | 4922 s |
| HP5 | 1.04 mm | 3781 s |
| HP5r | 0.93 mm | 4685 s |

Where the data lives:
- **Branch runs:** artifacts only, `benchmark-<machine>-<attempt>` with results, logs and the headless manifest. Fetch them with `gh run download <run id> -R tobbelobb/hp-sim5 -p 'benchmark-*'`. They expire after 90 days, for this run on 2027-02-18.
- **Default branch:** after a merge to main, the `collect-benchmark-history` job appends the measurements to `results.jsonl`, `summary.md` and `by-fingerprint.md` on the `benchmark-history` branch. Compare against that history (`git fetch origin benchmark-history`).

CI does not benchmark `--ellipsoid-shape`, and it never runs the disturbance refits. Those numbers exist only in this report's evidence.

## 10. Reproduction

```bash
# live run of one variant (from the variant's commit)
.venv/bin/python autocal/autocal.py --headless-sim --machine-type hangprinter_5 \
    --ellipsoid-shape --dataset output/heights4-hangprinter_5/sweeps.json
# disturbed refits and ranking
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/robust_compare.py heights4 hangprinter_5 \
    output/heights4-hangprinter_5/sweeps.json ellipsoid > robust/heights4_hangprinter_5.json
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/rank.py <dir containing robust/>
# misfit diagnostics on one dataset
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/row_resid.py hangprinter_5 output/heights4-hangprinter_5/sweeps.json
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/height_diag.py hangprinter_5 output/heights4-hangprinter_5/sweeps.json
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/stretch_scale.py hangprinter_5 output/heights4-hangprinter_5/sweeps.json
```

The scripts were run from a scratch directory with the repository root on `PYTHONPATH`.
The run directories under `output/` are not committed. The raw datasets are regenerable from
the commits above, because collection is deterministic.
