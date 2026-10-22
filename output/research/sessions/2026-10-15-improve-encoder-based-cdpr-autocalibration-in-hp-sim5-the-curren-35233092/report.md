# Faster HP3 encoder-only autocalibration: a consensus spread gate and a compliance-aware fixed-anchor tolerance

Status: complete for every measured hypothesis (E01-E14). Two source changes
shipped (spread gate, fixed-anchor tolerance); a third candidate (solver choice,
E14) was measured and **rejected**. All numbers in this report are **simulated**.
No hardware measurement was made.

## Abstract

Question: can HP3 encoder-only autocalibration collect the same data faster, and
can it stop choosing worse anchors because its selection score disagrees with
geometry? Two changes were tested on simulated HP3 data. First, a spread gate
was added so that `consensus_anchor_estimate` refuses to average a candidate
pair separated by more than 5 mm and keeps the best-ranked fit instead. On 11
stored datasets this lowers the summed applied-anchor error from 788.58 mm
(best-ranked only) and 627.48 mm (unconditional consensus) to 615.87 mm; on the
recorded regression corpus it is 11.62 mm better than the previous behaviour, and
a stop-chain replay is mixed. Second, the fixed-anchor guard tolerance was raised
from 0.01 mm to 0.05 mm, because in the reference run the residual it chased was
bit-identical across all 20 points of every sweep despite a correction before
each point, so it was compliance rather than positioning error. A matched live
HP3 A/B measured 223.870 s vs 170.032 s of simulated time for identical 40-point
sweep sets (24.05% less, 1.35 s per point), and a third collection reproduced the
patched arm bit-for-bit. Two further measurements bound the risk of the change
and did not ship: the 9-sweep replay fit is *itself* ill-conditioned against
per-point length differences of the size the tolerance change causes (a 0.02 mm
per-point change moves the reported anchors by up to 2.4 mm, and the default
solver occasionally flips to a 33.8 mm optimum), and switching the solver to
trust-region-reflective removes that flip on the replay but costs 59-202 mm on
three recorded-corpus datasets, so it was rejected as a default. Limitation: all
evidence is simulated; the gate's gain is corpus-dependent and mixed on the stop
chain, and the end-to-end anchor-quality claim is not settled at the reference's
sweep width.

## Shipped changes

Three source files plus one test file changed; nothing else.

| File | Change |
| --- | --- |
| `autocal/history_selection.py` | `HISTORY_CONSENSUS_MAX_SPREAD_MM = 5.0`; `consensus_anchor_estimate` returns `(None, (), spread)` when the closest pair is wider than the gate, so the caller keeps the best-ranked candidate |
| `autocal/autocal.py` | the consensus log line now also reports `max_spread_mm=` |
| `autocal/tests/test_history_selection.py` | bimodal-average test replaced by two gate tests |
| `autocal/control/behaviors/sweep_data_collection.mjs` | `DEFAULT_FIXED_TOLERANCE_MM` 0.01 -> 0.05, with the evidence recorded in a comment |

The one-click flow is untouched: no new CLI flag, no extra prompt, and
auto-stop/auto-accept behaviour is not modified by either change.

## E01 - the selection score really does disagree with geometry (reproduced)

`artifacts/scripts/rankcheck.py` reads the reference-003 replay's final history
stage (`artifacts/replay-base/stages/history-007.json`) and prints, for every
surviving candidate, the shipped ranking score, the held-out prediction score and
the true anchor error:

| iteration | held-out sweeps | prediction score | rank score | true anchor error |
| --- | --- | --- | --- | --- |
| 1 | 6 | 10.7706 | **0.1835** | **26.569** |
| 2 | 5 | 7.1338 | 0.2247 | 12.437 |
| 3 | 4 | 8.6562 | 0.2247 | 12.436 |
| 4 | 3 | 0.3215 | 0.1928 | 9.641 |
| 5 | 2 | 0.6316 | 0.3629 | 13.988 |
| 6 | 1 | 0.4525 | 0.2078 | 9.202 |
| 7 | - | - | 0.2359 | 9.854 |

The failure the task describes is reproducible and large: **the candidate with
the best `rank_score` (iteration 1, 0.1835) is the least accurate by a factor of
2.9 (26.569 mm vs 9.202 mm)**, and the two most accurate candidates (iterations 6
and 4) are not the highest ranked. The held-out prediction score is also
monotone in the *number* of held-out sweeps (6 downto 1), not in accuracy: with
six held-out sweeps the fit is weak and predicts badly, which is a data-coverage
effect rather than a statement about anchor geometry.

Two separate consequences follow, and they are what the two changes attack:

1. The score is an imperfect proxy, so *which* candidate is applied can be wrong
   by tens of millimetres (26.6 vs 9.2 mm here).
2. `consensus_anchor_estimate` then *averages* two candidates. When the two are
   close this is a good move; when they are far apart the mean sits outside both
   and can be worse than either. This is measured in E02.

## E02 - spread-gate replay over the 11 stored datasets (supported)

`artifacts/scripts/verify_gate.py` runs the **shipped** `evaluate_history_candidates`
/ `rank_history_candidates` / `consensus_anchor_estimate` over the final history
stage of every replay in `artifacts/stage-sweep2` and `artifacts/stage-sweep-slide`,
and applies three policies to the same candidates:

| policy | summed applied-anchor error over 11 datasets |
| --- | --- |
| best-ranked candidate only | 788.581 mm |
| previous behaviour (average the closest pair, always) | 627.476 mm |
| shipped gate at 5 mm | **615.866 mm** |

Only 2 of 11 datasets change, and both change in the right direction:

| dataset | best-ranked | previous | gated | gain | closest-pair spread | gate decision |
| --- | --- | --- | --- | --- | --- | --- |
| `seventh_hp3_dataset` | 198.59 | 206.22 | 198.59 | +7.63 | 26.843 mm | refuse average |
| `nineteenth_hp3_dataset_...` | 165.01 | 168.99 | 165.01 | +3.98 | 7.050 mm | refuse average |
| 9 others | - | - | unchanged | 0.00 | 0.33-3.96 mm | average kept |

So the gate is **not** a general improvement to averaging; it is a narrow safety
net that fires exactly when the averaged pair is wider than 5 mm, which is the
regime the E01 argument says is unsafe. Every pair under 4 mm was already
harmless, so the gate leaves those alone.

## E03 - recorded-corpus regression replay (supported, and predicted by E02)

`autocal/tools/regress_calibration_logs.py` replayed all 11 recorded datasets and
scored the generated logs against the stored reference logs.

| revision | summed `true_err_total` delta vs reference logs |
| --- | --- |
| base `72523b9d` (previous behaviour) | -161.097 mm |
| shipped gate | **-172.719 mm** |

Difference: **11.622 mm better**. The same two datasets are the only ones that
change, and the change is 7.629 + 3.994 = 11.623 mm.

Cross-check: E02's offline replay predicted +11.610 mm and E03 measured
+11.622 mm - agreement to 0.012 mm. The offline harness is therefore a valid
predictor for this change, which is why E02 was used to pick the gate value.

Caveat, stated plainly: this replay ends with `ONE OR MORE DATASETS: FAIL`. That
failure is **pre-existing at the base revision** (the stored reference logs
predate commit `ee8f2716`), not caused by this change. Evidence:
`artifacts/scripts/reg-head.txt` and `artifacts/scripts/reg-gate.txt`.

## E04 - stop-chain replay (mixed)

Six replays of the reference-003 sweep chain truncated to the first N sweeps
(`artifacts/stop-N*/` at the base revision, `artifacts/stop-gated/stop-N*/` with
the gate), scored the same way:

| N | previous | gated | delta (negative = gate better) |
| --- | --- | --- | --- |
| 4 | 19.095 | 26.578 | **+7.483** |
| 5 | 19.095 | 12.431 | **-6.664** |
| 6 | 10.839 | 10.839 | 0.000 |
| 7 | 10.839 | 10.839 | 0.000 |
| 8 | 9.279 | 9.279 | 0.000 |
| 9 | 9.677 | 9.677 | 0.000 |
| | | net | **+0.819** |

Honest reading: **mixed, and slightly negative on this chain.** The gate helps at
N5 and hurts at N4 by a similar amount. At N4 the gate refuses the average and
keeps the best-ranked candidate, which happens to be worse than the average - so
the gate is not a free win; it trades one failure mode (a wide mean) for another
(trusting a bad single fit). It is kept because the broader 11-dataset corpus
favours it by 11.6 mm and because it cannot produce a catastrophic average, but
this is a **mixed** result and the next session should treat the 5 mm value as
provisional.

## E05 - where the time goes

Profiled from `artifacts/reference-003/sweeps.full_auto_log.jsonl` and the sweep
dataset (`artifacts/scripts/pullout.py`, `phase.py`, `timeline.py`):

- The reference run is about 6756 s of simulated time; the 9 sweeps occupy
  about 3231 s of it (48%).
- **Slack-line pullout is the single largest sweep cost: 18 waits (2 per sweep)
  totalling 1265 s = 39% of sweep time and 19% of the whole run.** Individual
  waits ran 60-91 s and *never* hit the 120 s `FORCE_TRAVEL_TIMEOUT_MS`
  (`sweep_data_collection.mjs:77`), so the timeout is not what ends them.
- Mean inter-point gap is about 16-20 s, and each point ends with a full
  `waitForStableEncoders` (1000 ms quiet window, 500 ms poll,
  `uncalibrated_actions.mjs:80-250`).
- In this session's live 2-sweep traces the same shape appears at a smaller
  scale: 1.0-1.5 s per drive move, 1.6-3.3 s of settle per point, plus a
  ~17.7 s settle at the post-sweep fixed-anchor re-zero, twice per sweep.

E05 is what makes H4 (below) the recommended next target: the pullout is 39% of
sweep time and is a *measurement-model* decision, not an oversight.

## E06 - the correction moves nothing

`artifacts/scripts/fixederr.py` reads `fixed_error_angles_deg` from every data
point of the reference run. Every one of the 9 sweeps shows the same structure:
the residual on a given fixed anchor is **constant to the printed precision
(1e-4 deg) across all 20 points of the sweep**, even though a ~0.03 mm correction
move was issued before every point.

| sweep | fixed | per-sweep residual range (deg) | in mm |
| --- | --- | --- | --- |
| 001 | [0,3] | +0.0520 / 0.0000 | 0.0272 / 0 |
| 002 | [1,3] | +0.0520 / 0.0000 | 0.0272 / 0 |
| 003 | [2,3] | +0.0520 / 0.0000 | 0.0272 / 0 |
| 004 | [1,3] | +0.0244 / -0.0276 | 0.0128 / 0.0145 |
| 005 | [2,3] | +0.0068 / -0.0552 | 0.0036 / 0.0289 |
| 006 | [0,3] | -0.0208 / +0.0372 | 0.0109 / 0.0195 |
| 007 | [2,3] | -0.0484 / +0.0096 | 0.0253 / 0.0050 |
| 008 | [1,3] | +0.0340 / -0.0180 | 0.0178 / 0.0094 |
| 009 | [0,3] | +0.0164 / -0.0356 | 0.0086 / 0.0186 |

Aggregate over all 360 fixed-anchor readings: median 0.0260 deg (0.0136 mm),
**maximum 0.0552 deg (0.0289 mm)**, 61.1% above 0.01 mm, and **0.0% above
0.1 mm**. Every sweep has at least one reading above 0.01 mm; none reaches 0.05 mm.

Conclusion (supported): at a 0.01 mm tolerance the guard fires at essentially
every point, issues a ~0.03 mm move, and pays a full settle, but the residual it
is chasing is unchanged before and after - it is the loaded equilibrium of a
position-controlled anchor, i.e. compliance. A 0.05 mm tolerance is 1.7x the
largest residual ever observed here, about 5x the 0.02 deg encoder noise floor
(0.0105 mm at 30 mm radius), and about 16x below the 1.5 deg drift-recovery
threshold, so a genuine slipping drift is still caught.

Note on an earlier draft: an intermediate note in this session claimed "180/180
points exceeded 0.01 mm". That was wrong - the correct figure is 61.1% of
individual readings, with every sweep represented. The code comment was corrected
to state the measured numbers.

## E07 - matched live HP3 A/B (supported)

Two collections, identical in every respect except the collector source:

| | arm A (before) | arm B (after) |
| --- | --- | --- |
| `DEFAULT_FIXED_TOLERANCE_MM` | 0.01 | 0.05 |
| `bridge_source_sha256` | `9a9db935...` | `fb83c87b...` |
| configs | `[0,3]/1/2`, `[1,3]/0/2` | identical |
| options | `{returnToOrigin:true, sweepPoints:10}` | identical |
| sweeps / points | 2 / 40 | 2 / 40 |
| steps executed | 111935 | **85016** |
| simulated time | 223.870 s | **170.032 s** |
| wall time | 76.1 s | 64.5 s |
| drive span (sweep 1 / 2) | 95.5097 / 85.7341 mm | 95.5358 / 85.7079 mm |
| `firmware_anchor_residual_rms_mm` | 0.1798 | 0.1942 |

**Saving: 53.838 s of simulated time, 26919 steps, 24.05% - for exactly the same
40 points from exactly the same two configs.** Per point: 5.597 s -> 4.251 s,
i.e. **1.346 s saved per point**. The drive spans agree to 0.03 mm, so the
motion plan is unchanged; the saved work is the repeated no-op correction move
plus its settle.

Data equivalence, paired point-by-point in collection order
(`artifacts/ab-matched/`, `artifacts/scripts/ab_points.py`): the two arms differ
by **at most 0.094 mm** on any channel (about 0.18 deg), quantised at the 0.01 deg
encoder step, and most points differ by 0.0. The difference is not random noise
in the harness: it is the small physical displacement of the correction moves
that arm B no longer performs, which E06 shows is ~0.03 mm.

Wall time is reported but is not the measurement: the host was heavily loaded
(load average 10-63) and wall time varies. Simulated time and step count are
exact integers of the fixed-step clock.

## E08 - determinism control (supported)

A third collection with the patched collector in a fresh world reproduced arm B
**bit-for-bit**: 85016 steps, 170.032 s, and a maximum absolute data difference of
exactly **0** over every length and angle in both sweeps
(`artifacts/ab-matched/runC_tol0.05_repeat_*`). Run A was likewise reproduced
bit-for-bit at 223.870 s / 111935 steps on two separate executions.

This matters for interpretation: because the simulator is deterministic, the
A-vs-B differences in *both* time and data are fully attributable to the source
change, with no run-to-run noise term. It is what makes the E07 comparison an
exact measurement rather than a statistical one.

## E09 / E10 / E12 - what did not work

| ID | Idea | Result | Evidence |
| --- | --- | --- | --- |
| E09 | Fewer points per sweep (thinning 20 -> 16/12/10/8/6/4 per sweep) | **Rejected.** Thinning to 8 points per sub-sweep gives 11.975 mm vs 9.857 mm at full width, and some schemes are catastrophic (9308.99 mm, 4495.04 mm) because the retained subset becomes unidentifiable. At m<=4 every scheme collapses (4880.98 mm). | `artifacts/points_study.json` |
| E10 | Fewer sweeps (first k sweeps only) | **Inconclusive.** k=3 -> 26.572 mm, k=4 -> 5222.27 mm (unidentifiable), k=5 -> 12.439 mm, k=6 -> 9.644 mm, k=7 -> 13.987 mm, k=8 -> 9.205 mm, k=9 -> 9.857 mm. The k=6..9 values are within a few mm of each other and non-monotone, so the spread is a single-realisation effect, not evidence that 6 sweeps suffice. | `artifacts/sweep_count_study.json` |
| E12 | Fit `sigma_floor` or more solver restarts | **Rejected.** `sigma_floor_mm` changes the cost by orders of magnitude with no accuracy gain (0.5 -> 10.84 mm, 1.0 -> 14.18 mm, 2.0 -> 26.78 mm, 5.0 -> 50.60 mm on the second dataset), and `solve_restarts=16` is not better (10.906 / 12.885 mm vs 9.857 / 9.644 mm). | `artifacts/sigma_floor_study.json` |

E09 is the direct negative answer to "can we just collect fewer points": the
same *geometry* must be sampled, and the identifiability cliff is sharp.

## E13 - sensitivity probe on the wrong field (superseded by E13b)

An attempt was made to bound the effect of the E07 data difference offline:
replay the reference dataset with 0.05-0.5 mm added to the collected encoder
angles (`artifacts/scripts/sensitivity.py`).

| added offset | added to all channels | added to sensor channel only |
| --- | --- | --- |
| 0.00 mm | 9.677 mm | 9.677 mm |
| 0.05 mm | 9.677 mm | 9.677 mm |
| 0.10 mm | 9.583 mm | 9.677 mm |
| 0.20 mm | 9.677 mm | 9.677 mm |
| 0.50 mm | 9.279 mm | 9.677 mm |

The probe was recorded as "inert, because the data are delta-referenced within a
sweep". That explanation is **wrong**, and so is the probe. A direct control run
settles it: adding **+-5 deg per point** to `raw_angles_deg` leaves the fit
bit-identical (same `fit_q` 0.1928, same anchors), while adding +-0.094 mm per
point to `l_drive`/`l_sensor` changes it. On the `--dataset ... --no-collect`
replay path the fit consumes the collected **length pair `l_drive`/`l_sensor`
(and/or their `_mu` noise-mean variants)**, not `raw_angles_deg`. E13 perturbed
the wrong field, which is why it looked inert; the non-monotone 9.583/9.279 mm
values are second-order side effects of also overwriting the `mu` field.
`artifacts/scripts/e13b_rawangles_probe.py` is the control.

**E13 is not used to support any claim.** E13b below redoes the probe on the
field the fit actually reads.

## E11 - end-to-end A/B on the same 3 sweeps (time only)

The maximum distinct sweep geometry HP3 offers through a single collection is 3
configs (`[0,3]/1/2`, `[1,3]/0/2`, `[2,3]/0/1`); `loadSweepConfigFile` explicitly
skips duplicate configs (`sweep_data_collection.mjs:1019-1031`), so a 9-sweep
chain cannot be requested through one `start_collection` call. The end-to-end A/B
was therefore run at 3 sweeps x 20 points
(`artifacts/e11/e11_summary.json`, `artifacts/scripts/e11_compare.py`):

| | arm A | arm B |
| --- | --- | --- |
| tolerance | 0.01 mm | 0.05 mm |
| `bridge_source_sha256` | `9a9db935...` | `fb83c87b...` |
| sweeps / points | 3 / 60 | 3 / 60 |
| steps executed | 161992 | **122716** |
| simulated time | 323.984 s | **245.432 s** |
| wall time | 106.0 s | 67.8 s |

**Time: the saving reproduces, 78.552 s of simulated time and 39276 steps
(24.25%, 1.309 s per point).** That is the same relative saving as the 2-sweep
E07 arms, not a larger one.

**Anchor quality: not usable from this experiment.** Two separate reasons:

- The arms are only matched in their first two sweeps. Sweeps 0 and 1 agree to
  0.006 mm on the drive setpoint and 0.094 mm on the lengths - the same
  tolerance as E07 - but **sweep 2 is not matched**: arm A traversed
  `+43.4 .. -0.1` then `-47.1 .. -0.1` (90.478 mm of travel) while arm B traversed
  `-47.1 .. -0.1` twice (47.035 mm), so the two datasets do not describe the same
  trajectory. The extra travel is worth only ~1.9 s at feed 1400 mm/min, which is
  why the time number survives, but it makes the fitted anchors incomparable.
- Independently, k = 3 is not identifiable (E10): fitted with the stock replay
  harness both arms land far from the true HP3 anchors (arm A `fit_q` 5.727, arm B
  7.163), so neither arm's anchors are a usable quality measurement at this width.

What this does and does not establish:

- It **does** confirm the time saving in a second, independent arm pair, at a
  different sweep count, with the same relative size (24.25% vs 24.05%).
- It does **not** establish anything about anchor quality. The quality comparison
  needs k >= 6 and *matched* sweep geometry; that is E15.
- An earlier draft of this session recorded arm B as 112334 steps / 216.026 s and
  reported a 33.3% saving plus a 26.572 mm fit. Those numbers were not read from
  the artifacts and are wrong; the values above are the ones the manifests
  contain. The lesson is in the E14 pitfall note: every claim here is read back
  from a stored manifest rather than reconstructed from memory.

## E13b - the 9-sweep fit is ill-conditioned against per-point length noise

The tolerance change perturbs the collected drive/sensor lengths *point by point*,
not per sweep: in the matched E07 arms the two arms' `l_drive`/`l_sensor` differ
by 0 at 51% of readings and by 0.005-0.094 mm (mean 0.053 mm) at the rest, and the
pattern alternates within a sweep rather than being constant
(`artifacts/scripts/e13b_ab_distance.py`). So a per-point perturbation of that
shape and magnitude is the right model of the change's effect on the data.

Injecting such a perturbation into the full 9-sweep reference replay
(`artifacts/scripts/e13b_perpoint.py`, `artifacts/sensitivity-perpoint/`) gives:

| per-point perturbation on `l_drive`/`l_sensor` | n | true anchor error (mm) | `fit_q` |
| --- | --- | --- | --- |
| none (baseline) | 1 | **9.677** | 0.1928 |
| uniform +-0.001 mm | 4 | 9.820 / 9.875 / 10.734 / 11.001 | 0.20-0.45 |
| uniform +-0.005 mm | 4 | 10.750 / 11.161 / 11.227 / 13.360 | 0.18-0.39 |
| uniform +-0.010 mm | 3 | 9.811 / 10.320 / **18.884** | 0.06-0.30 |
| uniform +-0.020 mm | 4 | 9.672 / 9.802 / 12.016 / 12.183 | 0.26-0.29 |
| uniform +-0.030 mm | 3 | 10.545 / 10.550 / **30.284** | 0.26-0.36 |
| uniform +-0.050 mm | 3 | 10.703 / 11.058 / **33.810** | 0.20-0.58 |
| uniform +-0.094 mm | 6 | 9.685-14.084 | 0.86-1.53 |
| sparse, measured shape, <=0.094 mm | 6 | 10.407-12.351 | 0.66-0.97 |
| uniform +-0.150 mm | 4 | 9.503-14.706 | 1.10-3.25 |

Three findings, in order of importance:

1. **The fit is knife-edge.** A +-0.001 mm per-point change - one fifth of one
   0.01 deg encoder quantum, and far below any physical resolution - already
   moves the reported anchors by 0.14-1.32 mm. Over the well-identified 9-sweep
   set the four-anchor geometry therefore has a shallow, near-flat direction, and
   the anchor estimate slides along it. Multiplying the perturbation by 100
   (to +-0.094 mm) costs only about 4 mm more. **The 1-3 mm of variation is a
   property of the fit, not of the tolerance change.**
2. Consequently the tolerance change's own ~0.05 mm per-point difference is
   *inside* the fit's existing scatter band, not a distinct new error source: the
   sparse "measured" row is no worse than the tiny +-0.001 mm row's worst case.
3. Spread grows with magnitude (the +-0.094 mm row's `fit_q` is 0.86-1.53 against
   0.19 baseline), so the change is not *free* of quality risk either. It is
   bounded at roughly a few millimetres of anchor error, not zero.

This is the honest limit of the end-to-end claim: the time saving is measured and
reproducible, the data difference is measured and small, and the sensitivity of
the fit to a difference of that size is measured and only a few millimetres -
but a few millimetres is not nothing, and it cannot be separated from the fit's
own conditioning at this sweep width without the k >= 6 collection that was not
achieved (E15).

## E14 - solver choice: better on the replay, rejected on the corpus

Because E13b shows the fit is ill-conditioned, a solver that picks its optimum
more reliably would be valuable, and it would cost nothing to collect.
`--solve-restarts` and `--solve-iterations` are already covered by E12 (no gain),
so the remaining lever is `--solve-optimizer` (`lbfgsb` default, plus `lm`,
`trf`). On the 9-sweep reference replay and on five perturbed copies of it
(`artifacts/scripts/e14_solver_replication.py`, `artifacts/solver-study/`):

| dataset | `lbfgsb` (default) | `trf` | `lm` |
| --- | --- | --- | --- |
| base | 9.677 | 9.595 | 9.698 |
| uniform +-0.020 mm, seed 0 | 12.016 | 9.539 | 9.611 |
| uniform +-0.020 mm, seed 1 | 12.183 | 9.746 | 9.814 |
| uniform +-0.020 mm, seed 2 | 9.802 | 9.651 | 10.674 |
| uniform +-0.020 mm, seed 3 | 9.672 | 9.770 | 9.663 |
| uniform +-0.050 mm, seed 0 | **33.810** | 9.583 | 9.734 |

`trf` never exceeded 9.77 mm over the six datasets, where the default reached
33.81 mm, and it fit the same dataset in **13.8 s instead of 21.9 s**
(`--solve-iterations 2000` gives bit-identical `trf` results, so iterations were
not the binding constraint). That looks like a free win, so it was tried as the
default and validated on the recorded corpus:

| run (same tree, same reference logs) | sum of true-error delta over 11 datasets |
| --- | --- |
| `lbfgsb` (shipped default) | **-172.719** |
| `trf` | +249.768 |
| `trf` with `--solve-iterations 4000` | +249.768 |

The regression is concentrated: `sixteenth_hp3_dataset_even_more_pressure`
+201.9 mm, `23_april_2026` +162.1 mm, `seventh_hp3_dataset` +58.8 mm, and all
eight other datasets unchanged to 0.6 mm
(`artifacts/scripts/reg-trf.txt`, `reg-trf-it4000.txt`; control
`reg-gate.txt`). Two of the three regressing datasets are HP3, so this is not a
"wrong machine" effect.

**Outcome: rejected.** `trf` is more robust and 1.6x faster on the target regime
but it moves the chosen optimum on three recorded datasets by 59-202 mm, which a
global default change cannot justify. Leave `lbfgsb` as the default and instead
consider running `trf` as an *extra candidate inside the existing consensus /
selection pool* (E17), where a bad optimum can be outvoted rather than applied.

A methodological pitfall worth recording: a first version of this grid ran four
concurrent autocal processes against **one** `--dataset` path. The CLI derives
sidecar paths (`<dataset>.replay_tmp.json`, `<dataset>.full_auto_log_N.jsonl`)
from the dataset path, so the processes raced and produced irreproducible
numbers. Every concurrent replay needs its own dataset path.

## Limitations

- **All evidence is simulated** (HP3, layering off, radii 30 mm, no buildup). No
  hardware measurement was made, and no successful tool check establishes
  calibration accuracy.
- **The A/B arms are narrower than the documented reference run.** The E07/E11
  arm sweeps span about 95 mm of drive, while `reference-003`'s sweeps span about
  1191 mm with a 517 mm fixed-anchor target and 3.35 N pullout force. The
  *absolute* saving of about 1.31-1.35 s per point is what should transfer;
  expressed as a fraction of the reference's own ~18 s per point it is roughly
  7%, not 24%. The 24% figure is specific to these narrow arms. No matched A/B at
  the reference's sweep width was achieved, because the MCP collection tool
  exposes no way to set the fixed-anchor target or the drive range.
- **The end-to-end anchor-quality claim is not settled.** E11's third sweep is
  unmatched and k = 3 is unidentifiable, and E13b shows the effect of the change's
  data difference is a few millimetres but cannot be separated from the fit's own
  ill-conditioning at that width. Closing this needs a k >= 6 matched A/B (E15).
- **The fit, not the tolerance change, is the fragile part.** A +-0.001 mm
  per-point change to the 9-sweep dataset moves the reported anchors by
  0.14-1.32 mm, and larger per-point changes occasionally flip the default solver
  to a 19-34 mm optimum (E13b, E14). The measurements here therefore bound the
  tolerance change *inside* an already-noisy band; they do not show the band is
  small.
- `E14` rejected `trf` as a default on the recorded corpus (three datasets 59-202
  mm worse), while the same solver was clearly better and faster on the
  well-identified 9-sweep HP3 replay. Those two statements are both measurements;
  neither is derived from the other, and the conflict is unresolved.
- The gate's benefit is corpus-dependent (2 of 11 datasets) and its stop-chain
  effect is slightly negative (E04).
- E13 perturbed `raw_angles_deg`, which the `--dataset` replay path does not
  consume, so it produced no bound; it is superseded by E13b and used for nothing.
- Reference-003's `sweeps.json` stamps `machine_type: hangprinter_4` because the
  collector defaults `machineType` that way even for HP3 runs; the HP3 identity
  comes from `machine_design` and the true-anchor match.
- The `ONE OR MORE DATASETS: FAIL` line in the E03 regression is pre-existing at
  the base revision.

## Next steps

1. **E15 (proposed, not tested): a matched end-to-end A/B at k >= 6** at the
   reference's sweep width, to close the E11 question. Needs either a collector
   option for sweep width / fixed-anchor target, or a multi-job collection with
   verified matched geometry (E11 shows that `start_collection` in a continuing
   world does *not* automatically give matched placement).
2. **E16 (proposed, not tested): shorten the pullout wait.** It is 39% of sweep
   time (E05). A measurement-model change (when is the slack line actually taut?),
   not a tolerance tweak.
3. **E17 (proposed, not tested): run `trf` as an extra candidate inside the
   consensus / selection pool** rather than as the default. `trf` held 9.54-9.77 mm
   on the HP3 replay against the default's 9.67-33.81 mm and fit it 1.6x faster;
   the selection machinery already exists to outvote a bad optimum, and the
   E14 corpus regression shows that applying `trf`'s optimum directly is what
   hurts.
4. **E18 (proposed, not tested): re-derive the 5 mm gate on a larger corpus.**
   Its value comes from a 2-dataset signal and it is slightly negative on the
   stop chain, so the constant should be treated as provisional.

## References

- [Extended autocal reference data](autocal/data/references/extended-reference.md) -
  how to make full recorded collection runs. Supports E05/E11 provenance.
- [Recording performance budgets](research/autocal-recording-performance.md) -
  matched HP3 replay cost of `none`/`events`/`sampled`/`full` recording. Used to
  choose `events` for live collection and numerical-only replays offline.
- `output/extended-autocal/no-buildup-reference-003` - the comparison target
  named in the task; source of E01, E05, E06 and the E04 stop chain.
- `autocal/tools/regress_calibration_logs.py` - the recorded-corpus regression
  used in E03, including `SIM_3D_TRUE_ANCHORS` and `_anchor_distance_common_dims`.
- [E13b per-point sensitivity script](artifacts/scripts/e13b_perpoint.py) and
  [results](artifacts/sensitivity-perpoint/) - per-point length perturbation of
  the 9-sweep replay; supports E13b. The `raw_angles_deg` control is
  [e13b_rawangles_probe.py](artifacts/scripts/e13b_rawangles_probe.py).
- [E14 solver study](artifacts/solver-study/) and
  [e14_solver_replication.py](artifacts/scripts/e14_solver_replication.py) -
  `lbfgsb`/`trf`/`lm` on the 9-sweep replay and its perturbed copies.
- `artifacts/scripts/reg-trf.txt`, `reg-trf-it4000.txt` - recorded-corpus
  regression with `trf` as the default, against the `reg-gate.txt` control.
- [E11 summary](artifacts/e11/e11_summary.json) and
  [e11_compare.py](artifacts/scripts/e11_compare.py) - the 3-sweep A/B manifests,
  matched-ness check and time difference.
- [Research catalog](research/catalog.md) - index of prior sessions
  (`2026-10-08-...-c812e7b9` guarded settling, `2026-10-10-...-51aa157a`,
  `autocal-held-out-history-ranking-7ec4260f`, `fdb616a4...`, `b7deb1bc...`,
  `a71aec03...`) consulted for prior art; the disproved ideas listed in
  `research.md` come from those sessions.
- Local code revision: base commit `72523b9d` plus the four modified files listed
  above. Experiment IDs and artifact paths are relative to this session directory;
  scripts used are preserved in `artifacts/scripts/`.
