# Fewer calibration sweeps for encoder-based CDPR autocal: a consensus-spread early stop

Status: complete. Simulation evidence; the live HP3 confirmation run (E09c) finished and
is reported below.

## Abstract

The hp-sim5 full-auto encoder autocalibration is slow mainly because it keeps
collecting sweeps after the applied geometry has converged: it stops on patience
(three non-improving iterations) or an absolute cost/std threshold, and the
ranking scalar those use disagrees with geometrical accuracy. We tested whether
the run can stop as soon as the two mutually most consistent *distinct* candidate
fits agree to a small fraction of the machine's own extent, using the shipped
bracketed-consensus estimate as the applied geometry. Offline replay decks built
from 13 recorded runs (rule disabled, `--no-collect --patience 100`) selected a
fractional limit of `1.5e-3`, a minimum of 4 candidates and two consecutive
agreeing iterations. On the recorded reference-003 sweep deck this saves 2 of 9
sweeps (22%) for +0.18 mm applied-anchor RMSE; on the shipped 11-dataset regression
harness it saves 6 sweeps with aggregate RMSE improving 0.64 mm; across 8
converged offline decks it saves 23 sweeps with mean regret 0.051 mm (worst
0.221 mm). A live HP3 job stopped itself at iteration 6 instead of 7 (-11%
simulated collection time) for +0.21 mm, and exposed a `normal_completion`
regression that this session also fixes. All evidence is simulated.

## Implemented change

1. `autocal/history_selection.py` gains `consensus_spread_hit(anchors, spread, *, frac)`
   and the constants `SPREAD_STOP_FRAC = 1.5e-3`, `SPREAD_STOP_MIN_CANDIDATES = 4`,
   `SPREAD_STOP_CONSECUTIVE = 2`. The predicate compares the shipped consensus
   pair's separation to `frac * extent(consensus anchors)`, where extent is the
   largest pairwise anchor distance, so the limit scales with the machine and
   works for planar (3-anchor) and spatial (4-anchor) layouts. It returns False on
   non-finite, degenerate or short input, so it can never stop a run by itself.
2. `autocal/autocal.py` evaluates it every iteration once at least 4 candidates
   exist, requires two consecutive hits, and accepts when it fires -- alongside
   the existing patience/threshold logic, which is untouched. The console prints
   one line when it fires, the JSONL records `stop_spread_hit`,
   `consensus_spread_mm` and `consensus_spread_limit_mm`, and the stop reason
   becomes `spread-consensus`. `HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC=off` disables it
   (used for every A/B below); an invalid value keeps the default.
3. `autocal/_autocal_common.py` now owns `FULL_AUTO_STOP_REASON_PATIENCE`,
   `SPREAD_STOP_REASON` and `NORMAL_STOP_REASONS`, and
   `autocal/headless_sim.py` classifies a manifest as a normal completion when its
   stop reason is in that set. Without this second change a successful early stop
   was reported as a failed job (see "Robustness finding").

The user experience is unchanged in kind: still one command, still an automatic
stop, no new setting. Runs that would have stopped on patience keep their old
behaviour except where the new rule fires earlier.

## Method

- **Rule-independent decks.** Each recorded dataset is replayed through the
  production loop with `--no-collect --patience 100`, so the loop consumes every
  recorded sweep and emits the complete candidate sequence. A stopping rule can
  then be scored offline at every candidate index without re-collecting, and
  without the rule influencing which candidates exist.
- **Scoring.** Applied-anchor RMSE against the nominal layout,
  `sqrt(mean_i |a_i - a_true|^2)` over the machine's anchors. Truth is never an
  input: the loop sees only candidate anchors, rank scores and coverage
  adjustments. "Regret" compares the applied geometry at the rule's stop index
  with the applied geometry at the shipped rule's stop index on the same deck, so
  a saving is only credited when geometry does not degrade.
- **Identical-CLI A/B.** Every comparison varies exactly one input:
  `HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC` (`off` vs `1.5e-3`) on the same recorded data,
  or nothing at all between arms of the offline replay.

## Results

### Choosing the rule (E05, E06, E07)

The 96-point grid over `(frac, n_min, k)` in [E06](evidence/E06-stop-regret-grid.json)
picked `frac = 1.5e-3`, `n_min = 4`, `k = 2`. Tightening `n_min` to 3 fires at only
three candidates and then lets the spread blow up to 970 mm on
`headless_ext_verif`; `k = 1` is safe but weaker; `frac = 3e-3` stops on a wrong
plateau in two runs (worst regret 2.721 mm). [E07](evidence/E07-rule-family-compare.json)
rejected the competing consensus-*delta* stability family: stopping when the
running estimate stops moving fires on a still-wrong 5.11 mm plateau
(mean regret 0.668 mm, worst 2.721 mm).

### Offline decks, converged runs (E13)

[E13](evidence/E13-converged-regret.json), computed on the E05 decks:

| | value |
| --- | --- |
| converged decks (final consensus RMSE <= 10 mm) | 8 of 13 |
| sweeps saved total / mean per deck | 23 / 2.88 |
| mean regret vs shipped patience-3 stop | 0.051 mm |
| worst regret (`s13_E17_window1000`) | 0.221 mm |
| decks unchanged, slightly worse | 6, 2 |

On the five decks whose fits never converge the predicate does not fire before
patience, so those runs stop exactly where they did before, with one caveat noted
under limitations.

### Recorded reference-003 deck (E12)

[E12](evidence/E12-reference-003-ab.json): identical CLI, only the env var differs.

| arm | sweeps | iterations | applied-anchor RMSE |
| --- | --- | --- | --- |
| `off` (shipped stop) | 9 | 7 | 2.623 mm |
| `1.5e-3` (new stop) | 7 | 5 | 2.805 mm |

Two of nine sweeps saved (22% of collection) for +0.18 mm. The fire message is
`; full-auto: stopping early; the two closest distinct fits agree to 4.698mm (limit 4.945mm).`

### Shipped regression harness (E11)

[E11](evidence/E11-regression-harness-ab.json), 11 datasets, `--no-collect` replays,
identical CLI, logs compared arm-to-arm: 6 sweeps saved in total; RMSE sum over
datasets 177.656 -> 177.020 mm.

| dataset | sweeps | RMSE off -> on |
| --- | --- | --- |
| `layered_manual_some_bad_points` | 6 -> 4 | 1.758 -> 0.911 mm (better) |
| `ten_points_w_buildup2` | 6 -> 4 | 1.439 -> 1.650 mm (worse) |
| `five_points_bigger_deltas` | 5 -> 4 | 4.096 mm unchanged |
| `ten_points_bigger_deltas` | 5 -> 4 | 0.868 mm unchanged |
| other 7 datasets | unchanged | unchanged |

The harness's own verdict against the frozen October-8 reference logs is
`ONE OR MORE DATASETS: FAIL` with `worst_param_delta_total = 0.000` per iteration,
because four datasets now stop 1-2 iterations earlier and their final M669 changes.
That is the intended effect of a stopping-rule change on a frozen-log regression,
not a numerical fault; the frozen logs remain the pre-change baseline and were not
edited.

### Live HP3 end-to-end (E09b, E09c)

E09b (full default forces, no layering, `events` recording) stopped itself after
320 points and 8 sweeps:

```
; full-auto: stopping early; the two closest distinct fits agree to 1.679mm (limit 4.944mm).
```

The comparable prior default-force HP3 acceptance (session `2026-10-10-...-51aa157a`,
E12) used 360 points, 9 sweeps, 6,011 simulated seconds and reached 2.604 mm.
E09b used 5,332 simulated seconds for 2.814 mm: about 11% less simulated collection
time for +0.21 mm.

E09c re-ran the identical command on the fixed tree and was accepted. It reproduced
E09b exactly -- same 2,665,976 physics steps, 5,331.952 simulated seconds, 320 points,
same applied anchors and 2.8141 mm RMSE, same `spread 1.679mm (limit 4.944mm)` -- so the
pair isolates the lifecycle classification from the physics
([pair evidence](evidence/E09-live-hp3-pair.json)).

## Robustness finding (fixed)

E09b's manifest was `status: complete`, `stop_reason: spread-consensus`,
`applied_parameters` populated, recording finalized with `autocal_complete: true`
-- yet `normal_completion` was false and the supervisor therefore
reported the job as **failed**
([job.json](native/native/autocal/E09b-spread-hp3-live/job.json)). The cause was a
literal `stop_reason == "patience-or-threshold"` check in `autocal/headless_sim.py`:
the new stop reason was simply not in the accepted set. The fix puts the accepted
reasons in one shared constant (`NORMAL_STOP_REASONS`) used by both the loop and
the manifest. E09c, the same live job on the fixed tree, satisfies the full acceptance
chain: `supervisor status: accepted`, headless `status: complete`,
`stop_reason: spread-consensus`, `normal_completion: true`, applied `M669` present and a
finalized recording with `autocal_complete: true`
([pair evidence](evidence/E09-live-hp3-pair.json)).

## Tests run

- `autocal/tests`: 374 passed, 3 failed, 2 deselected. The three failures are
  pre-existing missing-fixture errors in `test_filter_pass_spool_fit.py` and
  `test_legacy_zero_buildup.py` (`autocal/auto-calibration-simulation-for-hangprinter/util.py`
  is absent); they are unrelated to this change.
- `autocal/tests/test_history_selection.py`, `autocal/tests/test_headless_sim.py`,
  `tests/python/test_autocal_jobs.py`: 73 passed, 7 deselected. Added coverage:
  the spread predicate (scale-relative limit, degenerate rejection, tighten/loosen,
  single-fit caveat) and "every automatic stop reason is a normal completion".
- Fixed integration suites via MCP (`headless-autocal`, `research-services`) and the
  live HP3 jobs below.

## Limitations

- This is a stopping rule, not a better estimator: it saves collection time and
  cannot make a converged run more accurate. It can freeze a marginally worse
  candidate than patience would have reached later (worst observed 0.221 mm
  offline, 0.211 mm on the harness, 0.18-0.21 mm on the reference and live HP3
  runs).
- Two distinct fits can agree while both are wrong when the data is uninformative
  (`s10_supervised` reaches 449 mm and the rule still fires). The rule does not
  detect that failure; it just stops a little earlier on a run that was already
  failing.
- All evidence is simulation. Hardware transfer is untested, and the live
  comparisons are single trials, not paired speed benchmarks.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A candidate-fit *spread* stop is safe and saves sweeps | E05, E06, E08, E11, E12, E13, E09b | Supported (shipped) | [E13](evidence/E13-converged-regret.json), [E11](evidence/E11-regression-harness-ab.json), [E12](evidence/E12-reference-003-ab.json) |
| `n_min = 4` and `k = 2` are needed for safety | E05, E06 | Supported | [E06](evidence/E06-stop-regret-grid.json) |
| `n_min = 3` also works | E06 | Rejected (spread blows to 970 mm) | [E05](evidence/E05-spread-deck.json) |
| `frac = 3e-3` is safe | E06 | Rejected (max regret 2.721 mm) | [E06](evidence/E06-stop-regret-grid.json) |
| Consensus-delta stability is a better stop | E07 | Rejected (fires on a 5.11 mm wrong plateau) | [E07](evidence/E07-rule-family-compare.json) |
| The new stop reason is compatible with the headless job lifecycle | E09b, E09c | Rejected as implemented, fixed, then supported | [E09 pair](evidence/E09-live-hp3-pair.json) |
| Score-based selection can be replaced by a better single pick | prior sessions | Rejected (not repeated) | see references |
| Information-driven sweep *placement* (not stopping) | none | Not tested | - |

## References

- [Research notebook](research.md), [abstract](abstract.md), [original task](prompt.txt).
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003` (main checkout),
  documented in [extended-reference.md](../../../../autocal/data/references/extended-reference.md).
- Recording-workload costs used to pick `events` for the live jobs:
  [autocal-recording-performance.md](../../../../research/autocal-recording-performance.md).
- Prior session `2026-10-10-...-51aa157a` report: default-force HP3 acceptance at
  iteration 7 / 360 points / 2.604 mm, and evidence that rank score disagrees with
  geometry; its [tracked review](../../../../research/autocal-settling-review.md).
- Prior session `2026-10-13-...-b65aa69c`: rejection of the stability-based stop
  and of single-pick selection (E37-E39 lineage behind E07 here).
- Prior session `2026-10-12-...-2f03cb59`: centered encoder averaging during settling.
- Session `autocal-ranking-bounded-updates-negative-results-b7deb1bc`: bounded-update
  and ranking negative results, referenced so they are not retried.
- This session's evidence: [E05](evidence/E05-spread-deck.json),
  [E06](evidence/E06-stop-regret-grid.json), [E07](evidence/E07-rule-family-compare.json),
  [E08](evidence/E08-regression-decks.json), [E11](evidence/E11-regression-harness-ab.json),
  [E12](evidence/E12-reference-003-ab.json), [E13](evidence/E13-converged-regret.json),
  live HP3 pair [E09b/E09c](evidence/E09-live-hp3-pair.json)
  (artifacts under [E09b](native/native/autocal/E09b-spread-hp3-live/) and
  [E09c](native/autocal/E09c-spread-hp3-live-fixed/)).
