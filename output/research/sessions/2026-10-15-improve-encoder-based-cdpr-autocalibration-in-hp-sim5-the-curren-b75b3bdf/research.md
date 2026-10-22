# Research record: fewer, faster calibration sweeps for encoder-based CDPR autocal

Objective (from [prompt.txt](prompt.txt)): improve encoder-based CDPR autocalibration in
hp-sim5. The current full-auto algorithm can be slow, fragile, or choose worse
anchors because its selection score disagrees with actual geometric accuracy.
Reproduce and measure those cases first, then improve speed, robustness and/or
user experience -- especially results that need fewer or faster-to-collect data
points. Preserve the one-click experience and the automatic stop when finished.

## Constraints and budgets

- Interpreter `.venv/bin/python`; small, shallow source changes; no commits or branches.
- HP3 (`hangprinter_3`), Line Layering **disabled**, `--buildup-factor 0`, `--find-radii off`,
  `--find-buildup-factor off`. No spool-radius search.
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003` in the
  main checkout (9 sweeps, accepted at iteration 7).
- Authoritative prior baseline for the same HP3 no-layering workflow: the E12 acceptance
  run in session `2026-10-10-...-51aa157a` (7 iterations, 360 points, 2.604 mm).
- Recording budget: `none` for offline/large sweeps, `events` for live runs; no
  physics-inspection recording was needed for the accepted hypothesis.
- Movement/compute: offline replays are cheap (minutes, no physics). Live HP3 jobs cost
  ~6 min wall / ~2.7 M physics steps each; two live jobs were run (E09b, E09c) plus the
  two fixed integration suites.
- Metric: applied-anchor RMSE against the nominal HP3 layout,
  `sqrt(mean_i |a_i - a_i_true|^2)` over the four anchors, in mm. Truth is never an input
  to the algorithm; it is only used to score results.

## Current hypothesis (leading, shipped)

The full-auto loop already selects a good applied geometry with the shipped bracketed
consensus of the two most consistent candidate fits, but it only *stops* on patience
(3 non-improving iterations) or an absolute cost/std threshold. The ranking scalar it
waits on disagrees with geometry, so the loop keeps collecting sweeps after the applied
geometry has converged. **Hypothesis: stop when the two closest distinct consensus fits
agree to a small fraction of the machine's own extent.** The spread between distinct fits
carries the geometric signal the scalar loses, and a fractional limit transfers between
machines. Expected benefit: fewer sweeps per run at unchanged or better geometry.

Accepted steering: default-on, automatic, env-var disable for A/B
(`HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC=off`); no new user-facing setting.

## Experiment log

| ID | Question | Method | Outcome |
| --- | --- | --- | --- |
| E01 | Does the score agree with geometry per iteration? | Audit recorded stage fits vs truth | Score and geometry disagree (confirms the task premise) |
| E03 | Which stop rules are cheap and safe? | First offline stop-rule pass | Superseded by E05/E06 |
| E04 | Build rule-independent candidate decks | `--no-collect --patience 100` replay of 13 recorded runs | Decks in `/tmp/rp/<name>/stages` |
| E05 | Is a spread rule viable at all? | Offline decks, sweep/regret curves | Viable; frac/n_min/k matter |
| E06 | Which `(frac, n_min, k)` is safe? | 96-point grid, regret vs reachable best | `1.5e-3 / n_min 4 / k 2` selected |
| E07 | Is the consensus-delta stability family better? | Rule-family comparison on the same decks | **Rejected**: unsafe and redundant |
| E08 | Regression datasets as decks | Per-dataset decks, rule disabled | Corpus support for E06 |
| E11 | Does the rule help on the shipped regression harness? | Identical-CLI A/B, 11 datasets | 6 sweeps saved, aggregate RMSE −0.636 mm |
| E12 | Does the rule help on reference-003? | Identical-CLI A/B on the recorded deck | 9→7 sweeps, RMSE 2.623→2.805 mm |
| E13 | Converged-deck regret vs the shipped stop | Reuse E05 decks, compare to patience-3 | 23 sweeps saved over 8 decks, mean regret 0.051 mm |
| E09b | Live HP3 end-to-end with the rule | Full-autocal MCP job, default forces | Fires at iteration 6; **exposed a `normal_completion` regression** |
| E09c | Live HP3 end-to-end after the fix | Same job on the fixed tree | Automatic stop reported as accepted |

## What was measured

Reference replay (E12, identical CLI, only the env var differs): 9 sweeps / iteration 7 /
2.623 mm before, 7 sweeps / iteration 5 / 2.805 mm after. The early stop spends 2 fewer
sweeps (22% of collection) and gives up 0.18 mm.

Shipped regression harness (E11, 11 datasets, `--no-collect` replays): 6 sweeps saved in
total; `layered_manual_some_bad_points` 6→4 sweeps and 1.758→0.911 mm (better);
`ten_points_w_buildup2` 6→4 sweeps and 1.439→1.650 mm (worse by 0.211 mm);
`five_points_bigger_deltas` and `ten_points_bigger_deltas` 5→4 sweeps with identical RMSE;
the rest unchanged.

Offline converged decks (E13): 23 sweeps saved across 8 decks (2.88 per deck), mean regret
0.051 mm, worst 0.221 mm, 6 of 8 identical. On the 5 decks whose fits never converge the
spread predicate does not fire before patience, so those runs stop exactly where they did
before (one exception: `s10_supervised`, whose consensus is stable but 449 mm wrong --
see limitations).

Live HP3 end-to-end (E09b/E09c, default forces, no layering): the loop stopped itself at
iteration 6 with the message
`; full-auto: stopping early; the two closest distinct fits agree to 1.679mm (limit 4.944mm).`
320 points / 8 sweeps / 2.814 mm / 5,332 simulated seconds, versus the comparable prior
baseline of 360 points / 9 sweeps / 2.604 mm / 6,011 simulated seconds: about 11% less
simulated collection time for +0.21 mm.

**Robustness finding (fixed in this session).** That same live job revealed that the new
`stop_reason` was not in the headless manifest's accepted set: `headless_sim` required
`stop_reason == "patience-or-threshold"` for `normal_completion`, so a successful
spread-consensus stop was reported to the supervisor as a **failed** job even though the
loop had applied its parameters and finalized its recording
([E09b job.json](native/native/autocal/E09b-spread-hp3-live/job.json)). The shipped fix
moves the accepted reasons into one shared constant and E09c re-runs the same live job to
show `status: accepted`.

## Disproved or rejected hypotheses

- **Consensus-delta stability as the stop criterion** (E07): stopping when the running
  estimate stops moving fires on the still-wrong 5.11 mm plateau (mean regret 0.668 mm,
  max 2.721 mm). Rejected; the distinct-fit spread is strictly safer.
- **`n_min = 3`** (E06): fires at 3 candidates, after which the spread blows up to 970 mm
  on `headless_ext_verif`. Rejected.
- **`frac = 3e-3`** (E06): two runs stop on a wrong plateau (max regret 2.721 mm). Rejected.
- **`k = 1`** (E06): safe but weaker than `k = 2`; requiring two consecutive agreeing
  iterations costs nothing and rejects one-off coincidences.
- Prior sessions already rejected: rank replacement, ensemble means / rank bands /
  centrality-k (blow up to 291-907 mm), single-pick stable-mean/medoid (cannot beat the
  bracketed pair), point-density reduction by default, and relaxing settle tolerance or
  the vibration window. Not repeated here.

## Open questions / limitations

- The winner is a *stopping* rule, not a better estimator: it saves collection time and
  cannot make a converged run more accurate. Occasionally it freezes a slightly worse
  candidate than patience would have reached later (worst observed 0.221 mm offline,
  0.211 mm on the harness, ~0.18-0.21 mm on the two reference/HP3 comparisons).
- Two distinct fits can agree while both are wrong when the collected data is
  uninformative (`s10_supervised`, 449 mm final error). The rule then stops earlier
  without making that already-failed run much worse, but it does not detect the failure.
- Hardware transfer is untested; every result here is simulation evidence.

## Next decision

Ship the spread stop plus the `normal_completion` fix, and record the residual regret.
Next candidate directions (not started): information-driven sweep *placement* so the
first four sweeps already bracket the anchors, and a stop that also requires the applied
geometry to be well conditioned.
