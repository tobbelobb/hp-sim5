# Consensus-spread early stopping: fewer HP3 autocalibration sweeps without losing geometry

Status: complete. Simulation evidence; one live HP3 confirmation run reported below.

## Abstract

hp-sim5's full-auto encoder autocalibration keeps collecting sweeps after the
applied geometry has converged, because it stops on patience or an absolute
cost/std threshold whose ranking scalar disagrees with geometrical accuracy. We
tested an automatic early stop that fires when the shipped bracketed-consensus
estimate has stabilised geometrically: the two mutually most consistent *distinct*
candidate fits must agree to within `1.5e-3` of the machine's own anchor extent for
two consecutive iterations, from at least four candidates. Parameters were
selected on rule-independent replay decks from 13 recorded runs. On the recorded
reference-003 deck the stop saves 2 of 9 sweeps for +0.18 mm applied-anchor RMSE;
on the shipped 11-dataset regression harness it saves 6 sweeps with aggregate RMSE
improving 0.64 mm; across 8 converged offline decks it saves 23 sweeps with mean
regret 0.051 mm (worst 0.221 mm). A live HP3 job stopped at iteration 6 instead of
7 (about 11% less simulated collection time) for +0.21 mm. The same job exposed,
and this session fixed, a `normal_completion` regression that had reported the
successful early stop as a failed job; the fixed re-run was accepted and reproduced
the physics and geometry exactly. Limit: a stopping rule cannot improve a converged
estimate, and two wrong fits can still agree on uninformative data.

## Topics

autocalibration, encoder-only, HP3 hangprinter, full-auto early stopping,
consensus spread, stopping rule, candidate ranking, sweep budget,
regression matrix, simulated calibration

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A distinct-fit spread stop is safe and saves sweeps | E05, E06, E08, E11, E12, E13, E09b | Supported (shipped) | [E13](evidence/E13-converged-regret.json), [E11](evidence/E11-regression-harness-ab.json), [E12](evidence/E12-reference-003-ab.json) |
| `n_min = 4` and `k = 2` are needed for safety | E05, E06 | Supported | [E06](evidence/E06-stop-regret-grid.json) |
| `n_min = 3` also works | E06 | Rejected | [E05](evidence/E05-spread-deck.json) |
| `frac = 3e-3` is safe | E06 | Rejected | [E06](evidence/E06-stop-regret-grid.json) |
| Consensus-delta stability is a better stop | E07 | Rejected | [E07](evidence/E07-rule-family-compare.json) |
| The new stop reason is compatible with the headless job lifecycle | E09b, E09c | Rejected as implemented, fixed, then supported | [E09 pair](evidence/E09-live-hp3-pair.json) |
| Better sweep *placement* rather than stopping | none | Not tested | - |

## References

- [Full report](report.md): method, measurements, tests and limitations.
- [Research notebook](research.md): experiment log, budgets and accepted steering.
- [Original task](prompt.txt).
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003`
  (main checkout); see [extended-reference.md](../../../../autocal/data/references/extended-reference.md).
- Recording-workload costs used to choose `events` recording:
  [autocal-recording-performance.md](../../../../research/autocal-recording-performance.md).
- Prior session `2026-10-10-...-51aa157a` (default-force HP3 acceptance at
  iteration 7, 360 points, 2.604 mm; score-vs-geometry audit) and its tracked
  [review](../../../../research/autocal-settling-review.md).
- Prior session `2026-10-13-...-b65aa69c` (stability-stop and single-pick rejections).
- Prior session `2026-10-12-...-2f03cb59` (centered encoder averaging).
- Session `autocal-ranking-bounded-updates-negative-results-b7deb1bc` (ranking negatives).
