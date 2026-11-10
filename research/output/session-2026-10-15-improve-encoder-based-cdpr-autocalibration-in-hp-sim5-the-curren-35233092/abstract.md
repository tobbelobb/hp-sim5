# Faster HP3 encoder-only autocalibration: a consensus spread gate and a compliance-aware fixed-anchor tolerance

Status: complete for the measured hypotheses (E01-E14). Both source changes
shipped; a third candidate (a `trf` solver default) was measured and rejected.
All evidence is simulated.

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
sweep sets (24.05% less, 1.35 s per point), a third collection reproduced the
patched arm bit-for-bit, and a second arm pair at 3 sweeps reproduced the saving
(323.984 s vs 245.432 s, 24.25%). Two further measurements bound the risk of the
tolerance change without settling it: the 9-sweep replay fit is itself
ill-conditioned, moving 0.14-1.32 mm when a single point's length is nudged by
0.001 mm, so the tolerance change's data difference is bounded to a few
millimetres rather than proven neutral; and switching the default solver to
trust-region-reflective removes the replay's rare 19-34 mm flip but costs 59-202
mm on three recorded-corpus datasets, so it was rejected as a default. Limitation:
all evidence is simulated; the gate's gain is corpus-dependent and mixed on the
stop chain, and the end-to-end anchor-quality equivalence of the tolerance change
is not settled at the reference's sweep width.

## Topics

autocalibration, encoder-only calibration, Hangprinter HP3, CDPR, candidate
ranking, held-out sweep ordering, consensus averaging, fixed-anchor compliance,
collection time budget, slack-line pullout, deterministic simulation replay

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Averaging the closest candidate pair is unsafe when that pair is wide, so a spread gate improves the applied anchors | E01, E02, E03, E04 | Mixed (corpus-supported, stop-chain neutral-to-slightly-negative) | [E02/E03](artifacts/scripts/verify_gate.py), [E03](artifacts/scripts/reg-gate.txt), [E04](report.md#e04-stop-chain-replay) |
| The 0.01 mm fixed-anchor guard only chases a compliance residual, so raising it to 0.05 mm removes overhead without changing the machine state | E06 | Supported | [E06](report.md#e06-the-correction-moves-nothing) |
| Raising the tolerance cuts collection time for an identical sweep set | E07, E08 | Supported | [E07](artifacts/ab-matched/), [E08](report.md#e08-determinism-control) |
| Time saving reproduces at a second sweep count | E11 | Supported (time only) | [E11](artifacts/e11/e11_summary.json) |
| The same-sweep data still fits to the same anchors end-to-end | E11 | Not settled (sweep 2 unmatched, k=3 unidentifiable) | [E11](report.md#e11---end-to-end-ab-on-the-same-3-sweeps-time-only) |
| Fewer points per sweep gives equivalent accuracy | E09 | Rejected | [E09](artifacts/points_study.json) |
| Fewer sweeps give equivalent accuracy | E10 | Inconclusive | [E10](artifacts/sweep_count_study.json) |
| `sigma_floor` or more solver restarts is a usable accuracy lever | E12 | Rejected | [E12](artifacts/sigma_floor_study.json) |
| The shipped fit is robust to the per-point length differences the tolerance change introduces | E13b, E14 | Rejected: the 9-sweep fit is knife-edge (0.001 mm -> up to 1.32 mm) | [E13b](artifacts/sensitivity-perpoint/), [E14](artifacts/solver-study/) |
| `trf` is a better default solver than `lbfgsb` | E14 | Rejected as default; better on replay, 59-202 mm worse on 3 corpus datasets | [E14](report.md#e14---solver-choice-better-on-the-replay-rejected-on-the-corpus) |
| Shortening the slack-line pullout waits is the next available speedup | E11-proposed | Not tested | [E05](report.md#e05-where-the-time-goes) |

## References

- [Full report](report.md): analysis, measurements and limitations.
- [Research notebook](research.md): experiment decisions and accepted steering.
- [Original task](prompt.txt).
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003`
  (local evidence; the comparison target named in the task).
- [Extended autocal reference data](autocal/data/references/extended-reference.md):
  how to make full recorded collection runs.
- [Recording performance budgets](research/autocal-recording-performance.md):
  matched HP3 replay cost of `none`/`events`/`sampled`/`full` recording.
- [Research catalog](research/catalog.md): index of prior sessions.
- [Research index tooling](scripts/research_index.py).
