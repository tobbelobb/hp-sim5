# HP3 autocalibration at fresh centered and asymmetric encoder references

Status: completed bounded study through E17; no production solver or stopping change accepted.

## Abstract

Does moving the encoder reference near the anchors' centroid improve HP3 calibration or reduce collection effort? A retrospective fixed-offset rebase worsened 33/36 paired fits, but a causal audit showed changed exact-reference/noise assumptions and origin-dependent seeds/bounds; those failures cannot establish an inherent advantage of the original origin. Removing or falsifying stored M669 anchors gave bit-identical fits, with no direct truth leakage found. We then collected fresh native-Rust HP3 datasets at the original reference, at the exact virtual-anchor centroid (+265 mm vertically), and at an asymmetric reference, using identical manual forces, known 30 mm radii, zero buildup, no layering and normal automatic stopping. Final anchor RMSE was 4.588/4.277/12.602 mm using 140/180/300 points and 1.051/1.131/2.072 million physics steps. Centering slightly improved final accuracy but increased collection effort. A larger asymmetric start failed initial encoder settling. Sampled recordings show small measurement-time rotation for symmetric references but up to 2.11° for the asymmetric run. Limitations: single conditions on one nominal layout, deterministic simulation, changed physical trajectories, and no demonstrated fewer-point or hardware gain.

## Topics

HP3, CDPR, encoder autocalibration, fresh centered reference, origin shift, anchor scale, solver conditioning, symmetric initialization, geometry prerequisites, ground truth leakage audit, warm start, automatic stopping, settling, Rerun, fewer data points

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Low fitting scores reliably identify accurate geometry | E01 | Rejected: seven material adjacent disagreements | [Baseline](E01/results.json), [summary](summary.json) |
| Exact centroid translation uniformly improves local conditioning | E02-bootstrap, E02-corrected | Mixed: modest bootstrap improvement, worse at five sweeps | [Bootstrap](E02-bootstrap/results.json), [five sweeps](E02-corrected/results.json) |
| The implemented nearest-center rebase improves recovery | E03–E06 | Rejected for this workflow; general coordinate-invariant relocation not tested | [Corrected matrix](E05/results.json), [CLI replay](E06/results.json) |
| A feasible asymmetric retrospective reference improves recovery with less data | E05–E06 | Mixed at three sweeps; rejected as a robust policy | [Corrected matrix](E05/results.json), [CLI replay](E06/results.json) |
| Rebased fits are useful seeds after restoring the original objective | E07 | Rejected as a reliable improvement | [Polish](E07/results.json) |
| Reference transformation preserves residuals at one exactly translated geometry and supports ground-truth-free inversion | E02, E08, unit tests | Supported locally; this does not establish equivalent objective families/noise | [CLI checks](E08-cli/checks.log), [tests](tests.log) |
| Shifting reduces catastrophic intermediate failures under default L-BFGS-B | E09 | Rejected for retrospective workflow: chosen fits >100 mm, original 0/17, center 7/17, edge 5/17 | [Reliability audit](E09-reliability.json) |
| Stored M669 ground truth is used by the tested fit path | E10 | Rejected: deletion/falsification produce bit-identical fits | [Controls](E10-causal-audit/results.json) |
| The fixed-offset rebase is a statistically equivalent coordinate change | E11 | Rejected | [Reference residuals](E11-reference-assumptions/results.json) |
| Correctly translating the bootstrap estimate can prevent shifted warm-chain catastrophes | E12 | Supported for one diagnostic chain; no full-auto policy claim | [Warm chain](E12-warm-chain/results.json) |
| Fresh centered-origin collection improves accuracy with less effort | E13–E15 | Mixed: 4.588 -> 4.277 mm, but 140 -> 180 points | [Fresh results](fresh-results.json) |
| TRF resolves the new cold bootstrap failures | E15 | Supported for these two recorded three-sweep prefixes; not a default-policy validation | [Fresh fits](E15-fresh-fits/results.json) |
| A larger asymmetric start yields usable calibration data | E16 | Inconclusive for accuracy: initial settling timed out before measurements | [Failed trial](E16-offset/sweeps.full_auto.log) |
| The smaller asymmetric start improves collection efficiency | E17 | Rejected in this trial: 12.602 mm, 300 points; rotation complicates interpretation | [Fresh results](fresh-results.json), [rotation](sampled-rotation.json) |

## References

- [Full report](report.md): methods, corrected interpretation, measurements and limitations.
- [Research notebook](research.md): decisions, budgets, accepted steering and failed pilots.
- [Fresh dataset/plant/result summary](fresh-results.json): E13–E17 manifests, stage fits and metrics.
- [Fresh source and input audit](fresh-manifest.json): exact scripts, scene/firmware inputs, native artifact and readiness checks.
- [Fresh baseline dataset](E14-baseline/sweeps.json), [centered dataset](E14-center/sweeps.json), [asymmetric dataset](E17-offset-small/sweeps.json): newly collected production encoder/noise records.
- [Causal controls](E10-causal-audit/results.json), [reference-assumption audit](E11-reference-assumptions/results.json) and [warm-chain audit](E12-warm-chain/results.json): qualify earlier retrospective results.
- [Median and reliability audit](E09-reliability.json): process completion, automatic stopping, cold/warm fits and chosen iterations counted separately.
- [Offline revision and source hashes](manifest.json): initial experiment provenance; original inputs verified unchanged.
- [Original transform snapshot](source/autocal/tools/rebase_dataset.py): numerical transform used in the earlier trials; its later documentation explicitly warns about changed reference assumptions.
- [Supplied HP3 reference](../../../extended-autocal/no-buildup-reference-003/sweeps.json): REF source observations.
- [Varied-world study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md) and [world matrix](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/environment-matrix-20261015-retry1/matrix_summary.json): W00/W08 source simulations and evaluation geometry.
- [Warm-start study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md): prior sparse-data and initialization failures motivating matched seeds.
