# Improve encoder-based CDPR autocalibration in hp-sim5

Session: `2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092`
Worktree: `improve-encoder-based-cdpr-autocalibrati-af9af5fc` (branch
`research/improve-encoder-based-cdpr-autocalibrati-af9af5fc`, base commit `72523b9d`).
No commits and no new branches were created.
Reference to beat: `output/extended-autocal/no-buildup-reference-003`
(9 sweeps, 180 points, 2 sub-sweeps of 10 points each, fit quality 0.1928,
9.677 mm summed anchor error under the HP3 replay harness).

## Objective

Make encoder-only HP3 autocalibration faster, more robust, and better at picking
anchors, without giving up the one-click flow. Two concrete claims to attack:

1. The candidate-selection score disagrees with true geometrical accuracy, so a
   run can select worse anchors than the data supports.
2. Data collection is slow, and a forced re-collection would ruin the user
   experience, so the per-point cost should come down while the collected data
   stays equivalent.

The second claim was named as the primary user-facing concern, so an
*end-to-end* claim was required, not only a code-level one: the same sweep set
must still fit to the same quality.

## Constraints (accepted steering)

- HP3, line layering disabled, base radii 30 mm, no buildup
  (`--find-radii off --find-buildup-factor off`). No spool-radius search.
- Keep the one-click flow: no new required CLI flag, no human acceptance step,
  automatic stopping preserved.
- Source changes stay small and shallow.
- Prefer offline replays of stored datasets over new collection where possible.
  Simulated evidence is labelled as simulated and is never presented as
  hardware accuracy.

## World and tool provenance

The launcher world was reconfigured through MCP to HP3 with layering disabled:
`scene=public/usd_scenes/hp3_rigid_body.usda`, `firmware_config=sys/config_hp3.g`,
`layering=false`, `base_radii_mm=[30,30,30,30]`, `buildup_factor=0`, backend
`headless-js` (native Rust cable solver,
`physics_artifact.sha256=ff44a93e57d9594024b2905dd84a88332fa9cd4d7db7c6c3471bf80ac869ba61`,
`bridge_source_sha256` differs per arm because the collector source is part of it).
True HP3 anchors used for scoring:
`[[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900]]`
from `autocal/tools/regress_calibration_logs.py`.

Collector options for every collection in this session:
`configs` cycling `[0,3]/1/2`, `[1,3]/0/2`, `[2,3]/0/1`;
`options={returnToOrigin:true, sweepPoints:10}` (10 points per direction = 20 per
sweep); `forceLow/Mid/Max=0.01/0.05/0.5 N`, `sensorCollectionForce=1`,
`noiseSampleCount=16`, `noiseSigmaFloorDeg=0.02`, `settleMs=200`.

Tool checks (successful tool checks, **not** calibration accuracy):

- `research_preflight` passed on the attached supervisor with the expected
  complete HP3 world configuration before live collection
  (`native/preflight/c87ab2ffdd8542a99c37af4202dce708/preflight.json`).
- `.venv/bin/python scripts/research_socket_probe.py` passed in the agent command
  context (`outcome: passed`).
- Collector unit tests pass: `npx jest tests/js/sweep_data_collection.test.js`
  -> 6 passed. History-selection unit tests pass:
  `.venv/bin/python -m pytest autocal/tests/test_history_selection.py` -> 9 passed.

Provenance note: the collector stamps `collector_options.machineType` as
`hangprinter_4` by default even for an HP3 run (visible in every manifest here and
in `reference-003/sweeps.json`), while `machine_design` and the true-anchor match
say HP3. The dataset `machine_type` field is therefore not evidence about the
physics; the manifest `machine_design` and world settings are.

## Hypotheses

| ID | Hypothesis | Status |
| --- | --- | --- |
| H1 | `consensus_anchor_estimate` averaged the closest candidate pair regardless of separation; when that pair is wide the mean sits outside both members and can be worse than either fit. A spread gate should refuse it and keep the best-ranked fit. | implemented; supported on the 11-dataset corpus and the recorded regression, mixed on the stop chain (E01-E04) |
| H2 | `enforceFixedAnchors` treats a compliance residual as a positioning error. With tolerance 0.01 mm it fires at every point, issues a no-op correction and pays a full encoder settle. Raising it to 0.05 mm removes pure overhead. | implemented; time saving reproduced twice (24.05% and 24.25%), anchor-quality equivalence not settled (E05-E08, E11) |
| H3 | Fewer points per sweep, or fewer sweeps, could give equivalent accuracy and therefore less collection. | rejected / inconclusive (E09, E10) |
| H4 | The slack-line pullout wait dominates sweep time and could be shortened. | proposed, not tested (E16); E11 measured no pullout change |
| H5 | The fit's `sigma_floor` or an increased restart count is a usable accuracy lever. | rejected (E12) |
| H6 | Trust-region-reflective (`trf`) is a better default solver than `lbfgsb`: it avoids the replay's rare 19-34 mm flip and fits faster. | rejected as a default (E14); proposed as an extra in-pool candidate (E17) |

Proposed-but-untested work is not a research result; H4 is listed so the next
session does not have to rediscover it.

## Experiment IDs

| ID | Experiment | Status |
| --- | --- | --- |
| E01 | Selection-score vs geometry disagreement: per-candidate cost matrix over 11 stored datasets | done |
| E02 | Spread-gate replay over the same 11 datasets, shipped code, gate sweep | done, supported |
| E03 | Recorded-corpus regression replay, HEAD vs gated | done, +11.62 mm |
| E04 | Stop-chain replay over reference-003 prefixes N4..N9, HEAD vs gated | done, mixed |
| E05 | Reference-003 time-budget profile (pullout, per-point settle, travel) | done |
| E06 | Offline proof that the fixed-anchor correction does not move the error | done, supported |
| E07 | Matched live HP3 A/B, 2 sweeps x 10 points, tolerance 0.01 vs 0.05 | done, 24.05% less sim time |
| E08 | Determinism control: third collection with the patched collector | done, bit-identical to E07 arm B |
| E09 | Point-thinning study on stored data (9 sweeps, m points per sub-sweep) | done, rejected |
| E10 | Sweep-count study on stored data (first k sweeps) | done, inconclusive |
| E11 | End-to-end larger A/B, tolerance 0.01 vs 0.05, then fit both | done; time saving reproduced (24.25%), quality unusable (sweep 2 unmatched, k=3 unidentifiable) |
| E12 | `sigma_floor` / restart-count study on stored data | done, rejected |
| E13 | Sensitivity replay perturbing `raw_angles_deg` | void: the `--dataset` path does not consume that field (superseded by E13b) |
| E13b | Per-point perturbation of `l_drive`/`l_sensor` on the 9-sweep replay | done: the fit is knife-edge against sub-quantum length noise (0.14-1.32 mm at +-0.001 mm) |
| E14 | Solver study, `lbfgsb` vs `trf` vs `lm` on replay + perturbed copies + recorded corpus | done: `trf` better/faster on replay but rejected as default (3 corpus datasets 59-202 mm worse) |

## Budgets used

- Movement: live HP3 collection on the attached supervisor only (no hardware).
  Nine collection jobs: two 2-sweep A/B arms, one repeated arm, two earlier
  partial attempts from the prior turn, two 3-sweep end-to-end arms (E11), and
  two further attempts whose manifests are kept under `artifacts/e11/`. The
  matched 2-sweep arms cost 170-224 s of simulated time each (~40-76 s wall);
  the 3-sweep arms cost 245-324 s of simulated time (68-106 s wall).
- Compute: offline replays of 11 stored datasets plus reference-003 stage
  replays, the E13b per-point perturbation sweep (about 40 replay fits) and the
  E14 solver study (18 replay fits across three solvers); no multi-hour browser
  full-auto run was started.
- Recording: `events`-level collector artifacts for native collection;
  numerical-only replays for the large offline matrix work, per
  `research/autocal-recording-performance.md`.

## Accepted steering

- Use the recorded reference run as the comparison target; do not start a fresh
  multi-hour browser full-auto run.
- Do not re-test ideas already disproved by earlier sessions: unconditional
  half-point collection, the common-fold rule, newest-iteration selection,
  age-score removal, bounded updates.
- Prefer changes that are checkable offline against the 11-dataset corpus.

## Next decision

Both changes ship: H1 on the corpus gain (+11.62 mm) with its stop-chain result
reported as mixed, and H2 on the twice-reproduced time saving (24.05% / 24.25%,
about 1.31-1.35 s per point in these narrow arms). H2's *anchor-quality*
equivalence is **not** settled: E11's third sweep is unmatched and k = 3 is not
identifiable, and E13b shows the changed data sits inside the fit's own
ill-conditioning, so the change is bounded to a few millimetres rather than
proven neutral. E14 rejected a `trf` default. The next session should close the
quality question with a matched k >= 6 A/B (E15) and attack the pullout wait
(E16, 39% of sweep time), then consider `trf`-in-pool (E17) and re-deriving the
5 mm gate on a larger corpus (E18) rather than re-deriving H1-H3. See `report.md`
for outcomes and limitations.
