# HP4 autocal D.z bias: effector roll and slack parallel lines

Interactive investigation, 2026-11-18. Simulation evidence only (native Rust physics, RRF host firmware). No hardware data.

## Question

The per-commit autocal benchmark puts nominal HP4 (no buildup) at about 4.6 mm worst-coordinate anchor error. HP3 is at about 1 mm. Every HP4 run keeps D.z about 4.6 mm low. Refits with line stiffness scaled 0.5–2× barely move it. HP4 and HP3 have the same point-anchor geometry, so why is HP4 biased, and how can it reach HP3's accuracy?

## Method

- **Exact replay.** [`scripts/replay_poses.mjs`](evidence/scripts/replay_poses.mjs) replays the benchmark's `sweeps.headless/events.jsonl` through the production headless world. It uses the same controller, `simulation_settings` (closed loop, 20 solver iterations) and step schedule. At every 4-axis encoder read it records the true effector pose (position and quaternion) and every cable joint's force. The replay must use today's `public/usd_scenes/hp4.usda`: `d8987ab6` reinterpreted `xformOpOrder`, so the run's own copied `scene.usda` mis-places wheels under HEAD code. With today's scene, all 16 062 HP4 reads and 15 269 HP3 reads reproduce the recorded encoder angles to 0.000°.
- **Data.** Benchmark run `0623060e2227` for HP4 (256 dataset points, 13 sweeps) and HP3 (179 points). Truth is the manifest's initial M669.
- **Attribution.** The production `fit_pose_least_squares` is run with home offsets on and the default prior. Lengths are substituted with true lengths, measured lengths, or models in between ([`attrib*.py`](evidence/scripts/)). The fitter itself is unchanged. Only `dataset_line_lengths` is replaced.
- **Geometry.** [`geom.py`](evidence/scripts/geom.py) computes strand-by-strand line lengths from the scene for any effector pose. It includes pinholes, effector wheels and the 4-strand D reeving with wraps.

The scripts were run from a scratch directory and contain absolute paths. The pose files are regenerable with the replay command and are not committed:

- HP4 poses: sha256 `26b35bc2…534a`, 35 MB
- HP3 poses: sha256 `3350d3c8…f238`, 13 MB

## Findings

### 1. The pipeline is exact, and HP4 reeving is exact under translation

Fitting true translation-only lengths gives 0.00 mm on both machines. With the effector translated but not rotated, the HP4 scene's ABC loops (anchor top → effector pinhole → two effector wheels → pinhole → anchor bottom) give exactly the M669 point model. Averaged over its three lines, so does the D reeving (≤0.01 mm). The bias is not a wrong pivot in the static geometry.

### 2. The HP4 effector rolls; HP3's does not

- HP4 effector rotation during measurements: median 0.097°, max 0.27°.
- HP3: median 0.003°, max 0.007°.

HP4 roll is linear in horizontal offset, about 0.21°/m about the radial axis (rotvec_x ≈ 0.214·x, rotvec_y ≈ 0.208·y; R² 0.94/0.89).

The driver is the D reeving. Its crossing strand runs between side-by-side wheels 6 mm apart along the axle, and its last strand runs to an attach point offset 6.5 mm. Those two strands make each of the three D lines pivot a few mm sideways, in a pattern rotated 120° per line, so the offsets are chiral. The offsets cancel in the mean, but all three lines share one spool. Off-centre they need up to ~0.5 mm (anchor units) different lengths, and the effector rolls to equalise them.

Geometry alone predicts an equalising roll of 0.20°/m at z = 300 mm with the same axis pattern; the replay shows 0.21°/m. The ABC loops run over free effector wheels and give almost no roll stiffness, because top-plus-bottom strand length is invariant to first order.

### 3. Roll makes one of each ABC line pair go slack

- At 0.99 N per strand, 51–54 % of HP4 poses have one of the two parallel lines on an ABC spool below 0.1 N. In one example, AL carries 0.41 N while AR carries 2.61 N.
- The spool then follows only the taut line. Predicting each axis from its highest-tension line's rotated geometry plus that line's own stretch at nominal EA 20 kN matches the encoders to −0.028 ± 0.012 mm (results: [per-line](evidence/results/hp4_per_line_model.txt)).
- The centre point model plus mean stretch, which is what autocal assumes, errs by +0.10 to +0.17 ± 0.15–0.24 mm on slack pairs. It errs by −0.01 to −0.03 ± 0.09–0.10 mm when both lines are taut.

### 4. Taut-line geometry, not stretch, produces the D.z bias

Refits from [hp4_taut_line_attribution.txt](evidence/results/hp4_taut_line_attribution.txt) and [hp4_attribution.txt](evidence/results/hp4_attribution.txt), on the same 256 points:

| Lengths fed to the production pose fit | worst | D.z |
|---|---:|---:|
| true point model | 0.00 | 0.00 |
| rotated geometry, mean of parallel lines (both taut) | 0.66 | −0.22 |
| rotated geometry, taut line only, **no stretch** | 7.02 | −7.02 |
| taut-line geometry on ABC only, ideal D | 6.04 | −6.04 |
| ideal ABC, taut-line geometry on D | 0.98 | −0.98 |
| measured | 5.91 | −5.91 |
| measured minus stretch predicted from recorded tensions (1×, 0.5×, 2×) | 4.3, 4.8, 3.5 | same |
| measured minus per-axis compliance fitted to the data | 4.25 | −4.25 |

Stretch-model corrections cannot remove the bias, whatever the stiffness. That matches the owner's 0.5–2× observation. No single sweep carries it ([influence](evidence/results/hp4_sweep_influence.txt)). Near-horizontal ABC lines turn ~0.1 mm of length error into mm of effector height, and the near-vertical D anchor follows that height.

HP3 on the same pipeline ([hp3_attribution.txt](evidence/results/hp3_attribution.txt)): measured lengths give D.z −0.28 mm, and its remaining 0.8–1 mm error is a common z shift.

### 5. Intervention: higher measurement strand tension

Taut pairs remove the nonlinearity and stiffen the effector against roll. Full headless HP4 autocal runs (`autocal.py --headless-sim --patience 8`, the benchmark protocol) were run with the equal-strain strand target raised. A temporary environment override set `_COLLECTION_FORCE_MULTIPLE`; the scorer is `benchmark_metrics`:

| HP4 run | strand target | accuracy | common shift z | worst shape error (translation-aligned) | precision | converged | total | sweeps |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| benchmark `0623060e` | 0.99 N | 4.57 | −1.89 | 2.68 | 1.51 | never | 5912 s | 13 |
| ×2 | 1.98 N | **1.52** | −1.49 | 0.80 | 0.79 | 4020 s | 5000 s | 9 |
| ×3 | 2.98 N | 2.22 | −2.20 | 0.61 | 0.69 | 3735 s | 6249 s | 14 (patience) |
| HP3 benchmark, for reference | 1.11 N | 0.96 | −0.93 | 0.32 | 1.23 | 3574 s | 5255 s | 9 |

Raising tension removes most of the HP4-specific shape error (2.68 → 0.6–0.8 mm). What remains is a common downward anchor shift that grows with tension. It is the home-reference stretch offset: ABC lines are slack at the encoder origin but carry T per strand during measurement, so a common ABC length offset T·d/EA reads as height through the ~8.4° line elevation. At 2 N that is about 0.19 mm, or 1.3 mm of height. HP3's 0.96 mm is almost entirely the same shift. ×2 is the better trade-off of the two tested values.

The ×2 target is what `5 × start force / min(N)` gives on HP4, compared with today's `/ min(N·U)`. [`autocal/equal_strain.py`](../../../autocal/equal_strain.py) now uses `min(N)`, which leaves U = 1 machines (HP3, HP5) unchanged by construction. The validation runs of the committed form are listed under *Validation of the code change* below.

## Strongest alternative explanations considered

- **Stretch / stiffness.** Rejected as the main cause (finding 4). Recorded-tension stretch with nominal EA explains the encoders only line by line, after taut-line selection.
- **Solver non-convergence.** The per-line model at nominal EA leaves 0.012 mm scatter, so effective stiffness matches the configuration at 20 iterations.
- **Averaging vs instant pose.** 16-sample means and the single matched read differ by ≤0.006 mm.
- **Effector rotation alone, with both lines taut.** Only 0.66 mm. The slack switching is the amplifier.

## Limitations and what would change the conclusion

- Single deterministic simulations per setting. No repeat spread, no worn-bearing or soft-stiffness variants, no hardware.
- The D-reeving roll is a property of the scene. If the real HP4 line verticalizer is not chiral, or its attach point sits on the other side so the per-line offsets cancel, the real machine would roll less. The scene geometry is not changed here; that is a hardware-fidelity question for the owner.
- HP5 pulley-reduced has the same ingredients (parallel ABC/D pairs, U = 2) and also loses accuracy versus HP5. Whether the same mechanism explains it is only tested by the validation run below.
- The remaining common z shift needs a home-reference protocol, for example zeroing or re-measuring home under measurement tension. Fitting more offsets cannot identify it.

## Validation of the code change

These are the committed code (`strand target = 5 × start force / min(N)`) on top of `7a7893e4`, run with the benchmark protocol (`autocal.py --headless-sim --patience 8`, 4.5 M physics steps). Run logs and manifests are in [`evidence/runs/`](evidence/runs/).

| Machine | strand target | accuracy | common shift z | worst shape error | precision | converged | total | sweeps |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| HP4 benchmark `0623060e` | 0.99 N | 4.57 | −1.89 | 2.68 | 1.51 | never | 5912 s | 13 |
| HP4 with change | 1.98 N | **1.52** | −1.49 | 0.80 | 0.79 | 4020 s | 5000 s | 9 |
| HP5 pulley-reduced benchmark `0623060e` | 1.29 N | 2.57 | −0.49 | 2.08 | 0.63 | 4131 s | 4200 s | 6 |
| HP5 pulley-reduced with change | 2.58 N | **1.34** | −0.15 | 1.19 | 1.41 | 3534 s | 4204 s | 6 |

The HP4 run reproduces the ×2 environment-override run exactly, with identical final anchors. HP3 and HP5 have U = 1, so their target is unchanged by construction. The `autocal/tests` suite passes (499).

HP4 is not yet at HP3's accuracy (1.52 vs 0.96 mm). Its remaining error is mostly the common z shift (−1.49 mm, against −0.93 mm on HP3). The next step is a home reference taken under measurement tension, not more tension. HP5 pulley-reduced precision got worse (0.63 → 1.41 mm) while accuracy improved. That is one run, and it needs repeats before reading anything into it.
