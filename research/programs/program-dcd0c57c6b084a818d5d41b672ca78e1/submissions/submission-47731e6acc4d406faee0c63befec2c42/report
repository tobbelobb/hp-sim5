# Fixed encoder associations and absolute scale in the current calibration model

Status: complete, awaiting separate review.

## Abstract

Three offline experiments tested fixed encoder sensitivity, absolute scale and excitation at source revision `c0f111c310cc562b7122bf606a59c60de758adbb`. Reordering fixed encoder observations across points changed no residual in either NumPy or JAX. Each condition retained 75 synthetic or 60 frozen simulated HP3 residuals per backend. Positive controls changed residuals by 0.16 to 1.13 mm. Scaling anchors, spool radii and the dimensionful buildup coefficient together preserved clean synthetic consistency, with RMS residual below 1e-12 mm across 300 points. Holding buildup or one radius fixed broke that invariance. Diverse fixed-length sweeps improved the weakest observable local direction by 7.51 times, while the common-scale direction remained unobservable. One radius observation with assumed 0.05 mm uncertainty raised local rank from 13 to 14. An initially inactive noise-normalization comparison was preserved and corrected after identifying the production 0.05 mm floor. These results concern source sensitivity and synthetic observation symmetry. Production priors, fixed spool values and noise choices can constrain scale separately. No fitting, physics, collection, hardware accuracy or implemented fix is claimed.

## Scope and evidence

All new files are inside this assigned session. The worker used `.venv/bin/python`, CPU JAX with float64, and no supervisor or simulation action. There were zero fits, collections and physics steps. Each numerical subprocess had a 120-second timeout. The final combined E1/E2/E3 run spent 2.97 seconds inside the experiments and 3.05 seconds after imports including provenance work. Earlier bounded probes are recorded separately.

The producer revision remains `c0f111c310cc562b7122bf606a59c60de758adbb`. Local git verification found `git diff origin/main..HEAD -- autocal` empty, with `origin/main` at `7fc26ba491be99c5632e21839be4b3afca201d85`. Referenced `autocal` files also match that commit byte for byte. Source links below use that verified GitHub commit. [source-links.json](source-links.json) records the comparison, paths, URLs and hashes. Prior audit links are context and are not required new evidence.

The current run is authoritative in [manifest-all.json](manifest-all.json), which records the machine, versions, input and script hashes, hashes of every Python file under `autocal`, unchanged source checks, and raw evidence hashes. It ran on AMD EPYC 9V74, Python 3.12.14, NumPy 2.5.4 and JAX 0.11.2, using CPU float64. HEAD matched the assigned revision.

- Script SHA-256: `2e74ef7c22f3aa1d2e740200eb8773753afc937e0d87a44f228bcc3b2478c6f9`.
- Input SHA-256: `99755d9cb59f4b0cde636cd08caccb769cd4b76de0ad5973bebf58bf8e7df4ea`.
- Review data: [summary.json](summary.json). Raw measurements: [E1.json](E1.json), [E2.json.gz](E2.json.gz), [E3.json](E3.json).
- Compression, raw/compressed hashes and recommended archive assets: [manifest-archive.json](manifest-archive.json). Raw JSON originals remain beside the compressed copies.

The deterministic synthetic generator and original compact inputs are in [analysis.py](analysis.py) and [inputs.json](inputs.json). Synthetic anchors in mm are `[[0,-1900,-120],[1645,950,-120],[-1645,950,-120],[0,0,2050]]`. Spool radii are `[39.2,40.1,38.7,39.8]` mm, common buildup is `0.636619` mm, and gearing, mechanical advantage and lines per spool are all one. Encoder origin is zero. Twelve sweeps cover six fixed-anchor pairs at two seed positions. Each has 25 points over a 0.84-radian circle arc. Every inverse spool conversion checks its radicand before conversion, and every round trip must agree within 1e-8 mm. The noisy case adds seeded Gaussian noise of 0.03 degrees to drive/sensor angles only.

Frozen evidence uses the first three sweeps of `autocal/data/references/fifth_hp3_dataset.json`, copied into the inputs with the original file hash. The declared simulation anchors come from the prior audit's reference constants. This is recorded simulated data, not new collection or hardware measurement. The old trial-004/trial-005 files are absent here; their old audit result is context only.

The measured signed residual is the current Sampson residual in squared-length space, in mm squared, divided by `drive_abs + sensor_abs`. It is an approximate geometric residual in mm, not a uniquely predicted encoder or sensor error. Filtering, robust loss, noise-mean substitution and noise normalization are disabled for the main sensitivity measurements. Noise normalization is enabled only in its explicit controls. JAX objectives use the unmodified compiled production function. To inspect JAX residuals, the script truncates an in-memory AST of that same function immediately before noise normalization and returns its existing intermediate arrays. No production source is edited. [jax-probe-provenance.json](jax-probe-provenance.json) records both hashes.

## E1: fixed encoder associations

The baseline adds distinct fixed-angle offsets spanning -0.7 to +0.7 degrees. The tested intervention reverses those values across points, leaving each fixed axis's value distribution unchanged. This deliberately isolates association with drive/sensor observations. Each condition constructs a fresh cost object, so constructor packing cannot hide mutations. Raw-length and modeled-length paths are tested on both datasets.

All six conditions in each dataset/path retained complete finite residuals, valid circle geometry and finite objectives in both backends. Reversing fixed observations changed residual arrays and objectives by exactly zero. Maximum NumPy/JAX residual disagreement across E1 was 1.55e-12 mm.

| Dataset and length path | Finite points per condition/backend | Fixed permutation maximum delta, mm | Active positive control | Maximum residual delta, mm |
| --- | ---: | ---: | --- | ---: |
| Synthetic, direct lengths | 75/75 | 0 | Drive length +0.5 mm | 0.460087 |
| Synthetic, direct lengths | 75/75 | 0 | Aggregate fixed length +0.5 mm | 0.441622 |
| Synthetic, modeled lengths | 75/75 | 0 | Fixed raw angles +1 degree | 1.126494 |
| Synthetic, modeled lengths | 75/75 | 0 | Drive raw angles +1 degree | 0.635190 |
| Frozen HP3, direct lengths | 60/60 | 0 | Drive length +0.5 mm | 0.366938 |
| Frozen HP3, direct lengths | 60/60 | 0 | Aggregate fixed length +0.5 mm | 0.162324 |
| Frozen HP3, modeled lengths | 60/60 | 0 | Fixed raw angles +1 degree | 0.422330 |
| Frozen HP3, modeled lengths | 60/60 | 0 | Drive raw angles +1 degree | 0.534389 |

The modeled path reduces fixed encoder values to one median length per axis per sweep. A uniform fixed-angle shift changes that median by up to 0.686794 mm for synthetic data and 0.683859 mm for frozen data. Permutation preserves the median exactly. Direct length changes are overwritten by encoder-to-length conversion in the modeled path, and raw angle changes do not affect the direct-length path. Those expected inactive controls also returned zero.

The frozen baseline's RMS is 56.1141 mm using direct dataset lengths and 0.899955 mm after the explicit spool transformation. The first number does not imply hardware error; the direct lengths and transformed lengths use different spool assumptions. E1 supports discarded pointwise fixed associations within this isolated current residual path, while showing that aggregate fixed information is active. It does not establish sensitivity of every default filtered/noise-mean configuration.

## E2: observation symmetry and separate scale constraints

For these unit gearing settings, write `D = 180/pi` and `k2 = -buildup`. The current spool equation expands to

```text
L(theta) = radius * theta / D + k2 * theta^2 / (4 * D^2).
```

Both radius and buildup have units of mm here. Multiplying both by `s` multiplies every modeled length by `s` for unchanged encoder angles. Multiplying all anchors and latent effector positions by `s` then preserves all geometric distance equations. This is a symmetry of the observation equations and their exact-consistency set. It does not require an objective or a noise model.

E2 evaluates `s = 0.95, 1, 1.05` on 300 clean and 300 deterministic noisy observations, with all 300 residuals finite in NumPy and JAX for every path.

| Clean-data path | RMS at s=0.95, mm | RMS at s=1.05, mm |
| --- | ---: | ---: |
| Scale anchors, radii and buildup together | 7.76e-13 | 9.40e-13 |
| Scale anchors only | 14.4302 | 13.3608 |
| Scale anchors/radii, hold buildup fixed | 1.94030 | 1.93817 |
| Scale anchors/buildup/other radii, hold radius 0 fixed | 6.34172 | 6.56256 |

The largest modeled-length homogeneity error was below 1.3e-12 mm, including noisy data. The noisy baseline RMS was 0.01878283 mm. Along the joint scale path it became 0.01784369 and 0.01972197 mm, exactly the expected factors 0.95 and 1.05 within 2.9e-12 mm per residual. Thus noisy residual magnitudes and an objective need not stay constant even when observations admit a structural scaling symmetry.

With fixed squared-length normalization, the noisy unnormalized objective scales as `s^4`. With a fixed mm uncertainty and noise normalization, it scales as `s^2`. Co-scaling the uncertainty can make that objective constant. The first attempt to test this latter distinction was inactive because the production 0.05 mm minimum sigma dominated the approximately 0.02057 mm point sigma. Both uncertainty variants returned identical scores. That adverse discriminator is preserved as [E2-initial-floor-limited.json.gz](E2-initial-floor-limited.json.gz), with [analysis-v1.py](analysis-v1.py) and its [original manifest](manifest-E2-initial-floor-limited.json). The original manifest's `E2.json` hash refers to this renamed initial result.

The corrected discriminator explicitly lowers the minimum sigma to 0.001 mm while preserving observations and point sigma. For `s = 0.95, 1, 1.05`, fixed-mm-sigma objectives are `0.9214276784, 1.0209724969, 1.1256221778`. Co-scaled-sigma objectives are all `1.0209724969`, within 1.8e-11. Effective sigma values are recorded, so this control verifies that the intervention actually reaches the objective.

These results reject a universal claim that scaling anchors and radii alone must be invariant under nonzero fixed buildup. Fixed buildup supplies a dimensional constraint. Production also has optional radius priors around nominal base radii, a default buildup prior sigma of 0.1 when buildup is searched, and a radius-similarity penalty with default 2 mm sigma for per-anchor radius fitting. Bounds and fixed low-anchor Z can constrain scale too. These are specified information or regularization, separate from encoder observations. The production scale-polish routine holds buildup fixed, so its objective may vary along its radius/anchor search. This experiment does not claim that the complete production objective is flat, or that its selected scale is accurate.

## E3: excitation and one radius observation

Central differences use steps `1e-4` and `1e-5`, with no optimization. The 14 coordinates are nine reduced anchor coordinates, each in units of 1000 mm, four log-radius changes and one log-common-buildup change. The exact scale direction is `[anchor_vector/1000, 1,1,1,1,1]`. The data Jacobian uses the signed approximate mm residual. Singular values below divide the Jacobian by the square root of its row count so duplicated observations have no artificial excitation advantage.

| Dataset | Residuals | Smallest observable singular value | Scale singular value | Data rank | Rank with radius observation |
| --- | ---: | ---: | ---: | ---: | ---: |
| First three fixed-pair sweeps | 75 | 0.0906903 | 1.34e-8 | 13/14 | 14/14 |
| Same three sweeps duplicated four times | 300 | 0.0906903 | 1.34e-8 | 13/14 | 14/14 |
| Twelve diverse pair/length sweeps | 300 | 0.681158 | 2.52e-8 | 13/14 | 14/14 |

These are the `1e-5` step results. All Jacobian entries were finite. Both step sizes gave the same ranks at relative threshold `1e-7`; all singular values are retained. The direct scale-direction derivative was below 4.4e-8 mm RMS per unit relative scale at the smaller step, consistent with numerical differentiation error. With buildup fixed, all 13 remaining coordinates had nonzero local sensitivity, with smallest singular values 0.083978 and 0.670624 for base and diverse data.

The radius observation is one appended residual, `(radius0 - 39.2 mm)/0.05 mm`, with data residuals divided by an assumed 0.02 mm uncertainty. Its scale sensitivity is 784 standard deviations per unit relative scale. At +/-5% scale it contributes +/-1.96 mm, or +/-39.2 standard deviations, while clean encoder consistency remains below 1e-12 mm RMS. The augmented smallest singular value is 33.36 for base data and 174.83 for diverse data in these specified whitened coordinates. A radius uncertainty is an experimental assumption here, not an actual measurement.

Diverse excitation helps identifiable directions by 7.51 times in this chosen metric; it does not remove this structural scale direction. One independent radius observation does remove that local null direction in the tested exact model. This is a local sensitivity result, not global uniqueness, recovery from poor initial estimates or a hardware accuracy guarantee.

## Tested hypotheses and decision

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Pointwise fixed encoder association is discarded while aggregate fixed length remains active | E1 | Supported | E1 residual arrays and positive controls |
| Free anchors, radii and dimensionful buildup admit a common-scale observation symmetry | E2 | Supported | Clean zero set, noisy homogeneity and dimensional controls |
| Scaling anchors/radii alone is always invariant with fixed nonzero buildup | E2 | Rejected | Fixed-buildup positive control |
| Extra encoder-only fixed-length excitation removes the common-scale direction | E3 | Rejected in tested model | Diverse and duplicate Jacobians |
| One independent radius constraint detects that scale direction | E3 | Supported locally | Augmented Jacobians and +/-5% controls |

The bounded evidence supports the recorded `model_gap` outcome. Retain the distinction between loss of pointwise fixed geometry and absolute scale. More diverse sweeps can help other weak directions, as E3 shows, but they cannot supply a missing physical length standard in this observation model. The cheapest decision-changing follow-up is an offline per-point fixed-length residual prototype plus an audit of which spool/buildup priors are truly measured versus nominal. Neither follow-up was executed here.

No production edits, long fits, new clients, shared builds or dependencies were needed. The shared index was already modified on entry and remains parent-owned. Separate scientific review and broader dynamics/hardware questions remain with the manager.

## Reproduction and archival

Use the supplied inputs directly. The script accepts explicit source/output paths and also discovers the repository from git when `--repo-root` is omitted. It does not infer repository depth from the session location. Use a fresh directory so sealed measurements are preserved:

```bash
TASK_REPO=/workspace/hp-sim5
TASK_EVIDENCE=/absolute/path/to/selected/session
TASK_RERUN=$(mktemp -d /tmp/hp-cal-reproduce.XXXXXX)
cp "$TASK_EVIDENCE/inputs.json" "$TASK_RERUN/inputs.json"
timeout 120s "$TASK_REPO/.venv/bin/python" -B \
  "$TASK_EVIDENCE/analysis.py" all \
  --repo-root "$TASK_REPO" --output "$TASK_RERUN"
```

Use `E1`, `E2` or `E3` instead of `all` for a single experiment. Run `prepare` only in a fresh directory to regenerate deterministic inputs from the reference file. The original floor-limited E2 can be reproduced with `analysis-v1.py E2` in another fresh output directory containing a copy of `inputs.json`. JAX caches are redirected into the specified output, and Python bytecode writes are disabled.

Archive generation reads existing results only. It accepts raw or gzip JSON and can summarize a fresh reproduction directory:

```bash
"$TASK_REPO/.venv/bin/python" -B "$TASK_EVIDENCE/archive.py" \
  --output "$TASK_RERUN"
```

All recommended assets in `manifest-archive.json` are below 1 MiB each and together below 8 MiB. Only the two oversized JSON files require gzip. Gzip uses level 9, timestamp zero and no original filename; decompression and both hashes are verified. Caches and launcher/native files are unnecessary for this offline evidence.

## Sources actually used

- [Current NumPy cost](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/ellipse_cost.py), revision above: sweep packing, finite residuals, squared-length-to-mm conversion and noise floor.
- [Current JAX objective](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/ellipse_objective_jax.py): compiled objective and instrumented existing residual stage.
- [Spool model](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/spool_model.py): fixed-axis median reduction and dimensionful scaling equation.
- [Geometry](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/theoretical_ellipse.py): constraint circles, reduced coordinates and bounds.
- [Production spool fitting](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/filter_pass.py), [shared calibration settings](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/_autocal_common.py), and [scale polish](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/scale_polish.py): priors, nominal-spool constraints, noise settings and fixed-buildup search semantics. These references support source interpretation; the production fitter was not run.
- [Prior fixed-encoder audit](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/output/research/2026-10-09-slip-recovery-and-encoder-model/report.md) and [its script](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/output/research/2026-10-09-slip-recovery-and-encoder-model/encoder-objective-audit.py): historical context, not substituted for fresh data.
- [Prior validation audit](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/report.md) and [script](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/audit.py): distinction between geometric residuals, sensor prediction, candidate-owned spool parameters and known-radius controls.
- [Reference constants](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/tools/regress_calibration_logs.py) and [frozen HP3 source](https://github.com/tobbelobb/hp-sim5/blob/7fc26ba491be99c5632e21839be4b3afca201d85/autocal/data/references/fifth_hp3_dataset.json): declared simulation truth and compact copied observations. Source/input hashes permit verification after archival relocation.
