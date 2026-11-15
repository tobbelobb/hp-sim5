# Review of the recorded motor-phase force diagnosis

Reviewed 2026-11-15 against frozen source `5251f749b24eaf0db851a8a3174ed6023f511e83`. The nominal recordings were acquired at `dfc0e3e3f47be4e7ae915211a42645d40e5e8746`.

The numerical diagnosis is supported. I independently reproduced all six saved amplitude, held-out RMS and explained-variance results to a maximum absolute difference of `1.40e-14`, and independently reread the RRD strand forces to reproduce both cached force proxies exactly. The force and reeving units are consistent. I found no train/test target leak or angle-selection defect that explains the favorable result. The supported conclusion is a dominant recorded force-proxy discrepancy at the simulator's cogging frequency and predicted amplitude. These observations do not measure its causal share of calibration bias or establish hardware behavior.

## Evidence and reproduction

The bounded inputs were the [workspace report](../validation/workspace-expansion/report.md), [force diagnostic](../validation/workspace-expansion/force_phase.py), [recording sampler](../validation/workspace-expansion/analyze_recording.py), and the closed `hp3-final` and `hp4-final` directories. The curated guide and `research/topics/autocal-stretch.md` were consulted for prior scope. No new physics, service changes, production/test edits, commits or parent-evidence writes were made.

[reproduce.py](reproduce.py) is an independent implementation. It writes only this investigation's [review-results.json](review-results.json) and [evidence-manifest.json](evidence-manifest.json); it does not execute either parent script's `main`, which would overwrite parent evidence. Run from the repository:

```bash
.venv/bin/python research/output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-review-the-recorded-motor-phase-diagnosis-in-validation-workspac-e3e7ab8f/reproduce.py --rrd
```

The original split trains three coefficients per physical sensor, using `sin(50θ)`, `cos(50θ)` and an intercept. All 197 HP3 and 179 HP4 accepted points are retained. Frequency 50 comes from the simulator source/scene, rather than optimization against the test targets. The force error is recorded resultant-force proxy minus commanded sensor force, in newtons.

| Recording / sensor | Train / test points | Amplitude N | Test raw RMS N | Test residual RMS N | Explained variance % | Test-centered R² % |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| HP3 A | 30 / 40 | 0.326237 | 0.236181 | 0.025683 | 98.8642 | 98.8172 |
| HP3 B | 48 / 20 | 0.335570 | 0.235584 | 0.026803 | 99.2821 | 98.6448 |
| HP3 C | 19 / 40 | 0.334904 | 0.217557 | 0.029481 | 98.3562 | 97.8492 |
| HP4 A | 20 / 39 | 0.669297 | 0.464442 | 0.028663 | 99.6222 | 99.6134 |
| HP4 B | 40 / 20 | 0.659114 | 0.524076 | 0.028156 | 99.7531 | 99.7034 |
| HP4 C | 40 / 20 | 0.671823 | 0.494553 | 0.028388 | 99.6700 | 99.6697 |

The saved statistic is `1 - var(residual)/var(error)`. It should be called **held-out explained variance**, not automatically an SSE reduction or R². Its invariance to residual mean matters modestly on HP3: the largest held-out residual mean is -0.01838 N on B. The actual reduction in squared error relative to predicting zero discrepancy is 98.16–98.82% on HP3 and 99.62–99.71% on HP4. This terminology difference does not defeat the dominant-sinusoid finding.

The array parity is **zero-indexed**: indices 0, 2, 4, … train, corresponding to saved IDs `sweep_001`, `sweep_003`, … . Thus the phrase “even-numbered sweeps train” is ambiguous if readers use the one-based IDs. Entire canonical sweeps, including both physical sub-sweep orientations, remain on one side. The nominal HP3 saved dataset contains ten canonical sweeps; HP4 contains nine. The report's broader count of eleven completed HP3 sweeps is not the phase-regression partition and was not audited here.

## Units and physical source connection

The active manifests name `rust-native`, not the JavaScript reference implementation. Both recorded native-build manifests match the current exact hashes of `engine.rs` and `lib.rs`; both input-scene and baked-scene hashes match their manifests. The manifest preserves native artifact SHA-256 `5b0507885a50fc7caea70011c2cd6f039c0441d5d28202fbb50677f657b4a013`. This review checked recorded build/source provenance; it did not rebuild or execute that binary.

In [engine.rs](../../../../src/wasm/cable_solver/src/engine.rs:1531), torque mode obtains `angle` from `spool_angle` and adds `-amp*sin(freq*angle)` to physical drive torque at line 1617. Defaults are `amp = 0.01*holdingTorque` and `freq = numPolePairs`. The same file's encoder path uses `spool_angle` and unwraps its turns at lines 1708–1741. The Rust bridge packs these motor fields at offsets 107 onward, and the scene builder loads holding torque and pole count. Both closed nominal scenes specify 1 Nm, 50 pole pairs, 0.03 m spool radii, and the relevant firmware motor/spool gear ratio is 20/20. `torqueModeSystem.js` agrees with the native expression but is supporting reference code, rather than proof of the active backend by itself.

The recorder scalar is a physical segment-tension magnitude. Native `lib.rs:1039–1040` converts accumulated constraint multiplier to force using `1/dt²`; the bridge exposes that force, and `flightRecorderSnapshot.js:106–113` emits it as `force_n`. The recorder writes those segment magnitudes to `/cable_forces/**`. The sampler takes the first segment on each path and constructs the proxy

`F_i = U_i × Σ_path T_path`.

It sums the two lower paths and multiplies reeving once. There is no additional division by strand count N when computing **total resultant force**. N belongs in conversion to average per-strand tension. Firmware `HangprinterKinematics.cpp:1303–1312` converts a requested resultant force to spool torque as `(F/U) × R_mm × 0.001`, then applies the gear ratio and sign. That is dimensionally consistent with the diagnostic.

For the lower axes, the ideal predicted resultant cogging amplitude is therefore

`A_F = U × (0.01 × 1 Nm) / 0.03 m`.

This is 0.333333 N on HP3 (`U=1`) and 0.666667 N on HP4 (`U=2`). It is not 0.333 N on each of two strands. The fitted amplitudes differ from these predictions by at most 2.13% on HP3 and 1.13% on HP4. The positive force-error sine coefficient is compatible with the negative rewinding torque convention: greater negative drive torque produces greater positive cable tension. No sensor uses the carrying D axis, so these observations do not validate D's amplitude or its different reeving.

These scalars remain a point-effector resultant proxy: multiplying a path tension by U does not independently measure a vector sum over finite attachment geometry or validate pulley friction. In these recordings, however, every segment on a given path has exactly the same stored scalar at the checked noise-window start, midpoint and end samples. A first-versus-last-segment tension mismatch is therefore not the source of this observed sinusoid at those samples.

## Angle, sampling and split checks

The collector records motor angles from `M569.3` and computes `mu` as the arithmetic mean of the same encoder channel; it does not divide these angles by U. The external encoder response converts the unwrapped physical shaft angle from radians to degrees. Reeving is already handled by `mm_per_degree` for length conversion: 0.523599 mm/degree on HP3 versus 0.261799 on HP4's lower axes. Applying that length conversion to the phase angle would be a unit error; the phase diagnostic correctly does not do so.

All points have four-element `mu` and simulation timestamps. The diagnostic respects `source_sensor_anchor` after canonical orientation remapping; this field is present on 99 HP3 and 89 HP4 points. Selecting the canonical sensor unconditionally would instead mix physical sensors. There is no such defect in the supplied code.

Maximum sensor raw-versus-mean angle difference is 0.0175° on HP3 and 0.01625° on HP4, under 0.875 electrical degrees at frequency 50. Using raw angles instead leaves held-out explained variance at 98.33–99.28% / 99.61–99.76%, respectively. Thus mean-versus-raw selection is not responsible for the conclusion.

Every NPZ timestamp equals the supplied noise-window midpoint exactly. Windows last 0.416 simulated seconds. The recordings sample every 100 steps at 0.002 s/step, so latest-at lag can approach 0.2 s. Independent extraction read 27,950 force samples per path on HP3 and 24,768 on HP4; it matched the cached force arrays with maximum absolute difference **0 N**. The latest-at force lag was at most 0.2 s, and every noise window contained at least two recorded force observations. Accepted points are at least 3.29 s apart on HP3 and 3.03 s apart on HP4, so a single physics sample is not reused across the train/test boundary.

Replacing midpoint force by the mean of recorded forces inside each encoder-noise window changes the sensor force by only 0.000470 N RMS on HP3 and 0.001267 N RMS on HP4. Window-mean phase fits still explain 98.35–99.28% / 99.62–99.75%. Start-to-midpoint and end-to-midpoint changes are also small, at most 0.00253 N RMS among the four comparisons. This check substantially weakens stale-force sampling as an explanation for a 0.22–0.52 N RMS discrepancy. It cannot bound unrecorded variation between the 0.2 s physics samples.

Each regression uses only training target errors. Sensor labels, encoder angles and timestamps are covariates, not target-dependent coefficients. The three-column designs are well conditioned (condition numbers 1.62–2.08). I found no direct target leakage. Still, these are post hoc splits of two adaptively collected deterministic simulation runs, not independent machines or preregistered new trials. Later sweeps can depend on earlier acquisition/fitting decisions. The split establishes within-recording prediction at new complete sweeps, with no estimate of between-run uncertainty.

## Adverse comparisons and strongest alternative

Several deliberately less favorable comparisons preserve the finding:

- A source-amplitude prediction with **no fitted phase, offset or amplitude**, `0.333333*sin(50θ)` or `0.666667*sin(50θ)`, leaves only 0.02490–0.02550 N RMS on HP3 and 0.02563–0.02748 N RMS on HP4's original test sets. It performs comparably to the trained model and prevents three fitted coefficients from being the explanation for the favorable match. Its implied source/encoder phase convention is specific to these nominal scenes.
- A forward split, training on the first five saved sweeps and testing all subsequent sweeps, retains 98.58–99.01% explained variance on HP3 and 99.46–99.72% on HP4. Reversing the original parity split also retains 98.70–98.75% / 99.61–99.66%. The phase match is not confined to one parity assignment.
- Identical three-parameter fits at prespecified control frequencies 1, 25, 49, 51 and 100 do not approach frequency 50: their best held-out explained variance across all six sensors is 21.20%, with many negative values. These controls test specificity; they are not a frequency search used to choose the reported model.
- A training-mean-only predictor leaves 0.208–0.242 N RMS on HP3 and 0.468–0.541 N RMS on HP4. The improvement is not merely removal of a constant bias.

The strongest remaining alternative is another angle-locked effect in the same simulator or force-proxy construction, correlated with the model's electrical phase. This is an observational comparison between simulator-derived encoder and force channels. A numerical motor/cable effect with the same phase, or an angular measurement artifact, is not formally excluded merely by a predictive harmonic. Smooth force/pose confounding, a mistaken reeving factor, ordinary sampling lag and regression overfitting are substantially weakened by the frequency controls, exact RRD reproduction, stationary-window comparison, cross-split persistence and unfitted source-amplitude match. Since the inspected physical torque path actually contains exactly that frequency and amplitude, cogging is the specific well-supported model explanation. A controlled change to that term would be needed to estimate its causal contribution independently.

## Claim boundary and disposition

Accept the parent report's scoped sentence that the dominant observed **force discrepancy** is consistent with simulated motor cogging. Retain the explicit distinction between this diagnostic and production inputs. The correct small wording improvements are to name zero-index parity and preserve “explained variance” as the reported metric.

These data do not establish how much cogging causes the HP4 anchor bias, whether force correction would improve a fitted geometry, whether phase dithering is an effective acquisition policy, or whether learned compensation is identifiable from real encoder-only data. All retained sensor motions are rewinding; exchanging drive/sensor axes is not an opposite-direction experiment on the same sensor. No hardware forces or hardware cogging were measured. Error in geometry, compliance, home reference and fitting can remain even if this scalar force discrepancy is removed.

The conclusion would weaken if a matched cogging-disabled or phase-shifted source comparison retained the same force harmonic, or if a genuinely independent force measurement failed the predicted amplitude/phase relation. A matched intervention that removed the harmonic while holding the acquisition plan fixed could establish its force-level causal contribution; anchor-level contribution additionally requires a matched calibration outcome. Those are future evidence conditions, not interventions performed by this review.

## Exact evidence identity

The [manifest](evidence-manifest.json) contains full SHA-256 and byte size for every reviewed source, parent helper, JSON/NPZ input, scene, manifest and RRD. Core identities are:

| Input | SHA-256 |
| --- | --- |
| HP3 `sweeps.json` | `e60c5641e074c8951a255059fb57a7d7a6911c4e0dc6dfcff3bccf188f28647a` |
| HP3 `physics-samples.npz` | `b2e822a527f7823060a7e6e8d69b6b7c5a3a37d5e84fe00edd6a7a8f3f027622` |
| HP3 `force-phase.json` | `fcaf23687beaa3b4a6832de9392248c5708e73cb5287fb71c76b9fcb150bc4ab` |
| HP3 RRD, 405,149,872 bytes | `dc0343840caaa0901fa296d896acdfc6406880fde0b8618f7fddb80f94fcbfa3` |
| HP4 `sweeps.json` | `41c6cc74148118e565261d4d5799b7c8f35f4ff609bd12398e7734277e1b12f1` |
| HP4 `physics-samples.npz` | `f3b8e30589c795e5c134862d59aaba67b66df7a8edf823a6e9b86fe2bdf40802` |
| HP4 `force-phase.json` | `d531453305be2a8ba5dd591bb9de13a6895f5fcedabc7005884906dfd47cca69` |
| HP4 RRD, 476,078,345 bytes | `de9d636e0a7ea5015e10ec0a1fd8243c4fc9f894da224274bb2182aaef7684ad` |
| Native `engine.rs` | `4150b7fd18ce7e5efa5cedc5c738ca263b9f292140fb3410be612dde25698b77` |
| Parent `force_phase.py` | `709e3fd3ebcda0dc04fd2fd65227f0b79840a71531d476f5eb900e86791f6a60` |
| Parent `analyze_recording.py` | `1349f74c2aee6a22ceea340f2a30b34f05bbb54b7e5df6edb4599f2787bee755` |

Raw recordings remain in the parent's local closed-run directories, named in `review-results.json` and their headless manifests. No remote durable-storage URI was supplied or created by this review. Topic/agenda/index files were not modified because this review was authorized to write only its assigned directory.
