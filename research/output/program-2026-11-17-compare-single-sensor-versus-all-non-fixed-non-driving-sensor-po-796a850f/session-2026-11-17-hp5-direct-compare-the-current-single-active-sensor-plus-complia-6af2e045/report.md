# HP5 direct: single active sensor plus compliant redundant axis vs both low axes as torque-controlled sensors

**Status:** complete small-experiment investigation; simulation only. **Date:** 2026-11-17.
**Scope:** HP5 direct scene, isolated headless worlds, no full autocal run, no production defaults changed.

## Independent review corrections (authoritative for interpretation)

The [independent review](../session-2026-11-17-independently-review-the-completed-direct-hp5-sensor-policy-expe-b225ae5c/report.md) reproduces the slack/tautness
finding but supersedes these original calibration-benefit interpretations:

- The physical narrow sweep poses showed no demonstrated fitting benefit. The
  broad-pose synthetic gains must not be described as gains measured on the sweep.
- Constant range bias can be absorbed by home offsets. Pose-dependent slack and
  geometric model error are the relevant threats; a slack pose can erase the gain.
- The 1.4x mode ratio describes the strongest mode, not the median. The median is
  about 6.2x in sigma; the weakest is 180x in information, 13.4x in sigma. Covariance
  norm ratios are not calibration accuracy ratios.
- The original archived sweep script has a home_geom NameError. The reviewed,
  one-line-corrected replay script below reproduces the four decisive runs exactly.
- Fast-reversal parameters were not fully archived, so do not use that stress
  case as a reproducible quantitative claim. Reported encoder_std is length jitter.

The original report and raw results below are preserved as the submitted record.
Production settings remain unchanged. See the review's evidence for corrections.

## Question

Production uses four active lines (`2 fixed + 1 drive + 1 sensor`) and leaves the fifth low axis at a low compliant force. Its encoder is sampled but its range is discarded. Does making that fifth axis a second torque-controlled sensor add usable calibration information, and what does it cost in slack, force/flex bias, acquisition time and fit stability?

Roles in the experiments (same as the HP5 planner rule):

| role | axis | control |
|---|---|---|
| fixed low | A (0) | position |
| fixed top | I (4) | position |
| drive | B (1) | position, ramped |
| sensor | D (3) | torque, 1.0 N |
| redundant | C (2) | torque, policy variable |

"Current" = C at 0.05 N, range discarded. "Both sensors" = C at 0.2/0.3/1.0 N and recorded. The live pulley dataset uses `force_low_n = 0.0516`; the code default is 0.01 N, so 0.05 N is a realistic stand-in.

## Method

- Physics: native Rust engine, `hp5.usda` geometry, `dt = 2 ms`, 50-step request chunks, no layering (`evidence/probe.json`).
- Primary world: `evidence/hp5_taut.usda`, a copy of the catalog scene with each spool link initialized at its geometric free length instead of the catalog's 2 m reservoir. Geometry is unchanged. This removes a scripted multi-metre pullout and keeps the spool in one approximately linear radius regime.
- Cross-check: the unmodified catalog scene with the same scripted pullout, to test whether results depend on initialization.
- Initialization: 500 settling steps, 0.12 Nm on all five axes for 6000 steps (pullout/tension), then rebase, hold A/B/I at the reached angles, and set C/D torque.
- Drive: B ramped at 0.8 rad/s (~24 mm/s), 0.5 rad per sampled step, six steps each side (≈90 mm effector span). 0.8 s settle, six 25-step samples per pose, then a 6 s recovery sample.
- Measured length: encoder-equivalent free length (sum of the two/four cable-joint rest lengths divided by the number of parallel lines). On the taut scene this agrees with `30 mm/rad × spool angle` to ≤0.01 mm for A, B, C and I; the sign is axis-dependent. The comparison quantity is **measured length change minus true geometric free-length change** from the same reference pose. Positive values are paid-out line beyond geometry (slack); small negative values are elastic stretch/model bias.
- Truth (true effector/corner positions, true anchors) is used only for diagnostics.

Scripts: `evidence/sweep_experiment.py`, `make_taut_scene.py`, `summarize.py`, `info.py`, `mc_fit.py`. Raw runs: `evidence/taut_*.json`, `slow2_*.json`, `altrole_*.json`, `rec_*.json`. Aggregated numbers: `evidence/results.json`.

## Results

### 1. Slack: cause, coverage, acquisition stability

Taut scene, realistic speed, 7 poses per direction. Error is the redundant axis's measured-length error; "slack poses" counts poses with mean error > 0.5 mm.

| C command | direction | slack poses | mean / max error (mm) | after 6 s (mm) | min segment force (N) | encoder std (mm) |
|---:|---|---:|---:|---:|---:|---:|
| 0.05 N | down | 1/7 | 0.16 / **1.18** | 1.19 | 0.000 | 0.011 |
| 0.05 N | up | 1/7 | 0.16 / **1.30** | 1.29 | 0.000 | 0.011 |
| 0.10 N | down | 0/7 | −0.25 / 0.05 | −0.03 | 0.000 | 0.008 |
| 0.20 N | down | 0/7 | 0.02 / 0.06 | 0.06 | 0.000 | 0.002 |
| 0.30 N | down | 0/7 | 0.02 / 0.06 | 0.07 | 0.071 | 0.002 |
| 1.00 N | down | 0/7 | 0.03 / 0.08 | 0.08 | 0.796 | 0.002 |

The catalog-scene cross-check reproduces the threshold: 1/7 slack at 0.05 N (max 1.28 mm), 1/7 at 0.10 N (1.08 mm), none at 0.2/0.3/1.0 N. A deliberately fast reversal (≈19 rad/s) produces 7/15 slack poses and up to 3.35 mm at 0.05 N, 6/15 at 0.1 N, none at ≥0.2 N, so speed amplifies but does not create the failure.

**Cause.** This is a take-up torque-floor failure, not gravity alone and not a settling transient:

- The 0.05 N command is 0.003 Nm on a 2-line, 30 mm spool. The simulated torque-mode floor is cogging 0.01 Nm plus Coulomb 0.002 Nm and stiction 0.003 Nm (≈0.012–0.015 Nm worst case, i.e. 0.2–0.25 N per line). The commanded torque is 4–5× below the floor.
- Paying line out is easy; reeling it back in when the geometry shortens is not. Slack appears as extra paid-out line on the take-up side, is one-sided and persists after 6 s at the same pose. It is a stable free-spool state, not an unsettled transient.
- The delivered force is also phase-biased: with C commanded at 0.05 N the measured mean was 0.04–0.12 N depending on spool phase; at 1.0 N it was 0.94–1.03 N. The error is comparable to the expected ±0.17 N cogging amplitude for a 2-line axis.
- At 0.2 N (0.012 Nm) the axis is marginal: force touches zero at some poses but no length slack was observed. 0.3 N (0.018 Nm) is above the floor and gives 0.067–0.13 N minimum segment force; 1 N gives 0.78–0.82 N.

**Not universal.** Rotating the roles (fixed B, drive C, sensor A, redundant D) gave 0/7 slack at 0.05 N (max 0.11 mm). The failure depends on axis, sweep direction and spool/cogging phase. Low-force coverage is therefore *unreliable*, not always zero: the current policy cannot guarantee a valid redundant range, and a single successful sweep is not evidence that it is safe.

### 2. Force and flex bias

- Delivered force at 1 N is 0.94–1.03 N; at 0.3 N it is 0.24–0.35 N; at 0.05 N it is 0.04–0.12 N. The offset changes sign with spool phase and is of order the cogging amplitude. It is a force bias, not a length-scale error.
- The fixed low partner A tracks C closely (0.05→0.05–0.12 N, 1.0→0.94–1.03 N), because A/C are the transverse pair. Raising C therefore raises the fixed line's tension and its unmodelled stretch.
- Line stiffness is `S = 20000 N/m` (M666 S). Elastic stretch is ≈0.05 mm at 1 N, ≈0.015 mm at 0.3 N, negligible at 0.05 N. This is small but must be included in the fixed-line force model if the passive force changes.
- A larger bias is geometric, not elastic: over the ±90 mm sweep the taut longitudinal pair B/D departs from the five-anchor midpoint model by up to ≈0.8 mm, while the transverse pair A/C stays within ≈0.08 mm. The extra C range is the *geometrically cleaner* line, not the noisier one, when it is taut.

### 3. Measured-range information gain

Two independent comparisons, both geometry-only and using truth anchors only for scoring:

- **Linearized Fisher information** (`evidence/info.py`, 30 broad poses, exact fixed lines, 0.01 mm measured-line noise): adding C reduces the linearized anchor σ-trace from **8.15 mm to 0.71 mm (11.5× smaller σ, ≈130× variance)**. The weakest informative singular value improves **≈180×**; the median direction improves only **≈1.4×**. The gain is concentrated in the direction the current two measured lines leave weak.
- **Synthetic anchor recovery** (`evidence/mc_fit.py`, 0.01 mm noise, independent offset-free fit, aligned RMS): current vs both-sensors is **1.47 → 0.89 mm at 20 points** and **1.57 → 0.89 mm at 30 points** (≈40% lower). At 10 points the gain is only **1.37 → 1.17 mm**.

The gain is fragile to bias. Adding a constant 0.05 mm bias to C raises the 20-point both-sensors error from 0.89 to **1.36 mm**, close to the current 1.47 mm. A production fit could absorb a *constant* per-line bias as a home offset, but it cannot absorb pose-dependent slack (1.2–1.3 mm) or the 0.8 mm longitudinal model bias. The 0.05 mm figure is a warning threshold, not a measured production bias.

### 4. Equal-time or equal-point value

The collector already samples every motor's encoder: `raw_angles_deg` is stored for all axes, while only `l_drive` and `l_sensor` become dataset ranges (`autocal/control/behaviors/sweep_data_collection.mjs:1360-1370`). The extra range therefore costs **no additional motion or sampling time**; it is currently acquired and thrown away. At ≥0.3 N the same 0.8 s settle produced no slack, so equal points is the natural comparison. If an extra sensor did cost 2× time, the synthetic equal-time comparison is 10 alt vs 20 current (1.17 vs 1.47 mm), still modestly favorable. The equal-time advantage is real but not large once bias is considered.

The costs are force (0.3–1 N instead of 0.05 N), a fitter/model change (record the passive range, give it a force-dependent length correction, and admit a measured passive line), and a tautness gate. They are not acquisition-time costs.

## Conclusions

1. **The current compliant redundant axis is not a trustworthy sensor at 0.05 N.** It intermittently loses tension during take-up; its encoder then measures a free-spool length biased by up to 1.2–1.3 mm (3.4 mm after a fast reversal). The failure persists after a 6 s hold. Discarding its range at the current force is justified.
2. **Using both non-fixed non-driving low axes as torque-controlled sensors is mechanically feasible, but only above the torque floor.** In the tested geometry, 0.3 N gave no slack in 14/14 poses and ≤0.07 mm error; 1 N gave ≤0.09 mm and a 0.78 N minimum force. 0.2 N was marginal (force reached zero). The delivered force remains cogging-biased by ~0.05–0.1 N, which is negligible for length but must not be mistaken for a calibrated force.
3. **The extra range carries real information, mainly in the weakest anchor direction.** Equal-point synthetic recovery improves ~40%; the linearized weak-mode improvement is much larger (≈11.5× σ-trace). Typical directions improve only ~1.4×.
4. **The value is conditional on bias control.** A 0.05 mm constant bias erases most of the synthetic gain; a slack range is far worse. The extra line is geometrically cleaner than the longitudinal pair when taut, which is an argument for raising its force rather than for using it as-is.
5. **Recommendation for the program:** keep production defaults unchanged. Do not promote the both-sensor policy without a production-fitter replay at ≥0.3 N and a tautness/bias gate. The result is a promising lead with a clear mechanism, not an established improvement.

## Limitations

- Simulation only. Cogging is a model parameter (1% of holding torque); real hardware cogging, friction and force floors may differ. The 0.05 N failure is a simulation result whose *mechanism* should transfer, not its exact threshold.
- One scene, one scale, five roles rotated once. Seven poses per sweep, 90 mm span; not the full HP5 workspace. Fast reversals exaggerate slack.
- The scripted pullout and the taut-scene initialization are not the production collector. The production equal-strain force profile, coupling probes, spool reserve and rejection logic were not run.
- The five-anchor midpoint model is a reduction of the 12 physical line segments. Its geometric bias (up to ~0.8 mm on the longitudinal pair) is a model error shared by all policies, not a slack measurement.
- The Monte Carlo is an independent geometry-only fit without home offsets; its absolute errors are not production accuracy. Home-offset absorption of a constant C bias was not completed; that is the main open modeling question.
- Encoder noise is the simulator's; no explicit hardware encoder noise or missed-step distribution was injected.

## What would change the conclusion

- A production-fitter replay on a 20-point recording at 0.3 N showing that the third range lowers anchor error without a pose-dependent bias would strengthen the case for a policy trial.
- A hardware measurement showing the passive-axis torque floor is well below 0.05 N would weaken the slack argument (but not the information/bias argument).
- A repeat on a second geometry/axis showing frequent slack even at 0.3 N would raise the recommended force or reject the policy.

## Reproduce

From `evidence/` (interpreter `/home/torbjorn/repos/hp-sim5/.venv/bin/python`; the scripts put the worktree `src/python` on `sys.path` and use the root checkout read-only for Node/native physics):

```bash
python make_taut_scene.py
SC="$PWD/hp5_taut.usda"
python sweep_experiment.py --scene "$SC" --policy 0.05,1.0 --steps 6 --drive-step-rad 0.5 \
  --drive-speed-rad-s 0.8 --settle 800 --samples 6 --recover-steps 3000 --direction down \
  --out taut_down_0.05,1.0.json
# repeat --policy 0.1,1.0 0.2,1.0 0.3,1.0 1.0,1.0 and --direction up
python info.py
python mc_fit.py
python summarize.py
```

Environment: worktree and root at `c2450975d7a3dcf1e6bd90d60afc1e7c3acc51f6`; native physics artifact `sha256 1c137e60…ed418b`; scene source `sha256 9d292166…c7286d7`; generated taut scene `sha256 944b66cc…35c9bf`. No runtime was started, nothing was written to the live root dataset or its processes (one read-only force-setting lookup), and no commit was made (manager integration pending).

## Evidence

- [results.json](evidence/results.json) — aggregated tables, commands, info and fit inputs.
- [sweep_experiment.py](evidence/sweep_experiment.py), [make_taut_scene.py](evidence/make_taut_scene.py), [summarize.py](evidence/summarize.py), [info.py](evidence/info.py), [mc_fit.py](evidence/mc_fit.py).
- Primary runs: `evidence/taut_down_*.json`, `evidence/taut_up_*.json`; cross-check `evidence/slow2_down_*.json`; role check `evidence/altrole_*.json`; speed stress `evidence/rec_*.json`.
- Prior context: [autocal stretch and identifiability](../../../topics/autocal-stretch.md) (cogging explains most sensor-force error; commanded forces differ from actual tensions) and the [coupling review](../../program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-review-the-revised-per-point-coupling-protocol-initial-reserve-c-010e9604/report.md), which lists slack as outside the equal-strain invariant.
