# Compliance observability for the top anchor height

Investigation, 2026-11-20/21, branch `t3/top-anchor-compliance-observability`.
**Evidence types.** Simulation evidence only:
- headless native physics with RRF host firmware;
- one exact replay of a recorded run;
- offline refits of recorded datasets, and synthetic data built on replayed true poses and tensions.

There is no hardware data. Errors are anchor coordinate errors in mm in the print convention frame (origin at home, z through the top anchor, A.x = 0; `imperfect_machine.convention_frame`) unless stated otherwise.

## Summary

1. **Where the stiffness→D.z trade is large.** The fit takes compliance from config.g in different ways per machine and collection shape, and the size of the trade follows from that. Offline slope scans on nominal datasets (section 3):

   | Machine and shape | How config S enters | dD.z per unit of compliance scale k | ±20% | Can the fit rms see k? |
   | --- | --- | --- | --- | --- |
   | HP3 default sweeps | not at all | 0 | 0 | — |
   | HP4 default sweeps | ABC home strain | +1.5 mm, as a **common z shift** (low z moves equally) | ±0.3 mm z | no |
   | HP5 default sweeps | per-point stretch | ≈0 on D.z, low z +0.9 mm | ±0.2 mm | no |
   | **HP5 pulley-reduced default sweeps** | per-point stretch | **+8.4 mm** | **±1.7 mm** | partly: rms minimum near k ≈ 1.04 |
   | **Ellipsoid shape, all four machines** | per-point stretch | **−4.8 to −10.4 mm** | **±1–2 mm** | no (flat) |

   So on HP3/HP4/HP5 default sweeps the trade is weak, which agrees with the harness's 1.09 / 0.88 mm for HP3 at ±20%. It is large on HP5 pulley-reduced sweeps and in the ellipsoid shape.
2. **HP3's common z shift is a compliance effect.** HP3's ~−1 mm common z shift (the "home reference under slack ABC") scales with the true compliance, at 0.93 mm per unit k at today's 1.11 N strand target. It is invisible to the fit rms (flat to 1e-5 mm). The cause: ABC lines are slack at the encoder origin, so their measurement stretch `T·(d0+Y)/EA` is a home offset the fit cannot hold. An exact replay reproduces it, and removing the replay's stretch takes HP3 from 0.97 to 0.14 mm (section 2).
3. **A fixed-pose force step makes k observable.** Hold the drive and fixed lines, step the sensor force to 2× and 3×, and read the sensor encoder.
   - **Precision.** Nine live HP3 runs with true EA ×0.8…1.2 (five with perfect geometry, four with random harness geometry) give k to −1.3 ± 3.0% (range −4.3…+4.1%).
   - **Cost.** 150–190 s simulated per run (≈3%, +128 s measured against a paired run without the probe). A reduced version (first sweep, one level, ≈13 s) stays within ±9%.
4. **D.z with the probe.** Feeding the probe's k into the HP4-style home-strain term on HP3 puts D.z within **0.041 mm** of the same fit given the true k in every run (mean −0.012, sd 0.026).

   | Variant | Worst gap to the true-k fit | Worst low z |
   | --- | --- | --- |
   | Configured S | 0.25 mm | 0.31 mm |
   | Today's HP3 fit | 1.25 mm (mean −0.92) | 1.31 mm |
   | Probe | 0.041 mm | ±0.11 mm |

   The max frame error falls from 1.07 (mean, today) to 0.44 mm.
5. **What remains is not compliance.** The D.z error left at the true k ranges from −0.35 to +0.65 mm. It is a D-relative-to-low shape error from force realisation (±0.36 N cogging); with exact replay tensions it drops to 0.14 mm (section 5). The probe does not address it.

## 1. Question

With today's collection and force tuning, a wrong configured stiffness (M666 S) was said to trade almost perfectly against D.z. The investigation asks three things:
1. Where is that trade actually large?
2. Which data makes the real compliance observable, and how much information does each candidate add?
3. Does the best candidate recover D.z on HP3 in the live simulator with true EA off by ±20%?

Radius variation and line buildup are out of scope.

## 2. Mechanism on HP3: the home-reference z shift is stretch

### 2.1 Where compliance enters the fit

- **HP3.** Its default sweeps use no compliance at all: `pose_fit.dataset_stretch` returns None on non-redundant machines, and the equal-strain profile (`autocal/equal_strain.py`) is meant to make stretch look like a scale plus home offsets.
- **HP4.** HP4 adds ABC home-strain offsets `(T/S)(d0+Y/U)` with the configured S (`pose_fit._hp4_home_strain`). This conflicts with the constraint that autocal must not rely on configured stiffness.
- **HP5, HP5r and ellipsoid meshes.** These use the per-point firmware flex model with configured S (`dataset_stretch`).

### 2.2 Exact replay attribution

The 2026-11-18 `replay_poses.mjs` ([copy](evidence/scripts/replay_poses.mjs)) replays the nominal run `fixsweeps-hangprinter_3` (`e022c095`; 0.97 mm, 4587 s). It reproduces all 11 645 four-axis reads in 3 min. [`replay_attrib.py`](evidence/scripts/replay_attrib.py) compares the result against the dataset; output in [`attrib_fixsweeps_hp3.txt`](evidence/results/attrib_fixsweeps_hp3.txt).

- **The sim follows the flex model.** Measured length minus the rigid model equals `T·(d+Y)/(EA·N)` from the replay's strand tensions:
  - correlation 0.99 per line;
  - effective EA 17.8–19.8 kN against 20 kN in the scene.
- **Stretch accounts for most of the error.** Subtracting it takes the production fit from 0.967 to 0.144 mm.
- **The encoder origin has slack ABC lines:** strand tension 0 on A, B and C, 0.265 N on D.
- **Tension realisation.** Commanded minus true sensor tension is −0.003 ± 0.21 N, bounded by ±0.36 N, which matches the cogging floor.

  **Pitfall:** a sweep's second direction swaps drive and sensor. The real sensor is `source_sensor_anchor`; using the sweep's `sensor_anchor` made a first synthetic wrong by 3×.

### 2.3 The trade on HP3

[`home_strain_scan.py`](evidence/scripts/home_strain_scan.py) adds the HP4 home-strain term to HP3 with an assumed compliance multiple k ([results](evidence/results/home_strain_scan_fixsweeps_hp3.txt)).
- Low z and D.z move together by +0.093 mm per 0.1 in k.
- The fit rms is identical (0.02178 mm) for k from 0 to 1.6. Replay-synthetic data with true compliance ×0.8 and ×1.25 shows the same.

A clean synthetic gives the purest form of the degeneracy ([`degeneracy.py`](evidence/scripts/degeneracy.py), [results](evidence/results/degeneracy_equilibrium_synthetic_hp3.txt)). It uses equilibrium tensions at exact commanded forces and a zero-tension home. Today's fit then errs by 1.075 mm × k while the fit rms is 0.0005 mm.

## 3. Where the trade is large: offline slope scan across machines and shapes

[`slope_scan.py`](evidence/scripts/slope_scan.py) refits the final nominal datasets of the ellipsoid investigation (`fixsweeps-*`, `fixellipsoid-*`; local, under `/home/torbjorn/repos/hp-sim5/output/`).
- **Fitters.** Each machine's production fitter: the sweep autocal's two-stage point fit, or `ellipsoid_autocal.fit`.
- **Scaling.** Every config-derived compliance (the flex model and HP4's home strain) is multiplied by k.
- **Interpretation.** The data was collected with true EA equal to configured EA, so k stands for configured/true compliance. To first order, the error at k is what that stiffness mismatch gives.

Results: [`slope_scan_summary.txt`](evidence/results/slope_scan_summary.txt), raw `slope_*.json`.

| Machine | Shape | D.z at k 0.8 / 1.0 / 1.2 | low z at k 0.8 / 1.0 / 1.2 | max | rms |
| --- | --- | --- | --- | --- | --- |
| HP3 | sweeps | −0.57 (all k) | −0.97 (all k) | 1.18 | flat |
| HP4 | sweeps | −0.33 / −0.04 / +0.26 | −0.20 / +0.10 / +0.40 | 0.72 (flat) | flat |
| HP5 | sweeps | +0.11 / +0.12 / +0.13 | −0.14 / +0.04 / +0.22 | 0.87 / 1.00 / 1.36 | flat |
| HP5r | sweeps | −1.31 / +0.36 / +2.05 | −0.65 / −0.07 / +0.52 | 3.32 / 0.92 / 2.24 | 0.0341 / 0.0256 / 0.0310 |
| HP3 | ellipsoid | +4.69 / +2.60 / +0.51 | ≈0 | 4.69 / 2.60 / 1.33 | flat |
| HP4 | ellipsoid | +1.16 / −0.46 / −2.08 | ≈+0.1 | 1.16 / 0.46 / 2.08 | flat |
| HP5 | ellipsoid | +2.97 / +1.27 / −0.44 | ≈0 | 2.97 / 1.27 / 0.53 | flat |
| HP5r | ellipsoid | +3.86 / +2.90 / +1.93 | ≈0 | 3.86 / 2.90 / 1.93 | flat |

**Readings.**
- **HP4 sweeps.** The home-strain term translates all anchors together, so the trade is a z translation, not a D.z-only error.
- **HP5 sweeps.** D.z does not move. The low anchors do, and the max error is some other coordinate.
- **HP5r sweeps.** D.z moves 8.4 mm per unit k. This is the only default-sweep case where the fit rms reacts: at k 0.9 / 1.0 / 1.1 it reads 0.0284 / 0.0256 / 0.0266, minimum near 1.04. Its redundant line therefore already carries some compliance information. A joint k fit on HP5r is untested.
- **Ellipsoid shape.** Here the held-anchor height trades directly and invisibly against the free-line stretch scale, as the ellipsoid report found. The slope is 5–10 mm per unit k.

## 4. Candidates for compliance information (HP3, offline)

| Candidate | Information on k | Cost | Verdict |
| --- | --- | --- | --- |
| Today's sweeps alone | none | 0 | degenerate |
| Configured S (HP4 method) | none; assumes k = 1 | 0 | error 0.93 mm × (k_true − 1) in z |
| Sweeps repeated at 2× strand tension, joint home-strain fit ([`two_level.py`](evidence/scripts/two_level.py), [results](evidence/results/two_level_synthetic_hp3.txt)) | rms falls 4× from k 1.0 to 1.5 | ~2× collection | **biased**: rms minimum near k ≈ 1.5 for a true 1.25. Tension also changes pose-dependent stretch, which this model ignores |
| Full per-point stretch model forced on for HP3 (equilibrium tensions from commanded forces, S/k) ([`full_stretch.py`](evidence/scripts/full_stretch.py)) | rms flat | 0 | at the true k no better than home strain: D.z +0.50 (nominal), +0.64 (EA×0.8) |
| **Fixed-pose force step** ([`probe.py`](evidence/scripts/probe.py)) | direct: sensor encoder change is a + k·b, b = 0.25–0.73 mm/N against ~0.006 mm encoder noise | ~7 s per probe | **chosen** |

**How the force-step estimator works.**
1. Equilibrium fixes the held lines' tension changes for a sensor force step dF, independent of k.
2. With held spools fixed, `−u_i·dp − k c_i dT_i = dm_i` gives the effector shift dp. Here `c_i = (U d_i + Y_i)/(S U² N_i)` from M666, and `dm_i` is the measured encoder change of held line i.
3. The sensor's encoder change is then linear in k, and k is the least-squares slope over all steps.
4. Anchors and pose come from the ordinary fit.

Cogging is nearly common to both levels, because the motor turns ~2° during a step.

## 5. Live confirmation on HP3

### 5.1 Setup

- **Collector.** The probe is opt-in through `HP_SIM5_COMPLIANCE_PROBE=2,3,1` (`sweep_data_collection.mjs::probeCompliance`). After each sweep direction's last point it:
  1. steps the sensor force to 2× and 3× the last point's force, then back to 1×, with all other modes unchanged;
  2. waits for stable encoders;
  3. records all encoders (16-sample mean) in `metadata.compliance_probes`.
- **Harness.** The imperfect-machine harness (cherry-picked `b77ec4aa`, `9fcec8c3`) perturbs only the scene; config.g stays nominal. The harness's environment allow-list now passes the probe variable through.
- **Runs:**
  - `s080…s120`: perfect geometry, EA × {0.8, 0.9, 1.0, 1.1, 1.2};
  - `r1…r4`: random seed geometry (low dz ±30 mm, top XY ≤ 50 mm), EA forced to ×0.8 or ×1.2;
  - `s100np`: nominal without the probe, as a control.

  At most two simulations ran at a time.
- **Refits.** [`refit.py`](evidence/scripts/refit.py) refits each dataset offline with the production two-stage point fit four ways:
  - **today**: as is;
  - **config**: plus ABC home strain with configured S;
  - **probe**: plus ABC home strain with S/k̂;
  - **oracle**: plus ABC home strain with the scene's true EA.

  "low z" is the mean z error of A, B and C, and k = configured/true EA.

### 5.2 Results

Convention frame, mm. Per-run JSON is in [`results/live/`](evidence/results/live/), and [`live_summary.txt`](evidence/results/live_summary.txt) comes from [`table.py`](evidence/scripts/table.py).

| Run | EA × | k true | k probe | D.z today | D.z config | D.z probe | D.z oracle | low z today | low z config | low z probe | max today | max probe | probe time | sim time |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r1s080b | 0.80 | 1.250 | 1.213 (−3.0%) | −1.46 | −0.52 | −0.32 | −0.29 | −1.12 | −0.18 | +0.02 | 1.46 | 0.70 | 153 s | 4759 s |
| r2s120 | 1.20 | 0.833 | 0.817 (−2.0%) | −0.51 | +0.27 | +0.13 | +0.14 | −0.71 | +0.07 | −0.07 | 0.87 | 0.38 | 157 s | 5237 s |
| r3s120 | 1.20 | 0.833 | 0.868 (+4.1%) | −0.52 | +0.25 | +0.15 | +0.12 | −0.61 | +0.16 | +0.06 | 0.69 | 0.27 | 172 s | 5489 s |
| r4s080 | 0.80 | 1.250 | 1.234 (−1.3%) | −0.59 | +0.40 | +0.64 | +0.65 | −1.31 | −0.31 | −0.08 | 1.35 | 0.64 | 166 s | 5095 s |
| s080 | 0.80 | 1.250 | 1.289 (+3.1%) | −0.62 | +0.32 | +0.59 | +0.55 | −1.16 | −0.22 | +0.05 | 1.18 | 0.59 | 190 s | 5620 s |
| s090 | 0.90 | 1.111 | 1.063 (−4.3%) | −1.06 | −0.21 | −0.16 | −0.12 | −1.01 | −0.16 | −0.11 | 1.14 | 0.26 | 162 s | 5050 s |
| s100 | 1.00 | 1.000 | 0.984 (−1.6%) | −1.03 | −0.09 | −0.11 | −0.09 | −0.95 | −0.02 | −0.03 | 1.09 | 0.42 | 151 s | 4715 s |
| s110 | 1.10 | 0.909 | 0.876 (−3.6%) | −1.06 | −0.21 | −0.31 | −0.29 | −0.71 | +0.14 | +0.04 | 1.06 | 0.31 | 158 s | 5183 s |
| s120 | 1.20 | 0.833 | 0.805 (−3.4%) | −0.73 | +0.12 | −0.05 | −0.03 | −0.60 | +0.25 | +0.08 | 0.73 | 0.42 | 154 s | 5289 s |

Spread over the nine runs:

| Variant | D.z mean ± sd | low z mean ± sd (worst) | max mean | D.z minus oracle D.z: mean ± sd (worst) |
| --- | --- | --- | --- | --- |
| today | −0.84 ± 0.33 | −0.91 ± 0.26 (1.31) | 1.07 | −0.92 ± 0.24 (1.25) |
| config | +0.04 ± 0.31 | −0.03 ± 0.20 (0.31) | 0.38 | −0.04 ± 0.17 (0.25) |
| probe | +0.06 ± 0.35 | −0.01 ± 0.07 (0.11) | 0.44 | −0.01 ± 0.03 (0.041) |
| oracle | +0.07 ± 0.34 | +0.01 ± 0.06 (0.11) | 0.44 | — |

**Observations.**
- **The probe is the best estimate of k.** With k̂, D.z is within 0.041 mm of the oracle in every run and low z is within 0.11 mm of zero.
- **Configured S can still score well.** Its mean max error (0.38 mm) is slightly lower than the probe's or oracle's (0.44 mm). At EA×0.8 its wrong correction happens to cancel the shape error of 5.4. That is luck of sign, not information: at EA×1.2 it is worse in low z (+0.16 to +0.25).
- **The probe does not perturb the run.** The run's own frame error at nominal is 1.18 mm without the probe (`s100np`, exactly the benchmark's 4587 s) and 1.09 mm with it (4715 s, +128 s).
- **Deterministic reproduction.** `r1s080` was interrupted by a machine reboot after its dataset was written. Its rerun `r1s080b` produced identical sweeps; only the rerun is kept as evidence.

### 5.3 Probe precision and cost

- **Per-step scatter is pose model error, not noise.** Six repeated probes at one pose agree to 0.001 ([`probe_structure.txt`](evidence/results/probe_structure.txt)):
  - low poses (z ≈ 40 mm) read 6–8% low in three runs and 2.5% low in one;
  - higher poses scatter by ±10%.
- **Likely sources:** effector rotation, routing and pinhole friction. Returning to 1× leaves 0.015 mm of hysteresis.
- **Cost.** 6.4–7 s simulated per probe, 150–190 s per run.
- **Reduced protocols** ([`probe_subsets.txt`](evidence/results/probe_subsets.txt)):

  | Protocol | Worst k error over nine runs |
  | --- | --- |
  | 2× level only | 8.4% |
  | first two sweeps | 7.4% |
  | first sweep's two probes at the 2× level (≈13 s) | 8.7% |

  At HP3's 1.11 N strand target, 9% of k is about 0.08 mm of z.

### 5.4 What remains: shape error from force realisation

D.z minus low z ranges from −0.35 to +0.54 mm, the same in all four variants, because the home-strain term is a translation. On the nominal replay:
- the stretch from true tensions leaves D.z +0.14 mm;
- the same model with tensions from commanded forces leaves +0.50 mm.

The remainder is the ±0.36 N commanded-vs-realised tension error passing through the fit's ill-conditioning. It is the single-run knife edge seen before, and phase-averaged or dithered forces remain the open lead.

## 6. Limitations and alternative explanations

- **Ideal sim.** The simulator's lines are linear-elastic with EA per length, which is exactly the probe's model. Real lines creep, stretch nonlinearly and have friction and hysteresis, so hardware validity is unknown. The probe measures compliance at 2–8 N on the sensor line, while measurement happens at 1–3 N.
- **Configured geometry terms.** The probe uses configured Y/U and N. A wrong guy length biases k much as a wrong EA does, and the correction reuses `(d0 + Y)`, so the two partly cancel. This is untested.
- **Offline only.** "Probe" results are offline refits with a monkeypatched `_hp4_home_strain`. The live runs' selection and stopping used today's fit.
- **Coverage.** HP3 only, one strand target. The live probe was not tried on HP4, HP5, HP5r or the ellipsoid shape, which section 3 shows is where the trade is large. HP5's estimator would need force-mode extra sensors treated as constant-tension lines; that generalisation is small but untested.
- **The slope scan approximates a mismatch.** It rescales the assumed compliance on data collected at matched stiffness, which is a first-order stand-in for a true mismatch. Collection itself (force profiles) does not use S.

## 7. Recommendations

1. **Use measured compliance where the trade is large.** The next experiment is the probe live on HP5 pulley-reduced sweeps and on the ellipsoid shape, at ±20% EA. There k̂ feeds `dataset_stretch`'s multiplier.
   - HP5r: compare against a joint k fit from its own rms, which reacts to k (section 3).
   - HP4: replace the configured S in `_hp4_home_strain` by k̂. That removes its ±0.3 mm z dependence and meets the "12 kN with 20 kN configured" constraint.
2. **HP3 decision for the user.** Enabling the home-strain term with the probe's k moves HP3's default common z shift from about −0.9 mm to ±0.1 mm (n = 9). It changes stored benchmark results.
3. **Productionise the probe in reduced form:** first one or two sweeps, one or two levels, a CLI flag instead of the research environment variable, and k̂ stored in the dataset.
4. **Do not collect multi-tension sweeps for compliance.** They cost twice as much and need a per-point tension model that today's force accuracy cannot supply.
5. **Remaining D.z shape error.** On HP3 it is force realisation (5.4), not compliance.

## 8. Reproduction

```bash
# replay of the nominal run (3 min), attribution and the HP3 degeneracy
node output/replay_poses.mjs <run>/sweeps.headless/scene.usda <run>/sweeps.headless/events.jsonl output/compliance/replay_fixsweeps_hp3.jsonl false
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/replay_attrib.py <run>/sweeps.json output/compliance/replay_fixsweeps_hp3.jsonl
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/home_strain_scan.py <run>/sweeps.json evidence/results/replay_fixsweeps_hp3.npz 0.8 1.25
# cross-machine slope scan
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/slope_scan.py hangprinter_5_pulley_reduced <output>/fixsweeps-hangprinter_5_pulley_reduced/sweeps.json sweep 0.8 1.0 1.2
# one live run with the probe (about 7 min wall), then the four refits
HP_SIM5_COMPLIANCE_PROBE=2,3,1 .venv/bin/python -m autocal.tools.imperfect_machine OUT --machine hangprinter_3 --seed 1 --stiffness-scale 0.8
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/refit.py OUT > OUT_refit.json
```

- **Replay.** `replay_poses.mjs` must sit one directory below the repo root to resolve its imports. Its output `replay_fixsweeps_hp3.jsonl` (9.7 MB, sha256 `3ba6c7cb1237b1863fe5b52c424861d583015b77393867a5af58c844b5a0081c`) is regenerable; the derived arrays and log are committed.
- **Live inputs.** Live datasets are committed gzipped under [`evidence/runs/`](evidence/runs/) with `imperfection.json`, the rescored `result.json` and the headless manifest. Refits read `sweeps.json`: gunzip into a run directory first.
- **Source data.** The nominal source datasets for sections 2–3 are local and ignored, under `/home/torbjorn/repos/hp-sim5/output/fixsweeps-*` and `fixellipsoid-*`. Their headless manifests carry git revision `e022c095`.
