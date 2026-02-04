# Substep 4: Orchestrate full-auto runs across sweep-metrics and stop criteria

## Objective
Implement a deterministic orchestration layer that can try multiple setups of settings, compares results with the same normalized objective and covariance metrics, then stops when criteria are met.

## Candidate runs
Run calibration with different flags enabled.
We can imagine they might want to try all `--sweep-metric` options:
- `mad`
- `median_abs`
- `outlier_ratio`

They might also expand the matrix of runs with:
- pointwise filtering on/off,
- sweep-wise filtering on/off,
- global vs per-sweep MAD.
- `--use-raw-lengths` vs `--use-noise-mean`

Let the user specify a list of different settings (flags), eg
`[--sweep-metric mad --use-raw-lengths, --sweep-metric median_abs, --use-raw-lengths]`.
If the user supplies no list then only run with default flags.

## Ranking strategy (per dataset)
1. **Filter invalid runs**:
   - non-finite cost,
   - covariance not finite.
2. **Select the best run** deterministically:
   - Primary: lowest noise normalized cost. This is like `J` but without the filtered points.
   - Tie-breaker: smaller scaled covariance summary in **dimensionless** units (e.g., `max_anchor_std_mm / workspace_diag`).
3. **Select the best run** (lowest primary + tie-breaker), avoiding unit-mixing unless explicitly normalized.

## Stop criteria
Accept and update config if all conditions are true:
- Cost improvement below epsilon for `K` iterations (converged). Use K=2 as default.
- No data-quality warnings.

Otherwise:
- If not converged but improving, gather more sweeps.
- If not converged and data quality poor, emit warning and stop.

## Deliverables
- Full-auto orchestration CLI or entrypoint.
- Structured result log summarizing cost, covariance, rank, and run flags.
- Deterministic run selection + clear warnings.

## Other Useful Info
This console output from semi-auto can help you understand what you'll be working with:
```
$ python autocal/active_calibrate.py --sim --machine-type slideprinter --plot-residual-histogram --dataset autocal/data/objective_metrics2.json --output-with-explanations --collector-args --speedup 35
; starting rrf_simulator at http://localhost:8081

; === iteration 1/20 dataset=autocal/data/objective_metrics2.json ===
[gnc] stage 1/3: wide-huber maxiter=400
[gnc] stage 2/3: tight-huber maxiter=240
[gnc] stage 3/3: trim maxiter=160
[robust] pointwise sigma_source: mode=auto point=126 origin=0 min=0 mixed=0 total=126
[robust] pointwise: enabled=True stage=2 (trim) hard_cut=True huber_mult=3
[robust] pointwise scale: source=global global=1.49 floor=8.84e-06 floor_mm=0.05
[robust] pointwise sigma_floor_mm: origin=0.00256314 mult=3 scaled=0.00768942 min=0.05 used=0.05 floor_deg=0.01 source=min
[robust] pointwise inlier_ratio: min=0.976 med=1 max=1
[robust] pointwise worst: sweep_002 41/42 (0.976), sweep_001 42/42 (1), sweep_003 42/42 (1)
[robust] sweep-wise: enabled=True status=insufficient-samples (3 < 5) metric=outlier_ratio threshold=n/a rejected=0
[robust] sweep-wise worst: sweep_002 m=0.0238 keep, sweep_003 m=0 keep, sweep_001 m=0 keep
[residuals] wrote 126 points to autocal/data/objective_metrics2.csv
; Active ellipse calibration
; Anchors: [[-1.0177739848615942e-13, -1902.3037243816245], [1646.5935905101492, 951.1037995425276], [-1645.7166085094675, 950.2815061826809]]
; cost=0.002946 cost_noise_normalized=2.641 origin_norms=[1902.304, 1901.544, 1900.373] pairwise=[d01=3294.420, d02=3293.270, d12=3292.310]
; noise_cost: J=3.04 chi2_red=3.192 |z|_med=1.113 |z|_p95=3.791 outlier_ratio=0.119 N=126 mode=sigma lengths=mu
; noise_norm_floor: min_sigma=0.05mm floor_deg=0.01deg source=auto
; info rank: 5/6 std(p0=964mm, p5=926mm, p3=926mm, p4=844mm, p2=844mm, p1=801mm)
; covariance_scaled: chi2_red=3.192 std_scaled(p0=1.72e+03mm, p5=1.65e+03mm, p3=1.65e+03mm, p4=1.51e+03mm, p2=1.51e+03mm, p1=1.43e+03mm)
; CI95: max_std=1722mm max_ci=3375mm workspace=2604 rel_std=0.6613 rel_ci=1.296 scale=chi2_red=3.192
; note: info rank deficiency can arise from symmetric setups or limited sweep/constraint diversity.
; note: covariance values can grow under rank deficiency, even when recovered anchors are good.
; encoder_noise: points=126/126 median_sigma=0deg floor=0.01deg min_samples=10
; encoder_noise_flags: low_samples=0 sigma_below_floor=126 sigma_nonfinite=0
; next_sweep score=0.380158 fixed=[0] targets=[897.841898339692] pair=[1,2]
; top_candidates:
;   0.380158 fixed=[0] targets=[897.841898339692] pair=[1,2]
;   0.380149 fixed=[1] targets=[897.841898339692] pair=[0,2]
;   0.380117 fixed=[2] targets=[897.841898339692] pair=[0,1]
;   0.377963 fixed=[0] targets=[875.076721459187] pair=[1,2]
;   0.377954 fixed=[1] targets=[875.076721459187] pair=[0,2]
; collect_command:
;   node autocal/control/cli/collect_sweep_data.mjs --machineType slideprinter --sweep-config-file autocal/data/objective_metrics2.active_sweep_cfg.txt --fixed-targets 897.841898339692 --output-file autocal/data/objective_metrics2.new_001.json --speedup 35 --no-spawn-rrf-simulator --force-low 0.017355279298559987 --force-mid 0.3471055859711997 --force-max 3.6589616075197386 --no-auto-tune-force --return-to-origin
; reusing force tuning: low=0.01736N mid=0.3471N max=3.659N
; plotting residual histogram:
;   /usr/bin/python /home/torbjorn/repos/hp-sim5/autocal/plot_residual_hist.py autocal/data/objective_metrics2.csv --output autocal/data/objective_metrics2_7.png
; histogram saved to autocal/data/objective_metrics2_7.png
Accept anchors [a], collect next sweep [c], quit [q]?
```

 - cost_noise_normalized=2.64 and J=3.04 look healthy. They line up in the "a few‑sigma" range.
 - chi2_red=3.19 is >1, which means either the noise floor is still a bit too optimistic or there's model mismatch (likely), but it’s not pathological.
 - The rank deficiency and large covariance are dominated by geometry + symmetry, not by the noise floor.

### What is a “low” noise‑scaled cost?

  Given how the code defines J:

- Ideal: J ≈ 1 and chi2_red ≈ 1
  That means residuals are roughly at the noise level.
- Good/acceptable: J around 1–4, chi2_red around 1–5
  That’s typical when the noise model is slightly optimistic or the model isn't perfect.
- Concerning: J > 10
  Usually means either a bad fit or unrealistically small sigmas.

In our output, J=3.04 → RMS z ≈ √3.04 ≈ 1.74σ, which is reasonable.

### Why CI is often big

Your CI uses the geometry‑only information matrix, then scales by chi2_red. With symmetry and rank‑deficiency, the covariance can blow up even if the recovered anchors look good in mm. So yes, symmetric setup can explain both the missing DoF and large covariance. This is not an error. Recovered anchors can still be good.

## Summary
The essence of this substep is to create a mode that automatically analyzes the same values as are printed during a semi-auto run, and automatically chooses between the
[a], [c], and [q] actions on behalf of the user.
Console output should be for end-users so very short, concise and user friendly.
Much of the info emitted in semi-auto can be delegated to a log file when in full-auto.
