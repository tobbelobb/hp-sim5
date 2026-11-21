# Home-stretch correction as the default autocal fit

**Status:** complete, 2026-11-21. Simulation evidence only: imperfect-machine harness, live, paired.
**Code:** commit `05d6e3d9` on `t3/default-home-stretch-fit`; the paired baseline is `338ded23` (integration branch).

## Question

The compliance study showed that adding the HP4-style ABC home-strain term to HP3's fit cuts the max print-frame error from 1.07 to 0.38 mm in offline refits:
- source: [compliance study](../session-2026-11-20-compliance-observability-for-the-top-anchor-height-with-todays-c-4f2510fd/report.md);
- attribution: [review-78e5d2a30d57](../reviews/review-78e5d2a30d57/review.md).

The term models the low lines as slack at encoder zero, so their stretch during measurement acts as a home offset: `(T/S)(d0 + Y/U)`, with the configured S.

The human approved making it the default, machine by machine (HP3 → HP4 → HP5 → HP5r), with changed stored benchmark numbers accepted. This session asks two things:
1. Does the term improve the live fit on each machine, without regressing stop reasons or time?
2. Where is it already modelled?

## Change

`pose_fit._hp4_home_strain` becomes `pose_fit._home_strain`. Its gate changes from `machine_type == "hangprinter_4"` to "a d+1-line machine (`num_anchors == dimensions + 1`) with equal-strain sweeps". That makes the term default on HP3 as well as HP4.
- **No other changes.** No new flag, and no new code path. `ellipse_fit` imports the renamed function.
- **Test.** The existing HP4 home-reference test in `autocal/tests/test_pose_fit.py` is now parametrised over HP3 and HP4. The HP3 case fails on the old gate.
- **Test suite.** The non-heavy suite passes: 745 passed.
- **No physics code changed**, so `npm run test:physics` is not required.

**Why not HP5/HP5r?** The redundant machines already model this effect. Their per-point flex model (`dataset_stretch` → `FlexModel.corrected_distance_mm`) treats `|anchor| + offset + encoder length` as the *unstretched* length, which is the same slack-at-encoder-zero assumption. Section 3 shows that adding the term again double-counts the stretch.

## 1. HP3: large, consistent improvement (live, paired)

The runs use the same seeds and the same harness command as the benchmark baseline (`autocal.tools.imperfect_machine`, the CI benchmark settings). The base arm runs from a pristine worktree at `338ded23` and the candidate arm from `05d6e3d9`.
- Table: [hp3_paired.md](evidence/results/hp3_paired.md).
- Per-run data: `evidence/runs/{base,home}/hp3-*` (`result.json`, `imperfection.json`, `sweeps.json.gz`, logs).

| Run | Frame max (mm), base → home | Δ | Top (D) z, base → home | Low z mean, base → home | Simulated s, base → home | Stop, base → home |
|---|---|---|---|---|---|---|
| nominal | 1.19 → 0.60 | −0.59 | −0.57 → +0.39 | −0.97 → −0.04 | 4585 → 4584 | stable-estimate → same |
| s0 | 0.72 → 0.34 | −0.37 | −0.51 → +0.34 | −0.68 → +0.13 | 4794 → 4791 | same |
| s1 | 1.12 → 0.44 | −0.69 | −1.01 → −0.08 | −0.97 → −0.06 | 5055 → 5040 | same |
| s2 | 0.74 → 0.46 | −0.28 | −0.37 → +0.46 | −0.67 → +0.17 | 5434 → 5121 | same |
| s3 | 0.93 → 0.58 | −0.35 | −0.71 → +0.33 | −0.74 → +0.07 | 5045 → 5045 | same |
| s4 | 1.20 → 0.29 | −0.91 | −1.20 → −0.29 | −0.89 → −0.06 | 5021 → 5017 | same |
| s5 | 1.62 → 0.50 | −1.12 | −1.62 → −0.47 | −0.85 → +0.10 | 5074 → 5064 | same |
| s6 | 0.94 → 0.48 | −0.45 | −0.71 → +0.31 | −0.81 → +0.02 | 5060 → 5366 | same |
| s7 | 0.92 → 0.53 | −0.39 | −0.53 → +0.10 | −0.83 → +0.12 | 4807 → 5078 | same |
| s8 | 1.54 → 0.29 | −1.25 | −1.54 → −0.20 | −0.83 → +0.03 | 4721 → 5329 | same |
| s9 | 0.82 → 0.47 | −0.35 | −0.69 → +0.47 | −0.79 → +0.04 | 4730 → 4736 | same |

- **Random seeds (n = 10).** Base 1.06 ± 0.31 mm, home **0.44 ± 0.10 mm**. Paired Δ −0.62 ± 0.11 mm (se), and home is better in **10/10**.
- **The baseline reproduces.** At `338ded23` the base arm reproduces the stored benchmark baseline (1.06 ± 0.31 mm, nominal 1.19 mm).
- **What moved.** The −0.7…−1.0 mm common low-z shift disappears: low-z mean goes from −0.80 to +0.06 mm.
  - D.z's sign flips. The residual D.z of −0.47…+0.47 mm is the shape error (D.z minus low z) that the compliance study attributed to force realisation; this term does not touch it.
  - The worst coordinate is now often an xy coordinate (A.y, B.x, C.x) of 0.3–0.6 mm.
- **Stops and time.** All 22 runs stop at `stable-estimate`. Simulated time is +85 s on average (−313…+608 s). Sweep counts: s6 and s7 took one more sweep, s8 two more (7 → 9), and s2 one fewer; the other 7 runs are unchanged.

**Stiffness tolerance.** The seeds draw EA ×0.86…1.17 (k_true = 1/scale).
- Home-arm low-z mean against (k_true − 1) has slope −0.54 mm per unit k, with residual sd 0.07 mm ([data](evidence/results/hp3_stiffness_vs_common_z.txt)).
- So ±20% EA error costs about ±0.11 mm of common z. The term relies on configured S only to first order and degrades gracefully, consistent with the compliance study's 0.9 mm/k offline slope.

## 2. HP4: unchanged (identical)

HP4 already used the term, and the renamed gate still admits it. The nominal and seed-0 pairs give byte-identical data points and identical frame errors: 0.72 / 0.57 mm, both stopping at `stable-estimate`, at 4926 / 4974 s. Only the dataset timestamp differs ([table](evidence/results/hp4_hp5_hp5r_paired.md)).

## 3. HP5 / HP5r: already modelled; the term would double-count

**Offline check.** [`forced_home_strain.py`](evidence/scripts/forced_home_strain.py) refits stored datasets from the rough-bootstrap session at `47f90e31` with the production two-stage point fit. It forces `(T/S)(d0 + Y/U)` onto A, B and C on top of the per-point flex model ([results](evidence/results/forced_home_strain_hp5_hp5r.jsonl)).

| Dataset | Shipped: max / top z (mm) | Forced term: max / top z (mm) | D free home offset, shipped → forced | rms |
|---|---|---|---|---|
| HP5r nominal | 1.10 / +0.53 | 2.43 / +2.43 | +0.005 → +0.280 | identical |
| HP5r seed 0 | 1.18 / +1.18 | 3.01 / +3.01 | +0.001 → +0.299 | identical |
| HP5 seed 0 | 0.52 / −0.52 | 1.14 / +0.50 | +0.002 → +0.169 | identical |

- **The shipped fit already accounts for home slack.** D's freely fitted home offset is ≈0 in the shipped fit. With the term forced, it rises to match the forced ABC value, which is the duplicated stretch.
- **The error doubles at identical rms.** The double count is a gauge-like translation that only truth reveals.
- **Independent confirmation.** This matches the force-realisation researcher's emulation (branch `t3/autocal-force-realisation` 1e2413f3, item 9): HP5r went to 2.5–3.5 mm with the term, against 0.4–0.9 mm without it.

**Live.** The new gate excludes both machines. HP5 seed 0 and HP5r seed 0 pairs give byte-identical data points and identical results: 0.52 mm and 1.18 mm, both `good-fit`, at 3778 s and 4966 s.

**So the default differs per machine, by physics, not by a flag:**
- HP3/HP4 use the home-strain term;
- HP5/HP5r use the per-point model, which contains it.

HP5r's +1.2 mm top-z error on random seeds is **not** a missing home-strain term. The compliance study's slope scan points to the stiffness scale (8.4 mm per unit k on HP5r sweeps) and force realisation.

## 4. CI benchmark (unperturbed)

`autocal/tests/test_autocal_benchmark.py` stores no expected numbers. It asserts completion and an applied M669, and records metrics to the CI-owned `benchmark-history` ref. So there are no in-repo numbers to update.
- **Expected changes.** On unperturbed machines only HP3 changes: the harness nominal run, which uses the CI command, goes from 1.19 to 0.60 mm print-frame max. HP4/HP5/HP5r are identical per sections 2–3.
- **HP3 heavy test, run locally** (`pytest -m heavy -k hangprinter_3` at `05d6e3d9`): passed, `accuracy_mm` **0.59** against 0.97 mm in CI run 44794416329, `total_sim_s` 4583.7 against 4586.7 s ([row](evidence/results/ci_benchmark_hp3_local.jsonl)).

The next CI benchmark-history entry will show HP3 `accuracy_mm` dropping. This was approved.

## Strongest alternative explanation, and limitations

- **Stopping or trajectory changes rather than the fit.** Four seeds changed sweep count, which changes the trajectory. The gain does not depend on that:
  - the same improvement appears on seeds with identical time (s3: 5045 s in both arms, −0.35 mm; nominal: −0.59 mm);
  - it matches the offline refits on fixed datasets from the compliance study.
- **Configured stiffness.** The term uses configured S, but the harness's ±17% EA draws cost only ≈±0.1 mm. A real machine with a grossly wrong S, for example 12 kN true against 20 kN configured, would carry about 0.36 mm of common z on HP3. Measuring k (the opt-in compliance probe) is the route there. The probe is out of scope here and remains opt-in.
- **Force realisation.** Real motors may realise larger tension errors than the sim's cogging. The term scales with the *commanded* strand tension, so a proportional force error acts like a k error, at the same ≈0.5–0.9 mm per unit. I did not tune anything to the sim's force quirks.
- **Not tested here:**
  - the HP3/HP4 ellipsoid shape. Its meshes are constant-force, not equal-strain, so the gate does not fire there and behaviour is unchanged;
  - HP4 random seeds 1–2 and HP5/HP5r seeds 1–3. Their code path is unchanged: identical data on the pairs run.
- **Hardware.** On hardware, line creep and nonlinearity at measurement tension are unknown; the sim's lines are linear EA/L.

## What would change the conclusion

- A hardware HP3 run whose common low-z error is not near zero with the term would show the slack-at-encoder-zero model does not hold there, for example because of residual home tension or creep.
- Datasets collected without equal strain get no term. That is the intended fallback, but it is untested here.

## Evidence

- **Scripts:**
  - [`queue.sh`](evidence/scripts/queue.sh) runs the paired arms (base from `.base`, a short-path worktree at `338ded23`; RRF rejects long config paths, see the note below);
  - [`hp3_cases.txt`](evidence/scripts/hp3_cases.txt) and [`other_cases.txt`](evidence/scripts/other_cases.txt) are the case lists;
  - [`paired.py`](evidence/scripts/paired.py) builds the tables;
  - [`forced_home_strain.py`](evidence/scripts/forced_home_strain.py) is the offline double-count check.
- **Results:**
  - [hp3_paired.md](evidence/results/hp3_paired.md), [hp4_hp5_hp5r_paired.md](evidence/results/hp4_hp5_hp5r_paired.md);
  - [hp3_stiffness_vs_common_z.txt](evidence/results/hp3_stiffness_vs_common_z.txt);
  - [forced_home_strain_hp5_hp5r.jsonl](evidence/results/forced_home_strain_hp5_hp5r.jsonl).
- **Runs:** `evidence/runs/{base,home}/<case>/` with `result.json`, `imperfection.json`, `manifest.json`, `sweeps.json.gz` and `sweeps.full_auto.log.gz`.
  - Raw headless directories (≈0.13 GB per run) stay local and are uncommitted.
  - [sweeps_sha256.txt](evidence/results/sweeps_sha256.txt) and [raw_run_sizes_bytes.txt](evidence/results/raw_run_sizes_bytes.txt) identify them.
- **Tooling note.** A baseline worktree under the long scratchpad path failed instantly with `bootstrap-collection-failed`. RRF printed "no configuration file found" for a 150-character `config_hp3.g` path, then M569.3 errors. Moving the worktree to a short path fixed it. Other researchers using long worktree paths may hit this.
