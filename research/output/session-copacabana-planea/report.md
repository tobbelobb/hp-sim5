# COPacabana dynamic PlaneA replay

The first 24.26 s of the 13.9 kg standard-control run separate drive tracking, swivel-pulley kinematics, proximal force-sensor scaling, and hardware geometry mismatch. With live moving swivel pulleys, the reflector at its physical bottom-rail height (`z = 0.0225 m`), physical-Newton rope parameters (`EA = 118.5 kN`, default radial compliance `1.50e-8 m²/N`), and the confirmed `√2` carriage-pulley sensor scaling applied explicitly to P0 initial pretension/storage tensions (preserving raw logged `f[i]` in all fixtures), feeding the eight recorded motor targets through closed-loop motor dynamics gives **25.5 mm** RMS reflector error (`50.0 mm` peak) and **173.0 N** raw / **258.2 N** `√2`-scaled cable-force RMS. Replaying the recorded encoder readings directly with motor dynamics disabled (`max_spool_tracking_error_rad = 3.55e-15 rad`) reduces reflector error to **16.0 mm** RMS (`22.5 mm` peak) and **172.4 N** raw / **258.0 N** `√2`-scaled cable-force RMS.

**Almost all of the settled error belongs to the real machine's anisotropic geometry mismatch and proximal force-sensor/friction chain, not to the simulated cables.** At every settled pose the simulated reflector lies within **1.6 mm** of the controller's commanded pose on every axis (within **1.3 mm** at P1–P4). The measured reflector misses that same commanded pose by up to **20.6 mm**, almost entirely along Y. Model-free, registration-independent raw tracker spans confirm an anisotropic discrepancy across every experiment and trajectory:
- **PlaneA (all 8 experiments):** `P2→P3` and `P4→P5` (pure 899.1 mm Y moves) travel **3.5–4.6 % short** (`0.954–0.966×`), while `P3→P4` (1674.4 mm `-X-Z` move) travels **0.4–0.5 % long** (`1.004–1.005×`).
- **CircPath (13.9 kg and 29.1 kg):** Y bounding span (1751.0 mm commanded) travels **4.8–5.7 % short** (`−83.7` to `−99.6 mm`), while X span (1669.9 mm) travels **0.7–0.9 % long** (`+11.4` to `+15.7 mm`) and Z span (526.5 mm) travels **0.6 % long** (`+3.1 mm`).

Because the shortfall persists across payloads and preload modes and has opposite sign on Y vs X/Z, neither linear cable elasticity nor a uniform encoder payout scale alone can explain it. The published `COPacabana.xml` geometry and transmission are **preserved untouched** as the reproducible baseline in `copacabana.usda` rather than overwritten by an unconstrained fit; see [Constrained calibration ablation](#constrained-calibration-ablation) below.

## Parameter provenance

Reichenbach et al. (2024 APC paper, §3) note that COPacabana's frame and platform geometry were modified during reconstruction and published as `COPacabana.xml` (verified against logged `x_tar[0..7]` to `0.084 mm` via swivel-pulley IK, proving it matches the controller's internal model). Every parameter in `copacabana.usda` falls into one of four provenance tiers:

| Parameter | Baseline value in `copacabana.usda` | Provenance |
|---|---|---|
| Frame swivel-pulley entries, radii (`37 / 33 mm`), axes (`±X`) | Exact from `COPacabana.xml` | Published controller/CAD XML (preserved) |
| Platform attachments `AttachA..O`, mass (`13.9 kg`), CoG `(0,0,0)`, `I = diag(1,1,1)` | Exact from `COPacabana.xml` | Published controller/CAD XML (preserved) |
| Drum radius (`0.049 m`), gearbox (`5:1`), motor torque (`32 × 5 = 160 Nm`) | Exact from `COPacabana.xml` (`wegaufz/wegaufn` implies `49.10 mm`; `r_drum = 0.049 m` cancels in step conversion) | Published controller XML (preserved) |
| Home `Reflector` offset `(0.006832, −0.198033, 0.022500) m` | Kabsch-registered P0 `(X, Y)` + bottom 45×45 mm rail centerline `z = 0.0225 m` (`cad/platform.stl`, DaRUS-4614 photos) | Confirmed physical/coordinate fix (`0.496232 m` `COP_Ref`-vs-XML Z-origin offset removed) |
| Initial span & storage tensions `F0` | Raw P0 `f[0..7] × √2` (`[90.51, 84.99, 90.51, 84.15, 40.16, 34.93, 35.64, 30.83] N`) | Confirmed `1/√2` 90° carriage-pulley sensor correction (raw `f[i]` preserved in CSV fixtures) |
| Cable `EA = 118.5 kN`, radial compliance `k_r = 1.50e-8 / R` (`0.306 µm/N` at `49 mm`) | Re-identified from DaRUS-5171 6 mm D-Pro XTR in physical Newtons (`F_true = √2 · f_logged`) | Independent rig ID (`DATA_CARD.md`) |
| Rotor inertia (`0.005 kg·m²`), damping (`0.11`), `0.2 m` drum offset | Unidentified simulation assumptions (bypassed in kinematic **Replay encoders**) | hp-sim5 simulation assumptions |

## Replay in hp-sim-3d

Select **COPacabana (13.9 kg)** as the only machine. **Replay targets** drives the eight drums from the logged cable-position targets through the simulated motor dynamics (turn on **Closed loop motors** and turn off line layering to reproduce the target comparison). **Replay encoders** disables motor dynamics (`kinematicMotors`) and animates all eight drums directly to the measured encoder angles (`3.55e-15 rad` max tracking error), independently of the closed-loop motor toggle. The white trace follows the `Reflector` sphere on the front bottom `-Y` aluminum rail (`z = 0.0225 m`). The blue path shows where the FARO tracker measured that reflector. The smaller sphere marks the payload center of mass, and the eight anchor pulleys swivel live around their `±X` entry axes as the platform moves. The nominal PlaneA G-code, Logo and Squares go through the catalogued firmware planners. The experiment buttons use a separate translation of the 100 Hz robot log, preserving reversals and all five stationary intervals.

## Physical assumptions

The scene uses the eight crossed attachments, 13.9 kg mass, center of gravity (`(0, 0, 0)` at P0), inertia tensor (`I = diag(1, 1, 1)`), and eight swivel-pulley entry points, radii (`37 mm` upper, `33 mm` lower), and `±X` swivel axes directly from `COPacabana.xml`. Its origin is the platform reference at P0, 0.6 m above the XML origin. Each `SwivelPulley` is simulated as a rolling wheel whose fork turns live about its entry axis toward the outgoing platform span, matching analytical WireX/IPAnema `pulley_ik` to `1e-15 m` while conserving cable material. The simulated `Reflector` sits at `(0.006832, −0.198033, 0.022500) m` — the front T-slot centerline of the bottom 45×45 mm `-Y` aluminum rail (`z ∈ [0, 0.045] m`, front face `y = −0.1725 m` in `cad/platform.stl`, aligned with `AttachA..D` at `z = 0.0225 m`), where the FARO 1.5" SMR nest is bolted in the DaRUS-4614 lab photos. Each drum has a 49 mm radius and feeds its swivel pulley along the swivel axis from 0.2 m back. Initial span and storage tensions apply the explicit `√2` carriage-pulley sensor factor to the eight raw P0 force readings (`[64.0, 60.1, 64.0, 59.5, 28.4, 24.7, 25.2, 21.8] N × √2`), bringing home vertical force (`138.4 N`) into static balance with the 13.9 kg weight (`136.4 N`, vs `97.9 N` unscaled).

The motion source is Export 1 in `robot/20_Std_Load13900/PlaneA`. The converter differences rounded cumulative displacements, with 3,200 microsteps per equivalent drum revolution, and encodes those intervals into the simulator's compact CAN format. The source machine ran TwinCAT; comparing `x(t)` against `x_tar(t)` shows a uniform **60.5 ms** CNC/drive tracking lag across all eight axes (`0.74–0.99 mm` residual RMS vs `7.9–14.4 mm` raw following error). The FARO and controller clocks are independent. We register coordinates from `00_Lasertracker_Ref/COPacabana_Ref/COP_Ref.csv`, remove the constant `0.496232 m` Z-origin offset between `COP_Ref.csv` and the XML homing frame so the P0 SMR sits at `z = 0.0225 m`, and subtract one constant time offset estimated from the measured reflector and controller pose crossing the five leg midplanes (`1.5331–1.5412 s`, median `1.5357 s`). Absolute transport latency cannot be distinguished from clock offset. [Replication data, Reichenbach, Clar, Pott and Verl, CC BY 4.0](https://doi.org/10.18419/DARUS-4614).

## Measured agreement

The validator samples predicted XYZ at the bottom-rail `Reflector` and the platform-side constraint force on each cable across 1,214 synchronized samples at 50 Hz. Cable forces exclude the initialization sample and are reported against both raw logged `f[i]` and `√2 · f[i]`. Both replays are covered by automated Jest regression tests in `tests/js/hp-sim-3d/copacabanaPlaneAReplay.test.js`.

| Input or predictor | Motor dynamics | Max spool tracking err (rad) | Reflector RMS (mm) | Peak (mm) | Force RMS vs raw `f[i]` (N) | Force RMS vs `√2·f[i]` (N) |
|---|---|---:|---:|---:|---:|---:|
| Recorded motor targets (`planeATargets`) | Closed-loop servo | `2.35e-2` | 25.5 | 50.0 | 173.0 | 258.2 |
| Recorded motor encoders (`planeAEncoders`) | Disabled (kinematic) | `3.55e-15` | 16.0 | 22.5 | 172.4 | 258.0 |
| Logged Cartesian target plus fixed reflector offset | — | — | 27.0 | | | |
| Hold each cable at its measured P0 force | — | — | | | 184.8 | |

The table below uses stationary intervals beginning 0.5 s after the platform target stops (`planeAEncoders`, moving swivel pulleys, kinematic motors, bottom-rail reflector, physical-Newton `EA = 118.5 kN` and `√2`-scaled P0 pretension). Repeat scatter is the root sum of the tracker coordinate variances across repeated visits in the earlier PlaneA fixture.

| Pose | Encoder replay reflector RMS (mm) | Simulated − commanded XYZ (mm) | Measured − commanded XYZ (mm) | Repeat scatter (mm) | Force RMS raw / `√2` (N) | Closure raw / `√2` (N) |
|---|---:|---:|---:|---:|---:|---:|
| P1 | 1.7 | (0.0, 0.0, 0.3) | (−0.5, 0.3, 1.9) | 0.63 | 60 / 102 | 127 / 127 |
| P2 | 16.1 | (−1.3, −0.9, −0.1) | (1.0, −16.8, −0.7) | 0.66 | 17 / 16 | 64 / 54 |
| P3 | 18.8 | (−0.9, 0.7, −0.2) | (3.7, 17.5, 6.8) | 0.70 | 19 / 23 | 77 / 62 |
| P4 | 19.9 | (1.1, 1.2, 0.7) | (−4.3, 19.2, 7.0) | 0.40 | 300 / 446 | 180 / 242 |
| P5 | 20.4 | (1.6, −0.7, 0.8) | (−2.7, −20.6, 0.6) | 0.48 | 212 / 322 | 92 / 90 |

![Settled reflector error and measured repeat scatter by pose](settled-error.png)

### Force-channel audit

Auditing the raw TwinCAT Scope logs (`Export 1.csv` across PlaneA and CircPath, 13.9 kg and 29.1 kg) together with Trautwein et al. (2020) and Reichenbach's 2026 thesis resolves the apparent static-equilibrium failure into four concrete effects:

1. **Channel mapping is verified 1-to-1 (no permutation).** Correlating motor electrical power `power[i]` against mechanical cable power `-v[i]·f[i]` across PlaneA yields `r = 0.970–0.996` on every single axis `i ∈ 0..7` (both payloads), ruling out force-channel miswiring.
2. **A `1/√2` carriage-pulley scale factor (`F_true ≈ √2 · f_logged`).** Figure 4 of Trautwein et al. (2020) and §3.4.1 / Abb. 3.13 of Reichenbach (2026) show that each Fraunhofer IPA `SW-G3.1` winch measures force via a thin-film sensor (`Tecsis F23061320001` / `F2301`) on a **90° carriage deflection pulley** between the drum and frame guidance. Five independent checks in DaRUS-4614 converge on `F_true / f_logged ≈ √2 = 1.414`:
   - P0 static gravity balance at 13.9 kg (`136.4 / 97.9 N` = **`1.393`**),
   - P0 static gravity balance at 29.1 kg (`285.5 / 206.0 N` = **`1.386`**),
   - Differential payload (`29.1 − 13.9 kg`) at P0 (`149.1 / 108.1 N` = **`1.379`**),
   - Motor electrical power slope `power[i]` vs `-v[i]·f[i]` across all 8 cables (`[1.464, 1.487, 1.429, 1.327, 1.465, 1.432, 1.226, 1.347]`, mean **`1.397`**, plus `56–119 N` per-axis winch/gearbox Coulomb friction),
   - Static force-closure fit across all 40 PlaneA holds (`g = 1.412`).
   - **Cross-dataset check & physical-Newton re-identification on DaRUS-3600 & DaRUS-5171:** The same `SW-G3.1` winch family exhibits the identical `~√2` scaling in DaRUS-3600 (`d(t_test·10)/d(f_test)` unwind/rewind midpoint = `0.07094 Nm/N = 1.4137 · R_D`), DaRUS-5171 SWP (regenerating unwind slope `d(P_T)/d(-v_T·f_T) = 1.201 > 1.0`), and DaRUS-5171 SDA (`Δf_kistler1 / Δf_T = 1.453` unwind / `1.476` rewind against the calibrated Kistler 9311C). All three Rust validation suites (`darus5171_ropes.rs`, `darus3600_rig.rs`, `darus4614_copacabana.rs`), `DEFAULT_SPOOL_RADIAL_COMPLIANCE` (`0.299e-6 * 0.05018 ≈ 1.50e-8 m²/N`, `E_t ≈ 67 MPa`), and `copacabana.usda` (`EA = 118.5 kN`) now operate in physical cable Newtons (`WINCH_FORCE_SCALE = √2`), while preserving raw logged sensor readings untouched in all CSV fixtures; see `DATA_CARD.md` for full re-validated numbers including the non-cancelling gravity-sag test (`−1.85 mm` simulated vs `−3.69 / −4.40 mm` FARO).
3. **Direction-dependent asymmetric upper vs lower cable-guidance friction.** Lower cables route over one frame pulley between winch and platform, while upper cables route up the vertical frame columns over three pulleys. Actuator-side readings therefore exceed platform tension during retraction (`F_act ≈ F_plat · (1 + η)`) and undershoot it during payout (`F_act ≈ F_plat / (1 + η)`). Fitting all 40 PlaneA holds with `g = 1.412`, lower loss `η_lo = 3.6 %`, and upper column loss `η_up = 12.8 %` collapses the static force-closure residual from **133.4 N to 46.2 N** (Trautwein et al. 2020 §4.1 already cautions that actuator sensors provide only qualitative platform forces).
4. **Unnormalized `wc[0..5]` in TwinCAT Scope & angular-unit audit.** Regressing the logged wrench `wc[0..5]` against `f[0..7]` at P0/P1 fits to `1.2e-12` with per-cable multipliers equal to the cable lengths `‖l_i‖` (~3.0–3.7 m): TwinCAT multiplied `f` by unnormalized cable vectors `l_i` instead of unit vectors `u_i` (a read-only dataset quirk; `hp-sim5` does not compute `wc`). In `evidence/force_balance.py`, passing `degrees=True` to `Rotation.from_euler('XYZ', ...)` matches the DaRUS-4614 metadata (verified `< 0.0007 N` change on PlaneA and `< 0.0029 N` on CircPath because logged `max|p[3..5]| ≤ 8.4e-5`). [Raw-log calculation](evidence/force_balance.py)

| Mean tension at P4 (N), encoder replay | A | B | C | D | I | J | L | O |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Logged (`f[i]`, unscaled actuator sensor) | 316 | 133 | 200 | 390 | 348 | 303 | 345 | 605 |
| Simulated at platform (XML geometry, `EA = 118.5 kN`, `F0 = √2·f[0]`) | 78 | 53 | 73 | 100 | 35 | 22 | 47 | 66 |

### Constrained calibration ablation (ongoing investigation)

To test which parameters genuinely improve out-of-sample prediction — rather than overfitting one run — we progressively relaxed parameters while training **only** on `20_Std_Load13900` PlaneA (P1..P5) and evaluating **blind** on all **7 held-out PlaneA experiments** (35 holds across 13.9 & 29.1 kg × STD/APC25/50/75) and both **held-out `CircPath` runs** (48 active trajectory points, `max f_log = 885 N`, independently confirmed real by `+958 W / −505 W` motor electrical power). Force RMS is always evaluated in logged-sensor units (`F_pred_act / g − f_log`) so the optimizer cannot reduce error by shrinking `g`.

| Stage (progressively relaxed) | Train PlaneA Pos / F | 7 Held-out PlaneA Pos / F | Held-out CircPath Pos / F | Fitted parameters (`d0` mm, `η = (lo, up)`) |
|---|---:|---:|---:|---|
| **A0:** Published XML, `EA = 83.8 kN`, `g = 1.0` | `17.76 mm` / `178.2 N` | `17.59 mm` / `184.2 N` | `23.70 mm` / `172.6 N` | baseline |
| **A1:** Published XML, `EA = 118.5 kN`, `g = √2` | `17.66 mm` / `176.9 N` | `17.48 mm` / `183.6 N` | `24.29 mm` / `170.5 N` | `EA = 83.8·√2`, `g = √2` |
| **B:** + fit `EA` (`g = √2`) | `17.29 mm` / `175.3 N` | `16.79 mm` / `168.7 N` | `23.32 mm` / `171.3 N` | `EA = 78.4 kN` |
| **C:** + fit `EA, g, η_lo, η_up` | `16.57 mm` / `173.2 N` | `15.71 mm` / `159.2 N` | `22.38 mm` / `171.7 N` | hits bounds (`g=2.0, η=35%`) |
| **D:** + fit home offset `d0` (`g = √2`) | `19.82 mm` / `52.8 N` | `19.90 mm` / `55.8 N` | `30.65 mm` / `102.8 N` | `EA = 149 kN`, `η = (10%, 10%)`, `d0 = (−23, −9, −37)` |
| **E:** + fit encoder scale `s_enc` (`g = √2`) | `9.95 mm` / `41.6 N` | `10.14 mm` / `50.1 N` | `15.52 mm` / `97.9 N` | `EA = 174 kN`, `η = (7.5%, 13.4%)`, `d0 = (−4, −5, −79)`, `s = 0.978` |
| **F2:** + fit anchor scale ONLY (`s_enc = 1`, `g = √2`) | `9.10 mm` / `40.0 N` | `9.19 mm` / `47.0 N` | `14.72 mm` / **`69.4 N`** | `EA = 185 kN`, `η = (8.7%, 15.9%)`, `d0 = (0, −5, −107)`, `sc = (1.002, 0.996, 0.938)` |
| **F1:** + fit `s_enc` and anchor scale (`g = √2`) | `2.56 mm` / `35.7 N` | **`2.74 mm` / `39.4 N`** | **`9.55 mm`** / `96.1 N` | `EA = 95.8 kN`, `η = (6.6%, 17.2%)`, `d0 = (+5, −5, −178)`, `s = 0.984`, `sc = (0.980, 0.976, 0.927)` |

**Key conclusions from the ablation:**
1. **Elasticity, force scaling, and pulley friction alone (Stages A–C) cannot resolve either position (`16–24 mm`) or force (`159–184 N`) errors.**
2. **A home offset alone (Stage D) collapses held-out force RMS by 3.3× (`184 → 55.8 N` on PlaneA, `173 → 103 N` on CircPath)** without altering anchor widths or payout scale, confirming that the P4/CircPath tension spikes come from geometric over-constraint relative to the homing pose — yet `d0` alone cannot fix the 3.5–5.7 % Y-travel shortfall.
3. **Anchor-width changes are strongly collinear with `s_enc` and `d0_z`:** Holding `s_enc = 1.000` (Stage F2) leaves `sx = 1.002, sy = 0.996` (within 0.4 % of `COPacabana.xml`!) and achieves the **lowest held-out CircPath force RMS (`69.4 N`)**, shifting only vertical span (`sz = 0.938`, `d0_z = −107 mm`). Allowing both `s_enc` and `(sx, sy, sz)` (Stage F1) reaches `2.74 mm` on held-out PlaneA and `9.55 mm` on CircPath, but trades off CircPath force accuracy (`69.4 → 96.1 N`).
4. **Therefore we do NOT overwrite `COPacabana.xml` geometry or transmission in `copacabana.usda`.** Reconciling the exact physical anchor/homing geometry requires independent survey measurements of the anchor points in the `COP_Ref.csv` frame (Trautwein et al. 2020 §3.1/§4.1 note that COPacabana anchors are reconfigured using 1D beam distances into CAD rather than full 6-DOF re-calibration). [Autocal handoff](autocal-handoff.md)

## Reproduce

From the repository root, run the following commands. Both validations use the bundled WASM module, machine scene, motion streams and observations.

```sh
node scripts/reference_data/validate_4614_replay.mjs planeATargets
node scripts/reference_data/validate_4614_replay.mjs planeAEncoders
npx jest tests/js/hp-sim-3d/copacabanaPlaneAReplay.test.js
```

To reproduce the raw-log audit across eight conditions, five poses and 30 exports, pass the cache directory (or the original `experiments/` directory) to `.venv/bin/python research/output/session-copacabana-planea/evidence/force_balance.py output/reference_cache/4614`. It writes one CSV row per settled visit to stdout.

To reproduce the constrained calibration ablation (Stages A0–F2, trained on `20_Std_Load13900` PlaneA and evaluated blind on the 7 held-out PlaneA experiments and both `CircPath` trajectories with explicit weights `1.0 mm` / `12.0 N` logged force and parameter bounds), run `.venv/bin/python research/output/session-copacabana-planea/evidence/calibration_ablation.py` (pass `--planea-only` if `CircPath` Scope/tracker files are not mounted).

With the raw measurements mounted, rebuild the comparison inputs with `.venv/bin/python scripts/reference_data/extract_4614_replay.py`. Recompile the nominal firmware examples with `.venv/bin/python scripts/compile_machine_commands.py --machine copacabana`.

The earlier strained-storage-only fixed-tangent replay is archived under run ID `copacabana-planeA-preload` at simulation revision `2bc87ece`. The updated bottom-rail observation fixture SHA-256 is `3cf5f146481d9246e3225d7a88d056d4b52c4706b8f21763fec86c6a7ec8bed6`. The target and encoder streams have SHA-256 prefixes `ca921001a3818ffd` and `7e912375d1d57628`.
