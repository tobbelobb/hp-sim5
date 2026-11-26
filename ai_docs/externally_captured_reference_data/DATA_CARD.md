# Data card: DaRUS cable datasets

Practical notes for using the three datasets against hp-sim5. Restore instructions are in
[README.md](README.md). Read the files with `src/python/hp_sim5_research/reference_data.py`
(`read_scope`, `read_tracker`, `dataset_root`, `incomplete`). Nothing in this repository
writes into a dataset directory.

## Rope SKUs across the three datasets (important transfer caveat)

The three datasets share 6 mm 12-strand braided LIROS D-Pro Dyneema rope, **but across two
closely related SKUs — not one identical SKU**:

- **DaRUS-3600 (2023)** and **DaRUS-4614 (COPacabana, 2025):** LIROS D-Pro `01505-0600`, 6 mm
  12-strand Dyneema SK78, standard coating (Reichenbach et al. 2023, section 3.1).
- **DaRUS-5171 (2026 thesis, Table 4.1):**
  - `LIROS_D-Pro`: **LIROS D-Pro XTR**, 6 mm 12-strand Dyneema SK99 with XTR-Grip-Coating
    (56 kN breaking load, 23 g/m);
  - `Trowis_VectranHT_T190PU`: 6 mm 12-strand Vectran HT T190, PU coating (38 kN, 29 g/m);
  - `Trowis_PolyTropFOX`: 6 mm 12-strand Polyester with TPU jacket (9 kN, 32 g/m).

Transferring `(EA, k_r)` identified on DaRUS-5171 `LIROS_D-Pro` (SK99 XTR) into DaRUS-3600 and
DaRUS-4614 (SK78 `01505-0600`) therefore tests transfer across closely related 6 mm 12-strand
Dyneema braids rather than identical production lots.

### Relevance & non-extrapolation warning for Hangprinter ropes

**Do NOT directly apply the 6 mm industrial EA (`118.5 kN`) to Hangprinter fishing line / thin
braided Dyneema (typically 0.5–1.5 mm, multi-layer ungrooved spools).** Each machine sets its own EA.

- Elastic spools store material at its winding strain by default, using the cable's configured
  axial stiffness. This needs no new parameter.
- Elastic spools also contract their running radius with tension by default. A rope pressed onto a
  drum of radius `R` carries line load `F/R`, so its centreline sinks by `F/(R·E_t)` independently of
  rope diameter. The default `k_r = 1.50e-8 m²/N / R` (effective `E_t ≈ 67 MPa`) reproduces the
  identified `0.299 µm/N` (in physical Newtons, `F_true = √2 · f_logged`) on the `R = 50.18 mm`
  D-Pro rig and gives `0.306 µm/N` on COPacabana's 49 mm drums. Coil-on-coil bedding of thin line
  on multi-layer spools is unmeasured; identify it on Hangprinter hardware before trusting absolute
  flattening there.
- Opt out with `strainedSpoolStorage = false` and `spoolRadialFlattening = 0` (world resources or
  `cablePath:` attributes) to isolate effects or reproduce legacy controls. A `cablePath:` value
  `>= 0` in m/N overrides the default per cable.

## File formats

- **TwinCAT Scope exports** (all rig and robot CSVs). Tab separated, CRLF, Latin-1. A header
  block of varying length (6 to 14 lines) gives the channel names in a `Name` row, then
  `Data-Type`, `SampleTime[ms]`, `SymbolName` and other rows. Each channel has its own time
  column (ms), so a row is `t, value, t, value, ...`. Sample time 1 ms on the rigs and 10 ms
  on COPacabana and the 3600 cable-robot profile. `read_scope` takes names from the header and
  checks that all time columns agree.
- **FARO laser-tracker CSVs** (4614 `experiments/tracker`). `;` separated, decimal commas:
  `x; y; z; unix time; 0; 0; 0`, metres, 100 Hz.
- **`.nc`** files are the NC motion programs that produced a run; **`.svdx`** files are TwinCAT
  Scope projects (not needed for analysis).

## Reproducibility & provenance summary

| Dataset | DOI / Version | Generator script | Committed fixture(s) | Split / Role | Units & Key Assumptions |
|---|---|---|---|---|---|
| DaRUS-3600 (CATi, 2023) | `doi:10.18419/DARUS-3600`, V1.0 | `scripts/reference_data/summarize_3600.py`, `extract_3600_pair.py`, `extract_replay_traces.py` | `darus3600_runs.csv` (348 passes), `darus3600_WF125_F120000_torque_pair.npz`, `darus3600_replay_pair.csv` | Cross-dataset blind evaluation (0.5, 2.0, 5.0 m/s; 70–180 N) + measured-encoder replay | Lengths in m, forces in N, speed in m/s; `R_D = 50.18 mm`; pre-wound `init_storage_tension = force`; `dTorque_*` logged at 1/10 scale |
| DaRUS-5171 (Thesis, 2026) | `doi:10.18419/DARUS-5171`, V1.0 | `scripts/reference_data/summarize_5171.py`, `extract_replay_traces.py` | `darus5171_fdp_fc_map.csv` (75 ident runs), `darus5171_stat_valid.csv` (54 held-out valid runs), `darus5171_replay_wc180_dp75.csv` | Ident: `FDP-FC-MAP` (25/rope); Held-out valid: `StatCompensationValid` (18/rope at +2, +3 m/s); Replay: `WC180/Export 1.csv` | `R_D = 50.18 mm`; LSV absolute scale accuracy ±500 ppm (nuisance intercept `b0` frozen from ident split, reported alongside centered RMSE); linear slope fit `np.polyfit(x_T, x_lsv, 1)` |
| DaRUS-4614 (COPacabana, 2025) | `doi:10.18419/DARUS-4614`, V1.0 | `scripts/reference_data/summarize_4614.py` | `darus4614_planeA.csv` (48 rows: 8 conditions × 6 poses) | Static swivel-pulley IK + static stiffness/force-distribution benchmark (all 5 non-home poses × 2 payloads × APC25/50/75) | Geometry & `I = diag(1,1,1)` from `COPacabana.xml` (pulley radii 37/33 mm, swivel axes `±X`); fixed pulley-exit anchors (no winches/dynamics); Kabsch tracker RMSD 0.33 mm |

### Lifecycle & solver design contract

- **Geometric occupancy vs material length:** Each active spool end tracks geometric drum arc
  occupancy (`geom_stored`) separately from unstretched stored material length (`c.stored(li)`).
  Layering radius/theta, hybrid rolling↔attachment transitions, and hybrid-knot synchronization
  use `geom_stored`, while material conservation (`sum(rest) + sum(stored) == totalRestLength`)
  and XPBD strain use unstretched rest length.
- **Effective lever arm (small-strain approximation):** When `k_r > 0`, end-spool constraint rows
  scale the lever arm to `r_eff = max(0.5 * r, r - k_r * F_current)` (instantaneous liftoff running
  radius) for torque-mode Jacobian and reported motor load torque. During unwinding of historical
  coils stored at `F_S != F_current`, `r_eff(F_current)` approximates the finite-difference material
  derivative `dL_0/dtheta` to `O(F_S/EA + k_r * |F_current - F_S| / R)` (`< 0.3 %` in the small-strain
  elastic regime), with shaft-vs-span work ratio governed by the stored elastic strain transition
  (verified in `spool_physics.rs::historical_unwinding_jacobian_and_work_balance`).
- **Explicit initial spool state:** Pre-wound rope on a spool end defaults to `initialStorageTension = 0 N`
  (unstretched) unless explicitly configured per cable (`set_cable_spool_params` in Rust,
  `cablePath:initialStorageTension` in USDA, or `path.initialStorageTension` in JS).
- **Winding-history lifecycle & supported persistence:** Per-cable winding history (`SpoolEnd` LIFO
  stacks, bounded to 256 material-preserving coalesced coils per end) crosses the C ABI via
  `engine_export_spool_state` / `engine_import_spool_state`, and is supported across:
  1. **Resumable numeric checkpoints:** `RustPhysicsEngine.exportCheckpoint()` / `importCheckpoint()`
     (formatVersion 2, capturing regions + per-cable spool histories, failing loudly on invalid or
     corrupted spool state).
  2. **ECS `structureVersion` rebuilds & Native↔WASM backend switches** via `CablePathComponent.spoolHistory`.
  3. **Authoritative JS queries:** `cableGeometricStoredLength()` and `cableStoredLengthAfterRotation()`
     delegate to Rust (`engine_spool_geom_stored` / `engine_predict_spool_stored_after_rotation`) so
     flight-recorder target diagnostics (`cableTargetLengthDelta`) and 3D wrap/knot rendering match the
     solver without duplicating spool physics in JavaScript.
  - **Explicitly unsupported persistence:** Flight-recorder snapshots (`flightRecorderSnapshot.js` /
    `hp_sim5_mcp.py`) are observations, not resumable checkpoints, and USDA scene files only serialize
    initial configuration attributes (`cablePath:spoolRadialFlattening`, `cablePath:strainedSpoolStorage`,
    `cablePath:initialStorageTension`), not mid-run coil stacks.
- **Runtime parameter-change semantics:** Spool parameters are intended to be set at initialization
  or between experiments. Changing `spoolRadialFlattening`, `strainedSpoolStorage` (while spool
  physics remains active), or cable stiffness mid-run preserves existing `(geom, rest)` coils and
  applies the new parameters to subsequent winding and current flattening. Disabling spool physics
  clears the internal coil history (re-enabling later reconstructs from `c.stored(li)` using
  `initialStorageTension`). Changing `initialStorageTension` affects only uninitialized ends or
  subsequent external-edit resyncs.
- **Single solver architecture:** `RustPhysicsEngine` (via `NativePhysicsEngine` or
  `WasmPhysicsEngine`) is the sole physics solver; there is no separate JS reference physics
  implementation (`cable_joints_core.js` only holds ECS components and snapshot helpers).

## DaRUS-3600: cable actuator test rig (CATi), 2023

Complete (393 files). Test winch (W7 in the settings) and load winch (W8) with the rope between
them. Grooved single-layer drums of nominal diameter 100.36 mm (`R_D = 50.18 mm`), groove pitch
7 mm, 5:1 planetary gear on the test winch.

- Grid: `{Torque,Velocity}_based/WF{70,125,180}_F{30000,120000,300000}/{Unwind,Rewind}`. `WF`
  is the target force in N and `F` the feed in mm/min (0.5, 2 and 5 m/s).
- Each export is one pass (`bTrigger_Record` over 10 m constant-speed window).
- `dTorque_test` and `dTorque_load` are logged at **1/10 scale** (`catanalyze.m` multiplies by 10).
- `dForce_test` and `dForce_load`: load side reads higher in both directions (+2 N unwind, +8.7 N
  rewind); treat the inter-load-cell difference as an unexplained calibration offset.
- Optical travel: `dPosition_lsv_enc` (Polytec LSV-2100, ±0.05 % absolute accuracy, 0.02–0.03 %
  repeatability, `ZeroVelocity = 0.15 m/s`) vs `dPosition_asco_enc` (0.3 m/s minimum). Use LSV.

### Stage 1 & cross-dataset validation (`darus3600_rig.rs`)

All force-dependent metrics below are expressed in **physical cable Newtons** (`F_true = √2 · f_logged`,
applied explicitly in Rust while preserving raw logged sensor readings in the committed CSV fixtures):

- Measured pooled LSV/drum slope across all 12 control × direction × speed cells (348 runs):
  **−6.15 ppm/N** (cell slopes −4.92 to −7.15 ppm/N; within-cell repeat RMS 74–164 ppm).
- Blind transfer of DaRUS-5171 `LIROS_D-Pro` parameters (`EA = 118.5 kN`, `k_r = 0.299 µm/N`,
  identified live from `darus5171_fdp_fc_map.csv`) predicts **−5.96 ppm/N** at all three rig
  speeds (0.5, 2.0, 5.0 m/s) against measured speed-cell means −6.54, −5.66, −6.25 ppm/N.
- **Measured-encoder replay** (`darus3600_replay_pair.csv`, 177 N physical / 125 N logged, 2 m/s,
  zero feedforward): upgraded solver reduces span-travel error on both unwind (1169 → 1008 ppm)
  and rewind (2856 → 1093 ppm) and cuts unwind/rewind asymmetry from **1760 ppm** (legacy) to
  **13 ppm** (measured asymmetry: 72 ppm).

## DaRUS-5171: synthetic fibre rope modelling and slip compensation, 2026

Complete (3488 files verified). Three 6 mm 12-strand ropes (`LIROS_D-Pro` SK99 XTR,
`Trowis_VectranHT_T190PU`, `Trowis_PolyTropFOX`) on both the two-winch SWP rig and the 4 m
linear direct-drive SDA (`*Lda`) testbed (1 µm `IMScompact` optical scale + `Kistler 9311C`
piezo load cell).

### SDA (`*Lda`) force scaling quirk
In `*Lda` runs, `f_kistler1`, `f_L`, and `f_Lm` agree to ~1 % and read **~1.43–1.54×** `f_T`
(`sqrt(2)` from the 90° traverse deflection pulley plus ±3.5 % pulley friction). Use `f_kistler1`
as the true span force on SDA.

### Stage 2 & 3 results: identification, 4-way blind ablation, and measured replay (`darus5171_ropes.rs`)

All parameters are identified strictly on `FDP-FC-MAP` in physical cable Newtons (`F_true = √2 · f_logged`)
via linear regression `np.polyfit(x_T, x_lsv, 1)` and evaluated blind on held-out `StatCompensationValid`
(`n = 18` per rope, +2 and +3 m/s) using both **strictly frozen RMSE** (using `b0` frozen from
`FDP-FC-MAP`, zero validation fitting) and **centered RMSE**:

| Rope (6 mm braid) | `EA` (kN) | `k_r` (µm/N) | `b0` (ppm) | Ident RMS | Legacy (frozen / centered) | Strained-only (frozen / centered) | Flattening-only (frozen / centered) | **Full upgraded (frozen / centered)** |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| LIROS D-Pro XTR (SK99) | 118.5 | 0.299 | +1670 | 91 ppm | 2786 / 889 | 1178 / 396 | 1648 / 537 | **158 / 155 ppm** |
| Trowis Vectran HT (LCP) | 139.2 | 0.298 | −6828 | 92 ppm | 2718 / 720 | 1352 / 304 | 1588 / 372 | **290 / 161 ppm** |
| Trowis PolyTropFOX (PES/TPU) | 72.1 | 1.093 | +1270 | 1252 ppm | 6945 / 1294 | 4343 / 540 | 2885 / 366 | **1043 / 980 ppm** |

- On **both bare 12-strand braids** (D-Pro XTR, Vectran HT), the full 2-parameter model beats
  Legacy, Strained-only, and Flattening-only on **both** frozen and centered RMSE (landing inside
  the 100–200 ppm LSV repeat scatter at both +2 and +3 m/s).
- On **TPU-jacketed PolyTropFOX**, although the full model achieves the best strictly frozen
  absolute held-out RMSE (1043 vs 2885–6945 ppm), **both single-effect models achieve better
  centered RMSE** (flattening-only `366 ppm` and strained-only `540 ppm` vs full `980 ppm`).
  This is direct evidence that the 2-parameter linear model does not explain all of the
  TPU-jacketed rope's tension-dependent and viscoelastic run-in behavior (between the `2025-02-21`
  ident split and `2025-03-03` validation split), and is preserved openly without fitting extra
  parameters.
- **True measured motor-trajectory replay** (`darus5171_replay_wc180_dp75.csv`, full 20 s / 28 m
  wind at ~109 N physical / 77 N logged → turnaround → 10 m unwind at ~250 N physical / 177 N logged,
  driving both drums strictly from recorded encoders `(x_T(t), x_L(t))` with **zero feedforward**
  and **zero rest-length edits**):
  - **Measured (physical N):** mean tension `250.1 N`, drift `−7.2 N`, `lsv_ratio = 1.002238`.
  - **Legacy solver:** loses tension during the wind/turnaround and enters unwind nearly slack
    (`30.2 N` mean, `+146.7 N` drift — **219.9 N** mean force error).
  - **Upgraded solver:** tracks the full 28 m trajectory to **`263.2 N` mean** (13.1 N / 5.2 % error),
    `−32.1 N` drift, and `lsv_ratio + b0 = 1.002033` (**206 ppm** error).

## DaRUS-4614: COPacabana static IK & cable-stiffness benchmark, 2025

8-cable parallel robot (`COPacabana.xml`: crossed layout, swivel pulley radii 37 mm upper /
33 mm lower, swivel entry axes `±X`, `I = diag(1, 1, 1)`). Scope in `darus4614_copacabana.rs` is
explicitly bounded to (1) exact swivel-pulley IK verification and (2) static cable-stiffness /
force-distribution benchmarking with fixed pulley-exit anchors (not winch drums or dynamic motion).
All force-dependent comparisons apply `WINCH_FORCE_SCALE = √2` explicitly in Rust (raw logged
values remain untouched in `darus4614_planeA.csv`).

- **Swivel-pulley IK:** Reproduces logged `x_tar[0..7]` across P0..P5 × 8 cables to **0.084 mm**.
- **All-pose blind APC stiffness benchmark** (`EA = 118.5 kN` live from DaRUS-5171 in physical N,
  baseline initialized from actual measured `√2 · lo_row.f[i]` across all 5 non-home poses P1..P5 ×
  both payloads 13.9 & 29.1 kg × APC25/50/75 = 25 pair×pose conditions, 200 cable readings):
  - Overall blind per-cable `dF` RMSE across all 200 readings: **45.3 N** (physical N).
  - Workspace center P1 (well-tensioned, `min_F >= 86 N` physical): 13.9 kg APC75−STD RMSE **20.3 N**
    (mean +105.5 vs +107.2 N); 29.1 kg APC75−APC25 RMSE **22.4 N** (mean +80.9 vs +98.6 N).
  - Low-tension boundary poses P2/P3 (`min_F` down to 1–49 N physical) show noticeably larger `dF`
    over-prediction (RMSE up to 95.6 N at P2 APC75), consistent with low-tension braid softening
    below ~80–110 N.
- **Blind payload-sag residual (29.1 − 13.9 kg):** Because gravity (`m·g`) is already in physical
  Newtons and does not cancel `√2`, updating `EA` from `83.8 → 118.5 kN` reduces simulated
  linear-elastic sag from `−2.62 mm` to **−1.85 mm** vs FARO tracker **−3.69 mm** at P0 (error
  **+1.84 mm**, 49.8 %) and **−4.40 mm** at P1 (error **+2.55 mm**, 58.0 %). Preserved openly as
  an unmodeled residual — strengthening the case for nonlinear low-tension braid softening,
  structural/frame compliance, re-homing preload offsets between `20_Std` and `10_Std`, and
  pulley/traverse friction redistributing load to upper cables.

## Status of planned validation items (completed vs deferred)

- **Completed in PR #129:**
  - DaRUS-3600: 348-run multi-speed validation, cross-dataset blind prediction from 5171, and
    measured-encoder unwind/rewind replay.
  - DaRUS-5171: 3-rope identification (`FDP-FC-MAP`), held-out 4-way ablation (`StatCompensationValid`)
    with both frozen and centered RMSE, and feedforward-free measured-encoder replay (`WC180/Export 1`).
  - DaRUS-4614: Swivel-pulley IK verification, all-pose (P1..P5) / all-APC-level static stiffness
    benchmark, and quantified payload-sag residual.
  - Solver correctness: Geometric occupancy vs material separation across multi-revolution layering,
    effective-radius torque Jacobian, explicit initial storage tension, lifecycle export/import
    across Rust/Native/WASM/ECS rebuilds, adversarial clamp & hybrid-knot invariants.
- **Deferred (future work, explicitly out of scope for PR #129):**
  - Dynamic capstan friction (`DynCapstanFrictionIdent`) and LuGre/viscous winch friction
    identification (`PreloadedLuGreFrictionIdent`) from DaRUS-5171.
  - Thermal slip (`TempSlip`) and dynamic SDA (`*Lda` / `VelDynCompValidLda` / `PowerDynCompValidLda`)
    replay tests.
  - Dynamic COPacabana `CircPath` closed-loop trajectory replay with modeled winches and
    nonlinear low-tension `EA(F)` / frame compliance investigation.

The catalog now includes dynamic replay of the first 13.9 kg standard-control PlaneA cycle with live moving swivel pulleys (`SwivelPulleyAPI`, matching `pulley_ik` to `1e-15 m`), the FARO `Reflector` at its physical bottom-rail height (`z = 0.0225 m`, removing the `0.496232 m` `COP_Ref.csv`-vs-XML Z-origin offset), and the explicit `√2` carriage-pulley sensor factor applied to P0 initial pretension/storage tensions (while preserving raw logged `f[i]` in all CSV fixtures). **Replay targets** drives the eight drums through closed-loop motor dynamics; **Replay encoders** disables motor dynamics (`kinematicMotors`) for exact spool animation (`3.55e-15 rad` max error). Each 10 ms measurement interval is subdivided into 2 ms simulator steps, including dwells, and both replays are covered by automated Jest regression tests (`tests/js/hp-sim-3d/copacabanaPlaneAReplay.test.js`).

### Cross-dataset `SW-G3.1` force-sensor scaling (`3600` / `5171` / `4614`) & DaRUS-4614 quirks

- **Shared `1/√2` carriage-pulley force scaling (`F_true ≈ √2 · f_logged`):** All three DaRUS datasets use Fraunhofer IPA `SW-G3.1` winches with a Tecsis thin-film force sensor (`F2301` / `F2306`) on a 90° carriage deflection pulley (Trautwein et al. 2020 Fig. 4; Reichenbach 2026 §3.4.1). Independent checks confirm the same `~√2` ratio across all three testbeds:
  - **DaRUS-4614 (COPacabana):** P0 static gravity balance at 13.9 kg (`1.393`), 29.1 kg (`1.386`), differential payload (`1.379`), motor electrical power vs `-v[i]·f[i]` (`r = 0.970–0.996` on all 8 axes, mean slope `1.397` + `56–119 N` winch Coulomb friction), and 40-hold force-closure fit (`1.412`, with `~3.6 %` lower 1-pulley vs `~12.8 %` upper 3-pulley column friction).
  - **DaRUS-3600 (CATi):** Test-winch `d(t_test·10)/d(f_test)` unwind/rewind midpoint is `0.07094 Nm/N = 1.4137 · R_D` (`R_D = 0.05018 m`).
  - **DaRUS-5171 (SWP & SDA):** On SWP (`FDP-FC-MAP`), regenerating unwind slope `d(P_T)/d(-v_T·f_T) = 1.201 > 1.0` (physically impossible without `F_true > f_T`; motoring wind slope `1.483`). On SDA (`StatSlipIdentLda`, 75–375 N), `Δf_kistler1 / Δf_T = 1.453` (unwind) and `1.476` (rewind).
  - **Re-identification applied in PR #130:** All three Rust benchmark suites (`darus5171_ropes.rs`, `darus3600_rig.rs`, `darus4614_copacabana.rs`), `DEFAULT_SPOOL_RADIAL_COMPLIANCE` (`0.299e-6 * 0.05018 ≈ 1.50e-8 m²/N`, `E_t ≈ 67 MPa`), and `copacabana.usda` (`EA = 118.5 kN`) have been converted to physical cable Newtons (`WINCH_FORCE_SCALE = √2`), while preserving raw logged sensor readings untouched in all CSV fixtures. All `5171` 4-way ablations, `5171 → 3600` blind transfer (`−5.96` vs `−6.15 ppm/N`), and encoder replays retain their full predictive improvements; on the non-cancelling `4614` gravity-sag test (`m·g` is already in physical N), stiffer `EA = 118.5 kN` reduces simulated elastic sag from `−2.62 mm` to `−1.85 mm` vs FARO `−3.69 / −4.40 mm` (50–58 % under-prediction), reinforcing the role of nonlinear low-tension rope stiffness and structural compliance.
- **Unnormalized `wc[0..5]` in DaRUS-4614:** TwinCAT multiplied `f[0..7]` by unnormalized cable vectors `l_i` (lengths ~3.0–3.7 m) instead of unit vectors `u_i` (read-only dataset artifact; not computed in `hp-sim5`).
- **Uniform `60.5 ms` drive lag in DaRUS-4614:** Recorded encoder payout `x(t)` lags `x_tar(t)` by `60.5 ms` across all eight axes (`0.74–0.99 mm` residual RMS vs `7.9–14.4 mm` raw following error).

[Dynamic comparison, provenance table, constrained ablation, and reproduction commands](../../research/output/session-copacabana-planea/report.md).
