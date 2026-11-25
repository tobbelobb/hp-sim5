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

**Do NOT directly apply the 6 mm industrial EA (`83.8 kN`) to Hangprinter fishing line / thin
braided Dyneema (typically 0.5–1.5 mm, multi-layer ungrooved spools).** Each machine sets its own EA.

- Elastic spools store material at its winding strain by default, using the cable's configured
  axial stiffness. This needs no new parameter.
- Elastic spools also contract their running radius with tension by default. A rope pressed onto a
  drum of radius `R` carries line load `F/R`, so its centreline sinks by `F/(R·E_t)` independently of
  rope diameter. The default `k_r = 2.12e-8 m²/N / R` (effective `E_t ≈ 47 MPa`) reproduces the
  identified `0.422 µm/N` on the `R = 50.18 mm` D-Pro rig and gives `0.432 µm/N` on COPacabana's
  49 mm drums. Coil-on-coil bedding of thin line on multi-layer spools is unmeasured; identify it on
  Hangprinter hardware before trusting absolute flattening there.
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

- Measured pooled LSV/drum slope across all 12 control × direction × speed cells (348 runs):
  **−8.69 ppm/N** (cell slopes −6.96 to −10.11 ppm/N; within-cell repeat RMS 74–164 ppm).
- Blind transfer of DaRUS-5171 `LIROS_D-Pro` parameters (`EA = 83.8 kN`, `k_r = 0.422 µm/N`,
  identified live from `darus5171_fdp_fc_map.csv`) predicts **−8.43 ppm/N** at all three rig
  speeds (0.5, 2.0, 5.0 m/s) against measured speed-cell means −9.25, −8.01, −8.83 ppm/N.
- **Measured-encoder replay** (`darus3600_replay_pair.csv`, 125 N, 2 m/s, zero feedforward):
  upgraded solver reduces span-travel error on both unwind (1169 → 1008 ppm) and rewind
  (2856 → 1093 ppm) and cuts unwind/rewind asymmetry from **1760 ppm** (legacy) to **13 ppm**
  (measured asymmetry: 72 ppm).

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

All parameters are identified strictly on `FDP-FC-MAP` via linear regression `np.polyfit(x_T, x_lsv, 1)`
and evaluated blind on held-out `StatCompensationValid` (`n = 18` per rope, +2 and +3 m/s) using
both **strictly frozen RMSE** (using `b0` frozen from `FDP-FC-MAP`, zero validation fitting) and
**centered RMSE**:

| Rope (6 mm braid) | `EA` (kN) | `k_r` (µm/N) | `b0` (ppm) | Ident RMS | Legacy (frozen / centered) | Strained-only (frozen / centered) | Flattening-only (frozen / centered) | **Full upgraded (frozen / centered)** |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| LIROS D-Pro XTR (SK99) | 83.8 | 0.422 | +1670 | 91 ppm | 2786 / 889 | 1178 / 396 | 1648 / 537 | **158 / 155 ppm** |
| Trowis Vectran HT (LCP) | 98.4 | 0.421 | −6828 | 92 ppm | 2718 / 720 | 1352 / 304 | 1588 / 372 | **290 / 161 ppm** |
| Trowis PolyTropFOX (PES/TPU) | 51.0 | 1.546 | +1270 | 1252 ppm | 6945 / 1294 | 4343 / 540 | 2885 / 366 | **1043 / 980 ppm** |

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
  wind at ~77 N → turnaround → 10 m unwind at ~177 N, driving both drums strictly from recorded
  encoders `(x_T(t), x_L(t))` with **zero feedforward** and **zero rest-length edits**):
  - **Measured:** mean tension `176.9 N`, drift `−5.1 N`, `lsv_ratio = 1.002238`.
  - **Legacy solver:** loses tension during the wind/turnaround and enters unwind nearly slack
    (`21.4 N` mean, `+103.7 N` drift — **155.5 N** mean force error).
  - **Upgraded solver:** tracks the full 28 m trajectory to **`186.1 N` mean** (9.2 N / 5.2 % error),
    `−22.7 N` drift, and `lsv_ratio + b0 = 1.002033` (**206 ppm** error).

## DaRUS-4614: COPacabana static IK & cable-stiffness benchmark, 2025

8-cable parallel robot (`COPacabana.xml`: crossed layout, swivel pulley radii 37 mm upper /
33 mm lower, swivel entry axes `±X`, `I = diag(1, 1, 1)`). Scope in `darus4614_copacabana.rs` is
explicitly bounded to (1) exact swivel-pulley IK verification and (2) static cable-stiffness /
force-distribution benchmarking with fixed pulley-exit anchors (not winch drums or dynamic motion).

- **Swivel-pulley IK:** Reproduces logged `x_tar[0..7]` across P0..P5 × 8 cables to **0.084 mm**.
- **All-pose blind APC stiffness benchmark** (`EA = 83.8 kN` live from DaRUS-5171, baseline
  initialized from actual measured `lo_row.f[i]` across all 5 non-home poses P1..P5 × both
  payloads 13.9 & 29.1 kg × APC25/50/75 = 25 pair×pose conditions, 200 cable readings):
  - Overall blind per-cable `dF` RMSE across all 200 readings: **31.1 N**.
  - Workspace center P1 (well-tensioned, `min_F >= 61 N`): 13.9 kg APC75−STD RMSE **14.4 N**
    (mean +74.6 vs +75.8 N); 29.1 kg APC75−APC25 RMSE **15.8 N** (mean +57.2 vs +69.7 N).
  - Low-tension boundary poses P2/P3 (`min_F` down to 1–34 N) show noticeably larger `dF`
    over-prediction (RMSE up to 67.6 N at P2 APC75), consistent with low-tension braid softening
    below ~60–80 N.
- **Blind payload-sag residual (29.1 − 13.9 kg):** Simulated **−2.62 mm** vs FARO tracker
  **−3.69 mm** at P0 (error **+1.07 mm**, 29.0 %) and **−4.40 mm** at P1 (error **+1.79 mm**,
  40.5 %). Preserved openly as an unmodeled residual; candidate hypotheses (not proven causes)
  include low-tension braid softening, re-homing preload differences between `20_Std` and `10_Std`,
  pulley/traverse friction shifting load to upper cables, and finite frame compliance.

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

The catalog now includes dynamic replay of the first 13.9 kg standard-control PlaneA cycle, using either the eight target channels or the eight encoder channels. Each 10 ms measurement interval is subdivided into 2 ms simulator steps, including dwells. The tracker overlay follows the registered reflector in the P0-relative frame. [Dynamic comparison and reproduction commands](../../research/output/session-copacabana-planea/report.md).
