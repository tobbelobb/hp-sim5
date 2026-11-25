# Data card: DaRUS cable datasets

Practical notes for using the three datasets against hp-sim5. Restore instructions are in
[README.md](README.md). Read the files with `src/python/hp_sim5_research/reference_data.py`
(`read_scope`, `read_tracker`, `dataset_root`, `incomplete`). Nothing in this repository
writes into a dataset directory.

## One rope across all three sets

LIROS D-Pro 01505-0600, 6 mm (a Dyneema braid), is the rope in DaRUS-3600, in the `LIROS_D-Pro`
experiments of DaRUS-5171, and on the COPacabana robot of DaRUS-4614 (Reichenbach et al. 2023,
section 3.1). Parameters identified on one set can therefore be carried to the others without
a material change.

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

## DaRUS-3600: cable actuator test rig (CATi), 2023

Complete (393 files). Test winch (W7 in the settings) and load winch (W8) with the rope between
them. Grooved single-layer drums of nominal diameter 100.36 mm, groove pitch 7 mm, 5:1 planetary
gear on the test winch.

- Grid: `{Torque,Velocity}_based/WF{70,125,180}_F{30000,120000,300000}/{Unwind,Rewind}`. `WF`
  is the target force in N and `F` the feed in mm/min (0.5, 2 and 5 m/s). Each cell has about
  10 exports per direction; the paper used 10.
- Each export is one pass. The NC program winds from 1 m to 18 m and sets `bTrigger_Record`
  between 4 m and 14 m, so the recorded window is 10 m at constant speed. Use
  `Scope.recorded('bTrigger_Record')`.
- Units: positions in m, velocities in m/s, forces in N, power in W.
- `dTorque_test` and `dTorque_load` are logged at **1/10 scale**: the authors' `catanalyze.m`
  multiplies by 10. With that correction the test-winch torque matches force × drum radius
  (0.655 × 10 / 0.049 ≈ 134 N against 125 N target). The load-winch torque and both power
  channels do not yet balance against the measured force; do not use them as ground truth.
- `dForce_test` and `dForce_load` come from compression load cells in the spooling traverses.
  On unwind the load side reads about 2 N higher (expected with pulley friction). On rewind the
  load side still reads higher, by about 8.7 N, which has the wrong sign for friction. Treat
  the force difference as unexplained until a calibration offset is ruled out.
- Optical travel: `dPosition_lsv_enc` (laser surface velocimeter, Polytec LSV-2100) and
  `dPosition_asco_enc` (ASCOspeed). Both are specified at 0.05 % absolute accuracy and 0.02 to
  0.03 % repeatability. Over one 10 m pass they disagree on scale by about 0.13 %, which is the
  same order as the rope strain. Absolute travel ratios are therefore not usable; only
  comparisons with the same sensor across forces and speeds are.
- Sensor validity: the LSV is configured with `ZeroVelocity=0.15` m/s and the ASCOspeed notes
  give a minimum feed of 0.3 m/s. Neither tracks the rope through a reversal or a slow segment.
- `Data/Cablerobot_profile/cablerobot_path.csv` is a single-cable force and length trajectory at
  10 ms (`f_ist[0]`, `l_ist[0]`), not a rig run.

### Stage 1 result (2026-11-25): rope travel per drum travel falls with force

`scripts/reference_data/summarize_3600.py` → `tests/fixtures/reference_data/darus3600_runs.csv`
(348 runs), then `scripts/reference_data/report_3600.py`:

- The LSV/drum travel ratio falls by **7 to 10 ppm per N** in all 12 control × direction × speed
  cells (each slope 11 to 30 standard errors from zero). From 70 N to 180 N that is about 1000 ppm,
  10 mm over the 10 m pass. Winding and unwinding show the same slope, so this is a force-dependent
  running radius (rope flattening in the groove, diameter shrinking under load), not hysteresis.
- hp-sim5's legacy spool model (`options[19..=20] = 0`) pays rope off a spool as rest length at
  fixed radius, giving a positive slope `+1e6/EA ppm/N`. Adding strained spool storage with LIFO
  storage-tension memory (`options[20] = 1`) and tension-dependent radial flattening (`options[19] = k_r`,
  m/N) in `engine.rs` closes this gap: LIROS D-Pro parameters identified purely on DaRUS-5171
  (`EA = 83.8 kN`, `k_r = 0.422 µm/N`) blindly predict `-8.41 ppm/N` against DaRUS-3600's `-8.69 ppm/N`.
  Verified in `src/wasm/cable_solver/tests/darus3600_rig.rs`.
- Run-to-run scatter: LSV ±100 to 200 ppm per run, ASCO ±400 to 700 ppm. Use LSV.
- Speed changes the ratio by up to ±400 ppm with no common trend between the two control modes.
  The paper attributes this to the preload controller's speed offset, so it is not rope physics.
- The intercept (the absolute ratio) is within the sensors' 0.05 % absolute accuracy and must
  not be fitted.

## DaRUS-5171: synthetic fibre rope modelling and slip compensation, 2026

Complete (3488 files verified). Ropes: `LIROS_D-Pro` (6 mm 12-strand Dyneema SK99, 56 kN breaking
load, 23 g/m), `Trowis_VectranHT_T190PU` (6 mm 12-strand Vectran HT T190, 38 kN, 29 g/m, PU coating),
`Trowis_PolyTropFOX` (6 mm 12-strand Polyester with TPU jacket, 9 kN, 32 g/m), plus `Winch_Testbed`
and `CableDirectDrive_Testbed` experiments. Each rope has identification runs (`*Ident*`) and
matching validation runs (`*Valid*`):

- friction: `PreloadedViscousFrictionIdent`, `PreloadedLuGreFrictionIdent` and `...Validation`;
- capstan friction under varying force: `DynCapstanFrictionIdent` with `{RW,UW}_{Rising,Falling}`
  (loading and unloading show the same Euler-Eytelwein slope `exp(mu*beta) - 1`: ~0.23 unwinding,
  ~0.47 rewinding including traverse pulley);
- slip: `SlipIdent` (`V-FC-MAP` force × velocity and `FDP-FC-MAP` 5×5 storage tension `F_S` ×
  unwinding tension `F_L` on the two-winch SWP rig);
- linear direct drive (`Lda` = *Lineardirektantrieb*, SDA testbed): `StatSlipIdentLda` (`WC75` ...
  `WC375`), `DynSlipIdentLda`, `VelDynCompValidLda`, `PowerDynCompValidLda`. Here the load winch
  is replaced by a 4 m Bosch Rexroth linear motor carriage (`ML3P11`) with an `IMScompact` linear
  encoder (±1 µm repeatability, valid at zero speed and through reversals) and an in-line `Kistler
  9311C` piezo load cell (`f_kistler1`).
- temperature: `TempSlip` (the `theta_T`, `theta_L` channels are IR surface temperatures in °C;
  PT1 saturation +6 to +9 K over ~25 min, slip effect ≤5 mm / 10 m on D-Pro and negligible on
  Vectran and PolyTropFOX);
- robot-like motion: `CableRobotProfileFullIdent`, validated by `StatCompensationValid`,
  `VelDynCompensationValid` and `PowerDynCompensationValid`, and their `...Lda` variants.

### SDA (`*Lda`) force scaling quirk
In `*Lda` runs, `f_kistler1`, `f_L`, and the linear motor's own electromagnetic force `f_Lm` all
agree to ~1 % and read **~1.43–1.54×** the test-winch traverse channel `f_T` in both directions
(ratio ~1.44 unwinding, ~1.51 rewinding, i.e. `sqrt(2) ≈ 1.414` from the 90° traverse deflection
pulley plus ±3.5 % pulley friction). Use `f_kistler1` as the true span force on SDA.

### Stage 2 & 3 result (2026-11-25): 2-parameter spool + stiffness law across 3 ropes

Separating winding storage tension `F_S` from unwinding tension `F_L` in `SlipIdent/Data/FDP-FC-MAP`
(`scripts/reference_data/summarize_5171.py` → `tests/fixtures/reference_data/darus5171_fdp_fc_map.csv`)
directly identifies axial stiffness `EA = 1 / c_FL` and drum radial flattening
`k_r = -(c_FS + c_FL) * R_D` (`R_D = 50.18 mm`) for all three ropes:

| Rope (6 mm braid) | `EA` (kN) | `k_r` (µm/N) | `c_FS`, `c_FL` (ppm/N) | Ident RMS | Blind `StatCompensationValid` RMSE (n=18) |
|---|---:|---:|---|---:|---|
| LIROS D-Pro (Dyneema) | 83.8 | 0.422 | −20.35, +11.93 | 91 ppm | 968 → **155 ppm** (+84.0%) |
| Trowis Vectran HT (LCP) | 98.4 | 0.421 | −18.55, +10.16 | 92 ppm | 821 → **160 ppm** (+80.5%) |
| Trowis PolyTropFOX (PES/TPU) | 51.0 | 1.546 | −50.41, +19.61 | 1252 ppm | 1449 → **972 ppm** (+33.0%) |

- Both bare 6 mm 12-strand braids (Dyneema, Vectran) share the exact same radial flattening
  `k_r = 0.42 µm/N` (~1.3 % diameter flattening at 180 N) and land inside the LSV's 100–200 ppm
  repeat scatter on held-out `StatCompensationValid` runs at +2 and +3 m/s.
- Soft TPU-jacketed PolyTropFOX flattens ~3.7× more and has ~1000 ppm viscoelastic/jacket scatter
  even in its identification split (consistent with thesis Table 4.10).
- On `DynSlipIdentLda` (1 µm IMScompact scale, 100–400 N dynamic reversals), loading-vs-unloading
  elongation hysteresis width is small (+0.05 to +0.08 mm on ~2 m span, <1 % of elastic stretch),
  whereas strained spool storage + radial flattening is 10–25 mm over 10 m.
- Verified in `src/wasm/cable_solver/tests/darus5171_ropes.rs`.

## DaRUS-4614: COPacabana adaptive preload control, 2025

An 8-cable robot (`COPacabana.xml`: anchor and platform points, crossed cable layout, pulley radii
0.037 m upper and 0.033 m lower, IPAnema3 winches, LIROS D-Pro 6 mm rope). The robot scope (10 ms
on CircPath, 100 ms on 10_Std PlaneA) logs `x[0..7]`, `x_tar`, `v`, `v_tar`, `f[0..7]`, `f_tar`,
platform pose `p[0..5]` and `p_tar`, wrench `wc`, and power. The FARO laser tracker logs platform
reflector position at 100 Hz in its own frame; Kabsch registration against `00_Lasertracker_Ref`
gives 0.33 mm RMSD.

- **Units and exact pulley kinematics:** `x[i]` and `x_tar[i]` are cable length deltas in **metres**
  relative to platform pose `[0, 0, 0]`. Each base anchor in `COPacabana.xml` is the entry point of
  a swivel pulley with radius `r` and entry axis `[a31, a32, a33]` (`±X`). Implementing this pulley
  IK reproduces logged `x_tar[0..7]` across all 6 PlaneA poses to **0.084 mm** without `WiPy.pyd`.
- **Experiments:** `10_Std_Load29100`, `20_Std_Load13900` (standard control, payloads 29.1 and
  13.9 kg) and `3x_Apc_Load29100_V{25,50,75}`, `4x_Apc_Load13900_V{25,50,75}` (adaptive preload).
- **Stage 4 result (2026-11-25):** `scripts/reference_data/summarize_4614.py` →
  `tests/fixtures/reference_data/darus4614_planeA.csv`, verified in
  `src/wasm/cable_solver/tests/darus4614_copacabana.rs`. Transferring `EA = 83.8 kN` **blind**
  from DaRUS-5171 into an 8-cable `PhysicsWorld` model of COPacabana predicts:
  - P1 APC75 − STD tension rise at 13.9 kg: mean **74.6 N** simulated vs **75.8 N** measured
    (1.6 % error, per-cable RMSE 14.4 N);
  - P1 APC75 − APC25 tension rise at 29.1 kg: mean **57.2 N** simulated vs **69.7 N** measured
    (per-cable RMSE 15.8 N);
  - 29.1 − 13.9 kg vertical elastic sag: **−2.62 mm** simulated (symmetric load sharing) vs
    **−3.69 mm** (P0) and **−4.40 mm** (P1) measured by the FARO laser tracker (larger in hardware
    because lower-cable preload/stiction shifts more of the +149 N delta onto the upper cables, and
    braided Dyneema softens to ~30–45 kN below ~80 N as seen at low-tension poses P2/P3).
- Note on absolute PlaneA/CircPath errors: measured tracker spans show ~−4.5 % Y compression
  (~±20 mm on PlaneA, −84 mm on CircPath) common to both payloads and all modes, while repeat
  scatter is 1.0–1.7 mm; differential comparisons (APC preload deltas and payload-mass deltas)
  cancel this static machine/frame offset cleanly.
