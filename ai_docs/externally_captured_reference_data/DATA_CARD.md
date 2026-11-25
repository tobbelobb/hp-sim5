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
- hp-sim5 pays rope off a spool as rest length (`stored_delta`, `Cable::set_stored` in
  `engine.rs`), so at constant tension its span rope runs (1 + F/EA) times the drum surface
  speed: a slope of **+1e6/EA ppm/N**, positive and small. The measured effect is negative and
  several times larger, so the spool–rope interface lacks the dominant real mechanism.
  `src/wasm/cable_solver/tests/darus3600_rig.rs` runs the rig in the solver (two speed-servoed
  drum spools, 10 m at 2 m/s) and confirms +1e6/EA. Its ignored test
  `rig_matches_measured_running_radius` is the target (−8.7 ppm/N pooled within cells);
  run it with `cargo test --test darus3600_rig -- --include-ignored`.
- Run-to-run scatter: LSV ±100 to 200 ppm per run, ASCO ±400 to 700 ppm. Use LSV.
- Speed changes the ratio by up to ±400 ppm with no common trend between the two control modes.
  The paper attributes this to the preload controller's speed offset, so it is not rope physics.
- The intercept (the absolute ratio) is within the sensors' 0.05 % absolute accuracy and must
  not be fitted.

## DaRUS-5171: synthetic fibre rope modelling and slip compensation, 2026

Ropes: `LIROS_D-Pro`, `Trowis_PolyTropFOX`, `Trowis_VectranHT_T190PU`, plus `Winch_Testbed` and
`CableDirectDrive_Testbed` experiments. Each rope has identification runs (`*Ident*`) and
matching validation runs (`*Valid*`):

- friction: `PreloadedViscousFrictionIdent`, `PreloadedLuGreFrictionIdent` and `...Validation`;
- capstan friction under varying force: `DynCapstanFrictionIdent` with `{RW,UW}_{Rising,Falling}`;
- slip: `SlipIdent` (force × velocity maps), `StatSlipIdentLda` (`WC75` ... `WC375`),
  `DynSlipIdentLda`;
- temperature: `TempSlip` (the `theta_T`, `theta_L` channels are temperatures in °C);
- robot-like motion: `CableRobotProfileFullIdent`, validated by `StatCompensationValid`,
  `VelDynCompensationValid` and `PowerDynCompensationValid`, and their `...Lda` variants.

Channels include two Kistler load cells (`f_kistler1`, `f_kistler2`), the traverse load cells
(`f_T`, `f_L`), motor positions and velocities, the same optical sensors as 3600, and in the
2025 runs the authors' own model outputs (`f_lugre`, `f_stribeck`, `f_Tdp`). These outputs are a
published baseline to compare against.

The 5171 rig is not the plain two-winch layout of 3600. In `LIROS_D-Pro/2025-03-07a_StatSlipIdentLda`
at 325 N on the test side, `f_L` and `f_kistler1` read about 487 N (ratio 1.5), so an element with
capstan-like friction sits between the winches. Read the rig description in the thesis PDF before
using these sets for stiffness. `Lda` most likely stands for Last-Dehnungs-Analyse (load-elongation).

While `fetch_darus.py` is running, a directory with a `*.download` file is still being written
(`reference_data.incomplete(path)`); on 2026-11-25 these were the three
`PreloadedViscousFrictionIdent` sets. A directory without one can still gain files until the
fetch finishes.

## DaRUS-4614: COPacabana adaptive preload control, 2025

An 8-cable robot (`COPacabana.xml`: anchor and platform points, pulley radii 0.037 and 0.033 m,
IPAnema3 winches). The robot scope (10 ms) logs `x[0..7]`, `x_tar`, `v`, `v_tar`, `f[0..7]`,
`f_tar`, platform pose `p[0..5]` and `p_tar`, wrench `wc`, and power. The laser tracker logs
platform position at 100 Hz in its own frame; `tracker/00_Lasertracker_Ref` holds the reference
points for registration.

- Experiments: `10_Std_Load29100`, `20_Std_Load13900` (standard control, payloads 29.1 and
  13.9 kg) and `3x_Apc_Load29100_V{25,50,75}`, `4x_Apc_Load13900_V{25,50,75}` (adaptive preload).
- Paths: `PlaneA` (ISO 9283 pose accuracy) and `CircPath` (path accuracy), each with a
  `*_Home_Z600` reference pass.
- The authors' analysis (`main.py`, `analyze.py`) needs `WiPy.pyd`, a Windows binary that cannot
  be used here; geometry has to be read from the XML directly.
- The units of `x[i]` (cable length or motor position) are not yet checked.
