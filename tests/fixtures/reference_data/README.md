# Reference-data fixtures

Small files derived from external measurements, committed so tests and CI need no download.
The raw datasets and how to restore them are described in
[ai_docs/externally_captured_reference_data](../../../ai_docs/externally_captured_reference_data/README.md).

| File | Size | Made by | Content |
|---|---|---|---|
| `darus3600_runs.csv` | 85 KB | `scripts/reference_data/summarize_3600.py` | One row per recorded 10 m constant-speed pass of DaRUS-3600 (348 passes): mean forces, speed, optical/drum travel ratios, torques (rescaled ×10) |
| `darus3600_WF125_F120000_torque_pair.npz` | 35 KB | `scripts/reference_data/extract_3600_pair.py` | One matched unwind/rewind pair (torque-based control, 125 N, 2 m/s), recorded window only, 11 channels decimated to 10 ms |
| `darus5171_fdp_fc_map.csv` | 11 KB | `scripts/reference_data/summarize_5171.py` | All 75 `FDP-FC-MAP` runs across 3 ropes (LIROS D-Pro, Vectran HT T190, PolyTrop FOX; 5 storage tensions × 5 unwinding tensions): mean forces and LSV/drum travel slopes |
| `darus5171_stat_valid.csv` | 8 KB | `scripts/reference_data/summarize_5171.py` | 54 held-out `StatCompensationValid` unwinding runs across the same 3 ropes (3×3 storage × unwinding tensions at +2 and +3 m/s) |
| `darus4614_planeA.csv` | 27 KB | `scripts/reference_data/summarize_4614.py` | COPacabana 8-cable robot PlaneA summary (8 load/control conditions × 6 poses P0..P5): Kabsch-registered FARO laser-tracker poses, repeatability, motor positions `x[0..7]` and cable forces `f[0..7]` |

Regenerate them from a restored dataset with, for example,
`.venv/bin/python scripts/reference_data/summarize_3600.py tests/fixtures/reference_data/darus3600_runs.csv`.
Reading the raw files from the Drive mount takes about 20 minutes for all 3600 runs.

## Source and licence

Derived from three DaRUS datasets from ISW, University of Stuttgart, all licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Values are summarised and decimated
from the original files:

- T. Reichenbach et al., *Measurement data for: Development of a Cable Actuator Test Rig for
  Identification of Dynamic Running Properties of Synthetic Fiber Ropes*, DaRUS, V1.0, 2023,
  [doi:10.18419/DARUS-3600](https://doi.org/10.18419/DARUS-3600).
- T. Reichenbach et al., *Experimental investigation of adaptive preload control for cable-driven
  parallel robots*, DaRUS, 2025, [doi:10.18419/DARUS-4614](https://doi.org/10.18419/DARUS-4614).
- T. Reichenbach, *Measurement data for: Experimental modeling of synthetic fiber ropes for slip
  compensation for application in cable robots*, DaRUS, 2026,
  [doi:10.18419/DARUS-5171](https://doi.org/10.18419/DARUS-5171).
