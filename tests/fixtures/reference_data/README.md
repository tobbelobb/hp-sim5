# Reference-data fixtures

Small files derived from external measurements, committed so tests and CI need no download.
The raw datasets and how to restore them are described in
[ai_docs/externally_captured_reference_data](../../../ai_docs/externally_captured_reference_data/README.md).

| File | Size | Made by | Content |
|---|---|---|---|
| `darus3600_runs.csv` | 85 KB | `scripts/reference_data/summarize_3600.py` | One row per recorded 10 m constant-speed pass of DaRUS-3600 (348 passes): mean forces, speed, optical/drum travel ratios, torques (rescaled ×10) |
| `darus3600_WF125_F120000_torque_pair.npz` | 35 KB | `scripts/reference_data/extract_3600_pair.py` | One matched unwind/rewind pair (torque-based control, 125 N, 2 m/s), recorded window only, 11 channels decimated to 10 ms |

Regenerate them from a restored dataset with, for example,
`.venv/bin/python scripts/reference_data/summarize_3600.py tests/fixtures/reference_data/darus3600_runs.csv`.
Reading the raw files from the Drive mount takes about 20 minutes for all 3600 runs.

## Source and licence

Derived from T. Reichenbach et al., *Measurement data for: Development of a Cable Actuator Test
Rig for Identification of Dynamic Running Properties of Synthetic Fiber Ropes*, DaRUS, V1.0,
2023, [doi:10.18419/DARUS-3600](https://doi.org/10.18419/DARUS-3600), licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The values are summarised and
decimated from the original files.
