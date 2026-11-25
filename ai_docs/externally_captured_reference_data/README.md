# External reference data

File formats, channel units, sensor limits and known quirks are in [DATA_CARD.md](DATA_CARD.md).

The raw datasets are not stored in Git. Restore their published file trees from DaRUS, version 1.0, with the repository's Python interpreter:

```sh
.venv/bin/python ai_docs/externally_captured_reference_data/fetch_darus.py 3600
.venv/bin/python ai_docs/externally_captured_reference_data/fetch_darus.py 4614
.venv/bin/python ai_docs/externally_captured_reference_data/fetch_darus.py 5171
```

The datasets require approximately 8.2 GB, 1.31 GB, and 170.4 GB respectively, plus temporary space for one file. The script downloads the original individual files (including original CSVs for tabular files), checks their size and published checksum, and verifies existing files when resumed. It retries temporary failures and accepts `--workers` to set the number of concurrent downloads. It refuses to overwrite a differing file. Use `--output /path/to/data` to choose a different writable data directory. The defaults are `data/` inside the corresponding dataset directory. The source ZIP bundles are not needed for reconstruction.

- [Cable actuator test rig, DaRUS-3600](https://doi.org/10.18419/darus-3600)
- [Adaptive preload control experiments, DaRUS-4614](https://doi.org/10.18419/DARUS-4614)
- [Synthetic fibre rope experiments, DaRUS-5171](https://doi.org/10.18419/DARUS-5171)

Each researcher is responsible for following the dataset's published terms. Locally, dataset directories may be symlinks to external storage. Such symlinks are ignored by Git and must be set up separately; a mounted Drive may be read-only, in which case select a writable destination or use a writable Drive client.

DaRUS-4614 contains robot controller and laser-tracker measurements, motion profiles, robot parameters, and Python analysis scripts for adaptive preload control experiments. The [associated 2025 paper](https://doi.org/10.1007/978-3-031-94608-0_17) is listed on the dataset page.
