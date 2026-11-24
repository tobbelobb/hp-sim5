---
name: experiment-lookup
description: Find, inspect, and keep reusable hp-sim5 simulation runs from local workspaces, research evidence, and Google Drive. Use before repeating a simulation, benchmark, sweep, or autocal run, or when preserving useful outputs for another agent.
---

Search by run ID, topic, or machine. The lookup checks `output/experiments/`,
research evidence, and Drive when Google credentials are available.

```bash
.venv/bin/python scripts/hp_sim5_run.py artifacts find --machine hangprinter_4 bootstrap
```

Open the returned `metadata_path` for local runs. Look alongside it for inputs,
`spec.json`, datasets, and reports. For stored runs, pass the returned `store_uri`
to `artifacts --store URI manifest ID`, then use `get ID DEST --file spec.json`
or another listed file. Check inputs, parameters, solver, and revision before
reusing a result. For dirty worktrees, look for the patch or exact source used.
Keep useful failures and partial runs when they answer a question or explain a
failure mode.

Keep local runs in `output/experiments/`. For shared Colab evidence, pass
`--store gdrive://hp-sim5-artifacts` to `run` or `batch`. Publish an existing run
with `artifacts --store gdrive://hp-sim5-artifacts push ID RUN_DIR --summary 'Observed result'`.
Use `metadata.summary` in batch specs. Cite the run ID, file, and checksum in reports.

See [simulation infrastructure](../../../human_docs/simulation-infrastructure.md)
for storage defaults, Drive setup, and examples.
