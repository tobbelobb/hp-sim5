# hp-sim5 Simulation Infrastructure

Portable architecture for running headless `hp-sim5` experiments locally or on Google Colab, with verified persistent artifact storage locally or on Google Drive.

## Architecture Overview

Five decoupled layers live under [`src/python/hp_sim5_sim/`](../src/python/hp_sim5_sim/):

1. **Experiment Control** ([`interfaces.py`](../src/python/hp_sim5_sim/interfaces.py))
   - Portable, JSON-serializable `ExperimentSpec` (`autocal`, `sweep_collection`, `command`) and `ExperimentResult`.
   - `ExperimentController` protocol decouples adaptive control decisions (G-code moves, settling waits, active-learning plans) from physics transport.
2. **Firmware** ([`firmware.py`](../src/python/hp_sim5_sim/firmware.py))
   - `FirmwareService` protocol and `RrfFirmwareService` managing isolated `rrf_simulator --server` instances with short VSD paths, free port allocation, readiness checks, and process-group cleanup.
   - `RRF/host/src/main.cpp` and `SocketHost`/`NetworkInterfaceHost` now update `HostIdle` immediately on socket activity and per spin instead of polling every 100 ms, cutting RRF host CPU usage by ~10× while keeping simulation outputs bit-identical.
3. **Physics** ([`scripts/autocal_headless.mjs`](../scripts/autocal_headless.mjs), [`hp-sim-3d/app/headlessWorld.js`](../hp-sim-3d/app/headlessWorld.js))
   - Direct in-process `RrfHttpBridge` wiring removes the self-WebSocket loopback serialization previously used per 2 ms command frame.
   - Buffered `events.jsonl` writes and throttled `clock.json` snapshots eliminate thousands of synchronous writes/renames per sweep while keeping identical event traces and snapshots.
   - Explicit solver selection via `HP_SIM5_PHYSICS_SOLVER=native|wasm` (default `native`, no silent fallback).
4. **Execution Scheduling** ([`runner.py`](../src/python/hp_sim5_sim/runner.py), [`colab.py`](../src/python/hp_sim5_sim/colab.py))
   - `ExecutionScheduler` protocol implemented by `LocalScheduler` (local parallel CPU pool) and `ColabScheduler` (remote Google Colab VMs via official `google-colab-cli`).
   - `ColabScheduler` bundles the workspace once (with optional portable `x86-64` `physics.node`), bootstraps minimal headless Node deps (`@kroxilon/usda-parser`, `ws`) and compiles `rrf_simulator` once on the VM, then executes all experiments in parallel **entirely inside the Colab VM** with zero per-G-code internet traffic.
   - **Artifact transfer & opt-in direct Colab → Google Drive uploads:** By default, Colab stages results back through the local host so no Drive credentials are ever sent to Colab. Pass `--direct-drive` with `--store gdrive://...` to upload each finished experiment directly from Colab to Google Drive using only a **short-lived OAuth access token** (no refresh token or client secret is ever transferred, and the handoff token file is deleted before experiments spawn).
   - **Safe persistence, lean transfers & verified shutdown:** Colab runs without `--store` or `--keep-scratch` persist results under `output/experiments/` on the host. On successful `recording_mode=none` runs, bulky `*.headless*/events.jsonl` traces (~70+ MB per sweep) and `.matplotlib/` caches are omitted from uploads while keeping `sweeps.json`, `manifest.json`, `rrf.log`, and scene inputs (~200 KB total); full `events.jsonl` is automatically retained on failures, when `recording_mode != "none"`, or when `metadata.keep_events_log=true`. Any artifact upload failure preserves local scratch with `upload_pending.json` and prints an exact `artifacts push` retry command. If remote execution fails or times out mid-batch, `ColabScheduler` performs a best-effort remote rescue before stopping the VM, recovering completed experiments (persisted to the store) and any partial scratch directories locally. Failed `colab stop` calls retry, verify with `colab sessions`, preserve `sessions.json`, and raise `ColabShutdownError` with the manual stop command (after persisting any already-downloaded results).
5. **Artifact Storage** ([`storage.py`](../src/python/hp_sim5_sim/storage.py))
   - `ScratchWorkspace` guarantees active simulation, telemetry, and Rerun recording stay on fast local scratch storage.
   - `ArtifactStore` protocol implemented by:
     - `LocalArtifactStore` (`local:///path/to/store`) — serialized `index.jsonl` updates (`threading.Lock` + `fcntl.flock`) safe for parallel threads and processes.
     - `DriveMountArtifactStore` (`drivemount:///content/drive/MyDrive/hp-sim5-artifacts`)
     - `GoogleDriveArtifactStore` (`gdrive://hp-sim5-artifacts`) — pure-stdlib Google Drive REST API v3 with 8 MiB resumable chunked uploads, 308 resume-after-failure recovery, exponential backoff on 429/5xx, SHA-256/MD5 checksum verification, deduplication, and lock-free per-experiment folder `appProperties` indexing (no concurrently overwritten shared `index.json`).

## Future GPU-Batching Design

[`interfaces.py`](../src/python/hp_sim5_sim/interfaces.py) defines `WorldStepRequest`, `ObservationBatch`, `BatchedPhysicsBackend`, and `ExperimentController` so a future GPU physics backend can batch `B` independent, adaptive simulations without changing callers:

- Each world advances independently with its own `active_mask`, command queue, and step count.
- Worlds that are currently stepping pack into one `BatchedPhysicsBackend.step_batch(requests)` call returning contiguous `[B, ...]` observation arrays.
- Worlds paused on adaptive decisions (firmware G-code planning, slip recovery, or JAX/SciPy fitting) set `active=False` for that tick without stalling the rest of the batch.

## CLI Usage (`scripts/hp_sim5_run.py`)

Check local tools, Colab auth (`~/bin/colab`), and Google Drive auth:

```bash
.venv/bin/python scripts/hp_sim5_run.py doctor
```

Run a single autocal experiment locally and persist artifacts:

```bash
.venv/bin/python scripts/hp_sim5_run.py run \
  --backend local --kind autocal --machine hangprinter_3 \
  --recording-mode events \
  --summary 'HP3 autocal baseline with event recording'
```

Run a parallel batch on a Google Colab high-RAM CPU VM (8 cores, 50 GiB RAM) and store artifacts:

```bash
.venv/bin/python scripts/hp_sim5_run.py batch experiments.json \
  --backend colab --high-mem --workers 6 \
  --store gdrive://hp-sim5-artifacts
```

(Pass `--direct-drive` for large multi-GB batches to upload directly from Colab → Google Drive using a short-lived access token instead of staging via the local host.)

### Find, inspect, and retain runs

Use the same commands from a regular checkout or a research session.

- `research/output/` holds program agendas, session reports, and selected reproducible evidence.
- `output/experiments/<id>/` holds reusable local run payloads. Omit `--store` for the repo-root default. Relative store URIs such as `local://output` resolve from the working directory.
- `gdrive://hp-sim5-artifacts` holds shared run payloads across hosts. Use it with `--store` to publish a Colab or local run.

A report can cite a shared run by ID and file path, plus SHA-256 for files relied on as evidence.
Live research runtimes keep their world state and in-progress captures with the
session. Archive a finished capture with `artifacts push` when it needs to be shared.

```bash
.venv/bin/python scripts/hp_sim5_run.py artifacts find --machine hangprinter_4 bootstrap
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts find bootstrap
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts manifest exp-001
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts \
  get exp-001 ./restored/exp-001 --file spec.json
```

`find` searches the default local store, recognized run manifests under
`output/` and `research/output/`, the active research session, and the default
Drive store when Google credentials are available. Use `--local-only` for local
files and explicitly selected local stores. Use `--store` to select a different store and
`--also-store` for another store, including `drivemount://...`, and `--path`
for another local run tree. Exact filters include `--machine`, `--kind`,
`--status`, and `--tag`. Use `--revision` with a Git hash or prefix.
Text terms match displayed metadata, including notes, IDs, and local paths.
Drive results show a short preview of the note. The stored manifest has the full
note, metrics, and file inventory with SHA-256 checksums. Open the local
`metadata_path` or fetch the stored `spec.json` to check exact parameters.
Historical manifests vary in which inputs and provenance they recorded. Use
`find` to include loose runs. `list` lists the packages in one artifact store.

Both CLI backends retain outcomes by default under `output/experiments/`.
For durable sharing across hosts, use `--store gdrive://hp-sim5-artifacts`
from either backend. Pass `--keep-scratch` to retain the working directory or
`--discard` for throwaway checks. Drive upload failures preserve the local copy
under `output/upload-pending/` with retry instructions.
Use `--summary` and `--tag` for single runs or `metadata.summary` and `tags`
in each batch spec. For example, a sweep that ruled out a candidate setting is
worth keeping with the setting and observed failure in the note.

New scheduled runs record the Git revision and worktree dirty status, the
portable spec, and the outcome and available metrics. Headless manifests
also record available scene, firmware, and solver hashes. Keep the actual
input files or source patch with the run when a revision alone cannot
reproduce it. Use a fresh ID for each distinct run.

Older local archives may still use `output/research/`, `research/sessions/`, or
the nested `output/experiments/experiments/` store from an earlier CLI.
On reorganized checkouts, raw legacy trees live in `output/legacy/`.
Local aliases preserve links from old reports and embedded provenance. Keep those
paths when verifying a historical manifest or rerunning an archived command.

Archive an existing run directory or retry a preserved upload with `push`.
Existing provenance is carried forward where available.

```bash
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts \
  push exp-001 path/to/run_dir --machine hangprinter_4 --summary 'Observed result and condition'
```

## Google Colab & Drive Authentication

To use `--backend colab`, install the official CLI once (`uv tool install google-colab-cli`). `GoogleDriveArtifactStore` and `colab --auth=adc` both use standard Google Application Default Credentials (`GOOGLE_APPLICATION_CREDENTIALS` or `~/.config/gcloud/application_default_credentials.json`), with no `hp-sim5`-specific auth environment variables.

1. Mint ADC with the `drive.file` scope (non-restricted, supported by the standard Google Cloud SDK OAuth client alongside Colab):

```bash
gcloud auth application-default login --scopes=\
openid,\
https://www.googleapis.com/auth/cloud-platform,\
https://www.googleapis.com/auth/userinfo.email,\
https://www.googleapis.com/auth/colaboratory,\
https://www.googleapis.com/auth/drive.file
```

2. Google Drive API requires ADC user credentials to have a quota project with `drive.googleapis.com` enabled. Configure it persistently using standard `gcloud` tooling:

```bash
gcloud config set project <PROJECT_ID>
gcloud config set billing/quota_project <PROJECT_ID>
gcloud auth application-default set-quota-project <PROJECT_ID>
gcloud services enable drive.googleapis.com
```

3. Verify both Colab and Drive access with:

```bash
.venv/bin/python scripts/hp_sim5_run.py doctor
```
