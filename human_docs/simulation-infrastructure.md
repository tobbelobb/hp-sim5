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
5. **Artifact Storage** ([`storage.py`](../src/python/hp_sim5_sim/storage.py))
   - `ScratchWorkspace` guarantees active simulation, telemetry, and Rerun recording stay on fast local scratch storage.
   - `ArtifactStore` protocol implemented by:
     - `LocalArtifactStore` (`local:///path/to/store`)
     - `DriveMountArtifactStore` (`drivemount:///content/drive/MyDrive/hp-sim5-artifacts`)
     - `GoogleDriveArtifactStore` (`gdrive://hp-sim5-artifacts`) — pure-stdlib Google Drive REST API v3 with 8 MiB resumable chunked uploads, 308 resume-after-failure recovery, exponential backoff on 429/5xx, SHA-256/MD5 checksum verification, deduplication, and queryable experiment metadata.

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
  --recording-mode events --store local://output/artifact-store \
  --exp-args --max-steps 5 --sweep-points 10
```

Run a parallel batch on a Google Colab high-RAM CPU VM (8 cores, 50 GiB RAM) and store artifacts:

```bash
.venv/bin/python scripts/hp_sim5_run.py batch experiments.json \
  --backend colab --high-mem --workers 6 \
  --store gdrive://hp-sim5-artifacts
```

Manage stored artifacts (usable independently of Colab):

```bash
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts list --tag nightly
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts push exp-001 path/to/run_dir --machine hangprinter_4
.venv/bin/python scripts/hp_sim5_run.py artifacts --store gdrive://hp-sim5-artifacts get exp-001 ./restored/exp-001
```

## Google Drive Authentication

`GoogleDriveArtifactStore` checks credentials in this order:
1. `HP_SIM5_DRIVE_ACCESS_TOKEN` (direct bearer token)
2. `HP_SIM5_DRIVE_CREDENTIALS` (OAuth2 `authorized_user` JSON with `client_id`, `client_secret`, `refresh_token`)
3. Application Default Credentials (`~/.config/gcloud/application_default_credentials.json`) with optional `HP_SIM5_DRIVE_QUOTA_PROJECT`.

To include Drive scope in ADC alongside Colab:

```bash
gcloud auth application-default login --scopes=\
openid,\
https://www.googleapis.com/auth/cloud-platform,\
https://www.googleapis.com/auth/userinfo.email,\
https://www.googleapis.com/auth/colaboratory,\
https://www.googleapis.com/auth/drive
```
