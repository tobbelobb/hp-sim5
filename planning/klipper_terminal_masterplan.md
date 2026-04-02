# Klipper Terminal Master Plan

Goal: build `integrations/klipper/klipper_terminal.mjs` as the Klipper equivalent of `integrations/rrf/rrf_terminal.mjs`: start and keep Klippy alive in API mode, send G-code through the Klipper API socket, and forward simulator commands to hp-sim over `gcode_ws`.

Primary references:
- `klipper/docs/API_Server.md`
- `klipper/docs/Status_Reference.md`
- `klipper/scripts/motan/data_logger.py`
- `klipper/klippy/extras/motion_report.py`
- `klipper/docs/Config_Reference.md`
- `klipper/docs/G-Codes.md`
- `integrations/rrf/rrf_terminal.mjs`
- `integrations/rrf/rrfSimulatorBridge.mjs`
- `integrations/klipper/klipperPacerWorker.js`
- `scripts/run_klippy_api_mode.sh`

Assumptions:
- MVP uses Klipper's native API socket, not Moonraker.
- MVP supports interactive and one-shot `gcode/script` first.
- "Reuse `KLIPPER_UPLOAD_PIPELINE = 'raw'`" means extracting shared motion decoding / pacing logic from `integrations/klipper/klipperPacerWorker.js` instead of re-implementing it in Node.

## 1. Process + API transport

Deliverables:
- `integrations/klipper/klipper_terminal.mjs` CLI skeleton mirroring the RRF terminal UX where it makes sense (`--cmd`, `--ws-port`, `--no-ws`, `--quiet`, `--keep-alive`).
- A small Node-side Klipper API client for the Unix domain socket: `0x03` framing, request ids, async subscriptions, reconnect handling.
- Klippy process manager that starts `klippy/klippy.py` with `-a <socket>` (see `scripts/run_klippy_api_mode.sh`) and tracks shutdown / restart.

Verification:
- Start Klippy locally and complete `info` and `objects/list`.
- Kill / restart Klippy and verify the terminal reconnects cleanly without leaking stale socket state.
- Kill `klipper_terminal.mjs` and verify klippy was also cleaned up (unless `--keep-alive` was provided).
- Restart `klipper_terminal.mjs` and verify it creates a new klippy process.

## 2. Runtime state + subscriptions

Deliverables:
- Initial login / readiness flow using `info`.
- Long-lived subscriptions for `objects/subscribe` and `gcode/subscribe_output`.
- Runtime state cache for `webhooks`, `toolhead`, `gcode_move`, `motion_report`, and, when configured, `print_stats` and `virtual_sdcard`.
- Motion source discovery from `motion_report.steppers` / `motion_report.trapq` as shown in `klipper/scripts/motan/data_logger.py` and implemented in `klipper/klippy/extras/motion_report.py`.

Verification:
- Confirm prompt / status changes track `webhooks.state`.
- Confirm terminal output arrives through `gcode/subscribe_output`.
- Confirm stepper names are discovered dynamically, not hard-coded.

## 3. Shared motion core extraction

Deliverables:
- Extract worker-only logic from `integrations/klipper/klipperPacerWorker.js` into a shared module usable by both browser worker and Node terminal code.
- Reuse existing motion math in `integrations/klipper/motionUtils.js` and `integrations/klipper/klipperFirmwareModel.js`.
- Add an API-mode adapter that consumes `motion_report/dump_stepper` batches and clock data, then emits the same simulator `command` payloads the current raw pipeline produces.
- Delete superseded duplicate code paths inside the worker once the shared module is in place.

Verification:
- Existing raw-upload tests still pass.
- New unit tests feed representative `dump_stepper` batches and verify emitted `Move` / `Add to reference` commands match current expectations.

## 4. hp-sim bridge + command execution

Deliverables:
- WebSocket fan-out layer for `gcode_ws`, reusing the message contract already consumed by `hp-sim` / `hp-sim-3d` (`command`, `commands`, `reply`, and related control messages).
- Sequential G-code send queue built on `gcode/script`.
- Command-to-motion session handling so one-shot and interactive commands stream motion into hp-sim while they execute.

Verification:
- Manual smoke test with `?gcode_ws=ws://localhost:8790` in both `hp-sim` and `hp-sim-3d`.
- `--cmd "G90"` returns reply text.
- `--cmd "G1 ..."` produces visible simulator motion and no command ordering regressions under queued inputs.

## 5. Operational hardening + file-backed flow

Deliverables:
- Restart / shutdown behavior for `gcode/restart`, `gcode/firmware_restart`, and `webhooks.state` transitions.
- Optional file-backed execution path using `[virtual_sdcard]` only after the interactive path is stable.
- Short local usage notes in the script help text, pointing to repo docs instead of duplicating them.
- Handling of the event that "Reset" is clicked in hp-sim-3d which might temporarily interrupt the websocket.

Verification:
- Ready -> shutdown -> ready transitions recover without restarting the terminal process.
- If `[virtual_sdcard]` is configured, `SDCARD_PRINT_FILE` / `M24` flow updates `print_stats` / `virtual_sdcard` state as expected.
- Manual end-to-end run covers startup, one-shot command, interactive session, restart, and clean shutdown.
