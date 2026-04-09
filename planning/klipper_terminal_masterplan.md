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
- The supported local runtime is direct Linux-process MCU startup via `examples/klipper/linux_mcu/klipper.elf` exposing `/tmp/klipper_host_mcu`, as documented by Klipper's Linux/RPi MCU docs. `klipper_terminal.mjs` must not depend on `examples/klipper/slideprinter/klipper_linux_mcu_bridge.py`, `pysimulavr`, `examples/klipper/slideprinter/klipper_avr_bridge.py`, or `examples/klipper/avr/klipper.elf`.
- The fake GPIO setup from `scripts/make-fake-pin-chip.sh` is still a local prerequisite for configs that reference a simulated `gpiochip*`; it is not a substitute for the MCU process itself.
- Local interactive runs should default to non-realtime Linux MCU startup so the terminal works without root/systemd privileges; realtime mode can remain an opt-in for dedicated environments.

## 1. Process + API transport

Deliverables:
- `integrations/klipper/klipper_terminal.mjs` CLI skeleton mirroring the RRF terminal UX where it makes sense (`--cmd`, `--ws-port`, `--no-ws`, `--quiet`, `--keep-alive`).
- A small Node-side Klipper API client for the Unix domain socket: `0x03` framing, request ids, async subscriptions, reconnect handling.
- Local process manager that starts the Linux MCU process first, waits for `/tmp/klipper_host_mcu`, then starts `klippy/klippy.py` with `-a <socket>` and tracks shutdown / restart. See and reuse/extend `scripts/run_klippy_api_mode.sh`.

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
- Confirm prompt / status changes track `webhooks.state`, including `startup` before the MCU side is ready and `ready` after the local Linux MCU process is up.
- Confirm terminal output arrives through `gcode/subscribe_output`.
- Confirm stepper names are discovered dynamically, not hard-coded.

## 3. Shared motion core extraction

Deliverables:
- Extract worker-only logic from `integrations/klipper/klipperPacerWorker.js` into a shared module usable by both browser worker and Node terminal code.
- Reuse existing motion math in `integrations/klipper/motionUtils.js` and `integrations/klipper/klipperFirmwareModel.js`.
- Keep the browser-side Klipper pacer as the single timing source for both raw upload playback and API-mode motion playback, so API batches follow the same scheduler / bucket timing path as raw MCU files.
- Add an API-mode adapter that consumes `motion_report/dump_stepper` batches and clock data, feeds them into the existing Klipper motion core, and schedules them without expanding them into websocket `Move` payloads in Node.
- Delete superseded duplicate code paths inside the worker once the shared module is in place.

Verification:
- Existing raw-upload tests still pass.
- New unit tests feed representative `dump_stepper` batches into the pacer worker and verify emitted `Move` / `Add to reference` commands match current expectations.

## 4. hp-sim bridge + command execution

Deliverables:
- WebSocket fan-out layer for `gcode_ws`, keeping existing `reply` / control messages but adding a lower-level Klipper API motion stream:
  - `klipper_api_session_start`
  - `klipper_api_stepper_batch`
  - `klipper_api_session_end`
- `hp-sim` and `hp-sim-3d` should route those lower-level messages into the existing Klipper raw pacing/scheduler path instead of pushing pre-expanded `Move` commands straight into `RemoteSpoolSystem`.
- Sequential G-code send queue built on `gcode/script`.
- Command-to-motion session handling so one-shot and interactive commands stream motion into hp-sim while they execute.
- Preserve raw batch timing fields (`first_clock`, `last_clock`, `start_mcu_position`, `data`, `clock_hz`) all the way to the browser so replay stays as close as possible to Klipper's batch dump semantics.

Verification:
- Manual smoke test with `?gcode_ws=ws://localhost:8790` in both `hp-sim` and `hp-sim-3d`.
- `--cmd "G90"` returns reply text.
- `--cmd "G1 ..."` produces visible simulator motion and no command ordering regressions under queued inputs.

Closer To `x100.txt` / Batch-Dump Replay:
- `motion_report/dump_stepper` is closer to raw replay than pre-expanded `Move` payloads because it preserves per-stepper `interval/count/add` timing and MCU clock anchoring.
- It is still not a byte-for-byte replacement for `dist/examples/mcu_commands/x100.txt`: the Klipper API does not expose the full raw MCU stream (`set_next_step_dir`, `reset_step_clock`, etc.) through `dump_stepper`.
- If tighter equivalence is needed later, the next candidates are:
  - forward `motion_report/dump_trapq` alongside `dump_stepper` so planner-level move boundaries can be correlated with stepper batches
  - add a diagnostic exporter that reconstructs a synthetic MCU-text trace from the API batches for side-by-side diffing against `x100.txt`
  - add an optional deeper trace path only if Klipper-side instrumentation is acceptable, because the stock API does not currently expose a true raw MCU command log

## 5. file-backed flow (future backlog. Not implemented. Apr 9, 2026)

Deliverables:
- Optional file-backed execution path using `[virtual_sdcard]` only after the interactive path is stable.
- Short local usage notes in the script help text, pointing to repo docs instead of duplicating them.

Verification:
- If `[virtual_sdcard]` is configured, `SDCARD_PRINT_FILE` / `M24` flow updates `print_stats` / `virtual_sdcard` state as expected.
- Manual end-to-end run covers startup, one-shot command, interactive session, restart, and clean shutdown.

## Can we reuse something already in the klipper repo?
There is no drop-in Klipper equivalent to your planned klipper_terminal.mjs inside klipper/. The closest reusable pieces are patterns, not a ready-made bridge.

Best Reuse Candidates

- klipper/scripts/whconsole.py:14 is the simplest client example for the API socket. It already does the two essentials correctly: connect to the Unix socket and frame JSON messages with 0x03. Its stdin loop is raw JSON, though, so it is only a
  transport/protocol reference, not a G-code terminal.
- klipper/scripts/motan/data_logger.py:55 is the best model for a real client. It has:
    - initial info login with client_info (klipper/scripts/motan/data_logger.py:72)
    - request/response dispatch by id (klipper/scripts/motan/data_logger.py:82)
    - async subscription routing via response_template tags (klipper/scripts/motan/data_logger.py:126)
    - dynamic discovery of motion_report.trapq and motion_report.steppers, then subscription to motion_report/dump_trapq and motion_report/dump_stepper (klipper/scripts/motan/data_logger.py:141)
- klipper/klippy/extras/motion_report.py:11 is the key server-side source for the motion stream you want to consume. It shows exactly what dump_stepper and dump_trapq produce, including headers and fields.
- klipper/klippy/extras/bulk_sensor.py:23 explains how Klipper pushes batched async data to webhooks clients. That matters because dump_stepper/dump_trapq are not ad hoc; they use this batch subscription mechanism.
- klipper/klippy/webhooks.py:429 is the authoritative implementation of gcode/script, gcode/subscribe_output, objects/list, and objects/subscribe. It is useful for confirming behavior, especially that:
    - gcode/script just runs the script and completes when done (klipper/klippy/webhooks.py:447)
    - gcode/subscribe_output is terminal-output only (klipper/klippy/webhooks.py:453)
    - objects/subscribe is diffed and refreshed on a 250ms timer (klipper/klippy/webhooks.py:469)
- klipper/scripts/graph_mesh.py:31 is a small one-shot client example. It is useful for a minimal request/response helper and message assembly, but it is not interactive and not motion-related.

What Is Not Already There

- There is no existing Klipper script that:
    - starts Klippy in API mode
    - accepts plain G-code terminal input
    - subscribes to motion streams
    - converts those streams into your hp-sim command websocket format
- There is also nothing in klipper/ that knows about gcode_ws, hp-sim, or your existing raw upload pipeline.

Practical Recommendation

Reuse the design patterns, not the Python files directly:

- Port the socket framing and reconnect behavior from klipper/scripts/whconsole.py:14.
- Port the client state machine from klipper/scripts/motan/data_logger.py:55, especially its response_template tagging and dynamic subscription discovery.
- Treat klipper/klippy/extras/motion_report.py:11 and klipper/klippy/webhooks.py:429 as protocol truth, not code to import.

The one strong conclusion is: there is no pre-existing terminal bridge to reuse wholesale, but there is enough in whconsole.py + motan/data_logger.py to avoid inventing the Klipper client behavior from scratch.
