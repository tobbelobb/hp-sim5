# Research Findings

## Q1: How does `autocal/autocal.py` trace from top-level CLI parsing into `full_auto_loop`, and where does it currently decide how collector arguments, simulator lifecycle, and final calibration-command sends are handled?

### Findings
- `main()` builds its CLI from `build_semi_auto_parser()`, parses args, normalizes optimizer mode, resolves spool options, normalizes the machine type alias, loads any `--full-auto-run` / `--shotgun` variants, cleans `--collector-args`, appends top-level `--speedup`, `--project-zero-tension`, and `--debug-sweep-actions` into `collector_args` when absent, then calls `full_auto_loop(...)` with those values. `autocal/autocal.py:1326-1413`, `autocal/_autocal_common.py:3603-3689`, `autocal/_autocal_common.py:3692-3765`
- The parser surface that feeds this path is defined in shared helpers, not in `autocal.py` itself. `--collector-args`, `--sim`, `--keep-sim-alive`, `--hp-sim-reset`, `--project-zero-tension`, and `--debug-sweep-actions` are all added in `_add_collector_args()`. `autocal/_autocal_common.py:3560-3592`
- `full_auto_loop()` normalizes collector/runtime decisions near the top of the function: it checks whether the user already requested `--no-spawn-rrf-simulator`, applies simulation defaults, injects spool-related collector overrides, computes effective hp-sim reset behavior, optionally appends `--sweepPoints`, resolves the RRF target/server, and chooses the simulator config. `autocal/autocal.py:174-230`
- The effective collector args are derived in this order: `_apply_simulation_defaults()` first, then `_inject_spool_collection_args()`, then optional `--sweepPoints` append from the top-level `sweep_points` arg when the collector args did not already set it. `autocal/autocal.py:187-221`
- Simulator lifecycle is handled in `full_auto_loop()` before any collection starts. When `sim=True`, `no_collect=False`, no explicit server was supplied, and the collector args do not already say `--no-spawn-rrf-simulator`, the code starts `rrf_simulator` and waits for readiness. `autocal/autocal.py:223-246`
- The bootstrap-data path also lives in `full_auto_loop()`. When the dataset file does not exist, the function writes a bootstrap sweep-config file, strips conflicting collector args, forces `--return-to-origin`, optionally adds `--hp-sim-reset`, and runs `node autocal/control/cli/collect_sweep_data.mjs ...` as a subprocess. `autocal/autocal.py:259-320`
- The collector subprocess command is constructed directly in Python, not by delegating to a bridge abstraction. It always points at `autocal/control/cli/collect_sweep_data.mjs` and passes `--machineType`, `--sweep-config-file`, `--output-file`, plus the normalized collector args. `autocal/autocal.py:304-318`
- During each full-auto iteration, `_execute_plan_run()` passes the already-normalized `collector_args_eff` into `plan_next_ellipse_sweep(...)`; that is the value used when the planner builds per-iteration collection commands. `autocal/autocal.py:580-690`, `autocal/_autocal_common.py:1670-1691`
- Final calibration-command handling is concentrated inside `_emit_summary_and_send()`. It computes both `m669` and `m666`, prints both in the summary, but only transmits `m669` back to firmware. `m666` is not sent in this function. `autocal/autocal.py:106-172`
- The final send path is hard-wired to RRF transport: `_emit_summary_and_send()` logs `Sending {m669} to {rrf_server}` and calls `_send_rrf_gcode(rrf_server, m669)`. `autocal/autocal.py:156-170`
- There is one explicit skip path for that final send: when `sim=True`, `no_collect=True`, and no server was explicitly targeted, the code logs `; --sim + --no-collect set; skipping M669 send.` instead of sending the command. `autocal/autocal.py:156-160`

## Q2: How do the shared helpers in `autocal/_autocal_common.py` encode firmware or simulator assumptions today, including target resolution, simulation defaults, simulator config selection, process startup, readiness checks, and G-code sending?

### Findings
- The file-level constants encode RRF-specific assumptions directly: default port `8081`, simulator binary `RRF/build/rrf_simulator`, VSD path `RRF/run/vsd`, and the default / line-layer config filenames for slideprinter and hp3-style hangprinter configs. `autocal/_autocal_common.py:30-36`
- Machine-type aliasing also lives in shared helpers; `hp3`, `hp4`, and `hangprinter_3` are normalized to `hangprinter_4`. `autocal/_autocal_common.py:112-129`
- `_resolve_rrf_target()` resolves firmware target information from `RRF_SERVER_URL`, `--server` / `--rrf`, and `--port`. If the server was not explicit, it forces `localhost` plus the default RRF port, so the effective default becomes `http://localhost:8081`. `autocal/_autocal_common.py:422-445`
- `_apply_simulation_defaults()` encodes the current split between simulated and non-simulated collection:
  - In sim mode it appends `--no-spawn-rrf-simulator`, expecting Python to own simulator startup.
  - Outside sim mode it appends both `--no-ws` and `--no-spawn-rrf-simulator`, which disables hp-sim WS usage and simulator autostart in the Node collector. `autocal/_autocal_common.py:448-458`
- `_inject_spool_collection_args()` only injects explicit spool overrides into collector args: `--force-base-radii` when base radii were supplied and no collector override is already present, and `--force-buildup-factor` when a buildup factor was supplied and no collector override is already present. It does not auto-add `--preserve-buildup-factor`. `autocal/_autocal_common.py:461-537`
- `_resolve_sim_config()` picks simulator config files from machine type plus spool-search mode:
  - environment override via `AUTOCAL_RRF_SIM_CONFIG` wins;
  - hangprinter aliases resolve to hp3 configs;
  - spool-search mode prefers the line-layer config when the file exists. `autocal/_autocal_common.py:540-565`
- `_effective_hp_sim_reset()` only returns `True` in simulation. It also auto-enables reset for spool-search modes even when `--hp-sim-reset` was not explicitly passed. `autocal/_autocal_common.py:567-590`
- `_send_rrf_gcode()` sends plain-text G-code by POSTing to `${server}/machine/code`. That helper is the shared transport used by Python’s final-summary send and readiness probing. `autocal/_autocal_common.py:593-600`
- `_wait_for_rrf_server()` treats readiness as “`M115` succeeds over `/machine/code` within the timeout”; it loops by calling `_send_rrf_gcode(server, "M115", ...)`. `autocal/_autocal_common.py:603-611`
- `_start_rrf_simulator()` spawns the RRF simulator binary with `--vsd`, `-c`, `--server`, and `-p`, runs it from repo root, and discards stdout/stderr. `autocal/_autocal_common.py:614-634`
- The helper that formats final anchor output is also firmware-shaped: `_format_m669_command()` chooses hangprinter anchor labels `A/B/C/D/I` for `hangprinter_4` / `hangprinter_5`, and a longer label list for other machine types. `autocal/_autocal_common.py:647-665`
- `_m669_from_plan()` prefers a precomputed `calibration.gcode` string that already starts with `M669`; otherwise it synthesizes one from the plan’s anchor matrix. `_m666_from_plan()` is computed separately, but Python’s final send path still only transmits `M669`. `autocal/_autocal_common.py:667-677`, `autocal/autocal.py:118-172`

## Q3: How does `autocal/control/cli/collect_sweep_data.mjs` trace from CLI parsing into bridge creation and sweep execution, and which parts of that flow are tied to the current RRF bridge and simulator model?

### Findings
- The CLI entrypoint imports RRF-specific bridge/runtime pieces at the top:
  - `createGcodeBridge` comes from `integrations/rrf/rrfSimulatorBridge.mjs`;
  - simulator helpers come from `encoder_utils.mjs`, which in turn start `rrf_simulator`;
  - machine-type normalization and the collection behavior are imported separately. `autocal/control/cli/collect_sweep_data.mjs:1-23`
- `parseBridgeArgs(argv)` parses the collector CLI into a flat argument object that includes RRF server fields, WS fields, simulator lifecycle flags, machine type, sweep parameters, spool override flags, trace controls, and output settings. `autocal/control/cli/collect_sweep_data.mjs:71-259`
- `parseBridgeArgs()` seeds `server` from `RRF_SERVER_URL` or `http://localhost:8080`, but `main()` later recomputes the effective target as `http://localhost:${targetPort}` when the server was not explicit, with `targetPort` defaulting to `DEFAULT_RRF_PORT` (`8081`). The effective default is therefore resolved in `main()`, not solely in parsing. `autocal/control/cli/collect_sweep_data.mjs:72-76`, `autocal/control/cli/collect_sweep_data.mjs:352-355`, `autocal/control/primitives/encoder_utils.mjs:7-10`
- `main()` normalizes the machine type, resolves `MACHINE_CONFIGS[machineType]`, checks the `MOTOR_IDS_BY_MACHINE` mapping, computes `speedup`, `encoderTimeoutMs`, and `waitForWsMs`, then resolves `targetPort`, `targetServer`, and `shouldSpawnRrf`. `autocal/control/cli/collect_sweep_data.mjs:315-355`
- Simulator startup in the collector is RRF-specific:
  - `shouldSpawnRrf = !args.noSpawnRrfSimulator && !args.serverExplicit`;
  - startup logs `Starting rrf_simulator at ${targetServer}...`;
  - it calls `startRrfSimulator({ port, debug, machineType })`;
  - it waits with `waitForRrfSimulator(targetServer)`. `autocal/control/cli/collect_sweep_data.mjs:354-365`
- Bridge creation is also RRF-specific. `main()` constructs `bridgeCtx = createGcodeBridge({ server: targetServer, wsPort, quiet, encoderTimeoutMs })`. `autocal/control/cli/collect_sweep_data.mjs:368-373`
- The send wrapper delegates every G-code line to `bridgeCtx.sendGcodeLine(...)`; when debug flags are enabled it prints `[rrf_gcode]` / `[rrf_reply]`, matching the RRF naming in the transport layer. `autocal/control/cli/collect_sweep_data.mjs:375-399`
- hp-sim coordination happens after bridge creation, but still through RRF-oriented naming:
  - wait for WS connection;
  - optional hp-sim reset;
  - optional speed-scale broadcast;
  - optional trace-mode broadcast. `autocal/control/cli/collect_sweep_data.mjs:406-417`
- The actual sweep collection is delegated to `collectSweepData(send, { args, machineType, machineConfig, motorIds, speedup, delayFn })`. The CLI does not contain sweep mechanics itself after this point. `autocal/control/cli/collect_sweep_data.mjs:419-428`
- Cleanup is tied to the spawned RRF process and RRF bridge context: the finally-block stops `rrfProcess` unless `--persist-rrf-simulator` was set, closes readline state, and closes `bridgeCtx`. `autocal/control/cli/collect_sweep_data.mjs:432-441`
- The RRF bridge implementation underneath this CLI uses `RrfHttpBridge`, which POSTs to `/machine/code`, parses the returned motion, and forwards resulting motion commands to hp-sim via `remoteSpoolSystem.addCommand(...)`. `integrations/rrf/rrfSimulatorBridge.mjs:199-248`, `integrations/rrf/rrfHttpBridge.js:15-27`, `integrations/rrf/rrfHttpBridge.js:46-80`
- There is no firmware-selection branch in this collector CLI. The imports, naming, simulator binary, and bridge class all point at the RRF stack directly. `autocal/control/cli/collect_sweep_data.mjs:4-15`, `autocal/control/primitives/encoder_utils.mjs:248-279`

## Q4: How do `autocal/control/primitives/encoder_utils.mjs`, `autocal/control/primitives/machine_type.mjs`, and `autocal/control/behaviors/sweep_data_collection.mjs` divide responsibility for machine configuration, movement timing, hp-sim coordination, and simulator startup during sweep collection?

### Findings
- `machine_type.mjs` is the narrowest layer:
  - it normalizes machine-type aliases;
  - it maps normalized machine types to RRF simulator config files;
  - it selects the default vs line-layer config. `autocal/control/primitives/machine_type.mjs:1-32`
- `encoder_utils.mjs` owns low-level runtime helpers around the RRF collector path:
  - parsing `M666`/encoder replies;
  - converting `M666` values into `mmPerDeg`;
  - estimating movement duration from returned motion data or feed-distance fallback;
  - building/spawning RRF simulator args;
  - sending hp-sim reset / speed-scale / trace broadcasts. `autocal/control/primitives/encoder_utils.mjs:17-118`, `autocal/control/primitives/encoder_utils.mjs:170-246`, `autocal/control/primitives/encoder_utils.mjs:248-323`
- `runMoveWithWait()` in `encoder_utils.mjs` is the movement-timing primitive. It calls `sendFn(gcode)`, prefers `parsed.motion` timing via `motionDurationSeconds(...)`, and falls back to feedrate and geometric distance when motion timing is unavailable. `autocal/control/primitives/encoder_utils.mjs:92-118`, `autocal/control/primitives/encoder_utils.mjs:170-246`
- Simulator startup is not handled inside `sweep_data_collection.mjs`. That behavior lives in CLI + `encoder_utils.mjs` via `startRrfSimulator()` / `waitForRrfSimulator()`. `autocal/control/cli/collect_sweep_data.mjs:354-365`, `autocal/control/primitives/encoder_utils.mjs:248-279`
- `sweep_data_collection.mjs` owns machine-configuration data used during collection:
  - `MACHINE_CONFIGS` defines per-machine anchor counts, dimensions, visible axes, fixed-anchor constraints, and optional bounds/scaling;
  - `MOTOR_IDS_BY_MACHINE` defines the CAN/driver IDs used for each machine type. `autocal/control/behaviors/sweep_data_collection.mjs:23-51`
- The collection behavior module also owns input normalization and defaults:
  - `validateSweepCollectionInput()` parses and validates sweep, force, fixed-target, base-radii, buildup, and noise options;
  - `applySweepDefaults()` fills in defaults for points, feed, settle, speedup, force thresholds, noise sampling, and auto-tune behavior. `autocal/control/behaviors/sweep_data_collection.mjs:400-489`, `autocal/control/behaviors/sweep_data_collection.mjs:491-540`
- Sweep planning / config selection also lives in `sweep_data_collection.mjs`:
  - `resolveSweepConfigs()` loads a config file or auto-generates configs;
  - `generateSweepConfigs()` and `selectRepresentativeConfigs()` enforce machine-specific fixed-anchor rules such as `hangprinter_4.mustBeInFixedSet = [3]`. `autocal/control/behaviors/sweep_data_collection.mjs:542-576`, `autocal/control/behaviors/sweep_data_collection.mjs:595-631`
- `collectSweepData()` is the main orchestration point. It:
  - validates inputs and applies defaults;
  - loads sweep configs;
  - reads `M666`, `M669`, and `M92`;
  - optionally sends an `M666` override;
  - computes `mmPerDeg`;
  - primes encoders;
  - optionally auto-tunes force;
  - performs size-tune / max-travel measurement;
  - runs each sub-sweep and writes the dataset. `autocal/control/behaviors/sweep_data_collection.mjs:1148-1540`
- Responsibility for machine calibration data is split across files:
  - `machine_type.mjs` decides which simulator config file to boot;
  - `sweep_data_collection.mjs` defines runtime machine geometry / axes / fixed-anchor constraints;
  - `encoder_utils.mjs` turns the runtime `M666` response into per-axis `mmPerDeg`. `autocal/control/primitives/machine_type.mjs:7-32`, `autocal/control/behaviors/sweep_data_collection.mjs:23-51`, `autocal/control/primitives/encoder_utils.mjs:72-90`
- hp-sim coordination during sweep collection is indirect inside `sweep_data_collection.mjs`: the module receives a `send` function plus `speedup`, uses `runMoveWithWait(...)`, `waitForStableEncoders(...)`, and encoder priming, but it does not open or manage the WS bridge itself. `autocal/control/behaviors/sweep_data_collection.mjs:1235-1237`, `autocal/control/behaviors/sweep_data_collection.mjs:1266-1287`, `autocal/control/behaviors/sweep_data_collection.mjs:1363-1397`
- Fixed-target positioning and motion execution are behavior-level concerns:
  - `prepareSweepPositioning()` retightens fixed anchors and repositions before each sub-sweep;
  - `measureMaxTravelMm()` performs the size-tune pull;
  - `performForceSweep()` executes the actual sweep points. `autocal/control/behaviors/sweep_data_collection.mjs:704-745`, `autocal/control/behaviors/sweep_data_collection.mjs:809-873`, `autocal/control/behaviors/sweep_data_collection.mjs:956-1145`

## Q5: What entrypoints and runtime patterns already exist under `integrations/klipper/*` for starting a Klippy-backed session, sending commands, collecting motion or encoder-related data, and forwarding events to hp-sim, and how do those flows differ from the RRF path?

### Findings
- The main Klipper terminal entrypoint is `integrations/klipper/klipper_terminal.mjs`. Its help text describes the intended flow: send G-code to a Klippy API socket, subscribe to `motion_report` stepper batches, and fan simulator commands out over WebSocket for hp-sim. `integrations/klipper/klipper_terminal.mjs:82-101`
- `klipper_terminal.mjs` constructs three core runtime objects:
  - `KlippyApiClient` for the Unix-domain API socket;
  - `KlippyRuntimeState` for priming/subscriptions/state tracking;
  - `createKlipperTerminalBridge(...)` for hp-sim WS fan-out and session handling. `integrations/klipper/klipper_terminal.mjs:162-201`
- Klippy autostart / readiness helpers live in `klippy_api_cli_config.mjs`. That file defines the default start script, socket path, config path, log path, local Python interpreter path (`./.venv/bin/python`), and host-MCU env vars; it can find/terminate stale `klippy.py` processes, wait for the socket, and spawn the managed stack. `integrations/klipper/klippy_api_cli_config.mjs:5-18`, `integrations/klipper/klippy_api_cli_config.mjs:99-143`, `integrations/klipper/klippy_api_cli_config.mjs:145-176`, `integrations/klipper/klippy_api_cli_config.mjs:186-255`
- `KlippyApiClient` implements framed JSON-RPC over the Unix socket:
  - `start()` / `waitForConnection()` establish the socket;
  - `request()` sends RPC requests;
  - `subscribe()` persists subscription metadata for async streams and resubscription after reconnects;
  - `_routeMessage()` dispatches replies vs async subscription payloads. `integrations/klipper/klippyApiClient.js:20-57`, `integrations/klipper/klippyApiClient.js:59-143`, `integrations/klipper/klippyApiClient.js:170-229`, `integrations/klipper/klippyApiClient.js:231-406`
- `KlippyRuntimeState` primes the session by calling `info` and `objects/list`, subscribes to `objects/subscribe` and `gcode/subscribe_output`, tracks the current `motion_report` stepper/trapq sources, and emits events such as `printer-state-changed` and `motion-sources-changed`. `integrations/klipper/klippyRuntimeState.js:3-15`, `integrations/klipper/klippyRuntimeState.js:128-230`, `integrations/klipper/klippyRuntimeState.js:303-340`, `integrations/klipper/klippyRuntimeState.js:342-475`
- `klipper_terminal.mjs` sends G-code through Klippy by calling `client.request('gcode/script', { script: trimmed })`, but it wraps the request in `bridgeContext.runGcodeCommand(...)` so that motion/encoder/reply forwarding can happen around the command session. `integrations/klipper/klipper_terminal.mjs:600-624`
- `createKlipperTerminalBridge(...)` is the WS / hp-sim side of the Klipper stack. It:
  - creates a WebSocket server;
  - queues broadcasts until hp-sim connects;
  - sends `encoder_request` messages and waits for `encoder_response`;
  - subscribes to `motion_report/dump_stepper` and `motion_report/dump_trapq`;
  - broadcasts session start, stepper batches, trapq batches, session end, replies, and force-mode changes. `integrations/klipper/klipperTerminalBridge.js:96-299`, `integrations/klipper/klipperTerminalBridge.js:360-388`, `integrations/klipper/klipperTerminalBridge.js:438-520`, `integrations/klipper/klipperTerminalBridge.js:522-610`, `integrations/klipper/klipperTerminalBridge.js:687-737`, `integrations/klipper/klipperTerminalBridge.js:739-815`
- Encoder-related data in the Klipper stack is bridged through hp-sim, not read directly from a firmware reply. `maybeOverrideEncoderReply()` resolves motor tokens to axes, asks hp-sim for angles, tracks reference values, and replaces the command reply with a synthesized encoder response string when appropriate. `integrations/klipper/klipperTerminalBridge.js:631-685`
- Motion forwarding in the Klipper stack is based on subscriptions and batching, not on a one-response-per-command model. `handleStepperBatch()` broadcasts `klipper_api_stepper_batch` messages, and `handleTrapqBatch()` broadcasts `klipper_api_trapq_batch` messages while tracking planned-vs-observed motion end time. `integrations/klipper/klipperTerminalBridge.js:450-520`
- Several lower-level Klipper motion utilities already exist:
  - `KlipperClockModel` and `KlipperBucketedMotionCore` in `klipperMotionCore.js`;
  - `KlipperApiMotionAdapter` to convert stepper batches into time-stamped commands;
  - `KlipperApiSessionTimelineBuffer` to rebase batched stepper data onto a session timeline. `integrations/klipper/klipperMotionCore.js:87-176`, `integrations/klipper/klipperMotionCore.js:178-243`, `integrations/klipper/klipperApiMotionAdapter.js:8-85`, `integrations/klipper/klipperApiSessionTimeline.js:93-126`, `integrations/klipper/klipperApiSessionTimeline.js:128-243`
- There is also a separate raw-motion playback path in `klipperSimulatorBridge.js`: `createKlipperRawBridge()`, `connectKlipperRaw()`, and `playKlipperRawFile()` create a worker-backed pacer that forwards raw / API-motion events into scheduled hp-sim commands. `integrations/klipper/klipperSimulatorBridge.js:284-310`, `integrations/klipper/klipperSimulatorBridge.js:332-438`, `integrations/klipper/klipperSimulatorBridge.js:440-455`
- The RRF path differs at every transport boundary:
  - RRF entrypoint: `integrations/rrf/rrf_terminal.mjs`;
  - RRF bridge: `createGcodeBridge(...)` wrapping `RrfHttpBridge`;
  - RRF command send: HTTP POST to `/machine/code`;
  - RRF motion forwarding: parse motion records from the same reply and emit `command` / `reply` WS messages. `integrations/rrf/rrf_terminal.mjs:13-26`, `integrations/rrf/rrf_terminal.mjs:72-77`, `integrations/rrf/rrf_terminal.mjs:118-146`, `integrations/rrf/rrfSimulatorBridge.mjs:199-248`, `integrations/rrf/rrfHttpBridge.js:46-80`
- Within the traced autocal sweep-collection path, imports point to the RRF stack and not to `integrations/klipper/*`. The Klipper integration exists as a separate runtime/tooling stack rather than a currently-selected backend in `autocal/autocal.py` or `collect_sweep_data.mjs`. `autocal/control/cli/collect_sweep_data.mjs:4-15`, `autocal/autocal.py:11-12`

## Q6: What tests currently cover the Python autocal CLI and the Node collector around `--sim`, collector argument forwarding, simulator startup, and bridge behavior, and what assumptions about the active firmware path are those tests enforcing?

### Findings
- Python CLI coverage for top-level forwarding lives in `autocal/tests/test_autocal_cli.py`:
  - `test_autocal_main_normalizes_hangprinter_alias` asserts `hp3` becomes `hangprinter_4`;
  - `test_autocal_main_forwards_top_level_speedup_to_collector_args` asserts top-level `--speedup` becomes `["--speedup", "40"]`;
  - `test_autocal_main_keeps_explicit_collector_speedup_untouched` asserts explicit collector args are preserved. `autocal/tests/test_autocal_cli.py:237-259`, `autocal/tests/test_autocal_cli.py:262-283`, `autocal/tests/test_autocal_cli.py:286-308`
- Python parser coverage includes `test_build_semi_auto_parser_accepts_no_collect`, which pins that `--no-collect` is accepted by the modular autocal CLI. `autocal/tests/test_autocal_no_collect.py:48-57`
- The main Python test that pins `--sim --no-collect` behavior is `test_full_auto_loop_no_collect_exits_when_replay_depletes`. It monkeypatches `_start_rrf_simulator`, `_wait_for_rrf_server`, and `subprocess.run` to fail if called, then runs `full_auto_loop(... sim=True, no_collect=True ...)` and asserts no final send occurred. This enforces that the simulated RRF stack is not started and the Node collector is not invoked in replay-only no-collect mode. `autocal/tests/test_autocal_no_collect.py:239-327`
- `test_full_auto_loop_logs_invoked_command_near_top` repeats the same `sim=True, no_collect=True` setup and checks that the full-auto log records the original `autocal/autocal.py --sim ...` invocation near the top. It uses the same fail-fast monkeypatches for `_start_rrf_simulator`, `_wait_for_rrf_server`, and `subprocess.run`. `autocal/tests/test_autocal_no_collect.py:330-430`
- `test_full_auto_loop_registers_cleanup_for_keyboard_interrupt_before_main_loop` monkeypatches `_start_rrf_simulator`, `_wait_for_rrf_server`, and `subprocess.run` to simulate an interrupt during collection startup, then verifies that the registered cleanup callback stops the simulator process. This pins the existence of the RRF simulator lifecycle and cleanup behavior in the simulated path. `autocal/tests/test_autocal_no_collect.py:432-525`
- `test_write_bootstrap_sweep_config_uses_machine_specific_constraints` checks that bootstrap sweep config generation for `hangprinter_4` always leaves anchor `3` fixed and never uses it as drive/sensor. That test enforces the same machine constraint that the Node sweep collector uses. `autocal/tests/test_autocal_no_collect.py:527-551`
- Shared-helper tests in `autocal/tests/test_filter_pass_spool_fit.py` cover spool/sim defaults:
  - explicit `R` / `Q` injection behavior;
  - no implicit `--preserve-buildup-factor`;
  - respecting explicit buildup overrides;
  - hp-sim reset auto-enable rules;
  - simulator config selection for spool-search and hangprinter aliases;
  - environment override of sim config. `autocal/tests/test_filter_pass_spool_fit.py:550-562`, `autocal/tests/test_filter_pass_spool_fit.py:924-942`, `autocal/tests/test_filter_pass_spool_fit.py:945-983`, `autocal/tests/test_filter_pass_spool_fit.py:986-1029`
- Node collector helper coverage in `autocal/control/tests/cli/collect_sweep_data.cli.test.mjs` pins the RRF-oriented collector assumptions:
  - hangprinter aliases resolve to `hangprinter_4`;
  - hangprinter uses hp3 RRF config files;
  - `buildRrfSimulatorArgs()` always builds `rrf_simulator` args under `RRF/run/vsd`;
  - `MACHINE_CONFIGS.hangprinter_4.axes` match the visible axes in the selected hp3 RRF config;
  - `parseBridgeArgs()` captures spool override flags. `autocal/control/tests/cli/collect_sweep_data.cli.test.mjs:22-47`, `autocal/control/tests/cli/collect_sweep_data.cli.test.mjs:49-67`, `autocal/control/tests/cli/collect_sweep_data.cli.test.mjs:152-161`
- Node bridge helper coverage in `autocal/control/tests/primitives/rrf_http_bridge_cli_config.test.mjs` asserts that `waitForRrfSimulator()` probes `POST /machine/code` with `M115`, which matches the RRF readiness model used by both Python and Node. `autocal/control/tests/primitives/rrf_http_bridge_cli_config.test.mjs:7-41`
- The end-to-end collector tests under `autocal/control/tests/e2e/*.e2e.test.mjs` use the RRF bridge stack as well. `collect_single_sweep.e2e.test.mjs` is representative:
  - it imports `parseBridgeArgs`, `createGcodeBridge`, `startRrfSimulator`, `waitForRrfSimulator`, and `collectSweepData`;
  - it starts `rrf_simulator` when needed;
  - it creates an RRF bridge context;
  - it calls `collectSweepData(send, ...)`. `autocal/control/tests/e2e/collect_single_sweep.e2e.test.mjs:5-16`, `autocal/control/tests/e2e/collect_single_sweep.e2e.test.mjs:149-224`
- Additional e2e harnesses using the same pattern are present for encoder-noise calibration, stable-encoder waits, return-to-origin variants, and force finding. Their filenames indicate RRF-backed collector scenarios rather than Klipper-backed ones. `autocal/control/tests/e2e/calibrate_encoder_noise.e2e.test.mjs:38`, `autocal/control/tests/e2e/wait_for_stable_encoders.e2e.test.mjs:27`, `autocal/control/tests/e2e/return_to_origin_all_at_once.e2e.test.mjs:31`, `autocal/control/tests/e2e/return_to_origin_one_at_a_time.e2e.test.mjs:31`, `autocal/control/tests/e2e/find_minimum_moving_force.e2e.test.mjs:49`, `autocal/control/tests/e2e/find_edge_force.e2e.test.mjs:49`
- I did not find tests in these paths that exercise the current autocal Python entrypoint or the current Node collector CLI against the Klipper bridge stack. The existing tests around simulation startup, readiness, and bridge behavior are all anchored to RRF naming, `rrf_simulator`, `/machine/code`, and `createGcodeBridge` / `RrfHttpBridge`. `autocal/autocal.py:223-246`, `autocal/control/cli/collect_sweep_data.mjs:354-373`, `integrations/rrf/rrfSimulatorBridge.mjs:199-248`

## Cross-Cutting Observations

- The traced autocal sweep path is RRF-specific at both layers: Python helper names use `rrf_*`, and the Node collector imports the RRF bridge directly. `autocal/_autocal_common.py:422-634`, `autocal/control/cli/collect_sweep_data.mjs:4-15`
- The effective collector startup split is “Python owns simulator startup in `--sim`, Node owns only collection”; Python injects `--no-spawn-rrf-simulator` into collector args in sim mode, then starts the simulator itself when appropriate. `autocal/_autocal_common.py:448-458`, `autocal/autocal.py:187-246`
- Hangprinter aliasing is mirrored across Python and Node. Both normalize `hp3` / `hp4` / `hangprinter_3` to `hangprinter_4`, and both map that normalized type to hp3 RRF config files. `autocal/_autocal_common.py:112-129`, `autocal/control/primitives/machine_type.mjs:1-32`
- The final calibration send path currently stops at `M669`. `M666` is surfaced in summaries and datasets, but not transmitted by `autocal.py`’s final send routine. `autocal/autocal.py:118-172`, `autocal/control/behaviors/sweep_data_collection.mjs:1488-1513`
- The Klipper stack is already substantial, but it is organized as a separate terminal / bridge / runtime-state system around a Unix socket and `motion_report` subscriptions, not as a selectable backend inside the traced autocal collector path. `integrations/klipper/klipper_terminal.mjs:162-201`, `integrations/klipper/klipperTerminalBridge.js:360-388`, `autocal/control/cli/collect_sweep_data.mjs:4-15`

## Open Areas

- I did not find direct tests for `_apply_simulation_defaults()` or `_resolve_rrf_target()` themselves; their behavior is mostly inferred from production code plus adjacent helper / full-loop tests. `autocal/_autocal_common.py:422-458`
- I did not find direct tests in these paths for top-level forwarding of `--project-zero-tension` or `--debug-sweep-actions` from `autocal.py` into `collector_args`. `autocal/autocal.py:1346-1349`
- I did not find a test that invokes the real `autocal/control/cli/collect_sweep_data.mjs` `main()` and asserts its startup path end-to-end; the current Node test surface is split between CLI-helper tests and separate e2e harness scripts that recreate the same RRF startup pattern. `autocal/control/cli/collect_sweep_data.mjs:315-441`, `autocal/control/tests/cli/collect_sweep_data.cli.test.mjs:21-162`, `autocal/control/tests/e2e/collect_single_sweep.e2e.test.mjs:76-224`
