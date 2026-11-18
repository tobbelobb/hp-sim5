# External command / bridge protocol lifecycle (two clients, queues, malformed input, pause/speed, Reset)

Session: `session-2026-11-18-investigate-the-external-command-bridge-protocol-lifecycle-two-c-312cceb7`
Program: `program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7`
Date: 2026-11-18. Revision: `74ecdc95` (clean tree except submodules; no source edits).
Environment: Linux x64, Node v26.13.0. Every experiment is pure JS with stubbed
WebSocket objects and stub timers; no browser, no Vite, no build, no socket, no
server, no physics. Peak RSS per run 50–63 MB, wall time ≤ 0.45 s.

Lens (agenda item 3): does the documented external command protocol behave as
documented under two clients, attach/detach/reconnect, queue overflow,
malformed/duplicate messages, pause/resume, `set_speed_scale`, and Reset versus
queued commands? Distinct from the known Rebase-reference leak (which is fixed at
this revision: `commandJobController.js` reset path calls `clearPlaybackState()`,
and `tests/js/hp-sim-3d/commandStreamRebase.test.js` covers it; I did not
re-report it).

## Short version

Three genuine protocol defects, one latent cluster, several clean negatives.

1. **A stopped flight recorder silently disables the whole external command
   protocol.** `handlePayload()` calls `FlightRecorder.recordEvent()` *before*
   dispatch; `recordEvent()` throws once `recorder.error` is set. The socket
   layer swallows it with one console warning, so commands are dropped and
   encoder requests go unanswered while the bridge believes everything is fine.
   Contradicts the in-code invariant "Backpressure applies to sampled physics,
   never to G-code dispatch" (`flightRecorder.js:167`). Genuine, user-reachable.
2. **Payloads that carry no motion un-pause the simulation.** The dispatcher's
   fallthrough calls `runtime.resume()` for *every* payload it does not
   explicitly recognise, including `{type:'reply'}` — which both bridges
   broadcast for every G-code line, including `M105`/`M114` status polls — and
   unknown/malformed-but-valid JSON. Genuine; contradicts "leave physics paused
   initially" (`extended-reference.md:120`).
3. **With two clients attached, an unresponsive newest client owns encoder/clock
   requests forever.** The timeout clears `preferredEncoderClient`, but the next
   call re-derives the same client ("highest connection sequence"), so the
   fallback path that exists for a *disconnected* client never runs for an
   *unresponsive* one. 3/3 requests went to the silent client; the healthy one
   was never asked. Genuine in a two-tab setup; the docs already say to use one
   tab, so this is a robustness defect with a documented workaround.
4. **(Latent) silent command loss / unenforced bound on the two backlogs**, and
   **Reset does not clear the browser backlog** (it is replayed after the
   rebuild). Measured, but the browser backlog is effectively dead code in the
   shipped app, so this is latent, not a shipped wrong result.

Negatives with numbers: attach/detach/reconnect is clean (one socket at a time,
no duplicate connects, no history replay to a late client, automatic reconnect
after close and after error); malformed JSON is ignored; duplicate commands and
duplicate encoder replies are handled correctly; `set_speed_scale` and command
ordering has no hazard in either order; `world.clear()` keeps systems, so the
remote spool system and its queue survive scene rebuilds (this refutes my own
first hypothesis that a machine swap loses queued commands).

## Method

Read: `hp-sim-3d/app/externalCommandSocket.js`, `hp-sim-3d/app/flightRecorder.js`,
`hp-sim-3d/app/commandJobController.js`, `hp-sim-3d/app/remoteSpoolSystem.js`,
`hp-sim-3d/app/runner.js`, `hp-sim-3d/app/workerController.js`,
`hp-sim-3d/app/machineSceneController.js`, `hp-sim-3d/app/sceneSystems.js`,
`hp-sim-3d/app/scene/sceneResources.js`, `hp-sim-3d/app/scene/machineScenePipeline.js`,
`integrations/rrf/rrfSimulatorBridge.mjs`, `integrations/klipper/klipperTerminalBridge.js`,
`autocal/control/primitives/encoder_utils.mjs`, `autocal/control/cli/collect_sweep_data.mjs`,
`src/python/hp_sim5_research/session.py`, `README_adv.md`, `hp-sim-3d/README.md`,
`hp-sim-3d/FLIGHT_RECORDER.md`, `autocal/data/references/extended-reference.md`, and
the existing tests (`autocalSettings`, `commandStreamRebase`, `rrfSimulatorBridge`).

Then micro-experiments, all stubbed, one process at a time, each under
`timeout 60` and `/usr/bin/time -v`:

| step | script | what it isolates |
|---|---|---|
| E1 | `probe_dispatch_table.mjs` | one row per inbound payload shape, through the real socket `message` path |
| E2 | `probe_recorder_blocks_commands.mjs` | recorder lifecycle × protocol dispatch (real `FlightRecorder`) |
| E3 | `probe_queue_overflow_and_reset.mjs` | browser backlog eviction, oversize message, Reset replay |
| E4 | `probe_reconnect_and_url.mjs` | attach/detach/reconnect, URL normalisation |
| E5 | `probe_bridge_two_clients.mjs` | real bridge server with an in-process `ws` stand-in, two clients |
| E6 | `probe_settings_barrier_and_speed_order.mjs` | settings barrier vs commands, speed/pause ordering |
| R1–R4 | `repro_*.mjs` | minimal one-command reproducers, exit nonzero on the defect |

E5 needs the real `integrations/rrf/rrfSimulatorBridge.mjs`, which imports the
bare `ws` package and would open a listening port. `ws-stub/fake-ws.mjs` is an
in-process `WebSocketServer`/socket pair (an `EventEmitter` with `clients`,
`connection`, `send`, `close`, `terminate`) and `ws-stub/ws-register.mjs` maps
the specifier `ws` to it with `module.registerHooks`. No socket is opened; the
bridge module itself is unmodified and runs its real routing code.

Tolerances: every claim here is a discrete protocol property (a message answered
or not, a state flag unchanged or not, a count exact), so no numeric tolerance
applies. Counts are exact integers.

## Finding 1 — a stopped flight recorder silently disables the external protocol (genuine, user-visible)

**Contract.** `flightRecorder.js:167`: "Backpressure applies to sampled physics,
never to G-code dispatch." `hp-sim-3d/README.md:47-50`: encoder replies
acknowledge the effective settings. The bridge's only feedback channel is
`encoder_response`. Nothing in the docs says the G-code/encoder bridge depends on
the optional Rerun recording.

**Code path.** `externalCommandSocket.js:242-243` records every inbound payload
*before* dispatch:

```js
if (payload.type !== 'encoder_request' || payload.axes?.length) {
  world.getResource('flightRecorder')?.recordEvent('external_payload_received', payload);
}
```

and `flightRecorder.js:192-194` throws whenever `error` is set
(`nonFatal` is false in the app: `appBootstrap.js:119-124` does not pass it):

```js
if (this.error) { if (this.nonFatal) return; throw new Error(this.error); }
```

The throw propagates out of `handlePayload`, is caught by
`createExternalCommandSocket`'s `message` listener
(`externalCommandSocket.js:98-104`), which logs
`"failed to process external G-code payload."` and returns. The payload is gone.

**Measured** (`evidence_recorder_blocks_commands.json`, `evidence_repro_recorder_blocks_protocol.out.json`):

| recorder state | commands accepted | encoder request (axes) answered | clock request (axes: []) answered | console warnings |
|---|---|---|---|---|
| recording, acked (control) | 1 / 1 | yes | yes | 0 |
| receiver closed mid-run | **0 / 1** | **no** | yes | 2 |
| receiver closed, 5 further commands | **0 / 5** | – | – | 5 |
| Rerun clicked, no receiver | **0 / 1** | – | – | 1 |
| clean `disconnect()` (error null) | 1 / 1 | yes | yes | 0 |
| `nonFatal: true` (tests only) | 1 / 1 | – | – | 0 |

`position_trace_mode` is also dropped (`traceModeApplied: false`). A single
`simulation_settings` message recovers the path (the code special-cases exactly
that payload at `externalCommandSocket.js:237-241` and disconnects the failed
recorder), which is what the existing test
`autocalSettings.test.js: "a disconnected recorder cannot prevent the next
collection from enabling closed loop"` exercises — commands are not covered.

**Why it matters.** A recording that was working and then drops (receiver
stopped, port taken, laptop slept) turns every later G-code line into a console
warning. The firmware/bridge side still gets HTTP replies, so `rrf_terminal`,
`autocal --sim` and the browser print recipe see success while the simulator
applies nothing: the run silently becomes a no-op (or never resumes from pause,
because the throw precedes the resume call). For autocal the
symptom is worse than silence: encoder requests with axes are not answered, so
`sendEncoderRequest` rejects after 2 s (clock) / 5 s (command) with
*"Browser simulator did not answer encoder request (A) … the browser may be busy
or suspended"* — a misleading diagnosis (see `rrfSimulatorBridge.mjs:124-127`),
while the clock poll (`axes: []`, which skips `recordEvent`) keeps succeeding so
the session looks half-alive.

**Reachability.** `?gcode_ws=…` plus any of: (a) click **Rerun** with no
receiver on 9877 — the documented browser-print step, and the button is in the
default control panel (`hp-sim-3d/index.html:187`); (b) an
`--extended-recording` session whose receiver exits mid-run; (c) a receiver
restart. (a) and (c) need no autocal at all. `extendedReference` does not matter:
the error check (line 192) precedes the `!this.extendedReference` early return
(line 196), so the default page without `?extended_autocal=1` is affected too.
Genuine, not latent.

**Reproducer.** `node repro_recorder_blocks_protocol.mjs` → exit 1.
Control traffic is accepted before the recorder loss; after it, the command is
dropped and the encoder request is never answered.

## Finding 2 — payloads without motion un-pause the simulation (genuine, low-medium)

**Contract.** `autocal/data/references/extended-reference.md:120`: "leave physics
paused initially". Pause is a user control (`hp-sim-3d/README.md:23`). Resuming
on a *command* is intended: `pushExternalCommands` already does
`setPrintActive(true); maybeResumeFromPause()` (`commandJobController.js:596-605`).

**Code path.** `externalCommandSocket.js:374-375`:

```js
pushCommands(batch);
runtime?.resume?.();
```

`runtime.resume` is `resumeSimulation` (`runner.js:583-597`: sets
`pauseState.paused = false`, restarts the loop). Every payload that reaches the
bottom of `handlePayload` hits it — `reply`, unknown types, `{}`, arrays, even a
bare JSON number. The recognised control types (`encoder_request`,
`simulation_settings`, `reset`, valid `set_speed_scale`, `position_trace_mode`,
`klipper_api_*`) return earlier and do not resume.

**Measured** (`evidence_dispatch_table.json`, `evidence_repro_reply_resumes_pause.out.json`):

| payload | command applied | run state |
|---|---|---|
| `{type:'command', command:{Move}}` | 1 | resumed (intended) |
| `{type:'reply', gcode:'M105', reply:'ok T:20.0'}` | 0 | **resumed** |
| `{type:'reply', reply:''}` | 0 | **resumed** |
| `{type:'heartbeat'}` (unknown) | 0 | **resumed** |
| `{}` | 0 | **resumed** |
| `[1,2,3]` | 0 | **resumed** |
| `42` | 0 | **resumed** |
| `{type:'position_trace_mode', enabled:true}` | 0 | stays paused |
| `{type:'set_speed_scale', value:25}` | 0 | stays paused |
| `{type:'reset'}` | 0 | stays paused |

5 of 5 non-motion payloads changed the run state in the reproducer; the probe's
extra `42` (a bare JSON number) resumes too.

Both bridges broadcast `reply` for every line they forward
(`rrfSimulatorBridge.mjs:298`, `klipperTerminalBridge.js:948`), so a temperature
or position poll during a pause resumes physics. Because `pushExternalCommands`
already resumes, the trailing call at line 375 adds nothing for commands — only
the unwanted cases.

**Second trigger, same mechanism (measured):** connecting the bridge resumes too.
`onOpen` pushes `{type:'Rebase reference'}` through `pushExternalCommands`
(`externalCommandSocket.js:391-395`), which sets print-active and resumes.
`connect_only.resumedAtOpen: true` in
`evidence_settings_barrier_and_speed_order.json`. A page that is paused per the
docs starts stepping (and shows a print as active) as soon as the WebSocket
opens, before any settings or command. This is adjacent to, but not the same as,
the fixed Rebase-origin bug: it is the *side effect of pushing a bookkeeping
command through the motion API*.

**Reachability.** Any `M105`/`M114`/`M115` poll (reply only, no command
broadcast) from an RRF or Klipper terminal while the simulator is paused; any
bridge connect on a paused page. Unmeasured consequence chain: `handleUserReset`
ends with `runtime.reset({autoPause:true})` and an empty command queue, so the
next reply restarts physics with nothing queued — uncommanded settling (the
48 mm HP5 layered drop of the 2026-11-16 report is that magnitude, not measured
here). Genuine but low severity: it cancels a pause and starts physics.

**Reproducer.** `node repro_reply_resumes_pause.mjs` → exit 1.

## Finding 3 — two clients: no failover after an encoder timeout (genuine, low-medium)

**Code path.** `rrfSimulatorBridge.mjs:27-45`: the preferred client is the one
with the highest connection sequence number (the newest). Line 152 sets it on
every connection. On timeout, lines 118-121:

```js
pendingEncoderRequests.delete(requestId);
if (preferredEncoderClient === encoderClient) preferredEncoderClient = null;
reject(new Error(`Browser simulator did not answer … within ${timeoutMs} ms; the browser may be busy or suspended`));
```

The next request calls `getPreferredEncoderClient()` again; with
`preferredEncoderClient === null` it recomputes the maximum sequence number and
picks the *same* client. The fallback at lines 137-143 only runs on `close`
(`tests/js/rrf/rrfSimulatorBridge.test.js: "falls back to an older ready client
when the preferred client disconnects"`). The timeout branch is therefore dead
with respect to failover.

**Measured** (`evidence_bridge_two_clients.json`, `evidence_repro_two_clients_sticky_encoder.out.json`),
two ready clients, the older one auto-answering in the same tick:

| attempt | timeout | request went to healthy client | request went to silent client |
|---|---|---|---|
| 1 | 50 ms (exceeded) | 0 | 1 |
| 2 | 50 ms (exceeded) | 0 | 2 |
| 3 | 50 ms (exceeded) | 0 | 3 |
| control after silent client disconnects | 200 ms | answered `[11]` | – |

With the production timeouts (`DEFAULT_COMMAND_ENCODER_TIMEOUT_MS = 5000`,
`DEFAULT_ENCODER_TIMEOUT_MS = 2000`) every encoder request pays the full 5 s
before failing and the error blames the browser; six of them exhaust the 30 s
HTTP command deadline (`rrfHttpBridge.js:53`), and collection wall deadlines
(120 s, `extended-reference.md:201-205`) follow.

**Reachability.** Two simulator tabs, or one tab plus a headless client
(`scripts/autocal_headless.mjs` is such a client: it attaches to the same
bridge and answers encoder requests from its own world), with the newest one
frozen/throttled/discarded but still connected. The docs say
"Use one simulator tab" (`extended-reference.md:127`) and `sendHpSimSettings`
has an error string about it, so this is a robustness/lifecycle defect rather
than a new contract violation. Genuine behaviour, documented workaround.

**Reproducer.** `node --import ./ws-stub/ws-register.mjs repro_two_clients_sticky_encoder.mjs` → exit 1.

## Latent cluster — silent loss and an unenforced bound in the two backlogs

Both protocol ends have a 5000-entry backlog that evicts the oldest silently, and
the browser bound is not applied to a single oversized `commands` array:

| check | sent | kept | loss reported |
|---|---|---|---|
| bridge pending queue (`MAX_PENDING_WS_PAYLOADS = 5000`, no client attached) | 5003 | 5000 (seq 3…5002) | **no** (0 log lines) |
| browser backlog, one `commands` message (`queueLimit = 5000`) | 6000 | **6000** (1000 over the bound) | no |
| browser backlog, steady drip | 5003 | 5000 (first 3 dropped) | no |

There is also no ack for a `command`: `sendGcodeLine` returns the *firmware*
reply, never whether the simulator applied anything, so a client cannot detect
any of this.

**Reset versus the browser backlog.** `{type:'reset'}` calls
`commands.handleUserReset()` immediately and the worker-side reset clears the
remote queue (`clearPlaybackState`), but the browser-side
`externalCommandQueue` is untouched. `machineSceneController.rebuildScene()`
ends with `getExternal()?.flushQueue?.()` (`machineSceneController.js:388`), so
the parked commands are applied *after* the reset. Measured: 3 commands parked
before `reset`, worker queue 0 after it, then 4 items replayed (3 commands + the
parked connect-time Rebase).

**Why this is latent, not a shipped wrong result.** I tried to refute my own
reachability claim and it did not survive:

* `controllers.external.connect()` runs *after* `await loadDefaultScene()`
  (`appBootstrap.js:295-311`), so the bridge does not connect before the first
  scene exists.
* `World.clear()` keeps systems and resources (`ecs.js:116-122`), the
  `RemoteSpoolSystem` is created once (`simulationSystems.js:16`) and
  `registerSceneSystems` only registers systems when
  `world.systems.length === 0` (`sceneSystems.js:14`). So `getRemoteSystem()`
  is non-null for the whole session and `pushExternalCommands` returns `false`
  only if no scene ever loaded (e.g. `setupScene` rejected the scene spec).
* Consequently the oversize/Reset-replay paths are close to dead code in the
  shipped app, and a machine swap does **not** drop queued commands (the queue
  survives `world.clear()`; the axis mapping is rebuilt by
  `machineScenePipeline.js:49-50`).

They remain real defects of the module contract (a configured `queueLimit` that
6000 items exceed; a Reset that replays commands received before it), and they
are reachable by any consumer that builds the controller before the world
(`tests/parity3d/*`, `scripts/autocal_headless.mjs` do exactly that) or in a
failed-scene recovery. Severity: low, latent.

**Reproducer.** `node --import ./ws-stub/ws-register.mjs repro_queue_loss_and_reset.mjs` → exit 1 (all three checks).

## Settings barrier, speed, and Reset ordering

**Settings barrier asymmetry (latent).** `simulation_settings` with
`machineType` / `recordingUrl` is applied behind a promise (`recordingReady`);
`encoder_request` replies wait for it (`externalCommandSocket.js:245-247`), but
commands do not. Measured with a pending machine load: an encoder request was
withheld until the load finished, while a command arriving in the same window was
accepted immediately (`evidence_settings_barrier_and_speed_order.json`). The
collector avoids this by awaiting an encoder ack before sending G-code
(`sendHpSimSettings`), so it needs a second, independently streaming client to
matter; then the command is queued for the *incoming* machine rather than the
outgoing one. Low, latent.

**`set_speed_scale` ordering is clean (negative).** With a faithful runner model
(`setTimeScale` resets the clock accumulators and is a no-op when unchanged):

| sequence | final speed | paused | queued |
|---|---|---|---|
| speed 25 then Move | 25 | false | 1 |
| Move then speed 25 | 25 | false | 1 |
| speed 25 only | 25 | **true** (does not resume) | 0 |

No ordering hazard in either direction, and a speed-only client does not silently
start physics.

**Invalid `set_speed_scale`.** `value: 0`, `-1`, `"25"`, `NaN` are ignored by the
`Number.isFinite && > 0` guard and then fall through to the command path, which
**resumes** the simulation while changing nothing. Silent, and the opposite of
the Python session, which raises on an invalid scale
(`src/python/hp_sim5_research/session.py:251-253`) and thereby ends the session.
Divergence between the two server implementations, latent.

**Reset forces 1× speed (code read only).** `handleUserReset()` calls
`runtime.setTimeScale(1.0)` (`commandJobController.js:173`), and the
`encoder_response.simulationSettings` payload has no `timeScale` field
(`externalCommandSocket.js:209-220`), so a client cannot observe that its
requested 25× was discarded. The supported autocal flow re-sends speed after
reset/settings (`collect_sweep_data.mjs:658-661` comments on exactly this), so
this is a hazard for a second client or a user pressing Reset, not a defect of
the documented flow. I could not execute this chain: `commandJobController.js`
imports a Vite-only `?worker&url` module and is not loadable under plain Node.

## Other protocol observations (latent / input handling)

* **`normalizeWsUrl` mangles absolute http(s) URLs.** `http://localhost:8790`
  becomes `ws://http://localhost:8790` (only a leading-slash prefix is stripped),
  so the constructor throws and the controller retries forever every
  `reconnectDelayMs` (5 s), logging only every tenth attempt — i.e. one line
  every 50 s, with no error surfaced. Latent: the docs say to use `ws://`.
  `evidence_reconnect_and_url.json`.
* **A hung handshake wedges the client.** If the socket neither opens nor closes
  (no `error`), `connecting` stays `true` and `scheduleReconnect` refuses to run
  (`externalCommandSocket.js:45,73`), so no retry ever happens. Requires a
  browser that never fires open/error/close; latent.
* **A throwing `close()` wedges the client too**: the `error` handler catches the
  throw (`externalCommandSocket.js:111-117`) and `socket` stays non-null, so
  `connect()` is a no-op from then on. Latent.
* **The two server implementations of the same protocol diverge.** The browser
  path ignores an invalid `set_speed_scale` (then resumes) and has no bound on
  the live command queue; `NativeSession` raises on an invalid scale
  (`session.py:251-253`) and refuses a queue beyond 60,000 commands
  (`session.py:262-263`), ending the session. A client that works against one can
  fail against the other. Latent, but worth knowing when comparing runs.

## Negative results (checked, no failure found)

* **Attach/detach/reconnect is clean.** One socket per attempt; `connect()` while
  open is a no-op; `send` returns false before open and while down (no throw);
  close and error both schedule exactly one reconnect after the delay; the tenth
  failed attempt logs the retry notice. A late client receives no history replay
  (`secondClientMessagesOnAttach: 0`), and remaining clients keep getting
  broadcasts after a peer detaches. `evidence_reconnect_and_url.json`,
  `evidence_bridge_two_clients.json`.
* **Malformed input is inert.** `{"type":"command",` → one parse warning and no
  state change; `{"type":"command"}` with no command applies nothing; `null`
  returns early; null entries in `commands[]` are filtered. A string command is
  queued verbatim (it becomes a no-op record in the spool system's history, not a
  crash). `evidence_dispatch_table.json` rows.
* **Duplicates.** A duplicated `command` message is applied twice. That is
  harmless for the absolute `Move`/mode commands that the G-code bridges send
  (setting the same target twice), but it would **accumulate** for
  `Add to reference`, which the step-batch paths do emit
  (`rrfHttpBridge.js:418`, `rrfCanPlayer.js:257`, `klipperMcuCommandPlayer.js:245`).
  No path resends a command today (next bullet), so this is a hazard to keep in
  mind if a resend is ever added. A duplicated `encoder_response` *is* handled:
  the first reply wins, later ones for the same `requestId` are ignored, and
  unknown request ids are ignored. `evidence_bridge_two_clients.json`.
* **No duplicate delivery across reconnects.** The bridge enqueues only when no
  client is ready; the browser flushes its parked queue *before* pushing the
  connect-time Rebase (`externalCommandSocket.js:391-395`), which is the correct
  order for a pre-disconnect backlog.
* **`world.clear()` keeps systems** — so the remote spool system, its queue and
  the axis remap survive scene rebuilds (measured directly: `systems.length` 1,
  queue 1 before and after).
* **Queue overflow does not throw** in either implementation; nothing crashes or
  corrupts state, it only loses work.

## Limitations

* All evidence is stub-level JS in Node, not a browser. The stubs reproduce the
  documented WebSocket surface (`addEventListener`, `readyState`, `send`,
  `close`) and the bridge's real routing code, but not browser event-loop
  ordering under load, tab freezing, or real `MessageEvent` cloning.
* No real socket or server was opened, by program rule. `ws` is replaced
  in-process; the bridge's own HTTP client was never exercised (`server` points
  at an unroutable port in all bridge probes).
* No physics, no native addon, no browser, no full app. Consequential claims
  (uncommanded drift while a recorder error hides commands) are stated as
  consequences of the code path, not measured trajectories.
* `commandJobController.js` is not importable under plain Node (Vite
  `?worker&url`), so the reset→1× speed and print-active consequences are code
  reads.
* I did not test the Klipper raw-upload bridge's `klipper_api_*` payload lifecycle
  beyond confirming it is gated by the same `recordEvent` call.
* Nothing here is hardware evidence.

## Reproducers

All commands run from this directory; each exits nonzero when the defect is
present and prints expected-vs-actual JSON first.

```bash
node repro_recorder_blocks_protocol.mjs                                         # finding 1
node repro_reply_resumes_pause.mjs                                              # finding 2
node --import ./ws-stub/ws-register.mjs repro_two_clients_sticky_encoder.mjs    # finding 3
node --import ./ws-stub/ws-register.mjs repro_queue_loss_and_reset.mjs          # latent cluster
```

Or all four: `./run_all.sh` (exit 1 if any defect is present).

## Evidence index

| file | content |
|---|---|
| `evidence_dispatch_table.json` | 25 payload shapes through the real socket path: applied commands, resume count, reset count, responses, warnings |
| `evidence_recorder_blocks_commands.json` | 8 recorder-lifecycle cases against the real `FlightRecorder`, including the default page (`extendedReference` false) |
| `evidence_queue_overflow_and_reset.json` | backlog eviction, oversize message, Reset replay |
| `evidence_reconnect_and_url.json` | connect/close/error/reconnect matrix, `normalizeWsUrl` table, hung-handshake and throwing-close cases |
| `evidence_bridge_two_clients.json` | two clients, encoder routing, silent-newest timeouts, pending overflow, attach/detach, malformed client messages |
| `evidence_settings_barrier_and_speed_order.json` | settings barrier vs commands; speed/pause ordering |
| `evidence_repro_*.out.json` | the four reproducers' own output |
| `harness.mjs`, `ws-stub/` | stubs used by every script |
