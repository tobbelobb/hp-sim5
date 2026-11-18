# New reproducible correctness failures in hp-sim5 (2026-11-18)

Program: `program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7`
Code under test: `74ecdc95` (only submodule pointers dirty). Environment: Linux x64,
Node 26.13.0, Python 3.12, checked-in native addon and checked-in
`public/wasm/cable_solver.wasm`. All evidence is simulation; no hardware claim.
Nothing was fixed and no source file was edited.

Every headline below was re-run by the manager on this checkout and fails as
documented. Each reproducer is one command from its session directory; a failing
reproducer exits nonzero. Prior findings from the 2026-11-16 and 2026-11-17 programs
are excluded; those programs' negative controls were reused where relevant.

## Findings

| # | Wrong behaviour | Reachability | Measured error | Repro |
|---|---|---|---|---|
| 1 | G-code upload path mis-accounts extrusion (relative-E default, `M83` and `G92 E` ignored, `last_e` leaks across jobs) | Interactive `.gcode` upload; shipped logo file | 49,950.460 vs 49,719.253 mm (**-231.21 mm, -0.463 %**); 337x if `M82` absent; 2nd job 5 mm -> 0 mm | `repro_gcode_extrusion_bookkeeping.mjs`, `repro_shipped_gcode_extrusion.mjs` |
| 2 | Quality History is recorded when the worker stops sending, before the world drains its queue | Every linear (non-ASAP) print | **680/1000** samples, 2/3 corners, score **100.0** vs completed **94.313** | `repro_quality_recorded_before_drain.mjs` |
| 3 | A stopped flight recorder silently disables the external bridge | Click **Rerun** with no receiver, then connect a G-code client | 0/1 commands accepted, encoder requests unanswered, clock replies still work | `repro_recorder_blocks_protocol.mjs` |
| 4 | Any payload without motion resumes the simulation | Paused run + bridge `reply`/unknown message | 5/5 non-motion payloads resumed; `{type:'reply'}` is sent for every `M105`/`M114` | `repro_reply_resumes_pause.mjs` |
| 5 | Python/native length telemetry aliases `commanded` to `actual`; `error` is always 0 | Every `run_experiment`, `read_run`, research-MCP read, native `.rrd` | JS reports **-15.0867 mm** error; Python/native report **0.000000**; torque mode not omitted | `repro_length_telemetry.py` |
| 6 | Native recordings write all 33 rigid members at the parent body pose and omit motors/encoders/velocities/tool points/extrusions | Every native `.rrd` | Worst member **0.424359 m** from true pose; 33/33 wrong; documented channels absent | `repro_recording_scene_content.py` |
| 7 | In-place reset is not a distinct replayable generation | Any native session reset | `scene_generation` stays 1, no timeline, `sim_step` repeats 0..50; poses 4.118 mm apart share step 50 | `repro_reset_generation.py` |
| 8 | Authored pulley inertia is ignored on frictionless passive wheels | Any cable over free wheels; shipped HP4 D-line idlers | 3.270 vs 2.4525 m/s^2 (**+33.3 %**, one pulley); +39.0 % with four HP4 idlers | `repro_atwood_wheel_inertia.mjs` |
| 9 | No gyroscopic term for a freely rotating asymmetric body | Latent: shipped bodies are symmetric | 14.19 deg orientation error, angular momentum direction off 27.25 deg after 0.5 s | `repro_gyroscopic_free_body.mjs` |
| 10 | `fastModeActive` mirror survives worker replacement | Every job after the first | Job 2 worker never receives `set_fast_mode(true)` at queue 159 < 160 | `repro_worker_replacement_state.mjs` |
| 11 | Two-client encoder requests stick to the newest (possibly silent) client | Two tabs (documented as unsupported) | 3/3 requests to silent tab; healthy tab never asked | `repro_two_clients_sticky_encoder.mjs` |

Latent extras preserved in the reports: the 5000-entry bridge backlogs evict oldest
silently and a 6000-command message overflows the browser limit; extrusion deposit
positions round-trip as NaN; the headless E contract says metres while browser players
feed millimetres; `physicsSubsteps` is ignored without a `RigidBodyComponent`;
`state.simDtSec` can keep the previous scene's dt.

## Severity and classification

- **Shipped, user-visible numerical error:** 1 (uploaded prints deposit 0.46 % less
  filament and never emit 121 unretract primes), 2 (saved quality record hides the
  last 0.64 s of every print), 8 (line dynamics over free wheels are too fast because
  authored inertia is dropped).
- **Shipped, user-visible functional error:** 3 (bridge dead after a recorder
  failure), 4 (paused run silently resumes), 10 (lost prefetch after the first job).
- **Research/telemetry trust:** 5, 6, 7. The physics state itself is fine; the
  observation layer is not, and `peak_abs_length_error_m` can never be nonzero.
- **Latent / non-default:** 9, 11, and the extras above.

## Strong negative results (keep)

- **Replay/serialization is sound.** Two recordings compare equal; `.rrd` vs
  telemetry within 1.9e-9 m; recording does not perturb physics; pause/resume and
  generation changes work inside `FlightRecorder`; handoff export/import verifies
  hashes and detects tampering. There is no record-resume feature, despite the
  question's premise.
- **Preset/RRF playback conserves extrusion** to -0.0015 mm (2.9e-8 relative) on the
  same shipped file that the upload path mis-accounts. Pause/resume deposits exactly
  once; retract/zero E deposits nothing.
- **Integration is first-order PBD as documented.** Free mass position error is
  exactly `-1/2*g*t*dt` with fitted order 1.000000; velocity exact; substeps reduce
  the error exactly 1/n; pendulum period +0.00075 %; no divergence or energy gain on
  the tested range.
- **Worker boundary is clean otherwise.** Substep changes preserve the
  one-command-per-`update` contract; superseded workers' late commands are gated out;
  a paused world consumes and duplicates nothing; every UI->worker message type has a
  handler; `set_dt`/`set_speed_scale`/`set_asap_mode` are re-sent per job.
- **Bridge lifecycle is clean otherwise.** Attach/detach/reconnect uses one socket with
  no duplicate history; malformed JSON is inert; duplicate encoder replies are
  dropped; speed/command ordering is harmless either way; machine swaps keep the
  queue.

## Limitations

- No browser/UI automation. Findings 2, 3, 4 and 10 were established from the
  production modules with stubbed workers/sockets and real `World`/`RemoteSpoolSystem`/
  `QualityMonitor` objects; the visual UI was read, not driven. Finding 2's reproducer
  mirrors four boundary lines because `workerController.js` imports Vite-only modules,
  and fails on source drift.
- Finding 1 is a text-level driver of the production `MoveCommander` with a large dt;
  extrusion sums are dt-independent by construction, and an independent text-level
  emulation reproduces the total.
- Finding 8 is a minimal Atwood over authored wheels, not an end-to-end HP4 print. It
  shows the authored inertia never enters the dynamics; the machine-level consequence
  is inferred, not measured.
- Findings 5-7 use the checked-in native addon through a byte copy to avoid a rebuild;
  finding 5 was independently reproduced on the checked-in Wasm build (agreement
  1e-9 m).
- Prior-program review guidance is adopted: latent/API-order/invalid-input hazards are
  labelled, not counted as shipped wrong results.

## Resource note

Host has 3.8 GiB RAM and 2 GiB swap with another user's headless Chrome active. All
runs were micro-experiments: <=2.9 s wall, <=205 MB RSS, one process at a time, no
builds after the first accidental native refresh, no browser, no test suite. No OOM.
