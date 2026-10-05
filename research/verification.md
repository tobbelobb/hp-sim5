# Verified integration behavior

Verified on 2026-10-05 using the repository `.venv`, Python 3.12.3, NumPy 2.4.3,
usd-core 26.3 and rerun-sdk 0.38.1. The initial live sessions used MCP SDK 1.30.0;
the final implementation and focused checks use its current 2.3.0 API. Codex reported an existing
ChatGPT login. No OpenAI API key was provisioned or used by the launcher.

## Live Codex physics experiment

The launcher ran an actual Codex task through hp-sim5 MCP. It constructed the
same native HP4 scene for a baseline and a trial, each with 200 steps at 2 ms.
The trial set axis A to 0.01 rad on the first step. Both completed with 201 finite
numeric samples, including step zero. Scene and native source hashes matched;
the command hash differed. Neither trial reported missed steps.

| Observation | Measured value |
| --- | --- |
| Baseline run ID | `59de4042e3e747bf910dc9957c710dc9` |
| Candidate run ID | `22f857e8987149ab93220cfc18f568a6` |
| Final simulation time | 0.4 s |
| Final candidate A encoder angle | 0.009999999915620332 rad |
| Distance between final effectors | 0.00015721724566650084 m |

The event log records successful `capabilities`, `run_experiment`,
`compare_experiments` and `read_experiment` calls. The session completed with
exit code zero. Its artifacts are under
`output/research/sessions/93c49db478ae4885aae10d8d4205eaa8/`.

## Live Codex Rerun inspection

A subsequent launcher session reused the candidate recording. Codex opened it
in the launcher-owned headless Viewer, inspected all six blueprint views,
selected `sim_step=200` with playback stopped, inspected Viewer logs and saved
`sim_step_200.png`. Viewer state reported no view warnings or errors. The PNG
was visually inspected: it shows the HP4 cable geometry and all five trace
panels at step 200. This session completed with exit code zero.

Artifacts are under
`output/research/sessions/338606b11fde452bb3886bb861a7e96c/`, including
`events.jsonl`, `report.md`, `viewer.log` and the screenshot. These generated
files remain locally available and are excluded from Git by `/output`.

The first live inspection exposed an approval configuration defect: Rerun calls
were refused by the unattended policy. The launcher now explicitly approves
its local simulation and Viewer tools for the invocation. The subsequent
successful calls verify that correction.

The final SDK 2.3.0 implementation was also checked in an actual Codex session,
`3725e1e99f2b4fa7a989ddd70b91aaa7`. Codex successfully ran a fresh three-step
numeric experiment and read its finite final sample at 0.006 s, then opened the
saved candidate in Rerun, selected step 200, inspected warnings and saved a
screenshot. The session exited zero. Its launch record confirms MCP 2.3.0 and
the OpenAI provider.

## Verification scope

Focused checks cover native input validation, frozen-scene replay, repeated
measurements, motion/torque observations, honest comparisons, MCP stdio execution,
prompt preservation, and real native RRD telemetry/lifecycle behavior. Run:

```bash
.venv/bin/python -m pytest -q \
  tests/python/test_research_agent.py \
  tests/python/cable_joints_3d/test_machine_recording.py
```

All 25 focused checks passed in this checkout. A launcher regression also proves
that the detailed report survives Codex's final-message output and that both
API-key environment variables are removed from the child process. The earlier
live sessions predate that report-path correction; their numeric and Viewer
evidence remains in the event logs and generated artifacts.

This proves a prompt can produce and inspect real simulation experiments.
It does not demonstrate an improved autocal solver, native online sweep
collection, physical robot transfer, or OmniSim runtime behavior.
