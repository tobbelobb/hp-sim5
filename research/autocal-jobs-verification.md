# Supervisor-owned full-autocal jobs

Verified 2026-10-10 against Git base `8cdfdac43cecdb45f65fec190feb6a13c1d0716a`
with the implementation changes in this checkout.

`start_autocal`, `autocal_status` and `cancel_autocal` delegate to the
launcher-owned supervisor. Each job runs the production CLI once in a separate
HP3/HP4 JS/RRF world. Structured settings bound optimization, physical steps,
collection forces and wall time. New artifact directories are confined to session
artifacts. Cancellation bypasses interactive accept-best-so-far, drains physics
before the recorder and retires the CLI process group, including after owner
failure. No arbitrary commands or external firmware/recorder targets are accepted.
See [settings and lifecycle](README.md#jobs-shared-browser-state-and-steering).

## Physical acceptance check

Job `2ec0b06774624962a18bdbd2818eb134` ran with known 30 mm radii, zero buildup,
layering requested disabled and explicit manual forces 0.01/0.23/9.2 N. The existing CLI
physically collected eight sweeps / 320 measurements and performed six
fit/plan/select iterations. Its default patience rule accepted after three
non-improving attempts, before the 20-iteration budget. It exited zero after
4198.148 simulation seconds and 780.154 wall seconds. No acceptance thresholds
were changed. The attached HP4 session retained its identity and step zero.

The SDK client was denied socket/connect syscalls using Linux seccomp after the
external MCP process started. MCP returned the job ID in 0.002766 seconds and
reported progress and acceptance. No research MCP connector was attached to the
current Codex chat, so direct launcher-attached Codex/desktop verification remains
a check for a new session; the actual restricted SDK stdio path was exercised.

The finalized 1,131,490,605-byte RRD is readable through `RrdReader.store()`.
Independent inspection found 209,908 physical samples, all 320 measurements,
six complete collections, the physical stop boundary, seven embedded stage
artifacts and exact final dataset/log content. Frozen scene and firmware hashes
match; every registered child PID was absent after completion.

The CLI called its final score of 6018 concerning. Automatic workflow acceptance
does not establish calibration accuracy. The preceding default-autotuning job
failed during pullout settling after 120 simulation seconds; it is preserved
with a finalized partial recording. The successful retry used explicit force
inputs through the existing CLI, without replacing its workflow or stopping rule.

## Checks and evidence

```bash
.venv/bin/python -m pytest -q tests/python/test_autocal_jobs.py tests/python/test_research_agent.py -k 'autocal or stdio' -m ''
.venv/bin/python -m pytest -q autocal/tests/test_autocal_no_collect.py autocal/tests/test_autocal_score_ui.py autocal/tests/test_autocal_cli.py tests/python/test_research_agent.py tests/python/test_native_collector.py -m 'not slow'
.venv/bin/python -m pytest -q autocal/tests/test_headless_sim.py -m 'not slow'
.venv/bin/python autocal/tools/regress_calibration_logs.py --dataset-name fifth_hp3_dataset --dataset-name seventh_hp3_dataset --jobs 1 --color never
.venv/bin/python output/research/sessions/2026-10-10-supervised-full-autocal/verify.py
.venv/bin/python output/research/sessions/2026-10-10-supervised-full-autocal/inspect_evidence.py
.venv/bin/python scripts/research_index.py
```

The focused suite passed 34 checks, the existing relevant suite passed 120,
and 12 headless checks passed. The two HP3 replay regressions were exactly equal
to their references. Real physical cancellation, HTTP/SIGTERM supervisor exit,
owner SIGKILL and physics-budget exhaustion finalized readable partial recordings.
Synthetic lifecycle tests additionally cover startup errors, concurrent cancel,
orphan retirement and acceptance versus iteration-budget classification.

The accepted supervisor retained an earlier progress counter reporting only its
120 bootstrap points. The final code counts subsequent journals too, covered by
a focused regression test. Final dataset and RRD counts independently verify
320 measurements. Restart launcher/MCP processes to load edited Python sources.

The [session report](../output/research/sessions/2026-10-10-supervised-full-autocal/report.md)
contains exact commands, retained failures, limitations and artifact links;
[acceptance summary](../output/research/sessions/2026-10-10-supervised-full-autocal/acceptance.json)
and [archive verification](../output/research/sessions/2026-10-10-supervised-full-autocal/rrd-verification.json)
provide numerical evidence. Session artifacts are ignored by Git and can be
absent in another checkout. Calibration accuracy, default-autotuning settling,
full HP4 calibration and hardware transfer remain separate research work.

## Settling follow-up

The [settling fix and commit review](autocal-settling-review.md) reproduces the
default-force failure exactly and fixes endpoint holding without relaxing
measurement settling. It also corrects an inherited configuration inconsistency:
unset layering enabled layered physics while job status reported it disabled.
The fix explicitly disables layering and rebakes initial lengths with zero width.
The original manual-force acceptance above remains workflow evidence; it does
not validate a physically consistent no-layering calibration.

With the corrected bake, a new default-force HP3 job accepted automatically after
seven iterations / 360 points, with 2.604 mm applied-anchor RMSE. Independent
inspection verified the finalized 1.58 GB RRD and all measurements. This is one
complete simulated trial; fewer points and hardware transfer are not established.
