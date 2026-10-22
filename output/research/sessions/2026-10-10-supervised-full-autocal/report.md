# Supervisor-owned HP3 full-autocal through restricted MCP clients

This session validates launcher-owned full-autocal jobs through MCP with client socket/connect syscalls denied by Linux seccomp. A fresh HP3 production JS/RRF job with known 30 mm radii, zero buildup, no layering and manual 0.01/0.23/9.2 N forces automatically accepted at iteration six through the default patience rule. It collected eight sweeps and 320 physical measurements over 4198.148 simulated seconds and 780.154 wall seconds. The finalized 1.13 GB RRD contains 209,908 physical samples, all measurements, six collection completions and exact final dataset/log/stage artifacts. The attached HP4 world stayed at step zero. Lifecycle checks retained readable partial recordings. The earlier default-autotuning trial failed pullout settling and is preserved. The returned score was 6018, concerning according to the CLI. These are simulated workflow/recording checks, not calibration accuracy or hardware evidence.

Status: complete for tooling implementation and validation.

Exact acceptance command:

```sh
.venv/bin/python output/research/sessions/2026-10-10-supervised-full-autocal/verify.py
```

The runner opens MCP stdio first, then denies socket/connect syscalls in the client with Linux seccomp. The existing MCP process and launcher-owned supervisor remain outside that restriction. This is a real restricted SDK client integration check; no research MCP connector is attached to the current Codex chat.

Failed full trial: [a792904dd547473ca0f6583f5ba547b9 manifest](native/autocal/a792904dd547473ca0f6583f5ba547b9/job.json) and [collector log](native/autocal/a792904dd547473ca0f6583f5ba547b9/sweeps.full_auto.log). After successful force and travel tuning, the first pullout did not settle within 120 simulation seconds. Status correctly returned failed / exit 1; partial recording finalized with 66,642 physical observations, no rejected messages and no automatic acceptance. This trial is preserved. A previous test's measurement-wait deadline failed during tuning; its temporary artifacts were removed by pytest before copying, and cannot be offered as preserved evidence.

Lifecycle evidence: [cancellation](lifecycle/cancel/), [HTTP shutdown](lifecycle/shutdown/), [owner SIGKILL](lifecycle/owner-crash/), [physics budget](lifecycle/physics-budget/). These real production-world tests stop during physical force tuning; they establish lifecycle behavior and readable partial recordings, not calibration accuracy. Original absolute paths remain in the copied manifests; source-directory.txt identifies the copied trial, and the RRD files are present beside those manifests.

Checks so far:

```sh
.venv/bin/python -m pytest -q tests/python/test_autocal_jobs.py -m 'not slow'
.venv/bin/python -m pytest -q tests/python/test_autocal_jobs.py -m slow
.venv/bin/python -m pytest -q autocal/tests/test_autocal_no_collect.py autocal/tests/test_autocal_score_ui.py autocal/tests/test_autocal_cli.py tests/python/test_research_agent.py tests/python/test_native_collector.py -m 'not slow'
.venv/bin/python -m pytest -q autocal/tests/test_headless_sim.py -m 'not slow'
.venv/bin/python autocal/tools/regress_calibration_logs.py --dataset-name fifth_hp3_dataset --dataset-name seventh_hp3_dataset --jobs 1 --color never
```

120 existing relevant checks pass; 12 headless checks pass; the final focused lifecycle/MCP suite passes 34 checks. Both selected replay regressions are exactly equal to their reference outputs (7 and 5 iterations). These replay regressions are used only for algorithm compatibility, not as substitutes for physical acceptance. The final targeted artifact/progress/retirement check passes five tests after the progress-counter correction. `git diff --check` passes.

## Accepted physical job

[Acceptance summary](acceptance.json), [final MCP status](mcp-status-2ec0b06774624962a18bdbd2818eb134.json), [job manifest](native/autocal/2ec0b06774624962a18bdbd2818eb134/job.json), [CLI output](native/autocal/2ec0b06774624962a18bdbd2818eb134/stdout.log).

Job ID: `2ec0b06774624962a18bdbd2818eb134`. Decisions: collect, collect, collect, collect, collect, accept. The last three iterations record non-improvement counts 1, 2, 3 with default patience 3. Automatic reason is `patience-or-threshold`; exit code is 0. It stopped at iteration 6 of the 20-iteration budget, using 2,099,074 of 5,000,000 physics steps and 780.154 of 3600 wall seconds. No manual stop file, stop-cost/std override, replay, synthetic collector or budget acceptance was used.

The production collector performed bootstrap travel measurement and five subsequent planned collections. Explicit manual forces were supplied through the existing CLI, following the default-autotuning failure. Optimizer mode fast, lbfgsb, four restarts and 400 solve iterations preserve the existing workflow. The selected anchors are saved in firmware_final and applied_parameters in the headless manifest. Source hashes, frozen scene/config files and command arrays preserve actual inputs. The final CLI selected a concerning score (6018), so no successful calibration accuracy claim is made.

The complete RRD is [hangprinter-20261010T202027.593662Z-0f8cff1f.rrd](native/autocal/2ec0b06774624962a18bdbd2818eb134/sweeps.headless/hangprinter-20261010T202027.593662Z-0f8cff1f.rrd). It has 209,908 physical samples and 383,582 events, no rejected messages, acknowledged drain and finalized status. Independent archive inspection verifies 320 measurement events, six collection-complete events, a physical service-stop event, seven embedded stage artifacts, exact final dataset/log content, and matching original/baked scene and firmware hashes. All registered job child PIDs were absent after completion. [Verification results](rrd-verification.json) and [inspection script](inspect_evidence.py).

```sh
.venv/bin/python output/research/sessions/2026-10-10-supervised-full-autocal/inspect_evidence.py
.venv/bin/python -m pytest -q tests/python/test_autocal_jobs.py tests/python/test_research_agent.py -k 'autocal or stdio' -m ''
.venv/bin/python -m pytest -q tests/python/test_autocal_jobs.py -m 'not slow' -k 'artifact_directory or retires_children'
.venv/bin/python scripts/research_index.py
```

## Limits and remaining integration

The accepted trial used explicit manual forces; default no-buildup autotuning still produced a physical settling failure. That is retained evidence and a separate collection research issue. Full HP4 calibration accuracy and hardware transfer were not tested. No direct research MCP connector was attached to this Codex chat; the real SDK client was restricted after spawning the external MCP process. Direct launcher-attached Codex/desktop verification with command networking disabled remains an integration check for a new chat. No sandbox policy was changed by the implementation.

A progress fix landed while the successful supervisor remained running: its status continued to count only 120 bootstrap points. The final code sums all per-sweep journals, tested separately; final dataset and RRD independently establish 320 actual measurements. Source hashes in the saved manifests distinguish the executed source from later edits. Restart the launcher/MCP process to load new tools and subsequent edits. The initial measurement-wait test's temporary evidence could not be retained before pytest cleanup; this record preserves its failure honestly without inventing artifacts.

References: [request](../../../../research/prompts/tooling_improvement_1.txt) defines the acceptance checks; [runtime/CLI guide](../../../../research/README.md) describes deployment and settings; [extended reference guide](../../../../autocal/data/references/extended-reference.md) defines the existing recording contract; [previous force investigation](../../autocal-comfortable-force-20261010/report.md) documents preceding tuning limitations. Git base revision: `8cdfdac43cecdb45f65fec190feb6a13c1d0716a`; the session's working.diff records the implemented source changes.
