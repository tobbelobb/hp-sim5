# Supervisor-owned HP3 full-autocal through restricted MCP clients

Status: complete for tooling validation; calibration quality remains concerning.

## Abstract

This session validates launcher-owned asynchronous full-autocal jobs through MCP with client socket/connect syscalls denied by Linux seccomp. A fresh HP3 production JS/RRF job with known 30 mm radii, zero buildup, no layering and manual 0.01/0.23/9.2 N forces automatically accepted at iteration six through the default patience rule. It collected eight sweeps and 320 physical measurements over 4198.148 simulated seconds and 780.154 wall seconds. The finalized 1.13 GB RRD contains 209,908 physical samples, all 320 measurements, six collection completions and exact final dataset/log/stage artifacts. The attached HP4 world stayed at step zero. Physical cancellation, HTTP/SIGTERM shutdown, owner SIGKILL and physics-budget exhaustion retained readable partial recordings. An earlier default-autotuning trial failed pullout settling and is preserved. The selected fit score was 6018, which the CLI calls concerning. This establishes complete workflow access and recording integrity, not calibration accuracy or hardware behavior. A direct Codex chat attachment was unavailable; real restricted SDK stdio MCP was exercised.

## Topics

autocalibration, HP3, HP4, supervisor, MCP, cancellation, process groups, Rerun, sandbox

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Socket-denied client can start and inspect real jobs through MCP | a792904dd547473ca0f6583f5ba547b9, 2ec0b06774624962a18bdbd2818eb134 | Supported | [First start](mcp-start.json), [retry start](mcp-start-2ec0b06774624962a18bdbd2818eb134.json) |
| Supervised physical cancellation, shutdown, crash and budgets finalize partial recordings | lifecycle trials | Supported | [Retained lifecycle evidence](lifecycle/) |
| Default autotuned HP3 zero-buildup bootstrap completes | a792904dd547473ca0f6583f5ba547b9 | Rejected for this trial | [Failed manifest](native/autocal/a792904dd547473ca0f6583f5ba547b9/job.json) |
| Bounded manual-force HP3 job physically reaches automatic acceptance | 2ec0b06774624962a18bdbd2818eb134 | Supported for workflow completion; quality concerning | [Acceptance](acceptance.json), [RRD verification](rrd-verification.json), [status](mcp-status-2ec0b06774624962a18bdbd2818eb134.json) |

## References

- [Research notebook](research.md): objective and budgets.
- [Full report](report.md): pending validation results.
- [Implementation request](../../../../research/prompts/tooling_improvement_1.txt): acceptance requirements.
- [Earlier force investigation](../../autocal-comfortable-force-20261010/report.md): preceding force-tuning limitations; the new default-autotuning trial still failed settling.
- [Compatibility regressions](../../../../autocal/data/references/fifth_hp3_dataset.full_auto_reference_run_october_8_2026.log) and [seventh HP3 reference](../../../../autocal/data/references/seventh_hp3_dataset.full_auto_reference_run_october_8_2026.log): unchanged solver outputs.
