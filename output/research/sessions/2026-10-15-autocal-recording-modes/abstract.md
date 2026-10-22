# Autocal recording-mode cost on a frozen production trace

Status: complete.

## Abstract

We measured the time and disk cost of four full-autocal recording modes using
one frozen production HP3 command/encoder schedule. Four sequential repetitions
per mode replayed the same 200,000 fixed steps (400 simulated seconds), 30,758
commands and 1,060 nonempty encoder reads through the current native Rust engine.
All 16 trials had identical full-state checkpoint and encoder hashes; recorded
modes also had identical 34,017-event streams after timestamp/session
normalization and disk readback. Median replay, drain and finalization times were
4.82 s (none), 5.05 s (events), 5.49 s (sampled) and 15.23 s (full); RRD sizes were
0, 3.44, 10.96 and 54.23 MB, plus a shared 6.58 MB journal. These results support
cheaper modes for routine experiments. The replay excludes fresh planning,
adaptive collection, fitting and Python/collector logs. It does not reproduce
the original older engine's encoders: maximum disagreement was 943.55 degrees.
Exact equality was established only among the current mode comparisons.

## Topics

autocalibration, HP3, recording modes, Rerun, recorder overhead, frozen replay,
performance, experiment design, native Rust

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Removing physics recording reduces time/bytes while preserving current physics and command/encoder evidence | matrix/R1–R4, all modes | Supported for this frozen replay | [Raw matrix](matrix/results.json) |
| Stride-100 sampled recording offers intermediate cost | matrix/R1–R4 sampled/full | Supported for this frozen replay | [Raw matrix](matrix/results.json) |
| The current replay reproduces original older-engine encoders | pilot; matrix/R1–R4 | Rejected; 943.55 degree maximum difference in main workload | [Per-read observations](matrix/R1-none/checkpoints.json), source trace in [input](matrix/input/contract.json) |
| Final harness preserves measured trajectory and event evidence | revalidation/R1, all modes | Supported; same checkpoint/schedule/event hashes as main matrix | [Revalidation](revalidation/results.json) |
| These ratios predict total full-autocal wall time or other-backend outcomes | none | Not tested | Scope excludes fitting, planning and adaptive decisions |

## References

- [Full report](report.md): methods, result table and interpretation.
- [Raw matrix](matrix/results.json): per-trial timings, bytes, identities, equality gates and native artifact metadata.
- [Frozen input contract](matrix/input/contract.json): exact source/prefix hashes and source manifest beside it.
- [Original accepted source job](../2026-10-10-supervised-full-autocal/native/autocal/2ec0b06774624962a18bdbd2818eb134/sweeps.headless/manifest.json): actual production HP3 schedule; not an equivalent original-engine trajectory or timing baseline.
- [Final harness revalidation](revalidation/results.json): four additional trials reproduce the main matrix hashes after deferring reads to saved response boundaries.
- [Pilot](pilot/results.json): short 20,000-step / 32-command preliminary check; not used for headline numbers.
- [Archived measured harness](harness-v1/autocal_recording_modes.py) and [Node runner](harness-v1/autocal_recording_modes.mjs): exact measured source; main Git base aba370774108cd856e88b524e4b65658021b4d7c with [dirty diff](matrix/working.diff) and hashes in results.
- [Published cost guidance](../../../../research/autocal-recording-performance.md): researcher-facing numbers, scope and reproduction command.
- [Research notebook](research.md): accepted steering, budgets and boundary audit.
