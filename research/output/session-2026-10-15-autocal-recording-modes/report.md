# Autocal recording-mode cost on a frozen production trace

Status: complete. Date: 2026-10-15.

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

## Results

| Mode | Median replay/drain/finalize s | Median complete trial s | Median RRD MB | Shared journal MB |
| --- | ---: | ---: | ---: | ---: |
| none | 4.82 | 5.15 | 0.00 | 6.58 |
| events | 5.05 | 5.77 | 3.44 | 6.58 |
| sampled | 5.49 | 6.20 | 10.96 | 6.58 |
| full | 15.23 | 15.91 | 54.23 | 6.58 |

MB means decimal MB. Replay timing includes journal writes, equal checkpoint
work, socket backpressure/drain and recorder finalization. Complete trial time
also includes startup, Node imports, construction, separate 5,000-step JIT warmup
and result/checkpoint serialization; readback is excluded. Four runs/mode, cyclic
mode order, no concurrent benchmark/tests. Host: i7-12700K, Node v24.14.1, Python
3.12.3, Linux 6.8.0. Sampling 100/10 with compact geometry and batched events.

## Equivalence and method audit

All 16 trials passed exact equality of 202 full-state checkpoints plus every
nonempty encoder read, the command schedule, original/baked scene, loaded native
artifact, steps/command/read counts and journal size. Twelve recorded trials
also finalized without rejected messages and passed exact normalized event
stream equality after disk readback (34,017 events each). Recorded wall time,
monotonic time and random session IDs are removed from event comparison.

A request-before-drain concern was audited: every nonempty request/response pair
in this chosen prefix occurs at the same recorded step. Thus main measurements
are valid; they were not superseded by that concern. The final runner evaluates
reads at saved response steps and reproduced the main checkpoint/schedule/event
hashes in four additional 200,000-step trials. These verification timings are
kept separate and do not change the four-repeat medians.

The replay is explicitly not accepted as reproducing the original older engine:
943.55 degrees maximum encoder disagreement. No accuracy inference or timing
comparison against that original full job follows. All four modes match each
other exactly under the current engine. Other backends, later runs and adaptive
full-autocal outcomes are not guaranteed to match.

## Decision

Use none for large numerical sweeps and events as the ordinary research default.
Sampled adds useful motion/force evidence at modest measured overhead here;
full is justified when denser physics inspection is needed. Within this replay,
events reduced replay/drain/finalization wall time by 67% and RRD bytes by 94%
relative to full. Full-job savings depend on fitting/planning/event-artifact mix.
No full-autocal run or alternate backend was benchmarked in this session.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Removing physics recording reduces time/bytes while preserving current physics and command/encoder evidence | matrix/R1–R4, all modes | Supported for this frozen replay | [Raw matrix](matrix/results.json) |
| Stride-100 sampled recording offers intermediate cost | matrix/R1–R4 sampled/full | Supported for this frozen replay | [Raw matrix](matrix/results.json) |
| The current replay reproduces original older-engine encoders | pilot; matrix/R1–R4 | Rejected; 943.55 degree maximum difference in main workload | [Per-read observations](matrix/R1-none/checkpoints.json), source trace in [input](matrix/input/contract.json) |
| Final harness preserves measured trajectory and event evidence | revalidation/R1, all modes | Supported; same checkpoint/schedule/event hashes as main matrix | [Revalidation](revalidation/results.json) |
| These ratios predict total full-autocal wall time or other-backend outcomes | none | Not tested | Scope excludes fitting, planning and adaptive decisions |

## References

- [Raw matrix](matrix/results.json): per-trial timings, bytes, identities, equality gates and native artifact metadata.
- [Frozen input contract](matrix/input/contract.json): exact source/prefix hashes and source manifest beside it.
- [Original accepted source job](../2026-10-10-supervised-full-autocal/native/autocal/2ec0b06774624962a18bdbd2818eb134/sweeps.headless/manifest.json): actual production HP3 schedule; not an equivalent original-engine trajectory or timing baseline.
- [Final harness revalidation](revalidation/results.json): four additional trials reproduce the main matrix hashes after deferring reads to saved response boundaries.
- [Pilot](pilot/results.json): short 20,000-step / 32-command preliminary check; not used for headline numbers.
- [Archived measured harness](harness-v1/autocal_recording_modes.py) and [Node runner](harness-v1/autocal_recording_modes.mjs): exact measured source; main Git base aba370774108cd856e88b524e4b65658021b4d7c with [dirty diff](matrix/working.diff) and hashes in results.
- [Published cost guidance](../../../../research/autocal-recording-performance.md): researcher-facing numbers, scope and reproduction command.
- [Research notebook](research.md): accepted steering, budgets and boundary audit.
