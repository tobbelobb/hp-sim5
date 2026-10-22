# Autocal flag audit and bounded experiment handoff

Status: complete static investigation; numerical experiments not tested.

## Abstract

This audit asks which autocal options deserve further tuning, consolidation or
removal. Static parser extraction and a literal search across 13,863 local files
produced an inventory of 92 option spellings with defaults, KEEP annotations and
source/log excerpts. Source tracing grouped them by numerical solver, physical
model, filtering, candidate ranking, collection and output behavior. The handoff
proposes bounded experiments, matched frozen-stage replay, physical/held-out error
metrics and explicit acceptance rules. Scale-polish placement, filtering/noise
normalization and solver budgets are prioritized. No numerical experiments were
run, no default was established as optimal, and no runtime flags were removed.
Literal mentions include help and planned commands and do not establish executed
feature use. Earlier simulation research constrains the proposed tests; no new
hardware or calibration-accuracy result is available.

## Topics

autocalibration, CLI flags, parameter tuning, robust filtering, scale ambiguity,
spool radius, candidate ranking, solver budgets, experiment design

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Numerical defaults can be improved or hard-coded | E1–E7 proposed | Not tested | [Experiment handoff](report.md) |
| Literal usage evidence and parser definitions can be inventoried | A01 | Supported as static tool check | [Inventory](inventory.md), [raw excerpts](usage.json) |

## References

- [Report](report.md): assessment, proposals and prior research references.
- [Notebook](research.md): scope, evidence limits and next decision.
- [User KEEP annotations](../../autocal-flags.md): preserved preferences.

Six independent [researcher handoffs](handoffs/README.md) now partition the work
for concurrent execution, with isolated artifacts and a later integration check.
No numerical results were added while preparing these tasks.
