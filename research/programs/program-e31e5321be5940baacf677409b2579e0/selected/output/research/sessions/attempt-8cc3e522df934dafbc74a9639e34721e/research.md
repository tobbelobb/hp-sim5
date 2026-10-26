# CM-E1 saved-source audit notebook

Status: complete. Scientific evidence is inspected producer evidence; fresh review pending.

Objective: audit the four named archived sources for per-dataset nonworsening, aggregate effects, the exact common-unseen-fold gap and provenance limits. Produce a portable review submission without numerical replay.

Assignment: `cm-saved-audit`, revision 1, program `program-e31e5321be5940baacf677409b2579e0`, attempt `attempt-8cc3e522df934dafbc74a9639e34721e`. Assigned checkout revision: `a1b4e559887550da6cac6a4c67819a618a766f65`. The original assignment and recorded strategy remain in launcher-owned prompt.txt, outside the review submission; this notebook replaces its pending scaffold. Session path stayed fixed.

## Hypothesis and constraints

H1: unconditional finite-only final consensus is nonworsening for every additional CM-E1 dataset. H2: changed applied models have demonstrable common-unseen predictive superiority. Working alternative: aggregate improvement can coexist with per-dataset regressions; finite diagnostics with training overlap cannot settle H2.

The user authorized only four scientific sources: `report.md`, `results.json`, `audit.json` and `inputs.json` from preceding session `2026-10-23-consensus-membership`. Read bytes/JSON, hash, copy and check internal structure only. No other historical scientific evidence or external sources were consulted. Read repository research instructions/README/integration rationale and the abstract template for workflow; these are not scientific evidence. The cloud environment runtime skill was read; no environment_status tool was exposed, no network work was needed and no credentials were inspected.

Budget: eight wall minutes including startup; zero collections, fresh fits, physics runs/steps, predictions and selections. Recording: none, fixed by assignment. Actual numerical consumption: zero in every category. No probe.py, audit.py, preflight, doctor or supervisor/world command; no movement. No source/runtime/dependency edits, commits or manager graph memory writes. Existing launcher services and their logs are outside the review submission.

## Inspection record

- 2026-10-26 11:59 UTC: began four-source inspection. Historical experiment ID `CM-E1`; new inspection ID `CM-SAVED-A1`. Initial shape-printer successfully read the report and first two JSON files, then assumed inputs.json had `.items()`. It raised `AttributeError: 'list' object has no attribute 'items'`; the source is a JSON array. This was a failed exploratory printer, not a scientific trial or negative result. Corrected parsing used the array. This partial failure is also preserved in evidence.json; the original tool output is in the launcher transcript.
- 12:00–12:01 UTC: inspected all eleven rows and the exact changed-case diagnostic/training IDs. Copied four immutable sources under [sources/](sources/), totaling 219,323 bytes. Each copy matches its original SHA-256; no source missing.
- 12:01 UTC: ran the standard-library [extractor](extract.py). It performed 91 identity, shape, arithmetic and saved-set consistency checks, with zero failed checks. No numerical fitting, selection or prediction functions were imported or called. Re-ran after compacting JSON serialization; same checks and source identities.
- 12:02 UTC: wrote [report.md](report.md), separating aggregate error from per-dataset nonworsening and recording both variant-only unseen terminal IDs and the absence of a common fold.

The extraction checks stored summary arithmetic, not geometric distances or prediction costs. All historical numbers remain producer evidence. `audit.json`'s original-input/script validation flag is recorded, not independently reverified; raw/gzip envelopes and producer code are outside the four-source boundary. Eleven recorded content hashes are distinct but fixture-family independence does not follow. Manifest labels hp3-named entries as hangprinter_4; preserved as a provenance caveat.

## Outcomes and next decision

| Hypothesis | Inspection / historical experiment | Outcome | Evidence |
| --- | --- | --- | --- |
| H1: every additional dataset is nonworsening | CM-SAVED-A1 / CM-E1 | Rejected within archived scope | [Rows 6 and 9](evidence.json) |
| Additional cohort aggregate error decreases | CM-SAVED-A1 / CM-E1 | Supported as inspected producer evidence | [Saved summary](sources/results.json) |
| H2: changed cases establish common-unseen superiority | CM-SAVED-A1 / CM-E1 | Inconclusive | [Rows 6, 9, 10 and saved training sets](evidence.json) |
| Withhold unconditional adoption pending fair holdout evidence | CM-SAVED-A1 | Supported within archived scope | [Report](report.md) |
| Narrower truth-free rule, fresh fits, stopping or hardware gains | None | Not tested | Not funded |

Accepted steering: the bounded assignment and recorded strategy govern throughout; no later steering. The conditional recorded outcome is supported: one improvement, two regressions and five unchanged among the eight additional names, with no common unseen fold for changed models. This notebook makes no manager graph mutation.

Next decision: retire only the unconditional nonworsening default-change proposal for CM-E1's archived final-consensus comparison. Separately inspect saved later-sweep provenance on the three changed cases, prioritizing the two regressions, to establish an actual whole sweep outside all applied-member and spool-fit training. Such a search was not performed here and must not be counted as failed. Numerical evaluation or new collection would require a new allocation.

## References

- [Archived CM-E1 report](sources/report.md): preceding session's method, interpretation and limitations; outgoing links not followed.
- [Archived results](sources/results.json): numerical producer evidence and revision `5400cefc0238007406f5cd572d21ef5a72d2958d`.
- [Archived audit](sources/audit.json): recorded candidate training IDs and observation hashes.
- [Archived inputs](sources/inputs.json): fixture truth/cohorts, input hashes and fit producer revision `72523b9dc64155bf58fc1b8f0ea7241266ff73d4`.
- [Evidence extract](evidence.json): exact source hashes, JSON pointers, individual changed rows and all consistency check outcomes.
- [Full report](report.md) and [searchable abstract](abstract.md): completed finding and reviewer handoff.

## Completion verification

At 12:04–12:05 UTC, verified source-copy hashes, exact changed-row extraction against saved JSON, local report/notebook/abstract links and identical 100–200-word abstracts. All 91 saved consistency checks passed. The final review file list is [submission.json](submission.json); launcher logs/private attachment files are excluded. The session-local catalog was regenerated with `.venv/bin/python scripts/research_index.py --root <session>/catalog`, using `catalog/review -> ..` to index this session. The shared catalog was not written because this assignment permits writes only inside the session. This is a completed evidence audit, not a new calibration experiment.
