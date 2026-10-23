# Research handoffs retain scoped evidence after worktree turnover

Status: complete.

## Abstract

Can another researcher inspect a useful result after its original worktree or
supervisor is gone? Task 006 added a standard-library archive CLI that copies
explicitly selected evidence, records hashes and relative references, and reads
saved autocal job outcomes offline. DH-E1 packaged task 004's saved consensus
result: 19 files, including eleven compressed inputs, totaling 4,509,141 bytes.
After relocating the bundle and deleting every temporary source path, all
selected hashes and its scoped decision remained readable. Focused tests also
preserved partial records for absent/damaged evidence and separated terminal job
observations from historical running/cancelling states. All 21 archive/handoff
tests passed. This demonstrates portability and historical lifecycle inspection,
not independent reproduction of the calibration result, current process liveness
or scientific success. Evidence selection remains explicit; runtime dependencies
and unselected document links are outside the bundle. No physics, fitting,
prediction evaluation or collection was performed.

## Implementation and boundary

The [task](../../../../research/program-tasks/006-read-ended-job-evidence.md)
required a usable handoff, including necessary inputs and historical job outcome.
Inspection confirmed that the existing archive only rendered abstracts/links,
while `AutocalJobs.status` required a current in-memory entry. `AutocalJobs.run`
already persisted terminal status, progress, settings, stop reason, exit code,
artifacts and supervisor source hashes to job.json. We reuse those records;
no live-job lifecycle or physics implementation changed.

The [archive implementation](../../../../src/python/hp_sim5_research/archive.py)
now supports explicit export and offline inspection through the
[CLI](../../../../scripts/research_handoff.py). The operator supplies the claim,
scope, decision, source revision and extra evidence files. Export always selects
the abstract, report and notebook, then copies only the selected additional
files. The manifest retains original sources, portable relative paths, copied
SHA-256 identities, optional expected identities and declared limitations. It
does not rewrite reports or chase arbitrary links. The
[procedure](../../../../research/handoffs.md) explains selection, source-layout
requirements and recovery after relocation.

Missing sources, known identity mismatches and invalid job records leave a
partial bundle with diagnostics and the remaining evidence. Destination traversal,
overlapping files and overwrite of existing exports are rejected. Recovery reads
the current bundle and checks selected hashes, refusing file symlinks that resolve
outside it. No dependency environment, recorder tree or attachment credential is
automatically selected.

Offline job reading reports saved accepted/failed/cancelled/budget_exhausted states
as terminal observations and saved running/cancelling as historical nonterminal.
Current liveness remains unknown, and scientific success remains not assessed
even for accepted/exit-0 records. No PID is checked or signalled; no world or job
is adopted. Current-supervisor `autocal_status` is unchanged. A saved final
snapshot cannot serve as a resumable world checkpoint through this route.

## DH-E1: saved CM-E1 portability

Ran once:

```bash
.venv/bin/python output/research/sessions/2026-10-23-durable-handoff/demonstrate.py
```

The [script](demonstrate.py) copies the
[CM-E1 session](../2026-10-23-consensus-membership/report.md) into a temporary
worktree, moves one selected input to a separate external path, exports nineteen
files, relocates the bundle and deletes the temporary worktree and selection
file. It then invokes `show` through a fresh process. CM-E1 had no supervised
autocal job, so lifecycle interpretation is covered by DH-T1 fixtures below.

The [recorded commands/results](results.json),
[export manifest](evidence/export-manifest.json) and
[recovery record](evidence/recovered.json) show:

- Nineteen selected files and eleven compressed input histories: 4,509,141 bytes.
- Every source path absent at recovery, including the external-worktree input.
- Every copied file's SHA-256 verified in the relocated bundle.
- The scoped decision preserved: keep consensus defaults; common-unseen
  predictive gain remains unestablished.

No CM-E1 selections or predictions were rerun. The experiment demonstrates
byte portability of its saved evidence. Inputs, results, input manifest, audit,
scripts and the three research documents are included in the temporary bundle.
Code/runtime dependencies are named by revision rather than duplicated. The
temporary bundles were retired after saving their manifests/results; the
producer files remain in this repository and the script can rebuild the example.

One context limitation in the original captured manifest was too broad: it said
both prior reports were absent in this checkout. Subsequent link inspection found
the October 15 report present; the initial-screen report is absent. Neither was
selected for this scoped byte-portability check. Captured records retain the
original declaration; the demonstration script corrects it for future use. This
does not affect any copied file, hash, measurement or decision.

## DH-T1: focused archive and lifecycle checks

Ran `.venv/bin/python -m pytest -q tests/python/test_research_archive.py
tests/python/test_research_handoff.py`: **21 passed in 0.17 s**. Existing catalog
tests pass unchanged. The [new tests](../../../../tests/python/test_research_handoff.py)
exercise the CLI across relocation/deletion, all four terminal states, stale
running/cancelling, unknown states, missing/corrupt manifests and jobs, known
input-identity mismatch, post-transfer corruption, source absence, traversal,
duplicate destinations, existing bundles and external symlinks. The fixture also
checks that unselected credential/recording files are absent. Calls to os.kill
and os.getpgid are forbidden in lifecycle-reading fixtures.

The first test collection found a missing closing list bracket in the new
exporter; it was fixed before the passing run. No experiment evidence was lost.
Checks and candidate source hashes are in [verification.json](verification.json).
These synthetic fixtures verify bookkeeping; they do not prove numerical
calibration quality or independently reproduce producer physics evidence.

## Decision and remaining limits

H1 is supported within DH-E1's selected files. H2 is supported within the saved
job fixtures. Deliver the lightweight command and procedure. Evidence sufficiency
still depends on the declared claim and explicit file selection. Hash equality
cannot validate artifact semantics, all old Markdown links or the scientific
claim; selected invalid job JSON gets a diagnostic, while other inputs need their
domain-specific decoder to establish semantic validity. A corrupt manifest
leaves the copied files available for manual inspection.

No physics, motor, cable transport or scene dynamics code changed, so the physics
delivery gate does not apply. No collection, fitting or numerical experiment was
used. Task 005 is the next useful queue decision; do not add automatic crawling,
new job adoption or a registry without evidence that explicit handoffs fall short.

The catalog was regenerated. This checkout lacks some prior untracked sessions,
so a full rebuild removed their catalog-only abstracts. Retained those existing
summaries and inserted the generated new-session entry, preserving prior history
without manufacturing missing reports or duplicating aliases.

## References

- [Task 006](../../../../research/program-tasks/006-read-ended-job-evidence.md) and
  [notebook](research.md): authorized outcome and zero numerical budget.
- [Autocal job implementation](../../../../src/python/hp_sim5_research/autocal_jobs.py),
  base [166f0352](https://github.com/tobbelobb/hp-sim5/commit/166f0352e032266711887d0abd01d194cbd4acf2): persisted lifecycle semantics reused without modification.
- [Tooling wish list](../../../../research/prompts/researcher-tooling-wish-list.md): historical lost-run motivation, inspected as context.
- [CM-E1 input manifest](../2026-10-23-consensus-membership/inputs.json): expected compressed input identities and producer revisions, reused without fitting or scoring.
- [DH-E1 results](results.json), [manifest](evidence/export-manifest.json),
  [recovered paths/hashes](evidence/recovered.json): direct evidence for portability only.
- [DH-T1 verification](verification.json): executed tests, candidate identities and scope.
